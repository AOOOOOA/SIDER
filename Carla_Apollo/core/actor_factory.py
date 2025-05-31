#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#




import itertools

from carla_bridge.actor.actor import Actor
from carla_bridge.actor.ego_vehicle import EgoVehicle
from carla_bridge.actor.pseudo_actor import PseudoActor
from carla_bridge.actor.spectator import Spectator
from carla_bridge.actor.static import Static
from carla_bridge.actor.traffic import TrafficLight, Traffic
from carla_bridge.actor.vehicle import Vehicle
from carla_bridge.actor.walker import Walker
from carla_bridge.sensor.camera import RgbCamera, DepthCamera, SemanticSegmentationCamera, DVSCamera, Camera
from carla_bridge.sensor.gnss import Gnss
from carla_bridge.sensor.imu import ImuSensor
from carla_bridge.sensor.lidar import Lidar, SemanticLidar
from carla_bridge.sensor.object_sensor import ObjectSensor
from carla_bridge.sensor.radar import Radar
from carla_bridge.sensor.sensor import Sensor
from carla_bridge.sensor.traffic_lights_sensor import TrafficLightsSensor

try:
    import queue
except ImportError:
    import Queue as queue
import time
from enum import Enum
from threading import Thread, Lock

import carla
import numpy as np

import carla_bridge.utils.transforms as trans

# to generate a random spawning position or vehicles
import random
secure_random = random.SystemRandom()

def get_actor_blueprints(world, filter, generation):
    # print("------------- ", world.get_blueprint_library())
    bps = world.get_blueprint_library().filter(filter)
    #FIXME: 为什么这里得到的bps 的结果是空的？
    # print("+========================== bps", bps)
    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
    
    
class ActorFactory(object):

    TIME_BETWEEN_UPDATES = 0.1

    class TaskType(Enum):
        SPAWN_ACTOR = 0
        SPAWN_PSEUDO_ACTOR = 1
        DESTROY_ACTOR = 2

    def __init__(self, node, client,  world, log, sync_mode=False, replay=False ):
        self.log = log
        self.node = node
        
        self.client=client
        self.world = world
        
        self.blueprint_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.sync_mode = sync_mode

        self._previous_actor_ids = []
        self.actors = {}

        self._task_queue = queue.Queue()
        self._known_actor_ids = []  # used to immediately reply to spawn_actor/destroy_actor calls

        self.lock = Lock()
        self.spawn_lock = Lock()

        # id generator for pseudo sensors
        self.id_gen = itertools.count(10000)

        self.thread = Thread(target=self._update_thread)

        self.vehicles_list=[]
        
        self.replay=replay


    def start(self):
        # create initially existing actors
        self.update_available_objects()
        self.thread.start()

    def _update_thread(self):
        """
        execution loop for async mode actor discovery
        """
        while not self.node.shutdown.is_set():
            time.sleep(ActorFactory.TIME_BETWEEN_UPDATES)
            self.world.wait_for_tick()
            self.update_available_objects()

    def update_available_objects(self):
        """
        update the available actors
        """
        # get only carla actors
        previous_actors = self._previous_actor_ids
        current_actors = [x.id for x in self.world.get_actors()]
        self._previous_actor_ids = current_actors

        new_actors = [x for x in current_actors if x not in previous_actors]
        deleted_actors = [x for x in previous_actors if x not in current_actors]

        # Actual creation/removal of objects
        self.lock.acquire()
        for actor_id in new_actors:
            carla_actor = self.world.get_actor(actor_id)
            if self.node.carla_parameters["register_all_sensors"] or not isinstance(carla_actor, carla.Sensor):
                self._create_object_from_actor(carla_actor)

        for actor_id in deleted_actors:
            self._destroy_object(actor_id, delete_actor=False)

        # update objects for pseudo actors here as they might have an carla actor as parent ######
        with self.spawn_lock:
            while not self._task_queue.empty():
                task = self._task_queue.get()
                if task[0] == ActorFactory.TaskType.SPAWN_ACTOR and not self.node.shutdown.is_set():
                    carla_actor = self.world.get_actor(task[1][0])
                    self._create_object_from_actor(carla_actor)
                elif task[0] == ActorFactory.TaskType.SPAWN_PSEUDO_ACTOR and not self.node.shutdown.is_set():
                    pseudo_object = task[1]
                    self._create_object(pseudo_object[0], pseudo_object[1].type, pseudo_object[1].id,
                                        pseudo_object[1].attach_to, pseudo_object[1].transform)
                elif task[0] == ActorFactory.TaskType.DESTROY_ACTOR:
                    actor_id = task[1]
                    self._destroy_object(actor_id, delete_actor=True)
        self.lock.release()

    def update_actor_states(self, frame_id, timestamp):
        """
        update the state of all known actors
        """
        with self.lock:
            for actor_id in self.actors:
                try:
                    self.actors[actor_id].update(frame_id, timestamp)
                except RuntimeError as e:
                    self.log.warn("Update actor {}({}) failed: {}".format(
                        self.actors[actor_id].__class__.__name__, actor_id, e))
                    continue
        
        # TODO: 在这里用存储的control command 进行update
         
                    
    def clear(self):
        for _, actor in self.actors.items():
            actor.destroy()
        self.actors.clear()
        
        
        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])
   
    def spawn_actor(self, req, spawn_replay_vehicle=False):
        """
        生成actor
        """
        with self.spawn_lock:
            try:
                # print("before pseudo judgement")
                # 原有的spawn逻辑
                if "pseudo" in str(req.type):
                    # print("if pseudo")
                    if req.attach_to != 0:
                        carla_actor = self.world.get_actor(req.attach_to)
                        if carla_actor is None:
                            raise IndexError(f"Parent actor {req.attach_to} not found")
                    id_ = next(self.id_gen)
                    self._task_queue.put((ActorFactory.TaskType.SPAWN_PSEUDO_ACTOR, (id_, req)))
                else:
                #     print("not pseudo ")
                #     #FIXME: 在下面这个函数中报的错
                #     # # 调试打印每个属性
                #     print("req.type:", req.type)
                #     print("req.id:", req.id)
                #     print("req.attach_to:", req.attach_to)
                #     print("req.random_pose:", req.random_pose)
                #     print("req.transform.position:", req.transform.position)
                #     print("req.transform.orientation:", req.transform.orientation)
                #     print("req.attributes:", req.attributes)
                #     print("req.initial_velocity:", req.initial_velocity)
                #     print("req.initial_angular_velocity:", req.initial_angular_velocity)
                #     print("req.initial_control_command:", req.initial_control_command)
                                
                # #FIXME: 这个对应的是中途生成新Actor时的校验机制，这里之后需要看下会不会有问题   
                # # 检查是否已存在相同ID的actor
                # if req.id in [actor.carla_actor.id for actor in self.actors.values()]:
                #     self.log.warning(f"Actor with id {req.id} already exists")
                #     return None
                
                    
                    id_ = self._spawn_carla_actor(req,spawn_replay_vehicle)
                    print("has id?",id_)
                    if id_ is not None:
                        self._task_queue.put((ActorFactory.TaskType.SPAWN_ACTOR, (id_, None)))
                        self._known_actor_ids.append(id_)
                return id_
                
            except Exception as e:
                self.log.error(f"Error in spawn_actor: {str(e)}")
                print(f"Detailed error message: {str(e)}")
                print(f"Error type: {type(e).__name__}")
                print(f"Error location: {e.__traceback__.tb_frame.f_code.co_filename}:{e.__traceback__.tb_lineno}")

                return None
   

    # def spawn_actor(self, req):
    #     """
    #     spawns an object

    #     No object instances are created here. Instead carla-actors are created,
    #     and pseudo objects are appended to a list to get created later.
    #     """
    #     with self.spawn_lock:
    #         if "pseudo" in str(req.type):
    #             # only allow spawning pseudo objects if parent actor already exists in carla
    #             if req.attach_to != 0:
    #                 carla_actor = self.world.get_actor(req.attach_to)
    #                 if carla_actor is None:
    #                     raise IndexError("Parent actor {} not found".format(req.attach_to))
    #             id_ = next(self.id_gen)
    #             self._task_queue.put((ActorFactory.TaskType.SPAWN_PSEUDO_ACTOR, (id_, req)))
    #         else:
    #             id_ = self._spawn_carla_actor(req)
    #             self._task_queue.put((ActorFactory.TaskType.SPAWN_ACTOR, (id_, None)))
    #         self._known_actor_ids.append(id_)
    #     return id_

    def destroy_actor(self, uid):

        def get_objects_to_destroy(uid):
            objects_to_destroy = []
            if uid in self._known_actor_ids:
                objects_to_destroy.append(uid)
                self._known_actor_ids.remove(uid)

            # remove actors that have the actor to be removed as parent.
            for actor in list(self.actors.values()):
                if actor.parent is not None and actor.parent.uid == uid:
                    objects_to_destroy.extend(get_objects_to_destroy(actor.uid))

            return objects_to_destroy

        with self.spawn_lock:
            objects_to_destroy = set(get_objects_to_destroy(uid))
            for obj in objects_to_destroy:
                self._task_queue.put((ActorFactory.TaskType.DESTROY_ACTOR, obj))
        return objects_to_destroy


    
    def _spawn_carla_actor(self, req, spawn_replay_vehicle=False):
        """
        spawns an actor in carla
        """
        try:
            if "*" in str(req.type):
                blueprint = secure_random.choice(
                    self.blueprint_lib.filter(str(req.type)))
            else:
                blueprint = self.blueprint_lib.find(str(req.type))
            #WEI: role name here equals to the number of id we recorded 
            blueprint.set_attribute('role_name', str(req.id))
            
            
            for attribute in req.attributes:
                blueprint.set_attribute(str(attribute.key), str(attribute.value))
            # print("before entering the role name setting")
            if req.random_pose is False:
                # print("replay is", self.replay)
                if self.replay == False:
                    transform = trans.cyber_pose_to_carla_transform(req.transform)
                else:
                    transform = trans.cyber_pose_to_carla_transform_orientation(req.transform)
                
                # transform = trans.cyber_pose_to_carla_transform(req.transform)
                
            else:
                
                # # get a random pose
                # transform = secure_random.choice(
                #     self.spawn_points) if self.spawn_points else carla.Transform()
                #FIXME: 使用固定的生成位置来保证我们的结果一致性
                # get a random pose
                transform = self.spawn_points[0] if self.spawn_points else carla.Transform()


            # print("before attach to ")
            attach_to = None
            if req.attach_to != 0:
                attach_to = self.world.get_actor(req.attach_to)
                if attach_to is None:
                    raise IndexError("Parent actor {} not found".format(req.attach_to))
            # print("blueprint is", blueprint)
            # print("transform is", transform)
            # print("attach to ", attach_to)
            # print("before carla world spawn actor")
            carla_actor = self.world.spawn_actor(blueprint, transform, attach_to)
            # print("after] carla world spawn actor")
            
            
            
            
            # we can wait here when process replay data and generate new vehicles;
            # remember to add the wait paer ---sync generate and sync determine?-- continuous 
            # if the new vehciles is generated and the position is not 000 , then set the intiial velocity and the control command?
            # and divide the condition as the vehicle and the walker 
            
            #This function may influence the first generation part
            
            
            if spawn_replay_vehicle:
                spawn_done=True
                # if req.type.startswith("vehicle"):
                #     while True: 
                #         location=carla_actor.get_location()
                #         print("+++++++++++++++++++in spawn carla actors:+++++++++")
                #         print("location x is:", location.x, 'location_y is', location.y,"req id is:",req.id)
                #         if location.x !=0 and location.y!=0:
                #             spawn_done=True
                #             break
                # else: spawn_down=True
                
                if spawn_done:
                # if self.replay == True:            
                    #同时，如果这里的速度，加速度，角速度，控制指令不为空，则需要赋值给carla 车辆
                    
                    
                    #TODO: walker cannot set velocity it is not suitable
                    if req.type.startswith("vehicle"):
                        if req.initial_velocity:
                            # print("checkcheck", req.type)
                            # print("setting velocity")
                            velocity = carla.Vector3D(req.initial_velocity["x"], req.initial_velocity["y"], req.initial_velocity["z"])
                            # 先关闭物理引擎
                            # carla_actor.set_simulate_physics(False)
                            carla_actor.set_target_velocity(velocity)
                            
                        if req.initial_angular_velocity:
                            # print("setting angular velocity")
                            
                            angular_velocity = carla.Vector3D(req.initial_angular_velocity["x"], req.initial_angular_velocity["y"], req.initial_angular_velocity["z"])
                            carla_actor.set_target_angular_velocity(angular_velocity)
                            
                            # carla_actor.set_simulate_physics(True) # 重新开启物理引擎。不确定这么做行不行，先试试--nope
                            
                            # print("****************************************** setting velocity and angular velocity ******************************************")
                            # self.world.tick()
                            current_vel = carla_actor.get_velocity()
                            # print("Ego vehicle velocity components after setting:")
                            # print("Velocity x: %.2f m/s" % current_vel.x)
                            # print("Velocity y: %.2f m/s" % current_vel.y)
                            # print("Velocity z: %.2f m/s" % current_vel.z)

                            current_ang_vel = carla_actor.get_angular_velocity()
                            # print("Ego vehicle angular velocity components after setting:")
                            # print("Angular velocity x: %.2f rad/s" % current_ang_vel.x)
                            # print("Angular velocity y: %.2f rad/s" % current_ang_vel.y) 
                            # print("Angular velocity z: %.2f rad/s" % current_ang_vel.z)
                    
                    if req.initial_control_command and req.type.startswith('vehicle'):
                        # print("setting control command")
                        
                        control = carla.VehicleControl()
                        control.throttle = float(req.initial_control_command.get("throttle", 0.0))
                        control.steer = float(req.initial_control_command.get("steer", 0.0))
                        control.brake = float(req.initial_control_command.get("brake", 0.0))
                        control.gear = int(req.initial_control_command.get("gear", 1))
                        control.hand_brake = bool(req.initial_control_command.get("hand_brake", False))
                        control.reverse = bool(req.initial_control_command.get("reverse", False))
                        control.manual_gear_shift = bool(req.initial_control_command.get("manual_gear_shift", False))

                        # 将控制指令应用到车辆
                        carla_actor.apply_control(control)
                        # self.world.tick()
                    if req.initial_control_command and req.type.startswith('walker'):
                                
                        direction=carla.Vector3D(req.initial_control_command.get("direction_x", 0.0),
                                                req.initial_control_command.get("direction_y", 0.0),
                                                req.initial_control_command.get("direction_z", 0.0),)
                        speed=req.initial_control_command.get("speed", 0.0)
                        jump=req.initial_control_command.get("jump", 0.0)
                        
                        
        
                        self.log.info(f"direction is {direction}, type is {type(direction)},speed is {speed}, type is {type(speed)},jump is {jump}, type is {type(jump)}")
                        control=carla.WalkerControl(direction, float(speed), bool(jump))
                                        
            # print("after all")
            return carla_actor.id
        except Exception as e:
            self.log.error(f"Error in _spawn_carla_actor: {str(e)}")
            print(f"Detailed error message: {str(e)}")
            print(f"Error type: {type(e).__name__}")
            print(f"Error location: {e.__traceback__.tb_frame.f_code.co_filename}:{e.__traceback__.tb_lineno}")

            return None
            

    def _create_object_from_actor(self, carla_actor):
        """
        create a object for a given carla actor
        Creates also the object for its parent, if not yet existing
        """
        parent = None
        # the transform relative to the map
        relative_transform = trans.carla_transform_to_cyber_pose(carla_actor.get_transform())
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self._create_object_from_actor(carla_actor.parent)
            # calculate relative transform to the parent
            actor_transform_matrix = trans.cyber_pose_to_transform_matrix(relative_transform)
            parent_transform_matrix = trans.cyber_pose_to_transform_matrix(
                trans.carla_transform_to_cyber_pose(carla_actor.parent.get_transform()))
            relative_transform_matrix = np.matrix(
                parent_transform_matrix).getI() * np.matrix(actor_transform_matrix)
            relative_transform = trans.transform_matrix_to_cyber_pose(relative_transform_matrix)

        parent_id = 0
        if parent is not None:
            parent_id = parent.uid

        name = carla_actor.attributes.get("role_name", "")
        if not name:
            name = str(carla_actor.id)
        obj = self._create_object(carla_actor.id, carla_actor.type_id, name,
                                  parent_id, relative_transform, carla_actor)
        return obj

    def _destroy_object(self, actor_id, delete_actor):
        if actor_id not in self.actors:
            return
        actor = self.actors[actor_id]
        del self.actors[actor_id]
        carla_actor = None
        if isinstance(actor, Actor):
            carla_actor = actor.carla_actor
        actor.destroy()
        if carla_actor and delete_actor:
            carla_actor.destroy()
        self.log.info("Removed {}(id={})".format(actor.__class__.__name__, actor.uid))

    def get_pseudo_sensor_types(self):
        pseudo_sensors = []
        for cls in PseudoActor.__subclasses__():
            if cls.__name__ != "Actor":
                pseudo_sensors.append(cls.get_blueprint_name())
        return pseudo_sensors

    def _create_object(self, uid, type_id, name, attach_to, spawn_pose, carla_actor=None):
        # check that the actor is not already created.
        if carla_actor is not None and carla_actor.id in self.actors:
            return None

        if attach_to != 0:
            if attach_to not in self.actors:
                raise IndexError("Parent object {} not found".format(attach_to))

            parent = self.actors[attach_to]
        else:
            parent = None

        if type_id == ObjectSensor.get_blueprint_name():
            actor = ObjectSensor(
                uid=uid,
                name=name,
                parent=parent,
                node=self.node,
                actor_list=self.actors,
            )

        elif type_id == TrafficLightsSensor.get_blueprint_name():
            actor = TrafficLightsSensor(
                uid=uid,
                name=name,
                parent=parent,
                node=self.node,
                actor_list=self.actors,
                log=self.log
            )
            
        elif carla_actor.type_id.startswith('traffic'):
            if carla_actor.type_id == "traffic.traffic_light":
                actor = TrafficLight(uid, name, parent, self.node, carla_actor)
            else:
                actor = Traffic(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("vehicle"):
            if carla_actor.attributes.get('role_name')\
                    in self.node.carla_parameters['ego_vehicle']['role_name']:
                actor = EgoVehicle(
                    uid, name, parent, self.node, carla_actor, self.world)
            else:
                actor = Vehicle(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("sensor"):
            if carla_actor.type_id.startswith("sensor.camera"):
                if carla_actor.type_id.startswith("sensor.camera.rgb"):
                    actor = RgbCamera(uid, name, parent, spawn_pose, self.node,
                                      carla_actor, self.sync_mode)
                elif carla_actor.type_id.startswith("sensor.camera.depth"):
                    actor = DepthCamera(uid, name, parent, spawn_pose,
                                        self.node, carla_actor, self.sync_mode)
                elif carla_actor.type_id.startswith(
                        "sensor.camera.semantic_segmentation"):
                    actor = SemanticSegmentationCamera(uid, name, parent,
                                                       spawn_pose, self.node,
                                                       carla_actor,
                                                       self.sync_mode)
                elif carla_actor.type_id.startswith("sensor.camera.dvs"):
                    actor = DVSCamera(uid, name, parent, spawn_pose, self.node,
                                      carla_actor, self.sync_mode)
                else:
                    actor = Camera(uid, name, parent, spawn_pose, self.node,
                                   carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.lidar"):
                if carla_actor.type_id.endswith("sensor.lidar.ray_cast"):
                    actor = Lidar(uid, name, parent, spawn_pose, self.node,
                                  carla_actor, self.sync_mode)
                elif carla_actor.type_id.endswith(
                        "sensor.lidar.ray_cast_semantic"):
                    actor = SemanticLidar(uid, name, parent, spawn_pose,
                                          self.node, carla_actor,
                                          self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.radar"):
                actor = Radar(uid, name, parent, spawn_pose, self.node,
                              carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.gnss"):
                actor = Gnss(uid, name, parent, spawn_pose, self.node,
                             carla_actor, self.sync_mode)
            elif carla_actor.type_id.startswith("sensor.other.imu"):
                actor = ImuSensor(uid, name, parent, spawn_pose, self.node,
                                  carla_actor, self.sync_mode)
            else:
                actor = Sensor(uid, name, parent, spawn_pose, self.node,
                               carla_actor, self.sync_mode)
        elif carla_actor.type_id.startswith("spectator"):
            actor = Spectator(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("walker"):
            actor = Walker(uid, name, parent, self.node, carla_actor)
        elif carla_actor.type_id.startswith("static.prop"):
            actor = Static(uid, name, parent, self.node, carla_actor)
        else:
            actor = Actor(uid, name, parent, self.node, carla_actor)

        self.actors[actor.uid] = actor
        self.log.info("Created {}(id={})".format(actor.__class__.__name__, actor.uid))

        return actor