#!/usr/bin/env python
#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning objects (carla actors and pseudo_actors) in Cyber

Gets config file from ros parameter ~objects_definition_file and spawns corresponding objects
through Cyber service /carla/spawn_object.

Looks for an initial spawn point first in the launchfile, then in the config file, and
finally ask for a random one to the spawn service.

"""


from cyber_record.record import Record



import sys

from carla_bridge.actor.ego_vehicle import EgoVehicle
from carla_bridge.core.spawn_object_param import SpawnObjectParam, KeyValue

sys.path.append("../")

import json
import math
import os

from transforms3d.euler import euler2quat

from modules.common_msgs.localization_msgs.pose_pb2 import Pose
#TODO: 研究一下这个pose 和 carla 回传的数据
sys.path.append("../")
sys.path.append("../../")
sys.path.append("../../../")
# sys.path.append("/home/w/workspace/carla_apollo/apollo")
sys.path.append("/apollo/modules")
sys.path.append("/apollo")

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.routing_msgs import routing_pb2
from modules.common_msgs.basic_msgs import error_code_pb2  # 导入 error_code 枚举


def read_routing_messages_from_log(log_path):
    """
    从日志文件中读取 routing_request 和 routing_response 消息
    """
    routing_request = None
    routing_response = None

    # 使用 Apollo 的 Record 接口读取日志文件
    record = Record(log_path)
    
    # 用于存储读取的消息
    message_all_request = []
    message_all_response = []

    # 遍历日志文件中的所有消息
    for topic, message, t in record.read_messages():
        if topic == "/apollo/routing_request":
            message_all_request.append(message)
        elif topic == "/apollo/routing_response":
            message_all_response.append(message)

    # 假设我们只使用日志中的第一个 routing request 和 response
    if message_all_request:
        routing_request = message_all_request[0]
    if message_all_response:
        routing_response = message_all_response[0]

    return routing_request, routing_response


class CarlaSpawnObjects:

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """
    # 相当于在这里继承了carla apollo bridge, 并且因为前者继承了cybernode, 所以现在变成了一个bridge node 
    def __init__(self, bridge_node,replay,params,objects_file=None):
        cyber.init()
        self.bridge_node = bridge_node
        self.log = bridge_node.log
        self.actor_factory = bridge_node.actor_factory
        print("===============================")
        print("objects_file is:",objects_file)
        print("replay is:",replay)
        print("===============================")
        # 使用传入的objects_file或默认值
        if objects_file and replay:
            
            self.objects_definition_file_replay= objects_file
        else:
            self.objects_definition_file_replay= None
            
            
        self.objects_definition_file = (
            os.path.dirname(os.path.dirname(__file__)) + "/config/objects.json")
            
        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []
        
        self.replay=replay
        self.actor_spawned=False
        
        if self.replay:
            self.node=cyber.Node("routing_publisher")
            self.routing_response_writer=self.node.create_writer("/apollo/routing_response",
                                                        routing_pb2.RoutingResponse)
            self.routing_request_writer = self.node.create_writer("/apollo/routing_request",
                                                             routing_pb2.RoutingRequest)
            
        else:
            self.node=None
            self.routing_response_writer=None
            self.routing_request_writer=None
        
        
        
        if not os.path.exists(params["replay_log"]):
            print("Error: Log file does not exist at path:",params["replay_log"])
            # read in the routing request & response from the log data
            self.routing_request=None
            self.routing_response=None
            
        else:
            # read in the routing request & response from the log data
            self.routing_request, self.routing_response = read_routing_messages_from_log(params["replay_log"])
            
            


    def publish_routing_request(self, routing_request):
        """
        发布从日志文件中提取的 Routing Request
        """
        try:
            # 直接使用原始消息，不进行复制
            if routing_request:
                # 创建 writer 用于发布 routing request
                # cyber.init()
                # node = cyber.Node("routing_request_publisher")
                # routing_writer = node.create_writer("/apollo/routing_request", routing_pb2.RoutingRequest)

                # 发布消息
                self.routing_request_writer.write(routing_request)
                print("Published routing request from log successfully")
                
                # 关闭节点
                # cyber.shutdown()
            else:
                print("No routing request available to publish")

        except Exception as e:
            print(f"Failed to publish routing request: {str(e)}")

    def publish_routing_response(self, routing_response):
        """
        发布从日志文件中提取的 Routing Response
        """
        try:
            # 直接使用原始消息，不进行复制
            if routing_response:
                # 创建 writer 用于发布 routing response
                # cyber.init()
                # node = cyber.Node("routing_response_publisher")
                # routing_writer = node.create_writer("/apollo/routing_response", routing_pb2.RoutingResponse)

                # 发布消息
                self.routing_response_writer.write(routing_response)
                print("Published routing response from log successfully")
                
                # 关闭节点
                # cyber.shutdown()

            else:
                print("No routing response available to publish")

        except Exception as e:
            print(f"Failed to publish routing response: {str(e)}")

    
    def spawn_objects(self):
        """
        Spawns the objects

        Either at a given spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Read sensors from file
        if not self.objects_definition_file or not os.path.exists(self.objects_definition_file):
            raise RuntimeError(
                f"Could not read object definitions from {self.objects_definition_file}")
            
        if self.replay == True:
            with open(self.objects_definition_file_replay) as handle:
                json_actors = json.loads(handle.read())
                # 这里看下json actors 后面需不需要大改
                     
        else:
            with open(self.objects_definition_file) as handle:
                json_actors = json.loads(handle.read())

        global_sensors = []
        vehicles = []

        for actor in json_actors["objects"]:
            actor_type = actor["type"].split('.')[0]
            if actor["type"] == "sensor.pseudo.actor_list":
                global_sensors.append(actor)
            elif actor_type == "sensor":
                global_sensors.append(actor)
            elif actor_type == "vehicle" or actor_type == "walker":
            # elif actor_type == "vehicle":
                
                vehicles.append(actor)
            else:
                self.log.warn(
                    "Object with type {} is not a vehicle, a walker or a sensor, ignoring".format(actor["type"]))

        self.setup_sensors(global_sensors)
        self.setup_vehicles(vehicles)
        
        
        
        
        self.log.info("All objects spawned.")
        
        self.actor_spawned=True

    # def are_actors_spawned(self):
        # return self.actor_spawned #返回生成状态
    
    
    def setup_vehicles(self, vehicles):
        #TODO: 这里生成的时候，判断如果有初识速度，就把初始速度和控制指令赋值给车辆。
        
        for vehicle in vehicles:
            spawn_object_param = SpawnObjectParam()
            spawn_object_param.type = vehicle["type"]
            spawn_object_param.id = vehicle["id"]
            spawn_object_param.attach_to = 0
            spawn_object_param.random_pose = False

            spawn_point = None

            if "spawn_point" in vehicle:
                # get spawn point from config file
                try:
                    spawn_point = self.create_spawn_point(
                        vehicle["spawn_point"]["x"],
                        vehicle["spawn_point"]["y"],
                        vehicle["spawn_point"]["z"],
                        vehicle["spawn_point"]["roll"],
                        vehicle["spawn_point"]["pitch"],
                        vehicle["spawn_point"]["yaw"]
                    )
                    self.log.info("Spawn point from configuration file")
                    self.log.info("-----Spawn Other Vehicles--------------------------------")
                    self.log.info(f"vehicle id is {spawn_object_param.id}, vehicle spawn position is {spawn_point}")
                    
                    
                    if vehicle["id"] == "ego_vehicle":
                        self.log.info(
                            "=======Ego Vehicle Spawn Angles===== "
                            f"roll: {vehicle['spawn_point']['roll']:.2f}, "
                            f"pitch: {vehicle['spawn_point']['pitch']:.2f}, "
                            f"yaw: {vehicle['spawn_point']['yaw']:.2f}"
                        )
                                        
                    
                except KeyError as e:
                    self.log.error("{}: Could not use the spawn point from config file, ".format(vehicle["id"]) +
                                "the mandatory attribute {} is missing, a random spawn point will be used".format(e))
                
               
                
            if spawn_point is None:
                # pose not specified, ask for a random one in the service call
                self.log.info("Spawn point selected at random")
                spawn_point = Pose()  # empty pose
                spawn_object_param.random_pose = True

            if "velocity" in vehicle:

                # 用json 里的值生成一个carla vector 3d object 然后赋值给车辆
                spawn_object_param.initial_velocity = {
                    "x": vehicle["velocity"]["x"],
                    "y": vehicle["velocity"]["y"],
                    "z": vehicle["velocity"]["z"]
                }
            if "angular_velocity" in vehicle:
                spawn_object_param.initial_angular_velocity = {
                    "x": vehicle["angular_velocity"]["x"],
                    "y": vehicle["angular_velocity"]["y"],
                    "z": vehicle["angular_velocity"]["z"]
                }
            #FIXME: 这里的加速度因为没办法直接还原，我们暂时先跳过
            # if "control" in vehicle:
            #     spawn_object_param.initial_control_command = {
            #         "throttle": vehicle["control"]["throttle"],
            #         "steer": vehicle["control"]["steer"],
            #         "brake": vehicle["control"]["brake"],
            #         "gear": vehicle["control"]["gear"],
            #         "hand_brake": vehicle["control"]["hand_brake"],
            #         "reverse": vehicle["control"]["reverse"],
            #         "manual_gear_shift": vehicle["control"]["manual_gear_shift"]
            #     }
            
            spawn_object_param.transform.CopyFrom(spawn_point)
            self.bridge_node.spawn_object(spawn_object_param)

            
            print("------ object type",spawn_object_param.type)
            ego_vehicle_spawned = False
            # FIXME: 这句就是问题所在。
                  
            # 修改判断逻辑：检查是否为ego vehicle
            if spawn_object_param.id == "ego_vehicle":
                # 对ego vehicle进行传感器设置
                while not ego_vehicle_spawned:
                    actors = self.actor_factory.actors.copy()
                    for uid, act in actors.items():
                        if isinstance(act, EgoVehicle):
                            ego_vehicle_spawned = True
                            self.setup_sensors(vehicle["sensors"], uid)
                            #TODO: move it to the main.py after setting the initial value
                            if self.replay:
                                self.log.info("Replay mode: Publishing routing Request...")
                                print(self.routing_request)
                                self.publish_routing_request(self.routing_request)
                                self.log.info("Replay mode: Publishing routing response...")
                                self.publish_routing_response(self.routing_response)
                            else:
                                self.log.info("Normal mode: Skipping routing response publication")
                            break
            elif self.replay:
                # replay模式下的其他车辆直接生成
                self.log.info(f"Replay mode: Spawning non-ego vehicle: {spawn_object_param.id}")
            else:
                # 正常模式下的其他处理（如果有的话）
                self.log.info(f"Normal mode: Spawning non-ego vehicle: {spawn_object_param.id}")
            # while not ego_vehicle_spawned and "vehicle" not in spawn_object_param.type:
            # # while not ego_vehicle_spawned:
                
            #     actors = self.actor_factory.actors.copy()
            #     for uid, act in actors.items():
            #         if isinstance(act, EgoVehicle):
            #             ego_vehicle_spawned = True
            #             self.setup_sensors(vehicle["sensors"], uid)
            #             # 在生成本车后，发布路由响应
            #             # self.publish_routing_response()
            #             if self.replay:
            #                 self.log.info("Replay mode: Publishing routing response...")
            #                 self.publish_routing_response()
            #             else:
            #                 self.log.info("Normal mode: Skipping routing response publication")
            #             break



                    
    def setup_sensors(self, sensors, attached_vehicle_id=None):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_id: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        sensor_names = []
        for sensor_spec in sensors:
            self.log.debug(f"sensor: {sensor_spec}, attach: {attached_vehicle_id}")
            if not self.bridge_node.ok():
                break
            try:
                sensor_type = str(sensor_spec.pop("type"))
                sensor_id = str(sensor_spec.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    raise NameError
                sensor_names.append(sensor_name)

                if attached_vehicle_id is None and "pseudo" not in sensor_type:
                    spawn_point = sensor_spec.pop("spawn_point")
                    sensor_transform = self.create_spawn_point(
                        spawn_point.pop("x"),
                        spawn_point.pop("y"),
                        spawn_point.pop("z"),
                        spawn_point.pop("roll", 0.0),
                        spawn_point.pop("pitch", 0.0),
                        spawn_point.pop("yaw", 0.0))
                else:
                    # if sensor attached to a vehicle, or is a 'pseudo_actor', allow default pose
                    spawn_point = sensor_spec.pop("spawn_point", 0)
                    if spawn_point == 0:
                        sensor_transform = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    else:
                        sensor_transform = self.create_spawn_point(
                            spawn_point.pop("x", 0.0),
                            spawn_point.pop("y", 0.0),
                            spawn_point.pop("z", 0.0),
                            spawn_point.pop("roll", 0.0),
                            spawn_point.pop("pitch", 0.0),
                            spawn_point.pop("yaw", 0.0))

                spawn_object_param = SpawnObjectParam()
                spawn_object_param.type = sensor_type
                spawn_object_param.id = sensor_id
                spawn_object_param.attach_to = attached_vehicle_id if attached_vehicle_id is not None else 0
                spawn_object_param.transform.CopyFrom(sensor_transform)
                spawn_object_param.random_pose = False  # never set a random pose for a sensor
                #TODO: 可以在里面加上速度之类的

                attached_objects = []
                for attribute, value in sensor_spec.items():
                    if attribute == "attached_objects":
                        for attached_object in sensor_spec["attached_objects"]:
                            attached_objects.append(attached_object)
                        continue
                    spawn_object_param.attributes.append(
                        KeyValue(key=str(attribute), value=str(value)))

                response_id = self.bridge_node.spawn_object(spawn_object_param)

                if attached_objects:
                    # spawn the attached objects
                    self.setup_sensors(attached_objects, response_id)

                if attached_vehicle_id is None:
                    self.global_sensors.append(response_id)
                else:
                    self.vehicles_sensors.append(response_id)

            except KeyError as e:
                self.log.error(
                    "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))
                continue

            except RuntimeError as e:
                self.log.error(
                    "Sensor {} will not be spawned: {}".format(sensor_name, e))
                continue

            except NameError:
                self.log.error("Sensor rolename '{}' is only allowed to be used once. The second one will be ignored.".format(
                    sensor_id))
                continue

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        spawn_point = Pose()
        spawn_point.position.x = x
        spawn_point.position.y = y
        spawn_point.position.z = z
        
        quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

        spawn_point.orientation.qx = quat[1]
        spawn_point.orientation.qy = quat[2]
        spawn_point.orientation.qz = quat[3]
        spawn_point.orientation.qw = quat[0]
        return spawn_point
    
    


    def destroy(self):
        """
        destroy all the players and sensors
        """
        self.log.info("Destroying spawned objects...")
        # destroy vehicles sensors
        for actor_id in self.vehicles_sensors:
            if self.bridge_node.destroy_object(actor_id):
                self.log.info("Object {} successfully destroyed.".format(actor_id))
        self.vehicles_sensors = []

        # destroy global sensors
        for actor_id in self.global_sensors:
            if self.bridge_node.destroy_object(actor_id):
                self.log.info("Object {} successfully destroyed.".format(actor_id))
        self.global_sensors = []

        # destroy player
        for player_id in self.players:
            if self.bridge_node.destroy_object(player_id):
                self.log.info("Object {} successfully destroyed.".format(player_id))
        self.players = []


        self.node.shutdown()
        cyber.shutdown()