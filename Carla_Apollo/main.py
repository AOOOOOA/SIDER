#!/usr/bin/env python
#
# Copyright (c) 2023 synkrotron.ai
#


# 在这个代码中, 嵌入traffic manager 的部分, 并且在生成Apollo 车辆之前生成traffic manager 的一系列车.



import os
import sys
import threading
from threading import Event, Thread

import carla
import yaml
from cyber.proto.clock_pb2 import Clock

from carla_bridge.core.actor_factory import ActorFactory
from carla_bridge.core.carla_spawn_objects import CarlaSpawnObjects
from carla_bridge.core.node import CyberNode
from carla_bridge.utils.logurus import init_log
import json
import socket

from carla_bridge.core.spawn_object_param import SpawnObjectParam, KeyValue  # 同时导入KeyValue类

import random 
import traceback
sys.path.append("../")
from transforms3d.euler import euler2quat
import math
import time

from absl import app
from absl import flags
import numpy as np
import traceback



#TODO: Finally remove the pipe and socket
#FIXME: 这里有问题，这里的参数和tester传入的参数是不一致的。尤其是最后一个settings和objects files
FLAGS = flags.FLAGS
flags.DEFINE_boolean('replay', None, 'Enable replay mode')
flags.DEFINE_string('replay_file', None, 'Path to replay file')
# flags.DEFINE_string('settings', 'config/settings.yaml', 'Path to settings file')
flags.DEFINE_string('objects_file', None, 'Path to objects definition file')
flags.DEFINE_string('replay_log', None, 'Path to original apollo log for set the routing')
# 在文件开头的FLAGS定义部分添加新的参数
flags.DEFINE_integer('num_vehicles', 0, 'Number of vehicles to spawn')
flags.DEFINE_integer('num_walkers', 0, 'Number of walkers to spawn')
flags.DEFINE_string('record_path', '', 'Path to save recorded data')
# flags.DEFINE_integer('record_interval', 10, 'Interval between recorded frames') #暂时不用这个，这个作为一个固定参数。
flags.DEFINE_string('record_replay_path', '', 'Path to save recorded replay data in json')
flags.DEFINE_string('metric_dir','', 'dir to save the metric results ')



def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)
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
    
class VehicleManager:
    def __init__(self,carla_world):
        self.vehicles=[]
        self.lock=threading.Lock()
        self.carla_world=carla_world
        
    def add_vehicles(self, actor_id, vehicle_data):
        carla_actor = self.carla_world.get_actor(actor_id)
        if carla_actor:
            with self.lock:
                self.vehicles.append((carla_actor, vehicle_data))

    def set_vehicle_velocities(self):
        while True:
            for carla_actor, vehicle_data in self.vehicles:
                try:
                    location = carla_actor.get_location()
                    if location.x != 0 and location.y != 0:
                        # 设置速度
                        recorded_vel = vehicle_data["velocity"]
                        velocity = carla.Vector3D(recorded_vel["x"], recorded_vel["y"], recorded_vel["z"])
                        carla_actor.set_target_velocity(velocity)

                        # 设置控制指令
                        control = carla.VehicleControl(
                            throttle=vehicle_data['throttle'],
                            steer=vehicle_data['steer'],
                            brake=vehicle_data['brake'],
                            hand_brake=vehicle_data['hand_brake'],
                            reverse=vehicle_data['reverse'],
                            manual_gear_shift=vehicle_data['manual_gear_shift'],
                            gear=vehicle_data['gear']
                        )
                        carla_actor.apply_control(control)

                        # 从列表中移除已设置速度的车辆
                        self.vehicles.remove((carla_actor, vehicle_data))
                except Exception as e:
                    self.log.error(f"Error setting vehicle velocities: {str(e)}")
            time.sleep(0.1)  # 每0.1秒检查一次



class CarlaCyberBridge(CyberNode):

    # in synchronous mode, if synchronous_mode_wait_for_vehicle_control_command is True,
    # wait for this time until a next tick is triggered.
    VEHICLE_CONTROL_TIMEOUT = 1.0

    def __init__(self):
        super().__init__("cyber_bridge_node")
        self.spawn_objects_node = None
        self.timestamp = None
        self._registered_actors = None
        self.on_tick_id = None
        self.synchronous_mode_update_thread = None
        self.clock_writer = None
        self.actor_factory = None
        self.sync_mode = None
        self.carla_settings = None
        self.shutdown = None
        self.carla_world = None
        self.carla_parameters = None
        self.log = None
        
        
        self.replay=None
        self.replay_data=None
        self.replay_init_vehicle_num=0
        self.replay_frame_index=1
        # 配置参数
        self.POSITION_THRESHOLD = 0.5  # 米
        self.VELOCITY_THRESHOLD = 0.5  # m/s
        
        self.call_carla_spawn_objs=False
        
        self.replay_frame_count=0
        
        
        self.pipe_path="/tmp/replay_pipe"      
        self.status_pipe=None
        
        self._pipe_open=False
        self._pipe_lock=threading.Lock()
        
        
        self.tcp_client_socket=None
        
        
        #TODO: 这里需要把generate traffic和 record data 的相关设置在main.py 中做出接口来
        # 添加交通生成相关属性
        self.vehicles_list = []
        self.walkers_list = []
        self.all_id = []
        self.traffic_manager = None
        
        # 添加数据记录相关属性
        self.record_data = []
        self.start_time = None
        self.output_path = None  # 需要从参数中获取
        self.record_interval = 100 # 每10帧记录一次
        self.last_record_frame = 0
        
        self.ego_vehicle=None
        self.frame_data=[]
        self.collision_detector=None
        self.collision_events=[]
                
        self.replay_frame_data=[]
        self.last_save_time=time.time()
        self.save_interval=10
        
        self.record_replay_path=None
        self.metric_dir=None
        
        
        self.fast_accel=False
        self.hard_braking=False
        self.speeding=False
        
        self.lane_invasion_detector=None
        self.lane_invasion_events=[]
        
        
    def setup_tcp_client(self, host, port):
        """设置TCP客户端"""
        self.tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client_socket.connect((host, port))
        self.log.info("TCP client connected to server")
        
            # 发送初始状态
        initial_status = {
            'replay_complete': False,
            'frame_index': 0,
            'total_frames': len(self.replay_data) if self.replay_data else 0
        }
        self.send_tcp_data(json.dumps(initial_status))
        

    def send_tcp_data(self, data):
        """通过TCP发送数据"""
        if self.tcp_client_socket:
            # self.tcp_client_socket.sendall(data.encode())
            self.tcp_client_socket.sendall((data + '\n').encode())
            self.log.info("Data sent to TCP server")


    def setup_pipe(self):
        """初始化FIFO管道，使用非阻塞模式"""
        self.log.info("Begin to set pipe")
        
        try:
            # 1. 清理已存在的管道
            if os.path.exists(self.pipe_path):
                self.log.info(f"Removing existing pipe at {self.pipe_path}")
                try:
                    os.unlink(self.pipe_path)
                except Exception as e:
                    self.log.error(f"Failed to remove existing pipe: {e}")
                    raise
            
            # 2. 创建新的FIFO管道
            try:
                os.mkfifo(self.pipe_path, mode=0o666)
                self.log.info(f"Created new FIFO pipe at {self.pipe_path}")
            except Exception as e:
                self.log.error(f"Failed to create FIFO pipe: {e}")
                raise
            
            # 3. 尝试以只读模式先打开管道（这样写入端就不会阻塞）
            try:
                dummy_read = os.open(self.pipe_path, os.O_RDONLY | os.O_NONBLOCK)
                self.log.info("Opened dummy read end of pipe")
            except Exception as e:
                self.log.error(f"Failed to open dummy read end: {e}")
                raise
            
            # 4. 现在尝试打开写入端
            try:
                self.status_pipe = os.open(self.pipe_path, os.O_WRONLY | os.O_NONBLOCK)
                self.log.info("Successfully opened write end of pipe")
            except Exception as e:
                self.log.error(f"Failed to open write end: {e}")
                os.close(dummy_read)  # 清理dummy read
                raise
            
            # 5. 设置非阻塞标志
            try:
                import fcntl
                flags = fcntl.fcntl(self.status_pipe, fcntl.F_GETFL)
                fcntl.fcntl(self.status_pipe, fcntl.F_SETFL, flags | os.O_NONBLOCK)
                self.log.info("Set non-blocking mode successfully")
            except Exception as e:
                self.log.error(f"Failed to set non-blocking mode: {e}")
                os.close(dummy_read)
                os.close(self.status_pipe)
                raise
            
            # 6. 清理dummy read end
            try:
                os.close(dummy_read)
            except Exception as e:
                self.log.warning(f"Failed to close dummy read end: {e}")
            
            self._pipe_open = True
            self.log.info("FIFO pipe setup complete")
            return True
        
        except Exception as e:
            self.log.error(f"Failed to setup pipe: {str(e)}")
            self._pipe_open = False
            # 确保清理所有资源
            if hasattr(self, 'status_pipe') and self.status_pipe is not None:
                try:
                    os.close(self.status_pipe)
                except:
                    pass
            if os.path.exists(self.pipe_path):
                try:
                    os.unlink(self.pipe_path)
                except:
                    pass
            return False
    
    #FIXME: 以及对比原代码看是不是正确的; 
    def spawn_traffic(self, num_vehicles=10, num_walkers=5):
        """生成交通参与者"""
        
        print("+++++++++++++++++++++++++++++++++++++ number of vehicles are", num_vehicles)
        print("====================================== number of walkers are", num_walkers)
        
        try:
            # 1. 生成车辆部分
            
            blueprints = get_actor_blueprints(self.carla_world, "vehicle.*", "All")
            spawn_points = self.carla_world.get_map().get_spawn_points()
            
            # 生成车辆
            batch = []
            for n, transform in enumerate(spawn_points):
                if n >= num_vehicles:
                    break
                blueprint = random.choice(blueprints)
                blueprint.set_attribute('role_name', 'autopilot')
                batch.append(carla.command.SpawnActor(blueprint, transform)
                    .then(carla.command.SetAutopilot(carla.command.FutureActor, True, self.traffic_manager.get_port())))
            
            for response in self.carla_client.apply_batch_sync(batch, True):
                if not response.error:
                    self.vehicles_list.append(response.actor_id)
                    
            # 2. 生成行人部分
            # 获取行人蓝图
            blueprintsWalkers = self.carla_world.get_blueprint_library().filter('walker.pedestrian.*')
            
            # 为行人获取随机位置
            spawn_points = []
            for _ in range(num_walkers):
                spawn_point = carla.Transform()
                loc = self.carla_world.get_random_location_from_navigation()
                if loc is not None:
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            
            # 生成行人
            batch = []
            walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                if walker_bp.has_attribute('speed'):
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])  # walking
                batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
                
            # 应用行人生成
            results = self.carla_client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if not results[i].error:
                    self.walkers_list.append({"id": results[i].actor_id})
                    
            # 生成行人AI控制器
            batch = []
            walker_controller_bp = self.carla_world.get_blueprint_library().find('controller.ai.walker')
            for i in range(len(self.walkers_list)):
                batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
                
            # 应用控制器生成
            results = self.carla_client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if not results[i].error:
                    self.walkers_list[i]["con"] = results[i].actor_id
                    
            # 收集所有id
            for walker in self.walkers_list:
                self.all_id.append(walker["con"])
                self.all_id.append(walker["id"])
                
            # 启动行人AI
            # self.carla_world.tick()
            
            # 设置行人行走
            for i in range(0, len(self.all_id), 2):
                self.carla_world.get_actor(self.all_id[i]).start()
                self.carla_world.get_actor(self.all_id[i]).go_to_location(
                    self.carla_world.get_random_location_from_navigation())
                    
            self.log.info(f"Spawned {len(self.vehicles_list)} vehicles and {len(self.walkers_list)} walkers")
            
        except Exception as e:
            self.log.error(f"Error spawning traffic: {e}")
            traceback.print_exc()
        
        
    def get_nearby_traffic_lights(self, world, vehicle_location, radius=50):
        """获取指定范围内的交通灯状态"""
        traffic_lights = []
        for actor in world.get_actors().filter('traffic.traffic_light*'):
            if actor.get_location().distance(vehicle_location) < radius:
                location = actor.get_location()
                traffic_light_data = {
                    "location": {
                        "x": round(location.x, 2),
                        "y": round(location.y, 2),
                        "z": round(location.z, 2)
                    },
                    "state": str(actor.get_state()),
                    "elapsed_time": actor.get_elapsed_time(),
                }
                traffic_lights.append(traffic_light_data)
        return traffic_lights
    
 
    def record_frame_data(self, frame_id, distance_threshold=75):
        try:
            snapshot = self.carla_world.get_snapshot()
            frame = snapshot.frame
            timestamp = snapshot.timestamp.elapsed_seconds

            # Ego Vehicle 的位置和朝向
            ego_transform = self.ego_vehicle.get_transform()
            ego_location = ego_transform.location
            ego_rotation = ego_transform.rotation

            # 获取 Ego Vehicle 的控制状态
            ego_control = self.ego_vehicle.get_control()

            # 获取 Ego Vehicle 的速度、角速度和加速度
            ego_velocity = self.ego_vehicle.get_velocity()
            ego_angular_velocity = self.ego_vehicle.get_angular_velocity()
            ego_acceleration = self.ego_vehicle.get_acceleration()

            def vector_magnitude(vector):
                return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


            ego_vehicle_data = {
                'id': self.ego_vehicle.id,
                'x': ego_location.x,
                'y': ego_location.y,
                'z': ego_location.z,
                'yaw': ego_rotation.yaw,
                'roll': ego_rotation.roll,
                'pitch': ego_rotation.pitch,
                'throttle': ego_control.throttle,
                'steer': ego_control.steer,
                'brake': ego_control.brake,
                'gear': ego_control.gear,
                'hand_brake': ego_control.hand_brake,
                'reverse': ego_control.reverse,
                'manual_gear_shift': ego_control.manual_gear_shift,
                'velocity': {
                    'x': ego_velocity.x,
                    'y': ego_velocity.y,
                    'z': ego_velocity.z,
                    'magnitude': vector_magnitude(ego_velocity)  # 速度的标量值
                },
                'angular_velocity': {
                    'x': ego_angular_velocity.x,
                    'y': ego_angular_velocity.y,
                    'z': ego_angular_velocity.z,
                    'magnitude': vector_magnitude(ego_angular_velocity)  # 角速度的标量值
                },
                'acceleration': {
                    'x': ego_acceleration.x,
                    'y': ego_acceleration.y,
                    'z': ego_acceleration.z,
                    'magnitude': vector_magnitude(ego_acceleration)  # 加速度的标量值
                }
            }
            
            
            print("[====ANGLE=====]recorded ego_vehicle_angle:", "yaw:", ego_rotation.yaw, "roll:", ego_rotation.roll, "pitch:", ego_rotation.pitch)

            nearby_vehicles = []
            nearby_vehicle_count = 0
            nearby_walkers=[]
            nearby_walker_count=0

            for actor in self.carla_world.get_actors().filter('vehicle.*'):
                if actor.id != self.ego_vehicle.id:
                    distance = actor.get_location().distance(ego_location)
                    if distance <= distance_threshold:
                        nearby_vehicle_count += 1
                        vehicle_transform = actor.get_transform()
                        vehicle_location = vehicle_transform.location
                        vehicle_rotation = vehicle_transform.rotation
                        vehicle_velocity = actor.get_velocity()
                        vehicle_angular_velocity = actor.get_angular_velocity()
                        vehicle_acceleration = actor.get_acceleration()
                        vehicle_control = actor.get_control()


                        
                        vehicle_data= {
                            'id': actor.id,
                            'type_id': actor.type_id,  # 记录车辆的类型
                            'x': vehicle_location.x,
                            'y': vehicle_location.y,
                            'z': vehicle_location.z,
                            'yaw': vehicle_rotation.yaw,
                            'roll': vehicle_rotation.roll,
                            'pitch': vehicle_rotation.pitch,
                            'throttle': vehicle_control.throttle,
                            'steer': vehicle_control.steer,
                            'brake': vehicle_control.brake,
                            'gear': vehicle_control.gear,
                            'hand_brake': vehicle_control.hand_brake,
                            'reverse': vehicle_control.reverse,
                            'manual_gear_shift': vehicle_control.manual_gear_shift,
                            'velocity': {
                                'x': vehicle_velocity.x,
                                'y': vehicle_velocity.y,
                                'z': vehicle_velocity.z,
                                'magnitude': vector_magnitude(vehicle_velocity)  # 速度的标量值
                            },
                            'angular_velocity': {
                                'x': vehicle_angular_velocity.x,
                                'y': vehicle_angular_velocity.y,
                                'z': vehicle_angular_velocity.z,
                                'magnitude': vector_magnitude(vehicle_angular_velocity)  # 角速度的标量值
                            },
                            'acceleration': {
                                'x': vehicle_acceleration.x,
                                'y': vehicle_acceleration.y,
                                'z': vehicle_acceleration.z,
                                'magnitude': vector_magnitude(vehicle_acceleration)  # 加速度的标量值
                            },
                            'distance_to_ego': distance
                        }
                        
                        
                        nearby_vehicles.append(vehicle_data)

        
        
            #TODO: move it as the record walker function 
            for actor in self.carla_world.get_actors().filter('walker.*'):
                if actor.id != self.ego_vehicle.id:
                    distance = actor.get_location().distance(ego_location)
                    if distance <= distance_threshold:
                        nearby_walker_count += 1
                        
                        ped=actor
                        ped_transform=ped.get_transform()
                        ped_location = ped_transform.location
                        ped_rotation = ped_transform.rotation
                        ped_velocity = ped.get_velocity()
                        ped_angular_velocity = ped.get_angular_velocity()
                        ped_acceleration = ped.get_acceleration()
                        ped_control = ped.get_control()
                        
                        
                        
                        walker_data={
                            'id': ped.id,
                            'type_id':ped.type_id,
                            'x': ped_location.x,
                            'y': ped_location.y,
                            'z': ped_location.z,
                            'yaw': ped_rotation.yaw,
                            'roll': ped_rotation.roll,
                            'pitch': ped_rotation.pitch,
                            "direction_x": ped_control.direction.x,
                            "direction_y": ped_control.direction.y,
                            "direction_z": ped_control.direction.z,
                            
                            "speed": ped_control.speed,
                            "jump": ped_control.jump,
                            'velocity': {
                                'x': ped_velocity.x,
                                'y': ped_velocity.y,
                                'z': ped_velocity.z,
                                'magnitude': vector_magnitude(ped_velocity)  # 速度的标量值
                            },
                            'angular_velocity': {
                                'x': ped_angular_velocity.x,
                                'y': ped_angular_velocity.y,
                                'z': ped_angular_velocity.z,
                                'magnitude': vector_magnitude(ped_angular_velocity)  # 角速度的标量值
                            },
                            'acceleration': {
                                'x': ped_acceleration.x,
                                'y': ped_acceleration.y,
                                'z': ped_acceleration.z,
                                'magnitude': vector_magnitude(ped_acceleration)  # 加速度的标量值
                            },
                            'distance_to_ego': distance
                        }
                        
                        nearby_walkers.append(walker_data)
                            
                        



            traffic_lights = self.get_nearby_traffic_lights(self.carla_world, ego_location, radius=distance_threshold)

            self.frame_data.append({
                'frame': frame,
                'timestamp': timestamp,
                'ego_vehicle': ego_vehicle_data,
                'nearby_vehicle_count': nearby_vehicle_count,
                'nearby_vehicles': nearby_vehicles,
                'nearby_walkers': nearby_walkers,
                'traffic_lights': traffic_lights
            })
            
            """记录当前帧的数据"""
            # 检查是否需要记录本帧
            if frame_id - self.last_record_frame < self.record_interval:
                return
            else:
                try:
                    record_path = self.carla_parameters.get('record_path')
                    if record_path:    
                        parent_dir = os.path.dirname(record_path)
                        if parent_dir:  # 如果父目录不为空
                            os.makedirs(parent_dir, exist_ok=True)

                        # 保存数据
                        with open(record_path, 'w') as f:
                            json.dump(self.frame_data, f, indent=4)
                            # f.flush()
                            # os.fsync(f.fileno()) # 这个是同步写入磁盘，但是会阻塞，所以不使用
                            
                        self.log.info(f"Saved {len(self.frame_data)} frames to {record_path}")
                except Exception as e:
                    self.log.error(f"Error saving frame data: {e}")
                    traceback.print_exc()
                
                self.last_record_frame = frame_id
            
        except Exception as e:
            traceback.print_exc()
            self.log.error(f"Error recording frame data: {e}")
    

            
        
    def initialize_bridge(self, carla_client, carla_world, params, log):
        """
        Initialize the bridge
        """
        self.log = log
        self.carla_parameters = params["carla"]
        self.carla_world = carla_world
        self.carla_client=carla_client

        self.synchronous_mode_update_thread = None
        self.shutdown = Event()
        

        self.carla_settings = carla_world.get_settings()
        
        

        self.replay= self.carla_parameters["replay"]
        self.vehicle_manager=None
       
        # print("after params replay")
                
        if self.replay:
            # # 在这里设置vehicle manager给后面的代码来使用，来设置车辆的速度等。
            self.vehicle_manager=VehicleManager(self.carla_world)
            threading.Thread(target=self.vehicle_manager.set_vehicle_velocities, daemon=True).start()
            
            
            print("before set up tcp client")
            self.setup_tcp_client(host="localhost", port=12345)
            replay_file = self.carla_parameters.get("replay_file")
            print("after set up tcp client")
            
            # 1. 首先进行参数检查
            if not replay_file:
                raise ValueError("Replay mode enabled but no replay file path provided")
            if not os.path.exists(replay_file):
                raise FileNotFoundError(f"Replay file not found: {replay_file}")
            if not replay_file.endswith('.json'):
                raise ValueError(f"Replay file must be a JSON file, got: {replay_file}")
            
            # 2. 加载和验证回放数据
            try:
                self.log.info(f"Loading replay data from {replay_file}")
                with open(replay_file, 'r') as f:
                    self.replay_data = json.load(f)
                
                
                self.replay_init_vehicle_num=len(self.replay_data[0]["nearby_vehicles"])
                
                
                
                
                # 基本数据验证
                if not isinstance(self.replay_data, list):
                    raise ValueError("Replay data must be a list of frames")
                if not self.replay_data:
                    raise ValueError("Replay data is empty")
                
                self.replay_frame_index = 0
                total_frames = len(self.replay_data)
                self.log.info(f"===================================Successfully loaded {total_frames} frames from {replay_file}")
                
                
                print("-----begin to set fifo---")
                # 3. 数据验证通过后，再设置FIFO
                try:
                    self.setup_pipe()
                    self.log.info("FIFO pipe setup completed")
                except Exception as e:
                    self.log.error(f"Failed to setup FIFO pipe: {e}")
                    # 如果FIFO设置失败，可以选择是否继续
                    raise RuntimeError(f"Failed to setup FIFO pipe: {e}")
                print("-----after setting fifo---")
                
                
                
                
                #4. 设置replay的时候record data path
                self.record_replay_path=self.carla_parameters.get("record_replay_path")
                # if not os.path.isdir(self.record_replay_path):
                #     raise ValueError(f"Invalid directory path for record_replay_path: {self.record_replay_path}")
                    
                
                
                
            except json.JSONDecodeError as e:
                raise ValueError(f"Failed to parse replay file: {str(e)}")
            except Exception as e:
                # 确保在任何错误发生时清理已创建的资源
                self.cleanup()  # 清理可能已创建的FIFO
                raise RuntimeError(f"Failed to initialize replay mode: {str(e)}")
                
            
        if not self.carla_parameters["passive"]:
            # workaround: settings can only applied within non-sync mode
            if self.carla_settings.synchronous_mode:
                self.carla_settings.synchronous_mode = False
                carla_world.apply_settings(self.carla_settings)

            self.carla_settings.synchronous_mode = self.carla_parameters[
                "synchronous_mode"
            ]

            self.carla_settings.fixed_delta_seconds = self.carla_parameters[
                "fixed_delta_seconds"
            ]
            
            self.carla_settings.substepping = True
            self.carla_settings.max_substep_delta_time = 0.001
            self.carla_settings.max_substeps = 100
            
            
            carla_world.apply_settings(self.carla_settings)

        self.sync_mode = (
            self.carla_settings.synchronous_mode
            and not self.carla_parameters["passive"]
        )
        # actor factory
        self.actor_factory = ActorFactory(self, carla_client, carla_world, self.log, self.sync_mode, self.replay)



        # move the generate traffic part here, generate the ego vehicle first and then the traffic flow

                        




        # Communication topics
        self.clock_writer = self.new_writer("/clock", Clock, 10)

        if self.sync_mode:
            self.synchronous_mode_update_thread = Thread(
                target=self._synchronous_mode_update
            )
            self.synchronous_mode_update_thread.daemo = True
            self.synchronous_mode_update_thread.start()
        self._registered_actors = []

        carla_spawn_objects_thread = threading.Thread(
            target=self.carla_spawn_objects(self.replay)
        )
        carla_spawn_objects_thread.daemo = True
        carla_spawn_objects_thread.start()

       
        self.spin()
                        
                        
        
    

    def carla_spawn_objects(self,replay):
        print("call carla spawn objects")
        self.spawn_objects_node = CarlaSpawnObjects(self,replay,self.carla_parameters,
                                                objects_file=FLAGS.objects_file)  
        
        
        
        self.spawn_objects_node.spawn_objects()
        time.sleep(1)
        self.call_carla_spawn_objs=True
        
        
        
        
        
        if self.replay==False and self.carla_parameters["num_vehicles"]>0:
            
            self.num_vehicles=self.carla_parameters["num_vehicles"]
            self.num_walkers=self.carla_parameters["num_walkers"]
        
            self.traffic_manager = self.carla_client.get_trafficmanager(8000)
            self.log.info("Successfully connect to carla traffic manager")
            self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            self.traffic_manager.set_synchronous_mode(True)
            self.traffic_manager.global_percentage_speed_difference(30.0)
            self.spawn_traffic(self.num_vehicles, self.num_walkers)
        
        else:
            self.traffic_manager=None
            
            
            
        self.spawn_objects_node.bridge_node.spin()
        
            
            

    def spawn_object(self, req):
        if not self.shutdown.is_set():
            id_ = self.actor_factory.spawn_actor(req)
            self._registered_actors.append(id_)

    def destroy_object(self, id_):
        destroyed_actors = self.actor_factory.destroy_actor(id_)
        success = bool(destroyed_actors)
        for actor in destroyed_actors:
            if actor in self._registered_actors:
                self._registered_actors.remove(actor)
        return success
    
    
    def _on_collision(self, event):
        ego_vehicle_collision={}
        if event.other_actor.type_id != "static.road":
            # if len(event.other_actor.attributes) != 0 and (event.other_actor.type_id == "vehicle" or event.other_actor.type_id=="pedestrian"):
            ego_vehicle_collision={
                "collision:":True,
                "collision with:": event.other_actor.type_id,
                "collision with 2:":event.other_actor.id}
                
            self.collision_events.append(ego_vehicle_collision)
         
            # with open(os.path.join(args.temp_folder,"ego_vehicle_collision.json"),'w', encoding='utf-8') as fp:
            #     json.dump(ego_vehicle_collision, fp, sort_keys=False, indent=4)
            
    def _on_lane_invasion(self,event):
        ego_vehicle_invasion={}
                
        ego_vehicle_invasion={
            "invasion:":True,
            "invasion on:": event.crossed_lane_markings}
                
        self.lane_invasion_events.append(ego_vehicle_invasion)
                
                
                
                
      
    #FIXME: 看起来现在引入了新的问题，现在是不进入下面的try 部分的，应该是生成的车辆数和应该生成的数量不符合。
    def _synchronous_mode_update(self):
        """
        执行同步模式的更新循环
        """
        print("Starting synchronous mode update thread")
        
        initial_check_done = False  # 初始化标志，表示初始检查是否完成
        
        while not self.shutdown.is_set():
            # 尝试发送车辆位置
            if not initial_check_done:  # 只在第一次调用时执行
                pos_n_0 = True  # 初始假设为True
                vehicles = self.carla_world.get_actors().filter('vehicle.*')

            # 记录有效位置的车辆数量
            valid_vehicle_count = 0

            for vehicle in vehicles:
                vehicle_position = vehicle.get_location()
                role_name = vehicle.attributes["role_name"]
                
                # 检查车辆位置是否为(0, 0)
                if vehicle_position.x == 0 and vehicle_position.y == 0:
                    pos_n_0 = False  # 如果有车辆位置为(0, 0)，则设置为False
                    break  # 退出循环，因为已经确定有车辆位置为(0, 0)
                else:
                    valid_vehicle_count += 1  # 统计有效位置的车辆数量

                # 记录其他信息
                # ind = role_name != "ego_vehicle"
                # determine Whether there are any misbehavior. If any, just recrod using the belowing self.log
                if role_name=="ego_vehicle":
                    accel=vehicle.get_acceleration() # type if vector3D
                    velocity=vehicle.get_velocity()# type is vector3D , in m/s
                    speed_in_kmh = velocity.length() * 3.6
                    speed_limit=vehicle.get_speed_limit() # float, km/h
                    
                    if accel.x >= 4 or accel.y >= 4 or accel.z >= 4:
                        self.fast_accel=True
                        
                    if accel.x <= -4 or accel.y<=-4 or accel.z <=-4:
                        self.hard_braking=True
                    if speed_in_kmh > speed_limit * 1.1:
                        self.speeding=True
                    

                ev_type = type(role_name)
                self.log.info(f"====================vehicle role name is {role_name}, position is {vehicle_position.x},{vehicle_position.y}, pos_n_0 is {pos_n_0}, ev type is {ev_type}")
                self.log.info("----Check vehicle position at line 779 in main.py--------------------------------")
                self.log.info(f"vehicle id is {vehicle.id}, vehicle spawn position is {vehicle_position.x},{vehicle_position.y},{vehicle_position.z} ")
                self.log.info(f"the initial replay vehicle number is {self.replay_init_vehicle_num}")
                self.log.info(f"the counted vehicle number is {valid_vehicle_count}")




            try:
                # if self.replay:
                if self.replay and self.replay_data and pos_n_0 and valid_vehicle_count == self.replay_init_vehicle_num+1:
                        #实际上这个indicator没有起到作用，不管了先这样吧
                        initial_check_done = True

                    # Replay 模式的更新逻辑
                    # if self.call_carla_spawn_objs and self.spawn_objects_node.actor_spawned and self.replay_data:
                    
                        try:
                            self.log.info("------------------------------Before Calling the Process Replay Frame--------------------")
                            self._process_replay_frame()
                            self.log.info("------------------------------After Calling the Process Replay Frame--------------------")
                            

                            if self.replay_frame_index >= len(self.replay_data):

                                status = {
                                    'replay_complete': True,
                                    'frame_index': self.replay_frame_index,
                                    'total_frames': len(self.replay_data)
                                }

                                self.send_tcp_data(json.dumps(status))

                                self.shutdown.set()
                                break
                            

                            if self.replay_frame_index % 100 == 0:
                                status = {
                                    'replay_complete': False,
                                    'progress': self.replay_frame_index / len(self.replay_data),
                                    'frame_index': self.replay_frame_index,
                                    'total_frames': len(self.replay_data)
                                }
                                self.send_tcp_data(json.dumps(status))
                            

                            
                        except Exception as e:
                            self.log.error(f"Error in replay update: {str(e)}")
                            traceback.print_exc()
                                
                else:
                    if not self.ego_vehicle:
                        vehicles = self.carla_world.get_actors().filter('vehicle.*')
                        for vehicle in vehicles:
                            if vehicle.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                                self.ego_vehicle = vehicle
                                
                                #FIXME: temp set physics as true to test 
                                # self.ego_vehicle.set_simulate_physics(True)
                                
                                
                                if not self.collision_detector:
                                    collision_bp=self.carla_world.get_blueprint_library().find('sensor.other.collision')
                                    self.collision_detector=self.carla_world.spawn_actor(collision_bp, carla.Transform(), attach_to = self.ego_vehicle)
                                    
                                    self.collision_detector.listen(lambda event: _on_collision(event))
                                    #TODO: 1. modify the on collision function 2. destroy the collision detector at the finally part 
                                if not self.lane_invasion_detector:
                                    lane_invasion_detector_bp = self.carla_world.get_blueprint_library().find('sensor.other.lane_invasion')
                                    self.lane_invasion_detector= self.carla_world.spawn_actor(lane_invasion_detector_bp, carla.Transform(), attach_to=self.ego_vehicle)
                                    self.lane_invasion_detector.listen(lambda event: _on_lane_invasion(event))
                                                                    
                                    
                                self.log.info(f"Ego Vehicle found: {vehicle.id}")
                                break
                            
                    # # 普通模式的更新逻辑--实际上就是没更新
                    # if self.call_carla_spawn_objs:
                    #     if self.spawn_objects_node.actor_spawned:
                    #         pass  # 普通模式下的其他逻辑
                
                # 共同的世界更新逻辑
                frame = self.carla_world.tick()
                world_snapshot = self.carla_world.get_snapshot()
                
                self.update_clock(world_snapshot.timestamp)
                self._update(frame, world_snapshot.timestamp.elapsed_seconds)
                self.actor_factory.update_available_objects()


                if not self.replay and self.ego_vehicle:
                    self.record_frame_data(world_snapshot.frame)

                

                
                
            except Exception as e:
                self.log.error(f"Error in synchronous update: {str(e)}")
                traceback.print_exc()
                time.sleep(0.1)    
                
      
      
      
                
                
                
    def _initialize_traffic_lights(self):
        """初始化交通灯状态"""
        try:
            if self.replay_frame_index == 0 and self.replay_data:
                traffic_lights_data = self.replay_data[0].get("traffic_lights", [])
                current_traffic_lights = self.carla_world.get_actors().filter('traffic.traffic_light*')
                
                # 通过位置匹配交通灯并设置初始状态
                for tl_data in traffic_lights_data:
                    target_loc = tl_data["location"]
                    
                    # 找到位置最接近的交通灯
                    for light in current_traffic_lights:
                        light_loc = light.get_location()
                        distance = abs(light_loc.x - target_loc["x"]) + \
                                abs(light_loc.y - target_loc["y"]) + \
                                abs(light_loc.z - target_loc["z"])
                                
                        if distance < 0.1:  # 0.1米的阈值
                            # 设置交通灯状态
                            target_state = tl_data["state"]
                            if target_state == "Red":
                                light.set_state(carla.TrafficLightState.Red)
                            elif target_state == "Yellow":
                                light.set_state(carla.TrafficLightState.Yellow)
                            elif target_state == "Green":
                                light.set_state(carla.TrafficLightState.Green)
                            elif target_state == "Off":
                                light.set_state(carla.TrafficLightState.Off)
                                
                            self.log.info(f"Initialized traffic light at {target_loc} to {target_state}")
                            break
                            
        except Exception as e:
            self.log.error(f"Error initializing traffic lights: {str(e)}")
            traceback.print_exc()


    
    #TODO: 这里缺少对于generate  traffic和 record data 的清理，等下加上-- otherwise everytime we need to kill the process on 8000 manually
    def cleanup(self):

        
        
        """清理FIFO资源"""
        # 只在replay模式下清理FIFO
        if not hasattr(self, 'replay') or not self.replay:
            return

        with self._pipe_lock:
            if not self._pipe_open:
                return

            try:
               
                if self.tcp_client_socket:
                    self.tcp_client_socket.close()
                    self.log.info("TCP client socket closed")


                if self.status_pipe is not None:
                    try:
                        os.close(self.status_pipe)
                    except OSError:
                        pass
                    self.status_pipe = None
                
                if os.path.exists(self.pipe_path):
                    try:
                        os.unlink(self.pipe_path)
                    except OSError:
                        pass
                
                self._pipe_open = False
                self.log.info("FIFO cleanup completed")
                
            except Exception as e:
                self.log.error(f"Error during FIFO cleanup: {e}")
            finally:
                self.status_pipe = None
                self._pipe_open = False



    def _get_vehicle_data(self, vehicle):
        """获取完整的车辆状态数据"""
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        angular_velocity = vehicle.get_angular_velocity()
        acceleration = vehicle.get_acceleration()
        control = vehicle.get_control()
        
        def vector_magnitude(vector):
            return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
        
        
        print("[====REPLAY RECORD=====]recorded ego_vehicle_velocity:", "x:", velocity.x, "y:",velocity.y, "z:", velocity.z)
        
        return {
            'id': vehicle.id,
            'type_id': vehicle.type_id,
            'transform': {
                'location': {'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z},
                'rotation': {'pitch': transform.rotation.pitch, 'yaw': transform.rotation.yaw, 'roll': transform.rotation.roll}
            },
            'velocity': {
                'x': velocity.x,
                'y': velocity.y,
                'z': velocity.z,
                'magnitude': vector_magnitude(velocity)
            },
            'angular_velocity': {
                'x': angular_velocity.x,
                'y': angular_velocity.y,
                'z': angular_velocity.z,
                'magnitude': vector_magnitude(angular_velocity)
            },
            'acceleration': {
                'x': acceleration.x,
                'y': acceleration.y,
                'z': acceleration.z,
                'magnitude': vector_magnitude(acceleration)
            },
            'control': {
                'throttle': control.throttle,
                'steer': control.steer,
                'brake': control.brake,
                'hand_brake': control.hand_brake,
                'reverse': control.reverse,
                'manual_gear_shift': control.manual_gear_shift,
                'gear': control.gear
            }
        }
        
    def _get_nearby_vehicles(self, world, ego_vehicle, max_distance=75.0):
        """获取附近车辆的数据"""
        nearby_vehicles = []
        ego_location = ego_vehicle.get_location()
        
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.id != ego_vehicle.id:
                distance = actor.get_location().distance(ego_location)
                if distance <= max_distance:
                    vehicle_data = {
                        **self._get_vehicle_data(actor),
                        'distance_to_ego': distance
                    }
                    
                    velocity = actor.get_velocity()
                    # if vehicle_data['velocity']['magnitude'] > 0:
                    #     self.logger.debug(
                    #         f"Other vehicle speed - ID: {actor.id}, "
                    #         f"Speed: x={velocity.x:.2f}, y={velocity.y:.2f}, z={velocity.z:.2f}"
                    #     )
                    
                    nearby_vehicles.append(vehicle_data)
        
        return nearby_vehicles
    
    
    
    def _get_walker_data(self, walker):
        """获取完整的车辆状态数据"""
        transform = walker.get_transform()
        velocity = walker.get_velocity()
        angular_velocity = walker.get_angular_velocity()
        acceleration = walker.get_acceleration()
        control = walker.get_control()
        
        def vector_magnitude(vector):
            return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)
        
        return {
            'id': walker.id,
            'type_id': walker.type_id,
            'transform': {
                'location': {'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z},
                'rotation': {'pitch': transform.rotation.pitch, 'yaw': transform.rotation.yaw, 'roll': transform.rotation.roll}
            },
            'velocity': {
                'x': velocity.x,
                'y': velocity.y,
                'z': velocity.z,
                'magnitude': vector_magnitude(velocity)
            },
            'angular_velocity': {
                'x': angular_velocity.x,
                'y': angular_velocity.y,
                'z': angular_velocity.z,
                'magnitude': vector_magnitude(angular_velocity)
            },
            'acceleration': {
                'x': acceleration.x,
                'y': acceleration.y,
                'z': acceleration.z,
                'magnitude': vector_magnitude(acceleration)
            },
            'control': {
        
                "direction_x": control.direction.x,
                "direction_y": control.direction.y,
                "direction_z": control.direction.z,
                
                "speed": control.speed,
                "jump": control.jump,
            }
        }
        
    def _get_nearby_walkers(self, world, ego_vehicle, max_distance=75.0):
        """获取附近车辆的数据"""
        nearby_walkers = []
        ego_location = ego_vehicle.get_location()
        
        for actor in world.get_actors().filter('walker.*'):
            if actor.id != ego_vehicle.id:
                distance = actor.get_location().distance(ego_location)
                if distance <= max_distance:
                    walker_data = {
                        **self._get_walker_data(actor),
                        'distance_to_ego': distance
                    }
                    
                    velocity = actor.get_velocity()
                    # if vehicle_data['velocity']['magnitude'] > 0:
                    #     self.logger.debug(
                    #         f"Other vehicle speed - ID: {actor.id}, "
                    #         f"Speed: x={velocity.x:.2f}, y={velocity.y:.2f}, z={velocity.z:.2f}"
                    #     )
                    
                    nearby_walkers.append(walker_data)
        
        return nearby_walkers
    
    
    
    
    
    
    
    
        
    def _get_traffic_lights(self, world, ego_vehicle, max_distance=75.0):
        """获取交通灯状态"""
        traffic_lights = []
        ego_location = ego_vehicle.get_location()
        
        for actor in world.get_actors().filter('traffic.traffic_light*'):
            distance = actor.get_location().distance(ego_location)
            if distance <= max_distance:
                location = actor.get_location()
                traffic_light_data = {
                    'id': actor.id,
                    'location': {
                        'x': round(location.x, 2),
                        'y': round(location.y, 2),
                        'z': round(location.z, 2)
                    },
                    'state': str(actor.get_state()),
                    'elapsed_time': actor.get_elapsed_time(),
                    # 删除了 time_until_change，因为这个属性在新版本中不存在
                    'distance': distance
                }
                traffic_lights.append(traffic_light_data)
                
        
    def save_replay_data(self):
        #  if not self.replay_frame_data:
        #     return
        try:
            # print("try to save the json data")
            os.makedirs(os.path.dirname(self.record_replay_path), exist_ok=True)
            with open(self.record_replay_path, 'w') as f:
                json.dump(self.replay_frame_data, f, indent=2)
        except Exception as e:
            print("Error saving data: ", e)
            # self.logger.error(f"Error saving data: {e}", exc_info=True)


    #FIXME: should we move it to a asyn thread to alleviate the influence of it on the main progress/
    def _process_replay_frame(self):
        """处理当前回放帧的数据"""
        try:            
            current_frame_data = self.replay_data[self.replay_frame_index]
            next_frame_data=None
            if self.replay_frame_index < len(self.replay_data) - 1:
                next_frame_data = self.replay_data[self.replay_frame_index + 1]
            
            current_vehicles = current_frame_data["nearby_vehicles"]
            next_vehicles=next_frame_data["nearby_vehicles"] if next_frame_data else None
            all_vehicles = self.carla_world.get_actors().filter('vehicle.*')
            self.log.info(f"next_frame_data: {next_frame_data}")
            
            # 记录当前帧中存在的vehicle ids
            current_vehicle_ids = set(str(vehicle["id"]) for vehicle in current_vehicles)
            existing_actor_ids = set()




            
            current_walkers = current_frame_data["nearby_walkers"]
            next_walkers=next_frame_data["nearby_walkers"] if next_frame_data else None
            all_walkers = self.carla_world.get_actors().filter('walker.*')
        

            # 记录当前帧中存在的walker ids
            current_walker_ids = set(str(walker["id"]) for walker in current_walkers)
            existing_walker_ids = set()
            
            

            
            original_frame_number = current_frame_data.get("frame", "unknown")
            self.log.info(
                f"\n{'='*50}\n"
                f"Processing Replay:\n"
                f"- Internal Frame Count: {self.replay_frame_count}\n"
                f"- Replay Frame Index: {self.replay_frame_index}\n"
                f"- Original JSON Frame: {original_frame_number}\n"
                f"- Replay Data Index: {self.replay_frame_index}/{len(self.replay_data)}\n"
                f"- Number of vehicles in Carla world: {len(all_vehicles)}\n"
                f"- Number of vehicles in replay data: {len(current_vehicles)}"
                f"- Number of walkers in Carla world: {len(all_walkers)}\n"
                f"- Number of walkers in replay data: {len(current_walkers)}"
            )

            snapshot=self.carla_world.get_snapshot()
            
        

            # 1. 更新现有actor的状态 # here the actor only contains the vehicle not the walker
            actors = self.actor_factory.actors.copy()
            for uid, actor in actors.items():
                if not hasattr(actor, 'carla_actor'):
                    continue
                # if not actor.carla_actor.type_id.startswith('vehicle.') or not actor.carla_actor.type_id.startswith('walker.'):
                if not actor.carla_actor.type_id.startswith('vehicle.'):
                    continue
                    
                current_actor_role_name = actor.carla_actor.attributes.get('role_name')
                if current_actor_role_name != 'ego_vehicle':
                    existing_actor_ids.add(current_actor_role_name)
                
                
                
                # 更新vehicle状态
                for vehicle_data in current_vehicles:
                    
                    if str(vehicle_data["id"]) == current_actor_role_name:
                        next_vehicle_data=None
                        if next_vehicles:
                            for next_vehicle in next_vehicles:
                                if str(next_vehicle["id"]) == current_actor_role_name:
                                    next_vehicle_data=next_vehicle
                                    break
                        
                        if self.replay_frame_index==0:
                            self._initialize_vehicle_state(actor, vehicle_data)
                        else:
                            self._update_vehicle_state(actor, vehicle_data, next_vehicle_data)
                        break
                
            
            #1.1 update the walker state
            actors=self.actor_factory.actors.copy()
            for uid, actor in actors.items():
                if not hasattr(actor, 'carla_actor'):
                    continue
                if not actor.carla_actor.type_id.startswith('walker.'):
                    continue
                    
                current_actor_role_name = actor.carla_actor.attributes.get('role_name')
                existing_walker_ids.add(current_actor_role_name)
                
                
                #update the walker state
                for walker_data in current_walkers:
                    if str(walker_data["id"]) == current_actor_role_name:
                        next_walker_data=None
                        if next_walkers:
                            for next_walker in next_walkers:
                                if str(next_walker["id"]) == current_actor_role_name:
                                    next_walker_data=next_walker
                                    break
                        if self.replay_frame_index==0:
                            self._initialize_walker_state(actor, walker_data)
                        else:
                            self._update_walker_state(actor, walker_data, next_walker_data)
                        break


            
            
            # 3. 销毁不再需要的actor
            actors_to_destroy = existing_actor_ids - current_vehicle_ids
            self.log.info(
                f"\nDestroy Check:\n"
                f"- Existing actor IDs in Carla (excluding ego): {existing_actor_ids}\n"
                f"- Vehicle IDs in current frame: {current_vehicle_ids}\n"
                f"- Actors to be destroyed: {actors_to_destroy}"
            )
            
            for actor_id in actors_to_destroy:
                for uid, actor in actors.items():
                    if (hasattr(actor, 'carla_actor') and 
                        actor.carla_actor.attributes.get('role_name') == actor_id):
                        actor.carla_actor.destroy()
                        del self.actor_factory.actors[uid]
                        self.log.info(f"Destroyed actor {actor_id}")
                        break
            
            
            
            #3.1 destroy the unneeded walker 
            walkers_to_destroy=existing_walker_ids - current_walker_ids
            self.log.info(
                f"\nDestroy Walker Check:\n"
                f"- Existing Walker IDs in Carla: {existing_walker_ids}\n"
                f"- Walker IDs in current frame: {current_walker_ids}\n"
                f"- Walkers to be destroyed: {walkers_to_destroy}"
            )
            
            for walker_id in walkers_to_destroy:
                for uid, actor in actors.items():
                    if (hasattr(actor, 'carla_actor') and 
                        actor.carla_actor.attributes.get('role_name') == walker_id):
                        actor.carla_actor.destroy()
                        del self.actor_factory.actors[uid]
                        self.log.info(f"Destroyed walker {walker_id}")
                        break
            
            
            
            
            
            
            
            
            # 4. 生成新的actor
            new_vehicle_ids = current_vehicle_ids - existing_actor_ids
            if new_vehicle_ids:
                self.log.info(
                    f"\nNew Actor Check:\n"
                    f"- New vehicle IDs to spawn: {new_vehicle_ids}"
                )
                
            #FIXME: 这两部分可以写在一起，避免重复遍历。
            for vehicle_data in current_vehicles:
                if str(vehicle_data["id"]) in new_vehicle_ids:
                    actor_id = self._spawn_replay_vehicle(vehicle_data)
                    #because previously the initiate state is setted in the actor_factor.py but now we need to set it by ourself
                    #TODO: adding the initialize vehicle state part
                    #first detect, if the vehicle is spawned,then allocate the initiate state        
                    if actor_id:
                        self.vehicle_manager.add_vehicles(actor_id, vehicle_data)
                        self._registered_actors.append(actor_id)
                        self.log.info(f"Spawned new vehicle {vehicle_data['id']}")
            
            vehicle_id_mapping = {}
            for vehicle_data in current_vehicles:
                old_id = str(vehicle_data["id"])  # json中记录的原始ID
                actors = self.actor_factory.actors.copy()
                for uid, actor in actors.items():
                    if (hasattr(actor, 'carla_actor') and 
                        actor.carla_actor.attributes.get('role_name') == old_id):
                        # 记录原始ID到新生成vehicle ID的映射
                        vehicle_id_mapping[old_id] = actor.carla_actor.id
                        break
                    
            
            
            #4.1 spawn new walker
            new_walker_ids=current_walker_ids - existing_walker_ids
            if new_walker_ids:
                self.log.info(
                    f"\nNew Walker Check:\n"
                    f"- New Walker IDs to spawn: {new_walker_ids}"
                )
            for walker_data in current_walkers:
                if str(walker_data["id"]) in new_walker_ids:
                    #TODO: need to supplement the replay actor data 
                    walker_id=self._spawn_replay_walker(walker_data)
                    #TODO: adding the initializing the walker data part 
                    if walker_id:
                        self._registered_actors.append(walker_id)
                        self.log.info(f"Spawned new walker {walker_data['id']}")
            
            #TODO: the final comparison part need to add the walker id      
            walker_id_mapping = {}
            for walker_data in current_walkers:
                old_id = str(walker_data["id"])  # json中记录的原始ID
                actors = self.actor_factory.actors.copy()
                for uid, actor in actors.items():
                    if (hasattr(actor, 'carla_actor') and 
                        actor.carla_actor.attributes.get('role_name') == old_id):
                        # 记录原始ID到新生成vehicle ID的映射
                        walker_id_mapping[old_id] = actor.carla_actor.id
                        break
            
            
            

            # print("----------------------------------------")
            # print("vehicle id mapping: ", vehicle_id_mapping)
            # print("----------------------------------------")
            
            """
            # 在所有处理完成后，才更新FIFO状态
            # the record replay data is within main.py, temp not use the FIFO now
            if self.status_pipe is not None:
                try:
                    status = {
                        'replay_index': self.replay_frame_index,
                        'original_frame': current_frame_data['frame'],
                        'carla_frame': snapshot.frame,
                        'replay_count': self.replay_frame_count,
                        'vehicle_id_mapping': vehicle_id_mapping
                    }
                    os.write(self.status_pipe, (json.dumps(status) + '\n').encode())
                except Exception as e:
                    self.log.error(f"Error writing to FIFO: {e}")
                    traceback.print_exc()
            """
            
                        
            #TODO: 应该是等所有处理都完成之后，再记录replay 的数据吧？ 
            #FIXME: 这里record的帧和replay播放的帧之间应该是会差1帧，这个应该是最后analysis 的时候需要注意的
            if self.record_replay_path is not None:
                ego_vehicle=None
                for actor in self.carla_world.get_actors():
                    if actor.attributes.get('role_name') == 'ego_vehicle':
                        ego_vehicle = actor
                        break
                if not ego_vehicle:
                    print("No ego vehicle found")
                    return
                

                frame_data = {
                    # 同步相关信息
                    'frame': snapshot.frame,
                    'replay_frame_index':self.replay_frame_index,
                    'original_frame': current_frame_data['frame'],
                    'replay_count': self.replay_frame_count,
                    # 'timestamp': frame_info['timestamp'],
        
                    # 使用get方法安全地获取ID映射关系
                    'vehicle_id_mapping': vehicle_id_mapping,
                    'walker_id_mapping' : walker_id_mapping,
                    # 车辆数据
                    'ego_vehicle': self._get_vehicle_data(ego_vehicle),
                    'nearby_vehicles': self._get_nearby_vehicles(self.carla_world, ego_vehicle),
                    'nearby_walkers':self._get_nearby_walkers(self.carla_world, ego_vehicle),
                    'traffic_lights': self._get_traffic_lights(self.carla_world, ego_vehicle),
                    # 性能指标
                    # 'processing_time': perf_counter() - start_time
                }
                self.replay_frame_data.append(frame_data)

                
                #save the data 
                current_time = time.time()
                if current_time - self.last_save_time >= self.save_interval:
                    # self.logger.info("Performing periodic save...")
                    self.save_replay_data()
                    self.last_save_time = current_time
                

            
            # 更新帧计数
            self.replay_frame_count += 1
            self.replay_frame_index += 1
            
            
            
        except Exception as e:
            self.log.error(f"Error in _process_replay_frame: {str(e)}")
            traceback.print_exc()
            
 
    def _update_vehicle_state(self, actor, vehicle_data, next_data):
        """
        更新车辆状态并检查差异
        """
        print("__________________________________update vehicle state")
        # 检查位置差异
        current_loc = actor.carla_actor.get_location()
        recorded_loc = carla.Location(
            x=vehicle_data["x"],
            y=vehicle_data["y"],
            z=vehicle_data["z"]
        )
        # 只使用x和y计算位置差异
        position_diff = math.sqrt(
            (current_loc.x - recorded_loc.x)**2 + 
            (current_loc.y - recorded_loc.y)**2
        )
        # position_diff = current_loc.distance(recorded_loc)
        
        # 检查速度差异
        current_vel = actor.carla_actor.get_velocity()
        recorded_vel = vehicle_data["velocity"]
        velocity_diff = math.sqrt(
            (current_vel.x - recorded_vel["x"])**2 +
            (current_vel.y - recorded_vel["y"])**2 +
            (current_vel.z - recorded_vel["z"])**2
        )
        
        # 记录显著差异
        # 获取当前车辆的role name
        if position_diff > self.POSITION_THRESHOLD or velocity_diff > self.VELOCITY_THRESHOLD:
            self.log.warning(
                f"Vehicle {actor.carla_actor.attributes.get('role_name')} has significant differences:\n"
                f"Position diff: {position_diff:.2f}m\n"
                f"Velocity diff: {velocity_diff:.2f}m/s\n"
                f"Current pos: ({current_loc.x:.2f}, {current_loc.y:.2f}, {current_loc.z:.2f})\n"
                f"Recorded pos: ({recorded_loc.x:.2f}, {recorded_loc.y:.2f}, {recorded_loc.z:.2f})"
            )
        
        # 记录当前速度
        self.log.info(
                    f"======In Update_Vehicle_State Function======"
                    f"[CURRENT] Vehicle {actor.carla_actor.attributes.get('role_name')} - "
                    f"Current Velocity: ({current_vel.x:.2f}, {current_vel.y:.2f}, {current_vel.z:.2f})")
        # 读取当前控制指令
        current_control = actor.carla_actor.get_control()
        self.log.info(f"[CURRENT] Vehicle {actor.carla_actor.attributes.get('role_name')} - "
                    f"Current Control: [Throttle: {current_control.throttle}, Steer: {current_control.steer}, "
                    f"Brake: {current_control.brake}, Hand Brake: {current_control.hand_brake}, "
                    f"Reverse: {current_control.reverse}, Gear: {current_control.gear}]")
        

        
        if next_data:
            control = carla.VehicleControl(
                throttle=next_data['throttle'],
                steer=next_data['steer'],
                brake=next_data['brake'],
                hand_brake=next_data['hand_brake'],
                reverse=next_data['reverse'],
                manual_gear_shift=next_data['manual_gear_shift'],
                gear=next_data['gear']
            )
            actor.carla_actor.apply_control(control)
        
            
            # 记录施加的控制指令
            self.log.info(f"[SETTINGS] Vehicle {actor.carla_actor.attributes.get('role_name')} - "
                        f"Applied Control: [Throttle: {control.throttle}, Steer: {control.steer}, "
                        f"Brake: {control.brake}, Hand Brake: {control.hand_brake}, "
                        f"Reverse: {control.reverse}, Gear: {control.gear}]")
            
            
    def _update_walker_state(self, actor, walker_data, next_data):
        """
        更新车辆状态并检查差异
        """
        print("__________________________________update walker state")
        # 检查位置差异
        current_loc = actor.carla_actor.get_location()
        recorded_loc = carla.Location(
            x=walker_data["x"],
            y=walker_data["y"],
            z=walker_data["z"]
        )
        # 只使用x和y计算位置差异
        position_diff = math.sqrt(
            (current_loc.x - recorded_loc.x)**2 + 
            (current_loc.y - recorded_loc.y)**2
        )
        # position_diff = current_loc.distance(recorded_loc)
        
        # 检查速度差异
        current_vel = actor.carla_actor.get_velocity()
        recorded_vel = walker_data["velocity"]
        velocity_diff = math.sqrt(
            (current_vel.x - recorded_vel["x"])**2 +
            (current_vel.y - recorded_vel["y"])**2 +
            (current_vel.z - recorded_vel["z"])**2
        )
        
        # 记录显著差异
        # 获取当前车辆的role name
        if position_diff > self.POSITION_THRESHOLD or velocity_diff > self.VELOCITY_THRESHOLD:
            self.log.warning(
                f"walker {actor.carla_actor.attributes.get('role_name')} has significant differences:\n"
                f"Position diff: {position_diff:.2f}m\n"
                f"Velocity diff: {velocity_diff:.2f}m/s\n"
                f"Current pos: ({current_loc.x:.2f}, {current_loc.y:.2f}, {current_loc.z:.2f})\n"
                f"Recorded pos: ({recorded_loc.x:.2f}, {recorded_loc.y:.2f}, {recorded_loc.z:.2f})"
            )
        
        # 记录当前速度
        self.log.info(
                    f"======In Update_Walker_State Function======"
                    f"[CURRENT] Walker {actor.carla_actor.attributes.get('role_name')} - "
                    f"Current Velocity: ({current_vel.x:.2f}, {current_vel.y:.2f}, {current_vel.z:.2f})")
        # 读取当前控制指令--temp not use it
        # current_control = actor.carla_actor.get_control()
        # self.log.info(f"[CURRENT] Walker {actor.carla_actor.attributes.get('role_name')} - "
        #             f"Current Control: [Throttle: {current_control.throttle}, Steer: {current_control.steer}, "
        #             f"Brake: {current_control.brake}, Hand Brake: {current_control.hand_brake}, "
        #             f"Reverse: {current_control.reverse}, Gear: {current_control.gear}]")
        

        
        if next_data:
            direction=carla.Vector3D(walker_data["direction_x"],walker_data["direction_y"],walker_data["direction_z"])
            speed=walker_data["speed"]
            jump=walker_data["jump"]
            
            control=carla.WalkerControl(direction, float(speed), bool(jump))
            
            actor.carla_actor.apply_control(control)
            
            
            # # 记录施加的控制指令---temp not use it 
            # self.log.info(f"[SETTINGS] Actor {actor.carla_actor.attributes.get('role_name')} - "
            #             f"Applied Control: [Throttle: {control.throttle}, Steer: {control.steer}, "
            #             f"Brake: {control.brake}, Hand Brake: {control.hand_brake}, "
            #             f"Reverse: {control.reverse}, Gear: {control.gear}]")
            
        
        
    def _initialize_vehicle_state(self, actor, vehicle_data):
        """
        更新车辆状态并检查差异
        """
        self.log.info("Call initializa vehicle state========================")
        current_loc = actor.carla_actor.get_location()
        recorded_loc = carla.Location(
            x=vehicle_data["x"],
            y=vehicle_data["y"],
            z=vehicle_data["z"]
        )
        # 只使用x和y计算位置差异
        position_diff = math.sqrt(
            (current_loc.x - recorded_loc.x)**2 + 
            (current_loc.y - recorded_loc.y)**2
        )
        # position_diff = current_loc.distance(recorded_loc)
        
        
        # FIXME: assign the velocity and control command, they are all received old data in the last tick  
        recorded_vel = vehicle_data["velocity"]
        velocity=carla.Vector3D(recorded_vel["x"],recorded_vel["y"],recorded_vel["z"])
        actor.carla_actor.set_target_velocity(velocity)
        
        
        control = carla.VehicleControl(
                throttle=vehicle_data['throttle'],
                steer=vehicle_data['steer'],
                brake=vehicle_data['brake'],
                hand_brake=vehicle_data['hand_brake'],
                reverse=vehicle_data['reverse'],
                manual_gear_shift=vehicle_data['manual_gear_shift'],
                gear=vehicle_data['gear']
            )
        actor.carla_actor.apply_control(control)
        
        

        # 记录显著差异
        # 获取当前车辆的role name
        if position_diff > self.POSITION_THRESHOLD:
            self.log.warning(
                f"Vehicle {actor.carla_actor.attributes.get('role_name')} has significant differences:\n"
                f"Position diff: {position_diff:.2f}m\n"
                # f"Velocity diff: {velocity_diff:.2f}m/s\n"
                f"Current pos: ({current_loc.x:.2f}, {current_loc.y:.2f}, {current_loc.z:.2f})\n"
                f"Recorded pos: ({recorded_loc.x:.2f}, {recorded_loc.y:.2f}, {recorded_loc.z:.2f})"
            )
        #TODO: 这个地方记录一下设置的速度以及控制指令，在后面的update 中也是，记录新的指令和旧的数据，我们来看看有没有完全的恢复状态


        # 记录设置的速度和控制指令
        self.log.info(f"[SETTINGS] Vehicle {actor.carla_actor.attributes.get('role_name')} - "
                    f"Set Velocity: ({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f}), "
                    f"Control: [Throttle: {control.throttle}, Steer: {control.steer}, "
                    f"Brake: {control.brake}, Hand Brake: {control.hand_brake}, "
                    f"Reverse: {control.reverse}, Gear: {control.gear}]")
        
        # 记录当前车辆的位置和控制指令
        current_vel=actor.carla_actor.get_velocity()
        current_accel = actor.carla_actor.get_acceleration()  # 获取当前加速度
        self.log.info(f"[CURRENT] Vehicle {actor.carla_actor.attributes.get('role_name')} - "
                    f"Current Position: ({current_loc.x:.2f}, {current_loc.y:.2f}, {current_loc.z:.2f}), "
                    f"Control: [Throttle: {actor.carla_actor.get_control().throttle}, "
                    f"Steer: {actor.carla_actor.get_control().steer}, "
                    f"Brake: {actor.carla_actor.get_control().brake}, "
                    f"Hand Brake: {actor.carla_actor.get_control().hand_brake}, "
                    f"Reverse: {actor.carla_actor.get_control().reverse}, "
                    f"Gear: {actor.carla_actor.get_control().gear}], "
                    f"Current Velocity: ({current_vel.x:.2f}, {current_vel.y:.2f}, {current_vel.z:.2f}), "
                    f"Current Acceleration: ({current_accel.x:.2f}, {current_accel.y:.2f}, {current_accel.z:.2f})")
        
    def _initialize_walker_state(self, actor, walker_data):
        """
        更新车辆状态并检查差异
        """
        self.log.info("Call initializa walker state========================")
        current_loc = actor.carla_actor.get_location()
        recorded_loc = carla.Location(
            x=walker_data["x"],
            y=walker_data["y"],
            z=walker_data["z"]
        )
        # 只使用x和y计算位置差异
        position_diff = math.sqrt(
            (current_loc.x - recorded_loc.x)**2 + 
            (current_loc.y - recorded_loc.y)**2
        )
        # position_diff = current_loc.distance(recorded_loc)
        
        
        # recorded_vel = walker_data["velocity"]
        # velocity=carla.Vector3D(recorded_vel["x"],recorded_vel["y"],recorded_vel["z"])
        # walker cannot use the set targey velocity fucntion thus we do not use it         
        # actor.carla_actor.set_target_velocity(velocity)
        
        
        
        direction=carla.Vector3D(walker_data["direction_x"],walker_data["direction_y"],walker_data["direction_z"])
        speed=walker_data["speed"]
        jump=walker_data["jump"]
        
        control=carla.WalkerControl(direction, float(speed), bool(jump))
        
        actor.carla_actor.apply_control(control)
        
        

        # 记录显著差异
        # 获取当前车辆的role name
        if position_diff > self.POSITION_THRESHOLD:
            self.log.warning(
                f"Walker {actor.carla_actor.attributes.get('role_name')} has significant differences:\n"
                f"Position diff: {position_diff:.2f}m\n"
                # f"Velocity diff: {velocity_diff:.2f}m/s\n"
                f"Current pos: ({current_loc.x:.2f}, {current_loc.y:.2f}, {current_loc.z:.2f})\n"
                f"Recorded pos: ({recorded_loc.x:.2f}, {recorded_loc.y:.2f}, {recorded_loc.z:.2f})"
            )

        # the applied value is not used since we are not in the debug mode
        
    def _spawn_replay_vehicle(self, vehicle_data):
        """
        根据回放数据生成新的车辆
        """
        try:
            # 创建spawn参数
            
            spawn_param = SpawnObjectParam()
            spawn_param.type = vehicle_data["type_id"]
            spawn_param.id = vehicle_data["id"]
            spawn_param.attach_to = 0  # 不需要附加到其他actor
            spawn_param.random_pose = False
            
    
            # 设置transform
            spawn_param.transform.position.x = float(vehicle_data["x"])
            spawn_param.transform.position.y = float(vehicle_data["y"])
            spawn_param.transform.position.z = float(vehicle_data["z"])
            
            roll=float(vehicle_data["roll"])
            pitch=float(vehicle_data["pitch"])
            yaw=float(vehicle_data["yaw"])
    
            
            quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

            spawn_param.transform.orientation.qx = quat[1]
            spawn_param.transform.orientation.qy = quat[2]
            spawn_param.transform.orientation.qz = quat[3]
            spawn_param.transform.orientation.qw = quat[0]
            
                
            # spawn_param.transform.rotation.yaw = float(vehicle_data["yaw"]) #FIXME: 应该是这里的transform 的key value 有问题
            # spawn_param.transform.rotation.roll = float(vehicle_data["roll"])
            # spawn_param.transform.rotation.pitch = float(vehicle_data["pitch"])
        
            # 设置初始速度
            spawn_param.initial_velocity = {
                "x": float(vehicle_data["velocity"]["x"]),
                "y": float(vehicle_data["velocity"]["y"]),
                "z": float(vehicle_data["velocity"]["z"])
            }
        
            # 设置初始角速度
            spawn_param.initial_angular_velocity = {
                "x": float(vehicle_data["angular_velocity"]["x"]),
                "y": float(vehicle_data["angular_velocity"]["y"]),
                "z": float(vehicle_data["angular_velocity"]["z"])
            }
                
            # 设置初始控制指令
            spawn_param.initial_control_command = {
                "throttle": float(vehicle_data["throttle"]),
                "steer": float(vehicle_data["steer"]),
                "brake": float(vehicle_data["brake"]),
                "gear": int(vehicle_data["gear"]),
                "hand_brake": bool(vehicle_data["hand_brake"]),
                "reverse": bool(vehicle_data["reverse"]),
                "manual_gear_shift": bool(vehicle_data["manual_gear_shift"])
            }
            
            # 设置角色名称属性
            # 这个replay_vehicle
            # role_name = KeyValue("role_name", "replay_vehicle")
            # spawn_param.attributes.append(role_name)
            
                
                
            # 调用actor_factory的spawn_actor方法
            actor_id = self.actor_factory.spawn_actor(spawn_param,spawn_replay_vehicle=True)
            
            if actor_id is not None:
                self.log.info(f"Successfully spawned replay vehicle {vehicle_data['id']} of type {vehicle_data['type_id']}")
                return actor_id
            else:
                self.log.error(f"Failed to spawn replay vehicle {vehicle_data['id']}")
                return None
                
        except Exception as e:
            self.log.error(f"Error spawning replay vehicle {vehicle_data['id']}: {str(e)}")
            traceback.print_exc()
            return None      
                
            
    def _spawn_replay_walker(self, walker_data):
        """
        根据回放数据生成新的车辆
        """
        try:
            # 创建spawn参数
            
            spawn_param = SpawnObjectParam()
            spawn_param.type = walker_data["type_id"]
            spawn_param.id = walker_data["id"]
            spawn_param.attach_to = 0  # 不需要附加到其他actor
            spawn_param.random_pose = False
            
    
            # 设置transform
            spawn_param.transform.position.x = float(walker_data["x"])
            spawn_param.transform.position.y = float(walker_data["y"])
            spawn_param.transform.position.z = float(walker_data["z"])
            
            roll=float(walker_data["roll"])
            pitch=float(walker_data["pitch"])
            yaw=float(walker_data["yaw"])
    
            
            quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

            spawn_param.transform.orientation.qx = quat[1]
            spawn_param.transform.orientation.qy = quat[2]
            spawn_param.transform.orientation.qz = quat[3]
            spawn_param.transform.orientation.qw = quat[0]
            
                
            # spawn_param.transform.rotation.yaw = float(vehicle_data["yaw"]) #FIXME: 应该是这里的transform 的key value 有问题
            # spawn_param.transform.rotation.roll = float(vehicle_data["roll"])
            # spawn_param.transform.rotation.pitch = float(vehicle_data["pitch"])
        
            # 设置初始速度
            spawn_param.initial_velocity = {
                "x": float(walker_data["velocity"]["x"]),
                "y": float(walker_data["velocity"]["y"]),
                "z": float(walker_data["velocity"]["z"])
            }
        
            # 设置初始角速度
            spawn_param.initial_angular_velocity = {
                "x": float(walker_data["angular_velocity"]["x"]),
                "y": float(walker_data["angular_velocity"]["y"]),
                "z": float(walker_data["angular_velocity"]["z"])
            }
                
            # 设置初始控制指令
            spawn_param.initial_control_command = {
                "direction_x": float(walker_data["direction_x"]),
                "direction_y": float(walker_data["direction_y"]),
                "direction_x": float(walker_data["direction_z"]),
                
                "speed": float(walker_data["speed"]),
                "jump": bool(walker_data["jump"]),
            }
            
            
            
            
            
            
            # direction=carla.Vector3D(walker_data["direction_x"],walker_data["direction_y"],walker_data["direction_z"])
            # speed=walker_data["speed"]
            # jump=walker_data["jump"]
            
            # control=carla.WalkerControl(direction, float(speed), bool(jump))
            
            # actor.carla_actor.apply_control(control)
            
            
            
            
                
                
            # 调用actor_factory的spawn_actor方法
            actor_id = self.actor_factory.spawn_actor(spawn_param,spawn_replay_vehicle=True)
            
            if actor_id is not None:
                self.log.info(f"Successfully spawned replay vehicle {walker_data['id']} of type {walker_data['type_id']}")
                return actor_id
            else:
                self.log.error(f"Failed to spawn replay vehicle {walker_data['id']}")
                return None
                
        except Exception as e:
            self.log.error(f"Error spawning replay vehicle {vehicle_walker_datadata['id']}: {str(e)}")
            traceback.print_exc()
            return None      

    def _update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        self.actor_factory.update_actor_states(frame_id, timestamp)

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if self.ok():
            self.timestamp = self.get_timestamp(
                carla_timestamp.elapsed_seconds, from_sec=True
            )
            self.clock_writer.write(
                Clock(clock=int(self.timestamp["secs"]))
            )


    def destroy(self):
        """Function to destroy this object."""
        self.log.info("Shutting down...")
        
        # 1. 先停止同步线程
        if not self.sync_mode:
            if self.on_tick_id:
                self.carla_world.remove_on_tick(self.on_tick_id)
        else:
            if self.synchronous_mode_update_thread:
                self.synchronous_mode_update_thread.join()
        self.log.info("Object update finished.")


        #new part added because of the generated traffic is moved here 
        try:
            # 停止所有行人
            for i in range(0, len(self.all_id), 2):
                try:
                    self.carla_world.get_actor(self.all_id[i]).stop()
                except Exception as e:
                    self.log.warning(f"Error stopping walker {self.all_id[i]}: {e}")
            
            # 清理所有车辆
            if self.vehicles_list:
                self.log.info(f"Destroying {len(self.vehicles_list)} vehicles")
                self.carla_client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])
                self.vehicles_list.clear()
            
            # 清理所有行人和行人控制器
            if self.all_id:
                self.log.info(f"Destroying {len(self.walkers_list)} walkers")
                self.carla_client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])
                self.all_id.clear()
                self.walkers_list.clear()
            
            # 重置traffic manager
            if self.traffic_manager is not None:
                self.traffic_manager.shut_down()
                self.traffic_manager = None
                
            # 等待一帧确保所有清理操作完成
            self.carla_world.tick()
            
            self.log.info("Traffic cleanup completed successfully")
            
            #destory 模式下存储最后的数据
            record_path = self.carla_parameters.get('record_path')
            if record_path:  
                parent_dir = os.path.dirname(record_path)
                if parent_dir:  # 如果父目录不为空
                    os.makedirs(parent_dir, exist_ok=True)
                    # 保存数据
                    with open(record_path, 'w') as f:
                        json.dump(self.frame_data, f, indent=4)
    
    
    
            metric_dir=self.carla_parameters.get('metric_dir')
            if metric_dir:
                os.makedirs(metric_dir, exist_ok=True)
                try:
                    if len(self.collision_events) !=0:
                        json_file_path = os.path.join(metric_dir, "collision_results.json")
                        with open(json_file_path, 'w') as f:
                            # 如果 collision_events 是空列表，也会正常序列化
                            json.dump(self.collision_events, f, indent=4)
                        print(f"Collision events saved to {json_file_path}")
                    #TODO: we need more infomation about the other metrics
                    if self.fast_accel:
                        json_file_path = os.path.join(metric_dir, "fast_accel.json")
                        with open(json_file_path, 'w') as f:
                            json.dump(self.fast_accel, f, indent=4)
                    if self.hard_braking:
                        json_file_path = os.path.join(metric_dir, "hard_braking.json")
                        with open(json_file_path, 'w') as f:
                            json.dump(self.hard_braking, f, indent=4)
                    if self.speeding:
                        json_file_path = os.path.join(metric_dir, "speeding.json")
                        with open(json_file_path, 'w') as f:
                            json.dump(self.speeding, f, indent=4)
                    if len(self.lane_invasion_events) != 0:
                        json_file_path = os.path.join(metric_dir, "lane_invasion_results.json")
                        with open(json_file_path, 'w') as f:
                            json.dump(self.lane_invasion_events, f, indent=4)
                    
                        
                except IOError as e:
                    print(f"IO error occurred: {e}")
                except TypeError as e:
                    print(f"Serialization error occurred: {e}")
    
            
        except Exception as e:
            self.log.error(f"Error during traffic cleanup: {e}")
            traceback.print_exc()






        # 2. 确保 actor_factory 存在并且可用
        if not hasattr(self, 'actor_factory') or self.actor_factory is None:
            self.log.error("Actor factory not initialized")
            return

        # 3. 清理所有已注册的 actors
        if hasattr(self, '_registered_actors') and self._registered_actors:
            self.log.info(f"Destroying {len(self._registered_actors)} registered actors")
            for uid in list(self._registered_actors):  # 使用列表副本进行迭代
                try:
                    self.log.info(f"Destroying actor {uid}")
                    self.actor_factory.destroy_actor(uid)
                except Exception as e:
                    self.log.error(f"Error destroying actor {uid}: {e}")
        
        # 4. 更新和清理 actor factory
        try:
            self.actor_factory.update_available_objects()
            self.actor_factory.clear()
        except Exception as e:
            self.log.error(f"Error clearing actor factory: {e}")

        self.log.info("All actors destroyed")
        super().destroy()




def main(argv):
    """
    main function for carla simulator Cyber bridge
    maintaining the communication client and the CarlaBridge object
    """
    
    log = init_log("bridge.log")
    carla_client = None
    carla_bridge = CarlaCyberBridge()
    # 1. 读取yaml配置
    config_file = os.path.dirname(os.path.abspath(__file__)) + "/config/settings.yaml"
    with open(config_file, encoding="utf-8") as f:
        parameters = yaml.safe_load(f)
        
    # 打印原始命令行参数
    print("="*50)
    print("Command Line Arguments:")
    print(f"Raw argv: {argv}")
    print(f"Replay flag: {FLAGS.replay}")
    print(f"Replay file: {FLAGS.replay_file}")
    print(f"objects file: {FLAGS.objects_file}")
    print(f"replay log: {FLAGS.replay_log}")
    
    print(f"Number of vehicles: {FLAGS.num_vehicles}")
    print(f"Number of walkers: {FLAGS.num_walkers}")
    print(f"Record path: {FLAGS.record_path}")
    print(f"Metric Dir: {FLAGS.metric_dir}")
    
    print("="*50)
    print("record replay path: ", FLAGS.record_replay_path)
    

    log.info("Command line arguments received:")
    log.info(f"Replay: {FLAGS.replay}")
    log.info(f"Replay file: {FLAGS.replay_file}")
    del argv
    
    # config_file = os.path.dirname(os.path.abspath(__file__)) + "/config/settings.yaml"
    # with open(config_file, encoding="utf-8") as f:
    #     parameters = yaml.safe_load(f)
        
    # 2. 命令行参数覆盖yaml配置
    if FLAGS.replay is not None:
        parameters['carla']['replay'] = FLAGS.replay
    if FLAGS.replay_file is not None:
        parameters['carla']['replay_file'] = FLAGS.replay_file
    
    if FLAGS.replay_log is not None:
        parameters['carla']['replay_log'] = FLAGS.replay_log
    
    
    if FLAGS.record_path:
        parameters['carla']['record_path'] = FLAGS.record_path
        
    if FLAGS.metric_dir:
        parameters['carla']['metric_dir']=FLAGS.metric_dir
    # if FLAGS.record_interval:
        # parameters['carla']['record_interval'] = FLAGS.record_interval
    if FLAGS.num_vehicles is not None:
        parameters['carla']['num_vehicles'] = FLAGS.num_vehicles
    if FLAGS.num_walkers is not None:
        parameters['carla']['num_walkers'] = FLAGS.num_walkers
    
    if FLAGS.record_replay_path is not None:
        parameters['carla']['record_replay_path'] = FLAGS.record_replay_path
    
    
    carla_parameters = parameters["carla"]
        
        
    log.info(
        f"Trying to connect to {carla_parameters['host']}:{carla_parameters['port']}"
    )

    #Read in the vehicle_log.json and pass the necessary arguments to the corresponding functions

    # replay=True
    try:
        carla_client = carla.Client(
            host=carla_parameters["host"], port=carla_parameters["port"]
        )
        carla_client.set_timeout(carla_parameters["timeout"])

        carla_client.load_world(carla_parameters["town"])
        carla_world = carla_client.get_world()



        log.info(f"Connect to {carla_parameters['host']}:{carla_parameters['port']} successfully.")

            
        traffic_lights = carla_world.get_actors().filter('traffic.traffic_light')

        # 将所有交通灯设置为绿灯
        for traffic_light in traffic_lights:
            traffic_light.set_state(carla.TrafficLightState.Green)  # 设置为绿灯
            traffic_light.set_green_time(99999)  # 设置绿灯持续时间为无限
        print("before entering the carla bridge initialize bridge")
        carla_bridge.initialize_bridge(carla_client, carla_world, parameters, log)
    except (IOError, RuntimeError) as e:
        log.error(f"Error: {e}")
        traceback.print_exc()
    except KeyboardInterrupt as e:
        log.error(f"Error: {e}")
    except Exception as e:  # pylint: disable=W0718
        log.error(e)
        traceback.print_exc()
        
    finally:
        try:
            log.warning("Starting shutdown sequence...")
            
            # 1. 重置 CARLA 设置
            if carla_world:
                settings = carla_world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                carla_world.apply_settings(settings)
            
            # 2. 设置关闭标志
            if carla_bridge and carla_bridge.shutdown:
                carla_bridge.shutdown.set()
                log.info("Shutdown flag set")
            
            # 3. 销毁 bridge（这会触发 actor_factory 的清理）
            if carla_bridge:
                log.info("Calling bridge destroy")
                carla_bridge.destroy()
                log.info("Bridge destroy completed")
            
            # # 4. 清理交通车辆
            # if vehicles_list:
            #     log.info(f'Destroying {len(vehicles_list)} traffic vehicles')
            #     carla_client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
            
            # 5. 最后清理 CARLA 资源
            if carla_world:
                log.info("Cleaning up CARLA world")
                del carla_world
            if carla_client:
                log.info("Cleaning up CARLA client")
                del carla_client
                
            log.warning("Shutdown sequence completed")
            
        except Exception as e:
            log.error(f"Error during cleanup: {e}")
            traceback.print_exc()
            
        
if __name__ == "__main__":
    try:
        app.run(main)
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        traceback.print_exc()
        raise e
    