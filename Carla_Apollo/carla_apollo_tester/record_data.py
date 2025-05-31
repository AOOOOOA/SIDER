# #!/usr/bin/env python

# # Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# # Barcelona (UAB).
# #
# # This work is licensed under the terms of the MIT license.
# # For a copy, see <https://opensource.org/licenses/MIT>.

# """Example script to generate traffic in the simulation"""

# import glob
# import os
# import sys
# import time

# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass

# import carla

# from carla import VehicleLightState as vls

# import argparse
# import logging
# from numpy import random
# import json

    
    
# import math

# def get_nearby_traffic_lights(world, vehicle_location, radius=50):
#     """获取指定范围内的交通灯状态"""
#     traffic_lights = []
#     for actor in world.get_actors().filter('traffic.traffic_light*'):
#         if actor.get_location().distance(vehicle_location) < radius:
#             location = actor.get_location()
#             traffic_light_data = {
#                 "location": {
#                     "x": round(location.x, 2),
#                     "y": round(location.y, 2),
#                     "z": round(location.z, 2)
#                 },
#                 "state": str(actor.state),
#                 "elapsed_time": actor.get_elapsed_time(),
#                 "remaining_time": actor.get_time_until_state_change()
#             }
#             traffic_lights.append(traffic_light_data)
#     return traffic_lights

# def record_vehicle_info(world, ego_vehicle, frame_data, distance_threshold=50.0):
#     """
#     记录 Ego Vehicle 及其附近车辆的状态信息，并存储在 frame_data 列表中。
    
#     :param world: CARLA 仿真世界对象
#     :param ego_vehicle: Ego Vehicle 对象
#     :param frame_data: 用于存储每帧数据的列表
#     :param distance_threshold: 记录的距离阈值，单位为米
#     """
#     # 获取仿真当前的帧和时间戳
#     snapshot = world.get_snapshot()
#     frame = snapshot.frame
#     if frame % 100 == 0:  # 每100帧显示一次
#         print(f"Recorded {len(frame_data)} frames, current frame: {frame}")
#     timestamp = snapshot.timestamp.elapsed_seconds

#     # Ego Vehicle 的位置和朝向
#     ego_transform = ego_vehicle.get_transform()
#     ego_location = ego_transform.location
#     ego_rotation = ego_transform.rotation

#     # 获取 Ego Vehicle 的控制状态
#     ego_control = ego_vehicle.get_control()

#     # 获取 Ego Vehicle 的速度、角速度和加速度
#     ego_velocity = ego_vehicle.get_velocity()
#     ego_angular_velocity = ego_vehicle.get_angular_velocity()
#     ego_acceleration = ego_vehicle.get_acceleration()

#     # 辅助函数，计算向量的标量值（大小）
#     def vector_magnitude(vector):
#         return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

#     # 存储 Ego Vehicle 的信息
#     ego_vehicle_data = {
#         'id': ego_vehicle.id,
#         'x': ego_location.x,
#         'y': ego_location.y,
#         'z': ego_location.z,
#         'yaw': ego_rotation.yaw,
#         'roll': ego_rotation.roll,
#         'pitch': ego_rotation.pitch,
#         'throttle': ego_control.throttle,
#         'steer': ego_control.steer,
#         'brake': ego_control.brake,
#         'gear': ego_control.gear,
#         'hand_brake': ego_control.hand_brake,
#         'reverse': ego_control.reverse,
#         'manual_gear_shift': ego_control.manual_gear_shift,
#         'velocity': {
#             'x': ego_velocity.x,
#             'y': ego_velocity.y,
#             'z': ego_velocity.z,
#             'magnitude': vector_magnitude(ego_velocity)  # 速度的标量值
#         },
#         'angular_velocity': {
#             'x': ego_angular_velocity.x,
#             'y': ego_angular_velocity.y,
#             'z': ego_angular_velocity.z,
#             'magnitude': vector_magnitude(ego_angular_velocity)  # 角速度的标量值
#         },
#         'acceleration': {
#             'x': ego_acceleration.x,
#             'y': ego_acceleration.y,
#             'z': ego_acceleration.z,
#             'magnitude': vector_magnitude(ego_acceleration)  # 加速度的标量值
#         }
#     }

#     # 获取所有车辆的列表
#     vehicles = world.get_actors().filter('vehicle.*')

#     # 存储附近车辆的信息
#     nearby_vehicles = []
#     for vehicle in vehicles:
#         if vehicle.id != ego_vehicle.id:  # 排除 Ego Vehicle 本身
#             vehicle_transform = vehicle.get_transform()
#             vehicle_location = vehicle_transform.location
#             vehicle_rotation = vehicle_transform.rotation

#             # 计算车辆与 Ego Vehicle 的距离
#             distance = ego_location.distance(vehicle_location)

#             if distance <= distance_threshold:
#                 # 获取车辆的控制状态
#                 vehicle_control = vehicle.get_control()

#                 # 获取车辆的速度、角速度和加速度
#                 vehicle_velocity = vehicle.get_velocity()
#                 vehicle_angular_velocity = vehicle.get_angular_velocity()
#                 vehicle_acceleration = vehicle.get_acceleration()

#                 # 获取车辆的类型 ID
#                 vehicle_type_id = vehicle.type_id  # 获取车辆的类型，例如 "vehicle.tesla.model3"

#                 # 存储附近车辆的信息
#                 nearby_vehicles.append({
#                     'id': vehicle.id,
#                     'type_id': vehicle_type_id,  # 记录车辆的类型
#                     'x': vehicle_location.x,
#                     'y': vehicle_location.y,
#                     'z': vehicle_location.z,
#                     'yaw': vehicle_rotation.yaw,
#                     'roll': vehicle_rotation.roll,
#                     'pitch': vehicle_rotation.pitch,
#                     'throttle': vehicle_control.throttle,
#                     'steer': vehicle_control.steer,
#                     'brake': vehicle_control.brake,
#                     'gear': vehicle_control.gear,
#                     'hand_brake': vehicle_control.hand_brake,
#                     'reverse': vehicle_control.reverse,
#                     'manual_gear_shift': vehicle_control.manual_gear_shift,
#                     'velocity': {
#                         'x': vehicle_velocity.x,
#                         'y': vehicle_velocity.y,
#                         'z': vehicle_velocity.z,
#                         'magnitude': vector_magnitude(vehicle_velocity)  # 速度的标量值
#                     },
#                     'angular_velocity': {
#                         'x': vehicle_angular_velocity.x,
#                         'y': vehicle_angular_velocity.y,
#                         'z': vehicle_angular_velocity.z,
#                         'magnitude': vector_magnitude(vehicle_angular_velocity)  # 角速度的标量值
#                     },
#                     'acceleration': {
#                         'x': vehicle_acceleration.x,
#                         'y': vehicle_acceleration.y,
#                         'z': vehicle_acceleration.z,
#                         'magnitude': vector_magnitude(vehicle_acceleration)  # 加速度的标量值
#                     },
#                     'distance_to_ego': distance
#                 })
#                 print("-----other vehicle speed:", vehicle_velocity.x, vehicle_velocity.y, vehicle_velocity.z)
                

#     # 计算 Ego Vehicle 周围的车辆数量
#     nearby_vehicle_count = len(nearby_vehicles)

#     # 获取交通灯信息
#     traffic_lights = get_nearby_traffic_lights(world, ego_location, radius=distance_threshold)

#     # 将每帧数据存入列表
#     frame_data.append({
#         'frame': frame,
#         'timestamp': timestamp,
#         'ego_vehicle': ego_vehicle_data,
#         'nearby_vehicle_count': nearby_vehicle_count,  # 记录附近车辆的数量
#         'nearby_vehicles': nearby_vehicles,
#         'traffic_lights': traffic_lights
#     })
    
#     return frame_data

# def main():
#     argparser = argparse.ArgumentParser(
#         description=__doc__)
#     argparser.add_argument(
#         '--host',
#         metavar='H',
#         default='127.0.0.1',
#         help='IP of the host server (default: 127.0.0.1)')
#     argparser.add_argument(
#         '-p', '--port',
#         metavar='P',
#         default=2000,
#         type=int,
#         help='TCP port to listen to (default: 2000)')
#     # 添加新的命令行参数
#     argparser.add_argument(
#         '-o', '--output',
#         metavar='O',
#         default='vehicle_log_extra.json',
#         help='output file path (default: vehicle_log_extra.json)')
#     argparser.add_argument(
#         '--threshold',
#         type=float,
#         default=75.0,
#         help='Distance threshold for recording nearby objects (default: 75.0)'
#     )
#     #TODO: 这个threshold 也需要做成一个接口

#     args = argparser.parse_args()

#     client = carla.Client(args.host, args.port)
#     client.set_timeout(10.0)

#     try:
#         # client.load_world('Town01')
#         world = client.get_world()

#         settings = world.get_settings()


#         # 找到ego vehicle; 否则不进入下一个循环（强制在这里循环等待apollo 车辆生成
#         ego_vehicle = None
#         while not ego_vehicle:
#             print("searching for ego vehicle...")
#             vehicles=world.get_actors().filter('vehicle.*')
#             for vehicle in vehicles:
#                 # if vehicle.attributes.get('role_name') == 'weiwei':
#                 # if vehicle.attributes.get('role_name') == 'ego_vehicle' or 'hero':
#                 if vehicle.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                
#                     ego_vehicle=vehicle
#                     print(f"Ego Vehicle found: {ego_vehicle.id}")
#                     break
#         #只有找到了ego vehicle, 才可以让程序进入tick 的循环里。
#         # output_file = 'vehicle_log_extra.json'
#         output_file=args.output
#         frame_data = []
        

#         while True:
        
#             world.wait_for_tick()
#             frame_data=record_vehicle_info(world, ego_vehicle, frame_data, distance_threshold=args.threshold)
#             ego_vehicle_velocity=ego_vehicle.get_velocity()
#             print(f"Ego Vehicle Speed: {ego_vehicle_velocity.x:.2f}, {ego_vehicle_velocity.y:.2f}, {ego_vehicle_velocity.z:.2f}")

#     finally:
#         with open(output_file, 'w') as f:
#             json.dump(frame_data, f, indent=4)
#         settings = world.get_settings()
#         settings.synchronous_mode = False
#         settings.no_rendering_mode = False
#         settings.fixed_delta_seconds = None
#         world.apply_settings(settings)

#         time.sleep(0.5)

# if __name__ == '__main__':

#     try:
#         main()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         print('\ndone.')



#!/usr/bin/env python
import glob
import os
import sys
import time
import argparse
import logging
import json
import math
import carla
from loguru import logger
import signal
import atexit
import traceback

frame_data = []
client = None
args = None

def save_data(force_sync=True):
    """保存数据的通用函数"""
    global frame_data, args, client
    try:
        if not frame_data:
            logger.warning("No data to save")
            return

        # 停止录制并保存数据
        if args and args.carla_record and client:
            client.stop_recorder()
            logger.info("Stopped Carla recording")

        print("-------------------------------------------- SAVE the JSON LOG DATA---------------")
        logger.info(f"++++++++++++++++++++++++++++++++++++++= the output dir is :, {args.output}")
        
        # 确保输出目录存在
        output_dir = os.path.dirname(os.path.abspath(args.output))
        os.makedirs(output_dir, exist_ok=True)
        
        # 使用临时文件保存
        temp_file = args.output + '.temp'
        with open(temp_file, 'w') as f:
            json.dump(frame_data, f, indent=4)
            if force_sync:
                f.flush()
                os.fsync(f.fileno())
        
        # 重命名临时文件为最终文件
        os.replace(temp_file, args.output)
        
        logger.info(f"Successfully saved {len(frame_data)} frames to {args.output}")
    except Exception as e:
        logger.error(f"Error saving data: {e}")
        traceback.print_exc()

#FIXME: cleanup is called for multiple times
def cleanup():
    """清理函数，注册为退出处理程序"""
    print("\nCleaning up...")
    save_data(force_sync=True)
    print("Cleanup completed")

def signal_handler(signum, frame):
    """处理终止信号"""
    print(f"\nReceived signal {signum}, saving data and cleaning up...")
    cleanup()
    sys.exit(0)

def get_nearby_traffic_lights(world, vehicle_location, radius=50):
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

def record_vehicle_info(world, ego_vehicle, frame_data, distance_threshold=75.0):
    """记录车辆和交通灯信息"""
    snapshot = world.get_snapshot()
    frame = snapshot.frame
    if frame % 100 == 0:  # 每100帧显示一次
        print(f"Recorded {len(frame_data)} frames, current frame: {frame}")
        # 定期保存数据
        save_data(force_sync=False)
        
    timestamp = snapshot.timestamp.elapsed_seconds

    # Ego Vehicle 的位置和朝向
    ego_transform = ego_vehicle.get_transform()
    ego_location = ego_transform.location
    ego_rotation = ego_transform.rotation

    # 获取 Ego Vehicle 的控制状态
    ego_control = ego_vehicle.get_control()

    # 获取 Ego Vehicle 的速度、角速度和加速度
    ego_velocity = ego_vehicle.get_velocity()
    ego_angular_velocity = ego_vehicle.get_angular_velocity()
    ego_acceleration = ego_vehicle.get_acceleration()

    def vector_magnitude(vector):
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


    ego_vehicle_data = {
        'id': ego_vehicle.id,
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

    for actor in world.get_actors().filter('vehicle.*'):
        if actor.id != ego_vehicle.id:
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

    traffic_lights = get_nearby_traffic_lights(world, ego_location, radius=distance_threshold)

    frame_data.append({
        'frame': frame,
        'timestamp': timestamp,
        'ego_vehicle': ego_vehicle_data,
        'nearby_vehicle_count': nearby_vehicle_count,
        'nearby_vehicles': nearby_vehicles,
        'traffic_lights': traffic_lights
    })
    
    return frame_data

def main():
    global client, args, frame_data
    
    # 注册清理函数
    atexit.register(cleanup)
    
    # 注册信号处理
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-o', '--output',
        metavar='O',
        default='vehicle_states.json',
        help='output file path for vehicle states (default: vehicle_states.json)')
    argparser.add_argument(
        '--carla-record',
        help='Path for Carla recorder file (default: carla_recorder.log)')

    args = argparser.parse_args()
    print("******************************output dir is:", args.output)
    # 确保输出目录存在并可写
    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)
    
    if not os.access(output_dir, os.W_OK):
        logger.error(f"Output directory {output_dir} is not writable")
        sys.exit(1)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    frame_data = []

    try:
        world = client.get_world()

        if args.carla_record:
            client.start_recorder(args.carla_record, True)
            logger.info(f"Started Carla recording: {args.carla_record}")

        ego_vehicle = None
        while not ego_vehicle:
            print("searching for ego vehicle...")
            vehicles = world.get_actors().filter('vehicle.*')
            for vehicle in vehicles:
                if vehicle.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                    ego_vehicle = vehicle
                    print(f"Ego Vehicle found: {ego_vehicle.id}")
                    break
            time.sleep(0.1)  # 防止CPU过度使用

        while True:
            world.wait_for_tick()
            frame_data = record_vehicle_info(world, ego_vehicle, frame_data)
            ego_vehicle_velocity = ego_vehicle.get_velocity()
            print(f"Ego Vehicle Speed: {ego_vehicle_velocity.x:.2f}, {ego_vehicle_velocity.y:.2f}, {ego_vehicle_velocity.z:.2f}")

    except Exception as e:
        logger.error(f"Error in main loop: {e}")
        traceback.print_exc()
        raise
    finally:
        cleanup()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        traceback.print_exc()
    finally:
        
        print('\ndone.')