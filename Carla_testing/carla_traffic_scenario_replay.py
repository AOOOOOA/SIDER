#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""


import glob
import os
import sys
import time
import math
import json

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

def vector_magnitude(vector):
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '--follow-id',
        metavar='F',
        type=int,
        default=None,
        help='Specify the original vehicle ID to follow (default: random)')
    argparser.add_argument(
            '--replay-path',
            type=str,
            required=True,
            help='Path to the recorded data file for replay'
        )
    argparser.add_argument(
        '--output-path',
        type=str,
        required=True,
        help='Path to save replay metrics'
    )
    
    
    # 添加town参数
    argparser.add_argument(
        '--town',
        type=str,
        default='Town01',
        help='Town to load (Town01-Town07, Town10)'
    )
    
    args = argparser.parse_args()

  
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False

    try:
        world = client.get_world()
  
        if args.town:
            print(f"Loading town: {args.town}")
            client.load_world(args.town)
            world = client.get_world()
            print(f"Successfully loaded world: {args.town}")


        settings = world.get_settings()
        if args.no_rendering:
            settings.no_rendering_mode = True
        
        synchronous_master = True
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        world.apply_settings(settings)

      
        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        replay_vehicles = {}  # 存储重放的车辆对象

        replay_frame_index = 0  # 用于追踪当前帧
        
        # 读取原始数据
        with open(args.replay_path, 'r') as f:
            ori_data = json.load(f)
        
        
        # 获取所有帧号并排序
        frame_numbers = sorted(map(int, ori_data.keys()))
        # print("the sorted frame numbers:", frame_numbers)
        
        current_frame = str(frame_numbers[replay_frame_index])
        
        # print("replay_frame_index is", replay_frame_index)
        # print("current_frame is", current_frame )
        # if replay_frame_index == 0:
        last_frame = str(frame_numbers[replay_frame_index])
        # else:
            # last_frame = str(frame_numbers[replay_frame_index-1])
        # print("last frame is", last_frame)
        # next_frame = str(frame_numbers[replay_frame_index])
        
        
        # 获取第一帧数据用于初始化
        current_frame_data = ori_data[current_frame]
        last_frame_data = ori_data[last_frame]
        
        # 为每个记录的车辆创建新车辆
        for vehicle_data in current_frame_data["vehicles"]:
            original_id = vehicle_data["id"]
            blueprint = world.get_blueprint_library().filter(vehicle_data["type"])[0]
            
            # 使用第一帧数据中的位置和旋转信息
            spawn_point = carla.Transform(
                carla.Location(
                    x=vehicle_data["x"],
                    y=vehicle_data["y"],
                    z=vehicle_data["z"]
                ),
                carla.Rotation(
                    roll=vehicle_data["roll"],
                    pitch=vehicle_data["pitch"],
                    yaw=vehicle_data["yaw"]
                )
            )
            
            # 生成车辆sub
            vehicle = world.spawn_actor(blueprint, spawn_point)
            replay_vehicles[original_id] = vehicle
            print(f"Spawned vehicle with ID: {vehicle.id} for original ID: {original_id}")
            vehicle.set_simulate_physics(True)
        # 等待所有车辆完全生成
        world.tick()
        
        # 设置spectator
        spectator = world.get_spectator()
        
        # 根据指定的ID或随机选择一辆车作为spectator跟随目标
        if args.follow_id is not None:
            # 尝试找到指定ID的车辆
            if args.follow_id in replay_vehicles:
                spectator_vehicle = replay_vehicles[args.follow_id]
                print(f"Following specified vehicle with original ID: {args.follow_id}, replay ID: {spectator_vehicle.id}")
            else:
                print(f"Warning: Specified vehicle ID {args.follow_id} not found, choosing random vehicle instead")
                spectator_vehicle = random.choice(list(replay_vehicles.values()))
                print(f"Following random vehicle with replay ID: {spectator_vehicle.id}")
        else:
            # 随机选择一辆车
            spectator_vehicle = random.choice(list(replay_vehicles.values()))
            
            # spectator_vehicle = list(replay_vehicles.values())[4]
            
            print(f"Following random vehicle with replay ID: {spectator_vehicle.id}")
        
        # 检查所有车辆是否成功生成并到达指定位置
        all_vehicles_ready = False
        while not all_vehicles_ready:
            all_vehicles_ready = True
            for original_id, vehicle in replay_vehicles.items():
                loc = vehicle.get_transform()
                # 获取当前车辆对应的数据
                vehicle_data = next(v for v in current_frame_data["vehicles"] if v["id"] == original_id)
                if loc.location.x ==0 or loc.location.y==0:
                    all_vehicles_ready = False
                    
                    break
            if not all_vehicles_ready:
                world.tick()
                print("Waiting for vehicles to be ready...")
        
        print("All vehicles are ready, applying initial controls...")
        
        # 所有车辆就绪后，设置初始状态
        for vehicle_data in current_frame_data["vehicles"]:
            original_id = vehicle_data["id"]
            vehicle = replay_vehicles[original_id]
            
            # 设置初始速度
            velocity = carla.Vector3D(
                vehicle_data["velocity"]["x"],
                vehicle_data["velocity"]["y"],
                vehicle_data["velocity"]["z"]
            )
            
            angular_velocity = carla.Vector3D(
                vehicle_data["angular_velocity"]["x"],
                vehicle_data["angular_velocity"]["y"],
                vehicle_data["angular_velocity"]["z"]
            )
            spawn_point = carla.Transform(
                carla.Location(
                    x=vehicle_data["x"],
                    y=vehicle_data["y"],
                    z=vehicle_data["z"]
                ),
                carla.Rotation(
                    roll=vehicle_data["roll"],
                    pitch=vehicle_data["pitch"],
                    yaw=vehicle_data["yaw"]
                )
            )
            vehicle.set_target_velocity(velocity)
            vehicle.set_target_angular_velocity(angular_velocity)
            vehicle.set_transform(spawn_point)
            
            
            
            
            
            """
            vehicle_data=next(v for v in current_frame_data["vehicles"] if v["id"] == original_id)
            
             # 使用下一帧的控制数据
            control = carla.VehicleControl(
                throttle=vehicle_data["throttle"],
                steer=vehicle_data["steer"],
                brake=vehicle_data["brake"],
                hand_brake=vehicle_data["hand_brake"],
                reverse=vehicle_data["reverse"],
                manual_gear_shift=vehicle_data["manual_gear_shift"],
                gear=vehicle_data["gear"]
            )
            vehicle.apply_control(control)
            
            """
            # 找到该车辆在下一帧中的数据
            last_vehicle_data = next(v for v in last_frame_data["vehicles"] if v["id"] == original_id)
            
            # 使用上一帧的控制数据
            control = carla.VehicleControl(
                throttle=last_vehicle_data["throttle"],
                steer=last_vehicle_data["steer"],
                brake=last_vehicle_data["brake"],
                hand_brake=last_vehicle_data["hand_brake"],
                reverse=last_vehicle_data["reverse"],
                manual_gear_shift=last_vehicle_data["manual_gear_shift"],
                gear=last_vehicle_data["gear"]
            )
            vehicle.apply_control(control)

        # 等待控制生效
        world.tick()

        
        
        
        frame_data = []  # 用于存储重放数据
        
          # 记录重放数据
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        angular_velocity = vehicle.get_angular_velocity()
        acceleration = vehicle.get_acceleration()
        control = vehicle.get_control()
            
        snapshot = world.get_snapshot()
        frame = snapshot.frame
        timestamp = snapshot.timestamp.elapsed_seconds
        delta_time = snapshot.timestamp.delta_seconds
        platform_timestamp = snapshot.timestamp.platform_timestamp
        
        
        last_platform_timestamp=None
        
        #calculate the platform delta time
        platform_delta_time = 0.0
        if last_platform_timestamp is not None:
            platform_delta_time = platform_timestamp - last_platform_timestamp
        last_platform_timestamp = platform_timestamp
        
        
        replay_data = {
            "ori_frame_id": int(current_frame),  # 添加原始帧ID
            "frame": frame,
            "timestamp": timestamp,
            "delta_time": delta_time,
            "platform_timestamp": platform_timestamp,
            "platform_delta_time": platform_delta_time,
            "original_id": original_id,
            "replay_id": vehicle.id,
            "x": transform.location.x,
            "y": transform.location.y,
            "z": transform.location.z,
            "yaw": transform.rotation.yaw,
            "roll": transform.rotation.roll,
            "pitch": transform.rotation.pitch,
            "throttle": control.throttle,
            "steer": control.steer,
            "brake": control.brake,
            "gear": control.gear,
            "hand_brake": control.hand_brake,
            "reverse": control.reverse,
            "manual_gear_shift": control.manual_gear_shift,
            "velocity": {
                "x": velocity.x,
                "y": velocity.y,
                "z": velocity.z,
                "magnitude": vector_magnitude(velocity)
            },
            "angular_velocity": {
                "x": angular_velocity.x,
                "y": angular_velocity.y,
                "z": angular_velocity.z,
                "magnitude": vector_magnitude(angular_velocity)
            },
            "acceleration": {
                "x": acceleration.x,
                "y": acceleration.y,
                "z": acceleration.z,
                "magnitude": vector_magnitude(acceleration)
            }
        }
        frame_data.append(replay_data)

        
        # 增加帧索引，准备进入主循环
        replay_frame_index += 1
        
        # 在主循环开始时添加
        print(f"\nTotal vehicles being replayed: {len(replay_vehicles)}")

        while True:
            if not args.asynch and synchronous_master:
                # print("run in sync")
                # 更新spectator的位置，跟随选定的车辆
                if spectator_vehicle and spectator_vehicle.is_alive:
                    transform = spectator_vehicle.get_transform()
                    # 设置spectator位置在车辆后上方
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=30),carla.Rotation(pitch=-90)))
                    

                
                current_frame = str(frame_numbers[replay_frame_index])
                current_frame_data = ori_data[current_frame]
                
                # 记录当前帧的数据
                snapshot = world.get_snapshot()
                frame = snapshot.frame
                timestamp = snapshot.timestamp.elapsed_seconds
                
                platform_timestamp = snapshot.timestamp.platform_timestamp
                
                
                
                #calculate the platform delta time
                platform_delta_time = 0.0
                if last_platform_timestamp is not None:
                    platform_delta_time = platform_timestamp - last_platform_timestamp
                last_platform_timestamp = platform_timestamp
                
                #try to show the frame id on the carla 
                
                debug = world.debug
                text_location = transform.location + carla.Location(z=4)  # 使用车辆位置作为参考
                debug.draw_string(
                    text_location,  # 跟随车辆的位置
                    f'Original Frame: {current_frame} | Replay Frame: {frame}',
                    draw_shadow=True,
                    color=carla.Color(r=255, g=0, b=0, a=255),  # 完整的颜色定义
                    life_time=0.005,  # 稍微短一点的持续时间
                    persistent_lines=False
                )
                # print("after draw string")
                
                # 先更新车辆控制
                if replay_frame_index < len(frame_numbers) + 1:
                    next_frame = str(frame_numbers[replay_frame_index-1])
                    next_frame_data = ori_data[next_frame]
                    
                    for original_id, vehicle in replay_vehicles.items():
                        if vehicle.is_alive:
                            next_vehicle_data = next(v for v in next_frame_data["vehicles"] if v["id"] == original_id)
                            control = carla.VehicleControl(
                                throttle=next_vehicle_data["throttle"],
                                steer=next_vehicle_data["steer"],
                                brake=next_vehicle_data["brake"],
                                hand_brake=next_vehicle_data["hand_brake"],
                                reverse=next_vehicle_data["reverse"],
                                manual_gear_shift=next_vehicle_data["manual_gear_shift"],
                                gear=next_vehicle_data["gear"]
                            )
                            vehicle.apply_control(control)
                            # 保存应用的控制数据，以便后续记录
                            vehicle.last_applied_control = {
                                "ori_frame_id": int(next_frame),
                                "control": control
                            }
                        else:
                            print(f"Cannot apply control to vehicle {original_id} - not alive!")
                
                # 执行world.tick()让控制生效

                
                # 然后记录所有车辆的数据
                for original_id, vehicle in replay_vehicles.items():
                    if vehicle.is_alive:
                        transform = vehicle.get_transform()
                        velocity = vehicle.get_velocity()
                        angular_velocity = vehicle.get_angular_velocity()
                        acceleration = vehicle.get_acceleration()
                        current_control = vehicle.get_control()
                        
                        replay_data = {
                            "ori_frame_id": int(current_frame),
                            "frame": frame,
                            "timestamp": timestamp,
                            "delta_time": snapshot.timestamp.delta_seconds,
                            "platform_timestamp": platform_timestamp,
                            "platform_delta_time": platform_delta_time,
                            "original_id": original_id,
                            "replay_id": vehicle.id,
                            "x": transform.location.x,
                            "y": transform.location.y,
                            "z": transform.location.z,
                            "yaw": transform.rotation.yaw,
                            "roll": transform.rotation.roll,
                            "pitch": transform.rotation.pitch,
                            "current_control": {
                                "throttle": current_control.throttle,
                                "steer": current_control.steer,
                                "brake": current_control.brake,
                                "gear": current_control.gear,
                                "hand_brake": current_control.hand_brake,
                                "reverse": current_control.reverse,
                                "manual_gear_shift": current_control.manual_gear_shift,
                            },
                            "applied_control": {
                                "ori_frame_id": vehicle.last_applied_control["ori_frame_id"],
                                "throttle": vehicle.last_applied_control["control"].throttle,
                                "steer": vehicle.last_applied_control["control"].steer,
                                "brake": vehicle.last_applied_control["control"].brake,
                                "gear": vehicle.last_applied_control["control"].gear,
                                "hand_brake": vehicle.last_applied_control["control"].hand_brake,
                                "reverse": vehicle.last_applied_control["control"].reverse,
                                "manual_gear_shift": vehicle.last_applied_control["control"].manual_gear_shift,
                            } if hasattr(vehicle, 'last_applied_control') else None,
                            "velocity": {
                                "x": velocity.x,
                                "y": velocity.y,
                                "z": velocity.z,
                                "magnitude": vector_magnitude(velocity)
                            },
                            "angular_velocity": {
                                "x": angular_velocity.x,
                                "y": angular_velocity.y,
                                "z": angular_velocity.z,
                                "magnitude": vector_magnitude(angular_velocity)
                            },
                            "acceleration": {
                                "x": acceleration.x,
                                "y": acceleration.y,
                                "z": acceleration.z,
                                "magnitude": vector_magnitude(acceleration)
                            }
                        }
                        frame_data.append(replay_data)
                    else:
                        print(f"Vehicle {original_id} is not alive!")
                
                replay_frame_index += 1
                world.tick()
                
                # 如果已经处理完所有帧，退出循环
                if replay_frame_index >= len(frame_numbers):
                    break
            else:
                world.wait_for_tick()
                
    finally:
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        
        # 销毁所有重放车辆
        for vehicle in replay_vehicles.values():
            if vehicle.is_alive:
                vehicle.destroy()
        
        # 在保存数据前添加调试信息
        print(f"准备保存数据，frame_data长度: {len(frame_data)}")
        print(f"frame_data的前两个元素: {frame_data[:2] if len(frame_data) >= 2 else frame_data}")

        # 尝试保存数据，并捕获特定异常
        try:
            with open(args.output_path, 'w') as f:
                json.dump(frame_data, f, indent=4)
            print(f"数据成功保存至 {args.output_path}")
        except Exception as e:
            print(f"保存数据时出错: {type(e).__name__}: {e}")
            # 检查frame_data中是否有不可序列化的对象
            if isinstance(e, TypeError):
                problem_elements = []
                for i, item in enumerate(frame_data):
                    try:
                        json.dumps(item)
                    except TypeError:
                        problem_elements.append(i)
                if problem_elements:
                    print(f"以下索引的元素无法序列化: {problem_elements}")
                    for idx in problem_elements[:3]:  # 只显示前三个问题
                        print(f"问题元素 {idx}: {frame_data[idx]}")
        
        time.sleep(0.5)
        
        # 销毁spectator
        # if spectator:
        #     spectator.destroy()
        #     print("spector destroyed !!!")

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')



