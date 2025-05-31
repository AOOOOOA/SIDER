#!/usr/bin/env python

import os
import sys
import carla
import json
import argparse
import time
import numpy as np
import math
from pathlib import Path
import logging
from datetime import datetime
from time import perf_counter
import os
import fcntl


def setup_logger(log_dir: Path, replay_index: str):
    """设置日志记录器，添加回放索引"""
    log_file = log_dir / f"replay_recorder_{replay_index}_{datetime.now():%Y%m%d_%H%M%S}.log"
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
    return logging.getLogger(__name__)


class ReplayRecorder:
    def __init__(self,args, output_path: str, log_dir: Path, replay_index: str ):
        self.output_path = output_path
        self.frame_data = []
        self.replay_index = replay_index
        self.logger = setup_logger(log_dir, replay_index)
        self.last_save_time = time.time()
        self.save_interval = 10
        
        # 添加FIFO相关属性
        self.pipe_path = "/tmp/replay_pipe"
        self.pipe_in = None
        self.pipe_file = None
        
        
        self.client=None
        self.args=args
        
    def setup_pipe(self):
        """设置并打开FIFO管道用于读取"""
        while not os.path.exists(self.pipe_path):
            self.logger.info(f"Waiting for pipe at {self.pipe_path}...")
            time.sleep(0.1)
        self.pipe_in = os.open(self.pipe_path, os.O_RDONLY)
        self.pipe_file = os.fdopen(self.pipe_in)
        # 设置非阻塞模式

        flags = fcntl.fcntl(self.pipe_file.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(self.pipe_file.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
        
        self.logger.info("FIFO pipe opened for reading")

    def read_status(self):
        """从FIFO读取状态"""
        try:
            line = self.pipe_file.readline().strip()
            if line:
                return json.loads(line)
            return None
        except (IOError, json.JSONDecodeError) as e:
            self.logger.debug(f"Error reading from pipe: {e}")
            return None

    def cleanup(self):
        """清理FIFO资源"""
        if self.pipe_file:
            self.pipe_file.close()
        if self.pipe_in:
            os.close(self.pipe_in)
        
        
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
                    if vehicle_data['velocity']['magnitude'] > 0:
                        self.logger.debug(
                            f"Other vehicle speed - ID: {actor.id}, "
                            f"Speed: x={velocity.x:.2f}, y={velocity.y:.2f}, z={velocity.z:.2f}"
                        )
                    
                    nearby_vehicles.append(vehicle_data)
        
        return nearby_vehicles
        
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
                
        return traffic_lights
    
    
    # def start_carla_recording(self, client):
    #     """启动Carla录制"""
    #     print("++++++++++++++++++++++start carla recording in the replay mode")
    #     if self.carla_record:
    #         self.client = client
            
    #         self.client.start_recorder(self.carla_record)
    #         self.logger.info(f"Started Carla recording: {self.carla_record}")
    #         print("type of carla record is:", type(self.carla_record))

    # def stop_carla_recording(self):
    #     """停止Carla录制"""
    #     if self.carla_record and self.client:
    #         self.client.stop_recorder()
    #         self.logger.info("Stopped Carla recording")
    def record_frame(self, world: carla.World, frame_info: dict):
        """记录一帧数据"""
        try:
            start_time = perf_counter()
            
            ego_vehicle = None
            for actor in world.get_actors():
                if actor.attributes.get('role_name') == 'ego_vehicle':
                    ego_vehicle = actor
                    break
                    
            if not ego_vehicle:
                self.logger.warning("No ego vehicle found")
                return
                
            frame_data = {
                # 同步相关信息
                'frame': frame_info['carla_frame'],
                'replay_frame_index': frame_info['replay_frame_index'],
                'original_frame': frame_info['original_frame'],
                'replay_count': frame_info['replay_count'],
                'timestamp': frame_info['timestamp'],
                    
    
                # 使用get方法安全地获取ID映射关系
                'vehicle_id_mapping': frame_info.get('vehicle_id_mapping', {}),
                
                
                # 车辆数据
                'ego_vehicle': self._get_vehicle_data(ego_vehicle),
                'nearby_vehicles': self._get_nearby_vehicles(world, ego_vehicle),
                'traffic_lights': self._get_traffic_lights(world, ego_vehicle),
                
                # 性能指标
                'processing_time': perf_counter() - start_time
            }
            
            

            # 更新车辆计数
            frame_data['nearby_vehicle_count'] = len(frame_data['nearby_vehicles'])
            
            # 记录ego车辆信息
            ego_velocity = ego_vehicle.get_velocity()
            ego_transform = ego_vehicle.get_transform()
            self.logger.debug(
                f"[EGO] Speed: {ego_velocity.x:.2f}, {ego_velocity.y:.2f}, {ego_velocity.z:.2f} | "
                f"Position: {ego_transform.location.x:.2f}, {ego_transform.location.y:.2f} | "
                f"Yaw: {ego_transform.rotation.yaw:.2f}"
            )
            
            self.frame_data.append(frame_data)
            
            # 定期保存数据
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_interval:
                self.logger.info("Performing periodic save...")
                self.save_data()
                self.last_save_time = current_time
                self.logger.info("Periodic save completed")
                
            # 定期输出状态
            if len(self.frame_data) % 100 == 0:
                self.logger.info(
                    f"Frame {frame_data['frame']} - "
                    f"Ego Vehicle ID: {ego_vehicle.id}, "
                    f"Nearby Vehicles: {frame_data['nearby_vehicle_count']}, "
                    f"Processing Time: {frame_data['processing_time']*1000:.2f}ms"
                )
                
        except Exception as e:
            self.logger.error(f"Error recording frame: {e}", exc_info=True)
            
    def save_data(self):
        """保存记录的数据"""

        # if self.args and self.args.carla_record and self.client:
        #     self.client.stop_recorder()
        #     self.logger.info("Stopped Carla recording")
        
        
        # print("-------- call save data for the json data(at the beginning of the save_data function)")
        if not self.frame_data:
            # print("inside the save json data function, there is not frame data")
            return
        try:
            # print("try to save the json data")
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            with open(self.output_path, 'w') as f:
                json.dump(self.frame_data, f, indent=2)
            self.logger.info(f"Saved {len(self.frame_data)} frames to {self.output_path}")
        except Exception as e:
            self.logger.error(f"Error saving data: {e}", exc_info=True)

def main():
    parser = argparse.ArgumentParser(description='Record replay data from CARLA')
    parser.add_argument('--output', type=str, required=True, help='Output file path')
    parser.add_argument('--log-dir', type=str, required=True, help='Log directory')
    parser.add_argument('--replay-index', type=str, required=True, help='Replay segment index')
    # parser.add_argument('--carla-record', type=str, help='Path for Carla recorder file (default: carla_recorder.log)')
    args = parser.parse_args()
    
    log_dir = Path(args.log_dir)
    log_dir.mkdir(exist_ok=True)
    
    try:
        
        # 连接到CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        recorder = ReplayRecorder(args,args.output, log_dir,args.replay_index)
        # recorder.start_carla_recording(client)
        recorder.setup_pipe()
        
        last_frame = None
        sync_errors_ahead = 0
        sync_errors_behind = 0
        max_sync_errors = 1000
        
        recorder.logger.info("Started recording replay data")
        print("before enter the while true loop")
        while True:
            try:
                snapshot = world.get_snapshot()
                current_carla_frame = snapshot.frame
                print("====================s", current_carla_frame)
            
                status = recorder.read_status()
                if status is None:
                    time.sleep(0.001)
                    continue
               
                replay_index = status['replay_index']
                original_frame = status['original_frame']
                main_carla_frame = status['carla_frame']
                replay_count = status['replay_count']
                
                
                # 检查同步状态
                if current_carla_frame != main_carla_frame:
                    if current_carla_frame > main_carla_frame:
                        sync_errors_ahead += 1
                        if sync_errors_ahead % 100 == 0:
                            recorder.logger.warning(
                                f"Record ahead - Current: {current_carla_frame}, "
                                f"Main: {main_carla_frame}, Diff: {current_carla_frame - main_carla_frame}, "
                                f"Errors: {sync_errors_ahead}"
                            )
                        time.sleep(0.002)
                    else:
                        sync_errors_behind += 1
                        if sync_errors_behind % 100 == 0:
                            recorder.logger.warning(
                                f"Record behind - Current: {current_carla_frame}, "
                                f"Main: {main_carla_frame}, Diff: {main_carla_frame - current_carla_frame}, "
                                f"Errors: {sync_errors_behind}"
                            )
                        time.sleep(0.001)
                    
                    total_errors = sync_errors_ahead + sync_errors_behind
                    if total_errors >= max_sync_errors:
                        recorder.logger.error(
                            f"Max sync errors reached ({max_sync_errors}). "
                            f"Ahead: {sync_errors_ahead}, Behind: {sync_errors_behind}"
                        )
                        break
                    continue
                
                # 同步恢复
                if sync_errors_ahead > 0 or sync_errors_behind > 0:
                    recorder.logger.info(
                        f"Sync restored. Previous errors - "
                        f"Ahead: {sync_errors_ahead}, Behind: {sync_errors_behind}"
                    )
                    sync_errors_ahead = 0
                    sync_errors_behind = 0
                
                # 避免重复记录
                if replay_count == last_frame:
                    time.sleep(0.001)
                    continue
                
                # 记录数据
                frame_info = {
                    'carla_frame': current_carla_frame,
                    'replay_frame_index': replay_index,
                    'original_frame': original_frame,
                    'replay_count': replay_count,
                    'vehicle_id_mapping': status.get('vehicle_id_mapping', {}),
                    'timestamp': time.time()
                }
                
                recorder.record_frame(world, frame_info)
                last_frame = replay_count
                
                if replay_count % 100 == 0:
                    recorder.logger.info(
                        f"Recording frame {replay_count}: "
                        f"replay_index={replay_index}, "
                        f"original={original_frame}, "
                        f"carla={current_carla_frame}"
                    )
                    
            except Exception as e:
                recorder.logger.error(f"Error in main loop: {e}", exc_info=True)
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        recorder.logger.info("Recording stopped by user")
    except Exception as e:
        recorder.logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        if recorder:
            recorder.logger.info("Entering finally block, attempting to save final data...")
            try:
                recorder.save_data()
                recorder.logger.info("Final data save completed")
            except Exception as e:
                recorder.logger.error(f"Error during final save: {e}", exc_info=True)
            
            try:
                recorder.cleanup()
                recorder.logger.info("Cleanup completed")
            except Exception as e:
                recorder.logger.error(f"Error during cleanup: {e}", exc_info=True)

if __name__ == '__main__':
    main()