import json
import numpy as np
from skopt.utils import use_named_args
from skopt.learning import GaussianProcessRegressor
from skopt.learning.gaussian_process.kernels import Matern
import subprocess
import time
import os
from typing import Dict, Any, Tuple, List
import pandas as pd 
import sys
import carla
import random
import traceback
import math
import logging
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from datetime import datetime
import socket  # Added socket import
import signal as process_signal  # Renamed process management signal
from scipy import signal as scipy_signal  # Renamed scipy's signal
import pickle

from sklearn.metrics import silhouette_score
from scipy.stats import spearmanr
from sklearn.ensemble import RandomForestRegressor
from sklearn.tree import DecisionTreeRegressor
from scipy.stats import spearmanr
import numpy as np
import os
import json
import time
from sklearn.preprocessing import StandardScaler



# Add resource monitoring module
import psutil
import threading
import datetime

# Define resource monitoring function
def get_system_resources():
    """Get system resource usage"""
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()
    
    return {
        "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
        "memory_rss_mb": memory_info.rss / (1024 * 1024),  # RSS memory (MB)
        "memory_vms_mb": memory_info.vms / (1024 * 1024),  # Virtual memory (MB)
        "memory_percent": process.memory_percent(),         # Memory usage percentage
        "cpu_percent": process.cpu_percent(interval=0.1),   # CPU usage rate
        "threads_count": process.num_threads(),             # Number of threads
        "system_memory": {
            "total_gb": psutil.virtual_memory().total / (1024**3),
            "available_gb": psutil.virtual_memory().available / (1024**3),
            "percent": psutil.virtual_memory().percent
        },
        "system_cpu": psutil.cpu_percent(interval=0.1)      # System CPU usage rate
    }

# Define resource monitor class
class ResourceMonitor:
    def __init__(self, interval=5, log_file="system_resources.log"):
        self.interval = interval
        self.log_file = log_file
        self.running = False
        self.thread = None
        self.resources = []
        
    def log_resources(self):
        """Record resource information"""
        while self.running:
            resources = get_system_resources()
            self.resources.append(resources)
            
            # Record to log file
            with open(self.log_file, "a") as f:
                f.write(json.dumps(resources) + "\n")
                
            # Print to console
            print(f"System resource monitoring: Memory: {resources['memory_rss_mb']:.2f}MB ({resources['memory_percent']:.1f}%), "
                  f"CPU: {resources['cpu_percent']:.1f}%, System memory usage: {resources['system_memory']['percent']}%")
                
            time.sleep(self.interval)
    
    def start(self):
        """Start resource monitoring"""
        self.running = True
        self.thread = threading.Thread(target=self.log_resources)
        self.thread.daemon = True  # Set as daemon thread, will exit when main thread exits
        self.thread.start()
        print(f"System resource monitoring started, log file: {self.log_file}")
        
    def stop(self):
        """Stop resource monitoring"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        print(f"System resource monitoring stopped, {len(self.resources)} resource information recorded")
        
        # Save resource usage summary
        if self.resources:
            summary_file = f"resource_summary_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(summary_file, "w") as f:
                json.dump({
                    "start_time": self.resources[0]["timestamp"],
                    "end_time": self.resources[-1]["timestamp"],
                    "peak_memory_mb": max(r["memory_rss_mb"] for r in self.resources),
                    "avg_memory_mb": sum(r["memory_rss_mb"] for r in self.resources) / len(self.resources),
                    "peak_cpu_percent": max(r["cpu_percent"] for r in self.resources),
                    "avg_cpu_percent": sum(r["cpu_percent"] for r in self.resources) / len(self.resources),
                    "full_data": self.resources
                }, f)
            print(f"Resource usage summary saved to {summary_file}")
            
    def log_snapshot(self, tag=""):
        """Record current resource snapshot"""
        resources = get_system_resources()
        resources["tag"] = tag
        self.resources.append(resources)
        
        # Record to log file
        with open(self.log_file, "a") as f:
            f.write(json.dumps(resources) + "\n")
            
        # Print to console
        print(f"Resource snapshot ({tag}): Memory: {resources['memory_rss_mb']:.2f}MB ({resources['memory_percent']:.1f}%), "
              f"CPU: {resources['cpu_percent']:.1f}%, System memory usage: {resources['system_memory']['percent']}%")
        
        return resources

 


def standardize(values):
    # Z-score standardization for numerical values
    mean = np.mean(values)
    std = np.std(values)
    if std == 0:  # Avoid division by zero
        return np.zeros_like(values)
    return (values - mean) / std


class CarlaExperimentManager:
    def __init__(self, carla_path="/home/w/workspace/carla_apollo/CARLA_0.9.14/CarlaUE4.sh", windowed=True):
        # Initialize resource monitor
        self.resource_monitor = ResourceMonitor(interval=10, 
                                               log_file=f"bo_resources_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        
        # Set up logging
        self.setup_logging()
        
        # Log initial resource status
        self.resource_monitor.log_snapshot("Initialization started")
       
        # Define encoding mappings for categorical variables
        self.weather_mapping = {
            'ClearNoon': 0, 'CloudyNoon': 1, 'WetNoon': 2, 'WetCloudyNoon': 3,
            'MidRainyNoon': 4, 'HardRainNoon': 5, 'SoftRainNoon': 6,
            'ClearSunset': 7, 'CloudySunset': 8, 'WetSunset': 9,
            'WetCloudySunset': 10, 'MidRainSunset': 11, 'HardRainSunset': 12,
            'SoftRainSunset': 13
        }
        self.min_samples = 60
        # Added around line 173 (after self.min_samples = 100)
        self.reverse_weather_mapping = {v: k for k, v in self.weather_mapping.items()}
        if hasattr(self, 'vehicle_type_mapping'):
            self.reverse_vehicle_mapping = {v: k for k, v in self.vehicle_type_mapping.items()}
        if hasattr(self, 'sensor_type_mapping'):
            self.reverse_sensor_mapping = {v: k for k, v in self.sensor_type_mapping.items()}
        if hasattr(self, 'town_mapping'):
            self.reverse_town_mapping = {v: k for k, v in self.town_mapping.items()}
        if hasattr(self, 'static_object_mapping'):
            self.reverse_object_mapping = {v: k for k, v in self.static_object_mapping.items()}
        self.save_dir = "optimization_data"
        
        self.vehicle_type_mapping = {
            "vehicle.lincoln.mkz_2017": 0,
             "vehicle.dodge.charger_2020":1,
             "vehicle.dodge.charger_police_2020":2,
             "vehicle.ford.crown":3,
             "vehicle.ford.mustang":4,
             "vehicle.jeep.wrangler_rubicon":5,
             "vehicle.lincoln.mkz_2020":6,
             "vehicle.mercedes.coupe":7,
             "vehicle.mercedes.coupe_2020":8,
             "vehicle.micro.microlino":9,
             "vehicle.nissan.patrol":10,
             "vehicle.nissan.patrol_2021":11,
             "vehicle.seat.leon":12,
             "vehicle.toyota.prius":13,
             
             
            # 'vehicle.carlamotors.firetruck': 1,
            # 'vehicle.harley-davidson.low_rider': 1
            # "vehicle.lincoln.mkz_2017": 0,
            
        }
        
        
        self.sensor_type_mapping = {
            'sensor.camera.rgb': 0,
            'sensor.camera.depth': 1,
            'sensor.camera.semantic_segmentation': 2,
            'sensor.lidar.ray_cast': 3,
            'sensor.lidar.ray_cast_semantic': 4
        }
         
         
        
        
        self.town_mapping = { 
            'Town01': 0,
            'Town02': 1,
            'Town03': 2,
            'Town04': 3,
            'Town05': 4
        }
        
        
        self.static_object_mapping = { 
            'static.prop.barrel': 0,
            ' static.prop.trashbag':1,
            'static.prop.gardenlamp':2,
            'static.prop.plasticbag':3,
            'static.prop.constructioncone':4,
            ' static.prop.streetsign':5,
            'static.prop.calibrator':6
            }
        
                
        # Add sensor parameter mapping
        self.sensor_tick_mapping = {
            'sensor.camera.rgb': {'tick_range': [0.0, 1.0]},
            'sensor.camera.depth': {'tick_range': [0.0, 1.0]},
            'sensor.camera.semantic_segmentation': {'tick_range': [0.0, 1.0]},
            'sensor.lidar.ray_cast': {
                'tick_range': [0.0, 1.0],
                'channels_range': [32, 128],
                'points_range': [100000, 500000],
                'rotation_range': [10, 20]
            },
            'sensor.lidar.ray_cast_semantic': {
                'tick_range': [0.0, 1.0],
                'channels_range': [32, 128],
                'points_range': [100000, 500000],
                'rotation_range': [10, 20]
            }
}
        
        
        # Modify space definition
        self.space = [
            Integer(0, 13, name='weather_preset'),  # 映射到天气预设
            Real(0, 100, name='rain_intensity'),
            Real(0, 100, name='fog_density'),
            Real(0, 100, name='wind_intensity'),
            Real(0, 90, name='sun_altitude_angle'),
            Real(0, 100, name='humidity'),
            Integer(1, 100, name='num_vehicles'),
            Integer(1, 100, name='num_walkers'),
            Real(-30, 30, name='global_speed_difference'),
            Real(-2.0, 2.0, name='global_lane_offset'),
            Real(0, 100, name='vehicles_running_red_light_ratio'), 
            Real(0, 100, name='red_light_running_chance'),
            Real(0, 100, name='vehicles_changing_lane_ratio'),
            Real(0, 100, name='lane_changing_chance'),
            Integer(0, 13, name='vehicle_type'),  # 映射到车辆类型
            Integer(0, 4, name='sensor_type'),   # 映射到传感器类型
            Integer(1, 20, name='sensor_number'),
            
            Integer(0, 4, name='town'),
            Integer(0, 6, name='static_object_type'),
            Integer(0, 20, name='static_object_number'),

            # 添加传感器参数比例相关的搜索空间
            Real(0.0, 1.0, name='sensor_tick_ratio'),  # 需要修改tick的传感器比例
            Real(0.05, 0.5, name='sensor_tick_value'),  # tick值
            
            # 对于LiDAR特有的参数
            Real(0.0, 1.0, name='lidar_param_ratio'),  # 需要修改参数的LiDAR比例
            Integer(32, 128, name='lidar_channels'),
            Integer(100000, 500000, name='lidar_points'),
            Real(10.0, 20.0, name='lidar_rotation'),
           
            
        ]
        
        
        self.logger.info("Initialized search space with dimensions:")
        for dim in self.space:
            self.logger.info(f"  {dim.name}: {type(dim).__name__}")
        

        self.gp = GaussianProcessRegressor(
            kernel=Matern(length_scale=1.0, nu=2.5),
            normalize_y=True,
            random_state=42
        )
        
        self.error_log = []
        self.carla_path = carla_path
        self.windowed = windowed
        self.carla_process = None

        self.objective_stage1 = use_named_args(dimensions=self.space)(self._objective_stage1)
        
        # 添加实验目录
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.exp_dir = os.path.join('experiments', timestamp)
        # os.makedirs(self.exp_dir, exist_ok=True)
        
        # 初始化历史记录
        self.X_history = []
        self.y_history = []
        
        self.logger.info(f"创建实验目录: {self.exp_dir}")
        
        # 添加两阶段搜索相关的属性
        self.stage1_results = []
        self.stage2_results = []
        self.clusters = None
        self.cluster_centers = None
        
        # 添加评估指标
        self.cv_scores = []
        self.acquisition_values = []
        self.predicted_means = []
        self.predicted_stds = []
        
        # 添加参数名称列表
               
        self.param_names = [
            'weather_preset',
            'rain_intensity',
            'fog_density',
            'wind_intensity',
            'sun_altitude_angle',
            'humidity',
            'num_vehicles',
            'num_walkers',
            'global_speed_difference',
            'global_lane_offset',
            'vehicles_running_red_light_ratio',
            'red_light_running_chance',
            'vehicles_changing_lane_ratio',
            'lane_changing_chance',
            'vehicle_type',
            'sensor_type',
            'sensor_number',
            'town',
            'static_object_type',
            'static_object_number',
            'sensor_tick_ratio',
            'sensor_tick_value',
            'lidar_param_ratio',
            'lidar_channels',
            'lidar_points',
            'lidar_rotation'
        ]
        
        # 确保参数名称列表长度与搜索空间维度匹配
        print("=====",len(self.param_names), len(self.space))
        assert len(self.param_names) == len(self.space), "参数名称数量与搜索空间维度不匹配"

        # 确保参数名称列表与搜索空间维度匹配
        space_names = [dim.name for dim in self.space]
        assert self.param_names == space_names, (
            f"参数名称列表与搜索空间维度不匹配:\n"
            f"param_names: {self.param_names}\n"
            f"space_names: {space_names}"
        )

        self.detailed_errors = []  # 存储详细的误差数据
        self.current_scenario_config=None
    
    def _apply_sensor_params(self, params):
        """生成传感器配置"""
        if not params.get('sensor_type') or params.get('sensor_number', 0) <= 0:
            return None
            
        try:
            # 处理sensor_type
            sensor_type = params['sensor_type']
            if isinstance(sensor_type, str):
                if sensor_type in self.sensor_type_mapping:
                    # 如果是传感器类型字符串（如 'sensor.camera.depth'），直接使用
                    sensor_type_str = sensor_type
                else:
                    # 如果是数字字符串，转换为索引
                    sensor_type = int(sensor_type)
                    sensor_type_str = list(self.sensor_type_mapping.keys())[sensor_type]
            else:
                # 如果是数字，使用索引
                sensor_type = int(sensor_type)
                sensor_type_str = list(self.sensor_type_mapping.keys())[sensor_type]
                
            self.logger.info(f"处理传感器类型: {sensor_type} -> {sensor_type_str}")
            
            sensor_configs = []
            for i in range(int(params['sensor_number'])):
                config = {
                    'type': sensor_type_str, 
                    'id': i
                }
                
                # 设置tick
                if i < int(params['sensor_number'] * float(params['sensor_tick_ratio'])):
                    config['tick'] = float(params['sensor_tick_value'])
                
                # 设置LiDAR特有参数
                if 'lidar' in sensor_type_str.lower():
                    lidar_param_count = int(params['sensor_number'] * float(params['lidar_param_ratio']))
                    if i < lidar_param_count:
                        config.update({
                            'channels': int(params['lidar_channels']),
                            'points_per_second': int(params['lidar_points']),
                            'rotation_frequency': float(params['lidar_rotation'])
                        })
                
                sensor_configs.append(config)
            
            return sensor_configs
            
        except Exception as e:
            self.logger.error(f"生成传感器配置失败: {str(e)}")
            raise
        
    
    
    
    def setup_logging(self):

        self.logger = logging.getLogger('BayesianOptimization')
        self.logger.setLevel(logging.INFO)
        
        # 创建日志文件，使用时间戳
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        log_file = f'bo_optimization_{timestamp}.log'
        
        # 文件处理器
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.INFO)
        
        # 控制台处理器
        ch = logging.StreamHandler(sys.stdout)  # 明确指定输出到标准输出
        ch.setLevel(logging.INFO)
        
        # 设置格式
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        
        # 确保处理器不会重复添加
        if not self.logger.handlers:
            self.logger.addHandler(fh)
            self.logger.addHandler(ch)
        
        # 禁用日志传播到根logger
        self.logger.propagate = False
        
        self.logger.info("="*50)
        self.logger.info("Starting new Bayesian Optimization session")
        self.logger.info("="*50)

    def start_carla(self):
        
        try:
            print("正在启动CARLA服务器...")
            cmd = [self.carla_path] #, "-quality-level=Low"]
            
            # 如果指定了windowed模式，添加参数
            if self.windowed:
                # cmd.extend(["-windowed", "-ResX=800", "-ResY=600"])
                cmd.extend(["-RenderOffScreen"])
            self.carla_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(5)  # 等待CARLA启动
            print("CARLA服务器已启动")
            
            # 测试连接
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)
            world = client.get_world()
            print(f"成功连接到CARLA，当前地图: {world.get_map().name}")
            
        except Exception as e:
            print(f"启动CARLA失败: {e}")
            if self.carla_process:
                self.stop_carla()
            raise
        
    def stop_carla(self):
        
        try:
            # 首先使用pkill强制终止所有CARLA相关进程
            if os.name != 'nt':  # Linux/Mac系统
                try:
                    # 终止所有包含'carla'的进程
                    subprocess.run("pkill -9 -f carla", shell=True)
                    self.logger.info("已强制终止所有CARLA相关进程")
                    
                    # 终止UE4进程
                    subprocess.run("pkill -9 -f UE4", shell=True)
                    self.logger.info("已强制终止所有UE4相关进程")
                except Exception as e:
                    self.logger.error(f"pkill命令执行失败: {e}")
            
            # 检查并关闭特定端口
            ports_to_check = [2000, 8000]
            for port in ports_to_check:
                try:
                    if os.name != 'nt':  # Linux/Mac系统
                        cmd = f"lsof -ti:{port}"
                        pids = subprocess.check_output(cmd, shell=True).decode().strip().split('\n')
                        for pid in pids:
                            if pid:
                                os.kill(int(pid), process_signal.SIGTERM)
                                self.logger.info(f"已终止端口 {port} 上的进程 (PID: {pid})")
                    else:  # Windows系统
                        cmd = f"netstat -ano | findstr :{port}"
                        output = subprocess.check_output(cmd, shell=True).decode()
                        for line in output.split('\n'):
                            if line.strip():
                                pid = line.strip().split()[-1]
                                subprocess.run(f"taskkill /F /PID {pid}", shell=True)
                                self.logger.info(f"已终止端口 {port} 上的进程 (PID: {pid})")
                except subprocess.CalledProcessError:
                    self.logger.debug(f"端口 {port} 上没有运行的进程")
                except Exception as e:
                    self.logger.error(f"关闭端口 {port} 上的进程时出错: {e}")

            # 关闭CARLA服务器进程
            if hasattr(self, 'carla_process') and self.carla_process:
                self.carla_process.terminate()
                try:
                    self.carla_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.carla_process.kill()
                self.carla_process = None
                self.logger.info("CARLA服务器进程已停止")

            # 等待进程完全关闭
            time.sleep(2)  # 增加等待时间确保进程完全关闭
            
            # 最后验证端口状态
            self._verify_ports_closed(ports_to_check)
            
            # 额外验证CARLA进程是否还在运行
            self._verify_carla_stopped()

        except Exception as e:
            self.logger.error(f"停止CARLA时出错: {e}")
            self.logger.exception("详细错误信息：")
    import socket 
    def _verify_ports_closed(self, ports):
        
        for port in ports:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                result = sock.connect_ex(('127.0.0.1', port))
                sock.close()
                if result == 0:
                    self.logger.warning(f"端口 {port} 可能仍然被占用")
                    # 可以选择在这里再次尝试关闭进程
                else:
                    self.logger.debug(f"端口 {port} 已成功关闭")
            except Exception as e:
                self.logger.error(f"检查端口 {port} 状态时出错: {e}")
    def _verify_carla_stopped(self):
        """验证CARLA进程是否完全停止"""
        try:
            if os.name != 'nt':  # Linux/Mac系统
                # 检查是否还有CARLA相关进程
                ps_output = subprocess.check_output("ps aux | grep -i carla | grep -v grep", shell=True).decode()
                if ps_output.strip():
                    self.logger.warning("检测到CARLA进程仍在运行，尝试强制终止")
                    subprocess.run("pkill -9 -f carla", shell=True)
                    subprocess.run("pkill -9 -f UE4", shell=True)
                    time.sleep(1)
                    
                    # 再次检查
                    try:
                        subprocess.check_output("ps aux | grep -i carla | grep -v grep", shell=True)
                        self.logger.error("CARLA进程仍然存在，可能需要手动处理")
                    except subprocess.CalledProcessError:
                        self.logger.info("所有CARLA进程已成功终止")
        except subprocess.CalledProcessError:
            self.logger.info("未检测到运行中的CARLA进程")
        except Exception as e:
            self.logger.error(f"验证CARLA进程状态时出错: {e}")
    
    
    
    
    def create_experiment_folder(self, params: Dict[str, Any], run_label: str = "", base_timestamp: str = None) -> str:
        # 创建实验文件夹并保存参数
        # Args:
        #     params: 参数字典
        #     run_label: 运行标签（H1/L1）
        #     base_timestamp: 基准时间戳（用于L1使用H1的时间戳）
        
        # 转换params中的numpy类型为Python原生类型
        converted_params = {}
        for key, value in params.items():
            if isinstance(value, (np.int64, np.int32, np.int16, np.int8)):
                converted_params[key] = int(value)
            elif isinstance(value, (np.float64, np.float32)):
                converted_params[key] = float(value)
            else:
                converted_params[key] = value
        
        # 使用提供的时间戳或创建新的
        timestamp = base_timestamp if base_timestamp else time.strftime('%Y%m%d_%H%M%S')
        
        config_parts = [
            f"w{converted_params['weather_preset']}",
            f"r{int(converted_params['rain_intensity'])}",
            f"f{int(converted_params['fog_density'])}",
            f"v{int(converted_params['num_vehicles'])}",
            f"spd{int(converted_params['global_speed_difference'])}",
            f"off{converted_params['global_lane_offset']:.1f}",
            f"rlr{int(converted_params['vehicles_running_red_light_ratio'])}",
            f"rlc{int(converted_params['red_light_running_chance'])}",
            f"vcr{int(converted_params['vehicles_changing_lane_ratio'])}",
            f"lcc{int(converted_params['lane_changing_chance'])}",
            f"nw{int(converted_params['num_walkers'])}",
            
            # Town和静态物体相关
            f"t{converted_params['town']}",
            f"so{converted_params['static_object_type']}",
            f"sn{int(converted_params['static_object_number'])}",
            
            # 传感器相关
            f"st{converted_params['sensor_type']}",
            f"snum{int(converted_params['sensor_number'])}",
            f"str{converted_params['sensor_tick_ratio']:.2f}",
            f"stv{converted_params['sensor_tick_value']:.2f}",
            
            # LiDAR特有参数
            f"lpr{converted_params['lidar_param_ratio']:.2f}",
            f"lch{int(converted_params['lidar_channels'])}",
            f"lpt{int(converted_params['lidar_points'])}",
            f"lrt{converted_params['lidar_rotation']:.1f}"
            

        ]
        
        exp_dir = os.path.join(
            "experiments",
            f"{timestamp}_{'_'.join(config_parts)}"
        )
        os.makedirs(exp_dir, exist_ok=True)
        
        # 保存参数到JSON文件
        params_file = os.path.join(exp_dir, f"params_{run_label}.json")
        with open(params_file, 'w') as f:
            json.dump({
                'timestamp': timestamp,
                'parameters': converted_params,
                'run_label': run_label
            }, f, indent=4)
        
        return exp_dir
    
    
    
    
    def run_traffic_simulation(self, params: Dict[str, Any], is_high_load: bool, replay_data: str = None, run_label: str = "", iteration: int = 0) -> str:
        # 记录开始运行模拟前的资源状态
        self.resource_monitor.log_snapshot(f"模拟开始_迭代{iteration}_{run_label}")
        
        # 运行交通场景模拟
        if replay_data is not None:
            # 如果是L1，从H1的路径提取完整时间戳
            h1_dir = os.path.dirname(replay_data)
            base_timestamp = '_'.join(os.path.basename(h1_dir).split('_')[:2])  # 获取完整时间戳
            exp_dir = self.create_experiment_folder(params, run_label, base_timestamp)
        else:
            # H1创建新文件夹
            exp_dir = self.create_experiment_folder(params, run_label)
            
            # 在H1场景运行时，保存所有相关配置以供L1重放使用
            self.current_scenario_config = {
                # 基本场景参数
                "town": str(params.get("town", "Town01")),
                "weather_preset": str(params.get("weather_preset", "ClearNoon")),
                
                # 天气参数
                "rain_intensity": str(params.get("rain_intensity", 0.0)),
                "fog_density": str(params.get("fog_density", 0.0)),
                "wind_intensity": str(params.get("wind_intensity", 0.0)),
                "sun_altitude": str(params.get("sun_altitude_angle", 45.0)),
                "humidity": str(params.get("humidity", 50.0)),
                
                # 车辆和行人参数
                "num_vehicles": str(params.get("num_vehicles", 30)),
                "num_walkers": str(params.get("num_walkers", 10)),
                
                # 交通行为参数
                "speed_difference": str(params.get("global_speed_difference", 30.0)),
                "lane_offset": str(params.get("global_lane_offset", 0.0)),
                "red_light_running_chance": str(min(100.0, float(params.get("red_light_running_chance", 0.0)))),
                "vehicles_changing_lane_ratio": str(min(100.0, float(params.get("vehicles_changing_lane_ratio", 0.0)))),
                "lane_changing_chance": str(min(100.0, float(params.get("lane_changing_chance", 0.0)))),
                "ignore_lights_percent": str(min(100.0, float(params.get("vehicles_running_red_light_ratio", 0.0)))),
                
                # 车辆类型和传感器参数
                "vehicle_type": str(params.get("vehicle_type", "vehicle.lincoln.mkz_2017")),
                "sensor_type": str(params.get("sensor_type", "sensor.camera.rgb")),
                "sensor_number": str(params.get("sensor_number", 0)),
                
                # 静态物体参数
                "static_object_type": str(params.get("static_object_type", "static.prop.streetbarrier")),
                "static_object_number": str(params.get("static_object_number", 0)),
                
                # 记录实验目录
                "experiment_dir": exp_dir,
                
                # 记录时间戳
                "timestamp": time.strftime('%Y%m%d_%H%M%S')
            }
            
            self.logger.info(f"已保存场景配置用于后续重放: {json.dumps(self.current_scenario_config, indent=2)}")
        
        output_file = os.path.join(exp_dir, f"metrics_iter{iteration}_{run_label}.json")
        
        # 基础命令
        cmd = [
            "python",
            "carla_traffic_scenario_generator.py" if replay_data is None else "carla_traffic_scenario_replay.py",
            "--host", "localhost",
            "--port", "2000",
        ]
        
        if replay_data is None:
            print("********************enter the generator*******************************")
            # 录制模式参数
            if 'sensor_type' in params and 'sensor_number' in params and params['sensor_number'] > 0:
                self.logger.info(f"检测到传感器配置: 类型={params['sensor_type']}, 数量={params['sensor_number']}")
            
            # 生成传感器配置
            try:
                sensor_configs = self._apply_sensor_params(params)
                self.logger.info(f"生成传感器配置: {json.dumps(sensor_configs, indent=2)}")
                
                # # 转换vehicle_type
                # vehicle_idx = int(params.get('vehicle_type', 0))
                # vehicle_type = list(self.vehicle_type_mapping.keys())[vehicle_idx]
                
                
                print("==================================================")
                # print("---------------------------- vehicle type is:",vehicle_type)
                print("check the params:",params)
                cmd.extend([
                    "--weather-preset", str(params["weather_preset"]),
                    "--rain-intensity", str(params["rain_intensity"]),  # 修正: 从--rain改为--rain-intensity
                    "--fog-density", str(params["fog_density"]),  # 修正: 从--fog改为--fog-density
                    "--wind-intensity", str(params["wind_intensity"]),  # 修正: 从--wind改为--wind-intensity
                    "--sun-altitude", str(params["sun_altitude_angle"]),  # 修正: 从--sun改为--sun-altitude
                    "--humidity", str(params["humidity"]),
                    "-n", str(params["num_vehicles"]),
                    "--speed-difference", str(params["global_speed_difference"]),
                    "--lane-offset", str(params["global_lane_offset"]),
                    "--red-light-running-chance", str(min(100.0, float(params["red_light_running_chance"]))),  # 添加范围限制
                    "--vehicles-changing-lane-ratio", str(min(100.0, float(params["vehicles_changing_lane_ratio"]))),  # 添加范围限制
                    "--lane-changing-chance", str(min(100.0, float(params["lane_changing_chance"]))),  # 添加范围限制
                    "--ignore-lights-percent", str(min(100.0, float(params["vehicles_running_red_light_ratio"]))),  # 添加范围限制
                    "-w", str(params["num_walkers"]),
                    "--filterv",str(params["vehicle_type"]),  # 确保添加vehicle_type参数
                    "--sensor-type", str(params["sensor_type"]),
                    "--sensor-number", str(params["sensor_number"]),
                    "--record-path", os.path.join(exp_dir, f"recorded_data_iter{iteration}_{run_label}.json"),
                    
                    "--town", str(params["town"]),
                    "--static-object-type", str(params["static_object_type"]),
                    "--static-object-number", str(params["static_object_number"]),
                ])
                
                # 如果有传感器配置，验证后再添加到命令行
                if sensor_configs:
                    # 验证每个传感器配置是否完整
                    validated_configs = []
                    for config in sensor_configs:
                        if 'type' not in config or 'id' not in config:
                            self.logger.warning(f"跳过不完整的传感器配置: {config}")
                            continue
                            
                        # 确保LiDAR传感器有必要的参数
                        if 'lidar' in config['type'].lower():
                            config.setdefault('channels', 32)
                            config.setdefault('points_per_second', 100000)
                            config.setdefault('rotation_frequency', 10)
                        
                        # 确保所有传感器都有tick参数
                        config.setdefault('tick', 0.05)
                        
                        validated_configs.append(config)
                    
                    if validated_configs:
                        cmd.extend(["--sensor-configs", json.dumps(validated_configs)])
                    else:
                        self.logger.warning("没有有效的传感器配置")

            except Exception as e:
                self.logger.error(f"run traffic simulation function: 生成传感器配置失败: {e}")
                self.logger.error(traceback.format_exc())
                
        else:
            # 重放模式参数
            # 防御性编程：检查self.current_scenario_config是否为None
            # if self.current_scenario_config is None:
            #     self.logger.warning("场景配置未初始化，使用默认配置")
            #     # 使用params中的town或默认值
            #     town = params.get("town", "Town01") if isinstance(params.get("town"), str) else "Town01"
            #     self.current_scenario_config = {"town": town}
            #     self.logger.info(f"已设置默认场景配置: {self.current_scenario_config}")
                
            # 安全地获取town值
            town = self.current_scenario_config.get("town", "Town01")
            self.logger.info(f"使用地图: {town}")
            
            cmd.extend([
                "--replay-path", replay_data,
                "--output", output_file,
                "--town", town,
            ])
        
        print(f"\n执行迭代{iteration} {run_label}场景模拟...")
        print(f"命令: {' '.join(cmd)}")  # 添加命令打印，便于调试
        
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"{run_label}模拟运行失败: {e}")
            # 记录错误时的资源状态
            self.resource_monitor.log_snapshot(f"模拟失败_迭代{iteration}_{run_label}")
            raise
        
        # 记录模拟完成后的资源状态
        self.resource_monitor.log_snapshot(f"模拟完成_迭代{iteration}_{run_label}")
        return output_file,exp_dir







    ################################################################
    ################## Global Search ################################
    ################################################################

    def _objective_stage1(self, *args, **params):
        """目标函数"""
        try:
            # 记录目标函数评估开始时的资源状态
            self.resource_monitor.log_snapshot(f"目标函数开始_迭代{len(self.X_history)}")
            
            # 处理参数
            if args and len(args) > 0:
                x = args[0]
                self.logger.info(f"收到位置参数: {x}")
                params = dict(zip([dim.name for dim in self.space], x))
                self.logger.info(f"转换后的参数字典: {params}")
            
            # 确保所有必需的参数都存在
            required_params = {
                'weather_preset': 0,
                'rain_intensity': 0.0,
                'fog_density': 0.0,
                'wind_intensity': 0.0,
                'sun_altitude_angle': 0.0,
                'humidity': 0.0,
                'num_vehicles': 10,
                'num_walkers': 0,
                'global_speed_difference': 0.0,
                'global_lane_offset': 0.0,
                'vehicles_running_red_light_ratio': 0.0,
                'red_light_running_chance': 0.0,
                'vehicles_changing_lane_ratio': 0.0,
                'lane_changing_chance': 0.0,
                'vehicle_type': 0,
                'sensor_type': 0,
                'sensor_number': 1,
                'town': 0,
                'static_object_type': 0,
                'static_object_number': 0,
                'sensor_tick_ratio': 0.5,
                'sensor_tick_value': 0.5,
                'lidar_param_ratio': 0.5,
                'lidar_channels': 32,
                'lidar_points': 100000,
                'lidar_rotation': 10.0
            }
            
            
            # 使用默认值更新参数
            final_params = required_params.copy()
            final_params.update(params)
            
            # 保存原始数值形式的参数用于历史记录
            numerical_params = final_params.copy()
            
            # 转换分类变量为字符串格式，但保留原始数值版本
            try:
                # 车辆类型转换
                vehicle_type_reverse = {v: k for k, v in self.vehicle_type_mapping.items()}
                if 'vehicle_type' in final_params:
                    vehicle_type_val = int(final_params['vehicle_type'])
                    numerical_params['vehicle_type'] = vehicle_type_val  # 保存数值版本
                    if vehicle_type_val not in vehicle_type_reverse:
                        self.logger.warning(f"未知的vehicle_type值: {vehicle_type_val}，使用默认值")
                        final_params['vehicle_type'] = "vehicle.lincoln.mkz_2017"
                    else:
                        final_params['vehicle_type'] = vehicle_type_reverse[vehicle_type_val]
                    
                # 传感器类型转换
                sensor_type_reverse = {v: k for k, v in self.sensor_type_mapping.items()}
                if 'sensor_type' in final_params:
                    sensor_type_val = int(final_params['sensor_type'])
                    numerical_params['sensor_type'] = sensor_type_val  # 保存数值版本
                    if sensor_type_val not in sensor_type_reverse:
                        self.logger.warning(f"未知的sensor_type值: {sensor_type_val}，使用默认值")
                        final_params['sensor_type'] = "sensor.camera.rgb"
                    else:
                        final_params['sensor_type'] = sensor_type_reverse[sensor_type_val]
                    
                # 天气预设转换
                weather_reverse = {v: k for k, v in self.weather_mapping.items()}
                if 'weather_preset' in final_params:
                    weather_val = int(final_params['weather_preset'])
                    numerical_params['weather_preset'] = weather_val  # 保存数值版本
                    if weather_val not in weather_reverse:
                        self.logger.warning(f"未知的weather_preset值: {weather_val}，使用默认值")
                        final_params['weather_preset'] = "ClearNoon"
                    else:
                        final_params['weather_preset'] = weather_reverse[weather_val]
                
                
                 # Town转换
                town_reverse = {v: k for k, v in self.town_mapping.items()}
                if 'town' in final_params:
                    town_val = int(final_params['town'])
                    numerical_params['town'] = town_val
                    if town_val not in town_reverse:
                        self.logger.warning(f"未知的town值: {town_val}，使用默认值")
                        final_params['town'] = "Town01"
                    else:
                        final_params['town'] = town_reverse[town_val]
                
                # 静态物体类型转换
                static_object_reverse = {v: k for k, v in self.static_object_mapping.items()}
                if 'static_object_type' in final_params:
                    obj_type_val = int(final_params['static_object_type'])
                    numerical_params['static_object_type'] = obj_type_val
                    if obj_type_val not in static_object_reverse:
                        self.logger.warning(f"未知的static_object_type值: {obj_type_val}，使用默认值")
                        final_params['static_object_type'] = "static.prop.barrel"
                    else:
                        final_params['static_object_type'] = static_object_reverse[obj_type_val]
                

                
            except Exception as e:
                self.logger.error(f"参数转换失败: {e}")
                self.logger.error(f"使用默认参数值")
                final_params['vehicle_type'] = "vehicle.lincoln.mkz_2017"
                final_params['sensor_type'] = "sensor.camera.rgb"
                final_params['weather_preset'] = "ClearNoon"
                numerical_params['vehicle_type'] = 0
                numerical_params['sensor_type'] = 0
                numerical_params['weather_preset'] = 0
            
            self.logger.info(f"转换后的最终参数: {final_params}")
            
            # 重启CARLA准备运行H1
            self.stop_carla()
            self.start_carla()
            time.sleep(5)
            
            # 运行高负载环境 H1
            self.logger.info("运行H1场景...")
            H1_output, exp_dir = self.run_traffic_simulation(
                final_params, 
                is_high_load=True,
                run_label="H1",
                iteration=len(self.error_log)
            )
            
            if H1_output is None:
                return 0.0
            
            H1_record = H1_output.replace("metrics", "recorded_data")
            
            # 确保H1的文件已经写入
            if not os.path.exists(H1_record):
                self.logger.error(f"H1记录文件未找到: {H1_record}")
                return 0.0
            
            # 重启CARLA准备运行L1
            self.stop_carla()
            time.sleep(3)  # 等待所有进程完全终止
            self.start_carla()
            time.sleep(5)  # 给更多时间初始化
            
            # 运行L1
            self.logger.info("运行L1场景...")
            L1_output, _ = self.run_traffic_simulation(
                final_params,
                is_high_load=False,
                replay_data=H1_record,
                run_label="L1",
                iteration=len(self.error_log)
            )
            
            if L1_output is None:
                return 0.0
            
            # 读取指标并计算误差
            with open(H1_record, 'r') as f:
                H1_metrics = json.load(f)
            with open(L1_output, 'r') as f:
                L1_metrics = json.load(f)
            
            error = self.compute_error_difference(H1_metrics, L1_metrics)
            
            # 使用numerical_params来更新历史记录
            self.X_history.append([float(v) for v in numerical_params.values()])
            self.y_history.append(-error)
            
            # 更新error_log (使用字符串版本的参数)
            self.error_log.append({
                "iteration": len(self.error_log),
                "local_rei": error,
                "prev_rei": self.error_log[-1]["local_rei"] if self.error_log else error,
                "search_type": "global",
                "params": final_params
            })
            
            
            
            self.logger.info(f"完成迭代，误差值: {error}")
            # self.logger.info(f"当前error_log: {self.error_log}")
            # 记录目标函数评估结束时的资源状态
            self.resource_monitor.log_snapshot(f"目标函数结束_迭代{len(self.X_history)}")
            return float(-error)
            
        except Exception as e:
            self.logger.error(f"目标函数评估失败: {e}")
            traceback.print_exc()
            # 记录异常时的资源状态
            self.resource_monitor.log_snapshot(f"目标函数异常_迭代{len(self.X_history)}")
            return 0.0
        finally:
            # 确保CARLA被关闭
            self.stop_carla()
            time.sleep(2)

    def compute_error_difference(self, high_metrics: Dict, low_metrics: List[Dict]) -> float:
        """计算误差并存储详细信息"""
        self.logger.info("\n" + "="*50)
        self.logger.info("开始计算误差差异:")
        
        def min_max_normalize(errors):
            """对误差进行最大值归一化"""
            if len(errors) == 0:
                return []
            errors_array = np.array(errors)
            min_val = np.min(errors_array)
            max_val = np.max(errors_array)
            if max_val == min_val:
                return np.zeros_like(errors_array)
            return (errors_array - min_val) / (max_val - min_val)

        # 将low_metrics转换为以ori_frame_id和original_id为键的嵌套字典
        replay_lookup = {}
        for item in low_metrics:
            frame_id = item['ori_frame_id']
            vehicle_id = item['original_id']
            if frame_id not in replay_lookup:
                replay_lookup[frame_id] = {}
            replay_lookup[frame_id][vehicle_id] = item

        # 收集所有误差
        position_errors = []
        velocity_errors = []
        acceleration_errors = []
        velocities = []  # 用于加速度误差加权

        matched_frames = 0
        matched_vehicles = 0
        total_vehicles = 0

        # 遍历高负载数据中的每一帧
        for frame_str, frame_data in high_metrics.items():
            frame_id = int(frame_str)
            total_vehicles += len(frame_data['vehicles'])

            if frame_id in replay_lookup:
                matched_frames += 1

                # 遍历该帧中的每个车辆
                for high_vehicle in frame_data['vehicles']:
                    vehicle_id = high_vehicle['id']

                    # 检查是否有对应的回放数据
                    if vehicle_id in replay_lookup[frame_id]:
                        matched_vehicles += 1
                        low_vehicle = replay_lookup[frame_id][vehicle_id]

                        try:
                            # 计算位置误差
                            pos_error = np.sqrt(
                                (high_vehicle['x'] - low_vehicle['x'])**2 +
                                (high_vehicle['y'] - low_vehicle['y'])**2 +
                                (high_vehicle['z'] - low_vehicle['z'])**2
                            )
                            position_errors.append(pos_error)

                            # 计算速度误差
                            high_vel_mag = np.sqrt(
                                high_vehicle['velocity']['x']**2 +
                                high_vehicle['velocity']['y']**2 +
                                high_vehicle['velocity']['z']**2
                            )
                            low_vel_mag = np.sqrt(
                                low_vehicle['velocity']['x']**2 +
                                low_vehicle['velocity']['y']**2 +
                                low_vehicle['velocity']['z']**2
                            )
                            vel_error = abs(high_vel_mag - low_vel_mag)
                            velocity_errors.append(vel_error)
                            velocities.append(high_vel_mag)  # 记录速度用于加权

                            # 计算加速度误差
                            high_acc_mag = np.sqrt(
                                high_vehicle['acceleration']['x']**2 +
                                high_vehicle['acceleration']['y']**2 +
                                high_vehicle['acceleration']['z']**2
                            )
                            low_acc_mag = np.sqrt(
                                low_vehicle['acceleration']['x']**2 +
                                low_vehicle['acceleration']['y']**2 +
                                low_vehicle['acceleration']['z']**2
                            )
                            acc_error = abs(high_acc_mag - low_acc_mag)
                            acceleration_errors.append(acc_error)

                            # 记录大误差的详细信息
                            if pos_error > 1.0 or vel_error > 1.0 or acc_error > 1.0:
                                self.logger.debug(f"\n发现大误差:")
                                self.logger.debug(f"帧ID: {frame_id}, 车辆ID: {vehicle_id}")
                                self.logger.debug(f"位置误差: {pos_error:.4f}")
                                self.logger.debug(f"速度误差: {vel_error:.4f}")
                                self.logger.debug(f"加速度误差: {acc_error:.4f}")

                        except Exception as e:
                            self.logger.warning(f"计算车辆 {vehicle_id} 在帧 {frame_id} 的误差时出错: {e}")
                            continue

        # 输出匹配统计
        self.logger.info("\n匹配统计:")
        self.logger.info(f"匹配帧数: {matched_frames}/{len(high_metrics)} ({matched_frames/len(high_metrics)*100:.1f}%)")
        self.logger.info(f"匹配车辆总数: {matched_vehicles}/{total_vehicles} ({matched_vehicles/total_vehicles*100:.1f}%)")

        if not position_errors:
            self.logger.warning("没有收集到有效的误差数据")
            return float('inf')

        # 最大值归一化所有误差
        norm_pos_errors = min_max_normalize(position_errors)
        norm_vel_errors = min_max_normalize(velocity_errors)
        norm_acc_errors = min_max_normalize(acceleration_errors)

        # 计算加速度误差的速度加权
        k = 1.0  # 速度加权系数，可以调整
        weighted_acc_errors = []
        for acc_err, vel in zip(norm_acc_errors, velocities):
            weight = 1.0 + k * vel
            weighted_acc_errors.append(acc_err * weight)

        # 计算最终误差（可以调整权重）
        final_error = (
            np.mean(norm_pos_errors) + 
            np.mean(norm_vel_errors) + 
            np.mean(weighted_acc_errors)
        )

        # 输出原始误差的统计信息
        self.logger.info("\n原始误差统计:")
        self.logger.info(f"位置误差 - 最大值: {np.max(position_errors):.4f}, 最小值: {np.min(position_errors):.4f}")
        self.logger.info(f"速度误差 - 最大值: {np.max(velocity_errors):.4f}, 最小值: {np.min(velocity_errors):.4f}")
        self.logger.info(f"加速度误差 - 最大值: {np.max(acceleration_errors):.4f}, 最小值: {np.min(acceleration_errors):.4f}")

        # 输出归一化后的误差统计
        self.logger.info("\n归一化误差统计:")
        self.logger.info(f"位置误差 - 归一化平均值: {np.mean(norm_pos_errors):.4f}")
        self.logger.info(f"速度误差 - 归一化平均值: {np.mean(norm_vel_errors):.4f}")
        self.logger.info(f"加速度误差 - 加权归一化平均值: {np.mean(weighted_acc_errors):.4f}")
        self.logger.info(f"最终误差值: {final_error:.4f}")
        self.logger.info("="*50 + "\n")

        frame_errors = {}  # 按帧存储误差
        for frame_str, frame_data in high_metrics.items():
            frame_id = int(frame_str)
            frame_errors[frame_id] = {
                'position': [],
                'velocity': [],
                'acceleration': []
            }
            
            if frame_id in replay_lookup:
                for high_vehicle in frame_data['vehicles']:
                    vehicle_id = high_vehicle['id']
                    if vehicle_id in replay_lookup[frame_id]:
                        low_vehicle = replay_lookup[frame_id][vehicle_id]
                        
                        # 计算并存储每帧的误差
                        pos_error = np.sqrt(
                            (high_vehicle['x'] - low_vehicle['x'])**2 +
                            (high_vehicle['y'] - low_vehicle['y'])**2 +
                            (high_vehicle['z'] - low_vehicle['z'])**2
                        )
                        frame_errors[frame_id]['position'].append(pos_error)
                        
                        # ... 速度和加速度误差计算 ...
                        frame_errors[frame_id]['velocity'].append(vel_error)
                        frame_errors[frame_id]['acceleration'].append(acc_error)

        # 计算误差趋势特征
        trend_features = self._compute_trend_features(frame_errors)
        
        # 存储本次实验的详细误差数据
        error_details = {
            'position_errors': position_errors,
            'velocity_errors': velocity_errors,
            'acceleration_errors': acceleration_errors,
            'frame_errors': frame_errors,
            'trend_features': trend_features,
            'final_error': final_error
        }
        self.detailed_errors.append(error_details)
        
        return float(final_error)

    def _compute_trend_features(self, frame_errors):
        """计算误差变化趋势的特征"""
        sorted_frames = sorted(frame_errors.keys())
        
        # 提取每帧的平均误差
        pos_means = [np.mean(frame_errors[f]['position']) for f in sorted_frames]
        vel_means = [np.mean(frame_errors[f]['velocity']) for f in sorted_frames]
        acc_means = [np.mean(frame_errors[f]['acceleration']) for f in sorted_frames]
        
        # 计算趋势特征
        trend_features = {
            'position': {
                'start_error': pos_means[0],
                'end_error': pos_means[-1],
                'max_error': max(pos_means),
                'trend_slope': np.polyfit(range(len(pos_means)), pos_means, 1)[0],
                'fluctuation': np.std(pos_means),
                'peak_count': len(scipy_signal.find_peaks(pos_means)[0])
            },
            'velocity': {
                'start_error': vel_means[0],
                'end_error': vel_means[-1],
                'max_error': max(vel_means),
                'trend_slope': np.polyfit(range(len(vel_means)), vel_means, 1)[0],
                'fluctuation': np.std(vel_means),
                'peak_count': len(scipy_signal.find_peaks(vel_means)[0])
            },
            'acceleration': {
                'start_error': acc_means[0],
                'end_error': acc_means[-1],
                'max_error': max(acc_means),
                'trend_slope': np.polyfit(range(len(acc_means)), acc_means, 1)[0],
                'fluctuation': np.std(acc_means),
                'peak_count': len(scipy_signal.find_peaks(acc_means)[0])
            }
        }
        return trend_features

    def _convert_to_serializable(self, obj):
        """递归转换所有数值为Python原生类型"""
        if isinstance(obj, (np.int_, np.intc, np.intp, np.int8, np.int16, np.int32, np.int64,
                          np.uint8, np.uint16, np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.float_, np.float16, np.float32, np.float64)):
            return float(obj)
        elif isinstance(obj, dict):
            return {k: self._convert_to_serializable(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [self._convert_to_serializable(item) for item in obj]
        return obj

    
    def check_ucb_convergence(self, X, delta=0.01):
        """检查UCB是否收敛 - 支持多峰模型"""
        if len(X) < 2:
            return False
        
        try:
            X_numeric = np.array(X).astype(np.float64)  # 现在X已经全是数值了
            
            # 检查是否有多个GP模型（多峰优化场景）
            if hasattr(self, 'gp_models') and isinstance(self.gp_models, list) and len(self.gp_models) > 0:
                self.logger.info(f"使用多峰模型({len(self.gp_models)}个)进行UCB收敛检查")
                all_std_max = []
                for i, gp in enumerate(self.gp_models):
                    _, std = gp.predict(X_numeric, return_std=True)
                    max_std = np.max(std)
                    all_std_max.append(max_std)
                    self.logger.info(f"模型{i+1}的最大不确定性: {max_std:.6f}")
                
                

                max_uncertainty = np.max(all_std_max)
                self.logger.info(f"多峰模型整体最大不确定性: {max_uncertainty:.6f}, 阈值: {delta}")
                return max_uncertainty < delta
            else:
                # 单一GP模型情况
                _, std = self.gp.predict(X_numeric, return_std=True)
                max_std = np.max(std)
                self.logger.info(f"单一模型最大不确定性: {max_std:.6f}, 阈值: {delta}")
                return max_std < delta
        except Exception as e:
            self.logger.warning(f"UCB收敛检查失败: {e}")
            self.logger.exception("详细错误:")
            return False

    def check_delta_rei_convergence(self, epsilon=0.01, N=5):
        """检查误差增量是否收敛"""
        if len(self.error_log) < N:
            self.logger.info(f"历史记录数量({len(self.error_log)})不足以进行REI增量收敛检查，需要至少{N}条记录")
            return False
            
        # 计算最近N次实验的REI增量
        recent_deltas = [entry["local_rei"] - entry.get("prev_rei", entry["local_rei"]) 
                        for entry in self.error_log[-N:]]
        
        delta_std = np.std(recent_deltas)
        delta_mean = np.mean(recent_deltas)
        self.logger.info(f"最近{N}次实验的REI增量: {recent_deltas}")
        self.logger.info(f"REI增量均值: {delta_mean:.6f}, 标准差: {delta_std:.6f}, 阈值: {epsilon}")
        
        # 如果均值接近零且标准差小，认为已收敛
        is_converged = delta_std < epsilon and abs(delta_mean) < epsilon * 2
        self.logger.info(f"REI增量收敛检查结果: {'收敛' if is_converged else '未收敛'}")
        
        return is_converged

    def check_max_rei_convergence(self, epsilon=0.01, N=5):
        """检查最大误差是否收敛"""
        if len(self.error_log) < N:
            self.logger.info(f"历史记录数量({len(self.error_log)})不足以进行REI最大值收敛检查，需要至少{N}条记录")
            return False
            
        # 计算最近N次迭代中的历史最优REI
        max_reis = [max(e["local_rei"] for e in self.error_log[:i+1])
                   for i in range(len(self.error_log)-N, len(self.error_log))]
        
        max_rei_std = np.std(max_reis)
        max_rei_diff = max_reis[-1] - max_reis[0]  # 最近N次迭代中最大REI的改进量
        
        self.logger.info(f"最近{N}次迭代的最大REI值: {max_reis}")
        self.logger.info(f"最大REI标准差: {max_rei_std:.6f}, 最大REI改进量: {max_rei_diff:.6f}, 阈值: {epsilon}")
        
        # 如果标准差小且改进量不大，认为已收敛
        is_converged = max_rei_std < epsilon and max_rei_diff < epsilon * 2
        self.logger.info(f"REI最大值收敛检查结果: {'收敛' if is_converged else '未收敛'}")
        
        return is_converged

    def check_cluster_convergence(self, epsilon=0.01, min_cluster_size=3, improvement_threshold=0.005):
        """检查各个聚类区域是否已收敛，多峰优化特有的收敛检查
        
        Args:
            epsilon: 收敛判断阈值
            min_cluster_size: 判断聚类收敛所需的最小样本数
            improvement_threshold: 认为有显著改进的阈值
            
        Returns:
            bool: 是否已收敛
        """
        # 确保有足够的样本进行聚类
        if len(self.X_history) < 10:
            self.logger.info(f"样本量({len(self.X_history)})不足以进行聚类收敛检查")
            return False
            
        try:
            X = np.array(self.X_history)
            y = np.array(self.y_history)
            
            # 动态确定聚类数量
            n_clusters = min(3, max(2, len(X) // 10))
            
            # 聚类前检查样本数量
            if len(X) < n_clusters * min_cluster_size:
                self.logger.info(f"样本量({len(X)})不足以进行{n_clusters}个聚类的收敛检查")
                return False
                
            # 使用KMeans聚类
            from sklearn.cluster import KMeans
            kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
            clusters = kmeans.fit_predict(X)
            
            # 检查每个聚类的收敛情况
            cluster_converged = []
            
            for i in range(n_clusters):
                # 提取该聚类的点和值
                mask = clusters == i
                if np.sum(mask) < min_cluster_size:
                    self.logger.info(f"聚类{i+1}样本量({np.sum(mask)})不足，跳过收敛检查")
                    continue
                    
                # 获取该聚类的点
                X_cluster = X[mask]
                y_cluster = y[mask]
                
                # 根据时间顺序排序（假设样本点是按时间顺序添加的）
                indices = np.where(mask)[0]
                sorted_indices = np.argsort(indices)
                y_sorted = y_cluster[sorted_indices]
                
                # 计算最近几次的改进情况
                if len(y_sorted) >= min_cluster_size:
                    # 获取最近几次的最大值
                    recent_max = [np.max(y_sorted[:i+1]) for i in range(len(y_sorted)-min_cluster_size, len(y_sorted))]
                    recent_improvements = [recent_max[i] - recent_max[i-1] for i in range(1, len(recent_max))]
                    
                    # 如果最近的改进都很小，认为该聚类已收敛
                    avg_improvement = np.mean(recent_improvements) if recent_improvements else 0
                    is_cluster_converged = avg_improvement < improvement_threshold
                    
                    self.logger.info(f"聚类{i+1}({len(y_sorted)}个点)最近改进: {recent_improvements}")
                    self.logger.info(f"聚类{i+1}平均改进: {avg_improvement:.6f}, 阈值: {improvement_threshold}")
                    self.logger.info(f"聚类{i+1}收敛检查结果: {'已收敛' if is_cluster_converged else '未收敛'}")
                    
                    cluster_converged.append(is_cluster_converged)
            
            # 计算收敛的聚类比例
            if not cluster_converged:
                self.logger.info("没有足够的聚类进行收敛判断")
                return False
                
            converged_ratio = sum(cluster_converged) / len(cluster_converged)
            self.logger.info(f"聚类收敛比例: {converged_ratio:.2f} ({sum(cluster_converged)}/{len(cluster_converged)})")
            
            # 当大部分聚类都收敛时，认为整体收敛
            is_converged = converged_ratio >= 0.7  # 70%的聚类收敛即可
            self.logger.info(f"聚类整体收敛检查结果: {'收敛' if is_converged else '未收敛'}")
            
            return is_converged
            
        except Exception as e:
            self.logger.warning(f"聚类收敛检查失败: {e}")
            self.logger.exception("详细错误:")
            return False



    def _find_unvisited_cell(self):
        """查找参数空间中未访问的网格单元"""
        try:
            # 记录函数开始时的内存
            mem_start = get_system_resources()
            self.logger.info(f"=====> DEBUG: _find_unvisited_cell开始, 内存使用: {mem_start['memory_percent']}%")
            
            # 获取参数空间维度
            dims = len(self.space)
            self.logger.info(f"=====> DEBUG: 参数空间维度: {dims}")
            
            # 使用智能维度选择策略，而不是简单的随机采样
            if dims > 15:  # 高维度情况
                self.logger.info(f"=====> DEBUG: 高维度参数空间({dims}), 使用智能子空间采样")
                
                # 策略1: 基于重要性的维度筛选
                # 我们选择最重要的几个维度进行网格划分，其余维度保持随机
                important_dims = self._select_important_dimensions(max_dims=6)
                self.logger.info(f"=====> DEBUG: 选择了{len(important_dims)}个重要维度: {important_dims}")
                
                # 为重要维度创建网格
                grid_divisions = 4  # 对重要维度可以使用更高的分辨率
                
                # 将历史点映射到重要维度的网格单元
                self.logger.info(f"=====> DEBUG: 将{len(self.X_history)}个历史点映射到选定维度的网格")
                visited_cells = set()
                
                for point in self.X_history:
                    cell_coords = []
                    for dim_idx in important_dims:
                        dim = self.space[dim_idx]
                        param_value = point[dim_idx]
                        
                        # 确定参数在该维度的范围
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                        else:
                            low, high = 0, 1  # 默认范围
                        
                        # 归一化并映射到网格
                        norm_value = (param_value - low) / (high - low) if high > low else 0.5
                        cell_index = min(int(norm_value * grid_divisions), grid_divisions-1)
                        cell_coords.append(cell_index)
                    
                    visited_cells.add(tuple(cell_coords))
                
                self.logger.info(f"=====> DEBUG: 在选定维度上，已访问单元格数量: {len(visited_cells)}")
                
                # 在重要维度上生成网格单元
                import itertools
                all_important_cells = list(itertools.product(range(grid_divisions), repeat=len(important_dims)))
                
                # 找出未访问的单元格
                unvisited_cells = [cell for cell in all_important_cells if cell not in visited_cells]
                
                if unvisited_cells:
                    self.logger.info(f"发现{len(unvisited_cells)}个未访问的重要维度网格单元")
                    # 选择一个未访问的单元格
                    chosen_cell = random.choice(unvisited_cells)
                    
                    # 创建完整的cell，重要维度使用选定值，其余维度随机
                    full_cell = []
                    important_idx = 0
                    
                    for i in range(dims):
                        if i in important_dims:
                            # 使用选定单元格的值
                            full_cell.append(chosen_cell[important_idx])
                            important_idx += 1
                        else:
                            # 其他维度随机取值
                            full_cell.append(random.randint(0, 1))  # 非重要维度使用较低分辨率
                    
                    self.logger.info(f"=====> DEBUG: 生成完整单元格，重要维度使用选定值，其他维度随机")
                    
                    # 记录函数结束时的内存
                    mem_end = get_system_resources()
                    self.logger.info(f"=====> DEBUG: _find_unvisited_cell结束, 内存使用: {mem_end['memory_percent']}%")
                    
                    return tuple(full_cell)
                else:
                    self.logger.info(f"未找到未访问的重要维度网格单元，尝试基于历史数据的探索")
                    
                    # 策略2: 基于历史数据分布的探索
                    X = np.array(self.X_history)
                    mean = np.mean(X, axis=0)
                    std = np.std(X, axis=0)
                    std = np.where(std < 1e-6, 1.0, std)  # 避免标准差为0的情况
                    
                    # 在每个维度上，生成一个远离历史数据中心的点
                    new_point_coords = []
                    for i, dim in enumerate(self.space):
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                        else:
                            low, high = 0, 1
                        
                        # 偏向于探索低密度区域
                        if i in important_dims:
                            # 对重要维度使用更极端的偏移
                            offset = np.random.choice([-2.5, 2.5]) * std[i]  # 更倾向于边界区域
                        else:
                            offset = np.random.uniform(-2, 2) * std[i]
                        
                        new_val = mean[i] + offset
                        # 确保在边界内
                        new_val = np.clip(new_val, low, high)
                        
                        # 转换为网格索引
                        norm_value = (new_val - low) / (high - low) if high > low else 0.5
                        grid_idx = min(int(norm_value * 2), 1)  # 使用低分辨率(2)
                        new_point_coords.append(grid_idx)
                    
                    self.logger.info(f"=====> DEBUG: 基于历史数据分布生成探索点")
                    
                    # 记录函数结束时的内存
                    mem_end = get_system_resources()
                    self.logger.info(f"=====> DEBUG: _find_unvisited_cell结束, 内存使用: {mem_end['memory_percent']}%")
                    
                    return tuple(new_point_coords)
            
            # 对于低维度参数空间，使用原来的网格方法
            else:
                # 参数空间划分为网格 - 降低分辨率以减少内存使用
                grid_divisions = 2  # 每个维度分2份
                
                # 将历史点映射到网格单元
                self.logger.info(f"=====> DEBUG: 将{len(self.X_history)}个历史点映射到网格")
                visited_cells = set()
                for point in self.X_history:
                    cell_coords = []
                    for i, dim in enumerate(self.space):
                        # 获取该维度的参数值，并映射到0-grid_divisions范围
                        param_value = point[i]  # X_history中点的值直接是列表
                        
                        # 确定参数在该维度的范围
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                        else:
                            low, high = 0, 1  # 默认范围
                        
                        # 归一化并映射到网格
                        norm_value = (param_value - low) / (high - low) if high > low else 0.5
                        cell_index = min(int(norm_value * grid_divisions), grid_divisions-1)
                        cell_coords.append(cell_index)
                    
                    visited_cells.add(tuple(cell_coords))
                
                self.logger.info(f"=====> DEBUG: 已访问单元格数量: {len(visited_cells)}")
                
                # 生成所有可能的单元格坐标
                import itertools
                max_possible_cells = grid_divisions ** dims
                self.logger.info(f"=====> DEBUG: 可能的单元格总数: {max_possible_cells}")
                
                if max_possible_cells > 10000:  # 如果可能的单元格超过10000个
                    self.logger.info(f"=====> DEBUG: 单元格数量过多，使用随机抽样")
                    
                    # 直接随机生成一些单元格坐标，而不是生成所有可能的组合
                    sampled_cells = []
                    for _ in range(1000):  # 最多考虑1000个随机单元格
                        cell = tuple(random.randint(0, grid_divisions-1) for _ in range(dims))
                        if cell not in visited_cells and cell not in sampled_cells:
                            sampled_cells.append(cell)
                    
                    # 从未访问的单元格中随机选择一个
                    if sampled_cells:
                        chosen_cell = random.choice(sampled_cells)
                        self.logger.info(f"=====> DEBUG: 通过随机抽样找到未访问单元格")
                        
                        # 记录函数结束时的内存
                        mem_end = get_system_resources()
                        self.logger.info(f"=====> DEBUG: _find_unvisited_cell结束, 内存使用: {mem_end['memory_percent']}%")
                        
                        return chosen_cell
                    
                    self.logger.info(f"=====> DEBUG: 未找到未访问单元格")
                    return None
                
                # 如果单元格数量不多，正常处理
                self.logger.info(f"=====> DEBUG: 开始生成所有可能单元格")
                all_cells = list(itertools.product(range(grid_divisions), repeat=dims))
                
                # 找出未访问的单元格
                self.logger.info(f"=====> DEBUG: 开始查找未访问单元格")
                unvisited_cells = [cell for cell in all_cells if cell not in visited_cells]
                
                if unvisited_cells:
                    self.logger.info(f"发现{len(unvisited_cells)}个未访问网格单元")
                    result = random.choice(unvisited_cells)
                    
                    # 记录函数结束时的内存
                    mem_end = get_system_resources()
                    self.logger.info(f"=====> DEBUG: _find_unvisited_cell结束, 内存使用: {mem_end['memory_percent']}%")
                    
                    return result
                
                self.logger.info(f"=====> DEBUG: 未找到未访问单元格")
                # 记录函数结束时的内存
                mem_end = get_system_resources()
                self.logger.info(f"=====> DEBUG: _find_unvisited_cell结束, 内存使用: {mem_end['memory_percent']}%")
                
                return None
            
        except Exception as e:
            self.logger.error(f"查找未访问单元格失败: {e}")
            self.logger.exception("详细错误信息：")
            return None

    def _select_important_dimensions(self, max_dims=6):
        """选择最重要的几个维度进行探索
        
        实现两种选择策略:
        1. 基于Y值敏感度的维度选择（主要方法）
        2. 随机选择策略（作为后备）
        """
        try:
            # 策略1: 基于Y值敏感度的维度选择（相关性分析）
            if len(self.X_history) >= 5 and len(self.y_history) >= 5:  # 降低样本量要求
                X = np.array(self.X_history)
                y = np.array(self.y_history)
                
                # 计算每个维度与Y的相关性
                correlations = []
                for i in range(X.shape[1]):
                    # 使用绝对值考虑正负相关
                    corr = np.abs(np.corrcoef(X[:, i], y)[0, 1])
                    if np.isnan(corr):
                        corr = 0
                    correlations.append(corr)
                
                # 选择相关性最强的几个维度
                important_idx = np.argsort(correlations)[-max_dims:]
                # self.logger.info(f"=====> DEBUG: 基于Y值相关性选择维度: {important_idx.tolist()}")
                # self.logger.info(f"=====> DEBUG: 相关性强度: {[correlations[i] for i in important_idx]}")
                
                
                sorted_strengths = sorted([correlations[i] for i in important_idx], reverse=True)
                self.logger.info(f"=====> DEBUG: 基于Y值相关性选择维度: {important_idx.tolist()}")
                self.logger.info(f"=====> DEBUG: 相关性强度(降序): {sorted_strengths}")
                return important_idx.tolist()
            
            # 策略2: 随机选择策略（后备方案）
            self.logger.info(f"=====> WARNING: 基于Y值敏感度选择的样本不足({len(self.X_history)}个), 使用随机选择策略")
            all_dims = list(range(len(self.space)))
            important_dims = random.sample(all_dims, min(max_dims, len(all_dims)))
            return important_dims
            
        except Exception as e:
            self.logger.error(f"选择重要维度失败: {e}")
            self.logger.exception("详细错误信息：")
            # 返回随机选择的维度作为后备
            dims = list(range(len(self.space)))
            return random.sample(dims, min(max_dims, len(dims)))



    def _generate_point_in_cell(self, cell):
        """在指定网格单元内生成随机点，确保参数在有效范围内且类型正确"""
        try:
            # 记录函数开始时的内存
            mem_start = get_system_resources()
            self.logger.info(f"=====> DEBUG: _generate_point_in_cell开始, 内存使用: {mem_start['memory_percent']}%")
            
            # 创建点字典
            point = {}
            
            # 检查cell是否为数字列表而非元组
            if isinstance(cell, (list, np.ndarray)) and all(isinstance(x, (int, float, np.number)) for x in cell):
                self.logger.info(f"=====> DEBUG: 直接使用提供的数值点坐标，但保证在有效范围内且类型正确")
                for i, dim in enumerate(self.space):
                    if i < len(cell):  # 防止索引越界
                        # 获取原始值，稍后会根据维度类型进行适当处理
                        raw_value = cell[i]
                    else:
                        # 如果cell维度不足，获取随机值
                        raw_value = dim.rvs()[0]
                    
                    # 将在下方统一处理类型和范围
                    point[dim.name] = raw_value
            else:
                # 从网格索引生成实际参数值
                grid_divisions = 2  # 使用与_find_unvisited_cell相同的网格分辨率
                
                # 判断cell长度是否与space维度匹配
                if len(cell) != len(self.space):
                    self.logger.warning(f"=====> DEBUG: cell长度({len(cell)})与维度({len(self.space)})不匹配，可能使用了子空间采样")
                    # 拓展cell或截断cell以匹配空间维度
                    if len(cell) < len(self.space):
                        # 如果cell维度小于space维度，随机补充缺失的维度
                        full_cell = list(cell)
                        for _ in range(len(self.space) - len(cell)):
                            full_cell.append(random.randint(0, grid_divisions-1))
                        cell = tuple(full_cell)
                    else:
                        # 如果cell维度大于space维度，截断
                        cell = cell[:len(self.space)]
                
                # 生成初始点值（稍后会根据类型处理）
                for i, dim in enumerate(self.space):
                    # 确定参数在该维度的范围
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                    else:
                        low, high = 0, 1  # 默认范围
                    
                    # 计算单元格在该维度的范围
                    cell_low = low + (high - low) * cell[i] / grid_divisions
                    cell_high = low + (high - low) * (cell[i] + 1) / grid_divisions
                    
                    # 在单元格范围内随机生成值
                    raw_value = np.random.uniform(cell_low, cell_high)
                    point[dim.name] = raw_value
            
            # ================ 统一处理所有参数类型和范围 ================
            # 此时point包含所有维度的原始值，现在根据每个参数的类型和范围进行处理
            
            # 1. 处理分类变量，确保在有效编码范围内
            # weather_preset
            if 'weather_preset' in point:
                # 确保weather_preset是整数并在有效范围内
                weather_val = int(min(max(0, point['weather_preset']), len(self.weather_mapping) - 1))
                point['weather_preset'] = weather_val
            
            # vehicle_type
            if 'vehicle_type' in point:
                # 确保vehicle_type是整数并在有效范围内
                vehicle_val = int(min(max(0, point['vehicle_type']), len(self.vehicle_type_mapping) - 1))
                point['vehicle_type'] = vehicle_val
            
            # sensor_type
            if 'sensor_type' in point:
                # 确保sensor_type是整数并在有效范围内
                sensor_val = int(min(max(0, point['sensor_type']), len(self.sensor_type_mapping) - 1))
                point['sensor_type'] = sensor_val
            
            # town
            if 'town' in point:
                # 确保town是整数并在有效范围内
                town_val = int(min(max(0, point['town']), len(self.town_mapping) - 1))
                point['town'] = town_val
            
            # static_object_type - 特别注意这里，确保在有效范围内
            if 'static_object_type' in point:
                # 确保static_object_type是整数并在有效范围内
                obj_val = int(min(max(0, point['static_object_type']), len(self.static_object_mapping) - 1))
                point['static_object_type'] = obj_val
            
            # 2. 处理整数参数
            # 直接处理这些参数为整数类型并确保在其范围内
            integer_params = [
                'num_vehicles', 'num_walkers', 'sensor_number', 
                'static_object_number', 'lidar_channels', 'lidar_points'
            ]
            
            for param in integer_params:
                if param in point:
                    # 查找该参数在space中的维度对象以获取范围
                    for dim in self.space:
                        if dim.name == param:
                            if hasattr(dim, 'bounds'):
                                low, high = dim.bounds
                                # 确保值在界限内并转换为整数
                                point[param] = int(min(max(low, point[param]), high))
                            else:
                                # 如果没有明确的界限，则至少确保是正整数
                                point[param] = max(1, int(point[param]))
                            break
            
            # 3. 处理连续变量，确保在范围内
            # 遍历其余所有参数，确保它们在各自的范围内
            for dim in self.space:
                if dim.name in point and dim.name not in integer_params and not dim.name in [
                    'weather_preset', 'vehicle_type', 'sensor_type', 'town', 'static_object_type'
                ]:
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        # 确保值在界限内
                        point[dim.name] = min(max(low, point[dim.name]), high)
            
            # 记录函数结束时的内存
            mem_end = get_system_resources()
            self.logger.info(f"=====> DEBUG: _generate_point_in_cell结束, 内存使用: {mem_end['memory_percent']}%")
            
            return point
            
        except Exception as e:
            self.logger.error(f"在单元格内生成点失败: {e}")
            self.logger.exception("详细错误信息：")
            
            # 如果失败，使用安全的后备方案
            safe_point = {}
            for dim in self.space:
                if isinstance(dim, Integer):
                    # 整数参数
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        safe_point[dim.name] = int(np.random.uniform(low, high))
                    else:
                        safe_point[dim.name] = max(1, int(dim.rvs()[0]))
                        
                elif dim.name in ['weather_preset', 'vehicle_type', 'sensor_type', 'town', 'static_object_type']:
                    # 分类变量
                    if dim.name == 'weather_preset':
                        safe_point[dim.name] = random.randint(0, len(self.weather_mapping) - 1)
                    elif dim.name == 'vehicle_type':
                        safe_point[dim.name] = random.randint(0, len(self.vehicle_type_mapping) - 1)
                    elif dim.name == 'sensor_type':
                        safe_point[dim.name] = random.randint(0, len(self.sensor_type_mapping) - 1)
                    elif dim.name == 'town':
                        safe_point[dim.name] = random.randint(0, len(self.town_mapping) - 1)
                    elif dim.name == 'static_object_type':
                        safe_point[dim.name] = random.randint(0, len(self.static_object_mapping) - 1)
                
                else:
                    # 连续参数
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        safe_point[dim.name] = np.random.uniform(low, high)
                    else:
                        safe_point[dim.name] = dim.rvs()[0]
            
            return safe_point

    def _get_next_point(self, exploration_boost=True):
        """根据当前优化状态生成下一个实验点

        Args:
            exploration_boost: 是否启用增强的探索机制，默认为True

        Returns:
            下一个待评估的实验点
        """
        try:
            self.logger.info(f"正在生成下一个实验点，当前数据长度: {len(self.X_history)}")
            # 检查是否有足够的历史点
            X = np.array(self.X_history)  # 直接使用X_history   
            if len(X) < 30:
                # 对于样本数据不足的情况，使用随机采样
                next_point = self._generate_random_point()
                self.logger.info(f"样本不足，使用随机生成的点: {next_point}")
                return next_point
            
            # 动态计算探索率，随着迭代次数增加逐渐减小，但保持最小探索率
            exploration_rate = max(0.3, min(0.9, 1.0 - len(self.X_history) / 200))
            if exploration_boost:
                exploration_rate = max(exploration_rate, 0.6)  # 确保探索率不低于0.5
            
            # 决定是否进行强制探索
            do_forced_exploration = np.random.rand() < exploration_rate
            self.logger.info(f"当前探索率: {exploration_rate:.2f}, 强制探索: {do_forced_exploration}")
            
            # 计算参数重要性权重
            param_weights = np.ones(len(self.space))    #所以这里比如我们有10个参数，那么我们就有10个权重，初始值都是1
            
            # 如果有聚类信息，利用聚类结果调整权重
            if hasattr(self, 'cluster_stats') and self.cluster_stats and np.random.rand() < 0.7:
                try:
                    # 获取所有错误类型和聚类中心
                    error_types = []
                    for cluster in self.cluster_stats:
                        if 'main_error_type' in cluster:
                            error_types.append(cluster['main_error_type'])
                    
                    # 随机选择一种错误类型进行探索
                    if len(error_types) > 0:
                        target_error = np.random.choice(error_types)
                        target_clusters = [c for c in self.cluster_stats if c.get('main_error_type') == target_error]
                        
                        if target_clusters:
                            # 随机选择一个目标聚类
                            target_cluster = np.random.choice(target_clusters)
                            
                            # 分析聚类中的重要参数
                            if 'feature_importance' in target_cluster:
                                importances = target_cluster['feature_importance']
                                # 调整权重：将重要参数的权重增加
                                for i, imp in enumerate(importances):
                                    param_weights[i] = max(1.0, imp * 2)
                                    # 这里的params weights是每次都根据选中的cluster不同而进行更新的
                                
                                self.logger.info(f"针对错误类型 '{target_error}' 调整参数权重: {param_weights}")
                except Exception as e:
                    self.logger.warning(f"使用聚类信息调整权重时出错: {e}")
            

            important_dims = np.argsort(param_weights)[-min(5, len(param_weights)):]
            
            # 生成候选点策略
            candidate_strategies = []
            
            # 1. 随机采样 - 基础策略
            random_sampling_weight = 0.3
            if do_forced_exploration:
                random_sampling_weight = 0.5  # 强制探索时增加随机采样权重
                
            candidate_strategies.append(("随机采样", random_sampling_weight, lambda: self._generate_diverse_candidates(20, X)))
            
            # 2. 基于最佳点附近采样
            if len(self.X_history) >= 5:
                y = np.array(self.y_history)
                top_k = min(5, len(y))
                best_indices = np.argsort(y)[:top_k]  # 获取REI值最小的前k个点
                
                candidate_strategies.append(
                    ("最佳点附近采样", 0.3 * (1 - exploration_rate),
                    lambda: self._generate_candidates_near_best(X[best_indices]))
                )
            
            # 3. 拉丁超立方采样 - 促进空间均匀覆盖
            candidate_strategies.append(
                ("拉丁超立方采样", 0.25, 
                lambda: self._generate_lhs_candidates(15))
            )
            
            # 4. Thompson采样 - 平衡探索与利用
            if hasattr(self, 'gp') and self.gp is not None and len(X) >= 10:
                candidate_strategies.append(
                    ("Thompson采样", 0.25 * (1 - exploration_rate),
                    lambda: self._generate_thompson_candidates(15))
                )
                
            # 5. 多模型贝叶斯采样 - 使用聚类模型进行预测
            if hasattr(self, 'clustered_gps') and self.clustered_gps and any(gp is not None for gp in self.clustered_gps):
                candidate_strategies.append(
                    ("多模型贝叶斯采样", 0.35 * (1 - exploration_rate),
                    lambda: self._generate_clustered_gp_candidates(20))
                )
            
            # 根据权重选择策略
            strategy_names, strategy_weights, strategy_funcs = zip(*candidate_strategies)
            strategy_weights = np.array(strategy_weights)
            strategy_weights = strategy_weights / np.sum(strategy_weights)  # 归一化权重
            
            selected_idx = np.random.choice(len(strategy_names), p=strategy_weights)
            selected_strategy = strategy_names[selected_idx]
            self.logger.info(f"选择的候选点生成策略: {selected_strategy}")
            
            # 生成候选点
            try:
                candidates = strategy_funcs[selected_idx]()
            except Exception as e:
                self.logger.warning(f"选定的策略 {selected_strategy} 生成候选点时出错: {e}")
                self.logger.warning("回退到随机采样")
                candidates = self._generate_diverse_candidates(10, X)
            
            # 确保candidates不为None且不为空
            if not candidates:
                self.logger.warning(f"策略 {selected_strategy} 未生成有效候选点，使用随机采样")
                candidates = self._generate_diverse_candidates(10, X)
            
            # 如果仍然没有候选点，直接返回随机点
            if not candidates:
                self.logger.warning("无法生成有效候选点，返回单个随机点")
                return self._generate_random_point()
            
            # 评估候选点
            candidate_scores = []
            for candidate in candidates:
                try:
                    score = self._evaluate_candidate_point(candidate, X, candidates, param_weights)
                    candidate_scores.append((candidate, score))
                except Exception as e:
                    self.logger.warning(f"评估候选点时出错: {e}")
                    # 出错时给一个随机分数，不中断整个流程
                    candidate_scores.append((candidate, np.random.rand()))
            
            # 如果没有有效的候选点评分，返回随机点
            if not candidate_scores:
                self.logger.warning("无有效候选点评分，返回随机点")
                return self._generate_random_point()
            
            # 按评分排序
            candidate_scores.sort(key=lambda x: x[1], reverse=True)
            best_candidate = candidate_scores[0][0]
            
            # 转换为参数字典
            next_point = {}
            for i, param_name in enumerate(self.param_names):
                # 检查 best_candidate 是否为字典类型
                if isinstance(best_candidate, dict):
                    # 如果是字典，直接用参数名作为键
                    if param_name in best_candidate:
                        next_point[param_name] = best_candidate[param_name]
                    else:
                        # 参数名不在字典中，使用默认值
                        dim = self.space[i]
                        next_point[param_name] = (dim.low + dim.high) / 2 if hasattr(dim, 'low') and hasattr(dim, 'high') else dim.rvs()[0]
                else:
                    # 如果是列表或数组，使用索引
                    if i < len(best_candidate):
                        next_point[param_name] = best_candidate[i]
                    else:
                        # 如果维度不足，使用默认值
                        dim = self.space[i]
                        next_point[param_name] = (dim.low + dim.high) / 2 if hasattr(dim, 'low') and hasattr(dim, 'high') else dim.rvs()[0]
                    
           

           
            self.logger.info(f"选择的下一个实验点: {next_point}")
            return next_point
            
        except Exception as e:
            self.logger.error(f"生成下一个点时出错: {str(e)}")
            self.logger.exception("详细错误:")
            # 出错时返回随机点
            return self._generate_random_point()



      
    def cluster_diverse_bugs(self, n_clusters=3, save_multi_level=True):
        """对多样化错误进行聚类分析，支持双层聚类结果
        
        Args:
            n_clusters: 默认聚类数量，将被动态方法覆盖
            save_multi_level: 是否保存多层聚类结果
        
        Returns:
            聚类统计信息字典，包含不同粒度的聚类结果
        """
        try:
            self.logger.info("\n=== 开始聚类分析 ===")
            
            # 检查是否有足够的数据点
            if len(self.X_history) < 10:
                self.logger.warning(f"样本数量({len(self.X_history)})不足以进行有效聚类分析")
                return None
                
            # 确保detailed_errors存在
            if not hasattr(self, 'detailed_errors') or not self.detailed_errors:
                self.logger.warning("缺少详细错误数据，无法进行聚类分析")
                return None
                
            # 准备数据 - 筛选所有有效样本
            valid_indices = []
            for i, (y, errors) in enumerate(zip(self.y_history, self.detailed_errors)):
                if errors:  # 只要有错误数据即可
                    valid_indices.append(i)
                

            if len(valid_indices) < 5:
                self.logger.warning(f"有效样本数量({len(valid_indices)})不足以进行聚类")
                return None
                
            self.logger.info(f"筛选出{len(valid_indices)}个有效样本进行聚类分析")
            
            # 提取特征 - 结合参数值和错误特征
            X_processed = []
            for idx in valid_indices:
                features = []
                
                # 1. 加入参数值作为特征
                if isinstance(self.X_history[idx], dict):
                    features.extend([self.X_history[idx].get(dim.name, 0) for dim in self.space])
                else:
                    features.extend(self.X_history[idx])
                    
                # 2. 加入错误特征
                error_data = self.detailed_errors[idx]
                
                # 位置误差统计特征
                pos_errors = error_data.get('position_error', [])
                features.extend([
                    np.mean(pos_errors) if pos_errors else 0,
                    np.max(pos_errors) if pos_errors else 0,
                    np.std(pos_errors) if pos_errors else 0
                ])
                
                # 速度误差统计特征
                vel_errors = error_data.get('velocity_error', [])
                features.extend([
                    np.mean(vel_errors) if vel_errors else 0,
                    np.max(vel_errors) if vel_errors else 0,
                    np.std(vel_errors) if vel_errors else 0
                ])
                
                # 加速度误差统计特征
                acc_errors = error_data.get('acceleration_error', [])
                features.extend([
                    np.mean(acc_errors) if acc_errors else 0,
                    np.max(acc_errors) if acc_errors else 0,
                    np.std(acc_errors) if acc_errors else 0
                ])
                
                # 添加趋势特征
                if len(pos_errors) > 5:
                    # 位置误差趋势
                    slope_pos, fluct_pos = self._compute_trend_features(pos_errors)
                    peak_count_pos = len(signal.find_peaks(pos_errors)[0])
                    features.extend([slope_pos, fluct_pos, peak_count_pos])
                else:
                    features.extend([0, 0, 0])  # 默认趋势特征
                    
                if len(vel_errors) > 5:
                    # 速度误差趋势
                    slope_vel, fluct_vel = self._compute_trend_features(vel_errors)
                    peak_count_vel = len(signal.find_peaks(vel_errors)[0])
                    features.extend([slope_vel, fluct_vel, peak_count_vel])
                else:
                    features.extend([0, 0, 0])
                    
                if len(acc_errors) > 5:
                    # 加速度误差趋势
                    slope_acc, fluct_acc = self._compute_trend_features(acc_errors)
                    peak_count_acc = len(signal.find_peaks(acc_errors)[0])
                    features.extend([slope_acc, fluct_acc, peak_count_acc])
                else:
                    features.extend([0, 0, 0])
                    
                X_processed.append(features)
                
            X_processed = np.array(X_processed)
            
            # 标准化特征以避免量纲影响
            scaler = StandardScaler()
            X_scaled = scaler.fit_transform(X_processed)
            
            # 记录特征名称以便解释
            feature_names = (
                [dim.name for dim in self.space] +
                # 位置误差统计特征
                ["pos_mean", "pos_max", "pos_std"] +
                # 速度误差统计特征
                ["vel_mean", "vel_max", "vel_std"] +
                # 加速度误差统计特征
                ["acc_mean", "acc_max", "acc_std"] +
                # 趋势特征
                ["pos_slope", "pos_fluct", "pos_peaks", 
                "vel_slope", "vel_fluct", "vel_peaks",
                "acc_slope", "acc_fluct", "acc_peaks"]
            )
            
            # 维度检查
            if X_processed.shape[1] != len(feature_names):
                self.logger.warning(f"特征维度({X_processed.shape[1]})与特征名称数量({len(feature_names)})不匹配，将调整")
                # 调整feature_names以匹配实际维度
                if X_processed.shape[1] > len(feature_names):
                    feature_names.extend([f"unknown_{i}" for i in range(len(feature_names), X_processed.shape[1])])
                else:
                    feature_names = feature_names[:X_processed.shape[1]]
            
            # 如果维度较高，先进行降维
            X_for_cluster = X_scaled
            pca = None
            if X_processed.shape[1] > 15:
                from sklearn.decomposition import PCA
                self.logger.info(f"特征维度较高({X_processed.shape[1]})，应用PCA降维")
                n_components = min(15, len(valid_indices) // 2)
                n_components = max(n_components, 3)  # 确保至少保留3个主成分
                
                pca = PCA(n_components=n_components)
                X_for_cluster = pca.fit_transform(X_scaled)
                
                self.logger.info(f"PCA降维: {X_processed.shape[1]} -> {n_components} 维度")
                # 输出各主成分解释的方差比例
                variance_ratio = pca.explained_variance_ratio_
                self.logger.info(f"各主成分解释的方差比例: {[f'{r:.4f}' for r in variance_ratio]}")
                self.logger.info(f"累计解释方差: {sum(variance_ratio):.4f}")
            
            # 动态确定最佳聚类数量 - 使用多种评估指标
            from sklearn.cluster import KMeans
            from sklearn.metrics import silhouette_score, calinski_harabasz_score, davies_bouldin_score
            
            # 确定可能的聚类数量范围
            min_clusters = 2
            max_clusters = min(20, len(valid_indices) // 3)  # 每个簇至少3个样本
            max_clusters = max(min_clusters + 1, max_clusters)  # 确保至少有两个可选值
            
            self.logger.info(f"使用多种评估指标评估聚类数量范围: {min_clusters}-{max_clusters}")
            
            # 初始化存储聚类评估结果的结构
            evaluation_results = {
                'k_values': list(range(min_clusters, max_clusters + 1)),
                'inertias': [],
                'silhouette_scores': [],
                'ch_scores': [],
                'db_scores': [],
                'combined_scores': [],
                'all_labels': {}
            }
            
            # 计算不同聚类数量的各评估指标
            for k in evaluation_results['k_values']:
                kmeans = KMeans(n_clusters=k, random_state=42, n_init=10, max_iter=300)
                labels = kmeans.fit_predict(X_for_cluster)
                evaluation_results['all_labels'][k] = labels
                evaluation_results['inertias'].append(kmeans.inertia_)
                
                # 计算各种评价指标
                try:
                    if len(set(labels)) > 1:  # 确保有多个聚类标签
                        s_score = silhouette_score(X_for_cluster, labels)
                        ch_score = calinski_harabasz_score(X_for_cluster, labels)
                        db_score = davies_bouldin_score(X_for_cluster, labels)
                        
                        evaluation_results['silhouette_scores'].append(s_score)
                        evaluation_results['ch_scores'].append(ch_score)
                        evaluation_results['db_scores'].append(db_score)
                    else:
                        evaluation_results['silhouette_scores'].append(-1)
                        evaluation_results['ch_scores'].append(0)
                        evaluation_results['db_scores'].append(float('inf'))
                except Exception as e:
                    self.logger.warning(f"计算k={k}的评估指标失败: {e}")
                    evaluation_results['silhouette_scores'].append(-1)
                    evaluation_results['ch_scores'].append(0)
                    evaluation_results['db_scores'].append(float('inf'))
            
            # 计算综合得分
            for i in range(len(evaluation_results['k_values'])):
                # 标准化各评分
                s_norm = evaluation_results['silhouette_scores'][i] if evaluation_results['silhouette_scores'][i] > 0 else 0
                
                # CH分数归一化(越高越好)
                ch_max = max(evaluation_results['ch_scores']) if evaluation_results['ch_scores'] else 1
                ch_norm = evaluation_results['ch_scores'][i] / ch_max if ch_max > 0 else 0
                
                # DB分数归一化(越低越好)
                db_values = [s for s in evaluation_results['db_scores'] if s < float('inf')]
                db_min = min(db_values) if db_values else 0
                db_max = max(db_values) if db_values else 1
                db_range = db_max - db_min
                db_norm = 1 - ((evaluation_results['db_scores'][i] - db_min) / db_range) if db_range > 0 and evaluation_results['db_scores'][i] < float('inf') else 0
                
                # 综合得分 - 各指标权重可调整
                combined = 0.5 * s_norm + 0.3 * ch_norm + 0.2 * db_norm
                evaluation_results['combined_scores'].append(combined)
                
            # 输出详细的评估指标数据
            evaluation_table = []
            for i, k in enumerate(evaluation_results['k_values']):
                cluster_counts = np.bincount(evaluation_results['all_labels'][k]) if k in evaluation_results['all_labels'] else []
                cluster_sizes = ", ".join([f"{count}" for count in cluster_counts]) if len(cluster_counts) > 0 else "N/A"
                
                row = {
                    "聚类数": k,
                    "惯性值": f"{evaluation_results['inertias'][i]:.2f}",
                    "轮廓系数": f"{evaluation_results['silhouette_scores'][i]:.4f}" if evaluation_results['silhouette_scores'][i] > 0 else "N/A",
                    "CH指数": f"{evaluation_results['ch_scores'][i]:.2f}" if evaluation_results['ch_scores'][i] > 0 else "N/A",
                    "DB指数": f"{evaluation_results['db_scores'][i]:.4f}" if evaluation_results['db_scores'][i] < float('inf') else "N/A",
                    "各簇样本数": cluster_sizes
                }
                evaluation_table.append(row)
            
            self.logger.info("各聚类数量评估指标详情:")
            for row in evaluation_table:
                self.logger.info(f"  k={row['聚类数']}: 惯性={row['惯性值']}, 轮廓={row['轮廓系数']}, CH={row['CH指数']}, DB={row['DB指数']}, 样本分布={row['各簇样本数']}")
                
            # 尝试创建图表并保存
            try:
                import matplotlib.pyplot as plt
                from matplotlib.backends.backend_pdf import PdfPages
                
                # 创建多页PDF来保存所有图表
                out_dir = self.output_dir if hasattr(self, 'output_dir') else "."
                pdf_path = os.path.join(out_dir, f"cluster_evaluation_{time.strftime('%Y%m%d_%H%M%S')}.pdf")
                
                with PdfPages(pdf_path) as pdf:
                    # 1. 绘制肘部法则图表
                    plt.figure(figsize=(10, 6))
                    plt.plot(evaluation_results['k_values'], evaluation_results['inertias'], 'bo-')
                    plt.title('肘部法则评估')
                    plt.xlabel('聚类数量')
                    plt.ylabel('惯性(Inertia)')
                    plt.grid(True)
                    pdf.savefig()
                    plt.close()
                    
                    # 2. 绘制轮廓系数图表
                    plt.figure(figsize=(10, 6))
                    valid_silhouette = [(k, score) for k, score in zip(evaluation_results['k_values'], evaluation_results['silhouette_scores']) if score > 0]
                    if valid_silhouette:
                        k_values, scores = zip(*valid_silhouette)
                        plt.plot(k_values, scores, 'ro-')
                        plt.title('轮廓系数评估')
                        plt.xlabel('聚类数量')
                        plt.ylabel('轮廓系数(越高越好)')
                        plt.grid(True)
                    else:
                        plt.text(0.5, 0.5, '无有效轮廓系数数据', ha='center', va='center')
                    pdf.savefig()
                    plt.close()
                    
                    # 3. 绘制CH指数图表
                    plt.figure(figsize=(10, 6))
                    valid_ch = [(k, score) for k, score in zip(evaluation_results['k_values'], evaluation_results['ch_scores']) if score > 0]
                    if valid_ch:
                        k_values, scores = zip(*valid_ch)
                        plt.plot(k_values, scores, 'go-')
                        plt.title('Calinski-Harabasz指数评估')
                        plt.xlabel('聚类数量')
                        plt.ylabel('CH指数(越高越好)')
                        plt.grid(True)
                    else:
                        plt.text(0.5, 0.5, '无有效CH指数数据', ha='center', va='center')
                    pdf.savefig()
                    plt.close()
                    
                    # 4. 绘制DB指数图表
                    plt.figure(figsize=(10, 6))
                    valid_db = [(k, score) for k, score in zip(evaluation_results['k_values'], evaluation_results['db_scores']) if score < float('inf')]
                    if valid_db:
                        k_values, scores = zip(*valid_db)
                        plt.plot(k_values, scores, 'mo-')
                        plt.title('Davies-Bouldin指数评估')
                        plt.xlabel('聚类数量')
                        plt.ylabel('DB指数(越低越好)')
                        plt.grid(True)
                    else:
                        plt.text(0.5, 0.5, '无有效DB指数数据', ha='center', va='center')
                    pdf.savefig()
                    plt.close()
                    
                    # 5. 综合评分图表
                    plt.figure(figsize=(10, 6))
                    plt.plot(evaluation_results['k_values'], evaluation_results['combined_scores'], 'ko-')
                    plt.title('聚类数量综合评分')
                    plt.xlabel('聚类数量')
                    plt.ylabel('综合评分(越高越好)')
                    plt.grid(True)
                    pdf.savefig()
                    plt.close()
                
                self.logger.info(f"聚类评估图表已保存到: {pdf_path}")
            except Exception as e:
                self.logger.warning(f"生成聚类评估图表失败: {str(e)}")
            
            # 确定最佳聚类数量 - 基于不同指标
            # 1. 基于惯性的肘部法则
            inertia_diffs = [evaluation_results['inertias'][i] - evaluation_results['inertias'][i+1] for i in range(len(evaluation_results['inertias'])-1)]
            if len(inertia_diffs) > 1:
                # 找到惯性下降最陡的点作为肘部
                elbow_k = min_clusters + inertia_diffs.index(max(inertia_diffs))
            else:
                elbow_k = min_clusters
                
            # 2. 基于轮廓系数
            valid_scores = [s for s in evaluation_results['silhouette_scores'] if s > 0]
            if valid_scores:
                silhouette_k = min_clusters + evaluation_results['silhouette_scores'].index(max(evaluation_results['silhouette_scores']))
            else:
                silhouette_k = elbow_k
                
            # 3. 基于Calinski-Harabasz指数
            if evaluation_results['ch_scores']:
                ch_k = min_clusters + evaluation_results['ch_scores'].index(max(evaluation_results['ch_scores']))
            else:
                ch_k = elbow_k
                
            # 4. 基于Davies-Bouldin指数
            valid_db = [s for s in evaluation_results['db_scores'] if s < float('inf')]
            if valid_db:
                db_k = min_clusters + evaluation_results['db_scores'].index(min(evaluation_results['db_scores']))
            else:
                db_k = elbow_k
                
            # 5. 基于综合评分
            if evaluation_results['combined_scores']:
                combined_k = min_clusters + evaluation_results['combined_scores'].index(max(evaluation_results['combined_scores']))
            else:
                combined_k = elbow_k
                
            # 综合多种指标投票选择最佳k
            k_votes = {elbow_k: 1}  # 肘部法则初始1票
            
            # 轮廓系数投票(权重2)
            if silhouette_k in k_votes:
                k_votes[silhouette_k] += 2
            else:
                k_votes[silhouette_k] = 2
                
            # CH指数投票(权重1)
            if ch_k in k_votes:
                k_votes[ch_k] += 1
            else:
                k_votes[ch_k] = 1
                
            # DB指数投票(权重1)
            if db_k in k_votes:
                k_votes[db_k] += 1
            else:
                k_votes[db_k] = 1
                
            # 综合得分投票(权重2)
            if combined_k in k_votes:
                k_votes[combined_k] += 2
            else:
                k_votes[combined_k] = 2
            
            # 确定最少聚类数量和最多聚类数量
            # 最少聚类：优先选择较少的聚类数量，至少2个
            min_cluster_candidates = [k for k in k_votes.keys() if k <= 5]
            min_cluster_k = min(min_cluster_candidates) if min_cluster_candidates else 2
            
            # 最多聚类：优先选择轮廓系数和DB指数建议的较大聚类数量
            max_cluster_k = max(silhouette_k, db_k, 10)
            
            # 输出各指标建议的聚类数量
            self.logger.info(f"肘部法则建议的聚类数量: {elbow_k}")
            self.logger.info(f"轮廓系数建议的聚类数量: {silhouette_k}")
            self.logger.info(f"CH指数建议的聚类数量: {ch_k}")
            self.logger.info(f"DB指数建议的聚类数量: {db_k}")
            self.logger.info(f"综合评分建议的聚类数量: {combined_k}")
            self.logger.info(f"投票结果: {k_votes}")
            self.logger.info(f"选定的最少聚类数量: {min_cluster_k} (用于模型训练)")
            self.logger.info(f"选定的最多聚类数量: {max_cluster_k} (用于错误分析)")
            
            # 辅助函数：处理聚类结果
            def process_cluster_results(clusters, kmeans, label):
                # 收集有效样本
                X_valid = [self.X_history[i] for i in valid_indices]
                y_valid = [self.y_history[i] for i in valid_indices]
                detailed_errors_valid = [self.detailed_errors[i] for i in valid_indices]
                
                # 检查聚类结果 - 确保至少有2个不同的聚类标签
                unique_labels = set(clusters)
                if len(unique_labels) < 2:
                    self.logger.error(f"{label}聚类失败: 只产生了 {len(unique_labels)} 个聚类")
                    # 强制分配到不同的簇
                    self.logger.info(f"尝试手动分配{label}聚类到不同的簇 - 根据错误值大小将样本分为高/低错误两组")
                    # 根据错误值大小将样本分为两组
                    y_array = np.array(y_valid)
                    median_error = np.median(y_array)
                    clusters = np.where(y_array > median_error, 1, 0)
                    unique_labels = set(clusters)
                    
                    # 统计手动分配结果
                    high_count = np.sum(clusters == 1)
                    low_count = np.sum(clusters == 0)
                    self.logger.info(f"{label}手动分配结果: 高错误组 {high_count} 个样本, 低错误组 {low_count} 个样本")
                    self.logger.info(f"{label}高错误组平均错误: {np.mean(y_array[clusters == 1]):.4f}, 低错误组平均错误: {np.mean(y_array[clusters == 0]):.4f}")
                    
                    if len(unique_labels) < 2:
                        self.logger.error(f"{label}手动分配聚类也失败，无法继续分析")
                        return None
                
                # 分析每个簇的特征，发现不同类型的bug
                bug_clusters = []
                for i in sorted(unique_labels):
                    mask = clusters == i
                    cluster_indices = np.where(mask)[0]
                    
                    if len(cluster_indices) == 0:
                        continue
                        
                    # 获取该簇的样本
                    cluster_X = [X_valid[j] for j in cluster_indices]
                    cluster_y = [y_valid[j] for j in cluster_indices]
                    cluster_errors = [detailed_errors_valid[j] for j in cluster_indices]
                    
                    # 获取参数空间中的点
                    cluster_points = []
                    for x in cluster_X:
                        if isinstance(x, dict):
                            point = [x.get(dim.name, 0) for dim in self.space]
                        else:
                            point = x
                        cluster_points.append(point)
                    
                    cluster_points = np.array(cluster_points)
                    
                    # 1. 分析每个簇的主要错误类型
                    error_types = ['position_error', 'velocity_error', 'acceleration_error']
                    error_counts = {typ: 0 for typ in error_types}
                    error_means = {typ: 0 for typ in error_types}
                    
                    for errors in cluster_errors:
                        for typ in error_types:
                            if typ in errors and errors[typ]:
                                err_values = errors[typ]
                                if isinstance(err_values, list) and len(err_values) > 0:
                                    error_counts[typ] += 1
                                    error_means[typ] += np.mean(err_values)
                    
                    # 找出主要错误类型
                    main_error_type = max(error_counts, key=error_counts.get)
                    if error_counts[main_error_type] > 0:
                        error_means[main_error_type] /= error_counts[main_error_type]
                    
                    # 2. 错误特征分析
                    error_characteristics = {}
                    for typ in error_types:
                        all_errors = []
                        all_slopes = []
                        all_flucts = []
                        
                        for errors in cluster_errors:
                            if typ in errors and errors[typ]:
                                err_values = errors[typ]
                                if isinstance(err_values, list) and len(err_values) > 5:
                                    all_errors.append(np.mean(err_values))
                                    slope, fluct = self._compute_trend_features(err_values)
                                    all_slopes.append(slope)
                                    all_flucts.append(fluct)
                        
                        if all_errors:
                            error_characteristics[typ] = {
                                'mean_error': np.mean(all_errors),
                                'mean_slope': np.mean(all_slopes) if all_slopes else 0,
                                'mean_fluctuation': np.mean(all_flucts) if all_flucts else 0
                            }
                    
                    # 3. 特征重要性分析
                    from sklearn.ensemble import RandomForestRegressor
                    
                    # 获取该簇的特征数据
                    cluster_features = X_processed[mask]
                    feature_importance = {}
                    
                    try:
                        # 对每种错误类型训练一个随机森林
                        for typ in error_types:
                            if typ in error_characteristics:
                                # 提取该错误类型的目标值
                                target_values = []
                                for errors in cluster_errors:
                                    if typ in errors and errors[typ]:
                                        target_values.append(np.mean(errors[typ]))
                                    else:
                                        target_values.append(0)
                                
                                if len(set(target_values)) > 1:  # 确保目标值不全相同
                                    rf = RandomForestRegressor(n_estimators=50, max_depth=5, random_state=42)
                                    rf.fit(cluster_features, target_values)
                                    
                                    # 记录特征重要性
                                    feature_importance[typ] = dict(zip(feature_names, rf.feature_importances_))
                    except Exception as e:
                        self.logger.warning(f"计算特征重要性失败: {e}")
                    
                    # 4. 关键参数分析
                    key_params = []
                    try:
                        # 计算参数与错误的相关性
                        for j, dim in enumerate(self.space):
                            param_values = cluster_points[:, j]
                            
                            # 检查该维度是否有变化
                            if len(set(param_values)) <= 1:
                                continue
                            
                            # 计算与错误值的相关性
                            correlation = np.corrcoef(param_values, cluster_y)[0, 1]
                            if np.isnan(correlation):
                                correlation = 0
                            
                            # 记录参数范围
                            param_range = (float(np.min(param_values)), float(np.max(param_values)))
                            
                            # 判断是否为关键参数
                            if abs(correlation) > 0.2 or (main_error_type in feature_importance and 
                                                        dim.name in feature_importance[main_error_type] and 
                                                        feature_importance[main_error_type][dim.name] > 0.05):
                                key_params.append((dim.name, param_range, float(correlation)))
                    except Exception as e:
                        self.logger.warning(f"分析关键参数失败: {e}")
                    
                    # 5. 整理该簇的特征
                    cluster_info = {
                        'size': len(cluster_indices),
                        'mean_error': float(np.mean(cluster_y)),
                        'max_error': float(np.max(cluster_y)),
                        'main_error_type': main_error_type,
                        'error_characteristics': error_characteristics,
                        'key_params': key_params,
                        'point_indices': [valid_indices[j] for j in cluster_indices],
                        'points': cluster_points.tolist()  # 保存原始参数点
                    }
                    
                    # 添加特征重要性
                    key_features = []
                    if main_error_type in feature_importance:
                        sorted_features = sorted(
                            feature_importance[main_error_type].items(), 
                            key=lambda x: x[1], 
                            reverse=True
                        )
                        key_features = [(f[0], float(f[1])) for f in sorted_features[:5] if f[1] > 0.05]
                    
                    cluster_info['key_features'] = key_features
                    
                    # 将聚类中心添加到信息中
                    if hasattr(kmeans, 'cluster_centers_'):
                        # 每个簇的中心在降维空间
                        cluster_center = kmeans.cluster_centers_[i]
                        
                        # 如果进行了降维，需要将中心从降维空间转回原始空间
                        if X_processed.shape[1] > 15:
                            # 近似还原 - 注意这是一个近似值
                            # 中心点不能完美还原，但可以得到一个合理的估计
                            cluster_center_original = np.mean(X_processed[mask], axis=0)
                        else:
                            cluster_center_original = cluster_center
                        
                        # 保存聚类中心
                        cluster_info['center'] = cluster_center_original.tolist()
                    else:
                        # 如果没有cluster_centers_属性，使用簇内点的平均值
                        cluster_info['center'] = np.mean(X_processed[mask], axis=0).tolist()
                    
                    bug_clusters.append(cluster_info)
                
                # 保存聚类中心用于后续分析
                cluster_centers = []
                if hasattr(kmeans, 'cluster_centers_'):
                    if X_processed.shape[1] > 15:
                        # 降维情况下，计算每个簇的平均特征作为中心
                        for i in sorted(unique_labels):
                            mask = clusters == i
                            if np.any(mask):
                                center = np.mean(X_processed[mask], axis=0)
                                cluster_centers.append(center)
                    else:
                        cluster_centers = kmeans.cluster_centers_
                else:
                    # 如果没有中心，计算簇内点的平均值
                    for i in sorted(unique_labels):
                        mask = clusters == i
                        if np.any(mask):
                            center = np.mean(X_processed[mask], axis=0)
                            cluster_centers.append(center)
                            
                return {
                    'bug_clusters': bug_clusters,
                    'cluster_centers': cluster_centers,
                    'clusters': clusters,
                    'unique_labels': unique_labels
                }
                
            # 存储不同粒度的聚类结果
            cluster_results = {}
            
            # 1. 执行最少聚类（用于模型训练）
            self.logger.info(f"执行最少聚类 (k={min_cluster_k}) 用于模型训练...")
            min_kmeans = KMeans(n_clusters=min_cluster_k, random_state=42, n_init=10, max_iter=300)
            min_clusters = min_kmeans.fit_predict(X_for_cluster)
            min_results = process_cluster_results(min_clusters, min_kmeans, "最少")
            
            if min_results:
                cluster_results['min'] = min_results
                self.cluster_stats_min = min_results['bug_clusters']
                self.cluster_centers_min = min_results['cluster_centers']
                
                # 默认使用最少聚类作为主要结果
                self.cluster_stats = self.cluster_stats_min
                self.cluster_centers = self.cluster_centers_min
                
                # 显示最少聚类结果
                self.logger.info("\n==================================================")
                self.logger.info("最少聚类分析结果摘要 (用于模型训练)")
                self.logger.info("==================================================")
                
                for i, cluster in enumerate(self.cluster_stats_min):
                    self.logger.info(f"\n聚类 {i+1}: {cluster['size']}个样本, '{cluster['main_error_type']}', 均值{cluster['mean_error']:.4f}, 最大值{cluster['max_error']:.4f}")
                    if cluster['key_params']:
                        key_param_names = ", ".join([p[0] for p in cluster['key_params']])
                        self.logger.info(f"关键参数: {key_param_names}")
            
            # 2. 如果需要，执行最多聚类（用于错误分析）
            if save_multi_level:
                self.logger.info(f"\n执行最多聚类 (k={max_cluster_k}) 用于细粒度bug分析...")
                max_kmeans = KMeans(n_clusters=max_cluster_k, random_state=42, n_init=10, max_iter=300)
                max_clusters = max_kmeans.fit_predict(X_for_cluster)
                max_results = process_cluster_results(max_clusters, max_kmeans, "最多")
                
                if max_results:
                    cluster_results['max'] = max_results
                    self.cluster_stats_max = max_results['bug_clusters']
                    self.cluster_centers_max = max_results['cluster_centers']
                    
                    # 显示最多聚类结果
                    self.logger.info("\n==================================================")
                    self.logger.info("最多聚类分析结果摘要 (用于错误分析)")
                    self.logger.info("==================================================")
                    
                    for i, cluster in enumerate(self.cluster_stats_max):
                        self.logger.info(f"\n聚类 {i+1}: {cluster['size']}个样本, '{cluster['main_error_type']}', 均值{cluster['mean_error']:.4f}, 最大值{cluster['max_error']:.4f}")
                        if cluster['key_params']:
                            key_param_names = ", ".join([p[0] for p in cluster['key_params']])
                            self.logger.info(f"关键参数: {key_param_names}")
            
            # 显示聚类结果和簇间差异
            self._display_cluster_analysis()
            
            # 增加敏感边界分析
            sensitive_boundaries = self._identify_sensitive_boundaries(self.cluster_stats)
            
            # 保存敏感边界信息
            if sensitive_boundaries:
                self.sensitive_boundaries = sensitive_boundaries
                boundaries_file = os.path.join(
                    self.output_dir if hasattr(self, 'output_dir') else ".", 
                    f"sensitive_boundaries_{time.strftime('%Y%m%d_%H%M%S')}.json"
                )
                with open(boundaries_file, 'w') as f:
                    json.dump(self._convert_to_serializable(sensitive_boundaries), f, indent=2)
                self.logger.info(f"敏感边界信息已保存到: {boundaries_file}")
            
            # 保存聚类分析数据
            self._save_cluster_analysis_data({
                'min_clusters': self.cluster_stats_min if hasattr(self, 'cluster_stats_min') else None,
                'max_clusters': self.cluster_stats_max if hasattr(self, 'cluster_stats_max') else None,
                'sensitive_boundaries': sensitive_boundaries if 'sensitive_boundaries' in locals() else None,
                'evaluation_metrics': {
                    'k_values': evaluation_results['k_values'],
                    'inertias': [float(x) for x in evaluation_results['inertias']],
                    'silhouette_scores': [float(x) if x > 0 else 0 for x in evaluation_results['silhouette_scores']],
                    'ch_scores': [float(x) if x > 0 else 0 for x in evaluation_results['ch_scores']],
                    'db_scores': [float(x) if x < float('inf') else 0 for x in evaluation_results['db_scores']],
                    'combined_scores': [float(x) for x in evaluation_results['combined_scores']]
                }
            })
            
            # 返回聚类统计信息
            return cluster_results
            
        except Exception as e:
            self.logger.error(f"聚类分析失败: {str(e)}")
            self.logger.exception("详细错误:")
            return None
    
  
    
    def _save_cluster_analysis_data(self, analysis_results=None):
        """Save cluster analysis data to JSON file"""
        try:
            # Ensure output directory exists
            out_dir = self.output_dir if hasattr(self, 'output_dir') else "."
            
            # Build timestamp and directory name
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            detail_dir = os.path.join(out_dir, f"cluster_analysis_{timestamp}")
            os.makedirs(detail_dir, exist_ok=True)
            
            # Use current instance data if no results provided
            if analysis_results is None:
                analysis_results = {
                    'min_cluster_labels': self.min_cluster_labels,
                    'min_cluster_centers': self.min_cluster_centers.tolist() if hasattr(self, 'min_cluster_centers') else [],
                    'min_n_clusters': self.min_n_clusters,
                    'max_cluster_labels': self.max_cluster_labels,
                    'max_cluster_centers': self.max_cluster_centers.tolist() if hasattr(self, 'max_cluster_centers') else [],
                    'max_n_clusters': self.max_n_clusters,
                    'evaluation_results': self.evaluation_results if hasattr(self, 'evaluation_results') else {},
                    'features': self.features.tolist() if hasattr(self, 'features') else [],
                    'feature_names': self.feature_names if hasattr(self, 'feature_names') else [],
                    'timestamps': self.timestamps if hasattr(self, 'timestamps') else [],
                    'X_history': self.X_history if hasattr(self, 'X_history') else [],
                    'y_history': self.y_history if hasattr(self, 'y_history') else [],
                    'sample_indices': self.sample_indices if hasattr(self, 'sample_indices') else []
                }
            
            # Save all samples to CSV
            all_samples_file = os.path.join(detail_dir, "all_samples.csv")
            self._save_all_samples_to_csv(all_samples_file)
            
            # Save detailed data for minimum clusters
            for i in range(self.min_n_clusters):
                self._save_cluster_details(i, "min", detail_dir)
            
            # Save detailed data for maximum clusters
            for i in range(self.max_n_clusters):
                self._save_cluster_details(i, "max", detail_dir)
            
            # Save relationship between clusters
            self._save_cluster_relations(detail_dir)
            
            # Try to create evaluation charts and save
            try:
                if hasattr(self, 'evaluation_results') and self.evaluation_results:
                    import matplotlib.pyplot as plt
                    from matplotlib.backends.backend_pdf import PdfPages
                    
                    # Save PDF in the cluster_analysis folder
                    pdf_path = os.path.join(detail_dir, f"cluster_evaluation.pdf")
                    
                    with PdfPages(pdf_path) as pdf:
                        evaluation_results = self.evaluation_results
                        
                        # 1. Draw elbow method chart
                        plt.figure(figsize=(10, 6))
                        plt.plot(evaluation_results['k_values'], evaluation_results['inertias'], 'bo-')
                        plt.title('Elbow Method Evaluation')
                        plt.xlabel('Number of Clusters')
                        plt.ylabel('Inertia')
                        plt.grid(True)
                        pdf.savefig()
                        plt.close()
                        
                        # 2. Draw silhouette score chart
                        plt.figure(figsize=(10, 6))
                        plt.plot(evaluation_results['k_values'], evaluation_results['silhouette_scores'], 'go-')
                        plt.title('Silhouette Score Evaluation')
                        plt.xlabel('Number of Clusters')
                        plt.ylabel('Silhouette Score')
                        plt.grid(True)
                        pdf.savefig()
                        plt.close()
                        
                        # 3. Draw Calinski-Harabasz index chart
                        plt.figure(figsize=(10, 6))
                        plt.plot(evaluation_results['k_values'], evaluation_results['calinski_harabasz_scores'], 'ro-')
                        plt.title('Calinski-Harabasz Index Evaluation')
                        plt.xlabel('Number of Clusters')
                        plt.ylabel('Calinski-Harabasz Index')
                        plt.grid(True)
                        pdf.savefig()
                        plt.close()
                        
                        # 4. Draw Davies-Bouldin index chart
                        plt.figure(figsize=(10, 6))
                        plt.plot(evaluation_results['k_values'], evaluation_results['davies_bouldin_scores'], 'mo-')
                        plt.title('Davies-Bouldin Index Evaluation')
                        plt.xlabel('Number of Clusters')
                        plt.ylabel('Davies-Bouldin Index')
                        plt.grid(True)
                        pdf.savefig()
                        plt.close()
                        
                        # 5. Draw combined score chart
                        plt.figure(figsize=(10, 6))
                        plt.plot(evaluation_results['k_values'], evaluation_results['combined_scores'], 'ko-')
                        plt.title('Cluster Count Combined Score')
                        plt.xlabel('Number of Clusters')
                        plt.ylabel('Combined Score')
                        plt.grid(True)
                        pdf.savefig()
                        plt.close()
                        
                        # 6. If PCA visualization exists
                        if 'pca_data' in evaluation_results and evaluation_results['pca_data']:
                            pca_data = evaluation_results['pca_data']
                            best_k = evaluation_results.get('best_k', self.min_n_clusters)
                            
                            plt.figure(figsize=(12, 8))
                            scatter = plt.scatter(pca_data[:, 0], pca_data[:, 1], 
                                                c=self.min_cluster_labels, 
                                                cmap='viridis', 
                                                s=50, 
                                                alpha=0.7)
                            plt.colorbar(scatter, label=f'Cluster (k={best_k})')
                            plt.title('PCA Visualization of Clusters')
                            plt.xlabel('Principal Component 1')
                            plt.ylabel('Principal Component 2')
                            plt.grid(True)
                            pdf.savefig()
                            plt.close()
                    
                    self.logger.info(f"Saved cluster evaluation charts to {pdf_path}")
            except Exception as e:
                self.logger.warning(f"Could not generate evaluation charts: {str(e)}")
            
            # Convert data to serializable format and save
            filename = os.path.join(detail_dir, f"cluster_analysis.json")
            with open(filename, 'w') as f:
                json.dump(self._convert_to_serializable(analysis_results), f, indent=2)
                
            self.logger.info(f"Cluster analysis data saved to: {filename}")
            self.logger.info(f"Detailed cluster data saved to: {detail_dir}")
            return filename
            
        except Exception as e:
            self.logger.error(f"Failed to save cluster analysis data: {str(e)}")
            return None


    def _save_cluster_details(self, clusters, output_dir, prefix=""):
        """Save detailed information for each cluster"""
        import pandas as pd
        import matplotlib.pyplot as plt
        
        prefix_str = f"{prefix}_" if prefix else ""
        
        for i, cluster in enumerate(clusters):
            cluster_id = i + 1
            
            # 1. Save samples to CSV
            samples = []
            for idx in cluster['point_indices']:
                if idx >= len(self.X_history) or idx >= len(self.y_history):
                    continue
                    
                sample = {}
                # Add parameter values
                if isinstance(self.X_history[idx], dict):
                    for dim in self.space:
                        sample[dim.name] = self.X_history[idx].get(dim.name, 0)
                else:
                    for j, dim in enumerate(self.space):
                        sample[dim.name] = self.X_history[idx][j] if j < len(self.X_history[idx]) else 0
                
                # Add error value
                sample['error'] = self.y_history[idx]
                
                samples.append(sample)
            
            if samples:
                df = pd.DataFrame(samples)
                csv_path = os.path.join(output_dir, f"{prefix_str}cluster_{cluster_id}_samples.csv")
                df.to_csv(csv_path, index=False)
                
            # 2. Draw error histogram
            errors = [self.y_history[idx] for idx in cluster['point_indices'] 
                    if idx < len(self.y_history)]
            
            if errors:
                plt.figure(figsize=(8, 6))
                plt.hist(errors, bins=20, alpha=0.7)
                plt.title(f"{prefix_str.upper()} Cluster {cluster_id} Error Distribution (Samples: {len(errors)})")
                plt.xlabel('Error Value')
                plt.ylabel('Sample Count')
                plt.grid(True, alpha=0.3)
                
                # Add statistics
                mean_err = np.mean(errors)
                std_err = np.std(errors)
                plt.axvline(mean_err, color='r', linestyle='--', label=f'Mean: {mean_err:.4f}')
                plt.text(0.05, 0.95, f'Mean: {mean_err:.4f}\nStd Dev: {std_err:.4f}', 
                        transform=plt.gca().transAxes, va='top')
                
                plt.savefig(os.path.join(output_dir, f"{prefix_str}cluster_{cluster_id}_error_hist.png"))
                plt.close()
            
            
            # 3. 新增：绘制详细的多维度误差直方图
            self.save_detailed_error_histograms(cluster_id, cluster['point_indices'], output_dir, prefix_str)
                
                
            # 3. Save key parameters summary
            if cluster['key_params']:
                with open(os.path.join(output_dir, f"{prefix_str}cluster_{cluster_id}_key_params.txt"), 'w') as f:
                    f.write(f"CLUSTER {cluster_id} KEY PARAMETERS\n")
                    f.write(f"-------------------------------\n")
                    f.write(f"Samples: {cluster['size']}\n")
                    f.write(f"Mean error: {cluster['mean_error']:.4f}\n")
                    f.write(f"Max error: {cluster['max_error']:.4f}\n\n")
                    f.write(f"KEY PARAMETERS AND CORRELATION:\n")
                    
                    for param_name, param_range, correlation in cluster['key_params']:
                        f.write(f"- {param_name}: range [{param_range[0]:.4f}, {param_range[1]:.4f}], correlation: {correlation:.4f}\n")



    def save_detailed_error_histograms(self, cluster_id, point_indices, output_dir, prefix=""):
        """Save histograms for different error dimensions"""
        import matplotlib.pyplot as plt
        import numpy as np
        
        # 收集每种误差类型
        pos_errors = []
        vel_errors = []
        acc_errors = []
        
        for idx in point_indices:
            if idx >= len(self.detailed_errors):
                continue
            errors = self.detailed_errors[idx]
            if 'position_error' in errors and errors['position_error']:
                pos_errors.append(np.mean(errors['position_error']))
            if 'velocity_error' in errors and errors['velocity_error']:
                vel_errors.append(np.mean(errors['velocity_error']))
            if 'acceleration_error' in errors and errors['acceleration_error']:
                acc_errors.append(np.mean(errors['acceleration_error']))
        
        # 确保有足够的数据点
        if not (pos_errors and vel_errors and acc_errors):
            self.logger.warning(f"Insufficient data for detailed error histograms in cluster {cluster_id}")
            return
        
        # 创建子图
        fig, axes = plt.subplots(3, 1, figsize=(8, 12), tight_layout=True)
        
        # 位置误差图
        axes[0].hist(pos_errors, bins=20, color='blue', alpha=0.7)
        axes[0].set_title(f'Cluster {cluster_id} Position Error Distribution')
        axes[0].axvline(np.mean(pos_errors), color='r', linestyle='--')
        axes[0].text(0.05, 0.95, f'Mean: {np.mean(pos_errors):.4f}\nStd: {np.std(pos_errors):.4f}', 
                    transform=axes[0].transAxes, va='top')
        
        # 速度误差图
        axes[1].hist(vel_errors, bins=20, color='green', alpha=0.7)
        axes[1].set_title(f'Cluster {cluster_id} Velocity Error Distribution')
        axes[1].axvline(np.mean(vel_errors), color='r', linestyle='--')
        axes[1].text(0.05, 0.95, f'Mean: {np.mean(vel_errors):.4f}\nStd: {np.std(vel_errors):.4f}', 
                    transform=axes[1].transAxes, va='top')
        
        # 加速度误差图
        axes[2].hist(acc_errors, bins=20, color='orange', alpha=0.7) 
        axes[2].set_title(f'Cluster {cluster_id} Acceleration Error Distribution')
        axes[2].axvline(np.mean(acc_errors), color='r', linestyle='--')
        axes[2].text(0.05, 0.95, f'Mean: {np.mean(acc_errors):.4f}\nStd: {np.std(acc_errors):.4f}', 
                    transform=axes[2].transAxes, va='top')
        
        # 设置所有子图的标签
        for ax in axes:
            ax.set_xlabel('Error Value')
            ax.set_ylabel('Count')
            ax.grid(True, alpha=0.3)
        
        # 保存图表
        plt.savefig(os.path.join(output_dir, f"{prefix}cluster_{cluster_id}_detailed_errors.png"))
        plt.close()




    def _save_cluster_relations(self, analysis_results, output_dir):
        """Save relationships between min and max clusters"""
        if not ('min_clusters' in analysis_results and 'max_clusters' in analysis_results):
            return
        
        min_clusters = analysis_results['min_clusters']
        max_clusters = analysis_results['max_clusters']
        
        if not (min_clusters and max_clusters):
            return
        
        # Calculate distribution of max cluster samples across min clusters
        relations = {}
        
        for max_i, max_cluster in enumerate(max_clusters):
            max_indices = set(max_cluster['point_indices'])
            distribution = {}
            
            for min_i, min_cluster in enumerate(min_clusters):
                min_indices = set(min_cluster['point_indices'])
                
                # Calculate intersection size
                overlap = len(max_indices.intersection(min_indices))
                
                if overlap > 0:
                    distribution[f"min_cluster_{min_i+1}"] = {
                        "samples": overlap,
                        "percentage": round(overlap / len(max_indices) * 100, 2)
                    }
            
            relations[f"max_cluster_{max_i+1}"] = {
                "total_samples": len(max_indices),
                "distribution": distribution
            }
        
        # Save to JSON file
        with open(os.path.join(output_dir, "cluster_relations.json"), 'w') as f:
            json.dump(relations, f, indent=2)
            
        # Generate visualization of relationship
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            
            # Create a matrix of overlap percentages
            matrix = np.zeros((len(max_clusters), len(min_clusters)))
            
            for max_i in range(len(max_clusters)):
                max_key = f"max_cluster_{max_i+1}"
                if max_key in relations:
                    for min_key, data in relations[max_key]["distribution"].items():
                        min_i = int(min_key.split('_')[-1]) - 1
                        matrix[max_i, min_i] = data["percentage"]
            
            # Draw heatmap
            plt.figure(figsize=(10, 8))
            plt.imshow(matrix, cmap='viridis', aspect='auto')
            plt.colorbar(label='Overlap Percentage')
            plt.title('Relationship Between Max and Min Clusters')
            plt.xlabel('Min Clusters')
            plt.ylabel('Max Clusters')
            
            # Add text annotations
            for max_i in range(len(max_clusters)):
                for min_i in range(len(min_clusters)):
                    text = f"{matrix[max_i, min_i]:.1f}%" if matrix[max_i, min_i] > 0 else ""
                    plt.text(min_i, max_i, text, ha='center', va='center', 
                            color='white' if matrix[max_i, min_i] > 50 else 'black')
            
            # Set ticks
            plt.xticks(np.arange(len(min_clusters)), [f"Min {i+1}" for i in range(len(min_clusters))])
            plt.yticks(np.arange(len(max_clusters)), [f"Max {i+1}" for i in range(len(max_clusters))])
            
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, "cluster_relationship.png"))
            plt.close()
        except Exception as e:
            self.logger.warning(f"Failed to generate cluster relationship visualization: {str(e)}")


    def _display_cluster_analysis(self):
        """显示聚类分析结果摘要"""
        try:
            # 检查是否有聚类结果
            if not hasattr(self, 'cluster_stats') or not self.cluster_stats:
                self.logger.warning("没有可显示的聚类结果")
                return
                
            self.logger.info("\n==================================================")
            self.logger.info("聚类分析结果摘要")
            self.logger.info("==================================================")
            
            # 显示每个簇的基本信息
            for i, cluster in enumerate(self.cluster_stats):
                self.logger.info(f"\n聚类 {i+1}: {cluster['size']}个样本, '{cluster['main_error_type']}', 均值{cluster['mean_error']:.4f}, 最大值{cluster['max_error']:.4f}")
                if cluster['key_params']:
                    key_param_names = ", ".join([p[0] for p in cluster['key_params']])
                    self.logger.info(f"关键参数: {key_param_names}")
                    
            # 添加对最小/最大聚类结果的比较说明（如果有）
            if hasattr(self, 'cluster_stats_min') and hasattr(self, 'cluster_stats_max'):
                min_count = len(self.cluster_stats_min)
                max_count = len(self.cluster_stats_max)
                
                self.logger.info("\n==================================================")
                self.logger.info(f"聚类比较: {min_count}簇(用于模型) vs {max_count}簇(用于分析)")
                self.logger.info("==================================================")
                self.logger.info(f"最少聚类({min_count}簇)提供更稳定的模型训练基础")
                self.logger.info(f"最多聚类({max_count}簇)提供更细致的错误分析视角")
                
                # 计算覆盖率 - 两种聚类方法找到的错误样本比例
                if hasattr(self, 'X_history') and self.X_history:
                    total_samples = len(self.X_history)
                    min_samples = sum([c['size'] for c in self.cluster_stats_min])
                    max_samples = sum([c['size'] for c in self.cluster_stats_max])
                    
                    self.logger.info(f"最少聚类覆盖率: {min_samples/total_samples:.2%}")
                    self.logger.info(f"最多聚类覆盖率: {max_samples/total_samples:.2%}")
                    
            # 参数空间聚类质量评估
            self._evaluate_cluster_quality_in_parameter_space()
            
            # 错误空间聚类质量评估
            self._evaluate_cluster_quality_in_error_space()
            
            # 参数空间与错误空间比较
            if hasattr(self, '_param_space_quality') and hasattr(self, '_error_space_quality'):
                param_quality = self._param_space_quality
                error_quality = self._error_space_quality
                
                self.logger.info("\n==================================================")
                self.logger.info("【参数空间 vs 错误空间】聚类质量比较")
                self.logger.info("==================================================")
                self.logger.info(f"参数空间聚类质量: {param_quality:.4f} ({self._get_quality_rating(param_quality)})")
                self.logger.info(f"错误空间聚类质量: {error_quality:.4f} ({self._get_quality_rating(error_quality)})")
                
                quality_diff = abs(param_quality - error_quality)
                combined_quality = (param_quality + error_quality) / 2
                
                self.logger.info(f"综合聚类质量: {combined_quality:.4f} (0-1，越高越好)")
                
                consistency_rating = "高度一致" if quality_diff < 0.1 else \
                                    "较为一致" if quality_diff < 0.2 else \
                                    "部分一致" if quality_diff < 0.3 else \
                                    "较大差异" if quality_diff < 0.4 else "显著差异"
                                    
                self.logger.info(f"参数空间与错误空间一致性: {consistency_rating} (差异: {quality_diff:.4f})")
                
                conclusion = "聚类在两个空间的质量相当" if quality_diff < 0.1 else \
                            f"聚类在{'错误' if error_quality > param_quality else '参数'}空间的质量优于{'参数' if error_quality > param_quality else '错误'}空间"
                            
                self.logger.info(f"结论: {conclusion}")
                
        except Exception as e:
            self.logger.error(f"显示聚类分析结果失败: {str(e)}")

    def _evaluate_cluster_quality_in_parameter_space(self):
        """评估参数空间的聚类质量"""
        try:
            from sklearn.metrics import silhouette_score, calinski_harabasz_score, davies_bouldin_score
            
            # 检查是否有聚类结果
            if not hasattr(self, 'cluster_stats') or not self.cluster_stats:
                return
                
            # 准备参数空间数据
            X_param = []
            cluster_labels = []
            
            for i, cluster in enumerate(self.cluster_stats):
                for point in cluster['points']:
                    X_param.append(point)
                    cluster_labels.append(i)
                    
            X_param = np.array(X_param)
            cluster_labels = np.array(cluster_labels)
            
            if len(set(cluster_labels)) < 2:
                self.logger.warning("【X参数空间】聚类标签不足，无法评估质量")
                return
                
            self.logger.info("\n==================================================")
            self.logger.info("【仅参数空间】聚类质量评估")
            self.logger.info("==================================================")
            
            # 计算聚类质量指标
            try:
                sil_score = silhouette_score(X_param, cluster_labels)
                self.logger.info(f"【X参数空间】轮廓系数 (Silhouette Score): {sil_score:.4f} (-1到1，越高越好)")
            except Exception as e:
                self.logger.warning(f"计算参数空间轮廓系数失败: {e}")
                sil_score = -1
                
            try:
                ch_score = calinski_harabasz_score(X_param, cluster_labels)
                self.logger.info(f"【X参数空间】Calinski-Harabasz指数: {ch_score:.2f} (越高越好)")
            except Exception as e:
                self.logger.warning(f"计算参数空间CH指数失败: {e}")
                ch_score = 0
                
            try:
                db_score = davies_bouldin_score(X_param, cluster_labels)
                self.logger.info(f"【X参数空间】Davies-Bouldin指数: {db_score:.4f} (越低越好)")
            except Exception as e:
                self.logger.warning(f"计算参数空间DB指数失败: {e}")
                db_score = float('inf')
                
            # 计算簇间距离
            self.logger.info(f"\n【X参数空间】簇间距离矩阵:")
            
            cluster_centers = [np.mean(np.array([p for p in cluster['points']]), axis=0) for cluster in self.cluster_stats]
            
            for i, center_i in enumerate(cluster_centers):
                distances = []
                for j, center_j in enumerate(cluster_centers):
                    dist = np.sum((center_i - center_j) ** 2)
                    distances.append(f"{dist:.2f}")
                self.logger.info(f"  簇 {i+1}: [{', '.join(distances)}]")
                
            # 综合评分
            # 标准化各指标
            sil_norm = (sil_score + 1) / 2  # 转换到0-1
            
            ch_scores = [ch_score]
            ch_max = max(ch_scores) if ch_scores else 1
            ch_norm = ch_score / ch_max if ch_max > 0 else 0
            
            db_scores = [db_score]
            db_min = min([s for s in db_scores if s < float('inf')]) if db_scores else 0
            db_max = max([s for s in db_scores if s < float('inf')]) if db_scores else 1
            db_range = db_max - db_min
            db_norm = 1 - ((db_score - db_min) / db_range) if db_range > 0 and db_score < float('inf') else 0
            
            # 综合得分 - 权重可调整
            quality_score = 0.5 * sil_norm + 0.3 * ch_norm + 0.2 * db_norm
            self._param_space_quality = quality_score
            
            self.logger.info(f"\n【X参数空间】聚类质量综合评分: {quality_score:.4f} (0-1，越高越好)")
            self.logger.info(f"【X参数空间】聚类质量评级: {self._get_quality_rating(quality_score)}")
            
        except Exception as e:
            self.logger.error(f"评估参数空间聚类质量失败: {str(e)}")

    def _evaluate_cluster_quality_in_error_space(self):
        """评估错误空间的聚类质量"""
        try:
            # 检查是否有聚类结果和错误数据
            if not hasattr(self, 'cluster_stats') or not self.cluster_stats:
                return
                
            self.logger.info("\n==================================================")
            self.logger.info("【仅错误空间】聚类质量评估")
            self.logger.info("==================================================")
            
            # 错误统计信息
            for i, cluster in enumerate(self.cluster_stats):
                # 获取簇内所有样本的错误值
                point_indices = cluster['point_indices']
                error_values = []
                
                for idx in point_indices:
                    if idx < len(self.y_history):
                        error_values.append(self.y_history[idx])
                
                if not error_values:
                    continue
                    
                # 计算基本统计量
                error_mean = np.mean(error_values)
                error_std = np.std(error_values)
                error_cv = abs(error_std / error_mean) if error_mean != 0 else 0
                error_min = np.min(error_values)
                error_max = np.max(error_values)
                
                # 四分位数
                q1 = np.percentile(error_values, 25)
                q2 = np.percentile(error_values, 50)
                q3 = np.percentile(error_values, 75)
                
                self.logger.info(f"簇 {i+1} 错误统计 ({len(error_values)}个样本):")
                self.logger.info(f"  平均值={error_mean:.4f}, 标准差={error_std:.4f}")
                self.logger.info(f"  变异系数={error_cv:.4f}")
                self.logger.info(f"  最小值={error_min:.4f}, 最大值={error_max:.4f}")
                self.logger.info(f"  四分位数: Q1={q1:.4f}, Q2={q2:.4f}, Q3={q3:.4f}")
                
            # 计算簇内一致性
            cluster_consistencies = []
            
            for i, cluster in enumerate(self.cluster_stats):
                point_indices = cluster['point_indices']
                error_values = []
                
                for idx in point_indices:
                    if idx < len(self.y_history):
                        error_values.append(self.y_history[idx])
                
                if not error_values:
                    continue
                    
                # 计算变异系数的倒数作为一致性指标（越小的变异系数表示越高的一致性）
                error_mean = np.mean(error_values)
                error_std = np.std(error_values)
                
                # 归一化一致性分数到0-1区间
                cv = error_std / abs(error_mean) if error_mean != 0 else float('inf')
                consistency = 1 / (1 + cv)  # 变换到0-1区间
                
                cluster_consistencies.append(consistency)
                self.logger.info(f"簇 {i+1} 错误一致性: {consistency:.4f} (0-1，越高越好)")
                
            # 计算簇间错误均值差异
            n_clusters = len(self.cluster_stats)
            error_means = [cluster['mean_error'] for cluster in self.cluster_stats]
            
            self.logger.info(f"\n【Y错误空间】簇内一致性平均得分: {np.mean(cluster_consistencies):.4f} (0-1，越高越好)")
            
            consistency_rating = "高度一致" if np.mean(cluster_consistencies) > 0.8 else \
                                "较为一致" if np.mean(cluster_consistencies) > 0.6 else \
                                "中等一致" if np.mean(cluster_consistencies) > 0.4 else \
                                "较低一致" if np.mean(cluster_consistencies) > 0.2 else "不一致"
                                
            self.logger.info(f"【Y错误空间】簇内一致性评级: {consistency_rating}")
            
            # 簇间差异矩阵
            self.logger.info(f"\n【Y错误空间】簇间错误均值差异矩阵:")
            
            for i, mean_i in enumerate(error_means):
                diffs = []
                for j, mean_j in enumerate(error_means):
                    diff = abs(mean_i - mean_j)
                    diffs.append(f"{diff:.4f}")
                self.logger.info(f"  簇 {i+1}: [{', '.join(diffs)}]")
                
            # 计算错误分布的JS散度
            from scipy.stats import entropy
            
            def js_divergence_from_samples(samples1, samples2, bins=20):
                """计算两组样本之间的JS散度"""
                min_val = min(np.min(samples1), np.min(samples2))
                max_val = max(np.max(samples1), np.max(samples2))
                
                # 设置共同的区间
                bin_edges = np.linspace(min_val, max_val, bins+1)
                
                # 计算直方图
                p, _ = np.histogram(samples1, bins=bin_edges, density=True)
                q, _ = np.histogram(samples2, bins=bin_edges, density=True)
                
                # 添加平滑处理
                p = p + 1e-10
                q = q + 1e-10
                
                # 归一化
                p = p / np.sum(p)
                q = q / np.sum(q)
                
                # 计算JS散度
                m = (p + q) / 2
                js_div = (entropy(p, m) + entropy(q, m)) / 2
                
                return js_div
                
            # 计算所有簇之间的JS散度
            js_matrix = np.zeros((n_clusters, n_clusters))
            
            for i in range(n_clusters):
                for j in range(n_clusters):
                    if i == j:
                        js_matrix[i, j] = 0
                        continue
                        
                    # 获取两个簇的错误值样本
                    errors_i = [self.y_history[idx] for idx in self.cluster_stats[i]['point_indices']]
                    errors_j = [self.y_history[idx] for idx in self.cluster_stats[j]['point_indices']]
                    
                    if len(errors_i) > 0 and len(errors_j) > 0:
                        js_matrix[i, j] = js_divergence_from_samples(errors_i, errors_j)
                    else:
                        js_matrix[i, j] = 0
                        
            # 显示JS散度矩阵
            self.logger.info(f"\n【Y错误空间】错误分布的JS散度矩阵 (越大表示分布差异越大):")
            
            for i in range(n_clusters):
                js_diffs = []
                for j in range(n_clusters):
                    js_diffs.append(f"{js_matrix[i, j]:.4f}")
                self.logger.info(f"  簇 {i+1}: [{', '.join(js_diffs)}]")
                
            # 计算平均JS散度作为簇间差异度量
            # 仅考虑非对角线元素
            non_diag_indices = ~np.eye(n_clusters, dtype=bool)
            mean_js = np.mean(js_matrix[non_diag_indices]) if np.any(non_diag_indices) else 0
            
            self.logger.info(f"\n【Y错误空间】簇间错误分布平均JS散度: {mean_js:.4f} (0-1，越高越好)")
            
            js_diff_rating = "显著差异" if mean_js > 0.8 else \
                            "较大差异" if mean_js > 0.6 else \
                            "中等差异" if mean_js > 0.4 else \
                            "轻微差异" if mean_js > 0.2 else "几乎无差异"
                            
            self.logger.info(f"【Y错误空间】簇间差异性评级: {js_diff_rating}")
            
            # 计算均值变异系数
            mean_var = np.std(error_means) / np.mean(np.abs(error_means)) if np.mean(np.abs(error_means)) != 0 else 0
            
            self.logger.info(f"\n【Y错误空间】簇间错误均值变异系数: {mean_var:.4f}")
            
            mean_diff_rating = "显著差异" if mean_var > 0.8 else \
                            "较大差异" if mean_var > 0.6 else \
                            "中等差异" if mean_var > 0.4 else \
                            "轻微差异" if mean_var > 0.2 else "几乎无差异"
                            
            self.logger.info(f"【Y错误空间】簇间均值差异评级: {mean_diff_rating}")
            
            # 综合评分
            intra_cluster_score = np.mean(cluster_consistencies)  # 簇内一致性
            inter_cluster_score = mean_js  # 簇间差异性
            
            # 综合考虑簇内一致性和簇间差异性
            quality_score = 0.6 * intra_cluster_score + 0.4 * inter_cluster_score
            self._error_space_quality = quality_score
            
            self.logger.info(f"\n【Y错误空间】聚类质量综合评分: {quality_score:.4f} (0-1，越高越好)")
            self.logger.info(f"【Y错误空间】聚类质量评级: {self._get_quality_rating(quality_score)}")
            
            # 如果有原始聚类结果，评估与原聚类的一致性
            if hasattr(self, 'orig_clusters') and self.orig_clusters is not None:
                from sklearn.metrics import adjusted_rand_score
                
                curr_labels = np.zeros(len(self.X_history))
                for i, cluster in enumerate(self.cluster_stats):
                    for idx in cluster['point_indices']:
                        curr_labels[idx] = i
                        
                orig_labels = self.orig_clusters
                
                # 确保标签长度一致
                min_len = min(len(curr_labels), len(orig_labels))
                curr_labels = curr_labels[:min_len]
                orig_labels = orig_labels[:min_len]
                
                try:
                    ari = adjusted_rand_score(orig_labels, curr_labels)
                    
                    self.logger.info(f"\n【Y错误空间】与原聚类的一致性评估:")
                    self.logger.info(f"  调整兰德指数: {ari:.4f} (-1到1，越高表示越一致)")
                    
                    consistency_rating = "高度一致" if ari > 0.8 else \
                                        "较高一致" if ari > 0.6 else \
                                        "中度一致" if ari > 0.4 else \
                                        "低度一致" if ari > 0.2 else "几乎不一致"
                                        
                    self.logger.info(f"  与原聚类一致性评级: {consistency_rating}")
                except Exception as e:
                    self.logger.warning(f"计算与原聚类一致性失败: {e}")
                    
        except Exception as e:
            self.logger.error(f"评估错误空间聚类质量失败: {str(e)}")

    def _get_quality_rating(self, score):
        """根据质量分数获取评级描述"""
        if score > 0.8:
            return "优秀"
        elif score > 0.6:
            return "良好"
        elif score > 0.4:
            return "一般"
        elif score > 0.2:
            return "较差"
        else:
            return "差"
                                
                            
                            
                            
                            

    def _identify_sensitive_boundaries(self, cluster_stats):
        """识别配置相似但错误不同的敏感边界样本
        
        Args:
            cluster_stats: 聚类结果统计信息
            
        Returns:
            敏感边界样本对列表
        """
        self.logger.info("开始分析敏感边界样本...")
        sensitive_pairs = []
        
        # 1. 首先标准化所有参数，确保距离计算公平
        all_points = []
        all_indices = []
        
        for i in range(len(self.X_history)):
            if isinstance(self.X_history[i], dict):
                feature_values = [self.X_history[i].get(dim.name, 0) for dim in self.space]
            else:
                feature_values = self.X_history[i]
            all_points.append(feature_values)
            all_indices.append(i)
        
        all_points = np.array(all_points)
        param_scaler = StandardScaler()
        all_points_scaled = param_scaler.fit_transform(all_points)
        
        # 2. 收集错误特征
        error_features = {}
        for i, error_data in enumerate(self.detailed_errors):
            if error_data:
                features = []
                # 位置误差
                pos_errors = error_data.get('position_error', [])
                features.append(np.mean(pos_errors) if pos_errors else 0)
                # 速度误差
                vel_errors = error_data.get('velocity_error', [])
                features.append(np.mean(vel_errors) if vel_errors else 0)
                # 加速度误差
                acc_errors = error_data.get('acceleration_error', [])
                features.append(np.mean(acc_errors) if acc_errors else 0)
                
                error_features[i] = features
        
        # 标准化错误特征
        error_vectors = [error_features[i] for i in error_features]
        if not error_vectors:
            self.logger.warning("没有有效的错误特征数据，无法分析敏感边界")
            return []
            
        error_scaler = StandardScaler()
        error_scaled = error_scaler.fit_transform(error_vectors)
        error_features_scaled = {idx: error_scaled[i] for i, idx in enumerate(error_features)}
        
        # 3. 跨聚类寻找配置相似但错误不同的样本对
        self.logger.info("寻找跨聚类边界样本...")
        
        # 收集所有点的索引和聚类标签
        all_cluster_points = []
        for cluster_idx, cluster in enumerate(cluster_stats):
            for point_idx in cluster['point_indices']:
                all_cluster_points.append((point_idx, cluster_idx))
        
        
        
        # 设置阈值
        config_similarity_threshold = 0.3  # 配置相似度阈值（越小越相似）
        error_difference_threshold = 0.3   # 错误差异阈值（越大差异越大）
        
        # 在所有样本对中找出符合条件的
        for i in range(len(all_cluster_points)):
            idx1, cluster1 = all_cluster_points[i]
            
            if idx1 not in error_features:
                continue
                
            for j in range(i+1, len(all_cluster_points)):
                idx2, cluster2 = all_cluster_points[j]
                
                if idx2 not in error_features:
                    continue
                    
                # 只关注不同聚类的样本对
                if cluster1 == cluster2:
                    continue
                    
                # 计算配置空间距离
                config_distance = np.sqrt(np.sum((all_points_scaled[all_indices.index(idx1)] - 
                                                all_points_scaled[all_indices.index(idx2)]) ** 2))
                
                # 计算错误空间距离
                error_distance = np.sqrt(np.sum((error_features_scaled[idx1] - 
                                            error_features_scaled[idx2]) ** 2))
                
                # 判断是否为敏感边界：配置相似但错误不同
                if config_distance < config_similarity_threshold and error_distance > error_difference_threshold:
                    # 找到了一个敏感边界样本对
                    original_distance = np.sqrt(np.sum((all_points[all_indices.index(idx1)] - 
                                                    all_points[all_indices.index(idx2)]) ** 2))
                    
                    sensitive_pairs.append({
                        'point1_idx': int(idx1),
                        'point2_idx': int(idx2),
                        'cluster1': int(cluster1),
                        'cluster2': int(cluster2),
                        'config_distance': float(config_distance),
                        'error_distance': float(error_distance),
                        'original_distance': float(original_distance)
                    })
                    
                    self.logger.info(f"发现敏感边界: 点{idx1}(簇{cluster1})和点{idx2}(簇{cluster2})，"
                                f"配置距离={config_distance:.4f}，错误距离={error_distance:.4f}")
        
        # 4. 分析敏感边界的参数差异
        if sensitive_pairs:
            self.logger.info(f"共发现{len(sensitive_pairs)}对敏感边界样本")
            self._analyze_boundary_parameters(sensitive_pairs)
        else:
            self.logger.info("未发现敏感边界样本")
        
        return sensitive_pairs


    def _analyze_boundary_parameters(self, sensitive_pairs):
        """分析敏感边界样本对的参数差异
        
        Args:
            sensitive_pairs: 敏感边界样本对列表
        """
        self.logger.info("分析敏感边界参数差异...")
        
        # 统计每个参数在边界样本中的差异情况
        param_differences = {dim.name: [] for dim in self.space}
        
        for pair in sensitive_pairs:
            idx1 = pair['point1_idx']
            idx2 = pair['point2_idx']
            
            # 提取两个点的参数
            if isinstance(self.X_history[idx1], dict):
                point1 = {dim.name: self.X_history[idx1].get(dim.name, 0) for dim in self.space}
                point2 = {dim.name: self.X_history[idx2].get(dim.name, 0) for dim in self.space}
            else:
                point1 = {dim.name: self.X_history[idx1][i] for i, dim in enumerate(self.space)}
                point2 = {dim.name: self.X_history[idx2][i] for i, dim in enumerate(self.space)}
            
            # 计算每个参数的差异
            for dim in self.space:
                diff = abs(point1[dim.name] - point2[dim.name])
                
                # 标准化差异（基于参数范围）
                if hasattr(dim, 'bounds'):
                    range_size = dim.bounds[1] - dim.bounds[0]
                    if range_size > 0:
                        diff = diff / range_size
                
                param_differences[dim.name].append(diff)
        
        # 找出差异最小的参数（可能不敏感）和差异最大的参数（可能是敏感边界的关键）
        avg_differences = {name: np.mean(diffs) for name, diffs in param_differences.items() if diffs}
        
        if not avg_differences:
            self.logger.warning("无法分析参数差异")
            return
        
        # 排序并输出
        sorted_params = sorted(avg_differences.items(), key=lambda x: x[1])
        
        self.logger.info("边界样本参数差异分析（从小到大排序）:")
        for name, diff in sorted_params:
            sensitivity = "高度稳定" if diff < 0.05 else "中等稳定" if diff < 0.2 else "敏感参数"
            self.logger.info(f"  {name}: 平均差异 {diff:.4f} - {sensitivity}")
        
        # 找出最敏感的参数（差异最大的前3个）
        sensitive_params = [name for name, diff in sorted_params[-3:] if diff > 0.1]
        if sensitive_params:
            self.logger.info(f"最敏感的边界参数: {', '.join(sensitive_params)}")
            
            # 生成这些敏感参数的建议测试范围
            self.logger.info("建议在以下参数范围内进行更细粒度测试:")
            for pair in sensitive_pairs[:min(5, len(sensitive_pairs))]:
                idx1 = pair['point1_idx']
                idx2 = pair['point2_idx']
                
                if isinstance(self.X_history[idx1], dict):
                    point1 = {dim.name: self.X_history[idx1].get(dim.name, 0) for dim in self.space}
                    point2 = {dim.name: self.X_history[idx2].get(dim.name, 0) for dim in self.space}
                else:
                    point1 = {dim.name: self.X_history[idx1][i] for i, dim in enumerate(self.space)}
                    point2 = {dim.name: self.X_history[idx2][i] for i, dim in enumerate(self.space)}
                
                self.logger.info(f"  边界样本对 {idx1}-{idx2}:")
                for param in sensitive_params:
                    val1 = point1[param]
                    val2 = point2[param]
                    self.logger.info(f"    {param}: [{min(val1, val2):.4f}, {max(val1, val2):.4f}]")
  

    def _display_cluster_analysis(self):
        """显示聚类分析结果和簇间差异，采用更可读的格式，分别从参数空间和错误空间进行评估"""
        if not hasattr(self, 'cluster_stats') or not self.cluster_stats:
            self.logger.warning("没有聚类结果，无法显示分析")
            return
            
        self.logger.info("\n" + "="*50)
        self.logger.info("聚类分析结果摘要")
        self.logger.info("="*50)
        
        # 1. 显示每个簇的简洁摘要信息
        for i, cluster in enumerate(self.cluster_stats):
            size = cluster.get('size', 0)
            err_type = cluster.get('main_error_type', '未知')
            mean_err = cluster.get('mean_error', 0)
            max_err = cluster.get('max_error', 0)
            
            self.logger.info(f"\n聚类 {i+1}: {size}个样本, '{err_type}', 均值{mean_err:.4f}, 最大值{max_err:.4f}")
            
            # 只显示最重要的参数（最多3个）
            if 'key_params' in cluster and cluster['key_params']:
                top_params = sorted(cluster['key_params'], key=lambda x: abs(x[2]) if len(x) > 2 else 0, reverse=True)[:3]
                param_str = ", ".join([f"{name}" for name, _, _ in top_params])
                self.logger.info(f"关键参数: {param_str}")
        
        # =============================================
        # 2. 仅基于参数空间(X_history)的评估
        # =============================================
        self.logger.info("\n" + "="*50)
        self.logger.info("【仅参数空间】聚类质量评估")
        self.logger.info("="*50)
        
        # 计算轮廓系数所需的样本点
        try:
            from sklearn.metrics import silhouette_score, calinski_harabasz_score, davies_bouldin_score
            import numpy as np
            
            # 收集所有点和它们的簇分配
            X_all = []
            labels = []
            
            for i, cluster in enumerate(self.cluster_stats):
                if 'point_indices' in cluster:
                    for idx in cluster['point_indices']:
                        if idx < len(self.X_history):
                            if isinstance(self.X_history[idx], dict):
                                x_values = [self.X_history[idx].get(dim.name, 0) for dim in self.space]
                                X_all.append(x_values)
                            else:
                                X_all.append(self.X_history[idx])
                            labels.append(i)
            
            if len(X_all) > 0 and len(set(labels)) > 1:
                X_array = np.array(X_all)
                labels_array = np.array(labels)
                
                # 计算轮廓系数
                try:
                    sil_score = silhouette_score(X_array, labels_array)
                    self.logger.info(f"【X参数空间】轮廓系数 (Silhouette Score): {sil_score:.4f} (-1到1，越高越好)")
                except Exception as e:
                    self.logger.warning(f"计算轮廓系数失败: {e}")
                
                # 计算Calinski-Harabasz指数
                try:
                    ch_score = calinski_harabasz_score(X_array, labels_array)
                    self.logger.info(f"【X参数空间】Calinski-Harabasz指数: {ch_score:.2f} (越高越好)")
                except Exception as e:
                    self.logger.warning(f"计算Calinski-Harabasz指数失败: {e}")
                
                # 计算Davies-Bouldin指数
                try:
                    db_score = davies_bouldin_score(X_array, labels_array)
                    self.logger.info(f"【X参数空间】Davies-Bouldin指数: {db_score:.4f} (越低越好)")
                except Exception as e:
                    self.logger.warning(f"计算Davies-Bouldin指数失败: {e}")
                
                # 参数空间中的簇间距离矩阵
                distances = []
                for i, cluster1 in enumerate(self.cluster_stats):
                    row = []
                    for j, cluster2 in enumerate(self.cluster_stats):
                        if i == j:
                            row.append(0)
                            continue
                        
                        # 如果有中心信息，计算中心间欧氏距离
                        if hasattr(self, 'cluster_centers') and i < len(self.cluster_centers) and j < len(self.cluster_centers):
                            dist = np.sqrt(np.sum((np.array(self.cluster_centers[i]) - np.array(self.cluster_centers[j]))**2))
                            row.append(dist)
                        else:
                            row.append(float('nan'))
                    distances.append(row)
                
                # 输出参数空间中的簇间距离
                self.logger.info("\n【X参数空间】簇间距离矩阵:")
                for i, row in enumerate(distances):
                    dist_str = ", ".join([f"{d:.2f}" if not np.isnan(d) else "NA" for d in row])
                    self.logger.info(f"  簇 {i+1}: [{dist_str}]")
                
                # X空间聚类质量综合评分
                x_combined_score = 0.0
                x_score_count = 0
                
                if 'sil_score' in locals():
                    x_combined_score += (sil_score + 1) / 2  # 转换到0-1区间
                    x_score_count += 1
                    
                if 'db_score' in locals() and not np.isnan(db_score) and db_score > 0:
                    x_combined_score += 1 / (1 + db_score)  # 转换到0-1区间，越低越好
                    x_score_count += 1
                    
                if 'ch_score' in locals() and not np.isnan(ch_score) and ch_score > 0:
                    # 对CH指数做对数变换，避免过大值
                    ch_norm = np.log1p(ch_score) / 10  # 限制在合理范围
                    ch_norm = min(ch_norm, 1.0)  # 上限为1
                    x_combined_score += ch_norm
                    x_score_count += 1
                    
                if x_score_count > 0:
                    x_final_score = x_combined_score / x_score_count
                    self.logger.info(f"\n【X参数空间】聚类质量综合评分: {x_final_score:.4f} (0-1，越高越好)")
                    
                    if x_final_score > 0.7:
                        x_rating = "优秀"
                    elif x_final_score > 0.5:
                        x_rating = "良好"
                    elif x_final_score > 0.3:
                        x_rating = "一般"
                    else:
                        x_rating = "较差"
                        
                    self.logger.info(f"【X参数空间】聚类质量评级: {x_rating}")
        except Exception as e:
            self.logger.warning(f"计算参数空间聚类性能指标时出错: {e}")
        
        # =============================================
        # 3. 仅基于错误空间(y_history)的评估
        # =============================================
        self.logger.info("\n" + "="*50)
        self.logger.info("【仅错误空间】聚类质量评估")
        self.logger.info("="*50)
        
        try:
            import numpy as np
            from scipy.cluster.hierarchy import linkage, fcluster
            from scipy.spatial.distance import pdist
            
            # 收集每个簇的错误值
            cluster_errors = []
            cluster_error_indices = []
            cluster_mean_errors = []
            cluster_std_errors = []
            cluster_sizes = []
            
            for i, cluster in enumerate(self.cluster_stats):
                if 'point_indices' in cluster:
                    indices = cluster['point_indices']
                    cluster_y = []
                    valid_indices = []
                    
                    for idx in indices:
                        if idx < len(self.y_history):
                            cluster_y.append(self.y_history[idx])
                            valid_indices.append(idx)
                    
                    if cluster_y:
                        cluster_errors.append(cluster_y)
                        cluster_error_indices.append(valid_indices)
                        cluster_mean_errors.append(np.mean(cluster_y))
                        cluster_std_errors.append(np.std(cluster_y))
                        cluster_sizes.append(len(cluster_y))
                        
                        self.logger.info(f"簇 {i+1} 错误统计 ({len(cluster_y)}个样本):")
                        self.logger.info(f"  平均值={np.mean(cluster_y):.4f}, 标准差={np.std(cluster_y):.4f}")
                        self.logger.info(f"  变异系数={(np.std(cluster_y)/np.mean(cluster_y)):.4f}")
                        self.logger.info(f"  最小值={np.min(cluster_y):.4f}, 最大值={np.max(cluster_y):.4f}")
                        self.logger.info(f"  四分位数: Q1={np.percentile(cluster_y, 25):.4f}, Q2={np.percentile(cluster_y, 50):.4f}, Q3={np.percentile(cluster_y, 75):.4f}")
            
            # 1. 错误空间中的簇内一致性评估
            y_intra_coherence = []
            
            for i, errors in enumerate(cluster_errors):
                # 计算变异系数
                if len(errors) > 1 and np.mean(errors) != 0:
                    cv = np.std(errors) / abs(np.mean(errors))
                    coherence = 1 / (1 + cv)  # 转换为0-1，越高越好
                    y_intra_coherence.append(coherence)
                    self.logger.info(f"簇 {i+1} 错误一致性: {coherence:.4f} (0-1，越高越好)")
            
            # 错误空间中的簇内一致性平均得分
            if y_intra_coherence:
                y_avg_coherence = np.mean(y_intra_coherence)
                self.logger.info(f"\n【Y错误空间】簇内一致性平均得分: {y_avg_coherence:.4f} (0-1，越高越好)")
                
                if y_avg_coherence > 0.7:
                    y_intra_rating = "高度一致"
                elif y_avg_coherence > 0.5:
                    y_intra_rating = "较为一致"
                elif y_avg_coherence > 0.3:
                    y_intra_rating = "一般一致"
                else:
                    y_intra_rating = "不一致"
                    
                self.logger.info(f"【Y错误空间】簇内一致性评级: {y_intra_rating}")
            
            # 2. 错误空间中的簇间差异性评估
            if len(cluster_mean_errors) > 1:
                # 计算错误均值之间的差异矩阵
                y_distance_matrix = []
                for mean1 in cluster_mean_errors:
                    row = []
                    for mean2 in cluster_mean_errors:
                        dist = abs(mean1 - mean2)
                        row.append(dist)
                    y_distance_matrix.append(row)
                
                # 输出错误空间中的簇间差异矩阵
                self.logger.info("\n【Y错误空间】簇间错误均值差异矩阵:")
                for i, row in enumerate(y_distance_matrix):
                    dist_str = ", ".join([f"{d:.4f}" for d in row])
                    self.logger.info(f"  簇 {i+1}: [{dist_str}]")
                
                # 计算错误分布的KL散度或JS散度
                from scipy.stats import entropy
                from scipy.spatial.distance import jensenshannon
                
                def js_divergence_from_samples(samples1, samples2, bins=20):
                    """计算两组样本之间的JS散度"""
                    min_val = min(min(samples1), min(samples2))
                    max_val = max(max(samples1), max(samples2))
                    bin_range = (min_val, max_val)
                    
                    hist1, bin_edges = np.histogram(samples1, bins=bins, range=bin_range, density=True)
                    hist2, _ = np.histogram(samples2, bins=bins, range=bin_range, density=True)
                    
                    # 避免零值
                    hist1 = hist1 + 1e-10
                    hist2 = hist2 + 1e-10
                    
                    # 归一化
                    hist1 = hist1 / np.sum(hist1)
                    hist2 = hist2 / np.sum(hist2)
                    
                    return jensenshannon(hist1, hist2)
                
                # 计算错误分布之间的JS散度矩阵
                js_matrix = []
                for errors1 in cluster_errors:
                    row = []
                    for errors2 in cluster_errors:
                        if errors1 == errors2:
                            row.append(0)
                        else:
                            try:
                                js = js_divergence_from_samples(errors1, errors2)
                                row.append(js)
                            except Exception as e:
                                row.append(float('nan'))
                    js_matrix.append(row)
                
                # 输出JS散度矩阵
                self.logger.info("\n【Y错误空间】错误分布的JS散度矩阵 (越大表示分布差异越大):")
                for i, row in enumerate(js_matrix):
                    js_str = ", ".join([f"{js:.4f}" if not np.isnan(js) else "NA" for js in row])
                    self.logger.info(f"  簇 {i+1}: [{js_str}]")
                
                # 计算平均JS散度
                flat_js = []
                for i in range(len(js_matrix)):
                    for j in range(len(js_matrix)):
                        if i != j and not np.isnan(js_matrix[i][j]):
                            flat_js.append(js_matrix[i][j])
                
                if flat_js:
                    avg_js = np.mean(flat_js)
                    self.logger.info(f"\n【Y错误空间】簇间错误分布平均JS散度: {avg_js:.4f} (0-1，越高越好)")
                    
                    # JS散度评级
                    if avg_js > 0.5:
                        js_rating = "高度差异"
                    elif avg_js > 0.3:
                        js_rating = "明显差异"
                    elif avg_js > 0.1:
                        js_rating = "轻微差异"
                    else:
                        js_rating = "几乎相同"
                        
                    self.logger.info(f"【Y错误空间】簇间差异性评级: {js_rating}")
                
                # 计算错误均值的变异系数
                if np.mean(cluster_mean_errors) != 0:
                    y_cv = np.std(cluster_mean_errors) / abs(np.mean(cluster_mean_errors))
                    self.logger.info(f"\n【Y错误空间】簇间错误均值变异系数: {y_cv:.4f}")
                    
                    # 变异系数评级
                    if y_cv > 0.5:
                        cv_rating = "高度差异"
                    elif y_cv > 0.3:
                        cv_rating = "显著差异"
                    elif y_cv > 0.1:
                        cv_rating = "轻微差异"
                    else:
                        cv_rating = "几乎相同"
                        
                    self.logger.info(f"【Y错误空间】簇间均值差异评级: {cv_rating}")
                
                # 3. 计算错误空间聚类评分
                
                # 错误空间聚类质量综合评分
                y_combined_score = 0.0
                y_score_count = 0
                
                # 簇内一致性得分
                if 'y_avg_coherence' in locals():
                    y_combined_score += y_avg_coherence
                    y_score_count += 1
                
                # 簇间JS散度得分
                if 'avg_js' in locals():
                    y_combined_score += avg_js
                    y_score_count += 1
                
                # 簇间变异系数得分
                if 'y_cv' in locals():
                    # 限制在0-1范围内，变异系数越大越好
                    cv_score = min(y_cv, 1.0)
                    y_combined_score += cv_score
                    y_score_count += 1
                
                # 计算最终得分
                if y_score_count > 0:
                    y_final_score = y_combined_score / y_score_count
                    self.logger.info(f"\n【Y错误空间】聚类质量综合评分: {y_final_score:.4f} (0-1，越高越好)")
                    
                    if y_final_score > 0.7:
                        y_rating = "优秀"
                    elif y_final_score > 0.5:
                        y_rating = "良好"
                    elif y_final_score > 0.3:
                        y_rating = "一般"
                    else:
                        y_rating = "较差"
                        
                    self.logger.info(f"【Y错误空间】聚类质量评级: {y_rating}")
                
                # 4. 错误空间的验证聚类（仅基于y值进行聚类）
                # 把所有错误值合并
                all_errors = []
                all_indices = []
                for cluster_idx, errors in enumerate(cluster_errors):
                    for i, error in enumerate(errors):
                        all_errors.append(error)
                        all_indices.append(cluster_error_indices[cluster_idx][i])
                
                if len(all_errors) > len(cluster_errors) and len(set(all_errors)) > 1:
                    # 使用错误值本身进行层次聚类
                    try:
                        # 重新聚类
                        y_array = np.array(all_errors).reshape(-1, 1)
                        
                        # 计算距离矩阵
                        y_dist = pdist(y_array)
                        
                        # 层次聚类
                        Z = linkage(y_dist, 'ward')
                        
                        # 切割得到与原聚类数量相同的簇
                        y_labels = fcluster(Z, len(cluster_errors), criterion='maxclust') - 1
                        
                        # 计算原聚类标签
                        original_labels = []
                        for idx, cluster_idx in enumerate(range(len(cluster_errors))):
                            original_labels.extend([cluster_idx] * len(cluster_errors[cluster_idx]))
                        
                        # 计算兰德指数
                        from sklearn.metrics.cluster import adjusted_rand_score
                        rand_score = adjusted_rand_score(original_labels, y_labels)
                        
                        self.logger.info(f"\n【Y错误空间】与原聚类的一致性评估:")
                        self.logger.info(f"  调整兰德指数: {rand_score:.4f} (-1到1，越高表示越一致)")
                        
                        # 兰德指数评级
                        if rand_score > 0.7:
                            rand_rating = "高度一致"
                        elif rand_score > 0.5:
                            rand_rating = "显著一致"
                        elif rand_score > 0.3:
                            rand_rating = "轻微一致"
                        elif rand_score > 0:
                            rand_rating = "低度一致"
                        else:
                            rand_rating = "不一致"
                            
                        self.logger.info(f"  与原聚类一致性评级: {rand_rating}")
                    except Exception as e:
                        self.logger.warning(f"错误空间再聚类失败: {e}")
                
                # 5. 如果同时有X空间和Y空间的评分，综合比较
                if 'x_final_score' in locals() and 'y_final_score' in locals():
                    self.logger.info("\n" + "="*50)
                    self.logger.info("【参数空间 vs 错误空间】聚类质量比较")
                    self.logger.info("="*50)
                    
                    self.logger.info(f"参数空间聚类质量: {x_final_score:.4f} ({x_rating})")
                    self.logger.info(f"错误空间聚类质量: {y_final_score:.4f} ({y_rating})")
                    
                    # 综合得分
                    xy_combined = (x_final_score + y_final_score) / 2
                    self.logger.info(f"综合聚类质量: {xy_combined:.4f} (0-1，越高越好)")
                    
                    # 比较两个空间的得分
                    score_diff = abs(x_final_score - y_final_score)
                    if score_diff > 0.3:
                        space_consistent = "不一致"
                    elif score_diff > 0.2:
                        space_consistent = "轻微一致"
                    elif score_diff > 0.1:
                        space_consistent = "较为一致"
                    else:
                        space_consistent = "高度一致"
                    
                    self.logger.info(f"参数空间与错误空间一致性: {space_consistent} (差异: {score_diff:.4f})")
                    
                    if x_final_score > y_final_score:
                        self.logger.info("结论: 聚类在参数空间的质量优于错误空间")
                        if score_diff > 0.2:
                            self.logger.info("建议: 参数空间聚类结构良好，但错误行为不够一致，考虑细化错误类型或增加样本")
                    else:
                        self.logger.info("结论: 聚类在错误空间的质量优于参数空间")
                        if score_diff > 0.2:
                            self.logger.info("建议: 错误行为分组清晰，但参数空间不够区分，考虑增加更多区分性参数")
                    
                    if 'rand_score' in locals() and rand_score < 0.3:
                        self.logger.info("警告: 参数空间聚类与实际错误行为相关性较低，可能需要重新聚类或调整参数重要性权重")
                        
                                
                                # 在分析结束后保存数据
            
            
            analysis_results = {
                "x_space_score": x_final_score if 'x_final_score' in locals() else None,
                "y_space_score": y_final_score if 'y_final_score' in locals() else None,
                "parameter_space_rating": x_rating if 'x_rating' in locals() else None,
                "error_space_rating": y_rating if 'y_rating' in locals() else None
            }
            
            save_dir = self._save_cluster_analysis_data(analysis_results)
            if save_dir:
                self.logger.info(f"\n聚类分析数据已保存到: {save_dir}")
                self.logger.info("您可以使用这些数据手动排查bug类型和分析聚类结果")
                
                
        except Exception as e:
            self.logger.warning(f"计算错误空间聚类性能指标时出错: {e}")
            self.logger.exception("详细错误:")
            
            
        
        
    def _summarize_statistical_evidence(self, basic_analysis):
        """总结统计分析证据"""
        try:
            evidence = {
                'correlations': [],      # 强相关关系
                'important_features': [], # 重要特征
                'sensitive_params': [],   # 敏感参数
                'threshold_effects': []   # 阈值效应
            }
            
            # 1. 分析相关性
            for param, metrics in basic_analysis['correlation'].items():
                for metric in metrics:
                    corr_value = metrics[metric]['pearson']
                    if abs(corr_value) > 0.5:  # 相关性阈值
                        evidence['correlations'].append({
                            'parameter': param,
                            'metric': metric,
                            'correlation': corr_value,
                            'strength': 'strong' if abs(corr_value) > 0.7 else 'moderate',
                            'direction': 'positive' if corr_value > 0 else 'negative'
                        })
            
            # 2. 分析特征重要性
            for metric, importances in basic_analysis['importance'].items():
                important_params = {
                    param: value for param, value in importances.items() 
                    if value > 0.2  # 重要性阈值
                }
                if important_params:
                    evidence['important_features'].append({
                        'metric': metric,
                        'parameters': important_params,
                        'description': f"对{metric}有显著影响的参数"
                    })
            
            # 3. 分析参数敏感性
            for param, metrics in basic_analysis['sensitivity'].items():
                high_sensitivity = {
                    metric: value for metric, value in metrics.items() 
                    if value > 0.3  # 敏感性阈值
                }
                if high_sensitivity:
                    evidence['sensitive_params'].append({
                        'parameter': param,
                        'sensitive_to': high_sensitivity,
                        'description': f"{param}对多个指标显示高敏感性"
                    })
                
            # 4. 分析阈值效应
            for metric, thresholds in basic_analysis['threshold'].items():
                for param, info in thresholds.items():
                    if info['importance'] > 0.2:  # 阈值重要性阈值
                        evidence['threshold_effects'].append({
                            'parameter': param,
                            'metric': metric,
                            'thresholds': info['thresholds'],
                            'importance': info['importance'],
                            'description': f"{param}在{info['thresholds']}处显示明显的阈值效应"
                        })
            
            # 5. 添加总结性描述
            evidence['summary'] = self._generate_evidence_summary(evidence)
            
            self.logger.info("统计证据总结完成")
            return evidence
            
        except Exception as e:
            self.logger.error(f"统计证据总结失败: {e}")
            self.logger.exception("详细错误信息：")
            return None

    def _generate_evidence_summary(self, evidence):
        """生成证据总结描述"""
        try:
            summary = []
            
            # 总结相关性发现
            if evidence['correlations']:
                strong_corrs = [e for e in evidence['correlations'] if e['strength'] == 'strong']
                if strong_corrs:
                    summary.append(f"发现{len(strong_corrs)}个强相关关系")
            
            # 总结重要特征
            if evidence['important_features']:
                n_important = sum(len(f['parameters']) for f in evidence['important_features'])
                summary.append(f"识别出{n_important}个重要参数")
            
            # 总结敏感参数
            if evidence['sensitive_params']:
                summary.append(f"发现{len(evidence['sensitive_params'])}个高敏感性参数")
            
            # 总结阈值效应
            if evidence['threshold_effects']:
                summary.append(f"发现{len(evidence['threshold_effects'])}个显著的阈值效应")
            
            return summary
            
        except Exception as e:
            self.logger.error(f"生成证据总结失败: {e}")
            return ["证据总结生成失败"]

        
        
    
    def analyze_error_causes(self, cluster_data):
        """分析局部空间中的错误原因"""
        try:
            X = np.array(cluster_data['points'])
            detailed_errors = cluster_data['errors']
            n_samples = len(X)
            
            self.logger.info(f"\n{'='*50}")
            self.logger.info(f"开始分析簇内错误原因 (样本量: {n_samples})")
            self.logger.info(f"{'='*50}\n")
            
            # 1. 基础分析
            self.logger.info("1. 执行基础统计分析...")
            basic_analysis = self._perform_basic_analysis(X, detailed_errors)
            
            # # 2. 因果分析
            # causal_analysis = None
            # min_samples_for_causal = 30
            
            # if n_samples >= min_samples_for_causal:
            #     self.logger.info("\n2. 执行因果分析...")
            #     causal_analysis = self._perform_causal_analysis(X, detailed_errors)
            # else:
            #     self.logger.info(f"\n2. 样本量不足 ({n_samples} < {min_samples_for_causal})，跳过因果分析")
            
            # 3. 生成分析总结
            self.logger.info("\n3. 生成分析总结...")
            summary = self._generate_analysis_summary(basic_analysis,None)
            
            # summary = self._generate_analysis_summary(basic_analysis, causal_analysis)
            
            return {
                'basic_analysis': basic_analysis,
                # 'causal_analysis': causal_analysis,
                'summary': summary  # 添加summary字段
            }
            
        except Exception as e:
            self.logger.error(f"错误原因分析失败: {e}")
            self.logger.exception("详细错误信息：")
            return None

    def _perform_basic_analysis(self, X, detailed_errors):
        """执行基础统计分析"""
        try:
            analysis_results = {}
            
            # 1. 提取错误特征
            self.logger.info("  提取错误特征...")

            error_features = self._extract_error_features(detailed_errors)
            
            for metric in ['position_errors', 'velocity_errors', 'acceleration_errors']:
                self.logger.info(f"\n  分析 {metric}:")
                metric_analysis = {}
                
                # 2. 特征重要性分析
                self.logger.info("    计算特征重要性...")
                importance = self._analyze_feature_importance_for_aspect(X, error_features[metric])
                
                # 确保importance中的值是数值而不是字典
                if importance:
                    self.logger.info("    重要特征:")
                    for feature_name, feature_importance in importance.items():
                        if isinstance(feature_importance, dict):
                            # 如果是字典，取其中的值
                            for param, value in feature_importance.items():
                                self.logger.info(f"      - {param}: {float(value):.4f}")
                        else:

                            self.logger.info(f"      - {feature_name}: {float(feature_importance):.4f}")
                
                # 3. 参数阈值分析
                self.logger.info("    分析参数阈值...")
                thresholds = self._analyze_parameter_thresholds_for_aspect(X, error_features[metric])
                if thresholds:
                    self.logger.info("    关键阈值:")
                    for param, values in thresholds.items():
                        if isinstance(values, dict):
                            for threshold_name, threshold_value in values.items():
                                self.logger.info(f"      - {param} ({threshold_name}): {threshold_value}")
                        else:
                            self.logger.info(f"      - {param}: {values}")
                
                metric_analysis['importance'] = importance
                metric_analysis['thresholds'] = thresholds
                analysis_results[metric] = metric_analysis
            
            return analysis_results
            
        except Exception as e:
            self.logger.error(f"基础分析失败: {e}")
            self.logger.exception("详细错误信息：")
            # 返回空字典而不是None
            return {}
    
    def _generate_analysis_summary(self, basic_analysis, causal_analysis):
        """生成分析总结"""
        try:
            summary_lines = []
            summary_lines.append("错误分析总结")
            summary_lines.append("=" * 30)
            
            # 1. 基础分析总结
            summary_lines.append("\n1. 主要影响因素:")
            for metric, analysis in basic_analysis.items():
                summary_lines.append(f"\n{metric}:")
                
                if 'importance' not in analysis or not analysis['importance']:
                    summary_lines.append("  没有发现显著的影响因素")
                    continue
                    
                # 处理importance字典的不同可能结构
                importance_dict = analysis['importance']
                sorted_features = []
                
                for feature, importance in importance_dict.items():
                    if isinstance(importance, dict):
                        # 如果importance是嵌套字典，处理每个子项
                        for param, value in importance.items():
                            sorted_features.append((f"{feature} ({param})", float(value)))
                    else:
                        # 如果importance是直接的数值
                        sorted_features.append((feature, float(importance)))
                
                # 按重要性值排序
                sorted_features.sort(key=lambda x: x[1], reverse=True)
                
                # 显示top-3重要特征
                for feature, importance in sorted_features[:3]:
                    summary_lines.append(f"  - {feature}: {importance:.4f}")
                    
                    # 显示对应的阈值（如果有）
                    if 'thresholds' in analysis and feature in analysis['thresholds']:
                        thresholds = analysis['thresholds'][feature]
                        if isinstance(thresholds, list):
                            summary_lines.append(f"    阈值: {thresholds}")
                        elif isinstance(thresholds, dict):
                            summary_lines.append("    阈值:")
                            for threshold_name, threshold_value in thresholds.items():
                                summary_lines.append(f"      {threshold_name}: {threshold_value}")
            
            # 2. 因果分析总结（如果有）
            if causal_analysis:
                summary_lines.append("\n2. 因果关系分析:")
                for metric, results in causal_analysis.items():
                    summary_lines.append(f"\n{metric}:")
                    if 'direct_causes' in results:
                        for cause in results['direct_causes'][:3]:  # top-3直接原因
                            summary_lines.append(f"  - {cause['feature']}: {cause['strength']:.4f}")
            
            # 3. 优化建议
            summary_lines.append("\n3. 优化建议:")
            recommendations = self._generate_recommendations(basic_analysis, causal_analysis)
            for rec in recommendations:
                summary_lines.append(f"  - {rec}")
            
            # 输出生成的总结到日志
            self.logger.info("\n生成的分析总结:")
            for line in summary_lines:
                self.logger.info(line)
            
            return "\n".join(summary_lines)
            
        except Exception as e:
            self.logger.error(f"生成分析总结失败: {e}")
            self.logger.exception("详细错误信息：")
            return "分析总结生成失败"

    def _generate_recommendations(self, basic_analysis, causal_analysis):
        """生成优化建议"""
        recommendations = []
        
        try:
            # 基于重要性生成建议
            for metric, analysis in basic_analysis.items():
                if 'importance' not in analysis or not analysis['importance']:
                    continue
                    
                importance_dict = analysis['importance']
                sorted_features = []
                
                # 处理importance字典的不同可能结构
                for feature, importance in importance_dict.items():
                    if isinstance(importance, dict):
                        # 如果importance是嵌套字典，处理每个子项
                        for param, value in importance.items():
                            sorted_features.append((f"{feature} ({param})", float(value)))
                    else:
                        # 如果importance是直接的数值
                        sorted_features.append((feature, float(importance)))
                
                # 按重要性值排序
                sorted_features.sort(key=lambda x: x[1], reverse=True)
                
                # 只关注top-2
                for feature, importance in sorted_features[:2]:
                    if importance > 0.2:  # 重要性阈值
                        thresholds = []
                        if 'thresholds' in analysis and feature in analysis['thresholds']:
                            threshold_data = analysis['thresholds'][feature]
                            if isinstance(threshold_data, list):
                                thresholds = threshold_data
                            elif isinstance(threshold_data, dict):
                                # 合并所有阈值
                                thresholds = [v for values in threshold_data.values() for v in (values if isinstance(values, list) else [values])]
                        
                        if thresholds:
                            recommendations.append(
                                f"调整 {feature} 参数 (重要性: {importance:.3f}), "
                                f"建议范围: {min(thresholds):.2f} - {max(thresholds):.2f}"
                            )
                        else:
                            recommendations.append(
                                f"关注 {feature} 参数 (重要性: {importance:.3f})"
                            )
            
            self.logger.info("\n生成的优化建议:")
            for rec in recommendations:
                self.logger.info(f"  {rec}")
                
        except Exception as e:
            self.logger.error(f"生成优化建议失败: {e}")
            self.logger.exception("详细错误信息：")
        
        return recommendations

    def _save_error_analysis(self, error_analysis_results):
        """保存误差分析结果"""
        try:
            # 确保实验目录存在
            if not hasattr(self, 'exp_dir'):
                self.exp_dir = 'experiment_results'
            os.makedirs(self.exp_dir, exist_ok=True)
            
            # 保存分析结果
            analysis_file = os.path.join(self.exp_dir, 'error_analysis.json')
            
            # 转换结果为可序列化格式
            serializable_results = []
            for result in error_analysis_results:
                serializable_result = {
                    'cluster_id': result['cluster_id'],
                    'analysis': self._convert_to_serializable(result['analysis'])
                }
                serializable_results.append(serializable_result)
            
            with open(analysis_file, 'w') as f:
                json.dump(serializable_results, f, indent=2)
            
            self.logger.info(f"\n误差分析结果已保存到: {analysis_file}")
            
            # 生成并保存分析报告摘要
            summary_file = os.path.join(self.exp_dir, 'error_analysis_summary.txt')
            with open(summary_file, 'w') as f:
                for result in error_analysis_results:
                    f.write(f"\n=== 簇 {result['cluster_id']} 分析结果 ===\n")
                    f.write(result['analysis']['summary'])  # 使用新生成的summary
                    f.write("\n" + "="*50 + "\n")
            
            self.logger.info(f"分析报告摘要已保存到: {summary_file}")
            
        except Exception as e:
            self.logger.error(f"保存分析结果失败: {e}")
            self.logger.exception("详细错误信息：")

    def _perform_additional_sampling(self, X, detailed_errors, n_samples_needed, cluster_bounds):
        """在cluster范围内执行补充采样
        
        Args:
            X: 现有的参数矩阵
            detailed_errors: 现有的误差数据
            n_samples_needed: 需要补充的样本数量
            cluster_bounds: cluster的参数范围
            
        Returns:
            tuple: (新的参数矩阵, 新的误差数据)
        """
        try:
            # 1. 基于现有数据分析，生成新的采样点
             # 打印输入X的维度
            self.logger.info(f"=======================输入X的维度: {X.shape}")
            self.logger.info(f"cluster_bounds包含的参数: {list(cluster_bounds.keys())}")
            
            new_points = []
            new_errors = []
            
            # 计算每个参数的初步相关性
            correlations = self._compute_preliminary_correlations(X, detailed_errors)
            
            # 2. 生成并执行新的实验点
            while len(new_points) < n_samples_needed:
                # 生成新的采样点
                self.logger.info(f"正在生成第{len(new_points)+1}个新的采样点")
                
                new_point = self._generate_new_sample_point(
                    cluster_bounds,
                    correlations,
                    X,
                    new_points
                )
                # 打印生成的新点的维度
                self.logger.info(f"生成的新点包含的参数: {list(new_point.keys())}")
                self.logger.info(f"生成的新点的维度: {len(new_point)}")
                
                
                # 执行实验
                self.logger.info(f"正在执行新的实验点: {new_point}")
                error_result = self._run_experiment(new_point)
                
                if error_result is not None:
                    
                    new_points.append(new_point)
                    new_errors.append(error_result)
                    self.logger.info("实验成功完成")
                else:
                    self.logger.warning("实验失败，将尝试新的点")
            
            return np.array(new_points), new_errors
            
        except Exception as e:
            self.logger.error(f"补充采样失败: {e}")
            return np.array([]), []

    def _generate_new_sample_point(self, param_ranges, correlations, X, new_points):
        """生成新的采样点，考虑已有点的分布和参数重要性"""
        try:
            # 1. 根据相关性确定参数重要性权重
            param_weights = {
                param: abs(corr) + 0.1  # 添加基础权重避免为0
                for param, corr in correlations.items()
            }
            
            # 2. 生成候选点
            n_candidates = 10
            candidates = []
            candidate_scores = []
            
            for _ in range(n_candidates):
                candidate = {}
                for param_name, range_info in param_ranges.items():
                    # 根据参数类型生成值
                    if isinstance(self.space[self.param_names.index(param_name)], Integer):
                        value = np.random.randint(range_info['min'], range_info['max'] + 1)
                    else:
                        value = np.random.uniform(range_info['min'], range_info['max'])
                    candidate[param_name] = value
                
                # 计算候选点的得分
                score = self._evaluate_candidate_point(
                    candidate,
                    X,
                    new_points,
                    param_weights
                )
                
                candidates.append(candidate)
                candidate_scores.append(score)
            
            # 3. 选择最佳候选点
            best_idx = np.argmax(candidate_scores)
            return candidates[best_idx]
            
        except Exception as e:
            self.logger.error(f"生成新采样点失败: {e}")
            return None


            
            

    def _evaluate_candidate_point(self, point, history, candidates, param_weights=None):
        """评估候选点的质量，计算基于多样性和潜在价值的综合分数
        
        Args:
            point: 候选点 (字典或数组格式)
            history: 历史点集合 (字典或数组格式)
            candidates: 所有候选点集合 (字典或数组格式)
            param_weights: 参数权重向量，默认为None
            
        Returns:
            float: 候选点的综合评分，分数越高表示点越好
        """
        try:
            # 统一参数类型处理
            if param_weights is None:
                param_weights = np.ones(len(self.space))
                
            # 将point转换为统一格式(数组)
            if isinstance(point, dict):
                point_array = np.array([point.get(dim.name, 0.0) for dim in self.space])
            else:
                point_array = np.array(point)
                
            # 确保历史点和候选点都是统一格式(数组列表)
            history_arrays = []
            for hist_point in history:
                if isinstance(hist_point, dict):
                    hist_array = np.array([hist_point.get(dim.name, 0.0) for dim in self.space])
                    history_arrays.append(hist_array)
                else:
                    history_arrays.append(np.array(hist_point))
                    
            candidate_arrays = []
            for cand in candidates:
                if isinstance(cand, dict):
                    cand_array = np.array([cand.get(dim.name, 0.0) for dim in self.space])
                    candidate_arrays.append(cand_array)
                else:
                    candidate_arrays.append(np.array(cand))
            
            # 计算多样性分数 - 考虑与历史点的距离
            diversity_score = 0
            if len(history_arrays) > 0:
                # 计算候选点到历史点的最小距离
                min_dist_to_history = float('inf')
                for hist_array in history_arrays:
                    # 使用加权欧氏距离，考虑参数重要性
                    weighted_diff = (point_array - hist_array) * param_weights
                    dist = np.sqrt(np.sum(weighted_diff ** 2))
                    min_dist_to_history = min(min_dist_to_history, dist)
                
                # 规范化距离到0-1范围
                diversity_score = min(1.0, min_dist_to_history / np.sqrt(len(point_array)))
            else:
                diversity_score = 1.0  # 如果没有历史点，给予最高多样性分数
            
            # 计算分散度分数 - 确保候选点之间不会过度拥挤
            dispersion_score = 0
            if len(candidate_arrays) > 1:
                # 计算到其他候选点的平均距离
                distances = []
                for other_point in candidate_arrays:
                    if not np.array_equal(point_array, other_point):
                        weighted_diff = (point_array - other_point) * param_weights
                        dist = np.sqrt(np.sum(weighted_diff ** 2))
                        distances.append(dist)
                
                if distances:
                    # 使用到最近的几个点的平均距离
                    k = min(3, len(distances))
                    avg_dist = np.mean(sorted(distances)[:k])
                    dispersion_score = min(1.0, avg_dist / np.sqrt(len(point_array)))
            else:
                dispersion_score = 1.0  # 如果只有一个候选点，给予最高分散度分数
            
            # 计算利用分数 - 使用高斯过程模型预测的值
            exploitation_score = 0
            if hasattr(self, 'gp') and self.gp is not None and len(history) >= 10:
                try:
                    # 使用高斯过程的下置信界（LCB）
                    mean, std = self.gp.predict(point_array.reshape(1, -1), return_std=True)
                    # LCB = 均值 - 权重 * 标准差，我们寻求最小化LCB
                    # 由于我们要最大化分数，所以取负值
                    beta = 0.5  # 均值和不确定性的权衡系数
                    lcb = mean[0] - beta * std[0]
                    
                    # 规范化到0-1范围，反转使得较低的LCB获得较高的分数
                    # 假设REI值的合理范围在0-1之间
                    exploitation_score = 1.0 - min(1.0, max(0.0, lcb))
                except Exception as e:
                            self.logger.warning(f"计算利用分数出错: {e}")
                            exploitation_score = 0.5  # 出错时使用中等分数
            else:
                exploitation_score = 0.5  # 没有GP模型时使用中等分数
            
            # 计算聚类分数 - 考虑点与不同聚类的关系
            cluster_score = 0
            if hasattr(self, 'cluster_stats') and self.cluster_stats and len(self.cluster_stats) > 1:
                try:
                    cluster_centers = []
                    for cluster in self.cluster_stats:
                        if 'center' in cluster and 'params' in cluster and 'center' in cluster['params']:
                            center_params = cluster['params']['center']
                            if isinstance(center_params, list):
                                cluster_centers.append(np.array(center_params))
                    
                    if cluster_centers:
                        # 计算到每个聚类中心的距离
                        distances_to_centers = []
                        for center in cluster_centers:
                            weighted_diff = (point_array - center) * param_weights
                            dist = np.sqrt(np.sum(weighted_diff ** 2))
                            distances_to_centers.append(dist)
                        
                        # 使用到最近聚类中心的距离，鼓励探索聚类之间的区域
                        min_dist = min(distances_to_centers)
                        second_min_dist = sorted(distances_to_centers)[1] if len(distances_to_centers) > 1 else min_dist
                        
                        # 如果点位于两个聚类之间，给予较高分数
                        if min_dist > 0.1 and min_dist < 0.5 and second_min_dist < 0.8:
                            cluster_score = 0.8
                        else:
                            cluster_score = 0.5
                except Exception as e:
                    self.logger.warning(f"计算聚类分数出错: {e}")
                    cluster_score = 0.5
            else:
                cluster_score = 0.5  # 没有聚类信息时使用中等分数
            
            # 根据调用次数动态调整权重
            n_calls = len(history)
            
            # 在早期阶段更重视多样性，后期更重视利用
            diversity_weight = max(0.2, min(0.8, 1.0 - n_calls / 200))
            exploitation_weight = 1.0 - diversity_weight
            
            # 计算最终分数（加权和）
            final_score = (
                diversity_weight * (0.6 * diversity_score + 0.4 * dispersion_score) +
                exploitation_weight * (0.7 * exploitation_score + 0.3 * cluster_score)
            )
            
            # 添加一小部分随机性，以避免陷入局部最优
            final_score += np.random.uniform(0, 0.05)
            
            return final_score
            
        except Exception as e:
            self.logger.warning(f"评估候选点时出错: {e}")
            # 出错时返回随机分数
            return np.random.uniform(0, 1)


    def _perform_layered_search(self, cluster_stats, max_samples_per_layer=30):
        """执行三层搜索策略
        
        Args:
            cluster_stats: 已识别的聚类统计信息
            max_samples_per_layer: 每层最大采样数
        """
        self.logger.info("\n======== 开始多层次搜索策略 ========")
        
        # 记录原始样本量，用于后续对比
        initial_samples = len(self.X_history)
        
        # 对每个聚类执行Layer 1搜索
        for i, cluster in enumerate(cluster_stats):
            self.logger.info(f"\n----- Layer 1: 聚类{i+1}关键变量组合探索 -----")
            self._perform_layer1_search(cluster, max_samples=max_samples_per_layer)
        
        layer1_samples = len(self.X_history) - initial_samples
        self.logger.info(f"Layer 1完成，新增样本: {layer1_samples}")
        
        # 执行Layer 2搜索：全局交互覆盖
        self.logger.info("\n----- Layer 2: 全局交互覆盖 -----")
        self._perform_layer2_search(cluster_stats, max_samples=max_samples_per_layer)
        
        layer2_samples = len(self.X_history) - initial_samples - layer1_samples
        self.logger.info(f"Layer 2完成，新增样本: {layer2_samples}")
        
        # 执行Layer 3搜索：非关键参数扰动
        self.logger.info("\n----- Layer 3: 非关键参数扰动鲁棒性测试 -----")
        self._perform_layer3_search(cluster_stats, max_samples=max_samples_per_layer)
        
        layer3_samples = len(self.X_history) - initial_samples - layer1_samples - layer2_samples
        self.logger.info(f"Layer 3完成，新增样本: {layer3_samples}")
        
        # 总结多层次搜索结果
        total_new_samples = len(self.X_history) - initial_samples
        self.logger.info(f"\n多层次搜索完成，总计新增样本: {total_new_samples}")
        self.logger.info(f"- Layer 1 (聚类关键变量组合): {layer1_samples}个样本")
        self.logger.info(f"- Layer 2 (全局交互覆盖): {layer2_samples}个样本")
        self.logger.info(f"- Layer 3 (非关键参数扰动): {layer3_samples}个样本")
        
        return total_new_samples

    def _perform_layer1_search(self, cluster, max_samples=10):
        """Layer 1: 聚类内关键变量组合探索
        
        针对特定聚类进行局部重要参数组合探索
        """
        points = np.array(cluster['points'])
        point_indices = cluster['point_indices']
        cluster_y = np.array([self.y_history[idx] for idx in point_indices])
        
        # 提取聚类中心和重要维度
        cluster_center = np.mean(points, axis=0)
        
        # 识别关键维度（使用相关性和方差）
        importance_scores = []
        for j in range(points.shape[1]):
            if np.std(points[:, j]) > 0:
                corr = np.abs(np.corrcoef(points[:, j], cluster_y)[0, 1])
                if np.isnan(corr):
                    corr = 0
                var_norm = np.std(points[:, j]) / (np.max(points[:, j]) - np.min(points[:, j]) + 1e-10)
                importance_scores.append(corr * var_norm)
            else:
                importance_scores.append(0)
        
        # 选择前k个重要维度
        k = min(5, len(importance_scores))
        top_dims = np.argsort(-np.array(importance_scores))[:k]
        
        self.logger.info(f"选择的关键维度: {[self.space[dim_idx].name for dim_idx in top_dims]}")
        
        # 生成网格采样点
        grid_samples = []
        for _ in range(max_samples):
            point = cluster_center.copy()
            
            # 对重要维度使用网格或成对采样
            for dim_idx in top_dims:
                # 从聚类中提取该维度的范围
                dim_min = np.min(points[:, dim_idx])
                dim_max = np.max(points[:, dim_idx])
                
                # 在范围内随机采样
                dim_value = np.random.uniform(dim_min, dim_max)
                point[dim_idx] = dim_value
            
            # 将点转换为字典格式
            point_dict = {self.space[i].name: point[i] for i in range(len(self.space))}
            grid_samples.append(point_dict)
        
        # 执行采样
        for i, point in enumerate(grid_samples):
            self.logger.info(f"Layer 1 - 聚类关键变量探索: 样本 {i+1}/{len(grid_samples)}")
            try:
                result = self._objective_stage1(**point)
                self.logger.info(f"结果: {result}")
            except Exception as e:
                self.logger.error(f"采样出错: {e}")

    def _perform_layer2_search(self, cluster_stats, max_samples=10):
        """Layer 2: 全局交互覆盖
        
        使用结构化采样方法测试全局重要变量的组合
        """
        # 从所有聚类中汇总全局重要维度
        all_important_dims = set()
        
        for cluster in cluster_stats:
            points = np.array(cluster['points'])
            point_indices = cluster['point_indices']
            cluster_y = np.array([self.y_history[idx] for idx in point_indices])
            
            # 计算每个维度的重要性
            for j in range(points.shape[1]):
                if np.std(points[:, j]) > 0:
                    corr = np.abs(np.corrcoef(points[:, j], cluster_y)[0, 1])
                    if not np.isnan(corr) and corr > 0.3:  # 重要性阈值
                        all_important_dims.add(j)
        
        important_dims = list(all_important_dims)
        if len(important_dims) > 8:  # 限制维度数量
            important_dims = important_dims[:8]
        
        self.logger.info(f"全局重要维度: {[self.space[dim_idx].name for dim_idx in important_dims]}")
        
        # 使用拉丁超立方体采样(LHS)生成样本
        try:
            from scipy.stats import qmc
            
            # 创建LHS采样器
            sampler = qmc.LatinHypercube(d=len(important_dims))
            samples = sampler.random(n=max_samples)
            
            # 转换为参数空间
            lhs_points = []
            for sample in samples:
                point = np.zeros(len(self.space))
                
                # 填充所有维度的默认值（使用历史平均值）
                X_history = np.array(self.X_history)
                default_values = np.mean(X_history, axis=0)
                point[:] = default_values
                
                # 更新重要维度
                for i, dim_idx in enumerate(important_dims):
                    dim = self.space[dim_idx]
                    # 将[0,1]范围映射到参数范围
                    point[dim_idx] = dim.low + sample[i] * (dim.high - dim.low)
                
                # 将点转换为字典格式
                point_dict = {self.space[i].name: point[i] for i in range(len(self.space))}
                lhs_points.append(point_dict)
            
            # 执行采样
            for i, point in enumerate(lhs_points):
                self.logger.info(f"Layer 2 - 全局交互覆盖: 样本 {i+1}/{len(lhs_points)}")
                try:
                    result = self._objective_stage1(**point)
                    self.logger.info(f"结果: {result}")
                except Exception as e:
                        self.logger.error(f"采样出错: {e}")
                        
        except ImportError:
            self.logger.warning("scipy.stats.qmc模块不可用，使用简单随机采样代替")
            # 使用简单随机采样
            for i in range(max_samples):
                point = np.zeros(len(self.space))
                
                # 填充所有维度的默认值（使用历史平均值）
                X_history = np.array(self.X_history)
                default_values = np.mean(X_history, axis=0)
                point[:] = default_values
                
                # 更新重要维度
                for dim_idx in important_dims:
                    dim = self.space[dim_idx]
                    point[dim_idx] = np.random.uniform(dim.low, dim.high)
                
                # 将点转换为字典格式
                point_dict = {self.space[i].name: point[i] for i in range(len(self.space))}
                
                self.logger.info(f"Layer 2 - 全局交互覆盖: 样本 {i+1}/{max_samples}")
                try:
                    result = self._objective_stage1(**point)
                    self.logger.info(f"结果: {result}")
                except Exception as e:
                    self.logger.error(f"采样出错: {e}")

    def _perform_layer3_search(self, cluster_stats, max_samples=10):
        """Layer 3: 非关键参数扰动鲁棒性测试
        
        保持关键参数固定，对非关键参数进行小幅扰动
        """
        # 识别全局重要和非重要维度
        X = np.array(self.X_history)
        y = np.array(self.y_history)
        
        # 计算全局重要性
        importance_scores = []
        for j in range(X.shape[1]):
            if np.std(X[:, j]) > 0:
                corr = np.abs(np.corrcoef(X[:, j], y)[0, 1])
                if np.isnan(corr):
                    corr = 0
                importance_scores.append(corr)
            else:
                importance_scores.append(0)
        
        # 区分重要和非重要维度
        median_importance = np.median(importance_scores)
        important_dims = [j for j in range(len(importance_scores)) if importance_scores[j] >= median_importance]
        non_important_dims = [j for j in range(len(importance_scores)) if importance_scores[j] < median_importance]
        
        self.logger.info(f"非关键维度: {[self.space[dim_idx].name for dim_idx in non_important_dims]}")
        
        # 找出历史表现最好的几个点
        best_indices = np.argsort(-y)[:3]  # 取前3个最好的点
        
        # 对每个最佳点生成若干扰动样本
        samples_per_point = max_samples // len(best_indices)
        
        for rank, idx in enumerate(best_indices):
            base_point = X[idx].copy()
            self.logger.info(f"对第{rank+1}好的点进行扰动测试")
            
            for i in range(samples_per_point):
                perturbed_point = base_point.copy()
                
                # 对非重要维度添加随机扰动
                for dim_idx in non_important_dims:
                    dim = self.space[dim_idx]
                    # 添加小幅扰动（范围的5-15%）
                    range_width = dim.high - dim.low
                    perturbation = np.random.uniform(-0.15, 0.15) * range_width
                    perturbed_value = base_point[dim_idx] + perturbation
                    # 确保在合法范围内
                    perturbed_point[dim_idx] = np.clip(perturbed_value, dim.low, dim.high)
                
                # 将点转换为字典格式
                point_dict = {self.space[i].name: perturbed_point[i] for i in range(len(self.space))}
                
                self.logger.info(f"Layer 3 - 非关键参数扰动: 基础点{rank+1}, 样本 {i+1}/{samples_per_point}")
                try:
                    result = self._objective_stage1(**point_dict)
                    self.logger.info(f"结果: {result}")
                except Exception as e:
                    self.logger.error(f"采样出错: {e}")
    
    
    
    
    def optimize(self, max_calls=400, epsilon=0.01, delta=0.01, 
        load_stage1=False, save_stage1=True, min_samples=100, 
        enable_layered_search=True, early_stop=True, max_cluster_analysis=100):
        """多层次贝叶斯优化算法 - 包含预分析和层次化搜索
        
        Args:
            max_calls: 最大函数调用次数
            epsilon: 收敛阈值ef
            delta: UCB不确定性阈值
            load_stage1: 是否加载之前保存的Stage 1数据
            save_stage1: 是否保存Stage 1数据
            min_samples: 最小样本量
            enable_layered_search: 是否启用分层搜索
            early_stop: 是否启用提前停止
            max_cluster_analysis: 最大聚类分析次数
            
        Returns:
            最优点参数
        """
        try:
            self.min_samples = min_samples
            convergence_checks = 0
            cluster_analyses = 0
            
            # 如果请求加载Stage 1数据，尝试加载
            if load_stage1:
                try:
                    self._load_stage1_data()
                    self.logger.info(f"成功加载Stage 1数据，当前样本量: {len(self.X_history)}")
                except Exception as e:
                    self.logger.warning(f"加载Stage 1数据失败: {e}")
                    self.logger.info("将从头开始优化流程")
            
            # 初始化数据结构
            self._initialize_data_structures()
            
            # 初始化参数空间网格
            try:
                self._initialize_space_grid(divisions_per_dim=5)
            except Exception as e:
                self.logger.warning(f"初始化参数空间网格失败: {e}")
            
            # 定义优化阶段
            self.logger.info("\n========= 优化流程开始 =========")
            self.logger.info(f"优化参数: max_calls={max_calls}, epsilon={epsilon}, delta={delta}")
            self.logger.info(f"特殊设置: min_samples={min_samples}, enable_layered_search={enable_layered_search}, early_stop={early_stop}")
            
            #=================================================================
            # Stage 1: 初始采样阶段 - 直到达到最小样本量
            #=================================================================
            current_samples = len(self.X_history)
            self.logger.info(f"当前样本量: {current_samples}, 目标最小样本量: {self.min_samples}")
            
            if current_samples < self.min_samples:
                self.logger.info("\n=== Stage 1: 初始采样阶段 ===")
                self.logger.info(f"当前样本量: {current_samples}")
                self.logger.info(f"目标最小样本量: {self.min_samples}")
                
                
                sampling_start_time = time.time()
                
                for i in range(current_samples, min_samples):  # 至少采样到min_samples数量
                    self.logger.info(f"\n 迭代 {i+1}/{max_calls} (Stage 1)")
                    
                    # 检查资源限制
                    resources = get_system_resources()
                    if resources['memory_percent'] > 85:
                        self.logger.warning(f"系统内存使用率({resources['memory_percent']:.1f}%)过高，暂停优化")
                        break
                        
                    # 生成下一个采样点并执行实验
                    try:
                        # 在初始阶段，直接使用随机采样
                        next_point = self._generate_random_point()
                        self.logger.info(f"随机生成采样点: {next_point}")
                        
                        # 这里的objectIve stage 1 是专门用来执行代码的
                        result = self._objective_stage1(**next_point)
                        self.logger.info(f"实验结果: {result}")
                            
                        # 定期保存数据（每5次迭代）
                        if save_stage1 and (i + 1) % 5 == 0:
                            self._save_stage1_data()
                            
                        # 定期记录优化状态
                        if (i + 1) % 5 == 0:
                            self._log_optimization_state()
                            
                    except Exception as e:
                        self.logger.error(f"迭代{i+1}发生错误: {str(e)}")
                        self.logger.exception("详细错误:")
                        if save_stage1:
                            self._save_stage1_data()  # 出错时也保存当前状态
                        continue
                    
                    # 更新当前样本量
                    current_samples = len(self.X_history)
                    if current_samples >= self.min_samples:
                        break
                        
                    # 检查时间限制 - 避免Stage 1耗时过长
                    elapsed_time = time.time() - sampling_start_time
                    if elapsed_time > 3600 * 3:  # 3小时限制
                        self.logger.warning(f"Stage 1已运行{elapsed_time/3600:.1f}小时，强制进入下一阶段")
                        break
                        
                    # 检查迭代次数限制
                    if i + 1 >= max_calls:
                        self.logger.warning(f"已达到最大调用次数({max_calls})，停止优化")
                        break
            else:
                self.logger.info(f"已有足够样本({current_samples} >= {self.min_samples})，跳过Stage 1")
            
            # 最终保存Stage 1数据
            if save_stage1:
                self._save_stage1_data()

            #=================================================================
            # Stage 2: 聚类分析阶段  
            #=================================================================
            self.logger.info("\n=== Stage 2: 聚类分析阶段 ===")
            
            # 检查是否有足够的样本进行聚类分析
            current_samples = len(self.X_history)
            cluster_stats = None
            multi_gp_trained = False  # 跟踪多模型训练状态
            
            if current_samples >= max(20, self.min_samples * 0.2):  # 或者较低的阈值
                try:

                    cluster_analyses += 1
                    self.logger.info(f"执行第{cluster_analyses}次聚类分析")
                    
                    cluster_stats = self.cluster_diverse_bugs()
                    
                    if cluster_stats and len(cluster_stats) >= 1:
                        
                        self.cluster_stats = cluster_stats  # 保存聚类信息供后续使用
                        self.logger.info(f"成功识别 {len(cluster_stats)} 个错误模式聚类")
                        
                        # 分析每个聚类
                        for i, cluster in enumerate(cluster_stats):
                            if 'bug_type' in cluster:
                                self.logger.info(f"聚类 {i+1}: {cluster['bug_type']} (包含{cluster['size']}个点)")
                            self._analyze_cluster(cluster, i)
                    else:
                        self.logger.warning("聚类分析未能识别有效的错误模式")
                except Exception as e:
                    self.logger.warning(f"聚类分析失败: {str(e)}")
                    self.logger.exception("聚类分析详细错误:")
            else:
                self.logger.warning(f"样本量({current_samples})不足以进行聚类分析，需要至少{max(20, self.min_samples * 0.2)}个样本")
            
            #=================================================================
            # Stage 2.5: 基于聚类训练多模型贝叶斯优化器
            #=================================================================
            if cluster_stats and len(cluster_stats) >= 1: 
                self.logger.info("\n=== Stage 2.5: 训练基于聚类的多模型贝叶斯优化器 ===")
                try:
                    # 直接使用Stage 2的聚类结果，不要再次聚类
                    success = self._train_clustered_gp_models()
                    multi_gp_trained = success
                    if success:
                        self.logger.info(f"成功训练了 {len(self.clustered_gps)} 个聚类GP模型")
                        
                        # 添加: 评估模型性能
                        try:
                            import numpy as np
                            from sklearn.model_selection import train_test_split
                            
                            # 准备数据
                            X_array = np.array(self.X_history)
                            y_array = np.array(self.y_history)
                            
                            # 划分训练集和测试集
                            X_train, X_test, y_train, y_test = train_test_split(
                                X_array, y_array, test_size=0.2, random_state=42)
                            
                            # 评估模型
                            self.logger.info("开始评估多模型贝叶斯优化器性能...")
                            eval_results = self._evaluate_gp_models(X_test, y_test, evaluate_clustered=True)
                            
                            # 输出评估结果
                            self.logger.info(f"多模型性能评估结果:")
                            self.logger.info(f"  整体模型性能: R² = {eval_results['overall'].get('r2', 'N/A')}, MSE = {eval_results['overall'].get('mse', 'N/A')}")
                            
                            # 记录各个聚类模型的性能
                            self.logger.info("各聚类模型性能:")
                            for model_name, metrics in eval_results['models'].items():
                                if 'error' not in metrics:
                                    self.logger.info(f"  {model_name}: R² = {metrics.get('r2', 'N/A'):.4f}, MSE = {metrics.get('mse', 'N/A'):.4f}, 样本数 = {metrics.get('samples', 'N/A')}")
                                else:
                                    self.logger.info(f"  {model_name}: 评估失败 - {metrics['error']}")
                        except Exception as e:
                            self.logger.warning(f"评估多模型性能时出错: {e}")
                            self.logger.exception("详细错误:")
                    else:
                        self.logger.warning("训练聚类GP模型失败，将继续使用单一模型")
                except Exception as e:
                    self.logger.warning(f"训练多模型贝叶斯优化器时出错: {e}")
                    self.logger.exception("详细错误:")
            
            #=================================================================
            # Stage 3: 多峰贝叶斯持续优化阶段（新增！先于分层搜索）
            #=================================================================
            # Stage 4: 多峰贝叶斯持续优化阶段
            self.logger.info("\n=== Stage 4: 多峰贝叶斯持续优化阶段 ===")

            # 计算剩余采样次数
            current_samples = len(self.X_history)
            remaining_samples = max_calls - current_samples 

            if remaining_samples > 0:
                self.logger.info(f"剩余采样次数: {remaining_samples}")
                
                # 如果前面已经训练了多模型贝叶斯，使用它进行优化
                if multi_gp_trained:
                    self.logger.info("使用多峰贝叶斯模型进行持续优化")
                else:
                    # 如果没有成功训练多模型，尝试训练单一GP模型
                    self.logger.info("多峰模型未训练成功，尝试训练单一GP模型")
                    try:
                        from sklearn.gaussian_process import GaussianProcessRegressor
                        from sklearn.gaussian_process.kernels import Matern, ConstantKernel
                        
                        # 准备训练数据
                        X = np.array(self.X_history)
                        y = np.array(self.y_history)
                        
                        # 定义GP模型
                        kernel = ConstantKernel(1.0) * Matern(length_scale=1.0, nu=2.5)
                        self.gp = GaussianProcessRegressor(
                            kernel=kernel,
                            alpha=1e-6,
                            normalize_y=True,
                            n_restarts_optimizer=5,
                            random_state=42
                        )
                        
                        # 训练模型
                        self.gp.fit(X, y)
                        self.logger.info("单一GP模型训练成功")
                        
                        # 添加: 评估单一模型性能
                        try:
                            from sklearn.model_selection import train_test_split
                            
                            # 划分训练集和测试集
                            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
                            
                            # 评估模型
                            self.logger.info("开始评估单一GP模型性能...")
                            eval_results = self._evaluate_gp_models(X_test, y_test)
                            
                            # 输出评估结果
                            self.logger.info(f"单一GP模型性能: R² = {eval_results['models'].get('base_model', {}).get('r2', 'N/A'):.4f}, " +
                                            f"MSE = {eval_results['models'].get('base_model', {}).get('mse', 'N/A'):.4f}")
                        except Exception as e:
                            self.logger.warning(f"评估单一GP模型性能时出错: {e}")
                    except Exception as e:
                        self.logger.warning(f"训练单一GP模型失败: {e}")
                
                # 建立探索状态跟踪
                exploration_state = {
                    'base_exploration_rate': 0.5,  # 基础探索率
                    'consecutive_no_improvement': 0,
                    'max_no_improvement': 10,  # 连续10次无改进则考虑收敛
                    'best_value': max(self.y_history) if self.y_history else 0,
                    'coverage': 0.0,  # 初始覆盖率
                    'last_coverage_check': 0,  # 上次检查覆盖率的迭代索引
                    'unvisited_cells': []  # 未访问的参数空间单元
                }
                
                # 尝试更新参数空间覆盖率
                try:
                    exploration_state['unvisited_cells'] = self._get_unvisited_cells(max_cells=1000)
                    if hasattr(self, 'total_grid_cells') and self.total_grid_cells > 0:
                        exploration_state['coverage'] = 1.0 - len(exploration_state['unvisited_cells']) / self.total_grid_cells
                        self.logger.info(f"初始参数空间覆盖率: {exploration_state['coverage']:.2f}")
                except Exception as e:
                    self.logger.warning(f"初始化空间覆盖率失败: {e}")
                
                # 持续优化循环
                self.logger.info("开始持续优化循环")
                
                # 模型更新计数器和间隔设置
                model_update_counter = 0
                model_update_interval = 15  # 每15次迭代更新一次模型
                
                for i in range(remaining_samples):
                    self.logger.info(f"\n迭代 {current_samples + i + 1}/{max_calls} (Stage 4)")
                    
                    # 检查资源限制
                    resources = get_system_resources()
                    if resources['memory_percent'] > 85:
                        self.logger.warning(f"系统内存使用率({resources['memory_percent']:.1f}%)过高，暂停优化")
                        break
                    
                    # 确定当前迭代的采样策略
                    sampling_strategy = "default"
                    
                    # 每10次迭代强制更新一次空间覆盖率
                    if i % 10 == 0:
                        try:
                            exploration_state['unvisited_cells'] = self._get_unvisited_cells(max_cells=1000)
                            if hasattr(self, 'total_grid_cells') and self.total_grid_cells > 0:
                                exploration_state['coverage'] = 1.0 - len(exploration_state['unvisited_cells']) / self.total_grid_cells
                                exploration_state['last_coverage_check'] = i
                                self.logger.info(f"当前参数空间覆盖率: {exploration_state['coverage']:.2f}")
                        except Exception as e:
                            self.logger.warning(f"更新空间覆盖率失败: {e}")
                    
                    # 选择采样策略
                    try:
                        # 策略1: 如果连续无改进次数过多，触发强制探索
                        if exploration_state['consecutive_no_improvement'] >= 5:
                            if np.random.rand() < 0.7:  # 70%几率进行强制探索
                                sampling_strategy = "forced_exploration"
                                self.logger.info("触发强制探索机制，连续无改进次数过多")
                        
                        # 策略2: 如果参数空间覆盖率低，优先探索未覆盖区域
                        elif exploration_state['coverage'] < 0.6 and np.random.rand() < 0.5:
                            sampling_strategy = "coverage_exploration"
                            self.logger.info(f"触发覆盖率探索机制，当前覆盖率: {exploration_state['coverage']:.2f}")
                        
                        # 策略3: 周期性强制探索（每8次迭代）
                        elif i % 8 == 0:
                            sampling_strategy = "periodic_exploration"
                            self.logger.info("触发周期性强制探索")
                        
                        # 策略4: Thompson采样增强（每5次迭代）
                        elif i % 5 == 0 and (multi_gp_trained or hasattr(self, 'gp')):
                            sampling_strategy = "thompson_sampling"
                            self.logger.info("使用Thompson采样策略")
                            
                    except Exception as e:
                        self.logger.warning(f"选择采样策略时出错: {e}")
                        sampling_strategy = "default"
                    
                    # 根据选定策略生成下一个采样点
                    try:
                        next_point = None
                        
                        if sampling_strategy == "forced_exploration":
                            # 强制探索 - 优先选择未探索区域
                            if exploration_state['unvisited_cells'] and np.random.rand() < 0.7:
                                # 随机选择一个未访问的网格单元
                                cell_idx = np.random.randint(0, len(exploration_state['unvisited_cells']))
                                cell = exploration_state['unvisited_cells'][cell_idx]
                                next_point = self._generate_point_in_cell(cell, max_attempts=50)
                                self.logger.info(f"在未访问区域生成点: {cell}")
                            else:
                                # 生成多样性测试点
                                candidates = self._generate_test_diversity_candidates(10)
                                if candidates:
                                    # 选择与现有点距离最远的
                                    distances = []
                                    X_array = np.array(self.X_history)
                                    for point in candidates:
                                        point_array = np.array([point.get(dim.name, 0) for dim in self.space])
                                        min_dist = np.min(np.sqrt(np.sum((X_array - point_array)**2, axis=1)))
                                        distances.append(min_dist)
                                    best_idx = np.argmax(distances)
                                    next_point = candidates[best_idx]
                                    self.logger.info(f"选择多样性最大的点，距离: {distances[best_idx]:.4f}")
                        
                        elif sampling_strategy == "coverage_exploration":
                            # 覆盖率探索 - 随机选择未探索区域
                            if exploration_state['unvisited_cells']:
                                cell_idx = np.random.randint(0, len(exploration_state['unvisited_cells']))
                                cell = exploration_state['unvisited_cells'][cell_idx]
                                next_point = self._generate_point_in_cell(cell, max_attempts=50)
                                self.logger.info(f"在未访问区域生成点以提高覆盖率: {cell}")
                            else:
                                # 回退到边界点探索
                                next_point = self._generate_boundary_test_points(1)[0]
                                self.logger.info("无未访问区域，生成边界测试点")
                        
                        elif sampling_strategy == "periodic_exploration":
                            # 周期性强制探索 - LHS采样或边界测试
                            if np.random.rand() < 0.5:
                                next_point = self._generate_lhs_candidates(1)[0]
                                self.logger.info("周期性探索：使用LHS采样")
                            else:
                                next_point = self._generate_boundary_test_points(1)[0]
                                self.logger.info("周期性探索：使用边界测试点")
                        
                        elif sampling_strategy == "thompson_sampling":
                            # Thompson采样增强
                            if multi_gp_trained:
                                try:
                                    # 使用多模型Thompson采样
                                    candidates = self._generate_clustered_gp_candidates(15)
                                    if candidates:
                                        # 用UCB评分选择最佳点
                                        best_score = -float('inf')
                                        for point in candidates:
                                            point_array = np.array([point.get(dim.name, 0) for dim in self.space])
                                            mean, std = self._predict_with_clustered_gps(point_array.reshape(1, -1))
                                            score = mean + 2.0 * std  # UCB策略
                                            if score > best_score:
                                                best_score = score
                                                next_point = point
                                        self.logger.info(f"多模型Thompson采样，最佳UCB得分: {best_score:.4f}")
                                    else:
                                        self.logger.warning("多模型Thompson采样失败，回退到普通采样")
                                except Exception as e:
                                    self.logger.warning(f"多模型Thompson采样出错: {e}")
                            elif hasattr(self, 'gp'):
                                try:
                                    # 使用单一GP模型的Thompson采样
                                    candidates = self._generate_thompson_candidates(15)
                                    if candidates:
                                        next_point = candidates[0]  # 第一个候选点通常是最佳点
                                        self.logger.info("使用单一GP模型Thompson采样")
                                    else:
                                        self.logger.warning("单一GP Thompson采样失败，回退到普通采样")
                                except Exception as e:
                                    self.logger.warning(f"单一GP Thompson采样出错: {e}")
                        
                        # 如果上述专门策略都没生成有效点，使用标准的_get_next_point
                        if next_point is None:
                            # 根据当前探索状态调整探索率
                            dynamic_exploration_boost = exploration_state['consecutive_no_improvement'] > 3
                            next_point = self._get_next_point(exploration_boost=dynamic_exploration_boost)
                            self.logger.info(f"使用标准采样方法，探索增强: {dynamic_exploration_boost}")
                        
                        # 确保点的类型正确
                        next_point = self._ensure_correct_feature_types(next_point)
                        self.logger.info(f"生成采样点: {next_point}")
                        
                        # 执行实验
                        result = self._objective_stage1(**next_point)
                        self.logger.info(f"实验结果: {result}")
                        
                        # 更新探索状态
                        current_best = max(self.y_history)
                        if current_best > exploration_state['best_value']:
                            exploration_state['best_value'] = current_best
                            exploration_state['consecutive_no_improvement'] = 0
                            self.logger.info(f"发现更好的解: {current_best}")
                        else:
                            exploration_state['consecutive_no_improvement'] += 1
                            self.logger.info(f"未发现改进，连续 {exploration_state['consecutive_no_improvement']}/{exploration_state['max_no_improvement']} 次")
                        
                        # 动态调整基础探索率
                        if exploration_state['consecutive_no_improvement'] > 5:
                            # 连续无改进，增加探索
                            exploration_state['base_exploration_rate'] = min(0.8, exploration_state['base_exploration_rate'] * 1.1)
                            self.logger.info(f"增加基础探索率至: {exploration_state['base_exploration_rate']:.2f}")
                        elif exploration_state['consecutive_no_improvement'] == 0:
                            # 有改进，适度减少探索
                            exploration_state['base_exploration_rate'] = max(0.3, exploration_state['base_exploration_rate'] * 0.95)
                            self.logger.info(f"降低基础探索率至: {exploration_state['base_exploration_rate']:.2f}")
                        
                        # 定期保存数据
                        if save_stage1 and (i + 1) % 5 == 0:
                            self._save_stage1_data()
                            
                        # 定期记录优化状态
                        if (i + 1) % 5 == 0:
                            self._log_optimization_state()
                        
                        # 收敛检查
                        if early_stop and (i + 1) % 5 == 0:
                            convergence_checks += 1
                            self.logger.info(f"\n=== 第{convergence_checks}次收敛检查 ===")
                            
                            # 两种收敛判断：
                            # 1. 多次迭代无改进
                            if exploration_state['consecutive_no_improvement'] >= exploration_state['max_no_improvement']:
                                self.logger.info(f"连续{exploration_state['max_no_improvement']}次无改进，判定为收敛")
                                break
                                
                            # 2. 基于UCB和REI的收敛判断
                            convergence_status = self._check_convergence_status(delta, epsilon)
                            if convergence_status.get("整体收敛", False):
                                self.logger.info("基于UCB和REI判定收敛，停止优化")
                                break
                            
                            # 3. 参数空间覆盖率高且连续无改进
                            if exploration_state['coverage'] > 0.8 and exploration_state['consecutive_no_improvement'] > 5:
                                self.logger.info(f"参数空间覆盖率高({exploration_state['coverage']:.2f})且连续无改进，判定接近收敛")
                                # 不直接退出，但降低收敛阈值
                                exploration_state['max_no_improvement'] = 7
                        
                        
                        
                        # 前面应该是基于各种方式在补充samples， 这里开始
                        # 更新模型计数器
                        model_update_counter += 1
                        
                        # 添加/修改: 根据模型更新间隔重新训练模型
                        if model_update_counter >= model_update_interval and cluster_analyses < max_cluster_analysis:
                            self.logger.info(f"\n=== 周期性更新贝叶斯模型 (第{model_update_counter}次迭代) ===")
                            try:
                                # 重新聚类
                                cluster_analyses += 1
                                self.logger.info(f"执行第{cluster_analyses}次聚类分析")
                                cluster_stats = self.cluster_diverse_bugs()
                                
                                if cluster_stats and len(cluster_stats) >= 1:
                                    self.cluster_stats = cluster_stats
                                    self.logger.info(f"成功识别 {len(cluster_stats)} 个错误模式聚类")
                                    
                                    # 重新训练多模型
                                    success = self._train_clustered_gp_models()
                                    if success:
                                        multi_gp_trained = True
                                        self.logger.info(f"成功更新 {len(self.clustered_gps)} 个聚类GP模型")
                                        
                                        # 评估更新后的模型性能
                                        try:
                                            from sklearn.model_selection import train_test_split
                                            
                                            # 准备数据
                                            X_array = np.array(self.X_history)
                                            y_array = np.array(self.y_history)
                                            
                                            # 划分训练集和测试集
                                            X_train, X_test, y_train, y_test = train_test_split(
                                                X_array, y_array, test_size=0.2, random_state=42)
                                            
                                            # 评估模型
                                            self.logger.info("评估更新后的多模型贝叶斯优化器性能...")
                                            eval_results = self._evaluate_gp_models(X_test, y_test, evaluate_clustered=True)
                                            
                                            # 输出评估结果
                                            self.logger.info(f"更新后的多模型性能: R² = {eval_results['overall'].get('r2', 'N/A'):.4f}, " +
                                                            f"MSE = {eval_results['overall'].get('mse', 'N/A'):.4f}")
                                        except Exception as e:
                                            self.logger.warning(f"评估更新后的模型性能时出错: {e}")
                                    else:
                                        self.logger.warning("更新聚类GP模型失败")
                                else:
                                    self.logger.warning("重新聚类未能识别有效的错误模式")
                                    
                                # 重置模型更新计数器
                                model_update_counter = 0
                                
                            except Exception as e:
                                self.logger.warning(f"更新模型时出错: {e}")
                            
                    except Exception as e:
                        self.logger.error(f"Stage 4迭代{i+1}发生错误: {str(e)}")
                        self.logger.exception("详细错误:")
                        if save_stage1:
                            self._save_stage1_data()  # 出错时也保存当前状态
                        continue
                    
                    # 更新当前样本量
                    current_samples = len(self.X_history)
                    if current_samples >= max_calls:
                        break
            else:
                self.logger.info("已达到最大调用次数，跳过Stage 4") 
                        
                        
                        
            #=================================================================
            # Stage 4: 分层搜索阶段 (可选) - 移到最后，使用最新聚类结果
            #=================================================================
            # 如果聚类分析成功且用户启用了分层搜索
            if enable_layered_search and hasattr(self, 'cluster_stats') and self.cluster_stats and len(self.cluster_stats) >= 1:
                self.logger.info("\n=== Stage 5: 分层搜索阶段 ===")
                
                try:
                    # Layer 1: 针对每个错误簇进行深度探索
                    for i, cluster in enumerate(self.cluster_stats):
                        if 'size' in cluster and cluster['size'] > 0:
                            self.logger.info(f"\nLayer 1: 探索错误簇 {i+1}/{len(self.cluster_stats)}")
                            self._perform_layer1_search(cluster, max_samples=5)
                    
                    # Layer 2: 全局交互覆盖
                    self.logger.info("\nLayer 2: 全局交互覆盖")
                    self._perform_layer2_search(self.cluster_stats, max_samples=10)
                    
                    # Layer 3: 局部性能微调
                    self.logger.info("\nLayer 3: 局部性能微调")
                    self._perform_layer3_search(self.cluster_stats, max_samples=5)
                    
                except Exception as e:
                    self.logger.error(f"分层搜索出错: {str(e)}")
                    self.logger.exception("分层搜索详细错误:")
            
            #=================================================================
            # Final: 总结结果
            #=================================================================
            self.logger.info("\n=== 优化完成 ===")
            self.logger.info(f"总采样次数: {len(self.X_history)}")
            
            if len(self.y_history) > 0:
                best_idx = np.argmax(self.y_history)
                best_point = self.X_history[best_idx]
                best_value = self.y_history[best_idx]
                
                if isinstance(best_point, np.ndarray):
                    best_params = dict(zip([dim.name for dim in self.space], best_point))
                else:
                    best_params = best_point
                
                self.logger.info(f"最佳误差值: {best_value}")
                self.logger.info(f"最佳参数: {best_params}")
                
                return best_params
            else:
                self.logger.warning("优化结束，但没有有效的评估结果")
                return {}
            
        except Exception as e:
            self.logger.error(f"优化过程中发生错误: {str(e)}")
            self.logger.exception("优化详细错误:")
            
            # 尝试返回当前最佳结果
            if len(self.y_history) > 0:
                best_idx = np.argmax(self.y_history)
                best_point = self.X_history[best_idx]
                
                if isinstance(best_point, np.ndarray):
                    best_params = dict(zip([dim.name for dim in self.space], best_point))
                else:
                    best_params = best_point
                
                return best_params
            else:
                return {}
        
        
    def _initialize_space_grid(self, divisions_per_dim=5, max_dims=6):
        """初始化参数空间网格，但限制维度数量以避免组合爆炸
        
        Args:
            divisions_per_dim: 每个维度划分的格数
            max_dims: 最大考虑的维度数量
        """
        self.logger.info(f"初始化参数空间网格，每维划分为{divisions_per_dim}格")
        
        # 选择最重要的几个维度
        if len(self.space) > max_dims:
            # 根据历史数据重要性选择重要维度
            important_dims = self._select_important_dimensions(max_dims)
            self.logger.info(f"参数空间维度较多({len(self.space)})，只为最重要的{len(important_dims)}个维度建立网格")
            grid_dims = [self.space[i] for i in important_dims]
        else:
            grid_dims = self.space
            important_dims = list(range(len(self.space)))
        
        # 初始化网格结构 - 使用稀疏存储
        self.grid_dims = grid_dims
        self.grid_dim_indices = important_dims
        self.grid_divisions = divisions_per_dim
        self.visited_cells = set()  # 只存储已访问的单元格索引元组
        
        # 为每个维度创建分箱信息
        self.dim_bins = []  # 这里添加了缺失的dim_bins属性
        for dim in grid_dims:
            if hasattr(dim, 'bounds'):
                low, high = dim.bounds
                bins = np.linspace(low, high, divisions_per_dim + 1)
                self.dim_bins.append(bins)
            else:
                # 对于没有明确边界的维度，创建默认分箱
                self.dim_bins.append(np.linspace(0, divisions_per_dim - 1, divisions_per_dim + 1))
        
        # 计算理论上的总单元格数 - 但不实际生成所有单元格
        self.total_grid_cells = divisions_per_dim ** len(grid_dims)
        self.logger.info(f"理论网格单元格总数: {self.total_grid_cells}")
        
        # 更新已访问单元格
        self._update_visited_cells()
  
  
    def _update_visited_cells(self):
        """更新已访问的参数空间单元格"""
        if not hasattr(self, 'grid_dims') or not self.grid_dims or not hasattr(self, 'dim_bins'):
            self.logger.warning("空间网格未完全初始化，无法更新已访问单元格")
            return
        
        try:
            # 仅处理新添加的历史点
            start_idx = getattr(self, 'last_visited_idx', 0)
            X_history = self.X_history[start_idx:]
            
            for point in X_history:
                cell_indices = []
                
                # 将点转换为数组格式
                if isinstance(point, dict):
                    point_values = []
                    for dim_idx in self.grid_dim_indices:
                        dim = self.space[dim_idx]
                        point_values.append(point.get(dim.name, 0))
                else:
                    point_values = [point[dim_idx] if dim_idx < len(point) else 0 for dim_idx in self.grid_dim_indices]
                
                # 确定每个维度的单元格索引
                for i, (value, bins) in enumerate(zip(point_values, self.dim_bins)):
                    # 使用数字搜索找到对应的分箱索引
                    idx = np.searchsorted(bins, value) - 1
                    # 确保索引在有效范围内
                    idx = max(0, min(idx, self.grid_divisions - 1))
                    cell_indices.append(idx)
                
                # 将单元格添加到已访问集合
                self.visited_cells.add(tuple(cell_indices))
            
            # 更新最后处理的索引
            self.last_visited_idx = len(self.X_history)
            
        except Exception as e:
            self.logger.warning(f"更新已访问单元时出错: {e}")
  
  
    def _get_unvisited_cells(self, max_cells=1000):
        """获取未访问的单元格，限制返回数量以避免内存问题
        
        Args:
            max_cells: 最大返回的单元格数量
        
        Returns:
            未访问单元格的列表(有限数量)
        """
        if not hasattr(self, 'grid_dims') or not self.grid_dims:
            self.logger.warning("空间网格未初始化，无法获取未访问单元格")
            return []
        
        # 更新已访问单元格信息
        self._update_visited_cells()
        
        # 使用采样而不是枚举所有可能的单元格
        unvisited_cells = []
        attempts = 0
        max_attempts = max_cells * 10  # 最大尝试次数
        
        while len(unvisited_cells) < max_cells and attempts < max_attempts:
            # 随机生成一个单元格索引
            cell_indices = tuple(np.random.randint(0, self.grid_divisions) 
                                for _ in range(len(self.grid_dims)))
            
            # 检查是否已访问
            if cell_indices not in self.visited_cells:
                unvisited_cells.append(cell_indices)
                # 添加到已访问集合中避免重复
                self.visited_cells.add(cell_indices)
            
            attempts += 1
        
        self.logger.info(f"找到{len(unvisited_cells)}个未访问单元格(尝试{attempts}次)")
        return unvisited_cells


    def _generate_point_in_cell(self, cell_indices, max_attempts=50):
        """在指定单元格中生成点，限制尝试次数以避免死循环
        
        Args:
            cell_indices: 单元格索引
            max_attempts: 最大尝试次数
        
        Returns:
            参数点字典
        """
        if not hasattr(self, 'grid_dims') or not self.grid_dims:
            self.logger.warning("空间网格未初始化，回退到随机生成点")
            return self._generate_random_point()
        
        # 创建结果字典
        point = {}
        
        try:
            # 为选择的网格维度生成值
            for i, (dim_idx, cell_idx) in enumerate(zip(self.grid_dim_indices, cell_indices)):
                dim = self.space[dim_idx]
                name = dim.name
                
                # 获取该单元格在此维度的范围
                if hasattr(dim, 'bounds'):
                    low, high = dim.bounds
                    cell_width = (high - low) / self.grid_divisions
                    cell_low = low + cell_idx * cell_width
                    cell_high = cell_low + cell_width
                    
                    # 生成该范围内的随机值
                    if name in ['num_vehicles', 'num_walkers', 'sensor_number', 'static_object_number', 'lidar_channels', 'lidar_points']:
                        # 整数参数
                        value = int(np.random.randint(math.ceil(cell_low), math.floor(cell_high) + 1))
                    elif name in ['weather_preset', 'vehicle_type', 'sensor_type', 'town', 'static_object_type']:
                        # 分类参数
                        value = int(cell_idx)  # 使用单元格索引作为分类值
                    else:
                        # 浮点参数
                        value = float(np.random.uniform(cell_low, cell_high))
                    
                    point[name] = value
            
            # 为未包含在网格中的维度生成随机值
            for i, dim in enumerate(self.space):
                if i not in self.grid_dim_indices:
                    point[dim.name] = self._generate_random_parameter_value(dim)
            
            # 确保参数类型正确
            return self._ensure_correct_feature_types(point)
        
        except Exception as e:
            self.logger.warning(f"在单元格{cell_indices}生成点失败: {e}")
            return self._generate_random_point()

    def _ensure_correct_feature_types(self, params):
        """确保参数值类型正确
        
        Args:
            params: 参数字典
            
        Returns:
            类型纠正后的参数字典
        """
        try:
            corrected_params = params.copy()
            
            # 整数类型参数列表
            integer_params = [
                'num_vehicles', 'num_walkers', 'sensor_count', 'actor_active_distance',
                'traffic_lights_count', 'collision_sensor_count', 'npc_focus_distance', 
                'weather_change_freq', 'spawn_distance'
            ]
            
            # 分类型参数列表
            categorical_params = [
                'town', 'weather_preset', 'sim_mode', 'static_object_type',
                'vehicle_type', 'sensor_type', 'traffic_behavior',
                'night_mode', 'fog_level', 'motion_model'
            ]
            
            # 对每个参数进行类型检查和转换
            for dim in self.space:
                name = dim.name
                
                # 跳过不存在的参数
                if name not in corrected_params:
                    continue
                    
                value = corrected_params[name]
                
                try:
                    if name in categorical_params or (hasattr(dim, 'type') and dim.type == 'categorical'):
                        # 分类参数处理
                        if hasattr(dim, 'categories') and value not in dim.categories:
                            # 如果值不在有效类别中，选择一个有效类别
                            corrected_params[name] = np.random.choice(dim.categories)
                            self.logger.debug(f"纠正分类参数 {name}: {value} -> {corrected_params[name]}")
                            
                    elif name in integer_params or (hasattr(dim, 'dtype') and dim.dtype == 'int'):
                        # 整数参数处理
                        if not isinstance(value, (int, np.integer)) or int(value) != value:
                            corrected_params[name] = int(round(value))
                            self.logger.debug(f"纠正整数参数 {name}: {value} -> {corrected_params[name]}")
                        
                        # 检查边界
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            if corrected_params[name] < low:
                                corrected_params[name] = int(low)
                                self.logger.debug(f"纠正整数参数下界 {name}: {value} -> {corrected_params[name]}")
                            elif corrected_params[name] > high:
                                corrected_params[name] = int(high)
                                self.logger.debug(f"纠正整数参数上界 {name}: {value} -> {corrected_params[name]}")
                    
                    else:
                        # 浮点参数处理
                        if not isinstance(value, (float, int, np.number)):
                            corrected_params[name] = float(value)
                            self.logger.debug(f"纠正浮点参数 {name}: {value} -> {corrected_params[name]}")
                            
                        # 检查边界
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            if corrected_params[name] < low:
                                corrected_params[name] = float(low)
                                self.logger.debug(f"纠正浮点参数下界 {name}: {value} -> {corrected_params[name]}")
                            elif corrected_params[name] > high:
                                corrected_params[name] = float(high)
                                self.logger.debug(f"纠正浮点参数上界 {name}: {value} -> {corrected_params[name]}")
                                
                except Exception as e:
                    # 参数转换失败，设置默认值
                    self.logger.warning(f"参数 {name} 类型转换失败: {e}，使用默认值")
                    if name in categorical_params and hasattr(dim, 'categories') and len(dim.categories) > 0:
                        corrected_params[name] = dim.categories[0]
                    elif hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        default_val = (low + high) / 2
                        if name in integer_params:
                            corrected_params[name] = int(round(default_val))
                        else:
                            corrected_params[name] = float(default_val)
                    else:
                        # 无法确定合适的默认值，移除该参数
                        del corrected_params[name]
            
            return corrected_params
        
        except Exception as e:
            self.logger.error(f"参数类型检查失败: {e}")
            # 返回原始参数，避免完全失败
            return params
                
    def _perform_layer2_search(self, cluster_stats, max_samples=10):
        """执行全局交互覆盖 (Layer 2) 搜索
        
        基于聚类信息，探索参数空间中可能的交互关系
        
        Args:
            cluster_stats: 聚类统计信息
            max_samples: 最大采样次数
        """
        self.logger.info(f"开始Layer 2搜索 (全局交互覆盖)，最多{max_samples}个样本")
        
        try:
            # 如果聚类信息不存在或为空，返回
            if not cluster_stats or len(cluster_stats) == 0:
                self.logger.warning("没有有效的聚类信息，跳过Layer 2搜索")
                return
                
            # 收集所有错误簇的关键参数
            key_params = set()
            for cluster in cluster_stats:
                if 'key_params' in cluster:
                    for param in cluster['key_params']:
                        key_params.add(param)
            
            if not key_params:
                self.logger.warning("没有识别到关键参数，使用所有参数")
                key_params = [dim.name for dim in self.space]
                
            self.logger.info(f"识别到的关键参数: {key_params}")
            
            # 生成测试候选点，探索关键参数之间的交互
            candidates = []
            
            # 组合LHS + 关键参数边界测试
            try:
                # LHS部分
                lhs_candidates = self._generate_lhs_candidates(max_samples // 2)
                if lhs_candidates:
                    candidates.extend(lhs_candidates)
                    self.logger.info(f"生成了{len(lhs_candidates)}个LHS候选点")
                    
                # 边界测试部分 - 专注于关键参数
                boundary_candidates = self._generate_boundary_test_points(max_samples // 2)
                if boundary_candidates:
                    candidates.extend(boundary_candidates)
                    self.logger.info(f"生成了{len(boundary_candidates)}个边界测试候选点")
                    
            except Exception as e:
                self.logger.warning(f"生成候选点时出错: {e}")
                
            # 如果候选点不足，添加随机点补充
            if len(candidates) < max_samples:
                n_random = max_samples - len(candidates)
                random_candidates = self._generate_random_candidates(n_random)
                if random_candidates:
                    candidates.extend(random_candidates)
                    self.logger.info(f"生成了{len(random_candidates)}个随机候选点补充")
            
            # 评估并选择最终的采样点
            selected_points = []
            for _ in range(min(max_samples, len(candidates))):
                if not candidates:
                    break
                    
                # 评估所有候选点
                best_score = -float('inf')
                best_point = None
                best_idx = -1
                
                for i, point in enumerate(candidates):
                    # 计算多样性得分
                    diversity_score = self._evaluate_test_diversity(point, self.X_history + selected_points, candidates)
                    
                    # 计算与关键参数相关的探索得分
                    param_exploration_score = 0
                    for param in key_params:
                        if param in point:
                            # 检查该参数是否接近边界值
                            for dim in self.space:
                                if dim.name == param and hasattr(dim, 'bounds'):
                                    low, high = dim.bounds
                                    val = point[param]
                                    # 距离边界越近得分越高
                                    norm_dist = min(abs(val - low), abs(val - high)) / (high - low)
                                    param_exploration_score += 1.0 - min(0.8, norm_dist)
                    
                    # 总分 = 多样性得分 + 参数探索得分
                    total_score = diversity_score + 0.5 * param_exploration_score
                    
                    if total_score > best_score:
                        best_score = total_score
                        best_point = point
                        best_idx = i
                
                if best_point is not None:
                    selected_points.append(best_point)
                    candidates.pop(best_idx)
            
            # 执行采样
            self.logger.info(f"Layer 2最终选择了{len(selected_points)}个采样点")
            
            for i, point in enumerate(selected_points):
                try:
                    self.logger.info(f"执行Layer 2采样 {i+1}/{len(selected_points)}")
                    result = self._objective_stage1(**point)
                    self.logger.info(f"实验结果: {result}")
                except Exception as e:
                            self.logger.error(f"采样点{i+1}执行失败: {e}")
                
        except Exception as e:
            self.logger.error(f"Layer 2搜索出错: {e}")
            self.logger.exception("详细错误:")







    def _initialize_data_structures(self):
        """初始化优化所需的数据结构"""
        if not hasattr(self, 'X_history') or self.X_history is None:
            self.X_history = []
        if not hasattr(self, 'y_history') or self.y_history is None:
            self.y_history = []
        if not hasattr(self, 'error_log') or self.error_log is None:
            self.error_log = []
        if not hasattr(self, 'detailed_errors') or self.detailed_errors is None:
            self.detailed_errors = []
        self.logger.info("优化数据结构已初始化")
    
    
    def _analyze_cluster(self, cluster, cluster_idx):
        """对单个簇进行简单分析"""
        try:
            self.logger.info(f"\n分析簇 {cluster_idx + 1}")
            
            # 获取点索引
            point_indices = cluster.get('point_indices', [])
            if not point_indices:
                self.logger.warning(f"聚类 {cluster_idx + 1} 没有有效的点索引")
                return
            
            # 从索引获取实际的点数据
            points = []
            for idx in point_indices:
                if idx < len(self.X_history):
                    if isinstance(self.X_history[idx], dict):
                        points.append(self.X_history[idx])
                    else:
                        points.append(dict(zip([dim.name for dim in self.space], self.X_history[idx])))
            
            # 提取该簇的性能指标
            errors = [self.y_history[i] for i in point_indices if i < len(self.y_history)]
            
            if not points or not errors:
                self.logger.warning(f"聚类 {cluster_idx + 1} 没有有效的数据点或错误值")
                return
            
            # 基本统计分析
            self.logger.info(f"簇大小: {len(points)}")
            self.logger.info(f"平均误差: {np.mean(errors):.4f}")
            self.logger.info(f"最大误差: {np.max(errors):.4f}")
            
            # 找出该簇中的最佳点
            best_idx = np.argmax(errors)
            best_point_idx = point_indices[best_idx]
            best_point = points[best_idx]
            
            self.logger.info(f"簇中最佳点 (索引: {best_point_idx}):")
            for param, value in best_point.items():
                self.logger.info(f"  {param}: {value}")
                
        except Exception as e:
            self.logger.error(f"簇分析失败: {str(e)}")
            self.logger.exception("详细错误:")
        
        
    
    def _check_convergence_status(self, delta, epsilon):
        """检查优化收敛状态"""
        # 运行各种收敛检查
        ucb_converged = self.check_ucb_convergence(self.X_history, delta)
        rei_delta_converged = self.check_delta_rei_convergence(epsilon)
        rei_max_converged = self.check_max_rei_convergence(epsilon)
        cluster_converged = self.check_cluster_convergence(epsilon)
        
        self.logger.info("\n===== 收敛检查结果 =====")
        self.logger.info(f"- UCB收敛: {ucb_converged}")
        self.logger.info(f"- REI增量收敛: {rei_delta_converged}")
        self.logger.info(f"- REI最大值收敛: {rei_max_converged}")
        self.logger.info(f"- 聚类收敛: {cluster_converged}")
        
        # 整理收敛状态字典
        convergence_status = {
            "UCB收敛": ucb_converged,
            "REI增量收敛": rei_delta_converged,
            "REI最大值收敛": rei_max_converged,
            "聚类收敛": cluster_converged
        }
        
        # 计算收敛比例和关键收敛指标
        converged_count = sum(1 for value in convergence_status.values() if value)
        converged_ratio = converged_count / len(convergence_status)
        
        # 判断是否整体收敛 - 在多峰场景下调整判断标准
        is_early_stage = len(self.X_history) < 50
        is_multi_modal = hasattr(self, 'gp_models') and isinstance(self.gp_models, list) and len(self.gp_models) > 1
        
        # 在不同阶段使用不同的收敛判断标准
        if is_early_stage:
            # 早期阶段：仅当全部条件满足时才判断为收敛
            overall_converged = all(convergence_status.values())
            self.logger.info(f"早期阶段收敛判断 - 要求所有条件满足")
        elif is_multi_modal:
            # 多峰优化阶段：特别重视聚类收敛和REI最大值收敛
            overall_converged = (cluster_converged and rei_max_converged) or (converged_ratio >= 0.75)
            self.logger.info(f"多峰优化阶段收敛判断 - 要求聚类和REI最大值收敛，或75%条件满足")
        else:
            # 常规阶段：大多数条件满足即可
            overall_converged = converged_ratio >= 0.5
            self.logger.info(f"常规阶段收敛判断 - 要求50%条件满足")
            
        self.logger.info(f"收敛条件满足比例: {converged_ratio:.2f} ({converged_count}/{len(convergence_status)})")
        self.logger.info(f"整体收敛判断结果: {'已收敛' if overall_converged else '未收敛'}")
        
        # 添加整体收敛状态到字典
        convergence_status["整体收敛"] = overall_converged
        
        # 记录详细的判断过程
        for check_name, is_converged in convergence_status.items():
            self.logger.info(f"{check_name}: {'是' if is_converged else '否'}")
        
        return convergence_status

    def _log_optimization_state(self):
        """记录优化状态"""
        self.logger.info("\n=== 优化状态 ===")
        self.logger.info(f"当前样本量: {len(self.X_history)}")
        
        if len(self.y_history) > 0:
            self.logger.info(f"当前最优值: {max(self.y_history)}")
            self.logger.info(f"最近5次结果: {self.y_history[-5:]}")
        
        resources = get_system_resources()
        self.logger.info(f"资源使用: 内存={resources['memory_mb']:.2f}MB, CPU={resources['cpu_percent']}%")
        
    def _supplement_cluster(self, cluster_points, cluster_errors):
        """补充簇内样本"""
        try:
            current_samples = len(cluster_points)
            target_samples = 10  # 目标样本量
            n_supplement = target_samples - current_samples
            
            self.logger.info(f"簇内样本量（{current_samples}）不足，需要补充{n_supplement}个样本")
            
            # 计算簇的统计特征
            points = np.array(cluster_points)
            center = np.mean(points, axis=0)
            cov = np.cov(points.T) + np.eye(points.shape[1]) * 1e-6  # 添加小扰动避免奇异性
            
            # 生成新采样点
            new_points = []
            new_errors = []
            
            for _ in range(n_supplement):
                # 生成一个新点
                new_point = np.random.multivariate_normal(center, cov)
                
                # 确保在参数范围内并处理整数参数
                point_dict = {}
                for i, dim in enumerate(self.space):
                    value = new_point[i]
                    
                    # 确保在范围内
                    value = np.clip(value, dim.low, dim.high)
                    
                    # 处理整数参数
                    if isinstance(dim, Integer):
                        value = int(round(value))
                    
                    
                    if dim.name == 'num_vehicles':
                        value = int(round(value))  # 确保是整数
                        value = max(1, value)      # 确保至少有1辆车
                    elif dim.name == 'num_walkers':
                        value = int(round(value))  # 确保是整数
                        value = max(0, value)      # 允许0个行人
                    elif dim.name == 'sensor_number':
                        value = int(round(value))  # 确保是整数
                        value = max(1, value)      # 确保至少有1个传感器
                    elif dim.name in ['vehicle_type', 'sensor_type', 'weather_preset', 
                                    'town', 'static_object_type']:  # 添加新的分类变量
                        value = int(round(value))  # 确保是整数
                    elif dim.name == 'static_object_number':
                        value = int(round(value))  # 确保是整数
                        value = max(0, value)      # 允许0个静态物体
                    elif dim.name in ['rain_intensity', 'fog_density', 'wind_intensity',
                                    'sun_altitude_angle', 'humidity']:
                        value = float(value)       # 保持为浮点数
                        value = max(0.0, value)    # 确保非负
                    elif dim.name in ['sensor_tick_ratio', 'lidar_param_ratio']:
                        value = float(value)       # 保持为浮点数
                        value = max(0.0, min(1.0, value))  # 确保在[0,1]范围内
                    elif dim.name == 'sensor_tick_value':
                        value = float(value)       # 保持为浮点数
                        value = max(0.0, value)    # 确保非负
                    elif dim.name == 'lidar_channels':
                        value = int(round(value))  # 确保是整数
                        value = max(32, value)     # 确保至少32通道
                    elif dim.name == 'lidar_points':
                        value = int(round(value))  # 确保是整数
                        value = max(100000, value) # 确保至少100000点
                    elif dim.name == 'lidar_rotation':
                        value = float(value)       # 保持为浮点数
                        value = max(10.0, value)   # 确保至少10Hz
                    
                    

                    point_dict[dim.name] = value
                
                # 评估新点
                error = self._objective_stage1(**point_dict)
                
                new_points.append([point_dict[dim.name] for dim in self.space])
                new_errors.append(error)
            
            # 返回补充的点和对应的误差
            return np.array(new_points), new_errors
            
        except Exception as e:
            self.logger.error(f"补充簇内样本失败: {e}")
            self.logger.exception("详细错误信息：")
            return np.array([]), []
                
                

    def _save_stage1_data(self):
        """保存Stage 1的数据"""
        try:
            stage1_data = {
                    'X_history': self.X_history,
                    'y_history': self.y_history,
                'error_log': self.error_log,
                'detailed_errors': self.detailed_errors
                }
                
            save_path =  'stage1_data.pkl'
            with open(save_path, 'wb') as f:
                pickle.dump(stage1_data, f)
            
            self.logger.info(f"Stage 1数据已保存到: {save_path}")
            return True
        except Exception as e:
            self.logger.error(f"保存Stage 1数据失败: {e}")
            return False


    def _load_stage1_data(self):
        """加载Stage 1的数据"""
        try:
            load_path =  'stage1_data.pkl'
            if not os.path.exists(load_path):
                self.logger.warning(f"找不到Stage 1数据文件: {load_path}")
                return False
                
            with open(load_path, 'rb') as f:
                stage1_data = pickle.load(f)
                
            self.X_history = stage1_data['X_history']
            self.y_history = stage1_data['y_history']
            self.error_log = stage1_data['error_log']
            self.detailed_errors = stage1_data['detailed_errors']
            
            self.logger.info(f"成功加载Stage 1数据，包含{len(self.X_history)}个实验点")
            return True
        except Exception as e:
            self.logger.error(f"加载Stage 1数据失败: {e}")
            return False
        
    
    
    def _save_error_analysis(self, error_analysis_results):
        """保存误差分析结果"""
        try:
            # 确保实验目录存在
            if not hasattr(self, 'exp_dir'):
                self.exp_dir = 'experiment_results'
            os.makedirs(self.exp_dir, exist_ok=True)
            
            # 保存分析结果
            analysis_file = os.path.join(self.exp_dir, 'error_analysis.json')
            
            # 转换结果为可序列化格式
            serializable_results = []
            for result in error_analysis_results:
                serializable_result = {
                    'cluster_id': result['cluster_id'],
                    'analysis': self._convert_to_serializable(result['analysis'])
                }
                serializable_results.append(serializable_result)
            
            with open(analysis_file, 'w') as f:
                json.dump(serializable_results, f, indent=2)
            
            self.logger.info(f"\n误差分析结果已保存到: {analysis_file}")
            
            # 生成并保存分析报告摘要
            summary_file = os.path.join(self.exp_dir, 'error_analysis_summary.txt')
            with open(summary_file, 'w') as f:
                for result in error_analysis_results:
                    f.write(f"\n=== 簇 {result['cluster_id']} 分析结果 ===\n")
                    f.write(result['analysis']['summary'])  # 使用新生成的summary
                    f.write("\n" + "="*50 + "\n")
            
            self.logger.info(f"分析报告摘要已保存到: {summary_file}")
            
        except Exception as e:
            self.logger.error(f"保存分析结果失败: {e}")
            self.logger.exception("详细错误信息：")

    def _extract_error_features(self, detailed_errors):
        """提取更全面的错误特征"""
        try:
            error_features = {}
            metrics = ['position_errors', 'velocity_errors', 'acceleration_errors']
            
            self.logger.info("  正在提取错误特征:")
            for metric in metrics:
                error_features[metric] = {}
                metric_data = []
                
                self.logger.info(f"    处理 {metric}...")
                for error_data in detailed_errors:
                    if metric not in error_data:
                        self.logger.warning(f"    未找到 {metric} 数据")
                        continue
                        
                    error_seq = np.array(error_data[metric])
                    if len(error_seq) == 0:
                        self.logger.warning(f"    {metric} 数据为空")
                        continue
                    
                    # 1. 基础统计特征
                    features = {
                        'final_error': float(np.mean(error_seq[-10:])),    # 最终稳态误差
                        'peak_error': float(np.max(np.abs(error_seq))),    # 最大绝对误差
                        'mean_error': float(np.mean(np.abs(error_seq))),   # 平均绝对误差
                        'std_error': float(np.std(error_seq)),             # 误差标准差
                    }
                    
                    # 2. 动态特征
                    features.update({
                        'settling_time': self._calculate_settling_time(error_seq),  # 稳定时间
                        'overshoot': self._calculate_overshoot(error_seq),         # 超调量
                        'rise_time': self._calculate_rise_time(error_seq),         # 上升时间
                        'oscillation_freq': self._calculate_oscillation(error_seq), # 振荡频率
                    })
                    
                    # 3. 趋势特征
                    features.update({
                        'error_trend': self._calculate_error_trend(error_seq),     # 误差趋势
                        'convergence_rate': self._calculate_convergence(error_seq), # 收敛速率
                    })
                    
                    # 4. 分段特征
                    features.update(self._calculate_phase_features(error_seq))
                    
                    metric_data.append(features)
                
                # 合并所有样本的特征
                if metric_data:
                    error_features[metric] = {
                        feature: [d[feature] for d in metric_data]
                        for feature in metric_data[0].keys()
                    }
                    self.logger.info(f"    成功提取 {len(metric_data)} 个样本的特征")
                else:
                    self.logger.warning(f"    {metric} 没有有效数据")
                    error_features[metric] = None
            
            return error_features
            
        except Exception as e:
            self.logger.error(f"提取错误特征失败: {e}")
            self.logger.exception("详细错误信息：")
            return None

    def _calculate_settling_time(self, error_seq, threshold=0.05):
        """计算稳定时间（误差进入±5%带内的时间）"""
        try:
            final_value = np.mean(error_seq[-10:])
            tolerance = abs(final_value * threshold)
            mask = np.abs(error_seq - final_value) <= tolerance
            if not np.any(mask):
                return len(error_seq)  # 如果没有稳定，返回整个序列长度
            return np.where(mask)[0][0]
        except:
            return len(error_seq)

    def _calculate_overshoot(self, error_seq):
        """计算超调量"""
        try:
            steady_state = np.mean(error_seq[-10:])
            peak = np.max(np.abs(error_seq))
            if steady_state == 0:
                return float(peak)
            return float((peak - abs(steady_state)) / abs(steady_state))
        except:
            return 0.0

    def _calculate_rise_time(self, error_seq):
        """计算上升时间（从10%到90%的时间）"""
        try:
            final_value = np.mean(error_seq[-10:])
            if final_value == 0:
                return 0.0
            normalized = error_seq / final_value
            t_10 = np.where(normalized >= 0.1)[0][0]
            t_90 = np.where(normalized >= 0.9)[0][0]
            return float(t_90 - t_10)
        except:
            return 0.0

    def _calculate_oscillation(self, error_seq):
        """计算振荡特征（使用FFT分析）"""
        try:
            from scipy import signal
            # 去除趋势
            detrended = signal.detrend(error_seq)
            # 计算功率谱密度
            f, Pxx = signal.welch(detrended, nperseg=min(256, len(detrended)))
            # 返回主要频率
            return float(f[np.argmax(Pxx)])
        except:
            return 0.0

    def _calculate_error_trend(self, error_seq):
        """计算误差趋势（使用线性回归）"""
        try:
            from scipy import stats
            x = np.arange(len(error_seq))
            slope, _, _, _, _ = stats.linregress(x, error_seq)
            return float(slope)
        except:
            return 0.0

    def _calculate_convergence(self, error_seq):
        """计算收敛速率（使用指数拟合）"""
        try:
            from scipy.optimize import curve_fit
            def exp_func(x, a, b, c):
                return a * np.exp(-b * x) + c
            x = np.arange(len(error_seq))
            popt, _ = curve_fit(exp_func, x, error_seq, maxfev=1000)
            return float(popt[1])  # 返回衰减率
        except:
            return 0.0

    def _calculate_phase_features(self, error_seq):
        """计算分段特征"""
        try:
            n = len(error_seq)
            # 将序列分为三段：初始、中间、最终
            initial = error_seq[:n//3]
            middle = error_seq[n//3:2*n//3]
            final = error_seq[2*n//3:]
            
            return {
                'initial_phase_mean': float(np.mean(np.abs(initial))),
                'initial_phase_std': float(np.std(initial)),
                'middle_phase_mean': float(np.mean(np.abs(middle))),
                'middle_phase_std': float(np.std(middle)),
                'final_phase_mean': float(np.mean(np.abs(final))),
                'final_phase_std': float(np.std(final))
            }
        except:
            return {
                'initial_phase_mean': 0.0, 'initial_phase_std': 0.0,
                'middle_phase_mean': 0.0, 'middle_phase_std': 0.0,
                'final_phase_mean': 0.0, 'final_phase_std': 0.0
            }

    def _analyze_feature_importance_for_aspect(self, X, error_features):
        """分析特定错误特征的参数重要性"""
        try:
            from sklearn.ensemble import RandomForestRegressor
            
            if error_features is None:
                self.logger.warning("    没有有效的错误特征数据")
                return {}
                
            importance_results = {}
            for feature_name, feature_values in error_features.items():
                if not feature_values:  # 检查是否为空
                    continue
                    
                y = np.array(feature_values)
                if len(y) != len(X):
                    self.logger.warning(f"    特征 {feature_name} 的样本数量不匹配")
                    continue
                    
                rf = RandomForestRegressor(n_estimators=100, random_state=42)
                rf.fit(X, y)
                
                # 获取重要性并排序
                importance = dict(zip(self.param_names, rf.feature_importances_))
                sorted_importance = sorted(importance.items(), key=lambda x: x[1], reverse=True)
                
                # 只保留重要性超过阈值的特征
                threshold = 0.1
                significant_features = {k: v for k, v in sorted_importance if v > threshold}
                
                if significant_features:
                    importance_results[feature_name] = significant_features
                    self.logger.info(f"    {feature_name} 的重要特征:")
                    for param, imp in significant_features.items():
                        self.logger.info(f"      - {param}: {imp:.4f}")
                else:
                    self.logger.info(f"    {feature_name} 没有发现显著重要的特征")
            
            return importance_results
            
        except Exception as e:
            self.logger.error(f"特征重要性分析失败: {e}")
            self.logger.exception("详细错误信息：")
            return {}

    def _analyze_parameter_thresholds_for_aspect(self, X, error_features):
        """分析特定错误特征的参数阈值"""
        try:
            from sklearn.tree import DecisionTreeRegressor
            
            if error_features is None:
                self.logger.warning("    没有有效的错误特征数据")
                return {}
                
            threshold_results = {}
            for feature_name, feature_values in error_features.items():
                if not feature_values:
                    continue
                    
                y = np.array(feature_values)
                if len(y) != len(X):
                    self.logger.warning(f"    特征 {feature_name} 的样本数量不匹配")
                    continue
                
                param_thresholds = {}
                for i, param_name in enumerate(self.param_names):
                    X_param = X[:, i].reshape(-1, 1)
                    dt = DecisionTreeRegressor(max_depth=3, min_samples_leaf=max(2, len(X)//10))
                    dt.fit(X_param, y)
                    
                    # 提取有意义的阈值
                    split_thresholds = sorted(dt.tree_.threshold[dt.tree_.threshold != -2])
                    if split_thresholds:
                        param_thresholds[param_name] = split_thresholds
                        self.logger.info(f"    {feature_name} - {param_name} 的阈值: {split_thresholds}")
                
                if param_thresholds:
                    threshold_results[feature_name] = param_thresholds
                else:
                    self.logger.info(f"    {feature_name} 没有发现显著的参数阈值")
            
            return threshold_results
            
        except Exception as e:
            self.logger.error(f"参数阈值分析失败: {e}")
            self.logger.exception("详细错误信息：")
            return {}

    def _save_pre_analysis_data(self, cluster_stats):
        """保存聚类后、分析前的数据"""
        try:
            analysis_data = {
                'X_history': self.X_history,
                'y_history': self.y_history,
                'detailed_errors': self.detailed_errors,
                'cluster_stats': cluster_stats,
                'timestamp': datetime.now().strftime('%Y%m%d_%H%M%S')
            }
            
            save_path = 'pre_analysis_data.pkl'
            with open(save_path, 'wb') as f:
                pickle.dump(analysis_data, f)
            
            self.logger.info(f"预分析数据已保存到: {save_path}")
            return True
        except Exception as e:
            self.logger.error(f"保存预分析数据失败: {e}")
            self.logger.exception("详细错误信息：")
            return False

    def _load_pre_analysis_data(self):
        """加载之前保存的预分析数据"""
        try:
            load_path = 'pre_analysis_data.pkl'
            if not os.path.exists(load_path):
                self.logger.warning(f"找不到预分析数据文件: {load_path}")
                return None
                
            with open(load_path, 'rb') as f:
                analysis_data = pickle.load(f)
                
            # 验证数据完整性
            required_keys = ['X_history', 'y_history', 'detailed_errors', 'cluster_stats']
            if not all(key in analysis_data for key in required_keys):
                self.logger.warning("加载的预分析数据格式不完整")
                return None
                
            return analysis_data
        except Exception as e:
            self.logger.error(f"加载预分析数据失败: {e}")
            self.logger.exception("详细错误信息：")
            return None

    # def _generate_random_point(self):
    #     """生成随机参数点"""
    #     return np.array([dim.rvs() for dim in self.space])
    def _generate_random_point(self):
        #TODO: 在被调用的时候应该说明
        """生成随机参数点，确保整数和分类参数正确处理"""
        try:
            point = {}
            
            # 整数类型参数列表
            integer_params = [
                'num_vehicles', 'num_walkers', 'sensor_number', 
                'static_object_number', 'lidar_channels', 'lidar_points'
            ]
            
            # 分类变量列表及其映射长度
            categorical_params = {}
            if hasattr(self, 'weather_mapping'):
                categorical_params['weather_preset'] = len(self.weather_mapping)
            if hasattr(self, 'vehicle_type_mapping'):
                categorical_params['vehicle_type'] = len(self.vehicle_type_mapping)
            if hasattr(self, 'sensor_type_mapping'):
                categorical_params['sensor_type'] = len(self.sensor_type_mapping)
            if hasattr(self, 'town_mapping'):
                categorical_params['town'] = len(self.town_mapping)
            if hasattr(self, 'static_object_mapping'):
                categorical_params['static_object_type'] = len(self.static_object_mapping)
            
            for dim in self.space:
                name = dim.name
                
                if name in integer_params:
                    # 整数参数
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        point[name] = int(np.random.randint(low, high + 1))
                    else:
                        point[name] = int(dim.rvs()[0])
                elif name in categorical_params:
                    # 分类变量
                    max_val = categorical_params[name] - 1  # 索引从0开始
                    point[name] = int(np.random.randint(0, max_val + 1))
                else:
                    # 浮点参数
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        point[name] = float(np.random.uniform(low, high))
                    else:
                        point[name] = float(dim.rvs()[0])
                        
            
            return self._ensure_correct_feature_types(point)
        except Exception as e:
            self.logger.warning(f"生成随机点时出错: {e}")
            # 生成最小可行的随机点
            return {dim.name: (int(dim.rvs()[0]) if dim.name in integer_params else dim.rvs()[0]) 
                    for dim in self.space}
    
    
    def _generate_random_candidates(self, n_candidates):
        """生成随机候选点"""
        candidates = []
        for _ in range(n_candidates):
            point = {}
            for dim in self.space:
                name = dim.name
                if hasattr(dim, 'bounds'):
                    low, high = dim.bounds
                    if name in ['num_vehicles', 'num_walkers', 'sensor_number', 
                                'static_object_number', 'lidar_channels', 'lidar_points']:
                        # 整数参数
                        point[name] = int(np.random.randint(low, high + 1))
                    else:
                        # 浮点参数
                        point[name] = float(np.random.uniform(low, high))
                else:
                    # 无边界信息时使用默认生成方法
                    point[name] = dim.rvs()[0]
            
            # 确保生成的点符合正确的数据类型
            point = self._ensure_correct_feature_types(point)
            candidates.append(point)
        
        return candidates


    def _generate_candidates_near_best(self, best_points, n_candidates=10):
        """在最佳点附近生成候选点"""
        try:
            candidates = []
            
            # 整数参数列表
            integer_params = [
                'num_vehicles', 'num_walkers', 'sensor_number', 
                'static_object_number', 'lidar_channels', 'lidar_points'
            ]
            
            # 分类参数列表
            categorical_params = [
                'weather_preset', 'vehicle_type', 'sensor_type', 
                'town', 'static_object_type'
            ]
            
            for _ in range(n_candidates):
                # 随机选择一个最佳点作为基础
                base_point_idx = np.random.randint(0, len(best_points))
                base_point = best_points[base_point_idx]
                
                # 转换为字典格式(如果不是)
                if not isinstance(base_point, dict):
                    base_dict = {}
                    for i, dim in enumerate(self.space):
                        if i < len(base_point):
                            base_dict[dim.name] = base_point[i]
                        else:
                            base_dict[dim.name] = 0.0
                    base_point = base_dict
                
                # 添加扰动生成新点
                new_point = {}
                
                for dim in self.space:
                    name = dim.name
                    
                    if name in integer_params:
                        # 整数参数
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            base_val = int(base_point.get(name, (low + high) // 2))
                            # 添加小的整数扰动
                            perturb = np.random.randint(-2, 3)  # -2 到 +2 的扰动
                            new_val = max(low, min(high, base_val + perturb))
                            new_point[name] = int(new_val)
                        else:
                            new_point[name] = int(base_point.get(name, 0))
                    
                    elif name in categorical_params:
                        # 分类参数 - 有0.7的概率保持不变，0.3的概率随机选择
                        if np.random.rand() < 0.7:
                            new_point[name] = base_point.get(name, 0)
                        else:
                            max_val = 0
                            if name == 'weather_preset' and hasattr(self, 'weather_mapping'):
                                max_val = len(self.weather_mapping) - 1
                            elif name == 'vehicle_type' and hasattr(self, 'vehicle_type_mapping'):
                                max_val = len(self.vehicle_type_mapping) - 1
                            elif name == 'sensor_type' and hasattr(self, 'sensor_type_mapping'):
                                max_val = len(self.sensor_type_mapping) - 1
                            elif name == 'town' and hasattr(self, 'town_mapping'):
                                max_val = len(self.town_mapping) - 1
                            elif name == 'static_object_type' and hasattr(self, 'static_object_mapping'):
                                max_val = len(self.static_object_mapping) - 1
                            
                            new_point[name] = int(np.random.randint(0, max_val + 1))
                    
                    else:
                        # 浮点参数
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            base_val = float(base_point.get(name, (low + high) / 2))
                            # 添加小的高斯扰动
                            std_dev = (high - low) * 0.1  # 参数范围的10%
                            perturb = np.random.normal(0, std_dev)
                            new_val = max(low, min(high, base_val + perturb))
                            new_point[name] = float(new_val)
                        else:
                            new_point[name] = float(base_point.get(name, 0))
                
                candidates.append(new_point)
                candidates = [self._ensure_correct_feature_types(point) for point in candidates]
            
            return candidates
        except Exception as e:
            self.logger.warning(f"在最佳点附近生成候选点时出错: {e}")
            self.logger.exception("详细错误:")
            # 回退到随机生成
            return [self._generate_random_point() for _ in range(n_candidates)]


    def _generate_lhs_candidates(self, n_candidates=15):
        """使用拉丁超立方采样生成候选点
        
        自动处理整数和分类参数类型，并确保始终返回有效的候选点
        """
        try:
            from sklearn.preprocessing import MinMaxScaler
            
            try:
                from pyDOE import lhs
                # 生成拉丁超立方样本
                samples = lhs(len(self.space), samples=n_candidates)
            except ImportError:
                try:
                    from scipy.stats import qmc
                    sampler = qmc.LatinHypercube(d=len(self.space))
                    samples = sampler.random(n=n_candidates)
                except ImportError:
                    self.logger.warning("拉丁超立方采样库不可用，使用随机采样代替")
                    return [self._generate_random_point() for _ in range(n_candidates)]
            
            # 整数参数列表
            integer_params = [
                'num_vehicles', 'num_walkers', 'sensor_number', 
                'static_object_number', 'lidar_channels', 'lidar_points'
            ]
            
            # 分类参数列表及其映射长度
            categorical_params = {}
            if hasattr(self, 'weather_mapping'):
                categorical_params['weather_preset'] = len(self.weather_mapping)
            if hasattr(self, 'vehicle_type_mapping'):
                categorical_params['vehicle_type'] = len(self.vehicle_type_mapping)
            if hasattr(self, 'sensor_type_mapping'):
                categorical_params['sensor_type'] = len(self.sensor_type_mapping)
            if hasattr(self, 'town_mapping'):
                categorical_params['town'] = len(self.town_mapping)
            if hasattr(self, 'static_object_mapping'):
                categorical_params['static_object_type'] = len(self.static_object_mapping)
            
            # 转换为实际参数范围
            candidates = []
            for sample in samples:
                point = {}
                for i, dim in enumerate(self.space):
                    name = dim.name
                    if name in integer_params:
                        # 整数参数
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            value = int(low + sample[i] * (high - low + 1))
                        else:
                            value = int(dim.rvs()[0])
                        point[name] = value
                    elif name in categorical_params:
                        # 分类参数
                        max_val = categorical_params[name] - 1
                        value = int(sample[i] * (max_val + 1))
                        point[name] = min(value, max_val)
                    else:
                        # 浮点参数
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            value = low + sample[i] * (high - low)
                        else:
                            value = dim.rvs()[0]
                        point[name] = value
                candidates.append(point)
            
            # 确保生成足够的候选点
            if len(candidates) < n_candidates:
                self.logger.warning(f"LHS采样仅生成了{len(candidates)}个点，补充随机点")
                additional_points = [self._generate_random_point() 
                                for _ in range(n_candidates - len(candidates))]
                candidates.extend(additional_points)
            
            # 在函数末尾返回前添加
            candidates = [self._ensure_correct_feature_types(point) for point in candidates]
            return candidates
        except Exception as e:
            self.logger.warning(f"使用拉丁超立方采样生成候选点时出错: {e}")
            self.logger.exception("详细错误:")
            # 回退到随机生成
            return [self._generate_random_point() for _ in range(n_candidates)]
        
        


    def _generate_thompson_candidates(self, n_candidates=15):
        """使用Thompson采样生成候选点
        
        即使在出现错误的情况下也确保返回指定数量的候选点
        """
        candidates = []
        
        # 整数参数列表
        integer_params = [
            'num_vehicles', 'num_walkers', 'sensor_number', 
            'static_object_number', 'lidar_channels', 'lidar_points'
        ]
        
        # 分类参数列表及其映射长度
        categorical_params = {}
        if hasattr(self, 'weather_mapping'):
            categorical_params['weather_preset'] = len(self.weather_mapping)
        if hasattr(self, 'vehicle_type_mapping'):
            categorical_params['vehicle_type'] = len(self.vehicle_type_mapping)
        if hasattr(self, 'sensor_type_mapping'):
            categorical_params['sensor_type'] = len(self.sensor_type_mapping)
        if hasattr(self, 'town_mapping'):
            categorical_params['town'] = len(self.town_mapping)
        if hasattr(self, 'static_object_mapping'):
            categorical_params['static_object_type'] = len(self.static_object_mapping)
        
        try:
            if hasattr(self, 'gp') and self.gp is not None and len(self.X_history) >= 10:
                # 使用高斯过程模型进行Thompson采样
                for _ in range(n_candidates):
                    # 生成随机候选点
                    point_dict = {}
                    for i, dim in enumerate(self.space):
                        name = dim.name
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            if name in integer_params:
                                value = int(np.random.randint(low, high + 1))
                            elif name in categorical_params:
                                max_val = categorical_params[name] - 1
                                value = int(np.random.randint(0, max_val + 1))
                            else:
                                value = float(np.random.uniform(low, high))
                        else:
                            if name in integer_params:
                                value = int(dim.rvs()[0])
                            elif name in categorical_params:
                                max_val = categorical_params[name] - 1
                                value = int(np.random.randint(0, max_val + 1))
                            else:
                                value = float(dim.rvs()[0])
                        
                        point_dict[name] = value
                    
                    # 将字典转换为数组用于GP预测
                    point_array = np.array([point_dict.get(dim.name, 0.0) for dim in self.space])
                    
                    # 使用GP预测均值和方差
                    try:
                        mean, std = self.gp.predict(point_array.reshape(1, -1), return_std=True)
                    except Exception as e:
                        self.logger.warning(f"GP预测单个点时出错: {e}")
                    
                    candidates.append(point_dict)
            else:
                self.logger.warning("GP模型不可用或样本不足，使用随机采样代替")
                candidates = [self._generate_random_point() for _ in range(n_candidates)]
        except Exception as e:
            self.logger.warning(f"使用Thompson采样生成候选点时出错: {e}")
            self.logger.exception("详细错误:")
            # 回退到随机生成
            candidates = [self._generate_random_point() for _ in range(n_candidates)]
        
        # 确保即使发生错误也有足够的候选点
        if len(candidates) == 0:
            self.logger.warning("Thompson采样未生成任何候选点，使用随机采样代替")
            candidates = [self._generate_random_point() for _ in range(n_candidates)]
        elif len(candidates) < n_candidates:
            self.logger.warning(f"Thompson采样仅生成了{len(candidates)}个候选点，补充随机点")
            additional_points = [self._generate_random_point() for _ in range(n_candidates - len(candidates))]
            candidates.extend(additional_points)
        
        return candidates
    
    
    
    def _generate_diverse_candidates(self, n_candidates, X):
        """生成多样化的候选点"""
        candidates = []
        
        # 使用cell-based划分策略
        n_dims = len(self.space)
        
        # 如果历史点超过20个，尝试找出未访问的区域
        if len(X) > 20:
            for _ in range(min(n_candidates // 2, 10)):
                cell = self._find_unvisited_cell()
                if cell is not None:
                    point_dict = self._generate_point_in_cell(cell)
                    candidates.append(point_dict)
        
        # 补充剩余的点，优先使用拉丁超立方采样
        remaining = n_candidates - len(candidates)
        if remaining > 0:
            try:
                # 尝试使用拉丁超立方采样
                from scipy.stats import qmc
                
                sampler = qmc.LatinHypercube(d=n_dims)
                samples = sampler.random(n=remaining)
                
                for sample in samples:
                    point = {}
                    for i, dim in enumerate(self.space):
                        # 将[0,1]映射到参数空间
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            if dim.name in ['num_vehicles', 'num_walkers', 'sensor_number', 
                                            'static_object_number', 'lidar_channels', 'lidar_points']:
                                # 整数参数
                                value = int(low + sample[i] * (high - low + 1))
                            elif dim.name in ['weather_preset', 'vehicle_type', 'sensor_type', 
                                            'town', 'static_object_type']:
                                # 分类参数
                                max_val = 0
                                if dim.name == 'weather_preset' and hasattr(self, 'weather_mapping'):
                                    max_val = len(self.weather_mapping) - 1
                                elif dim.name == 'vehicle_type' and hasattr(self, 'vehicle_type_mapping'):
                                    max_val = len(self.vehicle_type_mapping) - 1
                                elif dim.name == 'sensor_type' and hasattr(self, 'sensor_type_mapping'):
                                    max_val = len(self.sensor_type_mapping) - 1
                                elif dim.name == 'town' and hasattr(self, 'town_mapping'):
                                    max_val = len(self.town_mapping) - 1
                                elif dim.name == 'static_object_type' and hasattr(self, 'static_object_mapping'):
                                    max_val = len(self.static_object_mapping) - 1
                                
                                value = int(sample[i] * (max_val + 1))
                                value = min(value, max_val)  # 确保不超过最大值
                            else:
                                # 浮点参数
                                value = low + sample[i] * (high - low)
                        else:
                            # 使用rvs函数生成随机值
                            value = dim.rvs()[0]
                        
                        point[dim.name] = value
                    
                    candidates.append(point)
            except ImportError:
                # 如果scipy.stats.qmc不可用，使用随机采样
                self.logger.warning("LHS采样不可用，使用随机采样代替")
                for _ in range(remaining):
                    candidates.append(self._generate_random_point())
        
        return candidates


    def _train_clustered_gp_models(self):
        """训练基于聚类的多个高斯过程模型，确保训练测试集分离"""
        try:
            self.logger.info("开始训练基于聚类的多模型高斯过程回归器")
            
            # 检查是否已有聚类结果
            if not hasattr(self, 'cluster_stats') or not self.cluster_stats:
                self.logger.warning("没有有效的聚类结果，无法训练多模型")
                return False
                
            from sklearn.gaussian_process import GaussianProcessRegressor
            from sklearn.gaussian_process.kernels import Matern, ConstantKernel
            from sklearn.preprocessing import StandardScaler
            from sklearn.model_selection import train_test_split
            
            # 初始化模型和标准化器
            self.clustered_gps = {}
            self.cluster_scalers_X = []  # X标准化器列表
            self.cluster_scalers_y = []  # y标准化器列表
            self.cluster_stats_for_gp = []  # 用于GP的统计信息
            self.test_data = {}  # 存储每个聚类的测试数据，供后续评估使用
            
            # 对每个聚类训练模型
            for i, cluster in enumerate(self.cluster_stats):
                if 'point_indices' not in cluster or cluster.get('size', 0) < 5:
                    continue
                    
                # 获取数据点
                point_indices = cluster['point_indices']
                X_cluster = []
                y_cluster = []
                
                for idx in point_indices:
                    if idx < len(self.X_history) and idx < len(self.y_history):
                        if isinstance(self.X_history[idx], dict):
                            x_values = [self.X_history[idx].get(dim.name, 0) for dim in self.space]
                            X_cluster.append(x_values)
                        else:
                            X_cluster.append(self.X_history[idx])
                        y_cluster.append(self.y_history[idx])
                
                if len(X_cluster) < 5:
                    continue
                    
                # 转换为numpy数组
                X_cluster = np.array(X_cluster)
                y_cluster = np.array(y_cluster)
                
                # 分离训练集和测试集（20%作为测试集）
                X_train, X_test, y_train, y_test = train_test_split(
                    X_cluster, y_cluster, test_size=0.2, random_state=42)
                
                # 保存测试数据供后续评估
                self.test_data[i] = {
                    'X_test': X_test,
                    'y_test': y_test
                }
                
                # 标准化训练数据
                scaler_X = StandardScaler()
                scaler_y = StandardScaler()
                
                X_train_scaled = scaler_X.fit_transform(X_train)
                y_train_scaled = scaler_y.fit_transform(y_train.reshape(-1, 1)).ravel()
                
                # 保存标准化器
                self.cluster_scalers_X.append(scaler_X)
                self.cluster_scalers_y.append(scaler_y)
                
                # 训练GP模型
                kernel = ConstantKernel(1.0) * Matern(length_scale=1.0, nu=2.5)
                gp = GaussianProcessRegressor(
                    kernel=kernel,
                    alpha=1e-6,
                    normalize_y=True,
                    n_restarts_optimizer=5,
                    random_state=42
                )
                
                try:
                    gp.fit(X_train_scaled, y_train_scaled)
                    
                    self.clustered_gps[i] = {
                        'model': gp,
                        'error_type': cluster.get('main_error_type', 'unknown'),
                        'size': len(X_train),  # 使用训练集大小
                        'mean_error': np.mean(y_train)
                    }
                    
                    self.cluster_stats_for_gp.append({
                        'mean_error': float(np.mean(y_train)),
                        'size': len(X_train),
                        'error_type': cluster.get('main_error_type', 'unknown')
                    })
                    
                    self.logger.info(f"成功训练聚类 {i} 的GP模型，训练样本: {len(X_train)}，测试样本: {len(X_test)}")
                    
                except Exception as e:
                    self.logger.warning(f"训练聚类 {i} 的GP模型失败: {e}")
                    # 移除对应的标准化器和测试数据
                    if len(self.cluster_scalers_X) > 0:
                        self.cluster_scalers_X.pop()
                        self.cluster_scalers_y.pop()
                    if i in self.test_data:
                        del self.test_data[i]
            
            # 检查是否有成功训练的模型
            if len(self.clustered_gps) > 0:
                self.logger.info(f"成功训练了 {len(self.clustered_gps)} 个聚类GP模型")
                
                # 使用统一的评估函数评估模型性能
                # 不传入X_test和y_test，自动使用各模型自己的测试数据
                self._evaluate_gp_models(evaluate_clustered=True)
                
                return True
            else:
                self.logger.warning("没有成功训练任何聚类GP模型")
                return False
                
        except Exception as e:
            self.logger.error(f"训练聚类GP模型失败: {e}")
            import traceback
            self.logger.debug(f"详细错误: {traceback.format_exc()}")
            return False        
        
            
    def _identify_cluster_key_params(self, X_cluster, y_cluster):
        """识别聚类中的关键参数"""
        try:
            if len(X_cluster) < 5 or X_cluster.shape[1] == 0:
                return []
                
            # 计算每个参数与错误的相关性
            correlations = []
            for i in range(X_cluster.shape[1]):
                corr = np.corrcoef(X_cluster[:, i], y_cluster)[0, 1]
                if np.isnan(corr):
                    corr = 0.0
                correlations.append((i, abs(corr)))
            
            # 排序并选择最相关的参数
            correlations.sort(key=lambda x: x[1], reverse=True)
            
            # 选择相关性最高的几个参数，或相关性超过阈值的参数
            key_params = []
            for i, corr in correlations:
                if corr > 0.2 or len(key_params) < 3:  # 至少选3个参数，或相关性>0.2的参数
                    param_name = self.space[i].name
                    
                    # 计算参数范围
                    values = X_cluster[:, i]
                    param_range = (float(np.min(values)), float(np.max(values)))
                    
                    key_params.append((param_name, param_range, float(corr)))
                    
                    if len(key_params) >= 5:  # 最多选5个参数
                        break
            
            return key_params
            
        except Exception as e:
            self.logger.warning(f"识别关键参数失败: {e}")
            return []



    def _evaluate_gp_models(self, X_test=None, y_test=None, models=None, evaluate_clustered=False):
        """评估高斯过程模型的性能
        
        Args:
            X_test: 可选，测试特征数据，如果为None且evaluate_clustered=True，将使用各模型自己的测试数据
            y_test: 可选，测试标签数据，如果为None且evaluate_clustered=True，将使用各模型自己的测试数据
            models: 可选，GP模型字典，如果为None则使用self.gp或self.clustered_gps
            evaluate_clustered: 是否评估聚类模型
            
        Returns:
            包含性能指标的字典
        """
        import numpy as np
        from sklearn.metrics import r2_score, mean_squared_error, mean_absolute_error
        
        results = {
            'overall': {},
            'models': {}
        }
        
        try:
            # 确定要评估的模型
            if models is not None:
                eval_models = models
            elif evaluate_clustered and hasattr(self, 'clustered_gps') and self.clustered_gps:
                eval_models = self.clustered_gps
            elif hasattr(self, 'gp') and self.gp is not None:
                eval_models = {'base_model': self.gp}
            else:
                self.logger.warning("没有可用的GP模型进行评估")
                return results
            
            # 是否使用各模型自己的测试数据
            use_own_test_data = (X_test is None or y_test is None) and evaluate_clustered and hasattr(self, 'test_data')
            
            # 缓存所有预测和真实值，用于计算整体性能
            all_predictions = []
            all_true_values = []
            
            # 评估每个模型
            for model_name, model_data in eval_models.items():
                try:
                    # 确保正确访问模型对象
                    if isinstance(model_data, dict) and 'model' in model_data:
                        model_obj = model_data['model']
                    elif isinstance(model_data, (int, float, str, bool)):
                        self.logger.warning(f"模型 {model_name} 类型错误: {type(model_data)}")
                        continue
                    else:
                        model_obj = model_data
                    
                    # 检查模型是否有predict方法
                    if not hasattr(model_obj, 'predict'):
                        self.logger.warning(f"模型 {model_name} 没有predict方法: {type(model_obj)}")
                        continue
                    
                    # 确定使用哪个测试集
                    if use_own_test_data:
                        # 尝试将model_name转换为整数索引
                        try:
                            cluster_idx = int(model_name) if isinstance(model_name, str) else model_name
                        except ValueError:
                            cluster_idx = None
                        
                        if cluster_idx is not None and cluster_idx in self.test_data:
                            # 使用该模型自己的测试数据
                            test_data = self.test_data[cluster_idx]
                            curr_X_test = test_data['X_test']
                            curr_y_test = test_data['y_test']
                            
                            # 获取对应的标准化器
                            if hasattr(self, 'cluster_scalers_X') and cluster_idx < len(self.cluster_scalers_X):
                                scaler_X = self.cluster_scalers_X[cluster_idx]
                                scaler_y = self.cluster_scalers_y[cluster_idx]
                                
                                # 标准化测试数据
                                X_test_scaled = scaler_X.transform(curr_X_test)
                                
                                # 预测和逆标准化
                                y_pred_scaled = model_obj.predict(X_test_scaled)
                                y_pred = scaler_y.inverse_transform(y_pred_scaled.reshape(-1, 1)).ravel()
                            else:
                                # 无标准化器，直接预测
                                y_pred = model_obj.predict(curr_X_test)
                        else:
                            self.logger.warning(f"模型 {model_name} 没有对应的测试数据")
                            continue
                    else:
                        # 使用提供的测试数据
                        # 首先转换测试数据格式
                        if isinstance(X_test, list) and isinstance(X_test[0], dict):
                            # 将字典列表转换为数组
                            curr_X_test = np.array([[point.get(dim.name, 0.0) for dim in self.space] for point in X_test])
                        else:
                            curr_X_test = np.array(X_test)
                        
                        curr_y_test = np.array(y_test)
                        
                        # 检查是否需要应用标准化
                        cluster_idx = None
                        try:
                            cluster_idx = int(model_name) if isinstance(model_name, str) else model_name
                        except ValueError:
                            pass
                        
                        if evaluate_clustered and cluster_idx is not None and hasattr(self, 'cluster_scalers_X') and cluster_idx < len(self.cluster_scalers_X):
                            # 使用对应的标准化器
                            scaler_X = self.cluster_scalers_X[cluster_idx]
                            scaler_y = self.cluster_scalers_y[cluster_idx]
                            
                            # 标准化测试数据
                            X_test_scaled = scaler_X.transform(curr_X_test)
                            
                            # 预测和逆标准化
                            y_pred_scaled = model_obj.predict(X_test_scaled)
                            y_pred = scaler_y.inverse_transform(y_pred_scaled.reshape(-1, 1)).ravel()
                        else:
                            # 直接预测
                            y_pred = model_obj.predict(curr_X_test)
                    
                    # 计算性能指标
                    r2 = r2_score(curr_y_test, y_pred)
                    mse = mean_squared_error(curr_y_test, y_pred)
                    mae = mean_absolute_error(curr_y_test, y_pred)
                    
                    model_results = {
                        'r2': r2,
                        'mse': mse,
                        'mae': mae,
                        'samples': len(curr_y_test)
                    }
                    
                    results['models'][model_name] = model_results
                    
                    # 收集用于整体评估的数据
                    all_predictions.extend(y_pred)
                    all_true_values.extend(curr_y_test)
                    
                    self.logger.info(f"模型 {model_name} 性能: R² = {r2:.4f}, MSE = {mse:.4f}, MAE = {mae:.4f}")
                    
                except Exception as e:
                    self.logger.warning(f"评估模型 {model_name} 时出错: {e}")
                    results['models'][model_name] = {'error': str(e)}
            
            # 计算整体性能
            if all_predictions and all_true_values:
                all_predictions = np.array(all_predictions)
                all_true_values = np.array(all_true_values)
                
                overall_r2 = r2_score(all_true_values, all_predictions)
                overall_mse = mean_squared_error(all_true_values, all_predictions)
                overall_mae = mean_absolute_error(all_true_values, all_predictions)
                
                results['overall'] = {
                    'r2': overall_r2,
                    'mse': overall_mse,
                    'mae': overall_mae,
                    'models_evaluated': len(eval_models)
                }
                
                self.logger.info(f"整体模型性能: R² = {overall_r2:.4f}, MSE = {overall_mse:.4f}, MAE = {overall_mae:.4f}")
            
        except Exception as e:
            self.logger.warning(f"评估GP模型时出错: {e}")
            results['error'] = str(e)
        
        return results


    # 2. 修复使用聚类GP模型生成候选点的函数
    def _generate_clustered_gp_candidates(self, n_candidates=20):
        """使用聚类GP模型生成候选点"""
        try:
            if not hasattr(self, 'clustered_gps') or not self.clustered_gps:
                self.logger.warning("没有可用的聚类GP模型，无法生成候选点")
                return []
            
            import numpy as np
            from sklearn.preprocessing import StandardScaler
            
            candidates = []
            cluster_candidates = {}
            
            # 为每个聚类生成候选点
            for cluster_id, cluster_model in self.clustered_gps.items():
                try:
                    # 修复：确保获取正确的模型对象
                    if isinstance(cluster_model, dict) and 'model' in cluster_model:
                        model = cluster_model['model']
                    elif isinstance(cluster_model, (int, float, str, bool)):
                        self.logger.warning(f"聚类{cluster_id}的模型类型错误: {type(cluster_model)}")
                        continue
                    else:
                        model = cluster_model
                    
                    # 为每个聚类生成候选点
                    cluster_candidates[cluster_id] = []
                    
                    # 生成随机点
                    X_random = self._generate_random_candidates(n_candidates * 3)
                    X_random_array = []
                    
                    for point in X_random:
                        x_values = [point.get(dim.name, 0) for dim in self.space]
                        X_random_array.append(x_values)
                    
                    X_random_array = np.array(X_random_array)
                    
                    # 使用对应聚类的标准化器
                    if hasattr(self, 'cluster_scalers_X') and cluster_id < len(self.cluster_scalers_X):
                        X_scaled = self.cluster_scalers_X[cluster_id].transform(X_random_array)
                    else:
                        # 如果没有保存标准化器，就临时创建一个
                        scaler = StandardScaler()
                        X_scaled = scaler.fit_transform(X_random_array)
                    
                    # 预测性能并排序
                    try:
                        y_pred, std_pred = model.predict(X_scaled, return_std=True)
                        
                        # 使用置信上界 (UCB)
                        acq_values = y_pred + 1.96 * std_pred
                        
                        # 按获取值排序（从小到大）
                        indices = np.argsort(acq_values)
                        
                        # 选择前n_candidates个
                        for idx in indices[:n_candidates]:
                            candidates.append(X_random[idx])
                            cluster_candidates[cluster_id].append(X_random[idx])
                    except Exception as e:
                        self.logger.warning(f"聚类{cluster_id}的候选点评估出错: {e}")
                        # 如果预测失败，直接添加随机点
                        for idx in range(min(n_candidates, len(X_random))):
                            candidates.append(X_random[idx])
                            cluster_candidates[cluster_id].append(X_random[idx])
                except Exception as e:
                    self.logger.warning(f"生成聚类{cluster_id}的候选点时出错: {e}")
            
            return candidates
        except Exception as e:
            self.logger.warning(f"生成聚类GP候选点失败: {e}")
            return []

    # 3. 修复预测聚类GP模型的函数
    def _predict_with_clustered_gps(self, X_test):
        """使用聚类GP模型进行预测"""
        try:
            if not hasattr(self, 'clustered_gps') or not self.clustered_gps:
                raise ValueError("聚类GP模型未训练")
            
            # 格式化输入
            if isinstance(X_test, dict):
                X_test_array = np.array([X_test.get(dim.name, 0.0) for dim in self.space]).reshape(1, -1)
            elif len(X_test.shape) == 1:
                X_test_array = X_test.reshape(1, -1)
            else:
                X_test_array = X_test
            
            # 计算到每个聚类中心的距离
            distances = []
            for center in self.cluster_centers:
                # 计算欧氏距离
                dist = np.sqrt(np.sum((X_test_array - center) ** 2, axis=1))
                distances.append(dist)
            
            # 将距离转换为权重
            # 使用softmax函数: exp(-d_i) / sum(exp(-d_j))
            distances = np.array(distances)  # [n_clusters, n_samples]
            neg_distances = -distances  # 负距离，越近权重越大
            weights = np.exp(neg_distances)
            # 归一化权重，每个样本的所有聚类权重和为1
            weights_sum = np.sum(weights, axis=0)
            weights = weights / weights_sum
            
            # 初始化预测结果数组
            n_samples = X_test_array.shape[0]
            means = np.zeros(n_samples)
            variances = np.zeros(n_samples)
            
            # 循环每个聚类GP进行预测并加权
            for i, cluster_id in enumerate(self.clustered_gps.keys()):
                cluster_model = self.clustered_gps[cluster_id]
                
                # 修复：确保获取正确的模型对象
                if isinstance(cluster_model, dict) and 'model' in cluster_model:
                    model = cluster_model['model']
                elif isinstance(cluster_model, (int, float, str, bool)):
                    self.logger.warning(f"聚类{cluster_id}的模型类型错误: {type(cluster_model)}")
                    continue
                else:
                    model = cluster_model
                
                # 标准化输入
                if hasattr(self, 'cluster_scalers_X') and i < len(self.cluster_scalers_X):
                    X_test_scaled = self.cluster_scalers_X[i].transform(X_test_array)
                else:
                    # 临时标准化
                    scaler = StandardScaler()
                    X_test_scaled = scaler.fit_transform(X_test_array)
                
                # GP预测
                try:
                    mean_scaled, std_scaled = model.predict(X_test_scaled, return_std=True)
                    
                    # 逆标准化输出
                    if hasattr(self, 'cluster_scalers_y') and i < len(self.cluster_scalers_y):
                        mean = self.cluster_scalers_y[i].inverse_transform(mean_scaled.reshape(-1, 1)).ravel()
                        std = std_scaled * self.cluster_scalers_y[i].scale_
                    else:
                        # 无法逆标准化，直接使用原值
                        mean = mean_scaled
                        std = std_scaled
                    
                    # 加权组合预测结果
                    means += weights[i] * mean
                    # 方差需要考虑簇内方差和簇间方差
                    variances += weights[i] * (std**2 + mean**2)
                except Exception as e:
                    self.logger.warning(f"聚类{cluster_id}的预测出错: {e}")
                    # 出错时不贡献到预测结果
            
            # 调整最终方差
            variances -= means**2  # 减去均值的平方，得到真正的方差
            
            return means, np.sqrt(variances)
            
        except Exception as e:
            self.logger.error(f"使用聚类GP模型预测失败: {e}")
            # 预测失败时返回默认值
            n_samples = X_test_array.shape[0]
            return np.zeros(n_samples), np.ones(n_samples)



    def _generate_test_diversity_candidates(self, n_candidates=30):
        """生成高多样性的测试候选点，专为bug发现设计"""
        try:
            candidates = []
            
            # 1. 未访问区域强制探索 (30%)
            n_unvisited = int(n_candidates * 0.3)
            unvisited_candidates = []
            for _ in range(n_unvisited * 2):  # 尝试更多次以确保获得足够数量
                cell = self._find_unvisited_cell()
                if cell is not None:
                    point = self._generate_point_in_cell(cell)
                    unvisited_candidates.append(point)
                if len(unvisited_candidates) >= n_unvisited:
                    break
            candidates.extend(unvisited_candidates[:n_unvisited])
            
            # 2. LHS采样确保参数空间覆盖 (20%)
            n_lhs = int(n_candidates * 0.2)
            lhs_candidates = self._generate_lhs_candidates(n_lhs)
            candidates.extend(lhs_candidates)
            
            # 3. 边界值测试 (15%)
            n_boundary = int(n_candidates * 0.15)
            boundary_candidates = self._generate_boundary_test_points(n_boundary)
            candidates.extend(boundary_candidates)
            
            # 4. 已知bug区域附近探索 (20%)
            if hasattr(self, 'cluster_stats_for_gp') and self.cluster_stats_for_gp:
                n_near_bugs = int(n_candidates * 0.2)
                near_bug_candidates = self._generate_near_bug_candidates(n_near_bugs)
                candidates.extend(near_bug_candidates)
            
            # 5. 基于贝叶斯不确定性的探索 (15%)
            if hasattr(self, 'clustered_gps') and self.clustered_gps:
                n_uncertainty = int(n_candidates * 0.15)
                uncertainty_candidates = self._generate_uncertainty_candidates(n_uncertainty)
                candidates.extend(uncertainty_candidates)
            
            # 填补不足数量
            remaining = n_candidates - len(candidates)
            if remaining > 0:
                random_candidates = self._generate_random_candidates(remaining)
                candidates.extend(random_candidates)
            
            self.logger.info(f"生成了 {len(candidates)} 个多样性测试候选点")
            candidates = [self._ensure_correct_feature_types(point) for point in candidates]

            return candidates[:n_candidates]  # 确保返回正确数量
        
        except Exception as e:
            self.logger.warning(f"生成多样性测试候选点失败: {e}")
            self.logger.exception("详细错误：")
            return self._generate_random_candidates(n_candidates)

    def _generate_boundary_test_points(self, n_candidates):
        """生成边界值测试点"""
        try:
            candidates = []
            
            for _ in range(n_candidates):
                point = {}
                # 选择1-3个参数设置为边界值
                n_boundary_params = np.random.randint(1, 4)
                boundary_params = np.random.choice(len(self.space), n_boundary_params, replace=False)
                
                for i, dim in enumerate(self.space):
                    if i in boundary_params:
                        # 设置边界值
                        if hasattr(dim, 'bounds'):
                            low, high = dim.bounds
                            # 有80%几率选择极端值，20%几率选择接近边界的值
                            if np.random.rand() < 0.8:
                                # 极端值
                                value = low if np.random.rand() < 0.5 else high
                            else:
                                # 接近边界值
                                if np.random.rand() < 0.5:
                                    value = low + (high - low) * 0.05  # 接近下边界
                                else:
                                    value = high - (high - low) * 0.05  # 接近上边界
                        else:
                            # 无边界信息时使用随机值
                            value = dim.rvs()[0]
                    else:
                        # 其他参数随机设置
                        value = self._generate_random_parameter_value(dim)
                        
                    point[dim.name] = value
                    
                candidates.append(point)
            
            candidates = [self._ensure_correct_feature_types(point) for point in candidates]

            return candidates
        except Exception as e:
            self.logger.warning(f"生成边界测试点失败: {e}")
            return []

    def _generate_random_parameter_value(self, dim):
        """为单个参数生成随机值，考虑参数类型"""
        try:
            # 检查是否是整数参数
            is_integer = dim.name in ['num_vehicles', 'num_walkers', 'sensor_number', 
                                     'static_object_number', 'lidar_channels', 'lidar_points']
            
            # 检查是否是分类参数
            categorical_params = {}
            if hasattr(self, 'weather_mapping'):
                categorical_params['weather_preset'] = len(self.weather_mapping)
            if hasattr(self, 'vehicle_type_mapping'):
                categorical_params['vehicle_type'] = len(self.vehicle_type_mapping)
            if hasattr(self, 'sensor_type_mapping'):
                categorical_params['sensor_type'] = len(self.sensor_type_mapping)
            if hasattr(self, 'town_mapping'):
                categorical_params['town'] = len(self.town_mapping)
            if hasattr(self, 'static_object_mapping'):
                categorical_params['static_object_type'] = len(self.static_object_mapping)
            
            is_categorical = dim.name in categorical_params
            
            if hasattr(dim, 'bounds'):
                low, high = dim.bounds
                if is_integer:
                    return int(np.random.randint(low, high + 1))
                elif is_categorical:
                    max_val = categorical_params.get(dim.name, 1) - 1
                    return int(np.random.randint(0, max_val + 1))
                else:
                    return float(np.random.uniform(low, high))
            else:
                rvs = dim.rvs()
                if is_integer:
                    return int(rvs[0] if isinstance(rvs, (list, np.ndarray)) else rvs)
                elif is_categorical:
                    max_val = categorical_params.get(dim.name, 1) - 1
                    return int(np.random.randint(0, max_val + 1))
                else:
                    return float(rvs[0] if isinstance(rvs, (list, np.ndarray)) else rvs)
        except Exception as e:
            self.logger.warning(f"生成随机参数值失败: {e}")
            return 0

    def _generate_near_bug_candidates(self, n_candidates):
        """在已知bug区域附近生成测试点"""
        try:
            candidates = []
            
            if not hasattr(self, 'cluster_stats_for_gp') or not self.cluster_stats_for_gp:
                return candidates
                
            # 计算每个簇应该生成的候选点数量，确保较小的簇也至少有1个点
            n_clusters = len(self.cluster_stats_for_gp)
            points_per_cluster = [max(1, int(n_candidates / n_clusters))] * n_clusters
            
            # 按错误大小给簇分配权重
            cluster_errors = [stats.get('mean_error', 0) for stats in self.cluster_stats_for_gp]
            if sum(cluster_errors) > 0:
                # 归一化错误值
                normalized_errors = np.array(cluster_errors) / sum(cluster_errors)
                # 调整每簇点数
                points_per_cluster = [int(n_candidates * err) + 1 for err in normalized_errors]
                # 确保总和不超过n_candidates
                if sum(points_per_cluster) > n_candidates:
                    # 从最小的簇中逐渐减少
                    sorted_indices = np.argsort(points_per_cluster)
                    for idx in sorted_indices:
                        if sum(points_per_cluster) <= n_candidates:
                            break
                        if points_per_cluster[idx] > 1:
                            points_per_cluster[idx] -= 1
            
            # 为每个簇生成指定数量的点
            for i, stats in enumerate(self.cluster_stats_for_gp):
                n_points = points_per_cluster[i]
                if n_points <= 0 or 'key_params' not in stats:
                    continue
                    
                key_params = stats['key_params']
                if not key_params:
                    continue
                    
                # 获取聚类中心 (如果可用)
                if hasattr(self, 'cluster_centers') and i < len(self.cluster_centers):
                    center_array = self.cluster_centers[i]
                    center = {}
                    for j, dim in enumerate(self.space):
                        if j < len(center_array):
                            center[dim.name] = center_array[j]
                else:
                    # 无中心信息，创建随机中心
                    center = self._generate_random_point()
                
                # 在中心点附近生成候选点
                for _ in range(n_points):
                    point = {}
                    for j, dim in enumerate(self.space):
                        name = dim.name
                        
                        # 检查这个参数是否是关键参数
                        is_key = False
                        key_importance = 0.0
                        key_range = None
                        
                        for param_name, param_range, importance in key_params:
                            if param_name == name:
                                is_key = True
                                key_importance = importance
                                key_range = param_range
                                break
                        
                        if is_key and key_range is not None:
                            # 对关键参数，在其有效范围内生成值
                            low, high = key_range
                            
                            # 根据重要性决定如何生成值
                            if key_importance > 0.7:  # 高重要性，严格遵循范围
                                range_extension = 0.1  # 允许稍微超出范围
                            else:  # 低重要性，可以更自由
                                range_extension = 0.3
                                
                            # 扩展范围
                            range_width = high - low
                            extended_low = max(dim.bounds[0] if hasattr(dim, 'bounds') else -1e10, 
                                            low - range_width * range_extension)
                            extended_high = min(dim.bounds[1] if hasattr(dim, 'bounds') else 1e10, 
                                             high + range_width * range_extension)
                            
                            # 生成参数值
                            if name in ['num_vehicles', 'num_walkers', 'sensor_number', 
                                      'static_object_number', 'lidar_channels', 'lidar_points']:
                                value = int(np.random.randint(extended_low, extended_high + 1))
                            else:
                                value = float(np.random.uniform(extended_low, extended_high))
                        elif name in center:
                            # 非关键参数，基于中心点添加扰动
                            base = center[name]
                            
                            if hasattr(dim, 'bounds'):
                                low, high = dim.bounds
                                range_width = high - low
                                # 添加较大扰动
                                perturb = np.random.uniform(-0.3, 0.3) * range_width
                                value = base + perturb
                                # 确保在有效范围内
                                value = max(low, min(high, value))
                                
                                # 整数参数取整
                                if name in ['num_vehicles', 'num_walkers', 'sensor_number', 
                                          'static_object_number', 'lidar_channels', 'lidar_points']:
                                    value = int(round(value))
                            else:
                                # 无边界信息，添加百分比扰动
                                perturb_factor = 1.0 + np.random.uniform(-0.3, 0.3)
                                value = base * perturb_factor
                           
                            point[dim.name] = value
                            
                        candidates.append(point)
                candidates = [self._ensure_correct_feature_types(point) for point in candidates]

                return candidates
        except Exception as e:
            self.logger.warning(f"生成bug区域附近点失败: {e}")
            self.logger.exception("详细错误：")
            return []

    def _generate_uncertainty_candidates(self, n_candidates):
        """根据聚类GP模型的不确定性生成候选点"""
        try:
            if not hasattr(self, 'clustered_gps') or not self.clustered_gps:
                return []
                
            # 生成随机测试点池
            n_test = 5000
            test_points_dict = [self._generate_random_point() for _ in range(n_test)]
            
            # 将字典转换为数组
            test_points_array = []
            for point in test_points_dict:
                point_array = np.array([point.get(dim.name, 0.0) for dim in self.space])
                test_points_array.append(point_array)
            
            X_test = np.vstack(test_points_array)
            
            # 使用聚类GP预测
            _, stds = self._predict_with_clustered_gps(X_test)
            
            # 选择不确定性最高的点
            top_indices = np.argsort(-stds)[:n_candidates]
            
            # 转换回字典格式
            candidates = [test_points_dict[i] for i in top_indices]
            
            candidates = [self._ensure_correct_feature_types(point) for point in candidates]

            return candidates
        except Exception as e:
            self.logger.warning(f"生成不确定性候选点失败: {e}")
            self.logger.exception("详细错误：")
            return []

    def _evaluate_test_diversity(self, point, history, candidates):
        """评估测试点的多样性分数，专为bug发现设计
        
        评分关注:
        1. 与历史点的距离 (避免重复测试)
        2. 与其他候选点的距离 (避免候选点聚集)
        3. 参数空间覆盖 (关注未测试的参数组合)
        4. 不确定性 (如果有聚类GP模型)
        """
        try:
            # 将point转换为数组
            if isinstance(point, dict):
                point_array = np.array([point.get(dim.name, 0.0) for dim in self.space])
            else:
                point_array = np.array(point)
                
            # 确保历史点格式统一
            history_arrays = []
            for hist_point in history:
                if isinstance(hist_point, dict):
                    hist_array = np.array([hist_point.get(dim.name, 0.0) for dim in self.space])
                    history_arrays.append(hist_array)
                else:
                    history_arrays.append(np.array(hist_point))
                    
            # 1. 多样性分数 - 与历史点的最小距离
            diversity_score = 0
            if len(history_arrays) > 0:
                min_dist_to_history = float('inf')
                for hist_array in history_arrays:
                    dist = np.sqrt(np.sum((point_array - hist_array) ** 2))
                    min_dist_to_history = min(min_dist_to_history, dist)
                
                # 归一化距离
                diversity_score = min(1.0, min_dist_to_history / np.sqrt(len(point_array)))
            else:
                diversity_score = 1.0
            
            # 2. 与其他候选点的距离
            dispersion_score = 0
            candidate_arrays = []
            for cand in candidates:
                if isinstance(cand, dict):
                    cand_array = np.array([cand.get(dim.name, 0.0) for dim in self.space])
                    candidate_arrays.append(cand_array)
                else:
                    candidate_arrays.append(np.array(cand))
                    
            if len(candidate_arrays) > 1:
                distances = []
                for other_array in candidate_arrays:
                    if not np.array_equal(point_array, other_array):
                        dist = np.sqrt(np.sum((point_array - other_array) ** 2))
                        distances.append(dist)
                
                if distances:
                    # 使用平均距离
                    avg_dist = np.mean(distances)
                    dispersion_score = min(1.0, avg_dist / np.sqrt(len(point_array)))
            else:
                dispersion_score = 1.0
                
            # 3. 参数空间覆盖分数
            coverage_score = 0.5  # 默认中等分数
                
            # 4. 不确定性分数
            uncertainty_score = 0.5  # 默认值
            if hasattr(self, 'clustered_gps') and self.clustered_gps:
                try:
                    _, std = self._predict_with_clustered_gps(point_array.reshape(1, -1))
                    # 归一化不确定性分数
                    uncertainty_score = min(1.0, std[0] / 0.5)  # 假设0.5是合理的标准差上限
                except Exception as e:
                    self.logger.debug(f"计算不确定性分数失败: {e}")
                    
            # 组合分数 - 为bug发现优化权重
            final_score = (
                0.35 * diversity_score +    # 更重视与历史点的差异
                0.25 * dispersion_score +   # 避免候选点聚集
                0.2 * coverage_score +      # 关注参数空间覆盖
                0.2 * uncertainty_score     # 适度关注不确定性
            )
            
            # 添加少量随机性
            final_score += np.random.uniform(0, 0.05)
            
            return final_score
        
        except Exception as e:
            self.logger.warning(f"评估测试多样性时出错: {e}")
            return np.random.uniform(0, 1)  # 出错时返回随机分数
        
        
     
    def _ensure_correct_feature_types(self, params):
        """确保参数符合正确的数据类型
        
        Args:
            params: 待处理的参数字典
            
        Returns:
            处理后确保类型正确的参数字典
        """
        result = {}
        
        # 整数类型参数列表
        integer_params = [
            'num_vehicles', 'num_walkers', 'sensor_number', 
            'static_object_number', 'lidar_channels', 'lidar_points',
            'weather_preset', 'vehicle_type', 'sensor_type', 
            'town', 'static_object_type'
        ]
        
        # 分类变量列表及其映射长度
        categorical_params = {}
        if hasattr(self, 'weather_mapping'):
            categorical_params['weather_preset'] = len(self.weather_mapping)
        if hasattr(self, 'vehicle_type_mapping'):
            categorical_params['vehicle_type'] = len(self.vehicle_type_mapping)
        if hasattr(self, 'sensor_type_mapping'):
            categorical_params['sensor_type'] = len(self.sensor_type_mapping)
        if hasattr(self, 'town_mapping'):
            categorical_params['town'] = len(self.town_mapping)
        if hasattr(self, 'static_object_mapping'):
            categorical_params['static_object_type'] = len(self.static_object_mapping)
        
        for dim in self.space:
            name = dim.name
            
            if name not in params:
                # 如果没有提供该参数，跳过
                continue
                
            value = params[name]
            
            try:
                if name in integer_params:
                    # 整数参数
                    value = int(round(float(value)))
                    
                    # 对于分类参数，确保在有效范围内
                    if name in categorical_params and hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        max_val = categorical_params.get(name, 1) - 1
                        value = max(0, min(max_val, value))
                    # 其他整数参数确保在边界内
                    elif hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        value = max(low, min(high, value))
                else:
                    # 浮点参数
                    value = float(value)
                    
                    # 确保浮点参数在边界内
                    if hasattr(dim, 'bounds'):
                        low, high = dim.bounds
                        value = max(low, min(high, value))
            except:
                # 出错时设置默认值
                if name in integer_params:
                    value = 0 if not hasattr(dim, 'bounds') else dim.bounds[0]
                else:
                    value = 0.0 if not hasattr(dim, 'bounds') else dim.bounds[0]
                    
            result[name] = value
                    
        return result
    

def main():
    # 创建实验管理器
    experiment_manager = CarlaExperimentManager()
    
    # 记录全局资源快照
    resources_before = get_system_resources()
    experiment_manager.logger.info(f"优化开始前系统资源: 内存: {resources_before['memory_rss_mb']:.2f}MB, "
                                   f"CPU: {resources_before['cpu_percent']:.1f}%, 系统内存使用率: {resources_before['system_memory']['percent']}%")
    
    # 运行优化
    best_env = experiment_manager.optimize(
        max_calls=1100,
        epsilon=0.01,
        delta=0.01,
        load_stage1=True,
        save_stage1=True,
        min_samples=60,
        max_cluster_analysis=100,
    )
    
    # 记录优化完成后的资源
    resources_after = get_system_resources()
    experiment_manager.logger.info(f"优化完成后系统资源: 内存: {resources_after['memory_rss_mb']:.2f}MB, "
                                   f"CPU: {resources_after['cpu_percent']:.1f}%, 系统内存使用率: {resources_after['system_memory']['percent']}%")
    experiment_manager.logger.info(f"内存增长: {resources_after['memory_rss_mb'] - resources_before['memory_rss_mb']:.2f}MB")
    
    # 分析结果
    df = pd.DataFrame(experiment_manager.error_log)
    experiment_manager.logger.info("\n===== Optimization Results =====")
    experiment_manager.logger.info(df.sort_values(by="local_rei", ascending=False).head())

    try:
        data = []
        for i, (x, y) in enumerate(zip(experiment_manager.X_history, experiment_manager.y_history)):
            entry = dict(zip([dim.name for dim in experiment_manager.space], x))
            entry["error"] = y
            entry["local_rei"] = y
            data.append(entry)
        
        df = pd.DataFrame(data)
        experiment_manager.logger.info("\n===== 发现的错误模式 =====")
        
        if hasattr(experiment_manager, 'cluster_stats') and experiment_manager.cluster_stats:
            for i, cluster in enumerate(experiment_manager.cluster_stats):
                bug_type = cluster.get('bug_type', f"错误模式 {i+1}")
                experiment_manager.logger.info(f"\n{bug_type}:")
                experiment_manager.logger.info(f"包含样本数: {cluster['size']}")
                
                indices = cluster['point_indices']
                cluster_df = df.iloc[indices].sort_values(by="error", ascending=False)
                experiment_manager.logger.info("\n代表性样本 (Top 3):")
                experiment_manager.logger.info(cluster_df.head(3))
                
                if 'feature_importance' in cluster:
                    important_features = []
                    for j, imp in enumerate(cluster['feature_importance']):
                        if imp > 0.1:
                            important_features.append((experiment_manager.space[j].name, imp))
                    
                    if important_features:
                        experiment_manager.logger.info("\n关键参数及其重要性:")
                        for name, imp in sorted(important_features, key=lambda x: x[1], reverse=True):
                            experiment_manager.logger.info(f"  {name}: {imp:.3f}")
        else:
            experiment_manager.logger.info("未发现明确的错误模式，展示错误值最高的样本:")
            experiment_manager.logger.info(df.sort_values(by="error", ascending=False).head(10))
            
    except Exception as e:
        experiment_manager.logger.error(f"生成结果摘要时出错: {e}")
        import traceback
        traceback.print_exc()
        
        if experiment_manager.y_history:
            best_idx = np.argmax(experiment_manager.y_history)
            best_params = dict(zip([dim.name for dim in experiment_manager.space], experiment_manager.X_history[best_idx]))
            best_error = experiment_manager.y_history[best_idx]
            experiment_manager.logger.info(f"\n===== 最佳结果 =====")
            experiment_manager.logger.info(f"最佳误差值: {best_error}")
            experiment_manager.logger.info(f"最佳参数: {best_params}")

    return best_env



if __name__ == "__main__":
    main()
   

   