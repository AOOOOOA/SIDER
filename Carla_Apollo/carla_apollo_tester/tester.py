# 总的代码； 负责launch carla; apollo; record data; cut the data; rerun; analyze the data


from config import APOLLO_ROOT, PROJECT_NAME,CARLA_SIMULATOR,GENERATE_TRAFFIC,BASE_OUTPUT_DIR,PROJECT_ROOT
from container import ApolloContainer
from typing import Dict, List
from absl import app 
from absl.flags import FLAGS
from loguru import logger #这里我们也需要一个logger 来记录总的测试中的数据 
from utils import get_output_dir, set_up_gflags, set_up_logging,run_command
from time import sleep
import os
import time
from pathlib import Path
from datetime import datetime
import json
from analyzer import TestResultAnalyzer 
import signal
import sys
from reset_dreamview import DreamviewConnection
import subprocess
import traceback
import threading
from concurrent.futures import ProcessPoolExecutor, as_completed
import select
import numpy as np 
import socket
from threading import Lock
#wei: apollo_logs/runtime_log is the apollo running logs under /apollo/data/logs folder

global_server_socket = None




# 定义递归函数将 numpy 类型转换为原生 Python 类型
def convert_numpy(obj):
    if isinstance(obj, np.integer):  # 检查是否为 numpy 整数类型
        return int(obj)
    elif isinstance(obj, np.floating):  # 检查是否为 numpy 浮点数类型
        return float(obj)
    elif isinstance(obj, dict):  # 如果是字典，递归处理
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):  # 如果是列表，递归处理
        return [convert_numpy(i) for i in obj]
    else:
        return obj  # 其他类型直接返回



#TODO: 可以在tester中写成mult-process, 运行和随后的分析 并行计算，减少时间开销

def validate_paths(test_dir: Path, segment_file: Path, objects_file: Path):
    """验证所有路径是否在PROJECT_ROOT下"""
    for p in [test_dir, segment_file, objects_file]:
        try:
            _ = p.relative_to(PROJECT_ROOT)
        except ValueError:
            raise ValueError(f"Path {p} must be under {PROJECT_ROOT}")

def cleanup_handler(signum, frame):
    """处理退出信号的清理函数"""
    print("\nCleaning up processes before exit...")
    logger.info("Received signal to terminate. Cleaning up...")
    
    try:
        for container in containers:
            # 停止所有记录进程
            container.stop_traffic()
            container.stop_recorder()
            container.stop_carla_record()
            
            # 停止bridge
            container.stop_bridge()
            #FIXME: do not need to stop carla every time
            # container.stop_carla()
            
            # 清理Apollo数据
            container.clean_apollo_data()
            
            # 终止所有子进程
            #FIXME:  does not exit this function 
            container.terminate_all_processes()
        
        # [新增] 添加socket清理
        global global_server_socket
        if global_server_socket:
            try:
                global_server_socket.close()
                logger.info("Global TCP server socket closed during cleanup")
            except Exception as e:
                logger.error(f"Error closing global TCP server socket: {e}")
            finally:
                global_server_socket = None
        logger.info("Cleanup completed successfully")
    except Exception as e:
        logger.error(f"Error during cleanup: {e}")
        traceback.print_exc()
    finally:
        sys.exit(0)


class TestConfig:
    def __init__(self, 
                 num_vehicles: int = 20,
                 num_walkers: int = 20,
                 safe_mode: bool = True,
                 sync_mode: bool = True,
                 scenario_length: int = 100, #TODO: need to chnage
                 is_replay: bool = False,
                 replay_file: str = None,
                 segment_duration: int = 30,
                 routing_request = None,
                 routing_response=None):  # 添加路由请求字段
        self.num_vehicles = num_vehicles
        self.num_walkers = num_walkers
        self.safe_mode = safe_mode
        self.sync_mode = sync_mode
        self.scenario_length = scenario_length
        self.is_replay = is_replay
        self.replay_file = replay_file
        self.segment_duration = segment_duration
        self.routing_request = routing_request  # 存储路由请求
        self.routing_response = routing_response
def save_test_metadata(output_dir: Path, config: TestConfig):
    """保存测试配置和元数据"""
    metadata = {
        "timestamp": datetime.now().isoformat(),
        "test_config": {
            "num_vehicles": config.num_vehicles,
            "num_walkers": config.num_walkers,
            "safe_mode": config.safe_mode,
            "sync_mode": config.sync_mode,
            "scenario_length": config.scenario_length,
            "is_replay": config.is_replay,
            "replay_file": str(config.replay_file) if config.replay_file else None,
            "segment_duration": config.segment_duration
        },
        "environment": {
            "apollo_version": "8.0",  # 可以从环境变量或配置中获取
            "carla_version": "0.9.14",  # 可以从环境变量或配置中获取
            "test_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S")
        }
    }
    
    try:
        metadata_file = output_dir / "metadata" / "test_config.json"
        with open(metadata_file, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
        logger.info(f"Test metadata saved to {metadata_file}")
    except Exception as e:
        logger.error(f"Failed to save test metadata: {e}")
        raise

def create_test_output_dir() -> Path:
    """创建以时间戳命名的测试输出目录"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(BASE_OUTPUT_DIR, timestamp)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 创建原始场景目录
    original_dir = output_dir / "original_scenario"
    original_dir.mkdir(exist_ok=True)
    (original_dir / "carla_records").mkdir(exist_ok=True)
    (original_dir / "apollo_logs").mkdir(exist_ok=True)
    
    # 创建回放场景目录
    replay_dir = output_dir / "replay_scenarios"
    replay_dir.mkdir(exist_ok=True)
    
    # 创建元数据目录
    (output_dir / "metadata").mkdir(exist_ok=True)
    
    return output_dir


def split_scenario_data(scenario_file: Path, segment_duration: int) -> List[Path]:
    """将场景数据切分成固定时长的片段，并保存映射关系"""
    try:
        # 读取原始数据
        with open(scenario_file, 'r') as f:
            data = json.load(f)
            
        if not data:
            raise ValueError("Empty scenario data")
            
        # 计算总帧数和每个片段的帧数
        total_frames = len(data)
        frames_per_segment = int(segment_duration / 0.05)  # 假设每帧0.05秒
        
        logger.info(f"Splitting scenario data: {total_frames} frames into {segment_duration}s segments")
        
        # 创建片段目录
        segments_dir = scenario_file.parent / "segments"
        segments_dir.mkdir(exist_ok=True)
        
        # 创建映射信息
        segment_mappings = {
            "original_scenario": str(scenario_file),
            "segments": {}
        }
        
        segment_files = []
        
        # 按时间切分数据
        for i in range(0, total_frames, frames_per_segment):
            segment_data = data[i:i + frames_per_segment]
            
            # 生成片段文件名
            segment_index = (i // frames_per_segment) + 1
            segment_file = segments_dir / f"segment_{segment_index:03d}.json"
            
            # 保存片段数据
            with open(segment_file, 'w') as f:
                json.dump(segment_data, f, indent=2)
            
            # 为每个片段生成对应的objects_replay.json
            generate_objects_replay_json(segment_data, segment_file.parent / f"objects_replay_{segment_index:03d}.json")
            
            # 记录映射信息
            segment_mappings["segments"][f"segment_{segment_index:03d}"] = {
                "file_path": str(segment_file),
                "start_frame": i,
                "end_frame": min(i + frames_per_segment, total_frames),
                "start_timestamp": data[i].get("timestamp", i * 0.05),
                "end_timestamp": data[min(i + frames_per_segment - 1, total_frames - 1)].get("timestamp", (i + frames_per_segment) * 0.05),
                "frame_count": len(segment_data)
            }
            
            segment_files.append(segment_file)
            logger.info(f"Created segment {segment_file.name} with {len(segment_data)} frames")
        
        # 保存映射信息到metadata目录
        try:
            mapping_file = scenario_file.parent.parent.parent / "metadata" / "segment_mappings.json"
            logger.info(f"Attempting to save mapping file to: {mapping_file}")
            
            # 确保metadata目录存在
            mapping_file.parent.mkdir(exist_ok=True)
            
            # 保存映射文件
            with open(mapping_file, 'w') as f:
                json.dump(segment_mappings, f, indent=2)
            
            # 验证文件是否成功创建
            if mapping_file.exists():
                logger.info(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Successfully saved segment mappings to {mapping_file}")
            else:
                logger.error(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Failed to create mapping file at {mapping_file}")
                
        except Exception as e:
            logger.error(f"Failed to save mapping file: {e}")
            logger.error(f"Mapping file path: {mapping_file}")
            logger.error(f"Current working directory: {os.getcwd()}")
            raise
        
        return segment_files
        
    except Exception as e:
        logger.error(f"Failed to split scenario data: {e}")
        raise




def generate_objects_replay_json(segment_data: List[dict], output_file: Path):
    """
    根据场景片段数据生成objects_replay.json
    
    Args:
        segment_data: 场景片段数据
        output_file: 输出文件路径
    """
    try:
        # 读取基础配置模板
        base_config_path = Path(PROJECT_ROOT) / "config" / "objects.json"
        with open(base_config_path, 'r') as f:
            config_data = json.load(f)
            
        # 使用第一帧数据
        first_frame = segment_data[0]
        ego_vehicle_data = first_frame['ego_vehicle']
        nearby_vehicles_data = first_frame['nearby_vehicles']
        nearby_walkers_data=first_frame['nearby_walkers']
        
        
        # 更新ego vehicle信息
        for obj in config_data.get("objects", []):
            if obj.get("id") == "ego_vehicle":
                obj["spawn_point"] = {
                    "x": ego_vehicle_data["x"],
                    "y": ego_vehicle_data["y"],
                    "z": ego_vehicle_data["z"],
                    "yaw": ego_vehicle_data["yaw"],
                    "roll": ego_vehicle_data.get("roll", 0.0),
                    "pitch": ego_vehicle_data.get("pitch", 0.0)
                }
                obj["velocity"] = ego_vehicle_data["velocity"]
                obj["acceleration"] = ego_vehicle_data["acceleration"]
                obj["angular_velocity"] = ego_vehicle_data["angular_velocity"]
                break
                
        # 添加nearby vehicles
        for vehicle in nearby_vehicles_data:
            new_object = {
                'id': vehicle['id'],
                'type': vehicle['type_id'],
                'spawn_point': {
                    'x': vehicle['x'],
                    'y': vehicle['y'],
                    'z': vehicle['z'],
                    'yaw': vehicle['yaw'],
                    'roll': vehicle['roll'],
                    'pitch': vehicle['pitch']
                },
                'velocity': vehicle['velocity'],
                'acceleration': vehicle['acceleration'],
                'angular_velocity': vehicle['angular_velocity'],
                'distance_to_ego': vehicle['distance_to_ego']
            }
            config_data['objects'].append(new_object)
            
            
        for walker in nearby_walkers_data:
            new_object={
                'id':walker['id'],
                'type':walker['type_id'],
                'spawn_point': {
                    'x': walker['x'],
                    'y': walker['y'],
                    'z': walker['z'],
                    'yaw': walker['yaw'],
                    'roll': walker['roll'],
                    'pitch': walker['pitch']
                },
                
                'velocity': walker['velocity'],
                'acceleration': walker['acceleration'],
                'angular_velocity': walker['angular_velocity'],
                'distance_to_ego': walker['distance_to_ego']
            } 
        
        
        
        # 保存配置文件
        with open(output_file, 'w') as f:
            json.dump(config_data, f, indent=4)
            
        logger.info(f"Generated objects replay config: {output_file}")
        
    except Exception as e:
        logger.error(f"Failed to generate objects replay config: {e}")
        raise


def start_containers(num_adc: int, reuse_existing: bool = True) -> List[ApolloContainer]:
    """
    Args:
        num_adc: 需要的容器数量
        reuse_existing: 是否复用已存在的容器
    """
    #FIXME: improve to auto detect whether there is existing running docker 
    if reuse_existing:
        containers = [
            ApolloContainer(APOLLO_ROOT, PROJECT_NAME)  # 使用已存在的容器
        ]
        for ctn in containers:
            if not ctn.is_running():
                logger.warning("Container not running, starting it...")
                ctn.start_container()
            
            # 只在需要时启动这些服务
            if not ctn.check_dreamview_status():  # 需要添加这个方法
                ctn.start_dreamview()
                logger.info(f"Dreamview running at {ctn.dreamview_url}")
            
            logger.info(f"Using existing container: {ctn.container_name} @ {ctn.container_ip()}")
    else:
        # 原有的创建新容器的代码
        containers = [
            ApolloContainer(APOLLO_ROOT, PROJECT_NAME)
        ]
        for ctn in containers:
            ctn.start_container()
            ctn.start_dreamview()
            #FIXME:weired
            ctn.install_bridge_lib()
            
            ctn.start_ads_modules()
            logger.info(f"{ctn.container_name} @ {ctn.container_ip()}")
    
    return containers


def analyze_test_data(test_dir: Path, segment_id: int, segment_name: str):
    """分析单个测试片段的数据"""
    try:
        
        ori_segment_json = test_dir / "original_scenario" / "carla_records" / "segments"/ f"{segment_name}.json"
        replay_json = test_dir / "replay_scenarios" / segment_name / "carla_records" / f"replay_vehicle_states_{segment_name}.json"
        
        # [行 496-500] Apollo日志路径验证保留，但需要确保目录结构
        longrun_apollo_log_dir = test_dir / "original_scenario" / "apollo_logs"
        replay_apollo_log_dir = test_dir / "replay_scenarios" / segment_name / "apollo_logs"
        
        # [新增] 创建分析结果目录结构
        analysis_dir = test_dir / "analysis" / segment_name
        analysis_dir.mkdir(parents=True, exist_ok=True)
        
        # [行 513-515] 优化log文件检查函数
        def has_apollo_logs(log_dir):
            """检查目录中是否存在有效的Apollo日志文件"""
            record_files = list(log_dir.glob("apollo.record.*"))
            if not record_files:
                return False
            # 检查文件大小是否大于0
            return any(f.stat().st_size > 0 for f in record_files)
        
        # [行 518-523] 文件验证部分可以合并简化
        for path, desc in [
            (ori_segment_json, "Original scenario file"),
            (replay_json, "Replay scenario file"),
            (longrun_apollo_log_dir, "Original Apollo log directory"),
            (replay_apollo_log_dir, "Replay Apollo log directory")
        ]:
            if not path.exists():
                raise FileNotFoundError(f"{desc} not found: {path}")
            
            if "apollo_logs" in str(path):
                if not has_apollo_logs(path):
                    raise FileNotFoundError(f"No valid Apollo log files found in: {path}")
        
        # [行 530-536] 创建分析器实例
        analyzer = TestResultAnalyzer(
            ori_segment_json=ori_segment_json,
            replay_json=replay_json,
            extract_apollo_logs=True,
            longrun_apollo_log_dir=longrun_apollo_log_dir,
            replay_apollo_log_dir=replay_apollo_log_dir,
            analysis_dir=analysis_dir
        )
        
        # [行 541] 替换analyze_segment调用为run方法
        result = analyzer.run()  # 不再需要传入segment_id参数
        converted_result=convert_numpy(result)   
        
        
        # [行 544-550] 优化结果保存逻辑
        result_dir = test_dir / "analysis_results"
        result_dir.mkdir(exist_ok=True)
        
        # 保存详细分析结果
        detailed_result_file = result_dir / f"{segment_name}_detailed.json"
        with open(detailed_result_file, 'w') as f:
            json.dump(converted_result, f, indent=2)
            
        # 保存摘要结果
        #FIXME: not so useful, may need to remove
        summary_result_file = result_dir / f"{segment_name}_summary.json"
        summary = {
            "segment_id": segment_id,
            "segment_name": segment_name,
            "error_level": result.get("error_level", "UNKNOWN"),
            "analysis_timestamp": datetime.now().isoformat(),
            "key_metrics": {
                "trajectory_difference": result.get("trajectory_analysis", {}).get("max_difference"),
                "planning_changes": result.get("planning_analysis", {}).get("decision_changes"),
                "state_differences": result.get("state_analysis", {}).get("total_differences")
            }
        }
        with open(summary_result_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        # [行 552-554] 增强日志输出
        logger.info(f"Analysis completed for {segment_name}")
        logger.info(f"Detailed results saved to: {detailed_result_file}")
        logger.info(f"Summary results saved to: {summary_result_file}")
        
        return summary  # 返回摘要结果而不是完整结果
        
    except Exception as e:
        logger.error(f"Analysis failed for {segment_name}: {str(e)}")
        logger.error(traceback.format_exc())  # 添加堆栈跟踪
        return {
            "segment_id": segment_id,
            "segment_name": segment_name,
            "error_level": "ERROR",
            "error_message": str(e),
            "analysis_timestamp": datetime.now().isoformat()
        }
        
        
# 在现有的函数定义之前添加TCP服务器相关函数（约在第350行之前）

def setup_tcp_server(port):
    """设置TCP服务器"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(1)
    logger.info(f"TCP服务器正在监听端口 {port}")
    return server_socket
    return server_socket

# def receive_tcp_data(server_socket):
#     """接收TCP数据"""
#     logger.info("Waiting for client to connect...")
#     conn, addr = server_socket.accept()
#     logger.info(f"Client {addr} connected")
#     try:
#         while True:
#             data = conn.recv(1024)
#             if not data:
#                 break
#             message = json.loads(data.decode())
#             logger.info(f"Received data: {message}")
#             if message.get('replay_complete', False):
#                 logger.info("Replay finished")
#                 return True
#     except Exception as e:
#         logger.error(f"Error receiving data: {e}")
#     finally:
#         conn.close()
#         logger.info("Client connection closed")
#     return False
# socket_lock = Lock()
# def receive_tcp_data(server_socket):
#     """接收TCP数据"""
#     logger.info("Waiting for client to connect...")
#     server_socket.settimeout(30)  # 设置超时时间
#     try:
#         conn, addr = server_socket.accept()
#         logger.info(f"Client {addr} connected")
#         conn.settimeout(30)  # 设置连接超时
#         try:
#             while True:
#                 data = conn.recv(1024)
#                 if not data:
#                     break
#                 message = json.loads(data.decode())
#                 logger.info(f"Received data: {message}")
#                 if message.get('replay_complete', False):
#                     logger.info("Replay finished")
#                     return True
#         except socket.timeout:
#             logger.error("Connection timeout")
#         except Exception as e:
#             logger.error(f"Error receiving data: {e}")
#         finally:
#             conn.close()
#             logger.info("Client connection closed")
#     except socket.timeout:
#         logger.error("Accept timeout")
#     finally:
#         server_socket.settimeout(None)  # 重置超时设置
#     return False
def receive_tcp_data(server_socket, timeout=None):
    """接收TCP数据
    Args:
        server_socket: TCP服务器socket
        timeout: 超时时间（秒）
    """
    logger.info("Waiting for client to connect...")
    server_socket.settimeout(timeout)
    try:
        conn, addr = server_socket.accept()
        logger.info(f"Client {addr} connected")
        conn.settimeout(timeout)
        
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                
                try:
                    message = json.loads(data.decode())
                    logger.info(f"Received message: {message}")
                    print("===============================================================================")
                    print("Received message is:", message)
                    
                    if message.get('replay_complete', False):
                        logger.info("Replay completed successfully")
                        return True
                        
                except json.JSONDecodeError as e:
                    logger.error(f"Failed to parse JSON message: {e}")
                    logger.error(f"Problematic message: {data.decode()}")
                    
        except socket.timeout:
            logger.error(f"Connection timeout after {timeout} seconds waiting for replay complete message")
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
        finally:
            conn.close()
            
    except socket.timeout:
        logger.error(f"Timeout after {timeout} seconds waiting for client connection")
    except Exception as e:
        logger.error(f"Error accepting connection: {e}")
    finally:
        server_socket.settimeout(None)
    
    return False
socket_lock = Lock()
def check_port_in_use(port):
    """检查端口是否被占用"""
    try:
        # 使用 lsof 检测端口是否被占用
        cmd = f"lsof -i :{port}"
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return result.returncode == 0  # 返回码为 0 表示端口被占用
    except Exception as e:
        logger.error(f"Error checking port {port}: {e}")
        return False


def kill_process_on_port(port):
    """终止占用指定端口的进程"""
    try:
        # 使用 lsof 查找占用端口的进程 ID
        cmd = f"lsof -i :{port} -t"
        output = subprocess.check_output(cmd, shell=True).decode().strip()
        if output:
            pids = output.split('\n')
            for pid in pids:
                try:
                    logger.info(f"Attempting to kill process {pid} using port {port}")
                    subprocess.run(['kill', '-9', pid.strip()], check=True)
                    logger.info(f"Successfully killed process {pid}")
                except Exception as e:
                    logger.error(f"Failed to kill process {pid}: {e}")
            time.sleep(1)  # 给系统一些时间释放端口
            # 检查端口是否仍然被占用
            if check_port_in_use(port):
                logger.warning(f"Port {port} is still in use after killing processes")
                return False
            return True
    except subprocess.CalledProcessError:
        logger.warning(f"No processes found using port {port}")
    except Exception as e:
        logger.error(f"Unexpected error killing process on port {port}: {e}")
    return False


def get_existing_socket(port):
    """获取已存在的 socket 或创建新的 socket"""
    global global_server_socket

    with socket_lock:
        # 如果全局 socket 存在且有效，直接返回
        if global_server_socket:
            try:
                global_server_socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                logger.info("Using existing TCP server socket")
                return global_server_socket
            except socket.error:
                logger.info("Existing socket is invalid, closing it")
                try:
                    global_server_socket.close()
                except Exception as e:
                    logger.error(f"Error closing socket: {e}")
                global_server_socket = None

        # 检查端口是否被占用
        if check_port_in_use(port):
            logger.warning(f"Port {port} is in use")
            # 尝试终止占用端口的进程
            if kill_process_on_port(port):
                logger.info(f"Successfully cleared port {port}")
            else:
                logger.error(f"Failed to clear port {port}")
                raise RuntimeError(f"Port {port} is in use and couldn't be cleared")

        # 创建新的 socket
        try:
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 启用端口重用
            new_socket.bind(('0.0.0.0', port))
            new_socket.listen(1)
            logger.info(f"Created new TCP server socket on port {port}")
            global_server_socket = new_socket
            return new_socket
        except Exception as e:
            logger.error(f"Failed to create new socket: {e}")
            raise
    
def run_single_test(config: TestConfig, containers: List[ApolloContainer], base_dir: Path, dreamview):
    """运行单个测试配置"""
    # 记录测试开始时间

    test_start_time = datetime.now()
    test_start_timestamp = test_start_time.strftime("%Y%m%d_%H%M%S")
    
    # [添加这行] 更新日志配置
    #FIXME: duplicate set_up_logging at line 662 and line 692
    set_up_logging(FLAGS.log_level, base_dir)
    logger.info(f"Starting new test configuration at {test_start_timestamp}")
    global global_server_socket
    
    try:
        global_server_socket = get_existing_socket(12345)
        logger.info("TCP server ready for replay tests")
    except Exception as e:
        logger.error(f"Failed to setup TCP server: {e}")
        raise
            
    
    with ProcessPoolExecutor(max_workers=3) as executor:
        try:
            # 创建带时间戳的子测试目录
            test_name = f"test_{test_start_timestamp}_v{config.num_vehicles}_w{config.num_walkers}"
            test_dir = base_dir / test_name
            
            # 创建子测试的目录结构
            dirs = [
                test_dir / "original_scenario" / "carla_records",
                test_dir / "original_scenario" / "apollo_logs" / "runtime_logs",
                test_dir / "original_scenario" / "metric_dir",
                test_dir / "replay_scenarios",
                test_dir / "metadata",
                
                
            ]
            
            for dir_path in dirs:
                dir_path.mkdir(parents=True, exist_ok=True)
                
            # 更新日志配置到当前测试
            set_up_logging(FLAGS.log_level, test_dir)
            logger.info(f"Starting test at {test_start_timestamp}")
            logger.info(f"Configuration: {config.num_vehicles} vehicles, {config.num_walkers} walkers")


        
            # dreamview.reset_all()
            # time.sleep(5)
            # dreamview.setup()
            # time.sleep(5)
            
            
            
            
            # FIXED: when use the carla_record_data function, it is executed on the PC machine, so the file path is fine.
            # but now when we move the data into the main.py, the path should be converted into the docker machine
            original_data_file = test_dir / "original_scenario" / "carla_records" / "vehicle_states.json"
            # 启动bridge
            metric_dir=test_dir / "original_scenario" / "metric_dir"
            
            containers[0].launch_bridge(
                record_path=str(original_data_file),
                num_vehicles=int(config.num_vehicles),
                num_walkers=int(config.num_walkers),
                metric_dir=str(metric_dir)
            )
            

            
            
            containers[0].start_apollo_recorder(str(test_dir))

            

            containers[0].set_destination(
                test_dir=str(test_dir),
                mode='generate')
            # 记录原始场景数据
            
            original_carla_record = test_dir / "original_scenario" / "carla_records" / "carla_recorder.log"
           
            containers[0].carla_recorder(
                output_path=str(original_carla_record)
            )
            
            
            
            # 等待场景执行完成
            time.sleep(config.scenario_length)
            
            #FIXME: error happens here
            # 停止记录
            containers[0].stop_apollo_recorder()
            containers[0].stop_carla_record()
            containers[0].stop_bridge()
            containers[0].stop_traffic()
            containers[0].stop_routing()
            containers[0].stop_carla_recorder()
            time.sleep(5)
            
            # 保存Apollo日志
            #FIXME: check whether the log is under the correct folder or not 
            #  这里的这个目录路径看起来好像是有点问题的
            containers[0].clean_apollo_data(str(test_dir))
            
            # 切分场景数据
            segment_files = split_scenario_data(original_data_file, config.segment_duration)
            
            # 创建回放目录
            replay_dir = test_dir / "replay_scenarios"
            
            
            analysis_futures=[]
            # 对每个片段进行回放
            
            
            for i, segment_file in enumerate(segment_files, 1):
                segment_name = f"segment_{i:03d}"
                logger.info(f"Running replay for {segment_name}")
                set_up_logging(FLAGS.log_level, base_dir, is_replay=True, segment=segment_name)
                
                # 创建回放片段目录
                segment_dir = replay_dir / segment_name
                segment_dir.mkdir(exist_ok=True)
                (segment_dir / "carla_records").mkdir(exist_ok=True)
                (segment_dir / "apollo_logs").mkdir(exist_ok=True)
                (segment_dir / "apollo_logs/runtime_logs").mkdir(exist_ok=True)


                # 更新日志配置到当前回放片段
                set_up_logging(FLAGS.log_level, test_dir, is_replay=True, segment=segment_name)
                
                
                # 启动回放
                objects_replay_file = segment_file.parent / f"objects_replay_{i:03d}.json"
                
                
                replay_log=test_dir / "original_scenario" / "apollo_logs" / "apollo.record.00000"
                # re-start the apollo modules

                # containers[0].stop_ads_modules()
                # containers[0].start_ads_modules()
                # time.sleep(5)
                # restart methodo does not work FIXME: another q: when should we reset the ads modules
                
            
                
                #TODO: 这个可以和前面的routing handler结合起来
                cleanup_processes(containers)
                logger.info("*****************Checking if processes have been correctly terminated*****************")
                # time.sleep(5)
                
                
                dreamview.reset_backend()
                time.sleep(3)
                    
                #restart carla
                containers[0].stop_carla()
                containers[0].start_carla()
                time.sleep(12)
                
                
                

                # # before running, stop planning and prediction and relaunch them
                # containers[0].stop_planning_n_prediction()
                # time.sleep(2)
                # containers[0].start_planning()
                # containers[0].start_prediction()
                # containers[0].start_control()
                # time.sleep(5)
                
                # dreamview.reset_all()
                # time.sleep(5)
                # dreamview.setup()
                # time.sleep(5)
                
                replay_output = segment_dir / "carla_records" / f"replay_vehicle_states_{segment_name}.json"
                # 回放：使用保存的routing信息
                metric_dir=segment_dir / "metric_dir"
                
                containers[0].launch_bridge(
                    replay=True,
                    replay_file=str(segment_file),
                    objects_file=str(objects_replay_file),
                    replay_log=str(replay_log),
                    record_replay_path=str(replay_output),
                    metric_dir=str(metric_dir)
                )
                # time.sleep(5)
                # time.sleep(9)
                #TODO: may have further problem about the timing of launch
                # logger.info("Replaying saved routing")
                # containers[0].set_destination(
                    # test_dir=str(test_dir),
                    # mode='replay'
                # )
                
                containers[0].start_apollo_recorder(
                    test_output_dir=str(test_dir),
                    is_replay=True,
                    segment=segment_name
                )
                
                
                

                    
                    
                # 记录回放数据
                # 新的版本，尝试数据对齐。
                
                """
                replay_output = segment_dir / "carla_records" / f"replay_vehicle_states_{segment_name}.json"
                
                
                
                containers[0].record_replay_data(
                    output_path=str(replay_output),
                    log_dir=str(segment_dir / "carla_records"),
                    replay_index=segment_name
                )
                """

                
                carla_recorder_output=segment_dir/ "carla_records" / f"replay_{segment_name}.log"
                containers[0].carla_recorder(
                    output_path=str(carla_recorder_output)
                )



                #FIXME: 这里应该加一个故障处理机制
                #TODO: we should distinguish the carla record and carla recorder function name
                if global_server_socket:
                    logger.info("Waiting for replay to complete...")
                    replay_complete = receive_tcp_data(global_server_socket, timeout=config.scenario_length + 10)
                    
                    if replay_complete:
                        logger.info("Replay completed successfully")
                        # 执行清理操作
                        containers[0].stop_apollo_recorder()
                        containers[0].stop_carla_record()
                        containers[0].stop_bridge()
                        containers[0].stop_record_replay_data()
                        # containers[0].stop_carla_recorder() #this one is typically for carla recorder log
                        # containers[0].stop_traffic()
                        # containers[0].stop_routing()

                        containers[0].clean_apollo_data(str(test_dir), is_replay=True, segment=segment_name)
                        time.sleep(2)  # 给系统一些清理时间
                                
                            
                        future = executor.submit(
                                analyze_test_data,
                                test_dir=test_dir,
                                segment_id=i,
                                segment_name=segment_name
                                # container=containers[0]  # 传入容器实例
                            )
                        analysis_futures.append(future)
                
                    else:
                        logger.error("Replay did not complete successfully")
                        raise RuntimeError("Replay failed to complete")
                
            
                while system_resources_busy():
                    time.sleep(0.5)
            
            for future in as_completed(analysis_futures):
                try:
                    result=future.result()
                    logger.info(f"Analysis result: {result}")
                except Exception as e:
                    logger.error(f"=========Error during analysis: {e}")  # 行号

            # 不需要显式关闭executor，with语句会处理
            logger.info(f"Submitted {len(analysis_futures)} analysis tasks")
                
            # 记录测试结束时间和持续时间
            test_end_time = datetime.now()
            test_duration = test_end_time - test_start_time
            
            # 保存测试元数据
            metadata = {
                "test_config": {
                    "num_vehicles": config.num_vehicles,
                    "num_walkers": config.num_walkers,
                    "safe_mode": config.safe_mode,
                    "sync_mode": config.sync_mode,
                    "scenario_length": config.scenario_length,
                    "segment_duration": config.segment_duration
                },
                "timing": {
                    "start_time": test_start_time.isoformat(),
                    "end_time": test_end_time.isoformat(),
                    "duration_seconds": test_duration.total_seconds()
                }
            }
                
            metadata_file = test_dir / "metadata" / "test_info.json"
            with open(metadata_file, "w") as f:
                json.dump(metadata, f, indent=2)
                    
            logger.info(f"Test completed. Duration: {test_duration}")
                
        except Exception as e:
            logger.error(f"Test failed after {datetime.now() - test_start_time}")
            traceback.print_exc()
            raise
        finally:
            # 确保清理socket

            if global_server_socket:
                try:
                    global_server_socket.shutdown(socket.SHUT_RDWR)
                    global_server_socket.close()
                    global_server_socket = None
                    logger.info("Cleaned up TCP server socket")
                except Exception as e:
                    logger.error(f"Error cleaning up socket: {e}")
    
import psutil
def system_resources_busy():
    """检查系统资源使用情况"""
    try:
        # 检查CPU使用率
        cpu_percent = psutil.cpu_percent(interval=0.1)
        # 检查内存使用率
        memory_percent = psutil.virtual_memory().percent
        # 检查磁盘IO
        disk_io = psutil.disk_io_counters()
        
        # 如果资源使用率过高，返回True
        return (cpu_percent > 90 or memory_percent > 90)
    except:
        return False
    
# def cleanup_processes():
#     """清理残留进程"""
#     try:
#         # 使用更简单的匹配模式
#         subprocess.run(["pkill", "-2", "main.py"], check=False)
#         subprocess.run(["pkill", "-2", "generate_traffic.py"], check=False)
#         subprocess.run(["pkill", "-9", "record_data.py"], check=False)
#         subprocess.run(["pkill", "-9", "routing_handler.py"], check=False)
        
#         # 添加验证
#         time.sleep(1)  # 给进程一些时间来终止
        
#         # 检查是否还有残留进程
#         check_cmd = "ps aux | grep -E 'routing_handler.py|record_data.py|generate_traffic.py|main.py' | grep -v grep"
#         result = subprocess.run(check_cmd, shell=True, capture_output=True, text=True)
#         if result.stdout.strip():
#             logger.warning("Some processes might still be running:")
#             logger.warning(result.stdout)
#             # 如果还有进程，使用更强力的终止命令
#             subprocess.run(["pkill", "-9", "-f", "routing_handler"], check=False)
#             subprocess.run(["pkill", "-9", "-f", "record_data"], check=False)
            
#         logger.info("Cleanup process completed")
        
#     except Exception as e:
#         logger.error(f"Error during process cleanup: {e}")



def cleanup_processes(containers: List[ApolloContainer]):
    """检查并清理残留进程"""
    try:
        # 检查各个进程是否存在
        check_cmd = "ps aux | grep -E 'routing_handler.py|record_data.py|generate_traffic.py|main.py' | grep -v grep"
        result = subprocess.run(check_cmd, shell=True, capture_output=True, text=True)
        
        if result.stdout.strip():
            logger.warning("Found running processes:")
            logger.warning(result.stdout)
            
            # 对每个容器执行清理
            for container in containers:
                # 停止各个组件
                container.stop_bridge()  # 停止main.py
                container.stop_traffic()  # 停止generate_traffic.py
                container.stop_carla_record()  # 停止record_data.py
                container.stop_routing()  # 停止routing_handler.py
            
            # 等待进程终止
            time.sleep(2)
            
            # 再次检查是否还有残留进程
            result = subprocess.run(check_cmd, shell=True, capture_output=True, text=True)
            if result.stdout.strip():
                logger.error("Some processes are still running after cleanup:")
                logger.error(result.stdout)
            else:
                logger.info("All processes cleaned up successfully")
        else:
            logger.info("No residual processes found")
            
    except Exception as e:
        logger.error(f"Error during process cleanup: {e}")
        
        
        
def main(argv):
    # 设置信号处理
    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGTERM, cleanup_handler)
    
    del argv
    
    
    # ADD: 创建基础测试目录及其子目录
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_dir = Path(PROJECT_ROOT) / "test_outputs" / timestamp
    base_dir.mkdir(parents=True, exist_ok=True)
    
    # 设置基础日志配置
    set_up_logging(FLAGS.log_level, base_dir)
    logger.info(f"Created base test directory: {base_dir}")
    
    print("before connect to dreamview")
    
    dreamview=DreamviewConnection()
    print("after connect to dreamview")
    
    # 创建不同的测试配置
    #TODO: 用scenario generator的形式在这里有机的生成更多的测试配置
    test_configs = [
        # TestConfig(num_vehicles=10, num_walkers=10, sync_mode=False),
        
        # TestConfig(num_vehicles=20, num_walkers=20, sync_mode=False),
        # TestConfig(num_vehicles=30, num_walkers=30, sync_mode=False),
        # TestConfig(num_vehicles=40, num_walkers=40, sync_mode=False),
        # TestConfig(num_vehicles=50, num_walkers=50, sync_mode=False),
        
        # TestConfig(num_vehicles=1, num_walkers=1, sync_mode=False),
        # TestConfig(num_vehicles=60, num_walkers=30, sync_mode=False),
        # TestConfig(num_vehicles=70, num_walkers=40, sync_mode=False),
        # TestConfig(num_vehicles=80, num_walkers=50, sync_mode=False),
        # TestConfig(num_vehicles=90, num_walkers=60, sync_mode=False),
        # TestConfig(num_vehicles=100, num_walkers=70, sync_mode=False),
        # TestConfig(num_vehicles=110, num_walkers=80, sync_mode=False),
        # TestConfig(num_vehicles=120, num_walkers=90, sync_mode=False),
        # TestConfig(num_vehicles=130, num_walkers=100, sync_mode=False),
        # TestConfig(num_vehicles=140, num_walkers=110, sync_mode=False),
        
        
        
        
        # # TestConfig(num_vehicles=10, num_walkers=0, sync_mode=False),
        
        # TestConfig(num_vehicles=20, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=30, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=40, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=50, num_walkers=0, sync_mode=False),
        
        # # TestConfig(num_vehicles=1, num_walkers=1, sync_mode=False),
        # TestConfig(num_vehicles=60, num_walkers=0, sync_mode=False),
        TestConfig(num_vehicles=100, num_walkers=100, sync_mode=False),
        # TestConfig(num_vehicles=80, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=90, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=100, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=110, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=120, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=130, num_walkers=0, sync_mode=False),
        # TestConfig(num_vehicles=140, num_walkers=0, sync_mode=False),
        

    ]
    print("******************************************************")
    print("**********************START Container*****************")
    print("******************************************************")
    global containers
    containers = start_containers(1)

    
    #FIXME: problems here. Even we have a carla, the tester still launch a new one 
    if not containers[0].check_carla_status():
        print("******************************************************")
        print("***************START_ CARLA***************************")
        print("******************************************************")

        containers[0].start_carla()    
        time.sleep(10)    
        #FIXME: seems weired 
    if not containers[0].check_apollo_status():
        # containers[0].launch_apollo() 
        #FIXME: no need to start the dreamview at first 
        #TODO: need to add the modules status checking 
        pass
    
        #TODO: temp comment it 
        # containers[0].start_ads_modules()
    
    #FIXME： 其实这里会出问题，除非每次都重启Apollo，否则这里的log文件夹不会被自动创建
    try:
        # 开始前清理一次，但不需要保存日志--removed 
        # containers[0].ensure_log_dirs()  # 需要添加这个方法
        # containers[0].clean_apollo_data()
        
        # 执行多次测试
        for config in test_configs:
            # test_output_dir = create_test_output_dir()
            #TODO: generate the traffic flow after generate the ego vehicle
            logger.info(f"Starting test with {config.num_vehicles} vehicles and {config.num_walkers} walkers")
            dreamview.reset_backend()
            #stop all modules and relaunch for the test in  
            
            # containers[0].stop_ads_modules()
            # containers[0].start_ads_modules()
            
            time.sleep(10)
            run_single_test(config, containers, base_dir, dreamview)
            time.sleep(5)
    except Exception as e:
        logger.error(f"Test failed: {str(e)}")
        print("=======the error info is:")
        traceback.print_exc()
        cleanup_handler(None, None)  # 在异常时也执行清理
        raise
    finally:
        # 如果正常退出，也执行清理
        cleanup_handler(None, None)


if __name__== "__main__":
    set_up_gflags()
    try:
        app.run(main)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.exception(e)
        cleanup_handler(None, None)  # 确保在其他异常时也执行清理
        raise e
