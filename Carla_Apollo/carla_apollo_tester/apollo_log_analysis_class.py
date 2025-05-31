import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import logging
import json
from datetime import datetime
from typing import List, Tuple, Dict, Any
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
from scipy.interpolate import interp1d
import numpy as np
from scipy.spatial.distance import directed_hausdorff
from pathlib import Path




#FIXME: temp not use all the plot function. But actually they should be considered 
def _calculate_trajectory_length(df):
    dx = df['localization_pose_x'].diff()
    dy = df['localization_pose_y'].diff()
    distances = np.sqrt(dx**2 + dy**2)
    return np.sum(distances)


def normalize_trajectory(x, y):
    """归一化轨迹
    Args:
        x, y: 轨迹坐标序列
    Returns:
        normalized_path: 归一化后的轨迹点序列 [(x,y),...]
    """
    # 移除无效值
    valid = ~(np.isnan(x) | np.isnan(y))
    x, y = x[valid], y[valid]
    
    if len(x) < 2:
        return None
        
    # 计算路径长度
    path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    
    # 创建归一化的参数序列
    t = np.zeros(len(x))
    t[1:] = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    t = t / path_length
    
    # 插值得到均匀采样的点
    num_points = 100  # 归一化后的点数
    t_new = np.linspace(0, 1, num_points)
    
    try:
        fx = interp1d(t, x, kind='linear')
        fy = interp1d(t, y, kind='linear')
        
        x_new = fx(t_new)
        y_new = fy(t_new)
        
        return np.column_stack((x_new, y_new))
    except Exception as e:
        print(f"Trajectory normalization failed: {str(e)}")
        
        return None

def compute_frechet_distance(path1, path2):
    """计算两条路径的Fréchet距离"""
    if path1 is None or path2 is None:
        return float('inf')
    
    try:
        # 使用Hausdorff距离作为Fréchet距离的近似
        return max(directed_hausdorff(path1, path2)[0],
                  directed_hausdorff(path2, path1)[0])
    except Exception as e:
        print(f"Fréchet distance calculation failed: {str(e)}")
        return float('inf')



#TODO: 有些部分的代码是不断重复的，需要将这些代码给提取复用一下
class TrajectoryAligner:
    """轨迹对齐器类，用于对齐两条轨迹的关键点和相关数据"""
    
    def __init__(self, pos_threshold: float = 0.5, time_threshold: float = 0.1,
                 key_positions: List[Tuple[float, float]] = None):
        """初始化参数
        Args:
            pos_threshold: 位置匹配阈值(米)，判断两个点是否匹配的距离阈值
            time_threshold: 时间匹配阈值(秒)，判断两个点是否匹配的时间阈值
            key_positions: 关键位置点列表，每个元素为(x,y)坐标元组
        """
        self.pos_threshold = pos_threshold
        self.time_threshold = time_threshold
        self.key_positions = key_positions or []

    def find_key_points(self, loc_df: pd.DataFrame) -> List[int]:
        """在轨迹中找到最接近关键位置点的数据点索引"""
        key_indices = []
        for key_pos in self.key_positions:
            # 计算轨迹上每个点到关键位置点的距离
            distances = loc_df.apply(
                lambda row: np.sqrt(
                    (row['localization_pose_x'] - key_pos[0])**2 + 
                    (row['localization_pose_y'] - key_pos[1])**2
                ), 
                axis=1
            )
            # 找到最近的点
            closest_idx = distances.argmin()
            # 如果距离在阈值内，则记录该点
            if distances[closest_idx] < self.pos_threshold:
                key_indices.append(closest_idx)
        return key_indices

    def align_trajectories(self, loc1_df: pd.DataFrame, loc2_df: pd.DataFrame) -> List[Tuple[int, int]]:
        """对齐两条轨迹的关键点"""
        key_indices1 = self.find_key_points(loc1_df)
        key_indices2 = self.find_key_points(loc2_df)
        
        if len(key_indices1) != len(key_indices2):
            print("Warning: The number of key points matched for the two trajectories is different")

            
        return list(zip(key_indices1, key_indices2))

    def align_other_data(self, aligned_pairs: List[Tuple[int, int]], 
                        loc1_df: pd.DataFrame, loc2_df: pd.DataFrame,
                        data1_df: pd.DataFrame, data2_df: pd.DataFrame) -> List[Tuple[pd.Series, pd.Series]]:
        """根据定位数据的时间戳对齐其他数据（如planning数据）"""
        aligned_data = []
        for idx1, idx2 in aligned_pairs:
            # 获取定位点的时间戳
            time1 = loc1_df.iloc[idx1]['timestamp']
            time2 = loc2_df.iloc[idx2]['timestamp']
            
            # 找到最接近的数据点
            data1_idx = (data1_df['timestamp'] - time1).abs().idxmin()
            data2_idx = (data2_df['timestamp'] - time2).abs().idxmin()
            
            aligned_data.append((data1_df.iloc[data1_idx], data2_df.iloc[data2_idx]))
        
        return aligned_data

class LogAnalyzer:
    """Apollo日志分析器"""
    
    def __init__(self, ori_apollo_csv_dir: str, replay_apollo_csv_dir:str, output_dir: str):
        """初始化分析器
        Args:
            
            output_dir: 分析结果输出目录
        """

        # self.ori_apollo_csv_dir=ori_apollo_csv_dir
        # self.replay_apollo_csv_dir =replay_apollo_csv_dir
        # self.current_output_dir=output_dir
        
        # 转换为Path对象
        self.ori_apollo_csv_dir = Path(ori_apollo_csv_dir)
        self.replay_apollo_csv_dir = Path(replay_apollo_csv_dir)

        
        self.current_output_dir = Path(output_dir)
        
        self._validate_paths()
        
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 创建本次分析的特定输出目录
        # self.current_output_dir = os.path.join(output_dir, self.timestamp)
        # self.current_output_dir=output_dir
        self._create_output_dirs()
        
        # 设置日志记录器
        self.logger = self._setup_logger()
        
        # 数据帧
        self.loc1_df = None
        self.loc2_df = None
        self.planning1_df = None
        self.planning2_df = None
        self.chassis1_df = None
        self.chassis2_df = None
        self.aligner=None
        
        # 分析结果
        self.results = {
            'metadata': {
                'analysis_time': self.timestamp,
                'ori_apollo_csv_dir':ori_apollo_csv_dir,
                'replay_apollo_csv_dir':replay_apollo_csv_dir
            },
            'trajectory_analysis': None,
            'planning_analysis': None,
            'trajectory_shape_analysis': None,
            'planning_behavior_analysis': None,
            'trajectory_dynamics_analysis': None,
            'state_analysis': None
        }
        
        # 轨迹对齐相关
        self.shorter_df = None
        self.longer_df = None
        self.length_diff = None
        self.aligned_pairs = None
        self.key_positions = None
        
    #TODO: 暂时先这样吧，可能还是有问题，但是暂时先这样
    def _plot_speed_and_acc_comparison(self):
        """
        对比对象：原始场景和回放场景的速度和加速度时间序列
        格式：多子图的时间序列线图，展示xyz三个方向的速度和加速度对比
        """
        # 首先使用TrajectoryAligner对齐数据
        if not hasattr(self, 'aligner') or self.aligner is None:
            self.aligner = TrajectoryAligner(pos_threshold=0.5, time_threshold=0.1)
        
        # 获取对齐的点对
        aligned_pairs = self.aligner.align_trajectories(self.loc1_df, self.loc2_df)
        
        # 使用第一个对齐点作为参考时间零点
        ref_time = self.loc1_df.iloc[aligned_pairs[0][0]]['timestamp']
        time_series = [(self.loc1_df.iloc[idx1]['timestamp'] - ref_time) for idx1, _ in aligned_pairs]
        
        # 创建6个子图（3个速度，3个加速度）
        fig, axes = plt.subplots(3, 2, figsize=(15, 15))
        
        # 速度分量的列名
        velocity_columns = [
            'localization_angular_velocity_x',
            'localization_angular_velocity_y',
            'localization_angular_velocity_z'
        ]
        
        # 加速度分量的列名
        accel_columns = [
            'localization_linear_accel_x',
            'localization_linear_accel_y',
            'localization_linear_accel_z'
        ]
        
        # 绘制速度对比
        for i, col in enumerate(velocity_columns):
            ax = axes[i, 0]
            # 获取对齐后的数据点
            original_data = [self.loc1_df.iloc[idx1][col] for idx1, _ in aligned_pairs]
            replay_data = [self.loc2_df.iloc[idx2][col] for _, idx2 in aligned_pairs]
            
            ax.plot(time_series, original_data, 'b-', label='Original', linewidth=2, alpha=0.7)
            ax.plot(time_series, replay_data, 'r--', label='Replay', linewidth=2, alpha=0.7)
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'Velocity {col[-1]} (m/s)')
            ax.legend()
            ax.grid(True)
            if i == 0:
                ax.set_title('Velocity Comparison')
        
        # 绘制加速度对比
        for i, col in enumerate(accel_columns):
            ax = axes[i, 1]
            # 获取对齐后的数据点
            original_data = [self.loc1_df.iloc[idx1][col] for idx1, _ in aligned_pairs]
            replay_data = [self.loc2_df.iloc[idx2][col] for _, idx2 in aligned_pairs]
            
            ax.plot(time_series, original_data, 'b-', label='Original', linewidth=2, alpha=0.7)
            ax.plot(time_series, replay_data, 'r--', label='Replay', linewidth=2, alpha=0.7)
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'Acceleration {col[-1]} (m/s²)')
            ax.legend()
            ax.grid(True)
            if i == 0:
                ax.set_title('Acceleration Comparison')
        
        plt.tight_layout()
        output_path = self.current_output_dir / 'figures' / 'speed_and_acc_comparison.png'
        plt.savefig(output_path)
        plt.close()
        
        # 计算统计信息
        stats = {
            'velocity': {
                'max_difference': {
                    'x': max(abs(np.array(original_data) - np.array(replay_data))),
                    'y': max(abs(np.array(original_data) - np.array(replay_data))),
                    'z': max(abs(np.array(original_data) - np.array(replay_data)))
                },
                'mean_difference': {
                    'x': np.mean(abs(np.array(original_data) - np.array(replay_data))),
                    'y': np.mean(abs(np.array(original_data) - np.array(replay_data))),
                    'z': np.mean(abs(np.array(original_data) - np.array(replay_data)))
                }
            },
            'acceleration': {
                'max_difference': {
                    'x': max(abs(np.array(original_data) - np.array(replay_data))),
                    'y': max(abs(np.array(original_data) - np.array(replay_data))),
                    'z': max(abs(np.array(original_data) - np.array(replay_data)))
                },
                'mean_difference': {
                    'x': np.mean(abs(np.array(original_data) - np.array(replay_data))),
                    'y': np.mean(abs(np.array(original_data) - np.array(replay_data))),
                    'z': np.mean(abs(np.array(original_data) - np.array(replay_data)))
                }
            }
        }
        
        # 将统计信息添加到结果中
        self.results['dynamics_analysis'] = stats 
    
    
    def _plot_planning_decisions(self):
        """
        绘制规划决策对比图并保存到figures目录
        """
        def extract_main_decision(decision_str):
            # 从决策字符串中提取主要决策类型
            if pd.isna(decision_str):
                return "UNKNOWN"
            
            # 获取第一个大括号前的决策类型
            main_type = decision_str.split('{')[0].strip().upper()
            
            return main_type if main_type in ALL_DECISIONS else "UNKNOWN"
        
        # 定义所有可能的决策类型
        ALL_DECISIONS = {
            'CRUISE',
            'STOP',
            'EMERGENCY_STOP',
            'CHANGE_LANE',
            'MISSION_COMPLETE',
            'NOT_READY',
            'PARKING',
            'UNKNOWN'
        }
        
        # 提取主要决策类型
        self.planning1_df['MainDecision'] = self.planning1_df['decision'].apply(extract_main_decision)
        self.planning2_df['MainDecision'] = self.planning2_df['decision'].apply(extract_main_decision)
        
        plt.figure(figsize=(15, 10))
        
        # 统计每种决策的数量
        original_counts = self.planning1_df['MainDecision'].value_counts()
        replay_counts = self.planning2_df['MainDecision'].value_counts()
        
        # 使用所有可能的决策类型
        all_decisions = sorted(ALL_DECISIONS)
        
        # 准备数据
        original_data = [original_counts.get(decision, 0) for decision in all_decisions]
        replay_data = [replay_counts.get(decision, 0) for decision in all_decisions]
        
        # 设置柱状图的位置
        x = np.arange(len(all_decisions))
        width = 0.35
        
        # 绘制柱状图
        plt.subplot(2, 1, 1)
        plt.bar(x - width/2, original_data, width, label='Original', color='blue', alpha=0.7)
        plt.bar(x + width/2, replay_data, width, label='Replay', color='red', alpha=0.7)
        plt.xlabel('Decision Types')
        plt.ylabel('Count')
        plt.title('Planning Decision Counts Comparison')
        plt.xticks(x, all_decisions, rotation=45)
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # 绘制时序图
        plt.subplot(2, 1, 2)
        plt.plot(self.planning1_df.index, self.planning1_df['MainDecision'], 'b-', label='Original', alpha=0.7)
        plt.plot(self.planning2_df.index, self.planning2_df['MainDecision'], 'r-', label='Replay', alpha=0.7)
        plt.xlabel('Time')
        plt.ylabel('Decision')
        plt.title('Planning Decisions Over Time')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # 调整布局并保存
        plt.tight_layout()
        output_path = self.current_output_dir / 'figures' / 'planning_decisions.png'
        plt.savefig(output_path)
        plt.close()
        
        # 返回统计信息
        stats = {
            'original': dict(original_counts),
            'replay': dict(replay_counts),
            'differences': {
                decision: replay_counts.get(decision, 0) - original_counts.get(decision, 0)
                for decision in all_decisions
            }
        }
        
        # 将统计信息添加到结果中
        self.results['planning_analysis'] = stats
        
        return stats
    

    def _plot_differences(self):
        """
        对比对象：原始和回放场景的各项指标差异
        格式：多子图的组合图
        1. 位置差异：时间序列线图 + 阈值线
        2. 速度差异：时间序列线图 + 阈值线
        3. 差异分布：直方图
        """
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
        
        # 位置差异
        ax1.plot(self.aligned_timestamps, self.position_differences, 'b-', linewidth=2)
        ax1.axhline(y=self.POSITION_THRESHOLD, color='r', linestyle='--', 
                    label=f'Threshold ({self.POSITION_THRESHOLD}m)')
        ax1.set_ylabel('Position Difference (m)')
        ax1.legend()
        ax1.grid(True)
        
        # 速度差异
        ax2.plot(self.aligned_timestamps, self.velocity_differences, 'g-', linewidth=2)
        ax2.axhline(y=self.VELOCITY_THRESHOLD, color='r', linestyle='--', 
                    label=f'Threshold ({self.VELOCITY_THRESHOLD}m/s)')
        ax2.set_ylabel('Velocity Difference (m/s)')
        ax2.legend()
        ax2.grid(True)
        
        # 差异分布
        ax3.hist(self.position_differences, bins=30, alpha=0.5, label='Position')
        ax3.hist(self.velocity_differences, bins=30, alpha=0.5, label='Velocity')
        ax3.set_xlabel('Difference')
        ax3.set_ylabel('Frequency')
        ax3.legend()


    def _validate_paths(self):
        """验证输入路径和创建输出目录"""
        # 检查CSV目录
        if not self.ori_apollo_csv_dir.exists():
            raise NotADirectoryError(f"Original CSV directory not found: {self.ori_apollo_csv_dir}")
        if not self.replay_apollo_csv_dir.exists():
            raise NotADirectoryError(f"Replay CSV directory not found: {self.replay_apollo_csv_dir}")
            
        # 检查必要的CSV文件
        required_files = ['localization_pose.csv', 'planning.csv', 'chassis.csv']
        for file in required_files:
            if not (self.ori_apollo_csv_dir / file).exists():
                raise FileNotFoundError(f"Required file not found: {self.ori_apollo_csv_dir / file}")
            if not (self.replay_apollo_csv_dir / file).exists():
                raise FileNotFoundError(f"Required file not found: {self.replay_apollo_csv_dir / file}")


    def _setup_logger(self) -> logging.Logger:
        """设置日志记录器"""
        logger = logging.getLogger(f'apollo_analysis_{self.timestamp}')
        logger.setLevel(logging.INFO)
        
        # 文件处理器
        log_file = os.path.join(self.current_output_dir, 'apollo_log_analysis.log')
        fh = logging.FileHandler(log_file)
        fh.setLevel(logging.INFO)
        
        # 控制台处理器
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        
        # 格式化器
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        
        logger.addHandler(fh)
        logger.addHandler(ch)
        
        return logger

    def _create_output_dirs(self):
        """创建输出目录结构"""
        dirs = [
            '',  # 根目录
            'figures',  # 图表目录
            # 'figures/trajectory',  # 轨迹相关图表
            # 'figures/planning',    # 规划相关图表
            # 'figures/state',       # 状态相关图表
            # 'figures/dynamics',    # 动态特征图表
            'reports'              # 分析报告
        ]
        for d in dirs:
            os.makedirs(os.path.join(self.current_output_dir, d), exist_ok=True)

    def load_data(self):
        """加载所有数据文件"""
        try:
            self.logger.info("Starting to load data...")

            
            # 加载定位数据
            self.loc1_df = pd.read_csv(os.path.join(self.ori_apollo_csv_dir, 'localization_pose.csv'))
            self.loc2_df = pd.read_csv(os.path.join(self.replay_apollo_csv_dir, 'localization_pose.csv'))
            self.logger.info("after loading localization data")
            
            
            # 加载规划数据
            self.planning1_df = pd.read_csv(os.path.join(self.ori_apollo_csv_dir, 'planning.csv'))
            self.planning2_df = pd.read_csv(os.path.join(self.replay_apollo_csv_dir, 'planning.csv'))
            self.logger.info("after loading planning data")
            
            
            # 加载底盘数据
            self.chassis1_df = pd.read_csv(os.path.join(self.ori_apollo_csv_dir, 'chassis.csv'))
            self.chassis2_df = pd.read_csv(os.path.join(self.replay_apollo_csv_dir, 'chassis.csv'))
            self.logger.info("after loading chassis data")
            
            self._validate_data()
            
        except Exception as e:
            self.logger.error(f"data loading failed: {str(e)}")
            raise

    def _validate_data(self):
        """验证加载的数据是否有效"""
        # 检查数据是否为空
        if (self.loc1_df is None or self.loc2_df is None or 
            self.planning1_df is None or self.planning2_df is None or
            self.chassis1_df is None or self.chassis2_df is None):
            raise ValueError("some data files failed to load")
            
        # 检查数据是否包含必要的列
        required_loc_columns = ['timestamp', 'localization_pose_x', 'localization_pose_y']
        required_planning_columns = ['timestamp', 'adc_position_pose_x', 'adc_position_pose_y']
        required_chassis_columns = ['timestamp', 'chassis_speed_mps']
        
        for df, name, required in [
            (self.loc1_df, 'loc1_df', required_loc_columns),
            (self.loc2_df, 'loc2_df', required_loc_columns),
            (self.planning1_df, 'planning1_df', required_planning_columns),
            (self.planning2_df, 'planning2_df', required_planning_columns),
            (self.chassis1_df, 'chassis1_df', required_chassis_columns),
            (self.chassis2_df, 'chassis2_df', required_chassis_columns)
        ]:
            missing = [col for col in required if col not in df.columns]
            if missing:
                raise ValueError(f"{name} missing necessary columns: {missing}")

    # def _analyze_trajectory_difference(self):
    #     """分析轨迹差异"""
    #     self.logger.info("start analyzing trajectory difference...")
        
    #     # 计算轨迹长度
    #     def calc_trajectory_length(df):
    #         dx = df['localization_pose_x'].diff()
    #         dy = df['localization_pose_y'].diff()
    #         distances = np.sqrt(dx**2 + dy**2)
    #         return np.sum(distances)
        
    #     length1 = calc_trajectory_length(self.loc1_df)
    #     length2 = calc_trajectory_length(self.loc2_df)
        
    #     # 记录长度差异信息
    #     self.length_diff = {
    #         'length1': length1,
    #         'length2': length2,
    #         'diff_meters': abs(length1 - length2),
    #         'diff_percentage': abs(length1 - length2) / min(length1, length2) * 100
    #     }
        
    #     # 确定较短轨迹
    #     if length1 < length2:
    #         self.shorter_df = self.loc1_df
    #         self.longer_df = self.loc2_df
    #         self.logger.info(f"trajectory1 is shorter: {length1:.2f} meters")
    #         self.logger.info(f"trajectory2 is longer: {length2:.2f} meters")
    #     else:
    #         self.shorter_df = self.loc2_df
    #         self.longer_df = self.loc1_df
    #         self.logger.info(f"trajectory2 is shorter: {length2:.2f} meters")
    #         self.logger.info(f"trajectory1 is longer: {length1:.2f} meters")
            
    #     self.logger.info(f"length difference: {self.length_diff['diff_meters']:.2f} meters ({self.length_diff['diff_percentage']:.2f}%)")
        
    #     # 可视化轨迹
    #     self._visualize_trajectories()
        
    #     self.results['trajectory_analysis'] = self.length_diff
        
    def _analyze_trajectory_difference(self):
        """分析轨迹差异"""
        try:
             
            # 获取两条轨迹的长度
            length1 = _calculate_trajectory_length(self.loc1_df)
            length2 = _calculate_trajectory_length(self.loc2_df)
            
            # 检查轨迹是否有效（车辆是否移动）
            MIN_TRAJECTORY_LENGTH = 1.0  # 最小有效轨迹长度（米）
            if length1 < MIN_TRAJECTORY_LENGTH or length2 < MIN_TRAJECTORY_LENGTH:
                self.logger.error(f"Invalid trajectory detected: length1={length1:.2f}m, length2={length2:.2f}m")
                self.results["trajectory_analysis"] = {
                    "error": "Vehicle not moving or invalid trajectory",
                    "length1": length1,
                    "length2": length2,
                    "max_difference": float('inf'),  # 设置一个极大值表示严重错误
                    "status": "HIGH"  # 添加状态标记
                }
                return False
            
            # 正常的轨迹分析逻辑
            self.length_diff = {
                "length1": length1,
                "length2": length2,
                "diff_meters": abs(length1 - length2),
                "diff_percentage": abs(length1 - length2) / max(length1, length2) * 100
            }
        
            # 选择较短的轨迹作为基准
            if length1 <= length2:
                self.shorter_df = self.loc1_df
                self.longer_df = self.loc2_df
            else:
                self.shorter_df = self.loc2_df
                self.longer_df = self.loc1_df
                
            self.results["trajectory_analysis"] = {
                "length_difference": self.length_diff,
                "status": "NORMAL"
            }
            return True
                
        except Exception as e:
            self.logger.error(f"Trajectory difference analysis failed: {str(e)}")
            raise

        
        
        

    def _visualize_trajectories(self):
        """可视化轨迹对比"""
        plt.figure(figsize=(15, 10))
        
        # 绘制轨迹
        plt.plot(self.loc1_df['localization_pose_x'], 
                self.loc1_df['localization_pose_y'], 
                color='darkblue', 
                linewidth=2, 
                label='trajectory1', 
                alpha=0.8)
        
        plt.plot(self.loc2_df['localization_pose_x'], 
                self.loc2_df['localization_pose_y'], 
                color='lightblue', 
                linewidth=2, 
                label='trajectory2', 
                alpha=0.8)
        
        # 标记起点和终点
        
        plt.scatter(self.loc2_df.iloc[0]['localization_pose_x'],
                   self.loc2_df.iloc[0]['localization_pose_y'],
                   color='green', marker='*', s=200, label='start point')
        
        plt.scatter(self.loc2_df.iloc[-1]['localization_pose_x'],
                   self.loc2_df.iloc[-1]['localization_pose_y'],
                   color='red', marker='*', s=200, label='end point')
        
        plt.title(f'Trajectory Comparison\nLength Difference: {self.length_diff["diff_meters"]:.2f} meters ({self.length_diff["diff_percentage"]:.2f}%)', 
                 fontsize=14) 
 
        
        plt.grid(True)
        plt.legend(fontsize=12)
        plt.xlabel('X (m)', fontsize=12)
        plt.ylabel('Y (m)', fontsize=12)
        plt.axis('equal')
        
        # 保存图片
        plt.savefig(os.path.join(self.current_output_dir, 'figures/raw_trajectory_comparison.png'))
        plt.close()

    def _get_key_positions(self, sample_distance: float = 5.0):
            """基于较短轨迹的均匀距离采样选取关键位置点"""

            self.logger.info(f"start selecting key positions, sampling distance: {sample_distance} meters")
            
            if self.shorter_df is None:
                self.logger.error("no baseline trajectory, please run _analyze_trajectory_difference() first")
                raise ValueError("no baseline trajectory, please run _analyze_trajectory_difference() first")
                
            # 计算累计距离
            dx = self.shorter_df['localization_pose_x'].diff()
            dy = self.shorter_df['localization_pose_y'].diff()
            distances = np.sqrt(dx**2 + dy**2)
            cumulative_distance = np.cumsum(distances)
            
            # 均匀采样
            key_positions = []
            current_distance = 0
            while current_distance < cumulative_distance.iloc[-1]:
                idx = (cumulative_distance - current_distance).abs().idxmin()
                pos = (self.shorter_df.iloc[idx]['localization_pose_x'],
                    self.shorter_df.iloc[idx]['localization_pose_y'])
                key_positions.append(pos)
                current_distance += sample_distance
            
            self.key_positions = key_positions
            self.logger.info(f"selected {len(key_positions)} key positions")
            
            
    def _align_trajectories(self, pos_threshold: float = 0.5, time_threshold: float = 0.1):
            """对齐两条轨迹"""
            self.logger.info("start aligning trajectories...")
            if not self.key_positions:
                self.logger.error("no key positions, please run _get_key_positions() first")
                raise ValueError("no key positions, please run _get_key_positions() first")
                
            # 创建轨迹对齐器
            self.aligner = TrajectoryAligner(
                pos_threshold=pos_threshold,
                time_threshold=time_threshold,
                key_positions=self.key_positions
            )
            
            # 对齐轨迹
            
            self.aligned_pairs = self.aligner.align_trajectories(self.loc1_df, self.loc2_df)
        
            self.logger.info(f"found {len(self.aligned_pairs)} aligned pairs")

    def _align_other_data(self, data1_df: pd.DataFrame, data2_df: pd.DataFrame) -> List[Tuple[pd.Series, pd.Series]]:
        """对齐其他数据（如planning数据）"""
        if self.aligner is None or self.aligned_pairs is None:
            self.logger.error("no aligner or aligned pairs, please run _align_trajectories() first")
            raise ValueError("no aligner or aligned pairs, please run _align_trajectories() first")
            
        return self.aligner.align_other_data(
            self.aligned_pairs,
            self.loc1_df,
            self.loc2_df,
            data1_df,
            data2_df
        )
            
            
    def _analyze_planning_decisions(self):
        """分析规划决策"""
        print("-----------------------------analuze planning decisions")
        self.logger.info("start analyzing planning decisions...")
        
        if not self.aligned_pairs:
            self.logger.error("no aligned pairs, please run _align_trajectories() first")
            raise ValueError("no aligned pairs, please run _align_trajectories() first")

        def preprocess_decisions(df):
            """扁平化 decision 字段"""
            decision_records = []
            for _, row in df.iterrows():
 
                
                decision = row['decision']
                # 检查位置数据是否有效
                if pd.isna(row['adc_position_pose_x']) or pd.isna(row['adc_position_pose_y']):
                    continue  # 跳过无效数据
                    
                flattened_decision = {
                    'pose_x': row['adc_position_pose_x'],
                    'pose_y': row['adc_position_pose_y'],
                    'timestamp': row['timestamp']
                }
                if isinstance(decision, dict):
                    flattened_decision.update(decision)
                else:
                    flattened_decision['action'] = decision
                decision_records.append(flattened_decision)
            return pd.DataFrame(decision_records)

        # 预处理决策数据
        planning1_flattened = preprocess_decisions(self.planning1_df)
        planning2_flattened = preprocess_decisions(self.planning2_df)
        
        # 检查是否有足够的有效数据
        if len(planning1_flattened) == 0 or len(planning2_flattened) == 0:
            self.logger.warning("no enough valid planning data for analysis")
            return
        
        # 获取起止时间戳
        start_idx1, start_idx2 = self.aligned_pairs[0]
        end_idx1, end_idx2 = self.aligned_pairs[-1]
        
        start_time1 = self.loc1_df.iloc[start_idx1]['timestamp']
        end_time1 = self.loc1_df.iloc[end_idx1]['timestamp']
        start_time2 = self.loc2_df.iloc[start_idx2]['timestamp']
        end_time2 = self.loc2_df.iloc[end_idx2]['timestamp']
        
        # 筛选时间范围内的planning数据
        planning1_segment = planning1_flattened[
            (planning1_flattened['timestamp'] >= start_time1) & 
            (planning1_flattened['timestamp'] <= end_time1)
        ].copy()
        
        planning2_segment = planning2_flattened[
            (planning2_flattened['timestamp'] >= start_time2) & 
            (planning2_flattened['timestamp'] <= end_time2)
        ].copy()
        
        # 检查筛选后的数据是否足够
        if len(planning1_segment) < 2 or len(planning2_segment) < 2:
            self.logger.warning("not enough valid data points, cannot perform DTW analysis")
            return
        
        # 添加相对时间列
        planning1_segment['relative_time'] = planning1_segment['timestamp'] - start_time1
        planning2_segment['relative_time'] = planning2_segment['timestamp'] - start_time2
        
        # 确保数据是有序的
        planning1_segment = planning1_segment.sort_values('timestamp').reset_index(drop=True)
        planning2_segment = planning2_segment.sort_values('timestamp').reset_index(drop=True)
        
        # 使用DTW对齐两段数据
        seq1 = planning1_segment[['pose_x', 'pose_y']].values
        seq2 = planning2_segment[['pose_x', 'pose_y']].values
        
        self.logger.info(f"DTW input data shape: seq1: {seq1.shape}, seq2: {seq2.shape}")
        
        try:
            _, path = fastdtw(seq1, seq2, dist=euclidean)
            idx1, idx2 = zip(*path)
        except Exception as e:
            self.logger.error(f"DTW analysis failed: {str(e)}")
            return
        
        # 初始化统计结果
        trajectory1_stats = {}
        trajectory2_stats = {}
        similar_position_different_decisions = []

        def is_similar_position(pos1, pos2, threshold=2.0):
            return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2) < threshold

        # 比较DTW对齐后的决策
        for i, j in zip(idx1, idx2):
            row1 = planning1_segment.iloc[i]
            row2 = planning2_segment.iloc[j]
            
            decision1 = row1.get('action', 'UNKNOWN')
            decision2 = row2.get('action', 'UNKNOWN')
            
            # 统计决策
            trajectory1_stats[decision1] = trajectory1_stats.get(decision1, 0) + 1
            trajectory2_stats[decision2] = trajectory2_stats.get(decision2, 0) + 1
            
            pos1 = (row1['pose_x'], row1['pose_y'])
            pos2 = (row2['pose_x'], row2['pose_y'])
            
            if is_similar_position(pos1, pos2) and decision1 != decision2:
                similar_position_different_decisions.append({
                    'position1': pos1,
                    'position2': pos2,
                    'distance': np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2),
                    'decision1': decision1,
                    'decision2': decision2,
                    'relative_time1': row1['relative_time'],
                    'relative_time2': row2['relative_time'],
                    'timestamp1': row1['timestamp'],
                    'timestamp2': row2['timestamp']
                })

        # 转换为DataFrame格式
        trajectory1_df = pd.DataFrame(list(trajectory1_stats.items()), 
                                    columns=['Decision', 'Count'])
        trajectory2_df = pd.DataFrame(list(trajectory2_stats.items()), 
                                    columns=['Decision', 'Count'])
        differences_df = pd.DataFrame(similar_position_different_decisions)

        self.logger.info(f"trajectory1 decision statistics:\n{trajectory1_df}")
        self.logger.info(f"trajectory2 decision statistics:\n{trajectory2_df}")
        self.logger.info(f"found {len(differences_df)} decision differences")

        # 保存分析结果
        self.results['planning_analysis'] = {
            'trajectory1_decisions': trajectory1_df,
            'trajectory2_decisions': trajectory2_df,
            'similar_position_different_decisions': differences_df,
            'planning1_segment': planning1_segment,
            'planning2_segment': planning2_segment,
            'dtw_path': path
        }

    
    
    
    
    def _analyze_trajectory_shape(self):
        """分析轨迹形状和长度差异"""
        self.logger.info("start analyzing trajectory shape...")
        
        if not self.aligned_pairs:
            self.logger.error("no aligned pairs, please run _align_trajectories() first")
            raise ValueError("no aligned pairs, please run _align_trajectories() first")
        
        # 获取时间段内的数据
        start_idx1, start_idx2 = self.aligned_pairs[0]
        end_idx1, end_idx2 = self.aligned_pairs[-1]
        
        start_time1 = self.loc1_df.iloc[start_idx1]['timestamp']
        end_time1 = self.loc1_df.iloc[end_idx1]['timestamp']
        start_time2 = self.loc2_df.iloc[start_idx2]['timestamp']
        end_time2 = self.loc2_df.iloc[end_idx2]['timestamp']
        
        # 截取数据段
        planning1_segment = self.planning1_df[
            (self.planning1_df['timestamp'] >= start_time1) & 
            (self.planning1_df['timestamp'] <= end_time1)
        ].copy()
        
        planning2_segment = self.planning2_df[
            (self.planning2_df['timestamp'] >= start_time2) & 
            (self.planning2_df['timestamp'] <= end_time2)
        ].copy()
            
        # 提取路径坐标
        path1 = planning1_segment[['adc_position_pose_x', 'adc_position_pose_y']].dropna().values
        path2 = planning2_segment[['adc_position_pose_x', 'adc_position_pose_y']].dropna().values
        
        if len(path1) < 2 or len(path2) < 2:
            self.logger.warning("no enough valid trajectory points for analysis")
            return
        
        def compute_path_length(path):
            """计算路径长度"""
            return np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
        
        def normalize_shape(path, num_points=100):
            """归一化路径形状（保持形状特征，忽略尺寸）"""
            # 计算累积距离
            dists = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1))
            cum_dists = np.concatenate(([0], np.cumsum(dists)))
            
            # 归一化到[0,1]
            if cum_dists[-1] == 0:
                return None
            cum_dists = cum_dists / cum_dists[-1]
            
            # 均匀重采样
            try:
                uniform_dists = np.linspace(0, 1, num_points)
                fx = interp1d(cum_dists, path[:, 0], kind='linear')
                fy = interp1d(cum_dists, path[:, 1], kind='linear')
                
                path_resampled = np.column_stack((fx(uniform_dists), fy(uniform_dists)))
                
                # 中心化和缩放
                path_centered = path_resampled - np.mean(path_resampled, axis=0)
                scale = np.max(np.abs(path_centered))
                if scale > 0:
                    path_normalized = path_centered / scale
                else:
                    path_normalized = path_centered
                    
                return path_normalized
                
            except Exception as e:
                self.logger.error(f"shape normalization failed: {str(e)}") 
                return None
    
        # 1. 分析长度差异
        length1 = compute_path_length(path1)
        length2 = compute_path_length(path2)
        length_ratio = min(length1, length2) / max(length1, length2)
        length_diff = abs(length1 - length2)
        
        # 2. 分析形状差异
        path1_norm = normalize_shape(path1)
        path2_norm = normalize_shape(path2)
        
        if path1_norm is None or path2_norm is None:
            self.logger.error("shape normalization failed")
            return
        
        # 计算形状相似度（归一化后的Fréchet距离）
        shape_distance = compute_frechet_distance(path1_norm, path2_norm)
        
        # 3. 计算曲率差异
        def compute_curvature_profile(path):
            """计算路径的曲率特征"""
            dx = np.gradient(path[:, 0])
            dy = np.gradient(path[:, 1])
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            curvature = np.abs(dx * ddy - dy * ddx) / (dx * dx + dy * dy)**1.5
            return np.nan_to_num(curvature)
        
        curvature1 = compute_curvature_profile(path1_norm)
        curvature2 = compute_curvature_profile(path2_norm)
        curvature_diff = np.mean(np.abs(curvature1 - curvature2))
        
        # 保存分析结果
        self.results['trajectory_shape_analysis'] = {
            'length': {
                'path1_length': float(length1),
                'path2_length': float(length2),
                'length_difference': float(length_diff),
                'length_ratio': float(length_ratio)
            },
            'shape': {
                'frechet_distance': float(shape_distance),
                'curvature_difference': float(curvature_diff)
            },
            'raw_data': {
                'path1': path1.tolist(),
                'path2': path2.tolist(),
                'path1_normalized': path1_norm.tolist(),
                'path2_normalized': path2_norm.tolist()
            }
        }
        
        # 记录分析结果
        self.logger.info("=== trajectory analysis results ===")
        self.logger.info(f"trajectory1 length: {length1:.2f} meters")
        self.logger.info(f"trajectory2 length: {length2:.2f} meters")
        self.logger.info(f"length difference: {length_diff:.2f} meters")
        self.logger.info(f"length ratio: {length_ratio:.2%}")
        self.logger.info(f"shape difference (Fréchet distance): {shape_distance:.4f}")
        self.logger.info(f"curvature difference: {curvature_diff:.4f}")
        
        
        
        
    def _analyze_planning_behavior(self):
        """分析规划行为，包括决策和重规划"""
        self.logger.info("start analyzing planning behavior...")
        
        if not self.aligned_pairs:
            self.logger.error("no aligned pairs, please run _align_trajectories() first")
            raise ValueError("no aligned pairs, please run _align_trajectories() first")
        
        # 1. 提取时间段内的重规划事件
        def extract_replan_events(df, start_time, end_time):
            """提取指定时间段内的重规划事件"""
            segment = df[
                (df['timestamp'] >= start_time) & 
                (df['timestamp'] <= end_time)
            ]
            
            replan_events = []
            for _, row in segment[segment['is_replan']].iterrows():
                if pd.isna(row['adc_position_pose_x']) or pd.isna(row['adc_position_pose_y']):
                    continue
                    
                replan_events.append({
                    'timestamp': row['timestamp'],
                    'position': (row['adc_position_pose_x'], row['adc_position_pose_y']),
                    'reason': row['replan_reason'],
                    'decision': row['decision']
                })
                
            return replan_events, segment['is_replan'].sum()
        
        # 获取时间范围
        start_idx1, start_idx2 = self.aligned_pairs[0]
        end_idx1, end_idx2 = self.aligned_pairs[-1]
        
        start_time1 = self.loc1_df.iloc[start_idx1]['timestamp']
        end_time1 = self.loc1_df.iloc[end_idx1]['timestamp']
        start_time2 = self.loc2_df.iloc[start_idx2]['timestamp']
        end_time2 = self.loc2_df.iloc[end_idx2]['timestamp']
        
        # 提取重规划事件
        events1, replan_count1 = extract_replan_events(self.planning1_df, start_time1, end_time1)
        events2, replan_count2 = extract_replan_events(self.planning2_df, start_time2, end_time2)
        
        # 2. 分析重规划位置
        def compare_replan_positions(events1, events2, threshold=2.0):
            """比较两条轨迹的重规划位置"""
            similar_replans = []
            unique_events1 = []
            unique_events2 = []
            
            # 找到相似位置的重规划
            for e1 in events1:
                found_similar = False
                for e2 in events2:
                    dist = np.sqrt(
                        (e1['position'][0] - e2['position'][0])**2 + 
                        (e1['position'][1] - e2['position'][1])**2
                    )
                    if dist < threshold:
                        similar_replans.append({
                            'position1': e1['position'],
                            'position2': e2['position'],
                            'distance': dist,
                            'reason1': e1['reason'],
                            'reason2': e2['reason'],
                            'timestamp1': e1['timestamp'],
                            'timestamp2': e2['timestamp']
                        })
                        found_similar = True
                        break
                if not found_similar:
                    unique_events1.append(e1)
            
            # 找到轨迹2独有的重规划
            for e2 in events2:
                if not any(np.sqrt(
                    (e2['position'][0] - sr['position2'][0])**2 + 
                    (e2['position'][1] - sr['position2'][1])**2
                ) < threshold for sr in similar_replans):
                    unique_events2.append(e2)
            
            return {
                'similar_replans': similar_replans,
                'unique_events1': unique_events1,
                'unique_events2': unique_events2,
                'stats': {
                    'similar_count': len(similar_replans),
                    'unique_count1': len(unique_events1),
                    'unique_count2': len(unique_events2)
                }
            }
        
        replan_comparison = compare_replan_positions(events1, events2, threshold=2.0)
        
        # 3. 整合分析结果
        behavior_analysis = {
            'replanning': {
                'trajectory1': {
                    'total_count': replan_count1,
                    'events': events1,
                    'reasons': self.planning1_df[self.planning1_df['is_replan']]['replan_reason'].value_counts().to_dict()
                },
                'trajectory2': {
                    'total_count': replan_count2,
                    'events': events2,
                    'reasons': self.planning2_df[self.planning2_df['is_replan']]['replan_reason'].value_counts().to_dict()
                },
                'position_comparison': replan_comparison
            }
        }
        
        # 4. 记录分析结果
        self.results['planning_behavior_analysis'] = behavior_analysis
        
        # 5. 记录关键信息
        self.logger.info("\n=== planning behavior comprehensive analysis ===")
        self.logger.info(f"trajectory1 replan count: {replan_count1}")
        self.logger.info(f"trajectory2 replan count: {replan_count2}")
        self.logger.info("\nreplan reason distribution:")
        self.logger.info(f"trajectory1: {behavior_analysis['replanning']['trajectory1']['reasons']}")
        self.logger.info(f"trajectory2: {behavior_analysis['replanning']['trajectory2']['reasons']}")
        self.logger.info("\nreplan position comparison:")
        self.logger.info(f"similar replan positions: {replan_comparison['stats']['similar_count']} 处")
        self.logger.info(f"trajectory1 unique replan positions: {replan_comparison['stats']['unique_count1']} 处")
        self.logger.info(f"trajectory2 unique replan positions: {replan_comparison['stats']['unique_count2']} 处")
        
        
    def _analyze_trajectory_dynamics(self):
        """分析轨迹动态特征"""
        self.logger.info("start analyzing trajectory dynamics...")
        
        if not self.aligned_pairs:
            self.logger.error("no aligned pairs, please run _align_trajectories() first")
            raise ValueError("no aligned pairs, please run _align_trajectories() first")
        
        # 获取起止时间点
        start_idx1, start_idx2 = self.aligned_pairs[0]
        end_idx1, end_idx2 = self.aligned_pairs[-1]
    
        # 获取对应的时间戳
        start_time1 = self.loc1_df.iloc[start_idx1]['timestamp']
        end_time1 = self.loc1_df.iloc[end_idx1]['timestamp']
        start_time2 = self.loc2_df.iloc[start_idx2]['timestamp']
        end_time2 = self.loc2_df.iloc[end_idx2]['timestamp']

        

        
        # 筛选时间段内的底盘数据
        chassis1_segment = self.chassis1_df[
            (self.chassis1_df['timestamp'] >= start_time1) & 
            (self.chassis1_df['timestamp'] <= end_time1)
        ].copy()
        
        chassis2_segment = self.chassis2_df[
            (self.chassis2_df['timestamp'] >= start_time2) & 
            (self.chassis2_df['timestamp'] <= end_time2)
        ].copy()
        
        # 添加相对时间列
        chassis1_segment['relative_time'] = chassis1_segment['timestamp'] - start_time1
        chassis2_segment['relative_time'] = chassis2_segment['timestamp'] - start_time2
        
        # 准备DTW序列
        seq1 = chassis1_segment[['chassis_speed_mps', 'chassis_throttle_percentage', 
                            'chassis_brake_percentage', 'chassis_steering_percentage']].values
        seq2 = chassis2_segment[['chassis_speed_mps', 'chassis_throttle_percentage', 
                            'chassis_brake_percentage', 'chassis_steering_percentage']].values
        
        try:
            # 计算DTW并获取路径
            _, path = fastdtw(seq1, seq2, dist=euclidean)
            idx1, idx2 = zip(*path)
            
            # 创建对齐后的DataFrame
            diff_df = pd.DataFrame({
                'relative_time1': chassis1_segment['relative_time'].iloc[list(idx1)].values,
                'relative_time2': chassis2_segment['relative_time'].iloc[list(idx2)].values,
                'speed1': chassis1_segment['chassis_speed_mps'].iloc[list(idx1)].values,
                'speed2': chassis2_segment['chassis_speed_mps'].iloc[list(idx2)].values,
                'throttle1': chassis1_segment['chassis_throttle_percentage'].iloc[list(idx1)].values,
                'throttle2': chassis2_segment['chassis_throttle_percentage'].iloc[list(idx2)].values,
                'brake1': chassis1_segment['chassis_brake_percentage'].iloc[list(idx1)].values,
                'brake2': chassis2_segment['chassis_brake_percentage'].iloc[list(idx2)].values,
                'steering1': chassis1_segment['chassis_steering_percentage'].iloc[list(idx1)].values,
                'steering2': chassis2_segment['chassis_steering_percentage'].iloc[list(idx2)].values,
            })
            
            # 计算差异
            diff_df['speed_diff'] = diff_df['speed1'] - diff_df['speed2']
            diff_df['throttle_diff'] = diff_df['throttle1'] - diff_df['throttle2']
            diff_df['brake_diff'] = diff_df['brake1'] - diff_df['brake2']
            diff_df['steering_diff'] = diff_df['steering1'] - diff_df['steering2']
            
            # 计算统计信息
            def calculate_stats(series):
                return {
                    'mean': float(series.mean()),
                    'std': float(series.std()),
                    'max': float(series.max()),
                    'min': float(series.min()),
                    'abs_mean': float(series.abs().mean())
                }
            
            stats = {
                'speed': calculate_stats(diff_df['speed_diff']),
                'throttle': calculate_stats(diff_df['throttle_diff']),
                'brake': calculate_stats(diff_df['brake_diff']),
                'steering': calculate_stats(diff_df['steering_diff'])
            }
            
            # 保存分析结果
            self.results['trajectory_dynamics_analysis'] = {
                'time_series': diff_df,
                'statistics': pd.DataFrame(stats)
            }
            
            # 记录分析结果
            self.logger.info("=== dynamics analysis results ===")
            self.logger.info(f"speed difference statistics: {stats['speed']}")
            self.logger.info(f"throttle difference statistics: {stats['throttle']}")
            self.logger.info(f"brake difference statistics: {stats['brake']}")
            self.logger.info(f"steering difference statistics: {stats['steering']}")
            
        except Exception as e:
            self.logger.error(f"dynamics analysis failed: {str(e)}")
            raise
        
        
    
    def _analyze_aligned_states(self):
        print("==================================enter analyze aligned states")
        """分析对齐状态"""
        self.logger.info("start analyzing aligned states...")
        
        if not self.aligned_pairs:
            self.logger.error("no aligned pairs, please run _align_trajectories() first")
            raise ValueError("no aligned pairs, please run _align_trajectories() first")
        
        # 计算planning数据的采样间隔
        planning1_interval = self.planning1_df['timestamp'].diff().median()
        planning2_interval = self.planning2_df['timestamp'].diff().median()
        
        # 设置对齐误差阈值为采样间隔的一半
        time_threshold = min(planning1_interval, planning2_interval) / 2
        
        self.logger.info(f"\ndata sampling information:")
        self.logger.info(f"planning1 sampling interval: {planning1_interval*1000:.1f}ms")
        self.logger.info(f"planning2 sampling interval: {planning2_interval*1000:.1f}ms")
        self.logger.info(f"alignment error threshold: {time_threshold*1000:.1f}ms")
        
        aligned_states = []
        skipped_count = 0
        invalid_data_count = 0
        
        for pair in self.aligned_pairs:
            idx1, idx2 = pair
            # 获取定位数据中的时间戳
            timestamp1 = self.loc1_df.iloc[idx1]['timestamp']
            timestamp2 = self.loc2_df.iloc[idx2]['timestamp']
            
            # 在planning数据中找到最接近的时间点
            p1_idx = self.planning1_df['timestamp'].sub(timestamp1).abs().idxmin()
            p2_idx = self.planning2_df['timestamp'].sub(timestamp2).abs().idxmin()
            
            p1 = self.planning1_df.iloc[p1_idx]
            p2 = self.planning2_df.iloc[p2_idx]
            
            # 检查时间差是否在阈值范围内
            time_diff1 = abs(p1['timestamp'] - timestamp1)
            time_diff2 = abs(p2['timestamp'] - timestamp2)
            
            if time_diff1 > time_threshold or time_diff2 > time_threshold:
                skipped_count += 1
                continue
                
            # 检查位置数据是否有效
            if (pd.isna(p1['adc_position_pose_x']) or pd.isna(p1['adc_position_pose_y']) or
                pd.isna(p2['adc_position_pose_x']) or pd.isna(p2['adc_position_pose_y'])):
                invalid_data_count += 1
                continue
                
            state = {
                # 时间信息
                'timestamp1': timestamp1,
                'timestamp2': timestamp2,
                'time_diff': abs(timestamp1 - timestamp2),
                'planning_time_diff1': time_diff1,
                'planning_time_diff2': time_diff2,
                
                # 位置信息
                'position1_x': p1['adc_position_pose_x'],
                'position1_y': p1['adc_position_pose_y'],
                'position2_x': p2['adc_position_pose_x'],
                'position2_y': p2['adc_position_pose_y'],
                'position_diff': np.sqrt(
                    (p1['adc_position_pose_x'] - p2['adc_position_pose_x'])**2 +
                    (p1['adc_position_pose_y'] - p2['adc_position_pose_y'])**2
                ),
                
                # 速度信息
                'speed1': p1['chassis_speed_mps'],
                'speed2': p2['chassis_speed_mps'],
                'speed_diff': abs(p1['chassis_speed_mps'] - p2['chassis_speed_mps']),
                
                # 朝向信息
                'heading1': p1['heading'],
                'heading2': p2['heading'],
                'heading_diff': abs(p1['heading'] - p2['heading']),
                
                # 控制信息
                'throttle1': p1['chassis_throttle_percentage'],
                'throttle2': p2['chassis_throttle_percentage'],
                'throttle_diff': abs(p1['chassis_throttle_percentage'] - p2['chassis_throttle_percentage']),
                
                'brake1': p1['chassis_brake_percentage'],
                'brake2': p2['chassis_brake_percentage'],
                'brake_diff': abs(p1['chassis_brake_percentage'] - p2['chassis_brake_percentage']),
                
                'steering1': p1['chassis_steering_percentage'],
                'steering2': p2['chassis_steering_percentage'],
                'steering_diff': abs(p1['chassis_steering_percentage'] - p2['chassis_steering_percentage'])
            }
            
            aligned_states.append(state)
        
        if not aligned_states:
            self.logger.warning("no valid aligned states")
            return
        
        df = pd.DataFrame(aligned_states)
        
        # # 计算统计信息
        # stats = df[[
        #     'position_diff', 'speed_diff', 'heading_diff',
        #     'throttle_diff', 'brake_diff', 'steering_diff'
        # ]].describe()
        def calculate_stats(data_dict):
            metrics = [
                'position_diff', 'speed_diff', 'heading_diff',
                'throttle_diff', 'brake_diff', 'steering_diff'
            ]
            
            stats = {}
            for metric in metrics:
                values = data_dict[metric]
                stats[metric] = {
                    'count': int(len(values)),
                    'mean': float(np.mean(values)),
                    'std': float(np.std(values)),
                    'min': float(np.min(values)),
                    '25%': float(np.percentile(values, 25)),
                    '50%': float(np.percentile(values, 50)),
                    '75%': float(np.percentile(values, 75)),
                    'max': float(np.max(values))
                }
            return stats

        
        # 标记显著差异
        significant_diffs = df[
            (df['speed_diff'] > 1.0) |  # 速度差超过1m/s
            (df['heading_diff'] > 0.1) |  # 朝向差超过0.1弧度
            (df['position_diff'] > 0.5)    # 位置差超过0.5米
        ]
        
        # 将significant_diffs转换为可JSON序列化的格式
        significant_diffs_dict = {
            'data': significant_diffs.to_dict(orient='records'),
            # 'columns': significant_diffs.columns.tolist(),
            'index': significant_diffs.index.tolist()
        }
        
        print("|||||||||||||||||||||||||||||||||||||||||||before self.results[ state analysis]")
        # 记录分析结果
        
        def convert_df_to_dict(df):
            """将DataFrame转换为可JSON序列化的字典"""
            return {
                'data': df.to_dict(orient='records'),  # 将DataFrame转换为字典列表
                # 'columns': df.columns.tolist(),        # 保存列名
                'index': df.index.tolist()             # 保存索引
            }

        
        stats = calculate_stats(df.to_dict('list'))
        
        self.results['state_analysis'] = {
            'aligned_states': convert_df_to_dict(df),
            'stats': stats,
            'significant_differences': significant_diffs_dict,
            'metadata': {
                'total_pairs': len(self.aligned_pairs),
                'valid_pairs': len(df),
                'skipped_count': skipped_count,
                'invalid_count': invalid_data_count,
                'planning1_interval': planning1_interval,
                'planning2_interval': planning2_interval,
                'time_threshold': time_threshold
            }
        }
        print("ssssssssssssssssssssssssssstate analysis", self.results['state_analysis'])
        
        # 记录关键信息
        self.logger.info("\n=== state alignment analysis ===")
        self.logger.info(f"total aligned pairs: {len(self.aligned_pairs)}")
        self.logger.info(f"valid aligned pairs: {len(df)}")
        self.logger.info(f"time alignment error too large: {skipped_count}")
        self.logger.info(f"invalid data points: {invalid_data_count}")
        
        if len(significant_diffs) > 0:
            self.logger.info(f"Found {len(significant_diffs)} significant state differences:") 
            for _, row in significant_diffs.iterrows():
                self.logger.info(f"\nTime point: {row['timestamp1']:.3f}/{row['timestamp2']:.3f}")
                self.logger.info(f"Position difference: {row['position_diff']:.2f} meters")
                self.logger.info(f"Speed difference: {row['speed_diff']:.2f} m/s")
                self.logger.info(f"Heading difference: {row['heading_diff']:.3f} rad")
                
                # Output control quantity differences
                if row['throttle_diff'] > 5 or row['brake_diff'] > 5 or row['steering_diff'] > 5:
                    self.logger.info("Control quantity differences:")
                    if row['throttle_diff'] > 5:
                        self.logger.info(f"- Throttle difference: {row['throttle_diff']:.1f}%")
                    if row['brake_diff'] > 5:
                        self.logger.info(f"- Brake difference: {row['brake_diff']:.1f}%")
                    if row['steering_diff'] > 5:
                        self.logger.info(f"- Steering difference: {row['steering_diff']:.1f}%")
           
        
    
    
    
    

    def _generate_report(self, formats=['html', 'markdown', 'txt', 'json']):
        """生成分析报告
        Args:
            formats: 需要生成的报告格式列表，支持 'html', 'markdown', 'txt', 'json'
        """
        self.logger.info(f"start generating analysis report, formats: {formats}")
        
        report_data = {
            'analysis_metadata': {
                'analysis_time': self.timestamp,
                'ori_apollo_csv_dir': self.ori_apollo_csv_dir,
                'replay_apollo_csv_dir':self.replay_apollo_csv_dir,
                'output_directory': self.current_output_dir
            },
            'results': self.results
        }
        
        # 根据指定格式生成报告
        for format in formats:
            try:
                if format.lower() == 'html':
                    self._generate_html_report(report_data)
                elif format.lower() == 'markdown':
                    self._generate_markdown_report(report_data)
                elif format.lower() == 'txt':
                    self._generate_txt_report(report_data)
                elif format.lower() == 'json':
                    self._generate_json_report(report_data)
                else:
                    self.logger.warning(f"unsupported report format: {format}")
            except Exception as e:
                self.logger.error(f"failed to generate {format} format report: {str(e)}")
                
        self.logger.info("report generation completed")

    def _generate_markdown_report(self, report_data):
        """生成Markdown格式的分析报告"""
        markdown_content = f"""# Apollo轨迹分析报告

## 分析基本信息
- 分析时间: {self.timestamp}
- ori日志目录: {self.ori_apollo_csv_dir}
- replay日志目录: {self.replay_apollo_csv_dir}


## 1. 轨迹基本信息
| 项目 | 数值 |
|------|------|
| 轨迹1长度 | {self.length_diff['length1']:.2f} 米 |
| 轨迹2长度 | {self.length_diff['length2']:.2f} 米 |
| 长度差异 | {self.length_diff['diff_meters']:.2f} 米 ({self.length_diff['diff_percentage']:.2f}%) |

## 2. 轨迹对比
![轨迹对比图](../figures/trajectory/raw_comparison.png)

## 3. 规划决策分析
发现 {len(self.results['planning_analysis']['different_decisions'])} 处决策差异

## 4. 轨迹形状分析
- DTW距离: {self.results['trajectory_shape_analysis']['dtw_distance']:.4f}

## 5. 规划行为分析
发现 {self.results['planning_behavior_analysis']['total_differences']} 处行为差异

## 6. 状态差异分析
发现 {self.results['state_analysis']['total_differences']} 处状态差异

### 图表索引
- 轨迹分析:
  - 原始轨迹对比: `figures/trajectory/raw_comparison.png`
  - 关键位置点分布: `figures/trajectory/key_positions.png`
  - 轨迹对齐结果: `figures/trajectory/aligned_trajectories.png`
- 状态分析:
  - 速度差异: `figures/state/speed_diff.png`
  - 位置差异: `figures/state/position_diff.png`
  - 控制量差异: `figures/state/control_diff_box.png`
"""
        
        report_path = os.path.join(self.current_output_dir, 'reports', 'analysis_report.md')
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(markdown_content)
            
        self.logger.info(f"Markdown报告已生成: {report_path}")

    def _generate_txt_report(self, report_data):
        """生成纯文本格式的分析报告"""
        txt_content = f"""Apollo轨迹分析报告
{'='*50}

分析基本信息:
  分析时间: {self.timestamp}
  ori日志目录: {self.ori_apollo_csv_dir}
  replay日志目录: {self.replay_apollo_csv_dir}

1. 轨迹基本信息:
  - 轨迹1长度: {self.length_diff['length1']:.2f} 米
  - 轨迹2长度: {self.length_diff['length2']:.2f} 米
  - 长度差异: {self.length_diff['diff_meters']:.2f} 米 ({self.length_diff['diff_percentage']:.2f}%)

2. 规划决策分析:
  发现 {len(self.results['planning_analysis']['different_decisions'])} 处决策差异

3. 轨迹形状分析:
  DTW距离: {self.results['trajectory_shape_analysis']['dtw_distance']:.4f}

4. 规划行为分析:
  发现 {self.results['planning_behavior_analysis']['total_differences']} 处行为差异

5. 状态差异分析:
  发现 {self.results['state_analysis']['total_differences']} 处状态差异

6. 图表文件位置:
  - 轨迹对比图: figures/trajectory/raw_comparison.png
  - 速度差异图: figures/state/speed_diff.png
  - 位置差异图: figures/state/position_diff.png
"""
        
        report_path = os.path.join(self.current_output_dir, 'reports', 'analysis_report.txt')
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(txt_content)
            
        self.logger.info(f"text report generated: {report_path}")

    def _generate_json_report(self, report_data):
        """生成JSON格式的分析报告"""
        report_path = os.path.join(self.current_output_dir, 'reports', 'analysis_report.json')
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=4, ensure_ascii=False)
            
        self.logger.info(f"JSON report generated: {report_path}")
        
        
        
    def _generate_html_report(self, report_data):
        """生成HTML格式的分析报告"""
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Apollo轨迹分析报告 - {self.timestamp}</title>
            <style>
                body {{ font-family: Arial, sans-serif; margin: 40px; }}
                .section {{ margin-bottom: 30px; }}
                .figure {{ margin: 20px 0; }}
                .figure img {{ max-width: 100%; }}
                table {{ border-collapse: collapse; width: 100%; }}
                th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
                th {{ background-color: #f2f2f2; }}
            </style>
        </head>
        <body>
            <h1>Apollo轨迹分析报告</h1>
            <p>分析时间: {self.timestamp}</p>
            
            <div class="section">
                <h2>1. 轨迹基本信息</h2>
                <table>
                    <tr><th>项目</th><th>数值</th></tr>
                    <tr><td>轨迹1长度</td><td>{self.length_diff['length1']:.2f} 米</td></tr>
                    <tr><td>轨迹2长度</td><td>{self.length_diff['length2']:.2f} 米</td></tr>
                    <tr><td>长度差异</td><td>{self.length_diff['diff_meters']:.2f} 米 ({self.length_diff['diff_percentage']:.2f}%)</td></tr>
                </table>
            </div>
            
            <div class="section">
                <h2>2. 轨迹对比图</h2>
                <div class="figure">
                    <img src="../figures/trajectory/raw_comparison.png" alt="轨迹对比">
                </div>
            </div>
            
            <div class="section">
                <h2>3. 规划决策分析</h2>
                <p>发现 {len(self.results['planning_analysis']['different_decisions'])} 处决策差异</p>
            </div>
            
            <div class="section">
                <h2>4. 轨迹形状分析</h2>
                <p>DTW距离: {self.results['trajectory_shape_analysis']['dtw_distance']:.4f}</p>
            </div>
            
            <div class="section">
                <h2>5. 规划行为分析</h2>
                <p>发现 {self.results['planning_behavior_analysis']['total_differences']} 处行为差异</p>
            </div>
            
            <div class="section">
                <h2>6. 状态差异分析</h2>
                <p>发现 {self.results['state_analysis']['total_differences']} 处状态差异</p>
            </div>
        </body>
        </html>
        """
        
        report_path = os.path.join(self.current_output_dir, 'reports', 'analysis_report.html')
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(html_content)

    #wei: 没有生成reports 和 各种figure的原因是因为，这里我的实验数据中，replay的时候没有数据，所以没有执行后面的代码
    def analyze(self):
        """执行完整的分析流程"""
        try:
            self.logger.info("start executing the full analysis process...")
            
            # 1. 加载数据
            self.load_data()
            
            #TODO: For some data, the trajectory difference is very large so not running the following analyze part 
            # 2. 轨迹差异分析
            if not self._analyze_trajectory_difference():
                self.logger.warning("Trajectory analysis indicates severe issues, skipping detailed analysis")
                return False
            
            self._visualize_trajectories()
            
            # 3. 选取关键位置点
            self._get_key_positions(sample_distance=5.0)
             
            
            # 4. 轨迹对齐
            self._align_trajectories(pos_threshold=0.5, time_threshold=0.1)
            
            # 5. 规划决策分析--done
            self._analyze_planning_decisions()

            self._plot_planning_decisions()
            
            # 6. 轨迹形状分析--done
            self._analyze_trajectory_shape()

            
            # 7. 规划行为分析--done
            self._analyze_planning_behavior()
            
            # 8. 轨迹动态特征分析--done
            self._analyze_trajectory_dynamics()
            self._plot_speed_and_acc_comparison() 
            
            # 9. 状态对齐分析
            self._analyze_aligned_states()
            # self._plot_differences()
            #TODO: 感觉有了上面几张图，这个函数也不是很必要
            
            # 10. 生成报告
            self._generate_report()
            
            self.logger.info("analysis process completed")
            return True
            
        except Exception as e:
            self.logger.error(f"analysis process error: {str(e)}")
            raise

if __name__ == "__main__":
    ori_apollo_csv_dir = "/path/to/log/directory"
    replay_apollo_csv_dir = "/path/to/log/directory"
    
    output_dir = "/path/to/output/directory"
    
    analyzer = LogAnalyzer(ori_apollo_csv_dir, replay_apollo_csv_dir, output_dir)
    analyzer.analyze()
