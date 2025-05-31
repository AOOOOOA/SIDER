import pandas as pd
import numpy as np
from typing import List, Tuple
import matplotlib.pyplot as plt



########################################################
###1.Analyze the difference between two trajectories####
########################################################


def analyze_trajectory_lengths(loc1_df: pd.DataFrame, loc2_df: pd.DataFrame):
    """
    分析两条轨迹的长度差异，并确定较短的轨迹作为基准
    Args:
        loc1_df: 第一条轨迹的定位数据
        loc2_df: 第二条轨迹的定位数据
    Returns:
        shorter_df: 较短的轨迹数据
        longer_df: 较长的轨迹数据
        length_diff: 包含长度差异信息的字典
    """
    # 计算两条轨迹的长度
    def calc_trajectory_length(df):
        dx = df['localization_pose_x'].diff()
        dy = df['localization_pose_y'].diff()
        distances = np.sqrt(dx**2 + dy**2)
        return np.sum(distances)
    
    length1 = calc_trajectory_length(loc1_df)
    length2 = calc_trajectory_length(loc2_df)
    #test
    # 记录长度差异信息
    length_diff = {
        'length1': length1,
        'length2': length2,
        'diff_meters': abs(length1 - length2),
        'diff_percentage': abs(length1 - length2) / min(length1, length2) * 100
    }
    
    # 确定哪条轨迹更短
    if length1 < length2:
        shorter_df = loc1_df
        longer_df = loc2_df
        print(f"轨迹1更短: {length1:.2f}米")
        print(f"轨迹2更长: {length2:.2f}米")
    else:
        shorter_df = loc2_df
        longer_df = loc1_df
        print(f"轨迹2更短: {length2:.2f}米")
        print(f"轨迹1更长: {length1:.2f}米")
        
    print(f"长度差异: {length_diff['diff_meters']:.2f}米 ({length_diff['diff_percentage']:.2f}%)")
    
    return shorter_df, longer_df, length_diff
#test
def visualize_raw_trajectories(loc1_df: pd.DataFrame, loc2_df: pd.DataFrame, length_diff: dict = None):
    """
    可视化两条原始轨迹，并标注长度差异
    """
    plt.figure(figsize=(15, 10))
    
    # 绘制轨迹
    plt.plot(loc1_df['localization_pose_x'], 
            loc1_df['localization_pose_y'], 
            color='darkblue', 
            linewidth=2, 
            label='轨迹1', 
            alpha=0.8)
    
    plt.plot(loc2_df['localization_pose_x'], 
            loc2_df['localization_pose_y'], 
            color='lightblue', 
            linewidth=2, 
            label='轨迹2', 
            alpha=0.8)
    
    # 标记起点和终点
    plt.scatter(loc1_df.iloc[0]['localization_pose_x'],
               loc1_df.iloc[0]['localization_pose_y'],
               color='green', marker='*', s=200, label='起点')
    
    plt.scatter(loc1_df.iloc[-1]['localization_pose_x'],
               loc1_df.iloc[-1]['localization_pose_y'],
               color='red', marker='*', s=200, label='终点')
    
    # 添加长度差异信息
    if length_diff:
        plt.title(f'两条轨迹对比\n长度差异: {length_diff["diff_meters"]:.2f}米 ({length_diff["diff_percentage"]:.2f}%)', 
                 fontsize=14)
    else:
        plt.title('两条轨迹对比', fontsize=14)
    
    plt.grid(True)
    plt.legend(fontsize=12)
    plt.xlabel('X (m)', fontsize=12)
    plt.ylabel('Y (m)', fontsize=12)
    plt.axis('equal')
    plt.show()
    
    
def get_key_positions_by_distance(loc_df: pd.DataFrame, sample_distance: float = 20.0) -> List[Tuple[float, float]]:
    """
    基于较短轨迹的均匀距离采样选取关键位置点
    """
    key_positions = []
    
    # 计算累计距离
    dx = loc_df['localization_pose_x'].diff()
    dy = loc_df['localization_pose_y'].diff()
    distances = np.sqrt(dx**2 + dy**2)
    cumulative_distance = np.cumsum(distances)
    
    # 均匀采样
    current_distance = 0
    while current_distance < cumulative_distance.iloc[-1]:
        idx = (cumulative_distance - current_distance).abs().idxmin()
        pos = (loc_df.iloc[idx]['localization_pose_x'],
               loc_df.iloc[idx]['localization_pose_y'])
        key_positions.append(pos)
        current_distance += sample_distance
    
    return key_positions

class TrajectoryAligner:
    """
    轨迹对齐器类，用于对齐两条轨迹的关键点和相关数据
    """
    def __init__(self, pos_threshold: float = 0.5, time_threshold: float = 0.1,
                 key_positions: List[Tuple[float, float]] = None):
        """
        初始化参数
        Args:
            pos_threshold: 位置匹配阈值(米)，判断两个点是否匹配的距离阈值
            time_threshold: 时间匹配阈值(秒)，判断两个点是否匹配的时间阈值
            key_positions: 关键位置点列表，每个元素为(x,y)坐标元组
        """
        self.pos_threshold = pos_threshold
        self.time_threshold = time_threshold
        self.key_positions = key_positions or []

    def find_key_points(self, loc_df: pd.DataFrame) -> List[int]:
        """
        在轨迹中找到最接近关键位置点的数据点索引
        Args:
            loc_df: 定位数据
        Returns:
            关键点在数据中的索引列表
        """
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
        """
        对齐两条轨迹的关键点
        Args:
            loc1_df: 第一条轨迹的定位数据
            loc2_df: 第二条轨迹的定位数据
        Returns:
            匹配点对的索引列表，每个元素为(idx1, idx2)
        """
        key_indices1 = self.find_key_points(loc1_df)
        key_indices2 = self.find_key_points(loc2_df)
        
        if len(key_indices1) != len(key_indices2):
            print("警告：两条轨迹匹配到的关键点数量不同")
            
        return list(zip(key_indices1, key_indices2))




    def align_other_data(self, aligned_pairs: List[Tuple[int, int]], 
                        loc1_df: pd.DataFrame, loc2_df: pd.DataFrame,
                        data1_df: pd.DataFrame, data2_df: pd.DataFrame) -> List[Tuple[pd.Series, pd.Series]]:
        """
        根据定位数据的时间戳对齐其他数据（如planning数据）
        Args:
            aligned_pairs: 已对齐的定位点索引对
            loc1_df, loc2_df: 两条轨迹的定位数据
            data1_df, data2_df: 需要对齐的其他数据
        Returns:
            对齐后的数据点对列表
        """
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


#FIXME: need more detailed and comprehensive analysis
#combine with the detail in the planning data to analyze and give the results.
def analyze_trajectory_differences(aligned_data: List[Tuple[pd.Series, pd.Series]]):
    """
    分析关键位置点处的轨迹差异
    Args:
        aligned_data: 对齐后的数据点对列表
    Returns:
        包含各项差异指标的DataFrame
    """
    differences = []
    for i, (data1, data2) in enumerate(aligned_data):
        # 计算各项指标的差异
        diff = {
            'position': f"({data1['localization_pose_x']:.2f}, {data1['localization_pose_y']:.2f})",
            'speed_diff': abs(data1['speed_mps'] - data2['speed_mps']),  # 速度差
            'acc_diff': abs(data1.get('acceleration', 0) - data2.get('acceleration', 0)),  # 加速度差
            'heading_diff': abs(data1.get('heading', 0) - data2.get('heading', 0)),  # 航向角差
            'timestamp1': data1['timestamp'],  # 记录时间戳便于对照
            'timestamp2': data2['timestamp']
        }
        differences.append(diff)
        
    return pd.DataFrame(differences)

def visualize_trajectories(loc1_df: pd.DataFrame, loc2_df: pd.DataFrame, 
                         key_positions: List[Tuple[float, float]], 
                         aligned_pairs: List[Tuple[int, int]]):
    """
    可视化轨迹和关键点
    Args:
        loc1_df, loc2_df: 两条轨迹的定位数据
        key_positions: 关键位置点列表
        aligned_pairs: 匹配点对的索引列表
    """
    plt.figure(figsize=(12, 8))
    
    # 绘制完整轨迹
    plt.plot(loc1_df['localization_pose_x'], loc1_df['localization_pose_y'], 'b-', label='轨迹1', alpha=0.5)
    plt.plot(loc2_df['localization_pose_x'], loc2_df['localization_pose_y'], 'r-', label='轨迹2', alpha=0.5)
    
    # 绘制关键位置点
    key_pos_x, key_pos_y = zip(*key_positions)
    plt.scatter(key_pos_x, key_pos_y, c='g', marker='*', s=200, label='关键位置点')
    
    # 绘制匹配点
    for idx1, idx2 in aligned_pairs:
        plt.scatter(loc1_df.iloc[idx1]['localization_pose_x'], 
                   loc1_df.iloc[idx1]['localization_pose_y'], 
                   c='b', marker='o')
        plt.scatter(loc2_df.iloc[idx2]['localization_pose_x'], 
                   loc2_df.iloc[idx2]['localization_pose_y'], 
                   c='r', marker='o')
    
    plt.legend()
    plt.grid(True)
    plt.title('轨迹对比分析')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.show()



#TODO: change the plot function here to a image saving one
def plot_differences(differences_df: pd.DataFrame):
    """
    绘制差异指标的柱状图
    Args:
        differences_df: 包含差异数据的DataFrame
    """
    plt.figure(figsize=(10, 6))
    differences_df[['speed_diff', 'acc_diff', 'heading_diff']].plot(kind='bar')
    plt.title('关键位置点的差异对比')
    plt.ylabel('差异值')
    plt.xlabel('位置点索引')
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()





"""
主函数，用于执行完整的分析流程
"""
# 1. 加载数据
dir="/home/w/workspace/carla_apollo/apollo/modules/carla_bridge/multi_vehicle_fuzz/apollo_log_csv/"
loc1_df = pd.read_csv(dir+'ori/'+"localization_pose.csv")
loc2_df = pd.read_csv(dir+'replay/'+"localization_pose.csv")

planning1_df = pd.read_csv(dir+'ori/'+"planning.csv")
planning2_df = pd.read_csv(dir+'replay/'+"planning.csv")


chassis1_df = pd.read_csv(dir+'ori/'+"chassis.csv")
chassis2_df = pd.read_csv(dir+'replay/'+"chassis.csv")

# 1. 首先分析轨迹长度并确定基准轨迹
shorter_df, longer_df, length_diff = analyze_trajectory_lengths(loc1_df, loc2_df)

# 2. 可视化原始轨迹（包含长度差异信息）
visualize_raw_trajectories(loc1_df, loc2_df, length_diff)


#TODO: in the future, this should be an API
# 3. 基于较短轨迹选取关键点
key_positions = get_key_positions_by_distance(
    loc_df=shorter_df,  # 使用较短的轨迹作为基准
    sample_distance=20.0  # 每20米选一个点
)

# 3. 创建对齐器
aligner = TrajectoryAligner(
    pos_threshold=0.5,  # 位置匹配阈值
    time_threshold=0.1,  # 时间匹配阈值
    key_positions=key_positions
)

# 4. 对齐轨迹
aligned_pairs = aligner.align_trajectories(loc1_df, loc2_df)





########################################################
###2.Analyze the planning Decisions#####################
########################################################



#这里planning1_df 中的adc position 有NA值，为什么？--  这个时候显示的是not ready.
#应该移除最开始部分的NA值，然后重新对齐--done. 但是因为我的数据暂时是稍微有点问题的，所以先搁置。
#TODO: 等到数据正常之后回头看下这部分的代码是不是正常的

def analyze_planning_decisions(planning1_df, planning2_df, aligned_pairs, loc1_df, loc2_df):
    """分析决策统计信息"""
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
    planning1_flattened = preprocess_decisions(planning1_df)
    planning2_flattened = preprocess_decisions(planning2_df)
    
    # 检查是否有足够的有效数据
    if len(planning1_flattened) == 0 or len(planning2_flattened) == 0:
        print("警告：没有足够的有效planning数据进行分析")
        return None
    
    # 获取起止时间戳
    start_idx1, start_idx2 = aligned_pairs[0]
    end_idx1, end_idx2 = aligned_pairs[-1]
    
    start_time1 = loc1_df.iloc[start_idx1]['timestamp']
    end_time1 = loc1_df.iloc[end_idx1]['timestamp']
    start_time2 = loc2_df.iloc[start_idx2]['timestamp']
    end_time2 = loc2_df.iloc[end_idx2]['timestamp']
    
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
        print("警告：筛选后的数据点不足，无法进行DTW分析")
        return None
    
    # 添加相对时间列
    planning1_segment['relative_time'] = planning1_segment['timestamp'] - start_time1
    planning2_segment['relative_time'] = planning2_segment['timestamp'] - start_time2
    
    # 确保数据是有序的
    planning1_segment = planning1_segment.sort_values('timestamp').reset_index(drop=True)
    planning2_segment = planning2_segment.sort_values('timestamp').reset_index(drop=True)
    
    # 使用DTW对齐两段数据
    seq1 = planning1_segment[['pose_x', 'pose_y']].values
    seq2 = planning2_segment[['pose_x', 'pose_y']].values
    
    print(f"DTW输入数据形状: seq1: {seq1.shape}, seq2: {seq2.shape}")
    
    try:
        _, path = fastdtw(seq1, seq2, dist=euclidean)
        idx1, idx2 = zip(*path)
    except Exception as e:
        print(f"DTW分析失败: {str(e)}")
        return None
    
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

    print("\n决策分析结果:")
    print(f"轨迹1决策统计:\n{trajectory1_df}")
    print(f"\n轨迹2决策统计:\n{trajectory2_df}")
    print(f"\n发现 {len(differences_df)} 处决策差异")

    return {
        'trajectory1_decisions': trajectory1_df,
        'trajectory2_decisions': trajectory2_df,
        'similar_position_different_decisions': differences_df,
        'planning1_segment': planning1_segment,
        'planning2_segment': planning2_segment,
        'dtw_path': path
    }





######################################################################################
###3.Compare the dynamics of the vehicles in the two trajectories#####################
######################################################################################


# localization 中的数据不全， 我们应该用chassis 中的数据。 planning 和control中的都是期望值，并不是真实值，我们这里需要计算真实值
# 现在的对齐是先用aligned_pairs 对齐两端，中间的部分用dtw进行对齐。 可能还有问题，暂时先这样。
#FIXME: 这里我还需要调整一下上面的decision对比的部分。--最后分析的时候，这个画图的部分应该写成一个存图的方式。


from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def calculate_stats(series):
    """Calculate statistics for a difference series
    Args:
        series: pandas Series containing difference values
    Returns:
        dict containing statistical measures
    """
    return {
        'max_diff': series.max(),
        'min_diff': series.min(),
        'mean_diff': series.mean(),
        'std_diff': series.std(),
        'rmse': np.sqrt((series**2).mean())
    }
    
def plot_dynamics_diff(diff_df):
    """Plot dynamics differences and comparisons after DTW alignment
    Args:
        diff_df: DataFrame containing aligned values and differences
    Returns:
        matplotlib figure
    """
    fig, axes = plt.subplots(4, 2, figsize=(15, 16))
    
    # Raw data comparison (left column)
    plot_pairs = [
        ('speed', 'm/s'), 
        ('throttle', '%'),
        ('brake', '%'), 
        ('steering', '%')
    ]
    
    for i, (metric, unit) in enumerate(plot_pairs):
        # Original data comparison - using separate time series
        axes[i,0].plot(diff_df['relative_time1'], diff_df[f'{metric}1'], 
                      'b-', label='Trajectory 1', alpha=0.7)
        axes[i,0].plot(diff_df['relative_time2'], diff_df[f'{metric}2'], 
                      'r--', label='Trajectory 2', alpha=0.7)
        axes[i,0].set_title(f'{metric.capitalize()} Comparison')
        axes[i,0].set_ylabel(unit)
        axes[i,0].grid(True)
        axes[i,0].legend()
        
        # Difference plot - using time from trajectory 1
        axes[i,1].plot(diff_df['relative_time1'], diff_df[f'{metric}_diff'])
        axes[i,1].set_title(f'{metric.capitalize()} Difference')
        axes[i,1].set_ylabel(unit)
        axes[i,1].grid(True)
        
        # Add DTW matching lines for a subset of points
        if i == 0:  # Only for speed plot
            # Draw matching lines for every Nth point
            N = len(diff_df) // 20  # Adjust N to control density of matching lines
            for j in range(0, len(diff_df), N):
                axes[i,0].plot([diff_df['relative_time1'].iloc[j], 
                              diff_df['relative_time2'].iloc[j]],
                             [diff_df[f'{metric}1'].iloc[j], 
                              diff_df[f'{metric}2'].iloc[j]],
                             'g-', alpha=0.1)
    
    # Add x-labels to bottom plots
    axes[3,0].set_xlabel('Time (s)')
    axes[3,1].set_xlabel('Time (s)')
    
    plt.tight_layout()
    return fig

def analyze_trajectory_dynamics(chassis1_df, chassis2_df, loc1_df, loc2_df, aligned_pairs):
    """Analyze trajectory dynamics using chassis data between aligned points"""
    # Get start and end indices from aligned pairs
    start_idx1, start_idx2 = aligned_pairs[0]
    end_idx1, end_idx2 = aligned_pairs[-1]
    
    # Get corresponding timestamps from localization data
    start_time1 = loc1_df.iloc[start_idx1]['timestamp']
    end_time1 = loc1_df.iloc[end_idx1]['timestamp']
    start_time2 = loc2_df.iloc[start_idx2]['timestamp']
    end_time2 = loc2_df.iloc[end_idx2]['timestamp']
    
    # Filter chassis data between start and end timestamps
    chassis1_segment = chassis1_df[
        (chassis1_df['timestamp'] >= start_time1) & 
        (chassis1_df['timestamp'] <= end_time1)
    ].copy()
    
    chassis2_segment = chassis2_df[
        (chassis2_df['timestamp'] >= start_time2) & 
        (chassis2_df['timestamp'] <= end_time2)
    ].copy()
    
    # Add relative time column
    chassis1_segment['relative_time'] = chassis1_segment['timestamp'] - start_time1
    chassis2_segment['relative_time'] = chassis2_segment['timestamp'] - start_time2
    
    # Prepare sequences for DTW
    seq1 = chassis1_segment[['chassis_speed_mps', 'chassis_throttle_percentage', 
                            'chassis_brake_percentage', 'chassis_steering_percentage']].values
    seq2 = chassis2_segment[['chassis_speed_mps', 'chassis_throttle_percentage', 
                            'chassis_brake_percentage', 'chassis_steering_percentage']].values
    
    # Calculate DTW and get path
    print("=======================seq1", seq1)
    print("=======================seq2", seq2)
    
    _, path = fastdtw(seq1, seq2, dist=euclidean)
    idx1, idx2 = zip(*path)
    
    # Create aligned DataFrame
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
    
    # Calculate differences
    diff_df['speed_diff'] = diff_df['speed1'] - diff_df['speed2']
    diff_df['throttle_diff'] = diff_df['throttle1'] - diff_df['throttle2']
    diff_df['brake_diff'] = diff_df['brake1'] - diff_df['brake2']
    diff_df['steering_diff'] = diff_df['steering1'] - diff_df['steering2']
    
    # Calculate statistics
    stats = {
        'speed': calculate_stats(diff_df['speed_diff']),
        'throttle': calculate_stats(diff_df['throttle_diff']),
        'brake': calculate_stats(diff_df['brake_diff']),
        'steering': calculate_stats(diff_df['steering_diff'])
    }
    
    return {
        'time_series': diff_df,
        'statistics': pd.DataFrame(stats),
        'plot_dynamics': lambda: plot_dynamics_diff(diff_df)
    }




#TODO: move the calling part to the end of the file 
dynamics_analysis = analyze_trajectory_dynamics(
    chassis1_df, 
    chassis2_df, 
    loc1_df, 
    loc2_df, 
    aligned_pairs
)

# 打印统计结果
print("\n动态特征差异统计:")
print(dynamics_analysis['statistics'])


#change the plot image to the saving image
# 绘制时序图
dynamics_analysis['plot_dynamics']()
plt.show()
    
    
    
    
    
    
    
    

######################################################################################
### 4. compare the trajectory shape #################################################
######################################################################################

    
    
    
    #对比轨迹的shape--暂时看起来没有什么很大问题
# TODO: 之后对于形状差异很大的数据应该是需要再有一些局部的额外分析的。

from scipy.spatial.distance import directed_hausdorff
from scipy.interpolate import interp1d
import numpy as np

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
        print(f"轨迹归一化失败: {str(e)}")
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
        print(f"Fréchet距离计算失败: {str(e)}")
        return float('inf')

def analyze_trajectory_shape(planning1_df, planning2_df, loc1_df, loc2_df, aligned_pairs):
    """分析轨迹形状和长度差异
    """
    # 获取时间段内的数据
    start_idx1, start_idx2 = aligned_pairs[0]
    end_idx1, end_idx2 = aligned_pairs[-1]
    
    start_time1 = loc1_df.iloc[start_idx1]['timestamp']
    end_time1 = loc1_df.iloc[end_idx1]['timestamp']
    start_time2 = loc2_df.iloc[start_idx2]['timestamp']
    end_time2 = loc2_df.iloc[end_idx2]['timestamp']
    
    # 截取数据段
    planning1_segment = planning1_df[
        (planning1_df['timestamp'] >= start_time1) & 
        (planning1_df['timestamp'] <= end_time1)
    ].copy()
    
    planning2_segment = planning2_df[
        (planning2_df['timestamp'] >= start_time2) & 
        (planning2_df['timestamp'] <= end_time2)
    ].copy()
    
    # 提取路径坐标
    path1 = planning1_segment[['adc_position_pose_x', 'adc_position_pose_y']].dropna().values
    path2 = planning2_segment[['adc_position_pose_x', 'adc_position_pose_y']].dropna().values
    
    if len(path1) < 2 or len(path2) < 2:
        print("警告：没有足够的有效轨迹点进行分析")
        return None
        
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
            print(f"形状归一化失败: {str(e)}")
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
        print("形状归一化失败")
        return None
    
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
    
    results = {
        # 长度相关指标
        'length': {
            'path1_length': length1,
            'path2_length': length2,
            'length_difference': length_diff,
            'length_ratio': length_ratio
        },
        # 形状相关指标
        'shape': {
            'frechet_distance': shape_distance,
            'curvature_difference': curvature_diff
        },
        # 原始数据
        'raw_data': {
            'path1': path1,
            'path2': path2,
            'path1_normalized': path1_norm,
            'path2_normalized': path2_norm
        }
    }
    
    # 打印分析结果
    print("\n=== 轨迹分析结果 ===")
    print("\n1. 长度分析:")
    print(f"轨迹1长度: {length1:.2f}米")
    print(f"轨迹2长度: {length2:.2f}米")
    print(f"长度差异: {length_diff:.2f}米")
    print(f"长度比例: {length_ratio:.2%}")
    
    print("\n2. 形状分析:")
    print(f"形状差异(Fréchet距离): {shape_distance:.4f}")
    print(f"曲率差异: {curvature_diff:.4f}")
    
    # 解释结果
    print("\n=== 分析解释 ===")
    # 长度解释
    if length_ratio > 0.95:
        print("长度: 两条轨迹长度基本一致")
    elif length_ratio > 0.8:
        print("长度: 两条轨迹存在一定长度差异")
    else:
        print("长度: 两条轨迹长度差异显著")
    
    # 形状解释
    if shape_distance < 0.1:
        print("形状: 轨迹形状几乎完全相同")
    elif shape_distance < 0.3:
        print("形状: 轨迹形状非常相似")
    elif shape_distance < 0.5:
        print("形状: 轨迹形状基本相似")
    else:
        print("形状: 轨迹形状存在明显差异")
    
    return results




#TODO: 
results = analyze_trajectory_shape(
    planning1_df, 
    planning2_df, 
    loc1_df, 
    loc2_df, 
    aligned_pairs
)

# 可以进一步访问具体指标
if results:
    length_ratio = results['length']['length_ratio']
    shape_distance = results['shape']['frechet_distance']






    
    
    

######################################################################################
### 5. compare the replan decision(together with the previous decision comparison) ###
######################################################################################

    


def create_spatial_index(events, threshold):
    """创建空间网格索引
    Args:
        events: 重规划事件列表，每个事件包含position信息
        threshold: 位置匹配的阈值距离
    Returns:
        dict: 网格索引，键为网格坐标(x,y)，值为该网格内的事件列表
    """
    grid = {}
    cell_size = threshold * 2  # 网格大小设为阈值的2倍
    
    # 示意图：
    # 假设 threshold = 2.0米，则 cell_size = 4.0米
    # 一个事件位置(10.5, 8.3)会被映射到网格坐标(2, 2)，因为:
    # grid_x = int(10.5 / 4.0) = 2
    # grid_y = int(8.3 / 4.0) = 2
    #
    # +----+----+----+----+
    # |    |    |    |    |  每个格子大小为 4.0 x 4.0
    # +----+----+----+----+
    # |    | *  |    |    |  * 表示事件位置(10.5, 8.3)
    # +----+----+----+----+
    # |    |    |    |    |
    # +----+----+----+----+

    for i, event in enumerate(events):
        pos = event['position']
        # 将实际坐标转换为网格坐标
        grid_x = int(pos[0] / cell_size)
        grid_y = int(pos[1] / cell_size)
        grid_key = (grid_x, grid_y)
        
        # 将事件添加到对应的网格中
        if grid_key not in grid:
            grid[grid_key] = []
        grid[grid_key].append((i, event))
        
        # 打印调试信息
        print(f"事件位置 ({pos[0]:.1f}, {pos[1]:.1f}) 被映射到网格 {grid_key}")
    
    # 打印网格统计信息
    print(f"\n网格统计:")
    print(f"总网格数: {len(grid)}")
    print(f"网格大小: {cell_size:.1f}米 x {cell_size:.1f}米")
    for grid_key, events in grid.items():
        print(f"网格{grid_key}包含 {len(events)} 个事件")
    
    return grid

def compare_replan_positions(events1, events2, threshold=2.0):
    """使用网格优化的重规划位置比较"""
    if len(events1) == 0 or len(events2) == 0:
        return {
            'similar_replans': [],
            'unique_replans1': events1,
            'unique_replans2': events2,
            'stats': {'total_replans1': len(events1), 'total_replans2': len(events2)}
        }
    
    # 创建网格索引
    print("\n为轨迹2创建空间索引:")
    grid2 = create_spatial_index(events2, threshold)
    
    similar_replans = []
    unique_replans1 = []
    matched1 = set()
    matched2 = set()
    
    # 对轨迹1中的每个重规划事件
    for i, event1 in enumerate(events1):
        pos1 = event1['position']
        cell_size = threshold * 2
        grid_x = int(pos1[0] / cell_size)
        grid_y = int(pos1[1] / cell_size)
        
        found_match = False
        
        # 检查相邻的9个网格
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor_key = (grid_x + dx, grid_y + dy)
                if neighbor_key not in grid2:
                    continue
                
                # 检查该网格中的所有事件
                for j, event2 in grid2[neighbor_key]:
                    if j in matched2:
                        continue
                    
                    pos2 = event2['position']
                    distance = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                    
                    if distance < threshold:
                        similar_replans.append({
                            'position1': pos1,
                            'position2': pos2,
                            'distance': distance,
                            'reason1': event1['reason'],
                            'reason2': event2['reason']
                        })
                        matched1.add(i)
                        matched2.add(j)
                        found_match = True
                        break
                
                if found_match:
                    break
            if found_match:
                break
        
        if not found_match:
            unique_replans1.append(event1)
    
    # 收集未匹配的轨迹2事件
    unique_replans2 = [event for i, event in enumerate(events2) if i not in matched2]
    
    return {
        'similar_replans': similar_replans,
        'unique_replans1': unique_replans1,
        'unique_replans2': unique_replans2,
        'stats': {
            'total_replans1': len(events1),
            'total_replans2': len(events2),
            'similar_count': len(similar_replans),
            'unique_count1': len(unique_replans1),
            'unique_count2': len(unique_replans2)
        }
    }
    
    


def analyze_planning_behavior(planning1_df, planning2_df, loc1_df, loc2_df, aligned_pairs):
    """综合分析planning行为，包括决策和重规划
    Args:
        planning1_df, planning2_df: 规划数据
        loc1_df, loc2_df: 定位数据
        aligned_pairs: 对齐的点对
    Returns:
        dict: 包含决策和重规划的分析结果
    """
    # 1. 使用原有的决策分析函数
    decision_analysis = analyze_planning_decisions(
        planning1_df, planning2_df, aligned_pairs, loc1_df, loc2_df
    )
    
    if decision_analysis is None:
        return None
        
    # 2. 提取时间段内的重规划事件
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
    start_idx1, start_idx2 = aligned_pairs[0]
    end_idx1, end_idx2 = aligned_pairs[-1]
    
    start_time1 = loc1_df.iloc[start_idx1]['timestamp']
    end_time1 = loc1_df.iloc[end_idx1]['timestamp']
    start_time2 = loc2_df.iloc[start_idx2]['timestamp']
    end_time2 = loc2_df.iloc[end_idx2]['timestamp']
    
    # 提取重规划事件
    events1, replan_count1 = extract_replan_events(planning1_df, start_time1, end_time1)
    events2, replan_count2 = extract_replan_events(planning2_df, start_time2, end_time2)
    
    # 3. 分析重规划位置
    replan_comparison = compare_replan_positions(events1, events2, threshold=2.0)
    
    # 4. 整合所有分析结果
    results = {
        'decision_analysis': decision_analysis,
        'replanning': {
            'trajectory1': {
                'total_count': replan_count1,
                'events': events1,
                'reasons': planning1_df[planning1_df['is_replan']]['replan_reason'].value_counts().to_dict()
            },
            'trajectory2': {
                'total_count': replan_count2,
                'events': events2,
                'reasons': planning2_df[planning2_df['is_replan']]['replan_reason'].value_counts().to_dict()
            },
            'position_comparison': replan_comparison
        }
    }
    
    # 5. 打印综合分析结果
    print("\n=== Planning行为综合分析 ===")
    
    print("\n1. 决策统计:")
    print(f"轨迹1决策类型: {decision_analysis['trajectory1_decisions']['Decision'].tolist()}")
    print(f"轨迹2决策类型: {decision_analysis['trajectory2_decisions']['Decision'].tolist()}")
    print(f"发现 {len(decision_analysis['similar_position_different_decisions'])} 处决策差异")
    
    print("\n2. 重规划分析:")
    print(f"轨迹1重规划次数: {replan_count1}")
    print(f"轨迹2重规划次数: {replan_count2}")
    print("\n重规划原因分布:")
    print("轨迹1:", results['replanning']['trajectory1']['reasons'])
    print("轨迹2:", results['replanning']['trajectory2']['reasons'])
    
    print("\n3. 重规划位置比较:")
    print(f"相似位置重规划: {replan_comparison['stats']['similar_count']} 处")
    print(f"轨迹1独有重规划: {replan_comparison['stats']['unique_count1']} 处")
    print(f"轨迹2独有重规划: {replan_comparison['stats']['unique_count2']} 处")
    
    if len(replan_comparison['similar_replans']) > 0:
        print("\n相似位置重规划详情:")
        for i, replan in enumerate(replan_comparison['similar_replans']):
            print(f"\n位置对 {i+1}:")
            print(f"距离: {replan['distance']:.2f}米")
            print(f"轨迹1原因: {replan['reason1']}")
            print(f"轨迹2原因: {replan['reason2']}")
    
    return results
    
    
    
    






    

######################################################################################
################ 6. compare the behavior at the aligned position  ####################
######################################################################################

    



def analyze_aligned_states(planning1_df, planning2_df, loc1_df, loc2_df, aligned_pairs):
    """分析对齐位置的状态
    Args:
        planning1_df, planning2_df: 规划数据
        loc1_df, loc2_df: 定位数据
        aligned_pairs: 对齐的点对
    Returns:
        pd.DataFrame: 对齐状态的分析结果
    """
    # 计算planning数据的采样间隔
    planning1_interval = planning1_df['timestamp'].diff().median()
    planning2_interval = planning2_df['timestamp'].diff().median()
    
    # 设置对齐误差阈值为采样间隔的一半
    time_threshold = min(planning1_interval, planning2_interval) / 2
    
    print(f"\n数据采样信息:")
    print(f"Planning1采样间隔: {planning1_interval*1000:.1f}ms")
    print(f"Planning2采样间隔: {planning2_interval*1000:.1f}ms")
    print(f"对齐误差阈值: {time_threshold*1000:.1f}ms")
    
    aligned_states = []
    skipped_count = 0
    invalid_data_count = 0
    
    for pair in aligned_pairs:
        idx1, idx2 = pair
        # 获取定位数据中的时间戳
        timestamp1 = loc1_df.iloc[idx1]['timestamp']
        timestamp2 = loc2_df.iloc[idx2]['timestamp']
        
        # 在planning数据中找到最接近的时间点
        p1_idx = planning1_df['timestamp'].sub(timestamp1).abs().idxmin()
        p2_idx = planning2_df['timestamp'].sub(timestamp2).abs().idxmin()
        
        p1 = planning1_df.iloc[p1_idx]
        p2 = planning2_df.iloc[p2_idx]
        
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
        print("警告: 没有找到有效的对齐状态")
        return None
    
    df = pd.DataFrame(aligned_states)
    
    # 打印统计信息
    print("\n=== 状态对齐分析 ===")
    print(f"总对齐点数: {len(aligned_pairs)}")
    print(f"有效对齐点数: {len(df)}")
    print(f"时间对齐误差过大点数: {skipped_count}")
    print(f"无效数据点数: {invalid_data_count}")
    
    print("\n状态差异统计:")
    stats = df[[
        'position_diff', 'speed_diff', 'heading_diff',
        'throttle_diff', 'brake_diff', 'steering_diff'
    ]].describe()
    print(stats)
    
    # 标记显著差异
    significant_diffs = df[
        (df['speed_diff'] > 1.0) |  # 速度差超过1m/s
        (df['heading_diff'] > 0.1) |  # 朝向差超过0.1弧度
        (df['position_diff'] > 0.5)    # 位置差超过0.5米
    ]
    
    if len(significant_diffs) > 0:
        print(f"\n发现 {len(significant_diffs)} 处显著状态差异:")
        for _, row in significant_diffs.iterrows():
            print(f"\n时间点: {row['timestamp1']:.3f}/{row['timestamp2']:.3f}")
            print(f"位置差: {row['position_diff']:.2f}米")
            print(f"速度差: {row['speed_diff']:.2f}m/s")
            print(f"朝向差: {row['heading_diff']:.3f}rad")
            
            # 输出控制量差异
            if row['throttle_diff'] > 5 or row['brake_diff'] > 5 or row['steering_diff'] > 5:
                print("控制量差异:")
                if row['throttle_diff'] > 5:
                    print(f"- 油门差: {row['throttle_diff']:.1f}%")
                if row['brake_diff'] > 5:
                    print(f"- 刹车差: {row['brake_diff']:.1f}%")
                if row['steering_diff'] > 5:
                    print(f"- 转向差: {row['steering_diff']:.1f}%")
    
    return {
        'aligned_states': df,
        'stats': stats,
        'significant_differences': significant_diffs,
        'metadata': {
            'total_pairs': len(aligned_pairs),
            'valid_pairs': len(df),
            'skipped_count': skipped_count,
            'invalid_count': invalid_data_count,
            'planning1_interval': planning1_interval,
            'planning2_interval': planning2_interval,
            'time_threshold': time_threshold
        }
    }





#TODO: modify it
results = analyze_aligned_states(
    planning1_df,
    planning2_df,
    loc1_df,
    loc2_df,
    aligned_pairs
)

if results:
    # 获取状态数据
    states_df = results['aligned_states']
    
    # 查看统计信息
    print(results['stats'])
    
    # 查看显著差异
    significant_diffs = results['significant_differences']
    
    # 获取元数据
    metadata = results['metadata']














#together we have 6 comparing aspects; we need to modify them and combine them together
    

"""
主函数，用于执行完整的分析流程
"""
# 1. 加载数据
dir="/home/w/workspace/carla_apollo/apollo/modules/carla_bridge/multi_vehicle_fuzz/apollo_log_csv/"
loc1_df = pd.read_csv(dir+'ori/'+"localization_pose.csv")
loc2_df = pd.read_csv(dir+'replay/'+"localization_pose.csv")

planning1_df = pd.read_csv(dir+'ori/'+"planning.csv")
planning2_df = pd.read_csv(dir+'replay/'+"planning.csv")


chassis1_df = pd.read_csv(dir+'ori/'+"chassis.csv")
chassis2_df = pd.read_csv(dir+'replay/'+"chassis.csv")

# 1. 首先分析轨迹长度并确定基准轨迹
shorter_df, longer_df, length_diff = analyze_trajectory_lengths(loc1_df, loc2_df)

# 2. 可视化原始轨迹（包含长度差异信息）
visualize_raw_trajectories(loc1_df, loc2_df, length_diff)


#TODO: in the future, this should be an API
# 3. 基于较短轨迹选取关键点
key_positions = get_key_positions_by_distance(
    loc_df=shorter_df,  # 使用较短的轨迹作为基准
    sample_distance=20.0  # 每20米选一个点
)

# 3. 创建对齐器
aligner = TrajectoryAligner(
    pos_threshold=0.5,  # 位置匹配阈值
    time_threshold=0.1,  # 时间匹配阈值
    key_positions=key_positions
)

# 4. 对齐轨迹
aligned_pairs = aligner.align_trajectories(loc1_df, loc2_df)






#TODO: 1. maybe write the functions together as a class 2. change the print funcion into a log function. generate a analuze log as the final analyze results












