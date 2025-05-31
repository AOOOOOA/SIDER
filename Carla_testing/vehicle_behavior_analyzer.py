#!/usr/bin/env python
"""
Analyze vehicle behavior logs and correlate with error features
"""

import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import argparse
from datetime import datetime
from collections import defaultdict
import seaborn as sns
from tqdm import tqdm
import pickle
from scipy import stats

# Regular expressions for parsing logs
VEHICLE_STATE_PATTERN = r'\[(.*?)\] \[FRAME:(.*?)\] \[ACTOR:(.*?)\] \[(.*?)\] \[(.*?)\] VEL:\[(.*?),(.*?),(.*?)\] ACC:\[(.*?),(.*?),(.*?)\] ANG_VEL:\[(.*?),(.*?),(.*?)\] LOC:\[(.*?),(.*?),(.*?)\] ROT:\[(.*?),(.*?),(.*?)\](.*)'
TM_DECISION_PATTERN = r'\[(.*?)\] \[FRAME:(.*?)\] \[ACTOR:(.*?)\] \[TM:(.*?)\] \[DECISION:(.*?)\] \[PARAMS:(.*?),(.*?),(.*?)\](.*)'

class VehicleBehaviorAnalyzer:
    def __init__(self, log_file, error_features_file=None, cluster_data_file=None):
        """
        Initialize analyzer
        :param log_file: Vehicle behavior log file
        :param error_features_file: Error features file (optional)
        :param cluster_data_file: Cluster data file (optional)
        """
        self.log_file = log_file
        self.error_features_file = error_features_file
        self.cluster_data_file = cluster_data_file
        
        # Store parsed data
        self.vehicle_states = []
        self.tm_decisions = []
        self.error_features = None
        self.cluster_data = None
        
    def parse_log_file(self):
        """Parse log file"""
        print(f"Parsing log file: {self.log_file}")
        
        with open(self.log_file, 'r') as f:
            for line in tqdm(f, desc="Parsing logs"):
                # Try to match vehicle state logs
                vehicle_match = re.match(VEHICLE_STATE_PATTERN, line)
                if vehicle_match:
                    groups = vehicle_match.groups()
                    self.vehicle_states.append({
                        'timestamp': groups[0],
                        'frame': int(groups[1]),
                        'actor_id': int(groups[2]),
                        'module': groups[3],
                        'function': groups[4],
                        'velocity_x': float(groups[5]),
                        'velocity_y': float(groups[6]),
                        'velocity_z': float(groups[7]),
                        'acceleration_x': float(groups[8]),
                        'acceleration_y': float(groups[9]),
                        'acceleration_z': float(groups[10]),
                        'angular_velocity_x': float(groups[11]),
                        'angular_velocity_y': float(groups[12]),
                        'angular_velocity_z': float(groups[13]),
                        'location_x': float(groups[14]),
                        'location_y': float(groups[15]),
                        'location_z': float(groups[16]),
                        'rotation_pitch': float(groups[17]),
                        'rotation_yaw': float(groups[18]),
                        'rotation_roll': float(groups[19]),
                        'additional_info': groups[20].strip() if groups[20].strip() else None
                    })
                    continue
                
                # Try to match traffic manager decision logs
                tm_match = re.match(TM_DECISION_PATTERN, line)
                if tm_match:
                    groups = tm_match.groups()
                    self.tm_decisions.append({
                        'timestamp': groups[0],
                        'frame': int(groups[1]),
                        'actor_id': int(groups[2]),
                        'stage': groups[3],
                        'decision': groups[4],
                        'param1': float(groups[5]),
                        'param2': float(groups[6]),
                        'param3': float(groups[7]),
                        'additional_info': groups[8].strip() if groups[8].strip() else None
                    })
        
        print(f"Parsing completed: {len(self.vehicle_states)} vehicle state records, {len(self.tm_decisions)} decision records")
        
    def load_error_features(self):
        """Load error features data"""
        if not self.error_features_file:
            print("No error features file provided")
            return
            
        print(f"Loading error features data: {self.error_features_file}")
        
        if self.error_features_file.endswith('.json'):
            with open(self.error_features_file, 'r') as f:
                self.error_features = json.load(f)
        elif self.error_features_file.endswith('.pkl'):
            with open(self.error_features_file, 'rb') as f:
                self.error_features = pickle.load(f)
        else:
            print(f"Unsupported file format: {self.error_features_file}")
            
    def load_cluster_data(self):
        """Load cluster data"""
        if not self.cluster_data_file:
            print("No cluster data file provided")
            return
            
        print(f"Loading cluster data: {self.cluster_data_file}")
        
        if self.cluster_data_file.endswith('.json'):
            with open(self.cluster_data_file, 'r') as f:
                self.cluster_data = json.load(f)
        elif self.cluster_data_file.endswith('.pkl'):
            with open(self.cluster_data_file, 'rb') as f:
                self.cluster_data = pickle.load(f)
        else:
            print(f"Unsupported file format: {self.cluster_data_file}")
    
    def compute_vehicle_statistics(self):
        """Compute vehicle behavior statistics"""
        if not self.vehicle_states:
            print("No vehicle state data")
            return {}
            
        # Create DataFrame
        df = pd.DataFrame(self.vehicle_states)
        
        # Group by vehicle ID
        grouped = df.groupby('actor_id')
        
        statistics = {}
        for actor_id, group in grouped:
            # Calculate velocity and acceleration statistics
            velocity_magnitude = np.sqrt(
                group['velocity_x']**2 + 
                group['velocity_y']**2 + 
                group['velocity_z']**2
            )
            
            # Driving distance
            if len(group) > 1:
                locations = group[['location_x', 'location_y', 'location_z']].values
                distances = np.sqrt(
                    np.sum(np.diff(locations, axis=0)**2, axis=1)
                )
                total_distance = np.sum(distances)
            else:
                total_distance = 0
                
            # Acceleration changes
            if len(group) > 1:
                velocities = group[['velocity_x', 'velocity_y', 'velocity_z']].values
                acceleration_changes = np.diff(velocities, axis=0)
                acceleration_magnitude = np.sqrt(np.sum(acceleration_changes**2, axis=1))
            else:
                acceleration_magnitude = np.array([0])
            
            # Steering statistics
            angular_velocity_magnitude = np.sqrt(
                group['angular_velocity_x']**2 + 
                group['angular_velocity_y']**2 + 
                group['angular_velocity_z']**2
            )
            
            # Store statistics
            statistics[actor_id] = {
                'mean_velocity': velocity_magnitude.mean(),
                'max_velocity': velocity_magnitude.max(),
                'velocity_std': velocity_magnitude.std(),
                'mean_acceleration': acceleration_magnitude.mean() if len(acceleration_magnitude) > 0 else 0,
                'max_acceleration': acceleration_magnitude.max() if len(acceleration_magnitude) > 0 else 0,
                'mean_angular_velocity': angular_velocity_magnitude.mean(),
                'max_angular_velocity': angular_velocity_magnitude.max(),
                'total_distance': total_distance,
                'record_count': len(group),
                'start_frame': group['frame'].min(),
                'end_frame': group['frame'].max()
            }
            
        return statistics
    
    def compute_tm_decision_statistics(self):
        """Compute traffic manager decision statistics"""
        if not self.tm_decisions:
            print("No traffic manager decision data")
            return {}
            
        # Create DataFrame
        df = pd.DataFrame(self.tm_decisions)
        
        # Group by vehicle ID
        grouped = df.groupby('actor_id')
        
        statistics = {}
        for actor_id, group in grouped:
            # Calculate each type of decision count
            decision_counts = group['decision'].value_counts().to_dict()
            
            # Calculate decision count for each stage
            stage_counts = group['stage'].value_counts().to_dict()
            
            # Store statistics
            statistics[actor_id] = {
                'decision_counts': decision_counts,
                'stage_counts': stage_counts,
                'total_decisions': len(group),
                'start_frame': group['frame'].min(),
                'end_frame': group['frame'].max()
            }
            
        return statistics
        
    def analyze_trajectory(self):
        """Analyze vehicle trajectory"""
        if not self.vehicle_states:
            print("No vehicle state data")
            return
            
        # Create output directory
        os.makedirs('trajectory_analysis', exist_ok=True)
        
        # Create DataFrame
        df = pd.DataFrame(self.vehicle_states)
        
        # Group by vehicle ID
        actor_ids = df['actor_id'].unique()
        
        for actor_id in actor_ids:
            actor_data = df[df['actor_id'] == actor_id]
            
            # Draw trajectory plot
            plt.figure(figsize=(12, 10))
            
            plt.subplot(2, 2, 1)
            plt.plot(actor_data['location_x'], actor_data['location_y'])
            plt.scatter(actor_data['location_x'].iloc[0], actor_data['location_y'].iloc[0], 
                       color='green', s=100, label='Start')
            plt.scatter(actor_data['location_x'].iloc[-1], actor_data['location_y'].iloc[-1], 
                       color='red', s=100, label='End')
            plt.title(f'Actor {actor_id} Trajectory')
            plt.xlabel('X Position')
            plt.ylabel('Y Position')
            plt.legend()
            plt.grid(True)
            
            # Draw velocity over time plot
            plt.subplot(2, 2, 2)
            velocity_magnitude = np.sqrt(
                actor_data['velocity_x']**2 + 
                actor_data['velocity_y']**2 + 
                actor_data['velocity_z']**2
            )
            plt.plot(actor_data['frame'], velocity_magnitude)
            plt.title(f'Actor {actor_id} Velocity')
            plt.xlabel('Frame')
            plt.ylabel('Velocity Magnitude')
            plt.grid(True)
            
            # Draw angular velocity over time plot
            plt.subplot(2, 2, 3)
            angular_velocity_magnitude = np.sqrt(
                actor_data['angular_velocity_x']**2 + 
                actor_data['angular_velocity_y']**2 + 
                actor_data['angular_velocity_z']**2
            )
            plt.plot(actor_data['frame'], angular_velocity_magnitude)
            plt.title(f'Actor {actor_id} Angular Velocity')
            plt.xlabel('Frame')
            plt.ylabel('Angular Velocity Magnitude')
            plt.grid(True)
            
            # Draw heading over time plot
            plt.subplot(2, 2, 4)
            plt.plot(actor_data['frame'], actor_data['rotation_yaw'])
            plt.title(f'Actor {actor_id} Heading (Yaw)')
            plt.xlabel('Frame')
            plt.ylabel('Yaw Angle')
            plt.grid(True)
            
            plt.tight_layout()
            plt.savefig(f'trajectory_analysis/actor_{actor_id}_trajectory.png')
            plt.close()
            
            print(f"Generated vehicle {actor_id} trajectory analysis plot")
            
    def analyze_tm_decisions(self):
        """Analyze traffic manager decisions"""
        if not self.tm_decisions:
            print("No traffic manager decision data")
            return
            
        # Create output directory
        os.makedirs('tm_decision_analysis', exist_ok=True)
        
        # Create DataFrame
        df = pd.DataFrame(self.tm_decisions)
        
        # Group by vehicle ID
        actor_ids = df['actor_id'].unique()
        
        for actor_id in actor_ids:
            actor_data = df[df['actor_id'] == actor_id]
            
            # Group by decision type
            decision_counts = actor_data['decision'].value_counts()
            
            # Group by stage
            stage_counts = actor_data['stage'].value_counts()
            
            # Create chart
            plt.figure(figsize=(15, 10))
            
            # Draw decision type distribution
            plt.subplot(2, 1, 1)
            decision_counts.plot(kind='bar')
            plt.title(f'Actor {actor_id} Decision Types')
            plt.xlabel('Decision Type')
            plt.ylabel('Count')
            plt.xticks(rotation=45)
            plt.grid(True, axis='y')
            
            # Draw stage distribution
            plt.subplot(2, 1, 2)
            stage_counts.plot(kind='bar')
            plt.title(f'Actor {actor_id} Stage Distribution')
            plt.xlabel('Stage')
            plt.ylabel('Count')
            plt.xticks(rotation=45)
            plt.grid(True, axis='y')
            
            plt.tight_layout()
            plt.savefig(f'tm_decision_analysis/actor_{actor_id}_decisions.png')
            plt.close()
            
            print(f"Generated vehicle {actor_id} decision analysis plot")
    
    def correlate_with_error_features(self):
        """Correlate vehicle behavior with error features"""
        if not self.vehicle_states or not self.error_features:
            print("Missing vehicle state data or error features data")
            return
            
        # Calculate vehicle statistics
        vehicle_stats = self.compute_vehicle_statistics()
        tm_stats = self.compute_tm_decision_statistics()
        
        # Create correlation analysis result directory
        os.makedirs('error_correlation', exist_ok=True)
        
        # Analyze correlation between different error features and vehicle behavior
        # (This needs to be adjusted based on the actual structure of error_features)
        if isinstance(self.error_features, dict) and 'detailed_errors' in self.error_features:
            detailed_errors = self.error_features['detailed_errors']
            
            # Extract key error features
            error_data = []
            for frame, errors in detailed_errors.items():
                for actor_id, actor_errors in errors.items():
                    if isinstance(actor_errors, dict):
                        error_data.append({
                            'frame': int(frame),
                            'actor_id': int(actor_id),
                            **actor_errors
                        })
            
            error_df = pd.DataFrame(error_data)
            
            # Merge vehicle statistics with error features
            merged_data = []
            for actor_id, stats in vehicle_stats.items():
                actor_errors = error_df[error_df['actor_id'] == actor_id]
                if not actor_errors.empty:
                    # Calculate average error
                    mean_errors = actor_errors.mean(numeric_only=True).to_dict()
                    merged_data.append({
                        'actor_id': actor_id,
                        **stats,
                        **mean_errors
                    })
            
            if merged_data:
                merged_df = pd.DataFrame(merged_data)
                
                # Calculate correlation
                numeric_columns = merged_df.select_dtypes(include=[np.number]).columns
                correlation = merged_df[numeric_columns].corr()
                
                # Draw correlation heatmap
                plt.figure(figsize=(15, 12))
                sns.heatmap(correlation, annot=True, cmap='coolwarm', vmin=-1, vmax=1)
                plt.title('Correlation between Vehicle Behavior and Error Features')
                plt.tight_layout()
                plt.savefig('error_correlation/behavior_error_correlation.png')
                plt.close()
                
                # Save correlation data
                correlation.to_csv('error_correlation/behavior_error_correlation.csv')
                
                print("Generated behavior-error feature correlation analysis")
    
    def correlate_with_clusters(self):
        """Correlate vehicle behavior with clusters"""
        if not self.vehicle_states or not self.cluster_data:
            print("Missing vehicle state data or cluster data")
            return
            
        # Create correlation analysis result directory
        os.makedirs('cluster_correlation', exist_ok=True)
        
        # Analyze correlation between different clusters and vehicle behavior
        # (This needs to be adjusted based on the actual structure of cluster_data)
        if isinstance(self.cluster_data, dict) and 'clusters' in self.cluster_data:
            clusters = self.cluster_data['clusters']
            
            # Extract cluster information
            cluster_data = []
            for cluster_id, cluster_info in clusters.items():
                if 'points' in cluster_info:
                    for point_idx in cluster_info['points']:
                        if 'point_details' in self.cluster_data and point_idx < len(self.cluster_data['point_details']):
                            point_details = self.cluster_data['point_details'][point_idx]
                            cluster_data.append({
                                'cluster_id': int(cluster_id),
                                'point_idx': point_idx,
                                **point_details
                            })
            
            if cluster_data:
                cluster_df = pd.DataFrame(cluster_data)
                
                # Calculate vehicle behavior statistics for each cluster
                cluster_behavior_stats = []
                
                for cluster_id in cluster_df['cluster_id'].unique():
                    cluster_points = cluster_df[cluster_df['cluster_id'] == cluster_id]
                    
                    # Calculate vehicle behavior statistics for the cluster
                    behavior_stats = {}
                    for _, point in cluster_points.iterrows():
                        if 'actor_id' in point and point['actor_id'] in vehicle_stats:
                            for stat_key, stat_value in vehicle_stats[point['actor_id']].items():
                                if stat_key not in behavior_stats:
                                    behavior_stats[stat_key] = []
                                behavior_stats[stat_key].append(stat_value)
                    
                    # Calculate average
                    avg_stats = {k: np.mean(v) if v else 0 for k, v in behavior_stats.items()}
                    
                    cluster_behavior_stats.append({
                        'cluster_id': cluster_id,
                        'point_count': len(cluster_points),
                        **avg_stats
                    })
                
                # Create cluster-behavior statistics DataFrame
                cluster_behavior_df = pd.DataFrame(cluster_behavior_stats)
                
                # Save cluster behavior statistics
                cluster_behavior_df.to_csv('cluster_correlation/cluster_behavior_stats.csv', index=False)
                
                # Draw cluster behavior feature comparison plot
                if len(cluster_behavior_df) > 0:
                    # Select some key indicators for comparison
                    key_metrics = ['mean_velocity', 'max_velocity', 'mean_acceleration', 
                                'max_angular_velocity', 'total_distance']
                    
                    for metric in key_metrics:
                        if metric in cluster_behavior_df.columns:
                            plt.figure(figsize=(10, 6))
                            sns.barplot(x='cluster_id', y=metric, data=cluster_behavior_df)
                            plt.title(f'Cluster Comparison - {metric}')
                            plt.xlabel('Cluster ID')
                            plt.ylabel(metric)
                            plt.grid(True, axis='y')
                            plt.tight_layout()
                            plt.savefig(f'cluster_correlation/cluster_{metric}_comparison.png')
                            plt.close()
                    
                    print("Generated cluster-behavior feature comparison plot")
                    
                # Analyze relationship between traffic manager decisions and clusters
                if tm_stats:
                    tm_cluster_stats = []
                    
                    for cluster_id in cluster_df['cluster_id'].unique():
                        cluster_points = cluster_df[cluster_df['cluster_id'] == cluster_id]
                        
                        # Collect TM decisions for all vehicles in the cluster
                        cluster_tm_decisions = defaultdict(int)
                        decision_count = 0
                        
                        for _, point in cluster_points.iterrows():
                            if 'actor_id' in point and point['actor_id'] in tm_stats:
                                actor_tm_stats = tm_stats[point['actor_id']]
                                decision_count += actor_tm_stats.get('total_decisions', 0)
                                
                                # Merge decision counts
                                for decision, count in actor_tm_stats.get('decision_counts', {}).items():
                                    cluster_tm_decisions[decision] += count
                        
                        # Calculate percentage for each decision
                        if decision_count > 0:
                            decision_percentages = {k: (v / decision_count) * 100 
                                                for k, v in cluster_tm_decisions.items()}
                            
                            tm_cluster_stats.append({
                                'cluster_id': cluster_id,
                                'total_decisions': decision_count,
                                **decision_percentages
                            })
                    
                    # Create TM decision-cluster statistics DataFrame
                    tm_cluster_df = pd.DataFrame(tm_cluster_stats)
                    
                    if not tm_cluster_df.empty:
                        # Save TM decision statistics
                        tm_cluster_df.to_csv('cluster_correlation/tm_cluster_stats.csv', index=False)
                        
                        # Draw decision type proportion plot
                        decision_columns = [col for col in tm_cluster_df.columns 
                                        if col not in ['cluster_id', 'total_decisions']]
                        
                        if decision_columns:
                            # Create decision distribution plot
                            plt.figure(figsize=(15, 8))
                            tm_cluster_df_melted = tm_cluster_df.melt(
                                id_vars=['cluster_id'], 
                                value_vars=decision_columns,
                                var_name='Decision Type', 
                                value_name='Percentage'
                            )
                            
                            # Draw stacked bar chart
                            decision_plot = sns.barplot(
                                x='cluster_id', 
                                y='Percentage', 
                                hue='Decision Type', 
                                data=tm_cluster_df_melted
                            )
                            
                            plt.title('Decision Type Distribution by Cluster')
                            plt.xlabel('Cluster ID')
                            plt.ylabel('Percentage')
                            plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
                            plt.tight_layout()
                            plt.savefig('cluster_correlation/cluster_decision_distribution.png')
                            plt.close()
                        
                        print("Generated cluster-decision type distribution plot")
                
    def run_analysis(self):
        """Run complete analysis"""
        # Parse log
        self.parse_log_file()
        
        # Load error features and cluster data
        if self.error_features_file:
            self.load_error_features()
        
        if self.cluster_data_file:
            self.load_cluster_data()
        
        # Calculate statistics
        vehicle_stats = self.compute_vehicle_statistics()
        tm_stats = self.compute_tm_decision_statistics()
        
        # Save statistics
        os.makedirs('analysis_results', exist_ok=True)
        with open('analysis_results/vehicle_statistics.json', 'w') as f:
            json.dump(vehicle_stats, f, indent=2)
            
        with open('analysis_results/tm_decision_statistics.json', 'w') as f:
            json.dump(tm_stats, f, indent=2)
        
        # Analyze trajectory
        self.analyze_trajectory()
        
        # Analyze traffic manager decisions
        self.analyze_tm_decisions()
        
        # Correlate error features
        if self.error_features:
            self.correlate_with_error_features()
        
        # Correlate clusters
        if self.cluster_data:
            self.correlate_with_clusters()
        
        print("Analysis completed, results saved in subdirectories")

def main():
    parser = argparse.ArgumentParser(description='Analyze vehicle behavior logs and correlate with error features')
    parser.add_argument('--log_file', type=str, required=True, help='Vehicle behavior log file path')
    parser.add_argument('--error_features', type=str, help='Error features file path (.json or .pkl)')
    parser.add_argument('--cluster_data', type=str, help='Cluster data file path (.json or .pkl)')
    
    args = parser.parse_args()
    
    # Create analyzer and run
    analyzer = VehicleBehaviorAnalyzer(
        args.log_file, 
        args.error_features, 
        args.cluster_data
    )
    analyzer.run_analysis()

if __name__ == "__main__":
    main() 