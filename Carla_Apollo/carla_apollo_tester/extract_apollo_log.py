import os
import pandas as pd
from pathlib import Path
from cyber_record.record import Record


#TODO: remove the useless keywords
class ApolloLogExtractor:
    def __init__(self, record_dir, output_dir):
        self.record_dir = Path(record_dir)
        self.output_dir = Path(output_dir)
        self.record_files = sorted(self.record_dir.glob("apollo.record.*"))
        os.makedirs(self.output_dir, exist_ok=True)

        # 临时存储每个 topic 的 DataFrame
        self.temp_dataframes = {
            "control": pd.DataFrame(),
            "localization_pose": pd.DataFrame(),
            "chassis": pd.DataFrame(),
            "planning": pd.DataFrame(),
        }

    def initialize_dataframe(self, columns):
        # 显式指定 dtype 为 object 或其他合适的类型
        return pd.DataFrame(columns=columns, dtype='object')

    def process_topic(self, record, topic, topic_name, columns, extract_data_fn):
        print(f"Processing topic: {topic} from record")
        if self.temp_dataframes[topic_name].empty:
            self.temp_dataframes[topic_name] = self.initialize_dataframe(columns)
        for _, message, t in record.read_messages(topics=[topic]):
            new_data = extract_data_fn(message, t)
            self.temp_dataframes[topic_name] = pd.concat([
                self.temp_dataframes[topic_name], 
                pd.DataFrame([new_data])
            ], ignore_index=True)

    def extract_control(self, record):
        columns = [
            "timestamp", "t", "sequence_number", "throttle", "brake", "steering_rate", "steering_target", "acceleration",
            "lon_station_reference", "lon_current_station", "lon_station_error", "lon_station_error_limited",
            "lon_preview_station_error", "lon_speed_reference", "lon_speed_error", "lon_speed_controller_input_limited",
            "lon_speed_lookup", "lon_current_speed", "lon_speed_offset", "lon_preview_speed_reference",
            "lon_preview_speed_error", "lon_preview_acceleration_reference", "lon_acceleration_cmd_closeloop",
            "lon_acceleration_cmd", "lon_acceleration_lookup", "lon_acceleration_reference", "lon_current_acceleration",
            "lon_acceleration_error", "lon_throttle_cmd", "lon_brake_cmd", "lon_is_full_stop",
            "lon_slope_offset_compensation", "lon_path_remain", "lon_pid_saturation_status",
            # "lon_current_matched_point-x", "lon_current_matched_point-y", "lon_current_reference_point-x",
            # "lon_current_reference_point-y", "lon_preview_reference_point-x", "lon_preview_reference_point-y",
            "lateral_error", "lat_ref_heading", "lat_heading", "lat_heading_error", "lat_heading_error_rate",
            "lat_heading_error_feedback", "lat_current_target_point_x", "lat_current_target_point_y",
            "latency_ms", "total_time_exceeded"
        ]

        def extract_data(message, t):
            return {
                "timestamp": message.header.timestamp_sec,
                "t": t,
                "sequence_number": message.header.sequence_num,
                "throttle": message.throttle,
                "brake": message.brake,
                "steering_rate": message.steering_rate,
                "steering_target": message.steering_target,
                "acceleration": message.acceleration,
                "lon_station_reference": message.debug.simple_lon_debug.station_reference,
                "lon_current_station": message.debug.simple_lon_debug.current_station,
                "lon_station_error": message.debug.simple_lon_debug.station_error,
                "lon_station_error_limited": message.debug.simple_lon_debug.station_error_limited,
                "lon_preview_station_error": message.debug.simple_lon_debug.preview_station_error,
                "lon_speed_reference": message.debug.simple_lon_debug.speed_reference,
                "lon_speed_error": message.debug.simple_lon_debug.speed_error,
                "lon_speed_controller_input_limited": message.debug.simple_lon_debug.speed_controller_input_limited,
                "lon_speed_lookup": message.debug.simple_lon_debug.speed_lookup,
                "lon_current_speed": message.debug.simple_lon_debug.current_speed,
                "lon_speed_offset": message.debug.simple_lon_debug.speed_offset,
                "lon_preview_speed_reference": message.debug.simple_lon_debug.preview_speed_reference,
                "lon_preview_speed_error": message.debug.simple_lon_debug.preview_speed_error,
                "lon_preview_acceleration_reference": message.debug.simple_lon_debug.preview_acceleration_reference,
                "lon_acceleration_cmd_closeloop": message.debug.simple_lon_debug.acceleration_cmd_closeloop,
                "lon_acceleration_cmd": message.debug.simple_lon_debug.acceleration_cmd,
                "lon_acceleration_lookup": message.debug.simple_lon_debug.acceleration_lookup,
                "lon_acceleration_reference": message.debug.simple_lon_debug.acceleration_reference,
                "lon_current_acceleration": message.debug.simple_lon_debug.current_acceleration,
                "lon_acceleration_error": message.debug.simple_lon_debug.acceleration_error,
                "lon_throttle_cmd": message.debug.simple_lon_debug.throttle_cmd,
                "lon_brake_cmd": message.debug.simple_lon_debug.brake_cmd,
                "lon_is_full_stop": message.debug.simple_lon_debug.is_full_stop,
                "lon_slope_offset_compensation": message.debug.simple_lon_debug.slope_offset_compensation,
                "lon_path_remain": message.debug.simple_lon_debug.path_remain,
                "lon_pid_saturation_status": message.debug.simple_lon_debug.pid_saturation_status,
                # "lon_current_matched_point-x": message.debug.simple_lon_debug.current_matched_point.path_point.x,
                # "lon_current_matched_point-y": message.debug.simple_lon_debug.current_matched_point.path_point.y,
                # "lon_current_reference_point-x": message.debug.simple_lon_debug.current_reference_point.x,
                # "lon_current_reference_point-y": message.debug.simple_lon_debug.current_reference_point.y,
                # "lon_preview_reference_point-x": message.debug.simple_lon_debug.preview_reference_point.x,
                # "lon_preview_reference_point-y": message.debug.simple_lon_debug.preview_reference_point.y,
                "lateral_error": message.debug.simple_lat_debug.lateral_error,
                "lat_ref_heading": message.debug.simple_lat_debug.ref_heading,
                "lat_heading": message.debug.simple_lat_debug.heading,
                "lat_heading_error": message.debug.simple_lat_debug.heading_error,
                "lat_heading_error_rate": message.debug.simple_lat_debug.heading_error_rate,
                "lat_heading_error_feedback": message.debug.simple_lat_debug.heading_error_feedback,
                "lat_current_target_point_x": message.debug.simple_lat_debug.current_target_point.path_point.x,
                "lat_current_target_point_y": message.debug.simple_lat_debug.current_target_point.path_point.y,
                "latency_ms": message.latency_stats.total_time_ms,
                "total_time_exceeded": message.latency_stats.total_time_exceeded
            }

        self.process_topic(record, "/apollo/control", "control", columns, extract_data)

    def extract_localization_pose(self, record):
        columns = [
            "timestamp", "t", "sequence_number", "localization_module_name",
            "localization_pose_x", "localization_pose_y", "localization_pose_z",
            "localization_orientation_qx", "localization_orientation_qy", "localization_orientation_qz",
            "localization_orientation_qw", "localization_linear_accel_x", "localization_linear_accel_y",
            "localization_linear_accel_z", "localization_angular_velocity_x", "localization_angular_velocity_y",
            "localization_angular_velocity_z", "localization_heading", "localization_linear_acceleration_vrf_x",
            "localization_linear_acceleration_vrf_y", "localization_linear_acceleration_vrf_z",
            "localization_angular_velocity_vrf_x", "localization_angular_velocity_vrf_y",
            "localization_angular_velocity_vrf_z", "localization_euler_angles_x", "localization_euler_angles_y",
            "localization_euler_angles_z", "localization_measurement_time"
        ]

        def extract_data(message, t):
            return {
                "timestamp": message.header.timestamp_sec,
                "t": t,
                "sequence_number": message.header.sequence_num,
                "localization_module_name": message.header.module_name,
                "localization_pose_x": message.pose.position.x,
                "localization_pose_y": message.pose.position.y,
                "localization_pose_z": message.pose.position.z,
                "localization_orientation_qx": message.pose.orientation.qx,
                "localization_orientation_qy": message.pose.orientation.qy,
                "localization_orientation_qz": message.pose.orientation.qz,
                "localization_orientation_qw": message.pose.orientation.qw,
                "localization_linear_accel_x": message.pose.linear_acceleration.x,
                "localization_linear_accel_y": message.pose.linear_acceleration.y,
                "localization_linear_accel_z": message.pose.linear_acceleration.z,
                

                "localization_angular_velocity_x": message.pose.angular_velocity.x,
                "localization_angular_velocity_y": message.pose.angular_velocity.y,
                "localization_angular_velocity_z": message.pose.angular_velocity.z,
                "localization_heading": message.pose.heading,
                "localization_linear_acceleration_vrf_x": message.pose.linear_acceleration_vrf.x,
                "localization_linear_acceleration_vrf_y": message.pose.linear_acceleration_vrf.y,
                "localization_linear_acceleration_vrf_z": message.pose.linear_acceleration_vrf.z,
                "localization_angular_velocity_vrf_x": message.pose.angular_velocity_vrf.x,
                "localization_angular_velocity_vrf_y": message.pose.angular_velocity_vrf.y,
                "localization_angular_velocity_vrf_z": message.pose.angular_velocity_vrf.z,
                "localization_euler_angles_x": message.pose.euler_angles.x,
                "localization_euler_angles_y": message.pose.euler_angles.y,
                "localization_euler_angles_z": message.pose.euler_angles.z,
                "localization_measurement_time": message.measurement_time
            }

        self.process_topic(record, "/apollo/localization/pose", "localization_pose", columns, extract_data)

    def extract_chassis(self, record):
        columns = [
            "timestamp", "t", "sequence_number", "chassis_engine_started", "chassis_speed_mps",
            "chassis_throttle_percentage", "chassis_brake_percentage", "chassis_steering_percentage",
            "chassis_parking_brake", "chassis_driving_mode", "chassis_header_frame_id"
        ]

        def extract_data(message, t):
            return {
                "timestamp": message.header.timestamp_sec,
                "t": t,
                "sequence_number": message.header.sequence_num,
                "chassis_engine_started": message.engine_started,
                "chassis_speed_mps": message.speed_mps,
                "chassis_throttle_percentage": message.throttle_percentage,
                "chassis_brake_percentage": message.brake_percentage,
                "chassis_steering_percentage": message.steering_percentage,
                "chassis_parking_brake": message.parking_brake,
                "chassis_driving_mode": message.driving_mode,
                "chassis_header_frame_id": message.header.frame_id,
            }

        self.process_topic(record, "/apollo/canbus/chassis", "chassis", columns, extract_data)

    def extract_planning(self, record):
        columns = [
            "timestamp", "t", "sequence_number", 
            "adc_position_timestamp", "adc_position_pose_x", "adc_position_pose_y", "adc_position_pose_z",
            "adc_position_orientation_qx", "adc_position_orientation_qy", "adc_position_orientation_qz", "adc_position_orientation_qw",
            "adc_position_linear_accel_x", "adc_position_linear_accel_y", "adc_position_linear_accel_z",
            "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
            "heading",
            "linear_acceleration_vrf_x", "linear_acceleration_vrf_y", "linear_acceleration_vrf_z",
            "angular_velocity_vrf_x", "angular_velocity_vrf_y", "angular_velocity_vrf_z",
            "euler_angles_x", "euler_angles_y", "euler_angles_z",
            "chassis_engine_started", "chassis_speed_mps", "chassis_throttle_percentage", 
            "chassis_brake_percentage", "chassis_steering_percentage", "chassis_parking_brake",
            "chassis_driving_mode", "chassis_header_timestamp_sec", "chassis_header_frame_id",
            "init_point_path_point_x", "init_point_path_point_y", "init_point_path_point_z",
            "init_point_path_point_theta", "init_point_path_point_kappa", "init_point_path_point_s",
            "init_point_v", "init_point_a", "init_point_relative_time",
            "front_clear_distance", "is_replan", "decision", "trajectory_type", "replan_reason"
        ]

        def extract_data(message, t):
            return {
                "timestamp": message.header.timestamp_sec,
                "t": t,
                "sequence_number": message.header.sequence_num,
                "adc_position_timestamp": message.debug.planning_data.adc_position.header.timestamp_sec,
                "adc_position_pose_x": message.debug.planning_data.adc_position.pose.position.x,
                "adc_position_pose_y": message.debug.planning_data.adc_position.pose.position.y,
                "adc_position_pose_z": message.debug.planning_data.adc_position.pose.position.z,
                "adc_position_orientation_qx": message.debug.planning_data.adc_position.pose.orientation.qx,
                "adc_position_orientation_qy": message.debug.planning_data.adc_position.pose.orientation.qy,
                "adc_position_orientation_qz": message.debug.planning_data.adc_position.pose.orientation.qz,
                "adc_position_orientation_qw": message.debug.planning_data.adc_position.pose.orientation.qw,
                "adc_position_linear_accel_x": message.debug.planning_data.adc_position.pose.linear_acceleration.x,
                
                "adc_position_linear_accel_y": message.debug.planning_data.adc_position.pose.linear_acceleration.y,
                "adc_position_linear_accel_z": message.debug.planning_data.adc_position.pose.linear_acceleration.z,
                "angular_velocity_x": message.debug.planning_data.adc_position.pose.angular_velocity.x,
                "angular_velocity_y": message.debug.planning_data.adc_position.pose.angular_velocity.y,
                "angular_velocity_z": message.debug.planning_data.adc_position.pose.angular_velocity.z,
                "heading": message.debug.planning_data.adc_position.pose.heading,
                "linear_acceleration_vrf_x": message.debug.planning_data.adc_position.pose.linear_acceleration_vrf.x,
                "linear_acceleration_vrf_y": message.debug.planning_data.adc_position.pose.linear_acceleration_vrf.y,
                "linear_acceleration_vrf_z": message.debug.planning_data.adc_position.pose.linear_acceleration_vrf.z,
                "angular_velocity_vrf_x": message.debug.planning_data.adc_position.pose.angular_velocity_vrf.x,
                "angular_velocity_vrf_y": message.debug.planning_data.adc_position.pose.angular_velocity_vrf.y,
                "angular_velocity_vrf_z": message.debug.planning_data.adc_position.pose.angular_velocity_vrf.z,
                "euler_angles_x": message.debug.planning_data.adc_position.pose.euler_angles.x,
                "euler_angles_y": message.debug.planning_data.adc_position.pose.euler_angles.y,
                "euler_angles_z": message.debug.planning_data.adc_position.pose.euler_angles.z,
                
                
                
                "chassis_engine_started": message.debug.planning_data.chassis.engine_started,
                "chassis_speed_mps": message.debug.planning_data.chassis.speed_mps,
                "chassis_throttle_percentage": message.debug.planning_data.chassis.throttle_percentage,
                "chassis_brake_percentage": message.debug.planning_data.chassis.brake_percentage,
                "chassis_steering_percentage": message.debug.planning_data.chassis.steering_percentage,
                "chassis_parking_brake": message.debug.planning_data.chassis.parking_brake,
                "chassis_driving_mode": message.debug.planning_data.chassis.driving_mode,
                "chassis_header_timestamp_sec": message.debug.planning_data.chassis.header.timestamp_sec,
                "chassis_header_frame_id": message.debug.planning_data.chassis.header.frame_id,
                "init_point_path_point_x": message.debug.planning_data.init_point.path_point.x,
                "init_point_path_point_y": message.debug.planning_data.init_point.path_point.y,
                "init_point_path_point_z": message.debug.planning_data.init_point.path_point.z,
                "init_point_path_point_theta": message.debug.planning_data.init_point.path_point.theta,
                "init_point_path_point_kappa": message.debug.planning_data.init_point.path_point.kappa,
                "init_point_path_point_s": message.debug.planning_data.init_point.path_point.s,
                "init_point_v": message.debug.planning_data.init_point.v,
                "init_point_a": message.debug.planning_data.init_point.a,
                "init_point_relative_time": message.debug.planning_data.init_point.relative_time,
            
                
                "front_clear_distance": message.debug.planning_data.front_clear_distance,
                "is_replan": message.is_replan,
                "decision": message.decision.main_decision,
                "trajectory_type": message.trajectory_type,
                "replan_reason": message.replan_reason
                
                
            }

        self.process_topic(record, "/apollo/planning", "planning", columns, extract_data)

    def save_to_csv(self):
        for topic_name, df in self.temp_dataframes.items():
            if not df.empty:
                output_path = self.output_dir / f"{topic_name}.csv"
                df.to_csv(output_path, index=False)
                print(f"Saved {topic_name} data to {output_path}")

    def process_all_records(self):
        for record_file in self.record_files:
            print(f"Processing record file: {record_file}")
            record = Record(str(record_file))
            self.extract_control(record)
            self.extract_localization_pose(record)
            self.extract_chassis(record)
            self.extract_planning(record)
        self.save_to_csv()

# 使用示例
if __name__ == "__main__":
    record_dir = "/home/w/workspace/carla_apollo/apollo/modules/carla_bridge/multi_vehicle_fuzz/test_outputs/20241212_191334/test_20241212_191345_v10_w10-HIGH/replay_scenarios/segment_002/apollo_logs"  # 替换为实际目录
    output_dir = "/home/w/workspace/carla_apollo/apollo/modules/carla_bridge/multi_vehicle_fuzz/test_outputs/20241212_191334/test_20241212_191345_v10_w10-HIGH/replay_scenarios/segment_002/apollo_logs/extracted_csv"  # 替换为实际输出目录

    extractor = ApolloLogExtractor(record_dir, output_dir)
    extractor.process_all_records()