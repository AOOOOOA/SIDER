
#TODO: 1. compare the recorded json file
# 2. compare the extracted log file

#基本上这个文件中的对比代码没有可以直接用的。 还是1.load json file and compare 2. extract the necessary data from log file
# 3. compare the extracted data and generate the report as we did in the apollo_log_analysis_class.py

import threading
from pathlib import Path
import json
import numpy as np
from loguru import logger
from typing import Dict, List, Optional, Any
from apollo_log_analysis_class import LogAnalyzer
from extract_apollo_log import ApolloLogExtractor
import os 
import traceback
import pandas as pd
def convert_numpy_types(obj: Any) -> Any:
    """
    递归转换数据结构中的 NumPy 类型为原生 Python 类型。
    支持字典、列表和其他嵌套数据结构。
    """
    if isinstance(obj, np.bool_):  # 转换 NumPy 布尔值
        return bool(obj)
    elif isinstance(obj, (np.float64, np.float32)):  # 转换 NumPy 浮点数
        return float(obj)
    elif isinstance(obj, (np.int64, np.int32)):  # 转换 NumPy 整数
        return int(obj)
    elif isinstance(obj, dict):  # 如果是字典，递归处理每个键值对
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, list):  # 如果是列表，递归处理每个元素
        return [convert_numpy_types(i) for i in obj]
    else:  # 其他类型保持不变
        return obj
#TODO: add the analyze_segment function
#FIXME: why it inherits from the threading.thread?
class TestResultAnalyzer():
    """Test result analyzer that runs as a separate thread"""
    
    def __init__(self, ori_segment_json, replay_json, extract_apollo_logs, longrun_apollo_log_dir: Path, replay_apollo_log_dir: Path, analysis_dir: Path):
        """
        Initialize the analyzer
        Args:
            test_output_dir: Directory containing test output data
        """
        


         
        # self.original_segment_json = ori_segment_json
        # self.replay_json = replay_json
        
        # self.original_apollo_log_dir=longrun_apollo_log_dir
        # self.replay_apollo_log_dir = replay_apollo_log_dir        
        # self.analysis_dir=analysis_dir
        
        
        self.original_segment_json = Path(ori_segment_json)
        self.replay_json = Path(replay_json)
        self.original_apollo_log_dir = Path(longrun_apollo_log_dir)
        self.replay_apollo_log_dir = Path(replay_apollo_log_dir)
        self.analysis_dir = Path(analysis_dir)
        

        
        # Analysis results
        self.results = {}
        
        # Analysis thresholds
        self.POSITION_THRESHOLD = 0.5  # meters
        self.VELOCITY_THRESHOLD = 0.5  # m/s
        self.HIGH_ERROR_THRESHOLD = {
            "position": 2.0,  # meters
            "velocity": 2.0,  # m/s
            "deviation_count": 10
        }
        
        
        self.DETAILED_ANALYSIS_THRESHOLD = {
            "position": 1.0,  # 位置偏差阈值
            "velocity": 1.0,  # 速度偏差阈值
            "min_deviations": 5  # 最小偏差数量
        }


        self.extract_apollo_logs=extract_apollo_logs
    
    def _validate_paths(self):
        """验证所有必要的路径"""
        # 检查输入文件是否存在
        if not self.original_segment_json.exists():
            raise FileNotFoundError(f"Original JSON file not found: {self.original_segment_json}")
        if not self.replay_json.exists():
            raise FileNotFoundError(f"Replay JSON file not found: {self.replay_json}")
            
        # 检查日志目录是否存在
        if not self.original_apollo_log_dir.exists():
            raise NotADirectoryError(f"Original Apollo log directory not found: {self.original_apollo_log_dir}")
        if not self.replay_apollo_log_dir.exists():
            raise NotADirectoryError(f"Replay Apollo log directory not found: {self.replay_apollo_log_dir}")
            
        # 创建输出目录
        self.analysis_dir.mkdir(parents=True, exist_ok=True)
        

    # def run(self):
    #     """Main method for the analysis thread"""'
    
    #     try:
    #         logger.info("Starting test result analysis...")
            
    #         # 2. Compare JSON records
    #         json_comparison = self._compare_json_records()
    #         self._generate_json_comparison_report(json_comparison)
            
    #         # 2. 根据对比结果决定是否需要进行详细分析
    #         if self._needs_detailed_analysis(json_comparison):
    #             logger.info("Performing detailed log analysis...")
    #             ori_apollo_csv_dir,replay_apollo_csv_dir=self._extract_apollo_logs()
    #             log_analyzer = LogAnalyzer(
    #                 ori_apollo_csv_dir=str(ori_apollo_csv_dir),
    #                 replay_apollo_csv_dir=str(replay_apollo_csv_dir),
    #                 output_dir=str(self.analysis_dir)
    #             ) # done-- the dir and file path seems ok now
    #             log_analyzer.analyze()

    #             # 生成综合报告
    #                # 4. 生成综合报告
    #             self._generate_summary_report(json_comparison, log_analyzer.results)
    #             logger.info("Detailed analysis completed successfully")
                


    #         else:
    #             logger.info("No significant deviations found, skipping detailed analysis")
                
    #         logger.info("Analysis completed successfully")
    #     except Exception as e:
    #         logger.error(f"Analysis failed: {str(e)}")
    #         raise
    # def run(self) -> Dict:
    #     """
    #     运行分析流程
        
    #     Returns:
    #     Dict: 包含分析结果的字典
    #     1. 基础分析结果:
    #         - error_level: 初始错误等级 (HIGH/MEDIUM/LOW)
    #         - json_comparison_summary: JSON对比结果摘要
    #         - needs_detailed_analysis: 是否需要详细分析
            
    #     2. 如果needs_detailed_analysis为True，还会包含:
    #         - detailed_error_level: 详细分析后的错误等级
    #         - log_analysis_results: 完整的日志分析结果
    #         - trajectory_analysis: 轨迹分析结果
    #         - planning_analysis: 规划分析结果
    #         - state_analysis: 状态分析结果
    #     """
    #     try:
    #         logger.info("Starting test result analysis...")
            
    #         # 1. 验证路径
    #         self._validate_paths()
            
    #         # 2. 进行JSON记录对比
    #         json_comparison = self._compare_json_records()
    #         self._generate_json_comparison_report(json_comparison)
            
    #         # 3. 确定初始错误等级
    #         initial_error_level = self._determine_initial_error_level(json_comparison)
            
    #         # 构建基础分析结果
    #         analysis_result = {
    #             "error_level": initial_error_level,
    #             "json_comparison_summary": {
    #                 "ego_vehicle": {
    #                     "position": {
    #                         "max": float(np.max(json_comparison["ego_vehicle"]["position_differences"])),
    #                         "mean": float(np.mean(json_comparison["ego_vehicle"]["position_differences"])),
    #                         "significant_deviations": len(json_comparison["ego_vehicle"]["significant_deviations"])
    #                     },
    #                     "velocity": {
    #                         "max": float(np.max(json_comparison["ego_vehicle"]["velocity_differences"])),
    #                         "mean": float(np.mean(json_comparison["ego_vehicle"]["velocity_differences"]))
    #                     }
    #                 }
    #             },
    #             "needs_detailed_analysis": self._needs_detailed_analysis(json_comparison)
    #         }
            
    #         # 4. 判断是否需要详细分析
    #         if analysis_result["needs_detailed_analysis"]:
    #             logger.info("Performing detailed log analysis...")
    #             try:
    #                 # 提取Apollo日志
    #                 ori_apollo_csv_dir, replay_apollo_csv_dir = self._extract_apollo_logs()
                    
    #                 # 创建日志分析器
    #                 log_analyzer = LogAnalyzer(
    #                     ori_apollo_csv_dir=str(ori_apollo_csv_dir),
    #                     replay_apollo_csv_dir=str(replay_apollo_csv_dir),
    #                     output_dir=str(self.analysis_dir)
    #                 )
                    
    #                 # 执行详细分析
    #                 log_analyzer.analyze()
    #                 log_analysis_results = log_analyzer.results
                    
    #                 # 确定最终错误等级
    #                 final_error_level = self._determine_error_level(json_comparison, log_analysis_results)
                    
    #                 # 添加详细分析结果
    #                 analysis_result.update({
    #                     "detailed_error_level": final_error_level,
    #                     "log_analysis_results": log_analysis_results,
    #                     "trajectory_analysis": log_analysis_results.get("trajectory_analysis", {}),
    #                     "planning_analysis": log_analysis_results.get("planning_analysis", {}),
    #                     "state_analysis": log_analysis_results.get("state_analysis", {})
    #                 })
                    
    #                 # 生成综合报告
    #                 self._generate_summary_report(json_comparison, log_analysis_results)
                    
    #             except Exception as e:
    #                 logger.error(f"Detailed analysis failed: {e}")
    #                 analysis_result["detailed_analysis_error"] = str(e)
            
    #         # 5. 保存最终结果
    #         # result_path = self.analysis_dir / f"analysis_result_{analysis_result['error_level']}.json"
    #         # with open(result_path, 'w') as f:
    #             # json.dump(analysis_result, f, indent=4)
                
    #         logger.info(f"Analysis completed. Final error level: {analysis_result.get('detailed_error_level', initial_error_level)}")
    #         return analysis_result
                
    #     except Exception as e:
    #         logger.error(f"Analysis failed: {str(e)}")
    #         raise
                
            
            

    #     #     # 3. Analyze Apollo logs


    #     #     log_analyzer = LogAnalyzer(
    #     #         log_dir=str(self.original_apollo_log),
    #     #         replay_log_dir=str(self.replay_apollo_log),
    #     #         output_dir=str(self.analysis_dir)
    #     #     )
    #     #     log_analyzer.analyze()
            
    #     #     # 4. Generate summary report
    #     #     self._generate_summary_report(json_comparison, log_analyzer.results)
            
    #     # 
            
    #     # except Exception as e:
    #     #     logger.error(f"Analysis failed: {str(e)}")
    #     #     raise
        
        
    def run(self) -> Dict:
        """
        运行分析流程 - 始终执行完整的分析
        
        Returns:
            Dict: 包含分析结果的字典，包括：
                - json_comparison_summary: JSON对比结果
                - log_analysis_results: 日志分析结果
                - error_level: 最终错误等级
        """
        try:
            logger.info("Starting test result analysis...")
            
            # 1. 验证路径
            self._validate_paths()
            
            # 2. 进行JSON记录对比
            json_comparison = self._compare_json_records()
            self._generate_json_comparison_report(json_comparison)
            
            # 3. 进行日志分析（不再有条件判断）
            logger.info("Performing log analysis...")
            try:
                
                
                if self.extract_apollo_logs==True:
                # 提取Apollo日志
                    ori_apollo_csv_dir, replay_apollo_csv_dir = self._extract_apollo_logs()
                else:
                    ori_apollo_csv_dir= self.original_apollo_log_dir/ "extracted_log_csv"
                    replay_apollo_csv_dir= self.replay_apollo_log_dir / "extracted_log_csv" 
                
                
                # 创建日志分析器
                log_analyzer = LogAnalyzer(
                    ori_apollo_csv_dir=str(ori_apollo_csv_dir),
                    replay_apollo_csv_dir=str(replay_apollo_csv_dir),
                    output_dir=str(self.analysis_dir)
                )

                # 执行分析
                log_analyzer.analyze()
                log_analysis_results = log_analyzer.results
            except Exception as e:
                logger.error(f"Log analysis failed: {e}")
                traceback.print_exc()
                log_analysis_results = {}
                
            # 4. 确定最终错误等级
            final_error_level = self._determine_error_level(json_comparison, log_analysis_results)
            
            # 5. 构建完整的分析结果
            analysis_result = {
                "error_level": final_error_level,
                "json_comparison_summary": {
                    "ego_vehicle": {
                        "position": {
                            "max": float(np.max(json_comparison["ego_vehicle"]["position_differences"])),
                            "mean": float(np.mean(json_comparison["ego_vehicle"]["position_differences"])),
                            "significant_deviations": len(json_comparison["ego_vehicle"]["significant_deviations"])
                        },
                        "velocity": {
                            "max": float(np.max(json_comparison["ego_vehicle"]["velocity_differences"])),
                            "mean": float(np.mean(json_comparison["ego_vehicle"]["velocity_differences"]))
                        }
                    }
                },
                "log_analysis_results": {
                    "trajectory_analysis": log_analysis_results.get("trajectory_analysis", {}),
                    "planning_analysis": log_analysis_results.get("planning_analysis", {}),
                    "state_analysis": log_analysis_results.get("state_analysis", {})
                }
            }
            
            # 6. 生成最终报告
            self._generate_summary_report(json_comparison, log_analysis_results)
            
            logger.info(f"Analysis completed with error level: {final_error_level}")
            return analysis_result
                
        except Exception as e:
            logger.error(f"Analysis failed: {str(e)}")
            raise
    #添加新的判断方法
    def _needs_detailed_analysis(self, json_comparison: Dict) -> bool:
        """
        判断是否需要进行详细的日志分析
        根据JSON对比结果中的偏差情况来决定
        """
        return (
            len(json_comparison["ego_vehicle"]["significant_deviations"]) > self.DETAILED_ANALYSIS_THRESHOLD["min_deviations"] or
            max(json_comparison["ego_vehicle"]["position_differences"]) > self.DETAILED_ANALYSIS_THRESHOLD["position"] or
            max(json_comparison["ego_vehicle"]["velocity_differences"]) > self.DETAILED_ANALYSIS_THRESHOLD["velocity"]
        )
        
        

        

    def _extract_apollo_logs(self):
        """Extract Apollo logs to CSV files"""
        logger.info("Starting Apollo log extraction...")
        
        
        
        #extract the original apollo logs
        output_dir=self.original_apollo_log_dir / "extracted_log_csv"
        os.makedirs(output_dir, exist_ok=True)

        logger.info("Extracting original scenario logs...")        
        extractor = ApolloLogExtractor(record_dir=self.original_apollo_log_dir, output_dir=output_dir)
        extractor.process_all_records()
        
        
        logger.info("Log extraction completed for original Apollo log")
        

        
        #extract the replay apollo log
        output_dir_replay=self.replay_apollo_log_dir / "extracted_log_csv"
        os.makedirs(output_dir, exist_ok=True)
        logger.info("Extracting replay scenario logs...")        
        extractor_replay = ApolloLogExtractor(record_dir=self.replay_apollo_log_dir, output_dir=output_dir_replay)
        extractor_replay.process_all_records()
        
        
        logger.info("Log extraction completed for replay Apollo log")
    
        return output_dir, output_dir_replay
        
        
    def _compare_json_records(self) -> Dict:
        """Compare original and replay JSON records"""
        logger.info("Comparing JSON records...")
        
        # Load JSON files
        with open(self.original_segment_json, 'r') as f:
            original_data = json.load(f)
        with open(self.replay_json, 'r') as f:
            replay_data = json.load(f)
            
        # 创建原始帧的索引字典
        original_frame_map = {frame["frame"]: frame for frame in original_data}
        
        differences = {
            "ego_vehicle": {
                "position_differences": [],
                "velocity_differences": [],
                "significant_deviations": []
            },
            "other_vehicles": {
                "position_differences": [],
                "velocity_differences": [],
                "significant_deviations": []
            },
            "frame_pairs": [],
            "stats": {
                "total_original_frames": len(original_data),
                "total_replay_frames": len(replay_data),
                "matched_frames": 0
            }
        }
        
        logger.info(f"Original data has {len(original_data)} frames")
        logger.info(f"Replay data has {len(replay_data)} frames")
        
        # 遍历回放数据，根据original_frame找到对应的原始帧
        for replay_frame in replay_data:
            original_frame_id = replay_frame.get("original_frame")
            if original_frame_id not in original_frame_map:
                logger.warning(f"No matching original frame {original_frame_id} for replay frame {replay_frame['frame']}")
                continue
                
            orig_frame = original_frame_map[original_frame_id]
            differences["stats"]["matched_frames"] += 1
            
            # 记录帧对
            frame_pair = {
                "original_frame": original_frame_id,
                "replay_frame": replay_frame["frame"],
                "replay_count": replay_frame["replay_count"]
            }
            differences["frame_pairs"].append(frame_pair)
            
            # 比较自车
            orig_ego = orig_frame["ego_vehicle"]
            replay_ego = replay_frame["ego_vehicle"]
            
            ego_diff = self._compare_vehicles(orig_ego, replay_ego, frame_pair, is_ego=True)
            
            differences["ego_vehicle"]["position_differences"].append(ego_diff["position_diff"])
            differences["ego_vehicle"]["velocity_differences"].append(ego_diff["velocity_diff"])
            if ego_diff["is_significant"]:
                differences["ego_vehicle"]["significant_deviations"].append(ego_diff)
            
            # 比较周围车辆
            vehicle_id_mapping = replay_frame.get("vehicle_id_mapping", {})
            orig_vehicles = {str(v["id"]): v for v in orig_frame.get("nearby_vehicles", [])}
            replay_vehicles = {str(v["id"]): v for v in replay_frame.get("nearby_vehicles", [])}
            
            for orig_id, mapped_id in vehicle_id_mapping.items():
                orig_vehicle = orig_vehicles.get(orig_id)
                replay_vehicle = replay_vehicles.get(str(mapped_id))
                
                if orig_vehicle and replay_vehicle:
                    vehicle_diff = self._compare_vehicles(
                        orig_vehicle, 
                        replay_vehicle, 
                        frame_pair,
                        vehicle_type=replay_vehicle["type_id"]
                    )
                    
                    differences["other_vehicles"]["position_differences"].append(vehicle_diff["position_diff"])
                    differences["other_vehicles"]["velocity_differences"].append(vehicle_diff["velocity_diff"])
                    if vehicle_diff["is_significant"]:
                        differences["other_vehicles"]["significant_deviations"].append(vehicle_diff)
        
        logger.info(f"Matched {differences['stats']['matched_frames']} frames")
        logger.info(f"Found {len(differences['ego_vehicle']['significant_deviations'])} ego vehicle deviations")
        logger.info(f"Found {len(differences['other_vehicles']['significant_deviations'])} other vehicles deviations")
        
        return differences

    def _compare_vehicles(self, orig_vehicle, replay_vehicle, frame_pair, is_ego=False, vehicle_type=None):
        """比较单个车辆的状态差异"""
        # 获取原始位置
        orig_x = orig_vehicle.get("x", orig_vehicle.get("transform", {}).get("location", {}).get("x"))
        orig_y = orig_vehicle.get("y", orig_vehicle.get("transform", {}).get("location", {}).get("y"))
        orig_z = orig_vehicle.get("z", orig_vehicle.get("transform", {}).get("location", {}).get("z"))
        
        # 获取回放位置
        replay_loc = replay_vehicle["transform"]["location"]
        
        # 计算位置差异
        pos_diff = np.sqrt(
            (orig_x - replay_loc["x"])**2 +
            (orig_y - replay_loc["y"])**2 +
            (orig_z - replay_loc["z"])**2
        )
        
        # 计算速度差异
        orig_vel = orig_vehicle["velocity"]
        replay_vel = replay_vehicle["velocity"]
        orig_vel_mag = np.sqrt(orig_vel["x"]**2 + orig_vel["y"]**2 + orig_vel["z"]**2)
        replay_vel_mag = replay_vel["magnitude"]
        vel_diff = abs(orig_vel_mag - replay_vel_mag)
        
        # 判断是否为显著偏差
        is_significant = pos_diff > self.POSITION_THRESHOLD or vel_diff > self.VELOCITY_THRESHOLD
        
        return {
            "frame_pair": frame_pair,
            "vehicle_type": "ego_vehicle" if is_ego else vehicle_type,
            "position_diff": pos_diff,
            "velocity_diff": vel_diff,
            "is_significant": is_significant
        }
            
    def _determine_initial_error_level(self, json_comparison: Dict) -> str:
        """
        根据JSON对比结果确定初始错误级别
        :param json_comparison: JSON对比结果字典
        :return: 错误级别 ("LOW", "MEDIUM", "HIGH")
        """
        try:
            ego_vehicle = json_comparison["ego_vehicle"]
            
            # 获取最大偏差
            max_pos_diff = max(ego_vehicle["position_differences"])
            max_vel_diff = max(ego_vehicle["velocity_differences"])
            deviation_count = len(ego_vehicle["significant_deviations"])
            
            # 使用已定义的阈值进行判断
            if (max_pos_diff > self.HIGH_ERROR_THRESHOLD["position"] or
                max_vel_diff > self.HIGH_ERROR_THRESHOLD["velocity"] or
                deviation_count > self.HIGH_ERROR_THRESHOLD["deviation_count"]):
                return "HIGH"
            elif (max_pos_diff > self.DETAILED_ANALYSIS_THRESHOLD["position"] or
                max_vel_diff > self.DETAILED_ANALYSIS_THRESHOLD["velocity"] or
                deviation_count > self.DETAILED_ANALYSIS_THRESHOLD["min_deviations"]):
                return "MEDIUM"
            return "LOW"
            
        except Exception as e:
            logger.error(f"Error determining initial error level: {e}")
            return "HIGH"  # 如果出现错误，返回高风险级别以确保安全
            
    # # Line 300-343: 修改JSON对比报告生成方法
    # def _generate_json_comparison_report(self, json_comparison: Dict):
    #     """Generate JSON comparison report"""
    #     error_level = self._determine_initial_error_level(json_comparison)
    #     report = {
    #         "error_level": error_level,  # 添加错误等级
    #         "needs_detailed_analysis": self._needs_detailed_analysis(json_comparison),  # 添加是否需要详细分析的标志
    #         "statistics": {
    #             "ego_vehicle": {
    #                 "position": {
    #                     "mean": float(np.mean(json_comparison["ego_vehicle"]["position_differences"])),
    #                     "max": float(np.max(json_comparison["ego_vehicle"]["position_differences"])),
    #                     "std": float(np.std(json_comparison["ego_vehicle"]["position_differences"]))
    #                 },
    #                 "velocity": {
    #                     "mean": float(np.mean(json_comparison["ego_vehicle"]["velocity_differences"])),
    #                     "max": float(np.max(json_comparison["ego_vehicle"]["velocity_differences"])),
    #                     "std": float(np.std(json_comparison["ego_vehicle"]["velocity_differences"]))
    #                 }
    #             }
    #         },
    #         "frame_statistics": json_comparison["stats"],
    #         "significant_deviations": {
    #             "ego_vehicle": json_comparison["ego_vehicle"]["significant_deviations"],
    #             "other_vehicles": json_comparison["other_vehicles"]["significant_deviations"]
    #         }
    #     }
        
    #     # 生成带状态的文件名
    #     report_filename = f"json_comparison_report_{error_level}.json"
    #     report_path = self.analysis_dir / report_filename
    #     with open(report_path, 'w') as f:
    #         json.dump(report, f, indent=4)
       


    def _generate_json_comparison_report(self, json_comparison: Dict):
        """
        生成 JSON 比较报告，并递归转换数据结构中的 NumPy 类型。
        """
        # 确定错误等级
        error_level = self._determine_initial_error_level(json_comparison)

        # 生成报告（可能包含 NumPy 类型值）
        report = {
            "error_level": error_level,
            "needs_detailed_analysis": self._needs_detailed_analysis(json_comparison),
            "statistics": {
                "ego_vehicle": {
                    "position": {
                        "mean": np.mean(json_comparison["ego_vehicle"]["position_differences"]),
                        "max": np.max(json_comparison["ego_vehicle"]["position_differences"]),
                        "std": np.std(json_comparison["ego_vehicle"]["position_differences"])
                    },
                    "velocity": {
                        "mean": np.mean(json_comparison["ego_vehicle"]["velocity_differences"]),
                        "max": np.max(json_comparison["ego_vehicle"]["velocity_differences"]),
                        "std": np.std(json_comparison["ego_vehicle"]["velocity_differences"])
                    }
                }
            },
            "frame_statistics": json_comparison["stats"],
            "significant_deviations": {
                "ego_vehicle": json_comparison["ego_vehicle"]["significant_deviations"],
                "other_vehicles": json_comparison["other_vehicles"]["significant_deviations"]
            }
        }

        # 递归转换 NumPy 类型为原生 Python 类型
        report = convert_numpy_types(report)

        # 生成报告文件名
        report_filename = f"json_comparison_report_{error_level}.json"
        report_path = self.analysis_dir / report_filename

        # 写入 JSON 文件
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=4)

    def _generate_summary_report(self, json_comparison: Dict, log_analysis: Dict):
        """Generate final summary report"""
        def convert_to_serializable(obj):
            """将对象转换为可JSON序列化的格式"""
            if isinstance(obj, pd.DataFrame):
                return {
                    "type": "DataFrame",
                    "data": obj.to_dict(orient='records'),
                    "columns": obj.columns.tolist(),
                    "index": obj.index.tolist()
                }
            elif isinstance(obj, pd.Series):
                return {
                    "type": "Series",
                    "data": obj.to_dict(),
                    "name": obj.name
                }
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.int64, np.int32)):
                return int(obj)
            elif isinstance(obj, (np.float64, np.float32)):
                return float(obj)
            elif isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_to_serializable(item) for item in obj]
            return obj

        try:
            error_level = self._determine_error_level(json_comparison, log_analysis)
            
            # 构建summary字典
            summary = {
                "error_level": error_level,
                "json_comparison_summary": {
                    "ego_vehicle": {
                        "position": {
                            "max": float(np.max(json_comparison["ego_vehicle"]["position_differences"])),
                            "mean": float(np.mean(json_comparison["ego_vehicle"]["position_differences"])),
                            "significant_deviations": len(json_comparison["ego_vehicle"]["significant_deviations"])
                        },
                        "velocity": {
                            "max": float(np.max(json_comparison["ego_vehicle"]["velocity_differences"])),
                            "mean": float(np.mean(json_comparison["ego_vehicle"]["velocity_differences"]))
                        }
                    }
                },
                "apollo_log_analysis": {
                    "trajectory_analysis": log_analysis.get("trajectory_analysis"),
                    "planning_analysis": log_analysis.get("planning_analysis"),
                    "state_analysis": log_analysis.get("state_analysis")
                }
            }

            # 转换为可序列化格式
            serializable_summary = convert_to_serializable(summary)
            
            # 生成带状态的文件名
            report_filename = f"unified_report_{error_level}.json"
            report_path = self.analysis_dir / report_filename
            
            # 保存到文件
            with open(report_path, 'w') as f:
                json.dump(serializable_summary, f, indent=4)
                
            logger.info(f"Successfully saved summary report to {report_path}")
            
        except Exception as e:
            logger.error(f"Error generating summary report: {e}")
            # 保存错误信息
            error_report = {
                "error": str(e),
                "traceback": traceback.format_exc()
            }
            error_path = self.analysis_dir / f"error_report_{error_level}.json"
            with open(error_path, 'w') as f:
                json.dump(error_report, f, indent=4)
            logger.info(f"Saved error report to {error_path}")
    
    #TODO: 检查一下。 同时在实际用的时候check一下
    def _determine_error_level(self, json_comparison: Dict, log_analysis: Dict) -> str:
        """确定测试的整体错误等级"""
        # 1. JSON对比结果评估
        json_error_level = self._evaluate_json_comparison(json_comparison)
        
        # 2. 日志分析结果评估
        log_error_level = self._evaluate_log_analysis(log_analysis)
        
        # 3. 综合评估
        if json_error_level == "HIGH" or log_error_level == "HIGH":
            return "HIGH"
        elif json_error_level == "MEDIUM" or log_error_level == "MEDIUM":
            return "MEDIUM"
        else:
            return "LOW"

    def _evaluate_json_comparison(self, json_comparison: Dict) -> str:
        """评估JSON对比结果"""
        ego_vehicle = json_comparison["ego_vehicle"]
        
        # 位置偏差评估
        max_pos_diff = max(ego_vehicle["position_differences"])
        mean_pos_diff = np.mean(ego_vehicle["position_differences"])
        
        # 速度偏差评估
        max_vel_diff = max(ego_vehicle["velocity_differences"])
        mean_vel_diff = np.mean(ego_vehicle["velocity_differences"])
        
        # 显著偏差计数
        deviation_count = len(ego_vehicle["significant_deviations"])
        
        if (max_pos_diff > self.HIGH_ERROR_THRESHOLD["position"] or
            max_vel_diff > self.HIGH_ERROR_THRESHOLD["velocity"] or
            deviation_count > self.HIGH_ERROR_THRESHOLD["deviation_count"]):
            return "HIGH"
        elif (mean_pos_diff > self.DETAILED_ANALYSIS_THRESHOLD["position"] or
            mean_vel_diff > self.DETAILED_ANALYSIS_THRESHOLD["velocity"]):
            return "MEDIUM"
        return "LOW"

    def _evaluate_log_analysis(self, log_analysis: Dict) -> str:
        """
        评估日志分析结果，处理所有可能的空值情况
        
        Args:
            log_analysis: 包含分析结果的字典
            
        Returns:
            str: 评估等级 ("HIGH", "MEDIUM", "LOW")
        """
        if not log_analysis:
            return "LOW"
        
        # 获取并检查轨迹分析结果
        trajectory_analysis = log_analysis.get("trajectory_analysis", {})
        if not isinstance(trajectory_analysis, dict):
            trajectory_analysis = {}
        
        # 检查轨迹分析中的严重错误
        if trajectory_analysis.get("error") or trajectory_analysis.get("status") == "HIGH":
            return "HIGH"
        
        # 安全地获取各个指标值，提供默认值0
        trajectory_diff = trajectory_analysis.get("max_difference", 0)
        if not isinstance(trajectory_diff, (int, float)) or trajectory_diff is None:
            trajectory_diff = 0
            
        planning_analysis = log_analysis.get("planning_analysis", {})
        if not isinstance(planning_analysis, dict):
            planning_analysis = {}
        planning_changes = planning_analysis.get("decision_changes", 0)
        if not isinstance(planning_changes, (int, float)) or planning_changes is None:
            planning_changes = 0
            
        state_analysis = log_analysis.get("state_analysis", {})
        if not isinstance(state_analysis, dict):
            state_analysis = {}
        state_differences = state_analysis.get("total_differences", 0)
        if not isinstance(state_differences, (int, float)) or state_differences is None:
            state_differences = 0
        
        # 设定阈值
        TRAJECTORY_THRESHOLD = 2.0  # 米
        PLANNING_CHANGES_THRESHOLD = 5  # 次
        STATE_DIFFERENCES_THRESHOLD = 10  # 次
        
        # 评估逻辑
        try:
            if (trajectory_diff > TRAJECTORY_THRESHOLD or
                planning_changes > PLANNING_CHANGES_THRESHOLD or
                state_differences > STATE_DIFFERENCES_THRESHOLD):
                return "HIGH"
            elif (trajectory_diff > TRAJECTORY_THRESHOLD/2 or
                planning_changes > PLANNING_CHANGES_THRESHOLD/2):
                return "MEDIUM"
            return "LOW"
        except Exception as e:
            self.logger.error(f"Error in evaluation: {e}")
            return "LOW"  # 发生错误时返回默认值
        
        
        
        
        
      