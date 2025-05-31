from pathlib import Path
import json
from analyzer import TestResultAnalyzer
from loguru import logger
import sys
import datetime
# 配置logger
logger.remove()
logger.add(sys.stderr, level="INFO")
import traceback
import numpy as np


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

def test_analyzer_offline():
    """离线测试分析器功能"""
    try:
        # 测试数据路径
        test_dir = Path("test_outputs/20241212_154122/test_20241212_154133_v60_w30")
        segment_name = "segment_001"
        
        # 构建所需的文件路径
        ori_segment_json = test_dir / "original_scenario" / "carla_records" / "segments" / f"{segment_name}.json"
        replay_json = test_dir / "replay_scenarios" / segment_name / "carla_records" / f"replay_vehicle_states_{segment_name}.json"
        longrun_apollo_log_dir = test_dir / "original_scenario" / "apollo_logs"
        replay_apollo_log_dir = test_dir / "replay_scenarios" / segment_name / "apollo_logs"
        analysis_dir = test_dir / "analysis" / segment_name


        result_dir=Path("debug_test")

        # 验证文件是否存在
        logger.info("Validating input files...")
        for path, desc in [
            (ori_segment_json, "Original scenario file"),
            (replay_json, "Replay scenario file"),
            (longrun_apollo_log_dir, "Original Apollo log directory"),
            (replay_apollo_log_dir, "Replay Apollo log directory")
        ]:
            if not path.exists():
                raise FileNotFoundError(f"{desc} not found: {path}")
            logger.info(f"Found {desc}: {path}")

        # 创建分析器实例
        logger.info("Creating analyzer instance...")
        analyzer = TestResultAnalyzer(
            ori_segment_json=ori_segment_json,
            replay_json=replay_json,
            extract_apollo_logs=False,
            longrun_apollo_log_dir=longrun_apollo_log_dir,
            replay_apollo_log_dir=replay_apollo_log_dir,
            analysis_dir=analysis_dir
            
        )

        # 运行分析
        logger.info("Starting analysis...")
        result = analyzer.run()
        print("*********************************")
        print("aligned states:", result.keys())
        print("log analysis result", result["log_analysis_results"].keys())
        print("log analysis result", result["log_analysis_results"]["state_analysis"]["stats"])
        
        print("*********************************")
        
        print("result is:", result)
        converted_result=convert_numpy(result)   
        
        print("________")
        print("converted_result is", converted_result)
        detailed_result_file = result_dir / f"{segment_name}_detailed.json"
        with open(detailed_result_file, 'w') as f:
            json.dump(converted_result, f, indent=2)
            
        
        # 保存摘要结果
        
        summary_result_file = result_dir / f"{segment_name}_summary.json"
        summary = {
            "segment_id": segment_name,
            "segment_name": segment_name,
            "error_level": result.get("error_level", "UNKNOWN"),
            "analysis_timestamp": datetime.datetime.now().isoformat(),
            "key_metrics": {
                "trajectory_difference": result.get("trajectory_analysis", {}).get("max_difference"),
                "planning_changes": result.get("planning_analysis", {}).get("decision_changes"),
                "state_differences": result.get("state_analysis", {}).get("total_differences")
            }
        }
        
        print("+++++++++++++++++++++++++++++++")
        print("summary is:", summary)
        print("===============================")
        with open(summary_result_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        #TODO: try to save the result detailed json and the result summary json to see whetehr the json dump is right or not
        
        
        
        # 输出分析结果
        logger.info("Analysis completed. Results:")
        logger.info(f"Error Level: {result.get('error_level', 'UNKNOWN')}")
        
        # 保存结果
        # output_file = analysis_dir / "analysis_result.json"
        # with open(output_file, 'w') as f:
        #     json.dump(result, f, indent=2)
        # logger.info(f"Results saved to: {output_file}")

    except Exception as e:
        logger.error(f"Analysis failed: {e}")
        traceback.print_exc()
        raise

if __name__ == "__main__":
    test_analyzer_offline()
    
    
#TODO: 1. fix 要存数据的那些代码 2. 看下存的数据有没有问题 3. 看下error level 和 json comparison的逻辑对不对



