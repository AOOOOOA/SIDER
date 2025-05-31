from pathlib import Path
import json
from loguru import logger
from analyzer import TrajectoryAnalyzer
import sys

def analyze_existing_test(test_dir: Path, config_name: str, segment_id: int):
    """分析已存在的测试数据
    
    Args:
        test_dir: 时间戳测试目录
        config_name: 测试配置名称（如 'test_v90_w60'）
        segment_id: 要分析的片段ID
    """
    try:
        # 构建完整的测试配置目录路径
        config_dir = test_dir / config_name
        
        # 构建文件路径
        long_run_file = config_dir / "original_scenario" / "carla_records" / "vehicle_states.json"
        segment_file = config_dir / "replay_scenarios" / f"segment_{segment_id:03d}" / "carla_records" / "vehicle_states.json"
        mapping_file = config_dir / "metadata" / "segment_mappings.json"
        
        # 验证文件是否存在
        for file_path in [long_run_file, segment_file, mapping_file]:
            if not file_path.exists():
                logger.error(f"Required file not found: {file_path}")
                return None
                
        logger.info(f"Found all required files:")
        logger.info(f"Long run file: {long_run_file}")
        logger.info(f"Segment file: {segment_file}")
        logger.info(f"Mapping file: {mapping_file}")
        
        # 创建分析器并执行分析
        analyzer = TrajectoryAnalyzer(
            long_run_file=long_run_file,
            segment_files=[segment_file],
            mapping_file=mapping_file
        )
        
        result = analyzer.analyze_segment(segment_id)
        
        # 保存分析结果
        result_file = config_dir / "analysis_results" / f"segment_{segment_id:03d}_analysis.json"
        result_file.parent.mkdir(exist_ok=True)
        
        with open(result_file, 'w') as f:
            json.dump(result.__dict__, f, indent=2)
            
        logger.info(f"Analysis completed for segment {segment_id}")
        return result
        
    except Exception as e:
        logger.error(f"Analysis failed for segment {segment_id}: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    # 设置日志
    logger.remove()
    logger.add(sys.stderr, level="DEBUG")
    
    # 指定要分析的测试目录
    test_dir = Path("/home/w/workspace/carla_apollo/apollo/modules/carla_bridge/multi_vehicle_fuzz/test_outputs/20241122_104556")
    config_name = "test_20241122_104557_v90_w60"  # 替换为实际的测试配置名称
    segment_id = 1
    
    result = analyze_existing_test(test_dir, config_name, segment_id)
    
    if result:
        logger.info("Analysis successful!")
    else:
        logger.error("Analysis failed!")