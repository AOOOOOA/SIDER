import os
import string
import sys
from datetime import datetime
from pathlib import Path

from absl import flags
from absl.flags import FLAGS
from loguru import logger
#TODO: 用于生成id， 这里暂时先注释了
# from nanoid import generate

#TODO: 检查这些值是不是都是可用的
from config import LOGGING_FORMAT, PROJECT_NAME, PROJECT_ROOT
from typing import Union

import subprocess

def set_up_gflags():
    #TODO: need more flags
    
    
    flags.DEFINE_boolean("dreamview", True, "Enable Dreamview.")
    flags.DEFINE_string("log_level", "INFO", "Log level.")
    flags.DEFINE_string(
        "execution_id", datetime.now().strftime(r"%m%d_%H%M%S"), "Execution ID."
    )
    flags.DEFINE_string("map", "carla_town01", "Name of the map to use.")
    flags.DEFINE_boolean("colorize", True, "Colorize log output.") #没有这个colorize的话，log level 会报错
    
    
def get_output_dir(root: Path = PROJECT_ROOT, mkdir: bool = True) -> Path:
    result = Path(root, "out", f"{FLAGS.execution_id}_{FLAGS.map}")
    if not result.exists() and mkdir:
        result.mkdir(parents=True)
    return result


# MODIFY: 修改 get_log_file 函数
def get_log_file(test_output_dir: Path, is_replay: bool = False, segment: str = None) -> Path:
    """获取日志文件路径"""
    if is_replay and segment:
        # 回放模式的日志路径
        log_dir = test_output_dir / "replay_scenarios" / segment / "apollo_logs"
    else:
        # 使用提供的目录作为日志目录
        log_dir = test_output_dir
    
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir / "test.log"


def set_up_logging(level: Union[str, int], test_dir: Path, is_replay: bool = False, segment: str = None) -> None:
    """设置日志系统
    Args:
        level: 日志级别
        test_dir: 测试根目录路径 (例如: test_outputs/20240120_123456/)
        is_replay: 是否是回放模式
        segment: 回放片段名称
    """
    logger.remove()  # 移除所有已有的处理器
    
    # 在测试根目录下创建统一的日志文件
    log_file = test_dir / "full_test_run.log"
    
    # 定义日志格式，使用颜色区分不同类型的日志
    format_str = "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
    format_str += "<level>{level: <8}</level> | "
    
    # 根据不同的测试阶段使用不同的颜色
    if is_replay:
        # 回放日志使用黄色
        format_str += f"<yellow>[REPLAY-{segment}]</yellow> | "
    else:
        # 主测试日志使用蓝色
        format_str += "<blue>[MAIN]</blue> | "
    
    # 添加模块信息
    format_str += "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> | "
    format_str += "<level>{message}</level>"
    
    # 添加文件处理器
    logger.add(
        str(log_file),
        format=format_str,
        level=level,
        colorize=True,
        enqueue=True,
        backtrace=True,
        diagnose=True
    )
    
    # 添加控制台处理器
    logger.add(
        sys.stdout,
        format=format_str,
        level=level,
        colorize=True,
        enqueue=True
    )
    
    # 记录初始信息
    if not is_replay:
        logger.info("Starting new test run")

# def set_up_logging(log_level: str, test_dir: Path):
#     """设置日志记录器"""
#     log_dir = test_dir / "logs"
#     log_dir.mkdir(parents=True, exist_ok=True)
    
#     timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#     log_file = log_dir / f"test_{timestamp}.log"
    
#     logger.remove()  # 移除已有的handler
    
#     # 添加文件handler，使用rotation和compression
#     logger.add(
#         log_file,
#         level=log_level,
#         format="{time:YYYY-MM-DD HH:mm:ss} | {level} | {message}",
#         rotation="100 MB",  # 当文件达到100MB时轮转
#         compression="zip",  # 压缩旧日志
#         enqueue=True,      # 启用异步写入
#         backtrace=True,    # 错误时显示完整堆栈
#         diagnose=True      # 启用诊断信息
#     )
    
#     # 添加控制台handler
#     logger.add(
#         sys.stdout,
#         level=log_level,
#         format="{time:YYYY-MM-DD HH:mm:ss} | {level} | {message}",
#         enqueue=True
#     )
def run_command(command, args, cwd=None):
    """以非阻塞方式运行命令，并捕获输出"""
    try:
        full_command = [command] + args
        logger.debug(f"Running command: {' '.join(full_command)}")
        
        process = subprocess.Popen(
            full_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=cwd
        )
        return process
    except Exception as e:
        logger.error(f"Failed to run command {' '.join(full_command)}: {e}")
        return None