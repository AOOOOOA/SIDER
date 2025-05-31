from pathlib import Path

# PROJECT_ROOT = Path(__file__).parent.parent
# 路径映射配置
PATH_MAPPINGS = {
    "PROJECT_ROOT": "/apollo/multi_vehicle_fuzz",
    "DATA_DIR": "/apollo/data",
    "LOG_DIR": "/apollo/data/log",
    "CYBER_PATH": "/apollo/cyber"
}
PROJECT_ROOT = Path(__file__).parent

# 项目根目录
# PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PROJECT_NAME = "apollo_dev_w" #TODO: 这个看情况要不要修改


#FIXME: 但是这个后面还是要改，改为相对目录，因为要放在server 上去运行
DATA_DIR = "/home/w/workspace/carla_apollo"




# DATA_DIR = Path(PROJECT_ROOT, "data")
DOWNLOAD_DIR = Path(DATA_DIR, "download")

# Docker Configurations
DOCKER_CMD = "docker"

#TODO: 修改路已经
# Apollo Configurations
APOLLO_ROOT = Path(DATA_DIR, "apollo")
print("APOLLO_ROOT",APOLLO_ROOT)
APOLLO_FLAGFILE = Path(APOLLO_ROOT, "modules", "common", "data", "global_flagfile.txt")


BASE_OUTPUT_DIR = Path(PROJECT_ROOT, "test_outputs")

CONFIG_DIR = Path(__file__)
# WAYPOINTS_PATH=Path("my_lib/carla_wps","Town03_adj_waypoints_classified_with_random_points.json").resolve()
#TODO: 这个不用， 看下有什么后果
WAYPOINTS_PATH = CONFIG_DIR / "../my_lib/carla_wps/Town03_adj_waypoints_classified_with_random_points.json"

# 将路径标准化为绝对路径
WAYPOINTS_PATH = WAYPOINTS_PATH.resolve()

# APOLLO_RELEASE = "https://github.com/YuqiHuai/apollo/archive/refs/tags/v7.0.1.zip"
# APOLLO_RELEASE_NAME = "BaiduApollo-7.0.1"
# SIM_CONTROL_RELEASE = (
    # "https://github.com/YuqiHuai/sim_control_standalone/archive/refs/tags/v7.0.1.zip"
# )
# SIM_CONTROL_RELEASE_NAME = "sim_control_standalone-7.0.1"

#TODO: 这个map数据的地址是要修改一下的。
"""
# Map Configurations
MAPS_DIR = Path(DATA_DIR, "maps")
SUPPORTED_MAPS = list(x.name for x in MAPS_DIR.iterdir() if x.is_dir())
"""
#FIXME: 这里的scripts 也要跟着改， 不过我们项目里的dev_start 也是在apollo项目里面的，所以这个修改的工作量应该不会到那么大？

# Script Configurations
class SCRIPTS:
    SCRIPTS_DIR = Path(DATA_DIR, "apollo","docker","scripts")
    DEV_START = Path(SCRIPTS_DIR, "dev_start.sh")
    MULTI_CTN_DEV_START = Path(SCRIPTS_DIR, "multi_ctn_dev_start.sh")



#下面的是用于匹配logging 信息的正则表达式， 以及设置的用于记录logger的格式。
# Other Configurations
LOGGING_PREFIX_REGEX = (
    "^(?P<severity>[DIWEF])(?P<month>\d\d)(?P<day>\d\d) "
    "(?P<hour>\d\d):(?P<minute>\d\d):(?P<second>\d\d)\.(?P<microsecond>\d\d\d) "
    "(?P<filename>[a-zA-Z<][\w._<>-]+):(?P<line>\d+)"
)
LOGGING_FORMAT = (
    "<level>{level.name[0]}{time:MMDD}</level> "
    "<green>{time:HH:mm:ss.SSS}</green> "
    "<cyan>{file}:{line}</cyan>] "
    "<bold>{message}</bold>"
)


#TODO: 需要改成有结构的项目路径
CARLA_SIMULATOR={
        'path': "/home/w/workspace/carla_apollo/CARLA_0.9.14/CarlaUE4.sh",
        'args': ['--windowed']
    }

# TODO: 如何把这个数量写成一个参数？ 用cursor 来写一下
#TODO: 以及最后这个generate traffic 和 record data 要放在本目录下面--done, 更改下路径
# 获取当前文件所在目录的路径
CURRENT_DIR = Path(__file__).parent

GENERATE_TRAFFIC = {
    'path': str(CURRENT_DIR / 'generate_traffic.py'),
    'args': ['-n', '20', '-w', '20', '--safe']
}
# 添加输出目录配置
OUTPUT_DIR = Path(DATA_DIR, "recorded_data")
CARLA_RECORD_DATA = {
    'path': str(CURRENT_DIR / 'record_data.py'),
    'args': ['--output', str(OUTPUT_DIR / 'vehicle_log.json')]
}

# GENERATE_TRAFFIC = {
#         'path': "/home/w/workspace/carla_apollo/CARLA_0.9.14/PythonAPI/examples/generate_traffic.py",
#         'args': ['-n', '20', '-w', '20', '--safe']
#     }

# CARLA_RECORD_DATA = {
#         'path': "/home/w/workspace/carla_apollo/CARLA_0.9.14/PythonAPI/examples/carla_record_data.py",
#         'args': ['-n', '20', '-w', '20', '--safe']
#     }  # 这里的args 应该主要是要做出来

