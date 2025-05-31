import sys
import time

# 添加 Apollo 路径以导入需要的模块
sys.path.append("../")
sys.path.append("../../")
sys.path.append("../../../")
sys.path.append("/home/w/workspace/carla_apollo/apollo")
sys.path.append("/apollo/modules")
sys.path.append("/apollo")

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.routing_msgs import routing_pb2
from modules.common_msgs.basic_msgs import error_code_pb2  # 导入 error_code 枚举

def MockRoutingResponse():
    """
    Main node to mock routing response
    """
    # 初始化 Cyber RT 节点
    cyber.init()
    node = cyber.Node("mock_routing_responder")

    # 创建 RoutingResponse 消息
    routing_response = routing_pb2.RoutingResponse()

    # 填充 header 信息
    routing_response.header.timestamp_sec = 1730362508.4884717
    routing_response.header.module_name = 'routing'
    routing_response.header.sequence_num = 0

    # 填充 road 和 passage 数据
    roads = [
        {"id": "1", "segment_id": "road_1_lane_0_-1", "start_s": 32.34221698227307, "end_s": 157.55084655698514},
        {"id": "67", "segment_id": "road_67_lane_0_-1", "start_s": 0.0, "end_s": 23.060396414940456},
        {"id": "25", "segment_id": "road_25_lane_0_-1", "start_s": 0.0, "end_s": 35.18},
        {"id": "333", "segment_id": "road_333_lane_0_-1", "start_s": 0.0, "end_s": 15.649319335860513},
        {"id": "9", "segment_id": "road_9_lane_0_1", "start_s": 0.0, "end_s": 43.60000000000001},
        {"id": "193", "segment_id": "road_193_lane_0_1", "start_s": 0.0, "end_s": 15.50124377040209},
        {"id": "21", "segment_id": "road_21_lane_0_1", "start_s": 0.0, "end_s": 34.910000000000004},
        {"id": "131", "segment_id": "road_131_lane_0_1", "start_s": 0.0, "end_s": 16.886925341342838},
        {"id": "2", "segment_id": "road_2_lane_0_1", "start_s": 0.0, "end_s": 42.269134720166555},
        {"id": "61", "segment_id": "road_61_lane_0_1", "start_s": 0.0, "end_s": 23.120000000000033},
        {"id": "1", "segment_id": "road_1_lane_0_1", "start_s": 0.0, "end_s": 128.74917893116415}
    ]

    for road_data in roads:
        road_segment = routing_response.road.add()
        road_segment.id = road_data["id"]

        passage = road_segment.passage.add()
        lane_segment = passage.segment.add()
        lane_segment.id = road_data["segment_id"]
        lane_segment.start_s = road_data["start_s"]
        lane_segment.end_s = road_data["end_s"]

        passage.can_exit = True
        passage.change_lane_type = routing_pb2.FORWARD

    # 设置测量信息
    routing_response.measurement.distance = 504.13482808858873

    # 填充 RoutingRequest 部分
    routing_request = routing_response.routing_request
    routing_request.header.timestamp_sec = 1730362508.4838064
    routing_request.header.module_name = "dreamview"
    routing_request.header.sequence_num = 52

    # 添加起点和终点
    start_waypoint = routing_request.waypoint.add()
    start_waypoint.id = "road_1_lane_0_-1"
    start_waypoint.s = 32.34221698227307
    start_waypoint.pose.x = 293.288818359375
    start_waypoint.pose.y = 1.9600721597671509

    end_waypoint = routing_request.waypoint.add()
    end_waypoint.id = "road_1_lane_0_1"
    end_waypoint.s = 128.74917893116415
    end_waypoint.pose.x = 296.8299989731307
    end_waypoint.pose.y = -0.044337310916503725

    # 修复 map_version 的问题，将字符串转换为 bytes
    routing_response.map_version = b"1"  # 将字符串 "1" 转换为 bytes

    # 使用正确的枚举 error_code
    routing_response.status.error_code = error_code_pb2.OK  # 使用正确的 error_code 枚举
    routing_response.status.msg = "Success!"

    # 创建 Writer，发布到 /apollo/routing_response 话题
    writer = node.create_writer('/apollo/routing_response', routing_pb2.RoutingResponse)

    # 模拟发布 RoutingResponse
    time.sleep(2.0)  # 等待一些时间再发布消息
    print("Publishing routing_response:", routing_response)
    writer.write(routing_response)

    # 关闭 Cyber RT 节点
    cyber.shutdown()

if __name__ == '__main__':
    MockRoutingResponse()