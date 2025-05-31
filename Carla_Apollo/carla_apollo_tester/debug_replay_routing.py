from modules.common_msgs.routing_msgs import routing_pb2

# 加载并解析 RoutingRequest 文件
request_file = "test_outputs/20241211_113935/test_20241211_113946_v60_w30/routing_request.pb"

with open(request_file, "rb") as f:
    serialized_data = f.read()

routing_request = routing_pb2.RoutingRequest()
routing_request.ParseFromString(serialized_data)

print(routing_request)
print(type(routing_request))

print("================================================")

response_file = "test_outputs/20241211_113935/test_20241211_113946_v60_w30/routing_response.pb"

with open(response_file, "rb") as f:
    serialized_data = f.read()

routing_response = routing_pb2.RoutingResponse()
routing_response.ParseFromString(serialized_data)

print(routing_response)
print(type(routing_response))




print(routing_response.header.timestamp_sec)
# # 打印起点和终点信息
# for i, waypoint in enumerate(routing_request.waypoint):
#     print(f"Waypoint {i + 1}:")
#     print(f"  Pose X: {waypoint.pose.x}")
#     print(f"  Pose Y: {waypoint.pose.y}")
#     print(f"  ID: {waypoint.id}")
#     print(f"  S: {waypoint.s}")