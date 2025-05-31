#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Insert routing request
Usage:
    mock_routing_request.py
"""
import argparse
import os
import sys
import time

sys.path.append("../")
sys.path.append("../../")
sys.path.append("../../../")
sys.path.append("/home/w/workspace/carla_apollo/apollo")
sys.path.append("/apollo/modules")
sys.path.append("/apollo")
sys.path.append("/apollo/cyber/python")
sys.path.append("/apollo/bazel-bin")

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.routing_msgs import routing_pb2
# from set_route import RouteSettingNode
# from set_route import CarlaCyberBridge

#TODO: read in the result returned the set_route.py as the routing request 
from loguru import logger

#!/usr/bin/env python
#
# Copyright (c) 2023 synkrotron.ai
#

sys.path.append("../")

from cyber.python.cyber_py3 import cyber_time
from carla_bridge.core.node import CyberNode

import carla
import yaml

# to generate a random spawning position or vehicles
import random
secure_random = random.SystemRandom()




# class CarlaCyberBridge(CyberNode):

#     # in synchronous mode, if synchronous_mode_wait_for_vehicle_control_command is True,
#     # wait for this time until a next tick is triggered.
#     VEHICLE_CONTROL_TIMEOUT = 1.0

#     def __init__(self):
#         super().__init__("route_setting_node")
    
#     def initialize_bridge(self,carla_world):
#         """
#         Initialize the bridge
#         """
        
#         cyber.init()
#         node = cyber.Node("mock_routing_requester")
#         sequence_num = 0

#         routing_request = routing_pb2.RoutingRequest()

#         routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
#         routing_request.header.module_name = 'routing_request'
#         routing_request.header.sequence_num = sequence_num
#         sequence_num = sequence_num + 1

        
        
#         ev=None
#         vehicles=carla_world.get_actors().filter('vehicle*')
#         # vehicles=carla_world.get_actors()
#         print("vehicles",vehicles)
#         for vehicle in vehicles:
#             print("vehicle role name",vehicle.attributes['role_name'])
#             # print(".........",vehicle.attributes['role_name'])
#             if vehicle.attributes['role_name']=='ego_vehicle':
#                 ev=vehicle
#                 break
        
#         if ev!=None:
#             loc=ev.get_location()
#         else: 
#             print("-- no ev found")
#             return None
        

#         map=carla_world.get_map()
#         start_wp=map.get_waypoint(loc,project_to_road=True,  lane_type=carla.LaneType.Driving  )
        
#         target_points = carla_world.get_map().get_spawn_points()
#         random_pt=secure_random.choice(target_points) if target_points else print("No spawn points found for destination")
#         # random_pt=target_points[10] if target_points else print("No spawn points found for destination")
        
#         target_wp = carla_world.get_map().get_waypoint(random_pt.location,project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        

#         print("===================waypoint target waypoints is:",target_wp.transform.location.x, target_wp.transform.location.y,"road id is",target_wp.road_id,"lane_id is ", target_wp.lane_id)
#         print("target waypoint road id is:", target_wp.road_id,"target waypoint section id is:", target_wp.section_id,
#               "target waypoint lane id is:", target_wp.lane_id,"target waypoint s value is:", target_wp.s)
        
        
            

        
#         return [[start_wp.transform.location.x, start_wp.transform.location.y, "road_"+str(start_wp.road_id)+"_lane_"+str(start_wp.section_id)+"_"+str(start_wp.lane_id), start_wp.s],
#                 [target_wp.transform.location.x, target_wp.transform.location.y,"road_"+str(target_wp.road_id)+"_lane_"+str(target_wp.section_id)+"_"+str(target_wp.lane_id),target_wp.s]]
        
        

            
#     def return_wp(self):
#         """
#         main function for carla simulator Cyber bridge
#         maintaining the communication client and the CarlaBridge object
#         """


#         try:
#             carla_client = carla.Client(
#                 host="localhost", port=2000
#             )
#             carla_world = carla_client.get_world()
            
#             times=0
#             dst_allocated=False
#             while times <= 500 and dst_allocated==False:
#                 # print("times:",times)
#                 result=self.initialize_bridge(carla_world) 
#                 if result == None:
#                     times+=1
#                 else:
#                     start_pt, dst_pt=result[0],result[1]
#                     print("------------start_pt",start_pt)
#                     print("==============dst_pt",dst_pt)
#                     dst_allocated=True
#                     return start_pt, dst_pt
#         except Exception as e:
#             print(e)
#             print("Setting Route Error")
#             return None,None






# def MockRoutingRequest():
#     """
#     生成路由请求
#     Returns:
#         routing_request: 生成的路由请求对象,如果失败则返回None
#     """
#     try:
            
#         route_setting_node = CarlaCyberBridge()
#         result = route_setting_node.return_wp()
#         start_pt, dst_pt = result[0], result[1]
        
#         if start_pt is None and dst_pt is None:
#             return None
        
#         # cyber.init()
#         node = cyber.Node("mock_routing_requester")
        
        
        
#         # 创建response监听器
#         def on_response(msg):
#             routing_response = msg
#             print("Received routing response")

#         response_reader = node.create_reader(
#             '/apollo/routing_response',
#             routing_pb2.RoutingResponse,
#             on_response
#         )

        
#         sequence_num = 0

#         routing_request = routing_pb2.RoutingRequest()
#         routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
#         routing_request.header.module_name = 'routing_request'
#         routing_request.header.sequence_num = sequence_num

#         # 设置起点
#         waypoint = routing_request.waypoint.add()
#         waypoint.pose.x = start_pt[0]
#         waypoint.pose.y = start_pt[1]
#         waypoint.id = start_pt[2]
#         waypoint.s = start_pt[3]
#         # waypoint.id="vehicle_start_point"
#         # waypoint.s=0.0


#         # 设置终点
#         waypoint = routing_request.waypoint.add()
#         waypoint.pose.x = dst_pt[0]
#         waypoint.pose.y = dst_pt[1]
#         waypoint.id = dst_pt[2]
#         waypoint.s = dst_pt[3]
        
        
#         print("routing_request:", routing_request)
        
        

#         writer = node.create_writer('/apollo/routing_request', routing_pb2.RoutingRequest)
#         time.sleep(2.0)
#         writer.write(routing_request)
        
#          # 等待response
#         timeout = 10
#         start_time = time.time()
#         while not routing_response and time.time() - start_time < timeout:
#             time.sleep(0.1)
        
#         node.shutdown()
#         # cyber.shut_down()
#         return routing_request,routing_response

#     except Exception as e:
#         # [修改点6] 添加错误日志
#         print(f"Error in MockRoutingRequest: {e}", file=sys.stderr)
#         return None
    
# if __name__ == '__main__':
#     MockRoutingRequest()
    
    
    

      
    
def cleanup_and_exit(exit_code=0):
    """清理资源并退出"""
    # 向所有子进程发送终止信号
    try:
        current_process = os.getpid()
        children = os.popen(f'pgrep -P {current_process}').read().splitlines()
        for pid in children:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
    except Exception as e:
        print(f"Error during cleanup: {e}")
    
    # 等待所有子进程结束
    try:
        os.wait()
    except ChildProcessError:
        pass
    
    # 退出程序
    sys.exit(exit_code)

class CarlaCyberBridge(CyberNode):
    def __init__(self):
        super().__init__("route_setting_node")
        self.routing_request = None
        self.routing_response = None
    
    def return_wp(self):
        """
        连接Carla并获取路点信息
        """
        try:
            carla_client = carla.Client("localhost", 2000)
            carla_world = carla_client.get_world()
            
            times = 0
            dst_allocated = False
            while times <= 500 and dst_allocated == False:
                logger.info(f"Attempt {times} to get waypoints")
                result = self.initialize_bridge(carla_world) 
                if result == None:
                    times += 1
                    time.sleep(0.1)  # 添加短暂延时
                else:
                    start_pt, dst_pt = result[0], result[1]
                    logger.info(f"Start point: {start_pt}")
                    logger.info(f"Destination point: {dst_pt}")
                    dst_allocated = True
                    return start_pt, dst_pt
                    
            logger.error(f"Failed to get waypoints after {times} attempts")
            return None, None
                
        except Exception as e:
            logger.error(f"Error in return_wp: {e}")
            return None, None
    
    def initialize_bridge(self, carla_world):
        """
        Initialize the bridge
        """
        try:
            # cyber.init()
            # node = cyber.Node("mock_routing_requester")
            # sequence_num = 0

            # routing_request = routing_pb2.RoutingRequest()

            # routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
            # routing_request.header.module_name = 'routing_request'
            # routing_request.header.sequence_num = sequence_num
            # sequence_num = sequence_num + 1

        
            
            ev=None
            vehicles=carla_world.get_actors().filter('vehicle*')
            # vehicles=carla_world.get_actors()
            print("----vehicles",vehicles)
            print("carla world is:", carla_world)
            for vehicle in vehicles:
                print("vehicle role name",vehicle.attributes['role_name'])
                # print(".........",vehicle.attributes['role_name'])
                if vehicle.attributes['role_name']=='ego_vehicle':
                    ev=vehicle
                    
                    break
            
            if ev!=None:
                loc=ev.get_location()
                print("00000 loc is", loc)
                print("=========The loc of EV is:", loc.x, loc.y, loc.z)
            
            else: 
                print("-- no ev found")
                return None
            

            map=carla_world.get_map()
            start_wp=map.get_waypoint(loc, project_to_road=False)
            
            print("========start waypoints is:",start_wp.transform.location.x, start_wp.transform.location.y,"road id is",start_wp.road_id,"lane_id is ", start_wp.lane_id, "s is", start_wp.s)

            # Calculate the Euclidean distance between `loc` and `start_wp`
            # wp_loc = start_wp.transform.location
            # distance = ((loc.x - wp_loc.x) ** 2 + (loc.y - wp_loc.y) ** 2 + (loc.z - wp_loc.z) ** 2) ** 0.5
            # print(f"The distance between loc and start_wp is: {distance} meters")
            target_points = carla_world.get_map().get_spawn_points()
            random_pt=secure_random.choice(target_points) if target_points else print("No spawn points found for destination")
            # random_pt=target_points[10] if target_points else print("No spawn points found for destination")
            
            #TODO: 这个随后或许可以提出来到config 中作为一个可配置参数进行设置            
            MIN_DISTANCE = 50  # 最小距离阈值（米）
            MAX_TRIES = 5      # 最大随机尝试次数

            # 先随机尝试几次
            for _ in range(MAX_TRIES):
                pt = secure_random.choice(target_points)
                distance = ((pt.location.x - loc.x) ** 2 + 
                            (pt.location.y - loc.y) ** 2) ** 0.5
                if distance >= MIN_DISTANCE:
                    random_pt = pt
                    break
            else:  # 如果随机尝试都失败了
                print("Random attempts failed to find far point, searching all points...")
                # 遍历寻找最远点
                max_distance = 0
                random_pt = target_points[0]
                for pt in target_points:
                    distance = ((pt.location.x - loc.x) ** 2 + 
                            (pt.location.y - loc.y) ** 2) ** 0.5
                    if distance > max_distance:
                        max_distance = distance
                        random_pt = pt

            target_wp = carla_world.get_map().get_waypoint(random_pt.location, project_to_road=False)
                        
                    
            
            #what is the s value
            # print("===================waypoint target waypoints is:",target_wp.transform.location.x, target_wp.transform.location.y,"road id is",target_wp.road_id,"lane_id is ", target_wp.lane_id)
            # print("target waypoint road id is:", target_wp.road_id,"target waypoint section id is:", target_wp.section_id,
                # "target waypoint lane id is:", target_wp.lane_id,"target waypoint s value is:", target_wp.s)
       
            return [[start_wp.transform.location.x, start_wp.transform.location.y, "road_"+str(start_wp.road_id)+"_lane_"+str(start_wp.section_id)+"_"+str(start_wp.lane_id), start_wp.s],
                    [target_wp.transform.location.x, target_wp.transform.location.y,"road_"+str(target_wp.road_id)+"_lane_"+str(target_wp.section_id)+"_"+str(target_wp.lane_id),target_wp.s]]
        
        except Exception as e:
            logger.error(f"Error in initialize_bridge: {e}")
            return None, None


    def generate_routing(self):
        """生成新的routing request并监听response"""
        try:
            result = self.return_wp()
            start_pt, dst_pt = result[0], result[1]
            
            if start_pt is None or dst_pt is None:
                return None, None
            
            # 初始化cyber node
            cyber.init()
            node = cyber.Node("routing_generator")
            routing_response = None
            
            def on_response(msg):
                nonlocal routing_response
                routing_response = msg
                logger.info("Received routing response")

            # 创建response reader
            response_reader = node.create_reader(
                '/apollo/routing_response',
                routing_pb2.RoutingResponse,
                on_response
            )
            
            # 创建request writer
            writer = node.create_writer(
                '/apollo/routing_request', 
                routing_pb2.RoutingRequest
            )
            
            # 创建routing request
            sequence_num = 0
            self.routing_request = routing_pb2.RoutingRequest()
            self.routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
            self.routing_request.header.module_name = 'routing_request'
            self.routing_request.header.sequence_num = sequence_num
            sequence_num += 1
            
            # 设置起点
            waypoint = self.routing_request.waypoint.add()
            waypoint.pose.x = start_pt[0]
            waypoint.pose.y = start_pt[1]
            waypoint.id = start_pt[2]
            # waypoint.s = start_pt[3]
            waypoint.s = 0
            #Wei: the s makes the start point far away from ego vehicle. set it to 0

            # 设置终点
            waypoint = self.routing_request.waypoint.add()
            waypoint.pose.x = dst_pt[0]
            waypoint.pose.y = dst_pt[1]
            waypoint.id = dst_pt[2]
            waypoint.s = dst_pt[3]
            
            # 发送request并等待response
            time.sleep(2.0)  # 等待reader和writer准备好
            writer.write(self.routing_request)
            
            # 等待response
            timeout = 10
            start_time = time.time()
            while not routing_response and time.time() - start_time < timeout:
                time.sleep(0.1)
            
            print("==========================================================")
            print("----------------------------------------------------------")
            print("**************inside the generate routing function*********")
            print("==========================================================")
            print("----------------------------------------------------------")
            print("the routing request is:",self.routing_request )
            if routing_response:
                
                print("the routing response is:",routing_response )
                
                self.routing_response = routing_response
                logger.info("Successfully received routing response")
                        # 释放资源
                print("print-Successfully received routing response")
                return self.routing_request, self.routing_response

            else:
                logger.error("Timeout waiting for routing response")
                return None, None
                
        except Exception as e:
            logger.error(f"Error generating routing: {e}")
            return None, None

    def replay_routing(self, request_file: str, response_file: str):
        """重放保存的routing信息"""
        try:   
            
            print("==============================================")
            print("====Replay mode routing is :==================")
            print("==============================================")
            print("request file path is", request_file)
            print("response file path is", response_file)
            print("++++++++++++++++++++++++++++++++++")
            
            
            
            if not os.path.exists(request_file) or not os.path.exists(response_file):
                logger.error("Routing files not found")
                return None, None
                
            self.routing_request = routing_pb2.RoutingRequest()
            self.routing_response = routing_pb2.RoutingResponse()
            
            # 读取request和response
            with open(request_file, 'rb') as f:
                self.routing_request.ParseFromString(f.read())
            with open(response_file, 'rb') as f:
                self.routing_response.ParseFromString(f.read())
            
            # 发布routing信息
            cyber.init()
            node = cyber.Node("routing_replayer")
            
            request_writer = node.create_writer(
                '/apollo/routing_request',
                routing_pb2.RoutingRequest
            )
            response_writer = node.create_writer(
                '/apollo/routing_response',
                routing_pb2.RoutingResponse
            )
            
            # 等待writers准备好
            time.sleep(2.0)
            request_writer.write(self.routing_request)
            time.sleep(0.5)
            response_writer.write(self.routing_response)
         
            print("Routing request is:")
            print(self.routing_request)
            print("_--------------------------------------")
            print("Routing response is:")
            print(self.routing_response)
            return self.routing_request, self.routing_response
            
        except Exception as e:
            logger.error(f"Error replaying routing: {e}")
            return None, None
    
    def save_routing_info(self, test_dir: str):
        """保存当前的routing信息"""
        try:
            if not self.routing_request or not self.routing_response:
                logger.error("No routing information to save")
                return False
                
            request_file = os.path.join(test_dir, "routing_request.pb")
            response_file = os.path.join(test_dir, "routing_response.pb")
            
            with open(request_file, 'wb') as f:
                f.write(self.routing_request.SerializeToString())
            with open(response_file, 'wb') as f:
                f.write(self.routing_response.SerializeToString())
                
            logger.info(f"Saved routing info to {test_dir}")
            return True
            
        except Exception as e:
            logger.error(f"Error saving routing info: {e}")
            return False

def main():
    print("________________________enter the routing handler function!!!")
    parser = argparse.ArgumentParser(description='Handle Apollo routing')
    parser.add_argument('--mode', choices=['generate', 'replay'], 
                       default='generate',
                       help='running mode: generate new routing or replay the saved routing')
    parser.add_argument('--test_dir', type=str, required=True,
                       help='saved routng directory')
    args = parser.parse_args()
    
    bridge = CarlaCyberBridge()
    
    if args.mode == 'generate':
            routing_request, routing_response = bridge.generate_routing()
            if routing_request and routing_response:
                bridge.save_routing_info(args.test_dir)
                print("------------------------------------------")
                print("the routing info is saved to :", args.test_dir)
                logger.info("Routing info saved successfully")
                print("print-Routing info saved successfully")
                print("before sys exit 0 in the main function")
                cleanup_and_exit(0)
                
            else:
                logger.error("Failed to generate routing")
                cleanup_and_exit(1)

    else:
        request_file = os.path.join(args.test_dir, "routing_request.pb")
        response_file = os.path.join(args.test_dir, "routing_response.pb")
        routing_request, routing_response = bridge.replay_routing(request_file, response_file)
        if routing_request and routing_response:
            logger.info("Routing replayed successfully")
            cleanup_and_exit(0)

        else:
            logger.error("Failed to replay routing")
            cleanup_and_exit(1)

            
            
    if routing_request is None:
        cleanup_and_exit(0)

    cleanup_and_exit(1)


if __name__ == '__main__':
    main()      


