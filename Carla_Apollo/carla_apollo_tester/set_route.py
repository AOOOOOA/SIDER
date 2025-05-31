#!/usr/bin/env python
#
# Copyright (c) 2023 synkrotron.ai
#

import os
import sys
import threading
from threading import Event, Thread
sys.path.append("../")

from cyber.python.cyber_py3 import cyber_time
from carla_bridge.core.node import CyberNode

import carla
import yaml

# to generate a random spawning position or vehicles
import random
secure_random = random.SystemRandom()


import argparse
import os
import sys
import time

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.routing_msgs import routing_pb2


class CarlaCyberBridge(CyberNode):

    # in synchronous mode, if synchronous_mode_wait_for_vehicle_control_command is True,
    # wait for this time until a next tick is triggered.
    VEHICLE_CONTROL_TIMEOUT = 1.0

    def __init__(self):
        super().__init__("route_setting_node")
    
    def initialize_bridge(self,carla_world):

        
        cyber.init()
        node = cyber.Node("mock_routing_requester")
        sequence_num = 0

        routing_request = routing_pb2.RoutingRequest()

        routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
        routing_request.header.module_name = 'routing_request'
        routing_request.header.sequence_num = sequence_num
        sequence_num = sequence_num + 1

        
        
        #TODO: change the starting points to the current generated points
        ev=None
        vehicles=carla_world.get_actors().filter('vehicle*')
        # vehicles=carla_world.get_actors()
        print("vehicles",vehicles)
        for vehicle in vehicles:
            print("vehicle role name",vehicle.attributes['role_name'])
            # print(".........",vehicle.attributes['role_name'])
            if vehicle.attributes['role_name']=='ego_vehicle':
                ev=vehicle
                break
        if ev!=None:
            loc=ev.get_location()
        else: 
            print("-- no ev found")
            return None
            
        # target_points = carla_world.get_map().get_spawn_points()
        # transform=secure_random.choice(target_points) if target_points else print("No spawn points found for destination")
        map=carla_world.get_map()
        start_loc=loc

        start_wp=map.get_waypoint(start_loc)
        # print("start waypoint is:",start_wp,"start waypoint id is:", start_wp.id,"start waypoint road id is:", start_wp.road_id, 
            #   "start waypoint section id is:", start_wp.section_id,"start waypoint junction id is:", start_wp.junction_id,
            #   "start waypoint lane id is:", start_wp.lane_id,"start waypoint opendrive s-value is:", start_wp.s)
        print("---------------waypoint target waypoints is:",start_loc.x, start_loc.y,"road id is",start_wp.road_id,"lane_id is ", start_wp.lane_id)
        print("start waypoint road id is:", start_wp.road_id,"start waypoint section id is:", start_wp.section_id,
              "start waypoint lane id is:", start_wp.lane_id,"start waypoint s value is:", start_wp.s)

        
        
        #may need to request the position of the ev right now
        
        target_points = carla_world.get_map().get_spawn_points()
        random_pt=secure_random.choice(target_points) if target_points else print("No spawn points found for destination")
        # random_pt=target_points[10] if target_points else print("No spawn points found for destination")
        
        target_wp = carla_world.get_map().get_waypoint(random_pt.location,project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        

        print("===================waypoint target waypoints is:",target_wp.transform.location.x, target_wp.transform.location.y,"road id is",target_wp.road_id,"lane_id is ", target_wp.lane_id)
        print("target waypoint road id is:", target_wp.road_id,"target waypoint section id is:", target_wp.section_id,
              "target waypoint lane id is:", target_wp.lane_id,"target waypoint s value is:", target_wp.s)
        
        
            
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x =  start_loc.x
        waypoint.pose.y = start_loc.y
        # waypoint.id = '0-1'
        waypoint.id="road_"+str(start_wp.road_id)+"_lane_"+str(start_wp.section_id)+"_"+str(start_wp.lane_id)
        waypoint.s =  start_wp.s



        waypoint = routing_request.waypoint.add()

        waypoint.pose.x =  target_wp.transform.location.x
        waypoint.pose.y = target_wp.transform.location.y
        # waypoint.id = '0-1'
        waypoint.id="road_"+str(target_wp.road_id)+"_lane_"+str(target_wp.section_id)+"_"+str(target_wp.lane_id)
        waypoint.s =  target_wp.s


        writer = node.create_writer('/apollo/raw_routing_request',
                                    routing_pb2.RoutingRequest)
        time.sleep(2.0)
        print("routing_request", routing_request)
        writer.write(routing_request)

        
        return [[start_loc.x, start_loc.y, "road_"+str(start_wp.road_id)+"_lane_"+str(start_wp.section_id)+"_"+str(start_wp.lane_id), start_wp.s],
                [target_wp.transform.location.x, target_wp.transform.location.y,"road_"+str(target_wp.road_id)+"_lane_"+str(target_wp.section_id)+"_"+str(target_wp.lane_id),target_wp.s]]
        
            
    def return_wp(self):
        """
        main function for carla simulator Cyber bridge
        maintaining the communication client and the CarlaBridge object
        """


        try:
            # config_file =  "../config/settings.yaml"
            # with open(config_file, encoding="utf-8") as f:
                # parameters = yaml.safe_load(f)
            # carla_parameters = parameters["carla"]

            carla_client = carla.Client(
                host="localhost", port=2000
            )
            # carla_client.set_timeout(carla_parameters["timeout"])

            # carla_client.load_world(carla_parameters["town"])
            carla_world = carla_client.get_world()
            print("88888888",carla_world)
            
            
            # carla_bridge = CarlaCyberBridge()
            
            times=0
            dst_allocated=False
            while times <= 500 and dst_allocated==False:
                # print("times:",times)
                result=self.initialize_bridge(carla_world) 
                if result == None:
                    times+=1
                else:
                    start_pt, dst_pt=result[0],result[1]
                    print("------------start_pt",start_pt)
                    print("==============dst_pt",dst_pt)
                    dst_allocated=True
                    return start_pt, dst_pt
        except Exception as e:
            print(e)
            print("Setting Route Error")
            return None,None

