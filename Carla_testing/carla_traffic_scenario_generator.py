#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""



import random as python_random
import glob
import os
import sys
import time
import json
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
import random
import math
from carla import command


def get_safe_spawn_points(world, number_points_needed):
    """Get safe spawn points for static objects"""
    # Get map
    carla_map = world.get_map()
    
    # Get waypoints for all lanes
    all_waypoints = carla_map.generate_waypoints(2.0)  # 2.0 meter intervals
    
    # Get sidewalk waypoints
    sidewalks = []
    for waypoint in all_waypoints:
        if waypoint.lane_type == carla.LaneType.Sidewalk:
            sidewalks.append(waypoint)
    
    # Generate safe spawn points
    safe_points = []
    for sidewalk in sidewalks:
        # Get sidewalk edge points
        right_vector = sidewalk.transform.get_right_vector()
        # Offset 1 meter towards the outside of the sidewalk
        location = sidewalk.transform.location + carla.Location(
            x=right_vector.x * 1.0,
            y=right_vector.y * 1.0
        )
        # Ensure objects don't float or sink into the ground
        location.z = world.ground_projection(location, 10.0).location.z + 0.1
        
        # Create spawn point
        spawn_point = carla.Transform(location, sidewalk.transform.rotation)
        safe_points.append(spawn_point)
    
    # Randomly select required number of points
    if len(safe_points) < number_points_needed:
        print(f"Warning: Only found {len(safe_points)} safe spawn points")
        return random.sample(safe_points, min(len(safe_points), number_points_needed))
    return random.sample(safe_points, number_points_needed)



def spawn_single_sensor(world, sensor_bp, sensor_id, total_sensors):
    """
    Generate single sensor based on configuration
    Args:
        world: CARLA world object
        sensor_bp: Sensor blueprint
        sensor_id: Sensor ID
        total_sensors: Total number of sensors
    Returns:
        sensor: Generated sensor object
    """
    try:
        # Calculate sensor position
        sensor_id = int(sensor_id)
        total_sensors = int(total_sensors)
        
        # Distribute sensors evenly around vehicle
        angle = (360.0 / total_sensors) * sensor_id
        radius = 2.0  # Sensor distribution radius
        
        # Calculate sensor position
        x = radius * math.cos(math.radians(angle))
        y = radius * math.sin(math.radians(angle))
        z = 2.0  # Sensor height
        
        # Create sensor transform
        transform = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=-15.0, yaw=angle, roll=0.0)  # Sensor orientation
        )
        
        # Spawn sensor
        sensor = world.spawn_actor(sensor_bp, transform)
        print(f"Successfully spawned sensor {sensor_id}: {sensor.type_id} at {transform}")
        
        return sensor
        
    except Exception as e:
        print(f"Failed to spawn sensor {sensor_id}: {e}")
        return None
    
    
def vector_magnitude(vector):
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def vector_magnitude(vector):
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)



def setup_vehicle_behavior(vehicles_list, traffic_manager, args):
    """Set behavior for randomly selected vehicles"""
    num_vehicles = len(vehicles_list)
    
    # Calculate number of vehicles to set behavior for
    num_red_light_runners = int(num_vehicles * args.ignore_lights_percent / 100.0)
    num_lane_changers = int(num_vehicles * args.vehicles_changing_lane_ratio / 100.0)
    

    for vehicle in random.sample(vehicles_list, num_red_light_runners):
        traffic_manager.ignore_lights_percentage(vehicle, args.red_light_running_chance)
    
    # Set random lane change behavior for selected vehicles
    for vehicle in random.sample(vehicles_list, num_lane_changers):
        traffic_manager.random_left_lanechange_percentage(vehicle, args.lane_changing_chance)
        
        
        
def set_weather(world, args):
    """Set weather parameters"""
    weather = world.get_weather()
    
    # Check and set various weather parameters
    if hasattr(args, 'rain_intensity') and args.rain_intensity is not None:
        weather.precipitation = args.rain_intensity
        
    if hasattr(args, 'fog_density') and args.fog_density is not None:
        weather.fog_density = args.fog_density
        
    if hasattr(args, 'wind_intensity') and args.wind_intensity is not None:
        weather.wind_intensity = args.wind_intensity
        
    if hasattr(args, 'sun_altitude') and args.sun_altitude is not None:
        weather.sun_altitude_angle = args.sun_altitude
        
    if hasattr(args, 'humidity') and args.humidity is not None:
        weather.wetness = args.humidity  # wetness in CARLA corresponds to humidity
    
    # Apply weather settings
    world.set_weather(weather)

def spawn_sensors(world, sensor_type, sensor_number):
    """Spawn independent sensors in the world"""
    sensors = []
    sensor_bp = world.get_blueprint_library().find(sensor_type)
    
    # Set specific parameters based on sensor type
    if sensor_type == 'sensor.camera.rgb':
        sensor_bp.set_attribute('image_size_x', '800')
        sensor_bp.set_attribute('image_size_y', '600')
        sensor_bp.set_attribute('fov', '90')
    elif sensor_type == 'sensor.lidar.ray_cast':
        sensor_bp.set_attribute('channels', '32')
        sensor_bp.set_attribute('points_per_second', '90000')
        sensor_bp.set_attribute('rotation_frequency', '20')
        sensor_bp.set_attribute('range', '20')
    elif sensor_type == 'sensor.other.radar':
        sensor_bp.set_attribute('horizontal_fov', '30')
        sensor_bp.set_attribute('vertical_fov', '30')
        sensor_bp.set_attribute('range', '100')
    
    # Distribute sensors evenly in the world
    map_bounds = world.get_map().get_spawn_points()
    center_point = sum((p.location for p in map_bounds), carla.Location()) / len(map_bounds)
    radius = 50  # Sensor distribution radius
    
    for i in range(sensor_number):
        # Calculate sensor position, distribute evenly around circle
        angle = (2 * math.pi * i) / sensor_number
        x = center_point.x + radius * math.cos(angle)
        y = center_point.y + radius * math.sin(angle)
        z = 10  # Fixed height
        
        # Create sensor position and orientation
        sensor_location = carla.Location(x=x, y=y, z=z)
        # Orient sensor towards center
        direction = center_point - sensor_location
        sensor_rotation = carla.Rotation(
            pitch=-15,  # Slightly downward
            yaw=math.degrees(math.atan2(direction.y, direction.x))
        )
        
        sensor_transform = carla.Transform(sensor_location, sensor_rotation)
        sensor = world.spawn_actor(sensor_bp, sensor_transform)
        sensors.append(sensor)
        print(f"Spawned {sensor_type} at position {sensor_location}")
    
    return sensors

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument('--tm-port', type=int, default=8000, help='Port to communicate with TM')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')


    # Weather parameters
    argparser.add_argument('--weather-preset', type=str, help='Weather preset')
    argparser.add_argument('--rain-intensity', type=float, default=0.0, help='Rain intensity')
    argparser.add_argument('--fog-density', type=float, default=0.0, help='Fog density')
    argparser.add_argument('--wind-intensity', type=float, default=0.0, help='Wind intensity')
    argparser.add_argument('--sun-altitude', type=float, default=45.0, help='Sun altitude angle')
    argparser.add_argument('--humidity', type=float, default=50.0, help='Humidity')
    
    # Traffic Manager parameters
    argparser.add_argument('--speed-difference', type=float, default=30, 
                          help='Global speed difference percentage')
    argparser.add_argument('--lane-offset', type=float, default=0, help='Global lane offset')
    
    argparser.add_argument(
        '--red-light-running-chance', 
        type=float, 
        default=0.0,
        help='Chance for selected vehicles to actually run a red light'
    )
    argparser.add_argument(
        '--vehicles-changing-lane-ratio', 
        type=float, 
        default=0.0,
        help='Percentage of vehicles that will be set to potentially change lanes'
    )
    argparser.add_argument(
        '--lane-changing-chance', 
        type=float, 
        default=0.0,
        help='Chance for selected vehicles to perform random lane changes'
    ) 
    argparser.add_argument(
        '--ignore-lights-percent', 
        type=float, 
        default=0.0,
        help='Percentage of vehicles that will ignore traffic lights'
    )
    
    argparser.add_argument(
        '--sensor-type',
        type=str,
        default='sensor.camera.rgb',
        help='Type of sensor to attach to vehicles'
    )
    argparser.add_argument(
        '--sensor-number',
        type=int,
        default=0,
        help='Number of sensors to attach to each vehicle'
    )
    
    
    argparser.add_argument(
        '--record-path',
        type=str,
        help='Path to save recorded vehicle trajectories and controls'
    )
    

    argparser.add_argument(
        '-f', '--recorder_filename',
        metavar='F',
        default="test1.log",
        help='recorder filename (test1.log)')
    
    
    
    
    
    
    # New added
    argparser.add_argument(
        '--town',
        type=str,
        default='Town01',
        help='Town to load (Town01-Town07, Town10)'
    )

    # Static object related parameters
    argparser.add_argument(
        '--static-object-type',
        type=str,
        default='static.prop.barrel',
        help='Type of static object to spawn'
    )
    argparser.add_argument(
        '--static-object-number',
        type=int,
        default=0,
        help='Number of static objects to spawn'
    )

    # Sensor configuration related parameters
    argparser.add_argument(
        '--sensor-configs',
        type=str,
        help='JSON string containing sensor configurations'
    )
    
    

    
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    python_random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        if args.town:
            client.load_world(args.town)
            world = client.get_world()
            print(f"Loaded world: {args.town}")
        

        set_weather(world, args)

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5) 
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            print("=====================set hybrid physics mode=====================")
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(2000.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        
        if args.lane_offset is not None:
            traffic_manager.global_lane_offset(args.lane_offset)
        if args.speed_difference is not None:
            traffic_manager.global_percentage_speed_difference(args.speed_difference) 


        print("after setup vehicle behavior")
        settings = world.get_settings()

        print("before apply settings")
        if args.no_rendering:
            settings.no_rendering_mode = True
        print("after no rendering")
        synchronous_master = True
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        traffic_manager.set_synchronous_mode(True)
        print("after set synchronous mode")
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        print("before apply settings")
        world.apply_settings(settings)
        print("after apply settings")
        
        
        
        
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            python_random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        print("before spawn vehicles")
        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break

            # blueprint = random.choice(blueprints)
            if args.filterv is not None:
                print("filterv is:",args.filterv)
                print("search result is:",world.get_blueprint_library().filter(args.filterv))
                blueprint = world.get_blueprint_library().filter(args.filterv)[0]
            else:
                blueprint = world.get_blueprint_library().filter("vehicle.lincoln.mkz_2017")[0]
            
            if blueprint.has_attribute('color'):
                color = python_random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = python_random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

            # batch.append(SpawnActor(blueprint, transform)
            #     .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
            #     .then(command.SetSimulatePhysics(FutureActor, True))) 
            
        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            all_vehicle_actors = world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        print("after spawn vehicles")



        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        if args.seedw:
            world.set_pedestrians_seed(args.seedw)
            python_random.seed(args.seedw)
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = python_random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (python_random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()
        print("after spawn walkers")

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        print("before spawn static objects")
        ##### Generate static objects in positions that don't affect vehicles and pedestrians


        # Use in main code:
        if args.static_object_number > 0:
            try:
                static_objects = []
                blueprint = world.get_blueprint_library().find(args.static_object_type)
                safe_spawn_points = get_safe_spawn_points(world, args.static_object_number)
                
                for spawn_point in safe_spawn_points:
                    try:
                        static_object = world.spawn_actor(blueprint, spawn_point)
                        static_objects.append(static_object)
                        print(f"Spawned static object {args.static_object_type} at {spawn_point.location}")
                    except Exception as spawn_error:
                        print(f"Failed to spawn static object: {spawn_error}")
                        continue
                        
            except Exception as e:
                print(f"Error spawning static objects: {e}")


        print("after spawn static objects")
        
        if args.sensor_type is not None and args.sensor_number is not None:
            print(f"\nSpawning {args.sensor_number} {args.sensor_type}s in the world")
            
            # If sensor_configs parameter exists, use configuration information
            if hasattr(args, 'sensor_configs') and args.sensor_configs:
                try:
                    sensor_configs = json.loads(args.sensor_configs)
                    print(f"Applying sensor configurations: {sensor_configs}")
                    
                    all_sensors = []
                    for config in sensor_configs:
                        sensor_bp = world.get_blueprint_library().find(config['type'])
                        
                        # Set sensor parameters
                        if 'tick' in config and config['tick'] is not None:
                            print(f"Setting sensor tick to {config['tick']}")
                            sensor_bp.set_attribute('sensor_tick', str(config['tick']))
                        
                        # Set LiDAR specific parameters
                        if 'lidar' in config['type']:
                            if 'channels' in config:
                                print(f"Setting LiDAR channels to {config['channels']}")
                                sensor_bp.set_attribute('channels', str(config['channels']))
                            if 'points_per_second' in config:
                                print(f"Setting LiDAR points_per_second to {config['points_per_second']}")
                                sensor_bp.set_attribute('points_per_second', str(config['points_per_second']))
                            if 'rotation_frequency' in config:
                                print(f"Setting LiDAR rotation_frequency to {config['rotation_frequency']}")
                                sensor_bp.set_attribute('rotation_frequency', str(config['rotation_frequency']))
                        
                        # Use existing spawn_sensors function position logic
                        sensor = spawn_single_sensor(world, sensor_bp, config['id'], args.sensor_number)
                        if sensor:
                            all_sensors.append(sensor)
                            
                except json.JSONDecodeError as e:
                    print(f"Error parsing sensor configurations: {e}")
                    # If configuration parsing fails, use default settings
                    all_sensors = spawn_sensors(world, args.sensor_type, args.sensor_number)
            else:
                # If no configuration information, use default settings
                all_sensors = spawn_sensors(world, args.sensor_type, args.sensor_number)
        print("after spawn sensors")
        
        # Get all vehicles
        all_vehicles = world.get_actors(vehicles_list)
        
        # Set spectator and select vehicle to monitor
        spectator = world.get_spectator()
        spectator_vehicle = python_random.choice(all_vehicles)
        # spectator_vehicle.set_attribute('role_name', 'hero')
        print(f"Following vehicle ID: {spectator_vehicle.id}")


        # Select vehicles to monitor (including vehicle followed by spectator)
        num_vehicles_to_monitor = 7
        monitored_actors = [spectator_vehicle]  # Ensure spectator-followed vehicle is monitored
        
        # Randomly select from remaining vehicles until reaching specified number
        remaining_vehicles = [v for v in all_vehicles if v.id != spectator_vehicle.id]
        if len(remaining_vehicles) > num_vehicles_to_monitor - 1:  # -1 because spectator vehicle is already included
            monitored_actors.extend(python_random.sample(remaining_vehicles, num_vehicles_to_monitor - 1))
        else:
            monitored_actors.extend(remaining_vehicles)  # If not enough remaining vehicles, use all

        print(f"Monitoring {len(monitored_actors)} vehicles with IDs: {[v.id for v in monitored_actors]}")

        frame_data = {}  # Use frame as key to store data
        last_platform_timestamp=None
        
        # start recorder
        # print("Recording on file: %s" % client.start_recorder(args.recorder_filename))
        
        
        
        # while True:
        for i in range(1000):
        
            if not args.asynch and synchronous_master:
                # print("----sync sync")
                if spectator_vehicle:
                    transform = spectator_vehicle.get_transform()
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=30),carla.Rotation(pitch=-90)))
                    
                
                snapshot = world.get_snapshot()
                frame = snapshot.frame
                # print("elapsed time: ",snapshot.timestamp.delta_seconds)
                timestamp = snapshot.timestamp.elapsed_seconds
                platform_timestamp = snapshot.timestamp.platform_timestamp
                # print("frame is:",frame )                
                platform_delta_time = 0.0
                if last_platform_timestamp is not None:
                    platform_delta_time = platform_timestamp - last_platform_timestamp
                last_platform_timestamp = platform_timestamp

                # 为当前frame创建数据列表
                frame_data[frame] = {
                    "timestamp": timestamp,
                    "delta_time": snapshot.timestamp.delta_seconds,
                    "platform_timestamp": platform_timestamp,
                    "platform_delta_time": platform_delta_time,
                    "vehicles": []
                }
                #wei: directly use the timestamp to calculate the error distribution


                for vehicle in monitored_actors:
                    transform = vehicle.get_transform()
                    control = vehicle.get_control()
                    velocity = vehicle.get_velocity()
                    angular_velocity = vehicle.get_angular_velocity()
                    acceleration = vehicle.get_acceleration()

                    # physical_control=vehicle.get_physics_control()


                    vehicle_data = {
                        'id': vehicle.id,
                        'type': vehicle.type_id,
                        # 'timestamp':timestamp,
                        # 'delta_time':snapshot.timestamp.delta_seconds,
                        # 'platform_timestamp':platform_timestamp,
                        'x': transform.location.x,
                        'y': transform.location.y,
                        'z': transform.location.z,
                        'yaw': transform.rotation.yaw,
                        'roll': transform.rotation.roll,
                        'pitch': transform.rotation.pitch,
                        'throttle': control.throttle,
                        'steer': control.steer,
                        'brake': control.brake,
                        'gear': control.gear,
                        'hand_brake': control.hand_brake,
                        'reverse': control.reverse,
                        'manual_gear_shift': control.manual_gear_shift,
                        'velocity': {
                            'x': velocity.x,
                            'y': velocity.y,
                            'z': velocity.z,
                            'magnitude': vector_magnitude(velocity)
                        },
                        'angular_velocity': {
                            'x': angular_velocity.x,
                            'y': angular_velocity.y,
                            'z': angular_velocity.z,
                            'magnitude': vector_magnitude(angular_velocity)
                        },
                        'acceleration': {
                            'x': acceleration.x,
                            'y': acceleration.y,
                            'z': acceleration.z,
                            'magnitude': vector_magnitude(acceleration)
                        }
                    }
                    frame_data[frame]["vehicles"].append(vehicle_data)

                world.tick()
            else:
                world.wait_for_tick()

    finally:
        
        # client.stop_recorder()

        if not args.asynch and synchronous_master:
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        # 销毁spectator
        # if spectator:
        #     spectator.destroy()
        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
            
                    # 如果提供了record-path，保存轨迹数据
        if args.record_path:
            with open(args.record_path, 'w') as f:
                # json.dump(frame_data, f, indent=4)
                json.dump(frame_data, f, indent=4, ensure_ascii=False, separators=(',', ':'))
            
        time.sleep(0.5)

        # 清理传感器
        if all_sensors:
            print(f'\nDestroying {len(all_sensors)} sensors')
            for sensor in all_sensors:
                sensor.destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
