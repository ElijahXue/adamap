#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import glob
import os
import sys
import time
from queue import Queue
from queue import Empty
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
from numpy import random

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
        default=1000,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=20,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        default=False,
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
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        default=False,
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

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(10)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)


        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
            blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)


        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
       
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            

            # spawn the cars and set their autopilot and light state all together
            ego_vehicle = world.spawn_actor(blueprint, transform)
       
            vehicles_list.append(ego_vehicle)
            # batch.append(SpawnActor(blueprint, transform)
            #     .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))
        blueprint_library = world.get_blueprint_library()
        lidarlist = []
        for i in range(len(spawn_points)):
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', str(64))      
            lidar_bp.set_attribute('points_per_second', str(112000))
            lidar_bp.set_attribute('rotation_frequency', str(10))
            lidar_bp.set_attribute('range', str(20))
            lidarlist.append(lidar_bp)
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        activate_lidar_list = []
        for i in range(len(spawn_points)):
            lidar = world.spawn_actor(lidarlist[i], lidar_transform, attach_to=vehicles_list[i])
            activate_lidar_list.append(lidar)
        output_path = './poutputs'
        def sensor_callback(sensor_data, sensor_queue, sensor_name):
            sensor_queue.put((sensor_data.frame, sensor_data.timestamp,sensor_data.transform, sensor_name))
            sensor_data.save_to_disk(os.path.join(output_path, '%s%06d.ply' %  (sensor_name,sensor_data.frame)))
        sensor_queue = Queue()
        sensor_list = []
 
        # activate_lidar_list[0].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[0])))
        # sensor_list.append(activate_lidar_list[0])
        # activate_lidar_list[1].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[1])))
        # sensor_list.append(activate_lidar_list[1])
        # activate_lidar_list[2].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[2])))
        # sensor_list.append(activate_lidar_list[2])
        # activate_lidar_list[3].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[3])))
        # sensor_list.append(activate_lidar_list[3])
        # activate_lidar_list[4].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[4])))
        # sensor_list.append(activate_lidar_list[4])
        # activate_lidar_list[5].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[5])))
        # sensor_list.append(activate_lidar_list[5])
        # activate_lidar_list[6].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[6])))
        # sensor_list.append(activate_lidar_list[6])
        # activate_lidar_list[7].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[7])))
        # sensor_list.append(activate_lidar_list[7])
        # activate_lidar_list[8].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[8])))
        # sensor_list.append(activate_lidar_list[8])
        # activate_lidar_list[9].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[9])))
        # sensor_list.append(activate_lidar_list[9])
        # activate_lidar_list[10].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[10])))
        # sensor_list.append(activate_lidar_list[10])
        # activate_lidar_list[11].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[11])))
        # sensor_list.append(activate_lidar_list[11])
        # activate_lidar_list[12].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[12])))
        # sensor_list.append(activate_lidar_list[12])
        # activate_lidar_list[13].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[13])))
        # sensor_list.append(activate_lidar_list[13])
        # activate_lidar_list[14].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[14])))
        # sensor_list.append(activate_lidar_list[14])
        # activate_lidar_list[15].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[15])))
        # sensor_list.append(activate_lidar_list[15])
        # activate_lidar_list[16].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[16])))
        # sensor_list.append(activate_lidar_list[16])
        # activate_lidar_list[17].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[17])))
        # sensor_list.append(activate_lidar_list[17])
        # activate_lidar_list[18].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[18])))
        # sensor_list.append(activate_lidar_list[18])
        # activate_lidar_list[19].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[19])))
        # sensor_list.append(activate_lidar_list[19])
        # activate_lidar_list[20].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[20])))
        # sensor_list.append(activate_lidar_list[20])
        # activate_lidar_list[21].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[21])))
        # sensor_list.append(activate_lidar_list[21])
        # activate_lidar_list[22].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[22])))
        # sensor_list.append(activate_lidar_list[22])
        # activate_lidar_list[23].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[23])))
        # sensor_list.append(activate_lidar_list[23])
        # activate_lidar_list[24].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[24])))
        # sensor_list.append(activate_lidar_list[24])
        # activate_lidar_list[25].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[25])))
        # sensor_list.append(activate_lidar_list[25])
        # activate_lidar_list[26].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[26])))
        # sensor_list.append(activate_lidar_list[26])
        # activate_lidar_list[27].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[27])))
        # sensor_list.append(activate_lidar_list[27])
        # activate_lidar_list[28].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[28])))
        # sensor_list.append(activate_lidar_list[28])
        # activate_lidar_list[29].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[29])))
        # sensor_list.append(activate_lidar_list[29])
        # activate_lidar_list[30].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[30])))
        # sensor_list.append(activate_lidar_list[30])
        # activate_lidar_list[31].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[31])))
        # sensor_list.append(activate_lidar_list[31])
        # activate_lidar_list[32].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[32])))
        # sensor_list.append(activate_lidar_list[32])
        # activate_lidar_list[33].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[33])))
        # sensor_list.append(activate_lidar_list[33])
        # activate_lidar_list[34].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[34])))
        # sensor_list.append(activate_lidar_list[34])
        # activate_lidar_list[35].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[35])))
        # sensor_list.append(activate_lidar_list[35])
        # activate_lidar_list[36].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[36])))
        # sensor_list.append(activate_lidar_list[36])
        # activate_lidar_list[37].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[37])))
        # sensor_list.append(activate_lidar_list[37])
        # activate_lidar_list[38].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[38])))
        # sensor_list.append(activate_lidar_list[38])
        # activate_lidar_list[39].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[39])))
        # sensor_list.append(activate_lidar_list[39])
        # activate_lidar_list[40].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[40])))
        # sensor_list.append(activate_lidar_list[40])
        # activate_lidar_list[41].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[41])))
        # sensor_list.append(activate_lidar_list[41])
        # activate_lidar_list[42].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[42])))
        # sensor_list.append(activate_lidar_list[42])
        # activate_lidar_list[43].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[43])))
        # sensor_list.append(activate_lidar_list[43])
        # activate_lidar_list[44].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[44])))
        # sensor_list.append(activate_lidar_list[44])
        # activate_lidar_list[45].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[45])))
        # sensor_list.append(activate_lidar_list[45])
        # activate_lidar_list[46].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[46])))
        # sensor_list.append(activate_lidar_list[46])
        # activate_lidar_list[47].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[47])))
        # sensor_list.append(activate_lidar_list[47])
        # activate_lidar_list[48].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[48])))
        # sensor_list.append(activate_lidar_list[48])
        # activate_lidar_list[49].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[49])))
        # sensor_list.append(activate_lidar_list[49])
        # activate_lidar_list[50].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[50])))
        # sensor_list.append(activate_lidar_list[50])
        # activate_lidar_list[51].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[51])))
        # sensor_list.append(activate_lidar_list[51])
        # activate_lidar_list[52].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[52])))
        # sensor_list.append(activate_lidar_list[52])
        # activate_lidar_list[53].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[53])))
        # sensor_list.append(activate_lidar_list[53])
        # activate_lidar_list[54].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[54])))
        # sensor_list.append(activate_lidar_list[54])
        # activate_lidar_list[55].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[55])))
        # sensor_list.append(activate_lidar_list[55])
        # activate_lidar_list[56].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[56])))
        # sensor_list.append(activate_lidar_list[56])
        # activate_lidar_list[57].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[57])))
        # sensor_list.append(activate_lidar_list[57])
        # activate_lidar_list[58].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[58])))
        # sensor_list.append(activate_lidar_list[58])
        # activate_lidar_list[59].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[59])))
        # sensor_list.append(activate_lidar_list[59])
        # activate_lidar_list[60].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[60])))
        # sensor_list.append(activate_lidar_list[60])
        # activate_lidar_list[61].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[61])))
        # sensor_list.append(activate_lidar_list[61])
        # activate_lidar_list[62].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[62])))
        # sensor_list.append(activate_lidar_list[62])
        # activate_lidar_list[63].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[63])))
        # sensor_list.append(activate_lidar_list[63])
        # activate_lidar_list[64].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[64])))
        # sensor_list.append(activate_lidar_list[64])
        # activate_lidar_list[65].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[65])))
        # sensor_list.append(activate_lidar_list[65])
        # activate_lidar_list[66].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[66])))
        # sensor_list.append(activate_lidar_list[66])
        # activate_lidar_list[67].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[67])))
        # sensor_list.append(activate_lidar_list[67])
        # activate_lidar_list[68].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[68])))
        # sensor_list.append(activate_lidar_list[68])
        # activate_lidar_list[69].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[69])))
        # sensor_list.append(activate_lidar_list[69])
        # activate_lidar_list[70].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[70])))
        # sensor_list.append(activate_lidar_list[70])
        # activate_lidar_list[71].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[71])))
        # sensor_list.append(activate_lidar_list[71])
        # activate_lidar_list[72].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[72])))
        # sensor_list.append(activate_lidar_list[72])
        # activate_lidar_list[73].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[73])))
        # sensor_list.append(activate_lidar_list[73])
        # activate_lidar_list[74].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[74])))
        # sensor_list.append(activate_lidar_list[74])
        # activate_lidar_list[75].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[75])))
        # sensor_list.append(activate_lidar_list[75])
        # activate_lidar_list[76].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[76])))
        # sensor_list.append(activate_lidar_list[76])
        # activate_lidar_list[77].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[77])))
        # sensor_list.append(activate_lidar_list[77])
        # activate_lidar_list[78].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[78])))
        # sensor_list.append(activate_lidar_list[78])
        # activate_lidar_list[79].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[79])))
        # sensor_list.append(activate_lidar_list[79])
        # activate_lidar_list[80].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[80])))
        # sensor_list.append(activate_lidar_list[80])
        # activate_lidar_list[81].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[81])))
        # sensor_list.append(activate_lidar_list[81])
        # activate_lidar_list[82].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[82])))
        # sensor_list.append(activate_lidar_list[82])
        # activate_lidar_list[83].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[83])))
        # sensor_list.append(activate_lidar_list[83])
        # activate_lidar_list[84].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[84])))
        # sensor_list.append(activate_lidar_list[84])
        # activate_lidar_list[85].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[85])))
        # sensor_list.append(activate_lidar_list[85])
        # activate_lidar_list[86].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[86])))
        # sensor_list.append(activate_lidar_list[86])
        # activate_lidar_list[87].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[87])))
        # sensor_list.append(activate_lidar_list[87])
        # activate_lidar_list[88].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[88])))
        # sensor_list.append(activate_lidar_list[88])
        # activate_lidar_list[89].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[89])))
        # sensor_list.append(activate_lidar_list[89])
        # activate_lidar_list[90].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[90])))
        # sensor_list.append(activate_lidar_list[90])
        # activate_lidar_list[91].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[91])))
        # sensor_list.append(activate_lidar_list[91])
        # activate_lidar_list[92].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[92])))
        # sensor_list.append(activate_lidar_list[92])
        # activate_lidar_list[93].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[93])))
        # sensor_list.append(activate_lidar_list[93])
        # activate_lidar_list[94].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[94])))
        # sensor_list.append(activate_lidar_list[94])
        # activate_lidar_list[95].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[95])))
        # sensor_list.append(activate_lidar_list[95])
        # activate_lidar_list[96].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[96])))
        # sensor_list.append(activate_lidar_list[96])
        # activate_lidar_list[97].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[97])))
        # sensor_list.append(activate_lidar_list[97])
        # activate_lidar_list[98].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[98])))
        # sensor_list.append(activate_lidar_list[98])
        # activate_lidar_list[99].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[99])))
        # sensor_list.append(activate_lidar_list[99])
        # activate_lidar_list[100].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[100])))
        # sensor_list.append(activate_lidar_list[100])
        # activate_lidar_list[101].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[101])))
        # sensor_list.append(activate_lidar_list[101])
        # activate_lidar_list[102].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[102])))
        # sensor_list.append(activate_lidar_list[102])
        # activate_lidar_list[103].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[103])))
        # sensor_list.append(activate_lidar_list[103])
        # activate_lidar_list[104].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[104])))
        # sensor_list.append(activate_lidar_list[104])
        # activate_lidar_list[105].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[105])))
        # sensor_list.append(activate_lidar_list[105])
        # activate_lidar_list[106].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[106])))
        # sensor_list.append(activate_lidar_list[106])
        # activate_lidar_list[107].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[107])))
        # sensor_list.append(activate_lidar_list[107])
        # activate_lidar_list[108].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[108])))
        # sensor_list.append(activate_lidar_list[108])
        # activate_lidar_list[109].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[109])))
        # sensor_list.append(activate_lidar_list[109])
        # activate_lidar_list[110].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[110])))
        # sensor_list.append(activate_lidar_list[110])
        # activate_lidar_list[111].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[111])))
        # sensor_list.append(activate_lidar_list[111])
        # activate_lidar_list[112].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[112])))
        # sensor_list.append(activate_lidar_list[112])
        # activate_lidar_list[113].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[113])))
        # sensor_list.append(activate_lidar_list[113])
        # activate_lidar_list[114].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[114])))
        # sensor_list.append(activate_lidar_list[114])
        # activate_lidar_list[115].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[115])))
        # sensor_list.append(activate_lidar_list[115])
        # activate_lidar_list[116].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[116])))
        # sensor_list.append(activate_lidar_list[116])
        # activate_lidar_list[117].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[117])))
        # sensor_list.append(activate_lidar_list[117])
        # activate_lidar_list[118].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[118])))
        # sensor_list.append(activate_lidar_list[118])
        # activate_lidar_list[119].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[119])))
        # sensor_list.append(activate_lidar_list[119])
        # activate_lidar_list[120].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[120])))
        # sensor_list.append(activate_lidar_list[120])
        # activate_lidar_list[121].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[121])))
        # sensor_list.append(activate_lidar_list[121])
        # activate_lidar_list[122].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[122])))
        # sensor_list.append(activate_lidar_list[122])
        # activate_lidar_list[123].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[123])))
        # sensor_list.append(activate_lidar_list[123])
        # activate_lidar_list[124].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[124])))
        # sensor_list.append(activate_lidar_list[124])
        # activate_lidar_list[125].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[125])))
        # sensor_list.append(activate_lidar_list[125])
        # activate_lidar_list[126].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[126])))
        # sensor_list.append(activate_lidar_list[126])
        # activate_lidar_list[127].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[127])))
        # sensor_list.append(activate_lidar_list[127])
        # activate_lidar_list[128].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[128])))
        # sensor_list.append(activate_lidar_list[128])
        # activate_lidar_list[129].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[129])))
        # sensor_list.append(activate_lidar_list[129])
        # activate_lidar_list[130].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[130])))
        # sensor_list.append(activate_lidar_list[130])
        # activate_lidar_list[131].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[131])))
        # sensor_list.append(activate_lidar_list[131])
        # activate_lidar_list[132].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[132])))
        # sensor_list.append(activate_lidar_list[132])
        # activate_lidar_list[133].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[133])))
        # sensor_list.append(activate_lidar_list[133])
        # activate_lidar_list[134].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[134])))
        # sensor_list.append(activate_lidar_list[134])
        # activate_lidar_list[135].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[135])))
        # sensor_list.append(activate_lidar_list[135])
        # activate_lidar_list[136].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[136])))
        # sensor_list.append(activate_lidar_list[136])
        # activate_lidar_list[137].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[137])))
        # sensor_list.append(activate_lidar_list[137])
        # activate_lidar_list[138].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[138])))
        # sensor_list.append(activate_lidar_list[138])
        # activate_lidar_list[139].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[139])))
        # sensor_list.append(activate_lidar_list[139])
        # activate_lidar_list[140].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[140])))
        # sensor_list.append(activate_lidar_list[140])
        # activate_lidar_list[141].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[141])))
        # sensor_list.append(activate_lidar_list[141])
        # activate_lidar_list[142].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[142])))
        # sensor_list.append(activate_lidar_list[142])
        # activate_lidar_list[143].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[143])))
        # sensor_list.append(activate_lidar_list[143])
        # activate_lidar_list[144].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[144])))
        # sensor_list.append(activate_lidar_list[144])
        # activate_lidar_list[145].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[145])))
        # sensor_list.append(activate_lidar_list[145])
        # activate_lidar_list[146].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[146])))
        # sensor_list.append(activate_lidar_list[146])
        # activate_lidar_list[147].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[147])))
        # sensor_list.append(activate_lidar_list[147])
        # activate_lidar_list[148].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[148])))
        # sensor_list.append(activate_lidar_list[148])
        # activate_lidar_list[149].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[149])))
        # sensor_list.append(activate_lidar_list[149])
        # activate_lidar_list[150].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[150])))
        # sensor_list.append(activate_lidar_list[150])
        # activate_lidar_list[151].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[151])))
        # sensor_list.append(activate_lidar_list[151])
        # activate_lidar_list[152].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[152])))
        # sensor_list.append(activate_lidar_list[152])
        # activate_lidar_list[153].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[153])))
        # sensor_list.append(activate_lidar_list[153])
        # activate_lidar_list[154].listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, str(activate_lidar_list[154])))
        # sensor_list.append(activate_lidar_list[154])

        for i in vehicles_list:
            i.set_autopilot(True)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        walkers_list=[]
        all_id=[]
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        if args.seedw:
            world.set_pedestrians_seed(args.seedw)
            random.seed(args.seedw)
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
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
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

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

    
       

        print('spawned %d vehicles and %d lidar_sensor, press Ctrl+C to exit.' % (len(vehicles_list), len(activate_lidar_list)))

        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
            try:
                for _ in range(len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
         
                    print("    Frame: %d    timestamp:  %s location: %s Sensor: %s" % (s_frame[0], s_frame[1],s_frame[2],s_frame[3]))
                    file = open("carla-img-infomation.txt","a")
                    file.write("    Frame: %d    timestamp:  %s location: %s Sensor: %s" % (s_frame[0], s_frame[1],s_frame[2],s_frame[3])+ "\n")
                    file.close()

            except Empty:
                print("    Some of the sensor information is missed")

    finally:
            
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
       
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
      

        print('\ndestroying %d walkers' % len(activate_lidar_list))
        for sensor in activate_lidar_list:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
