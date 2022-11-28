import carla
import os
import random


def main():
    actor_list = []
    sensor_list = []
    number_of_vehicles = 100
    output_path = './outputs'
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        # world = client.load_world() # you can also retrive another world by specifically defining
        blueprint_library = world.get_blueprint_library()

        # synchronization
        settings =world.get_settings()
        settings.fixed_delta_seconds=0.05 #20fps,5ms
        settings.synchronous_mode=True
        world.apply_settings(settings)

        blueprints=world.get_blueprint_library().filter('vehicle.*')
        blueprint = random.choice(blueprints)
        blueprint2 = random.choice(blueprints)
        # black color
        # ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # get a random valid occupation in the world
        transform = random.choice(world.get_map().get_spawn_points())
        transform2 = random.choice(world.get_map().get_spawn_points())
        # spawn the vehilce
        ego_vehicle = world.spawn_actor(blueprint, transform)
        ego_vehicle2 = world.spawn_actor(blueprint, transform2)
        # set the vehicle autopilot mode
        ego_vehicle.set_autopilot(True)
        ego_vehicle2.set_autopilot(True)
        # collect all actors to destroy when we quit the script
        actor_list.append(ego_vehicle)
        actor_list.append(ego_vehicle2)
        # add a camera
        blueprint_library = world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp2 = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))      
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))

        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar2 = world.spawn_actor(lidar_bp2, lidar_transform, attach_to=ego_vehicle2)
        sensor_list.append(lidar)
        sensor_list.append(lidar2)

        lidar.listen(lambda point_cloud: point_cloud.save_to_disk(os.path.join(output_path, '%06d.ply' % point_cloud.frame)))
        lidar2.listen(lambda point_cloud: point_cloud.save_to_disk(os.path.join(output_path, 'car2%06d.ply' % point_cloud.frame)))
        
        while True:
     
            pass

    finally:
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')