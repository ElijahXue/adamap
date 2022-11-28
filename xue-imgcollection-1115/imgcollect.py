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


        ## synchronization
        # settings =world.get_settings()
        # settings.fixed_delta_seconds=0.5 #20fps,5ms
        # settings.synchronous_mode=True
        # world.apply_settings(settings)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        if number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif number_of_vehicles > number_of_spawn_points:
            number_of_vehicles = number_of_spawn_points

        blueprint_library = world.get_blueprint_library()
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprints=world.get_blueprint_library().filter('vehicle.*')
            blueprint = random.choice(blueprints)
        # black color
        # ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # get a random valid occupation in the world
            transform = world.get_map().get_spawn_points()[n]
        # spawn the vehilce
            ego_vehicle = world.spawn_actor(blueprint, transform)
        # set the vehicle autopilot mode
            ego_vehicle.set_autopilot(True)

        # collect all actors to destroy when we quit the script
            actor_list.append(ego_vehicle)

        # add a camera
        # camera_bp = blueprint_library.find('sensor.camera.rgb')
        # # camera relative position related to the vehicle
        # camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        
        # set the callback function
        # camera.listen(lambda image: image.save_to_disk(os.path.join(output_path, '%06d.png' % image.frame)))
        # sensor_list.append(camera)

        # we also add a lidar on it
          

        while True:
            # set the sectator to follow the ego vehicle
            # spectator = world.get_spectator()
            # transform = ego_vehicle.get_transform()
            # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
            #                                         carla.Rotation(pitch=-90)))
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