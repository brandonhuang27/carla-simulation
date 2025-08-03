import carla
import random
import time
import logging
import pygame
from pygame.locals import K_w, K_s, K_a, K_d, K_q, K_e, K_SPACE, K_ESCAPE
import os

# Initialize pygame for manual control
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Manual Control")

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# --------------
# Spawn ego vehicle
# --------------
ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
ego_bp.set_attribute('role_name','ego')
print('\nEgo role_name is set')
ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
ego_bp.set_attribute('color',ego_color)
print('\nEgo color is set')

spawn_points = world.get_map().get_spawn_points()
number_of_spawn_points = len(spawn_points)

if 0 < number_of_spawn_points:
    random.shuffle(spawn_points)
    ego_transform = spawn_points[0]
    ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
    print('\nEgo is spawned')
else: 
    logging.warning('Could not find any spawn points')
    exit()

# --------------
# Attach camera to follow vehicle (spectator mode)
# --------------
spectator = world.get_spectator()

def update_spectator():
    transform = ego_vehicle.get_transform()
    spectator_location = transform.transform(carla.Location(x=-6, z=3))
    spectator_rotation = transform.rotation
    spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))

# --------------
# Spawn multiple attached cameras and save poses
# --------------
os.makedirs('tutorial/poses', exist_ok=True)

def spawn_camera(bp_library, location, rotation, directory, name, converter=None):
    cam_bp = bp_library.find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x", str(1920))
    cam_bp.set_attribute("image_size_y", str(1080))
    cam_bp.set_attribute("fov", str(105))
    transform = carla.Transform(location, rotation)
    camera = world.spawn_actor(cam_bp, transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

    def save_image(image):
        filename = f'{directory}/%.6d.jpg' % image.frame
        image.save_to_disk(filename, converter) if converter else image.save_to_disk(filename)
        cam_transform = camera.get_transform()
        with open(f'tutorial/poses/{name}_poses.txt', 'a') as f:
            f.write(f"{image.frame},{cam_transform.location.x},{cam_transform.location.y},{cam_transform.location.z},"
                    f"{cam_transform.rotation.pitch},{cam_transform.rotation.yaw},{cam_transform.rotation.roll}\n")

    camera.listen(save_image)
    return camera

camera_front = spawn_camera(world.get_blueprint_library(), carla.Location(x=2.0, z=1.4), carla.Rotation(), 'tutorial/camera_front', 'front')
camera_left = spawn_camera(world.get_blueprint_library(), carla.Location(x=0.0, y=-1.0, z=1.4), carla.Rotation(yaw=-90), 'tutorial/camera_left', 'left')
camera_right = spawn_camera(world.get_blueprint_library(), carla.Location(x=0.0, y=1.0, z=1.4), carla.Rotation(yaw=90), 'tutorial/camera_right', 'right')
camera_back = spawn_camera(world.get_blueprint_library(), carla.Location(x=-2.0, z=1.4), carla.Rotation(yaw=180), 'tutorial/camera_back', 'back')

# Sensors: Collision, Lane Invasion, Obstacle
col_bp = world.get_blueprint_library().find('sensor.other.collision')
col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego_vehicle)
def col_callback(colli):
    print("Collision detected:\n" + str(colli) + '\n')
col_sensor.listen(lambda colli: col_callback(colli))

lane_bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
lane_sensor = world.spawn_actor(lane_bp, carla.Transform(), attach_to=ego_vehicle)
def lane_callback(lane):
    print("Lane invasion detected:\n" + str(lane) + '\n')
lane_sensor.listen(lambda lane: lane_callback(lane))

obs_bp = world.get_blueprint_library().find('sensor.other.obstacle')
obs_bp.set_attribute("only_dynamics", str(True))
obs_sensor = world.spawn_actor(obs_bp, carla.Transform(), attach_to=ego_vehicle)
def obs_callback(obs):
    print("Obstacle detected:\n" + str(obs) + '\n')
obs_sensor.listen(lambda obs: obs_callback(obs))

# GNSS and IMU sensors
gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
gnss_bp.set_attribute("sensor_tick", str(3.0))
gnss_sensor = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=ego_vehicle)
def gnss_callback(gnss):
    print("GNSS measure:\n" + str(gnss) + '\n')
gnss_sensor.listen(lambda gnss: gnss_callback(gnss))

imu_bp = world.get_blueprint_library().find('sensor.other.imu')
imu_bp.set_attribute("sensor_tick", str(3.0))
imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego_vehicle)
def imu_callback(imu):
    print("IMU measure:\n" + str(imu) + '\n')
imu_sensor.listen(lambda imu: imu_callback(imu))

# Depth and semantic segmentation cameras
depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
depth_transform = carla.Transform(carla.Location(x=2.0, z=1.4))
depth_cam = world.spawn_actor(depth_bp, depth_transform, attach_to=ego_vehicle)
depth_cam.listen(lambda image: image.save_to_disk('tutorial/depth/%.6d.jpg' % image.frame, carla.ColorConverter.LogarithmicDepth))

sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
sem_bp.set_attribute("image_size_x", str(1920))
sem_bp.set_attribute("image_size_y", str(1080))
sem_bp.set_attribute("fov", str(105))
sem_transform = carla.Transform(carla.Location(x=2.0, z=1.4))
sem_cam = world.spawn_actor(sem_bp, sem_transform, attach_to=ego_vehicle)
sem_cam.listen(lambda image: image.save_to_disk('tutorial/semantic/%.6d.jpg' % image.frame, carla.ColorConverter.CityScapesPalette))

# Manual control loop
control = carla.VehicleControl()
running = True
while running:
    world.tick()
    update_spectator()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == K_ESCAPE:
                running = False

    keys = pygame.key.get_pressed()
    control.throttle = 0.5 if keys[K_w] else 0.0
    control.brake = 1.0 if keys[K_s] else 0.0
    control.steer = -0.5 if keys[K_a] else (0.5 if keys[K_d] else 0.0)
    control.hand_brake = keys[K_SPACE]
    ego_vehicle.apply_control(control)
    time.sleep(0.03)

# Clean up
print("Exiting and cleaning up...")
camera_front.stop()
camera_left.stop()
camera_right.stop()
camera_back.stop()
depth_cam.stop()
sem_cam.stop()
col_sensor.stop()
lane_sensor.stop()
obs_sensor.stop()
gnss_sensor.stop()
imu_sensor.stop()

camera_front.destroy()
camera_left.destroy()
camera_right.destroy()
camera_back.destroy()
depth_cam.destroy()
sem_cam.destroy()
col_sensor.destroy()
lane_sensor.destroy()
obs_sensor.destroy()
gnss_sensor.destroy()
imu_sensor.destroy()
ego_vehicle.destroy()

pygame.quit()