"""
CARLA — the counterpart of webdrivesim.launch.py.
  0. prepare the world    : right town (load it only if needed), rendering on/off (CARLA_RENDER)
  1. carla_ros_bridge     : CARLA world <-> /carla/... topics (synchronous mode, 20 Hz)
  2. carla_spawn_objects  : ego vehicle + sensors from config/carla_ego*.json (CARLA_CAMERAS)
  3. carla_adapter        : lane_state + waypoints + blinkers (what WebDriveSim has and CARLA lacks)
All settings come from docker/.env (environment variables).
"""
import json
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

env = os.environ.get
CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')


def prepare_world():
    """Connect to CARLA, load CARLA_TOWN only if another map is open, switch rendering on/off.
    Returns the map's full name ("Carla/Maps/Town04"): the bridge compares it with its 'town'
    parameter, and with the short name ("Town04") it would reload the map on EVERY start (minutes)."""
    import carla
    client = carla.Client(env('CARLA_HOST', 'localhost'), int(env('CARLA_PORT', '2000')))
    client.set_timeout(300.0)                                  # s; loading a town on a slow PC takes long
    world = client.get_world()

    settings = world.get_settings()
    if settings.synchronous_mode:                              # left over by a previous bridge: CARLA is frozen
        settings.synchronous_mode = False                      # until someone ticks it -> release it first
        world.apply_settings(settings)

    # Remove our car (and its sensors) left by a previous run: a restarted bridge would add a second one
    old = [a for a in world.get_actors() if a.attributes.get('role_name') == 'ego_vehicle']
    old_ids = {a.id for a in old}
    old += [a for a in world.get_actors().filter('sensor.*') if a.parent and a.parent.id in old_ids]
    if old:
        print(f'[carla.launch] removing {len(old)} actors of a previous run')
        client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in old])

    town = env('CARLA_TOWN', 'Town04')
    if world.get_map().name.split('/')[-1] != town:
        print(f'[carla.launch] loading {town} (open: {world.get_map().name}), this can take minutes...')
        world = client.load_world(town)

    settings = world.get_settings()
    settings.no_rendering_mode = env('CARLA_RENDER', 'true') != 'true'   # False: GPU idle, physics still run
    world.apply_settings(settings)                             # the bridge keeps this setting
    return world.get_map().name


def ego_config():
    """Our car + its sensors. CARLA_CAMERAS=true: all cameras + radar, false: no cameras (lighter).
    The rgb_view camera (shown by the keyboard-driving window) gets the size KEYBOARD_WINDOW_SIZE."""
    path = os.path.join(CONFIG, 'carla_ego.json' if env('CARLA_CAMERAS', 'false') == 'true' else 'carla_ego_light.json')
    if env('CARLA_CAMERAS', 'false') != 'true':
        return path
    w, h = (int(n) for n in env('KEYBOARD_WINDOW_SIZE', '800x600').split('x'))
    objects = json.load(open(path))
    for s in objects['objects'][0]['sensors']:
        if s['id'] == 'rgb_view':
            s['image_size_x'], s['image_size_y'] = w, h
    path = '/tmp/carla_ego.json'
    json.dump(objects, open(path, 'w'))
    return path


def setup(context):
    town = prepare_world()
    sensors = ego_config()
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('carla_ros_bridge'), '/carla_ros_bridge.launch.py']),
        launch_arguments={'host': env('CARLA_HOST', 'localhost'), 'port': env('CARLA_PORT', '2000'),
                          'town': town, 'timeout': '120',      # s; slow PCs need long
                          'synchronous_mode': 'true', 'fixed_delta_seconds': '0.05'}.items())
    ego = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
        launch_arguments={'objects_definition_file': sensors}.items())
    adapter = Node(package='bringup', executable='carla_adapter', parameters=[{'use_sim_time': True}], output='screen')
    return [bridge, ego, adapter]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])
