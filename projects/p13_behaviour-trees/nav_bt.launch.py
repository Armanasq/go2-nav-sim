"""Nav2 + AMCL exactly as nav.launch.py starts them, but with bt_navigator's default_nav_to_pose_bt_xml set to the tree you pass.
usage:  ros2 launch projects/p13_behaviour-trees/nav_bt.launch.py bt_xml:=$PWD/projects/p13_behaviour-trees/trees/recovery_swapped.xml
Without bt_xml it loads Nav2's own navigate_to_pose_w_replanning_and_recovery.xml, i.e. today's behaviour. No passive mapper."""
import os, tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def with_tree(context):
    me = share('go2_nav_sim')
    # Copy the course's Nav2 parameters and add one key. (nav2_common's RewrittenYaml only rewrites keys that already exist
    # in the file; this one does not, so it was silently dropped. See "If it goes wrong".)
    params = yaml.safe_load(open(os.path.join(me, 'config/nav2_params.yaml')))
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = LaunchConfiguration('bt_xml').perform(context)
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_params_bt_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out); out.close()
    print(f'nav_bt: {out.name} uses {params["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]}')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('nav2_bringup'), 'launch/bringup_launch.py')),
        launch_arguments={'map': os.path.join(me, 'maps/arena.yaml'), 'params_file': out.name,
                          'use_sim_time': 'true', 'autostart': 'true', 'use_composition': 'False'}.items())]


def generate_launch_description():
    default_tree = os.path.join(share('nav2_bt_navigator'), 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    return LaunchDescription([
        DeclareLaunchArgument('bt_xml', default_value=default_tree, description='absolute path of the NavigateToPose tree'),
        OpaqueFunction(function=with_tree),
    ])
