from pathlib import Path
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from baja_perception_sim.scene import generate


def start(context):
    def arg(name):return context.launch_configurations[name]
    def enabled(name):return arg(name).lower()=='true'
    share=Path(get_package_share_directory('baja_perception_sim'))
    config=Path(arg('config')) if arg('config') else share/'config/scenario.yaml'
    cfg=yaml.safe_load(config.read_text())
    if arg('seed'):cfg['seed']=int(arg('seed'))
    if arg('speed'):cfg['vehicle']['speed']=float(arg('speed'))
    if int(arg('obstacles'))>=0:cfg['obstacles']['count']=int(arg('obstacles'))
    run=Path(arg('run_dir')).resolve();run.mkdir(parents=True,exist_ok=True)
    generate(cfg,run/'scene',share/'models/vehicle.sdf')
    params={'use_sim_time':True,'scene_file':str(run/'scene/scene.json'),'results_dir':str(run/'results')}
    command=['gz','sim','--force-version','8','-r','-v','2','--render-engine','ogre2']
    if enabled('headless'):command+=['-s','--headless-rendering']
    command+=[str(run/'scene/world.sdf')]
    gazebo=ExecuteProcess(cmd=[sys.executable,'-m','baja_perception_sim.gazebo_runner',*command],
                          sigterm_timeout='12',sigkill_timeout='3',
                          output='screen',additional_env={'GZ_SIM_RESOURCE_PATH':str(run/'scene')})
    algorithm = arg('algorithm').strip()
    if algorithm and enabled('example'):
        raise ValueError('--algorithm and --example are mutually exclusive')
    if enabled('example'):
        algorithm='baja_algorithm_template'
    mode = arg('rviz_mode')
    if mode not in ['debug','evaluation','both']:
        raise ValueError('rviz_mode must be debug, evaluation or both')
    nodes=[LogInfo(msg=f'Speed: {cfg["vehicle"]["speed"]} m/s. Score file: {run / "results/summary.json"}; view with ./score.sh --run-dir {run}'),
           gazebo,RegisterEventHandler(OnProcessExit(target_action=gazebo,on_exit=[Shutdown(reason='Gazebo exited')])),
           Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(share/'config/bridge.yaml'),'use_sim_time':True}],output='screen')]
    nodes.append(Node(package='baja_gz_bridge',executable='cloud_bridge',parameters=[{'use_sim_time':True}],output='screen'))
    for executable in ['platform','evaluate']:
        nodes.append(Node(package='baja_perception_sim',executable=executable,parameters=[params],output='screen'))
    if algorithm:
        entry = Path(get_package_share_directory(algorithm)) / 'launch/perception.launch.py'
        if not entry.is_file():
            raise ValueError(f'Algorithm package {algorithm} must install launch/perception.launch.py')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(entry))))
    if enabled('rviz'):
        for view, config in [('debug','debug.rviz'),('evaluation','perception.rviz')]:
            if mode in [view,'both']:
                nodes.append(Node(package='rviz2',executable='rviz2',name=f'{view}_rviz',
                                  arguments=['-d',str(share/'config'/config)],parameters=[{'use_sim_time':True}],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=''),
        DeclareLaunchArgument('seed',default_value=''),
        DeclareLaunchArgument('obstacles',default_value='-1'),
        DeclareLaunchArgument('speed',default_value=''),
        DeclareLaunchArgument('algorithm',default_value=''),
        DeclareLaunchArgument('rviz_mode',default_value='both'),
        DeclareLaunchArgument('run_dir',default_value=str(Path.cwd()/'generated/current')),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('example',default_value='false'),
        OpaqueFunction(function=start)])
