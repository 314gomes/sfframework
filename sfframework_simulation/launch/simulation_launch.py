import os

import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from launch.actions import LogInfo

from ament_index_python.packages import get_package_share_directory

from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Union, Optional

from launch import SomeSubstitutionsType, SomeSubstitutionsType_types_tuple
from launch.substitutions import SubstitutionFailure
from launch.frontend.parse_substitution import parse_substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.utilities.typing_file_path import FilePath
from launch.substitution import Substitution
from launch import LaunchContext

class ConfigFile(Substitution):
    """Substitution to get the path of the configuration file."""

    def __init__(
        self,
        param_file: Union[FilePath, SomeSubstitutionsType],
    ) -> None:
        """
        Construct a parameter file description.

        :param param_file: The path to the parameter file or a substitution that resolves to it.
        """
        self.__evaluated_param_file: Optional[Path] = None
        self.__created_tmp_file = False

        self.__param_file = param_file
        if isinstance(param_file, SomeSubstitutionsType_types_tuple):
            self.__param_file = normalize_to_list_of_substitutions(param_file)  # type: ignore

    def perform(self, context: LaunchContext) -> str:
        """Substitute the parameter file path."""
        param_file = self.__param_file
        if isinstance(param_file, list):
            # list of substitutions
            param_file = perform_substitutions(context, self.__param_file)  # type: ignore

        param_file_path: Path = Path(param_file)  # type: ignore
        with open(param_file_path, 'r') as f, NamedTemporaryFile(
                mode='w', prefix='launch_params_', delete=False
            ) as h:
                parsed = perform_substitutions(context, parse_substitution(f.read()))  # type: ignore
                try:
                    yaml.safe_load(parsed)
                except Exception:
                    raise SubstitutionFailure(
                        'The substituted parameter file is not a valid yaml file')
                h.write(parsed)
                param_file_path = Path(h.name)
                self.__created_tmp_file = True
        self.__evaluated_param_file = param_file_path
        return str(param_file_path)

    def cleanup(self) -> None:
        """Remove the temporary file if it was created."""
        if self.__created_tmp_file and self.__evaluated_param_file is not None:
            try:
                self.__evaluated_param_file.unlink()
            except FileNotFoundError:
                # The file may have been deleted already, ignore this error
                pass
            self.__evaluated_param_file = None

    def __del__(self):
        """Clean up the temporary file when the object is deleted."""
        self.cleanup()


def substitute_param_context(param, context):
    """Resolve a parameter if it is a LaunchConfiguration."""
    if isinstance(param, LaunchConfiguration):
        return param.perform(context)
    return param

def launch_setup(context, params):

    # 1. Resolve the substitution to a simple string
    resolved_xacro_path = params['robot_xacro']
    
    # If it is still a LaunchConfiguration object, we resolve it using the context
    if not isinstance(resolved_xacro_path, str):
        resolved_xacro_path = context.perform_substitution(params['robot_xacro'])

    # 2. Print it immediately using Python print (since we are inside OpaqueFunction)
    print(f"\n\n[DEBUG] LOADED XACRO: {resolved_xacro_path}\n\n")

    ret = []

    # Robot Description
    ret.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robotnik_description'), '/launch/robot_description.launch.py'
        ]),
        launch_arguments={
            'verbose': 'false',
            'robot_xacro_path': params['robot_xacro'],
            'frame_prefix': [params['robot_id'], '_'],
            'namespace': params['robot_id'],
            'gazebo_ignition': 'true',
            'low_performance_simulation': params['low_performance_simulation']
        }.items(),
    ))

    # Spawner
    ret.append(Node(
        package='ros_gz_sim',
        executable='create',
        namespace=params['robot_id'],
        arguments=[
            '-name', params['robot_id'],
            '-topic', "robot_description",
            '-robot_namespace', params['robot_id'],
            '-x', params['x'],
            '-y', params['y'],
            '-z', params['z'],
        ],
        output='screen',
    ))

    # Gazebo bridge
    def generate_bridge_yaml(params) -> str:
        robot_id = substitute_param_context(params['robot_id'], context)
        bridge_raw = [
            (f"/{robot_id}/imu/data", f"/{robot_id}/imu/data", "sensor_msgs/msg/Imu", "ignition.msgs.IMU", "GZ_TO_ROS"),
            (f"/{robot_id}/gps/data", f"/{robot_id}/gps/fix", "sensor_msgs/msg/NavSatFix", "ignition.msgs.NavSat", "GZ_TO_ROS"),
        ]
        def add_camera(camera_name):
            bridge_raw.extend([
                (f"/{robot_id}/{camera_name}_camera_color/color/camera_info", f"/{robot_id}/{camera_name}_rgbd_camera/color/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"),
                (f"/{robot_id}/{camera_name}_camera_color/color/image_raw", f"/{robot_id}/{camera_name}_rgbd_camera/color/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
            ])
        def add_laser(laser_name):
            bridge_raw.extend([
                (f"/{robot_id}/{laser_name}_laser/scan", f"/{robot_id}/{laser_name}_laser/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
            ])
        def add_pointcloud(points_name):
            bridge_raw.extend([
                # ( f"/{robot_id}/{points_name}_lidar/scan/points", f"/{robot_id}/{points_name}_laser/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", "GZ_TO_ROS"),
                ( f"/{robot_id}_{points_name}_lidar/scan/points/points", f"/{robot_id}/{points_name}_laser/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", "GZ_TO_ROS"),
            ])

        def add_depth_camera(camera_name):
            bridge_raw.extend([
                (f"/{robot_id}/{camera_name}_camera_depth/depth/camera_info", f"/{robot_id}/{camera_name}_rgbd_camera/depth/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"),
                (f"/{robot_id}/{camera_name}_camera_depth/depth/image_raw", f"/{robot_id}/{camera_name}_rgbd_camera/depth/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
            ])

        add_camera("front")
        add_camera("rear")
        add_camera("top_ptz")
        #add_depth_camera("front")
        add_laser("front")
        add_laser("rear")
        add_pointcloud("top")

        bridge_config = [{"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type, "gz_type_name": gz_type, "direction": direction} for gz, ros, ros_type, gz_type, direction in bridge_raw]
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as tmp:
            yaml.dump(bridge_config, tmp)
            return tmp.name

    bridge_yaml = generate_bridge_yaml(params)
    ret.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {'config_file': bridge_yaml},
        ],
        namespace=params['robot_id'],
    ))


    def extract_controllers_from_yaml(yaml_path):

        data = {}
        existing_controllers = []
        # Load the YAML file
        with open(yaml_path, 'r') as f:

            # Read the file content
            content = f.read()
            # Remove the string "---\n/**:" if it exists at the beginning
            if content.startswith('---\n/**:'):
                content = content[len('---\n/**:'):]
            # Move file pointer back to start for yaml.safe_load
            f.seek(0)
            f = tempfile.SpooledTemporaryFile(mode='w+')
            f.write(content)
            f.seek(0)

            try:
                data = yaml.safe_load(f)
            except Exception as e:
                raise RuntimeError(f"Failed to parse YAML file '{yaml_path}': {e}")

        for controller in data:
            existing_controllers.append(controller)
        return existing_controllers

    def get_ros2_control_yaml_path(params):
        return str(
            Path(
                FindPackageShare('sfframework_simulation').perform(context)
            )
            / 'config'
            / substitute_param_context(params['robot'], context)
        ) + '_ros2_control.yaml'

    path = get_ros2_control_yaml_path(params)

    new_controllers = extract_controllers_from_yaml(path)
    print(f"\n\n[DEBUG] LOADED CONTROL YAML: {path}\n\n")
    print("Controllers from YAML:", new_controllers)

    # ROS2 control
    controllers = ['joint_state_broadcaster']
    # Replace default joint_state_broadcaster by the one defined in the specific
    # ros2_control.yamlrobot model
    if 'joint_state_broadcaster' in new_controllers:
        controllers.remove('joint_state_broadcaster')

    if 'controller_manager' in new_controllers:
        print("Removing controller_manager from controllers to be spawned")
        new_controllers.remove('controller_manager')
    controllers.extend(new_controllers)

    print("Controllers to be spawned:", controllers)

    robot_controller_config = ConfigFile(
        [
            FindPackageShare('sfframework_simulation'), '/config/', LaunchConfiguration('robot'), '_ros2_control.yaml',
        ],
    )

    controllers.append('--param-file')
    controllers.append(
         robot_controller_config, # type: ignore
    )

    ret.append(Node(
        package='controller_manager',
        executable='spawner',
        namespace=params['robot_id'],
        arguments=controllers,
        output='screen',
    ))

    # Check if rviz config path is modified, if not use default fixed frame
    rviz_config_default = str(
        Path(
            FindPackageShare('sfframework_simulation').perform(context)
        )
        / 'config'
        / 'rviz_config.rviz'
    )

    print("Default RViz config path:", rviz_config_default)

    use_fixed_frame = False
    # Determine if fixed frame should be used
    if isinstance(params['rviz_config'], LaunchConfiguration):
        rviz_config_value = params['rviz_config'].perform(context)
        use_fixed_frame = (rviz_config_value == "")
    else:
        use_fixed_frame = (params['rviz_config'] == "")

    if use_fixed_frame:
        params['rviz_config'] = rviz_config_default

    # RViz
    ret.append(Node(
        package="rviz2",
        executable="rviz2",
        namespace=params['robot_id'],
        arguments=[
            # Fixed frame
            ['-f', params['robot_id'], '_base_footprint'] if use_fixed_frame else [],
            # Config file
            '-d', [params['rviz_config']],
            # Window name
            '-t', [params['robot_id'], ' - ', params['robot_model'], ' - RViz'],
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(params['run_rviz'])
    ))
    return ret


def generate_launch_description():
    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)
    world = LaunchConfiguration('world')

    arg = ExtendedArgument(
        name='world',
        description='world in gazebo classic',
        default_value='demo',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='world_path',
        description='world path in gazebo classic',
        default_value=[FindPackageShare('robotnik_gazebo_ignition'), '/worlds/', world, '.world'], # type: ignore
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='gui',
        description='Set to true to enable gazebo gui, headless mode (default: true)',
        default_value='true',
    )
    add_to_launcher.add_arg(arg)


    raw_args = [
        ("robot_id", "Unique Robot Identifier", "robot", "ROBOT_ID"),
        
        ("robot", "Robot Model Name", "rbwatcher", "ROBOT"),
        ("robot_model", "Robot Variant or Type", LaunchConfiguration('robot'), "ROBOT_MODEL"),
        # ("robot_xacro", "Path to Robot Xacro File", [FindPackageShare('robotnik_description'), '/robots/', LaunchConfiguration('robot'), '/', LaunchConfiguration('robot_model'), '.urdf.xacro'], "ROBOT_XACRO"),
        ("robot_xacro", "Path to Robot Xacro File", [FindPackageShare('sfframework_simulation'), '/descriptions/', LaunchConfiguration('robot_model'), '_with_lidar.urdf.xacro'], "ROBOT_XACRO"),
        
        ("x", "Initial X Coordinate", "0.0", "X"),
        ("y", "Initial Y Coordinate", "0.0", "Y"),
        ("z", "Initial Z Coordinate", "0.0", "Z"),
        ("has_arm", "Enable Arm Controller", "False", "HAS_ARM"),
        ("run_rviz", "Run RViz", "True", "RUN_RVIZ"),
        ("rviz_config", "RViz configuration file", "", "CONFIG_RVIZ"),
        ("use_sim_time", "Use simulation time", "True", "USE_SIM_TIME"),
        ("low_performance_simulation", "Enable Low Performance Simulation", "False", "LOW_PERFORMANCE_SIMULATION"),
    ]


    for arg in raw_args:
        extended_arg = ExtendedArgument(
            name=arg[0],
            description=arg[1],
            default_value=arg[2],
            use_env=True,
            environment=arg[3],
        )
        add_to_launcher.add_arg(extended_arg)


    params = add_to_launcher.process_arg()

    gazebo_ignition_launch_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory(
                                'ros_gz_sim'
                            ),
                            'launch'
                        ),
                        'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args':[
                        '-r ',
                        '-s ',
                        params['world_path']
                    ],
                    'on_exit_shutdown':'true'
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory(
                                'ros_gz_sim'
                            ),
                            'launch'
                        ),
                        'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args':[
                        '-g '
                    ],
                    'on_exit_shutdown':'true'
                }.items(),
                # gui is in (true, 1, yes, on) (case insensitive)
                condition = IfCondition(PythonExpression(["'", params['gui'], "'.strip().lower() in ('true','1','yes','on')"])),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            ),
            Node(
                package='sfframework',
                executable='sfgenerator',
                name='sfgenerator',
                parameters=[{
                    'use_sim_time': True,
                    'filter_plugins': ['stat_outlier_removal', 'voxel_downsample'],
                    'stat_outlier_removal': {
                        'plugin': 'sfframework/StatisticalOutlierRemovalFilter',
                        'nb_neighbors': 30,
                        'std_ratio': 2.0,
                    },
                    'voxel_downsample': {
                        'plugin': 'sfframework/VoxelDownSampleFilter',
                        'voxel_size': 0.2,
                    },
                }],
                output='screen'
            )
        ]
    )

    ld.add_action(gazebo_ignition_launch_group)
    ld.add_action(OpaqueFunction(function=launch_setup, args=[params]))

    return ld