import launch
import launch_ros

# # 用launch文件同时启动多个节点
# def generate_launch_description():
#     action_node_face_detect = launch_ros.actions.Node(
#         package='demo_python_service',
#         executable='face_detect_node',
#         output = 'screen',
#     )

#     action_node_face_client_detect = launch_ros.actions.Node(
#         package='demo_python_service',
#         executable='face_detect_client_node',
#         output = 'log',
#         # output = 'both'
#     )

#     # 合成启动描述并返回
#     launch_description = launch.LaunchDescription([
#         action_node_face_detect,
#         action_node_face_client_detect
#     ])

#     return launch_description

# 用launch文件同时启动多个节点，并在启动节点时传参数
def generate_launch_description():
    # 创建参数声明action，用于解析launch命令后跟着的命令行参数
    action_declare_arg_face_locations_model = launch.actions.DeclareLaunchArgument(
        'launch_face_locations_model',default_value='hog')

    action_node_face_detect = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_node', 
        # 使用launch中参数launch_face_locations_model值去替换节点中的face_locations_model参数值
        parameters=[{
            'face_locations_model':launch.substitutions.LaunchConfiguration(
                'launch_face_locations_model',default='hog')}],
        output = 'screen',
    )

    action_node_face_client_detect = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_client_node',
        output = 'screen',
        # output = 'log'
        # output = 'both'
    )

    # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        action_declare_arg_face_locations_model,
        action_node_face_detect,
        action_node_face_client_detect
    ])

    return launch_description

