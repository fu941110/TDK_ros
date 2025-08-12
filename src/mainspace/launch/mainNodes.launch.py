from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnTopicMessage
from launch.events.process import ShutdownProcess
from std_msgs.msg import Bool


def generate_launch_description():
    # 永遠開啟的節點
    always_nodes = [
        Node(package='SerialTest', executable='serial_node', name='serial_node', output='screen'),
        Node(package='mainspace', executable='navigator', name='navigator', output='screen'),
        Node(package='camera', executable='cameraBackground', name='cameraBackground', output='screen'),
    ]

    # Stage 2 節點
    stage2_node = Node(package='mainspace', executable='Stage2', name='Stage2', output='screen')
    cameraStage2_node = Node(package='camera', executable='cameraStage2', name='cameraStage2', output='screen')
    cameraDesk2_node = Node(package='camera', executable='cameraDesk2', name='cameraDesk2', output='screen')

    # Stage 3 節點
    stage3_nodes = [
        Node(package='mainspace', executable='Stage3', name='Stage3', output='screen'),
        Node(package='camera', executable='cameraStage3', name='cameraStage3', output='screen'),
    ]

    # Stage 4 節點
    stage4_nodes = [
        Node(package='mainspace', executable='Stage4', name='Stage4', output='screen'),
    ]

    ld = LaunchDescription(always_nodes)

    # Stage 2 控制
    def control_stage2(msg):
        if msg.data:
            return [stage2_node]  # 啟動
        else:
            return [EmitEvent(event=ShutdownProcess(
                process_matcher=lambda proc: proc.action == stage2_node
            ))]
    ld.add_action(RegisterEventHandler(
        OnTopicMessage(topic='/stage2_start', msg_type=Bool, actions=control_stage2)
    ))

    def control_cameraStage2(msg):
        if msg.data:
            return [cameraStage2_node]
        else:
            return [EmitEvent(event=ShutdownProcess(
                process_matcher=lambda proc: proc.action == cameraStage2_node
            ))]
    ld.add_action(RegisterEventHandler(
        OnTopicMessage(topic='/cameraStage2_start', msg_type=Bool, actions=control_cameraStage2)
    ))

    def control_cameraDesk2(msg):
        if msg.data:
            return [cameraDesk2_node]
        else:
            return [EmitEvent(event=ShutdownProcess(
                process_matcher=lambda proc: proc.action == cameraDesk2_node
            ))]
    ld.add_action(RegisterEventHandler(
        OnTopicMessage(topic='/cameraDesk2_start', msg_type=Bool, actions=control_cameraDesk2)
    ))

    # Stage 3 控制
    def control_stage3(msg):
        if msg.data:
            return stage3_nodes
        else:
            return [
                EmitEvent(event=ShutdownProcess(process_matcher=lambda proc: proc.action == node))
                for node in stage3_nodes
            ]
    ld.add_action(RegisterEventHandler(
        OnTopicMessage(topic='/stage3_start', msg_type=Bool, actions=control_stage3)
    ))

    # Stage 4 控制
    def control_stage4(msg):
        if msg.data:
            return stage4_nodes
        else:
            return [
                EmitEvent(event=ShutdownProcess(process_matcher=lambda proc: proc.action == node))
                for node in stage4_nodes
            ]
    ld.add_action(RegisterEventHandler(
        OnTopicMessage(topic='/stage4_start', msg_type=Bool, actions=control_stage4)
    ))

    return ld
