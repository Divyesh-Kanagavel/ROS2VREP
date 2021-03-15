from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            namespace='pioneer1',
            executable='pioneer_pub',
            name='pub',
            remappings=[('leftMotorSpeed','/leftMotorSpeed1'),('rightMotorSpeed','/rightMotorSpeed1'),]



        ),
        Node(
            package='my_package',
           namespace='pioneer1',
            executable='pioneer_sub',
            name='sub',
            remappings=[('forcevalue','/forcevalue1'),]
        ),
        Node(
            package='my_package',
            namespace='pioneer2',
            executable='pioneer_pub',
            name='pub',
            remappings=[('leftMotorSpeed','/leftMotorSpeed2'),('rightMotorSpeed','/rightMotorSpeed2'),]



        ),
        Node(
            package='my_package',
           namespace='pioneer2',
            executable='pioneer_sub',
            name='sub',
            remappings=[('forcevalue','/forcevalue2'),]
        ),
        Node(
            package='my_package',
            namespace='pioneer3',
            executable='pioneer_pub',
            name='pub',
            remappings=[('leftMotorSpeed','/leftMotorSpeed3'),('rightMotorSpeed','/rightMotorSpeed3'),]



        ),
        Node(
            package='my_package',
           namespace='pioneer3',
            executable='pioneer_sub',
            name='sub',
            remappings=[('forcevalue','/forcevalue3'),]
        )


        
    ])