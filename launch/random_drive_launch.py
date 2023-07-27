
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(package='random_drive',
			executable='random_drive',
			output='screen',
			parameters=[{
			'use_sim_time': True
			}],
			remappings=[
			('rnd_scan', '/scan'),
			('rnd_vel', '/cmd_vel')
			])
		                
	])



