from setuptools import find_packages, setup

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maquinox',
    maintainer_email='maquinox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tcp_server = mobile_robot.tcp_server_node:main',
            'cmd_ = mobile_robot.cmd_publisher_node:main',
            'robot_data = mobile_robot.robot_data:main',
            'path_plotter = mobile_robot.path_plotter:main'
        ],
    },
)
