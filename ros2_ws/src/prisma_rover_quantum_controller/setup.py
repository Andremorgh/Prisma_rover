from setuptools import setup

package_name = 'prisma_rover_quantum_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/goal_generator.launch.py']),
        ('share/' + package_name + '/config', ['config/LUT_lin.csv']),
        ('share/' + package_name + '/config', ['config/LUT_ang.csv']),
        ('share/' + package_name + '/config', ['config/LUT_POS_X.csv']),
        ('share/' + package_name + '/config', ['config/LUT_POS_Y.csv']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Andrea',
    maintainer_email='andrea@example.com',
    description='Controllore ROS2 con LUT fisse per rover differenziale',
    license='MIT',
    entry_points={
    'console_scripts': [
        'quantum_controller_node = prisma_rover_quantum_controller.quantum_controller_node:main',
        'lidar_listener = prisma_rover_quantum_controller.lidar_listener:main',
        'quantum_goal_generator = prisma_rover_quantum_controller.quantum_goal_generator:main', 
        'send_goal = prisma_rover_quantum_controller.send_goal:main', 
        ],
    },
)
