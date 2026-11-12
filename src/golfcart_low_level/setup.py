from glob import glob
from setuptools import find_packages, setup

package_name = 'golfcart_low_level'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools', 'python-can', 'aenum'],
    zip_safe=True,
    maintainer='GolfCart Team',
    maintainer_email='user@example.com',
    description='Low-level ROS 2 to CAN bridge and KEYA steering driver for the golf cart.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'ros_can_bridge = golfcart_low_level.ros_can_bridge:main',
            'steering_node = golfcart_low_level.steering_node:main',
            'joy_cmd_vel = golfcart_low_level.joy_cmd_vel:main',
            'fake_cmd_vel = golfcart_low_level.fake_cmd_vel:main',
        ],
    },
)
