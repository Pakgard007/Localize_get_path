from glob import glob
from setuptools import setup

package_name = 'nav_path_launch'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GolfCart Team',
    maintainer_email='user@example.com',
    description='Launch files for src/sim/nav_path.py and the CBF-QP control chain, '
                'plus the CBF-QP solver node.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cbf_qp_node = nav_path_launch.cbf_qp_node:main',
            'cbf_qp_logger = nav_path_launch.cbf_qp_logger:main',
        ],
    },
)
