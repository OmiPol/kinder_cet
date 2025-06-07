import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kinder_cet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share',package_name),glob('launch/*launch.[pxy][yam]*')),
        (os.path.join('share',package_name),glob('launch/*.[pxy][yam]*')),
        (os.path.join('share',package_name, 'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omi',
    maintainer_email='polancobomar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #"kinder_cet_node = kinder_cet.kinder_cet_node:main",
            #"Odometry = kinder_cet.Odometry:main",
            #"Path_gen = kinder_cet.Path_gen:main",
            "Puzzle_move = kinder_cet.Puzzle_move:main",
            #"Puzzlebot_PP = kinder_cet.Puzzlebot_PP :main",    
            #"motor_test = kinder_cet.motor_test:main",
            #"mybug = kinder_cet.mybug:main",
            #"odom2 = kinder_cet.Odometry_param:main",
            #"test_lidar = kinder_cet.test_lidar:main",
            "odomc = kinder_cet.kalmanfull:main",
            "robug = kinder_cet.robot_bug:main",
            #"odom3 = kinder_cet.kalman_on_steroids:main",
            "move_bot = kinder_cet.Puzzle_move:main",
            "move_ghost = kinder_cet.Puzzle_move_ghost:main",
            #"logger = kinder_cet.logger:main",
            "scan_rect = kinder_cet.lidar_rect:main",
            "high_SM = kinder_cet.high_level:main"

            
            
            
        ],
    },
)
