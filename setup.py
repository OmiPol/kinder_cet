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
            "kinder_cet_node = kinder_cet.kinder_cet_node:main"
        ],
    },
)
