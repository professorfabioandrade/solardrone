from setuptools import setup
from glob import glob

package_name = 'mission'
submodules = 'mission.utils'
controllers_submodule = 'mission.controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules, controllers_submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest', 'unittest'],
    entry_points={
        'console_scripts': [
            'dummy_mission_node = mission.dummy_mission:main',
            'pid_node = mission.pid_node:main',
        ],
    },
)
