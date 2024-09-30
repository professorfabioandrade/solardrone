from setuptools import setup
from glob import glob

package_name = 'solar_panels_img_processing'
submodules = 'solar_panels_img_processing.utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'canny_node = solar_panels_img_processing.canny_node:main',
            'dsef_node = solar_panels_img_processing.dsef_node:main',
            'georef_node = solar_panels_img_processing.georef_node:main',
        ],
    },
)
