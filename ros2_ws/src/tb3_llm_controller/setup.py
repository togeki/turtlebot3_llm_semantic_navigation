from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'tb3_llm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), [
            'launch/tb3_semantic_world.launch.py',
        ]),
        (os.path.join('share', package_name, 'worlds'), [
            'worlds/semantic_room.world',
        ]),
        (os.path.join('share', package_name, 'urdf'), [
            'urdf/tb3_with_cam.urdf',
        ]),

    ],
    install_requires=['setuptools','google-generativeai'],
    zip_safe=True,
    maintainer='tgq',
    maintainer_email='togqi0725@gmail.com',
    description='TurtleBot3 LLM-based semantic navigation (LV1)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_script_controller = tb3_llm_controller.simple_script_controller:main',
            'llm_controller = tb3_llm_controller.llm_controller:main',
            'nav2_client = tb3_llm_controller.nav2_client:main',
            'place_nav_client = tb3_llm_controller.place_nav_client:main',
            'llm_nav_bridge = tb3_llm_controller.llm_nav_bridge:main',
            'semantic_place_bridge = tb3_llm_controller.semantic_place_bridge:main',
        ],
    },
)
