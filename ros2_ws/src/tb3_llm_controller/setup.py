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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),

    ],
    install_requires=['setuptools','google-genai==2.34.0'],
    zip_safe=True,
    maintainer='tgq',
    maintainer_email='maintainer@example.com',
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
        ],
    },
)
