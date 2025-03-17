from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_controlled_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd3194',
    maintainer_email='abd3194@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speak = voice_controlled_robot.SpeechNode:main',
            'control = voice_controlled_robot.ControlNode:main',
            'energy = voice_controlled_robot.EnergyNode:main',
        ],
    },
)
