from setuptools import find_packages, setup
import os


package_name = 'shapes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/shapes']),
    ('share/shapes', ['package.xml']),
    ('share/shapes/launch', ['launch/shapes_launch.py']), 
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasmin',
    maintainer_email='jasmin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "shapeNode= shapes.shapeNode:main",
            "turtleCommander=shapes.turtleCommander:main"
        ],
    },
)
