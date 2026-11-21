from setuptools import find_packages, setup

package_name = 'p1_reactive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        ('share/' + package_name + '/resource',
            ['resource/gazebo_world.sdf']),

        ('share/' + package_name + '/launch',
            ['launch/simulation.launch.py', 'launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdawgg',
    maintainer_email='Jordan.DeJarnett@ou.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = p1_reactive.controller:main',
        ],
    },
)
