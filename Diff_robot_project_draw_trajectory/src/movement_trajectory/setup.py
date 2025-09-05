from setuptools import setup

package_name = 'movement_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omnia',
    maintainer_email='oyoussef@nuedueg',
    description='Publishes a Path message based on Odometry to track robot trajectory',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # maps: ros2 run movement_trajectory trajectory_publisher
            'trajectory_publisher = movement_trajectory.trajectory_publisher:main'
        ],
    },
)
