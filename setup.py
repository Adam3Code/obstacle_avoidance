from setuptools import setup, find_packages

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adamrustom',
    maintainer_email='adamrustom@todo.todo',
    description='A package for Remote ID and Obstacle Avoidance Nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remote_id_publisher = obstacle_avoidance.remote_id_publisher:main',
            'obstacle_movement = obstacle_avoidance.obstacle_movement:main',
            'obstacle_avoidance_1D = obstacle_avoidance.obstacle_avoidance_1D:main'
            
            
            
        ],
    },
)
