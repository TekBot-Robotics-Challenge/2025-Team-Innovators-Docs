from setuptools import find_packages, setup

package_name = 'sensor_data_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='innovator',
    maintainer_email='innovator@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = sensor_data_evaluation.publisher_node:main',
            'subscriber_node = sensor_data_evaluation.subscriber_node:main',
        ],
    },
    
)
