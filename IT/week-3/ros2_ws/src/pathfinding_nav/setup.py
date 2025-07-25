from setuptools import setup

package_name = 'pathfinding_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/pathfinding_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Innovator',
    maintainer_email='joelthegentle@gmail.com',
    description='Algorithme simple de pathfinding pour TekBot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pathfinder = pathfinding_nav.pathfinder:main',
        ],
    },
)
