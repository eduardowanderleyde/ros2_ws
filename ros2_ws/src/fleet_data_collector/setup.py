from setuptools import setup

package_name = 'fleet_data_collector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduardo',
    maintainer_email='eduardo@example.com',
    description='Coleta de dados de sensores por robot_id (enable/disable, rosbag2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_collector = fleet_data_collector.main:main',
        ],
    },
)
