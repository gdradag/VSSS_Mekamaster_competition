from setuptools import setup

package_name = 'launch_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriela',
    maintainer_email='gdradag@gmail.com',
    description='Teleoperation package for robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_interface = launch_teleop.teleop_interface:main',
            'teleop_node = launch_teleop.teleop_node:main',
            'teleop_Snode = launch_teleop.teleop_Snode:main',
        ],
    },
)
