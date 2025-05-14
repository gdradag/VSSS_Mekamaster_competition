from setuptools import find_packages, setup

package_name = 'launch_teleop2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_interface_launch2.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriela',
    maintainer_email='gdradag@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_interface2 = launch_teleop.teleop_interface2:main",
            "teleop_node2 = launch_teleop.teleop_node2:main",
            "teleop_Snode2 = launch_teleop.teleop_Snode2:main",
        ],
    },
)
