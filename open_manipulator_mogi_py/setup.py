from setuptools import find_packages, setup

package_name = 'open_manipulator_mogi_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='Python package for Open Manipulator X for Gazebo Harmonic and ROS Jazzy for BME MOGI ROS2 course',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_joint_angles = open_manipulator_mogi_py.send_joint_angles:main',
            'open_gripper = open_manipulator_mogi_py.open_gripper:main',
            'close_gripper = open_manipulator_mogi_py.close_gripper:main',
            'inverse_kinematics = open_manipulator_mogi_py.inverse_kinematics:main',
        ],
    },
)
