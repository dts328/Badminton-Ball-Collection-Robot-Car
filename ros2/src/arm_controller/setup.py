from setuptools import setup

package_name = 'arm_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Arm Controller Package for 5-DOF Robot Arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_client = arm_controller.moveit_client:main',
            'gripper_control = arm_controller.gripper_control:main',
        ],
    },
)
