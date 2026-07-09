from setuptools import setup

package_name = 'chassis_driver'

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
    description='Chassis Driver Package for Mecanum Wheel Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = chassis_driver.serial_bridge:main',
            'odom_publisher = chassis_driver.odom_publisher:main',
        ],
    },
)
