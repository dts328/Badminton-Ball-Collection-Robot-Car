from setuptools import setup

package_name = 'vision_bridge'

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
    description='Vision Bridge Package for RDK X5 and Depth Camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rdkx5_receiver = vision_bridge.rdkx5_receiver:main',
            'depth_converter = vision_bridge.depth_converter:main',
            'target_tracker = vision_bridge.target_tracker:main',
        ],
    },
)
