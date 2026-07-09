from setuptools import setup

package_name = 'orbbec_camera'

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
    description='Orbbec Depth Camera Driver Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orbbec_node = orbbec_camera.orbbec_node:main',
            'pointcloud = orbbec_camera.pointcloud:main',
        ],
    },
)
