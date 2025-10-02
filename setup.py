from setuptools import setup

package_name = 'aruco_id_checker'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Publishes PoseStamped for a selected Aruco marker ID',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_publisher = aruco_id_checker.pose_publisher_node:main'
        ],
    },
)