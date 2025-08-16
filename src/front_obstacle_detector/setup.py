from setuptools import find_packages, setup

package_name = 'front_obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mazils',
    maintainer_email='arturasmaziliauskas2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'front_obstacle_detector = front_obstacle_detector.FrontObstacleDetector:main',
        'teleop_twist_keyboard = front_obstacle_detector.teleop_twist_keyboard:main',
        ],
    },
)
