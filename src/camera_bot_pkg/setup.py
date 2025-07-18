from setuptools import find_packages, setup

package_name = 'camera_bot_pkg'

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
    maintainer='romy',
    maintainer_email='romy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_data_publisher = camera_bot_pkg.camera_data_publisher_node:main',
            'human_detector = camera_bot_pkg.human_detector_node:main'
        ],
    },
)
