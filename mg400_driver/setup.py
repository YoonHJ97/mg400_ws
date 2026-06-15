from setuptools import find_namespace_packages, setup

package_name = 'mg400_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={package_name: ['files/*.json']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hjpc',
    maintainer_email='xxbb96@gmail.com',
    description='Single-connection ROS2 driver for the Dobot MG400 arm '
                '(state topics, motion actions, mode services).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mg400_driver = mg400_driver.driver_node:main',
        ],
    },
)
