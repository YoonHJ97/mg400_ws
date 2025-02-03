from setuptools import find_packages, setup

package_name = 'mg400_pkg'

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
    maintainer='hjpc',
    maintainer_email='xxbb96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_node = mg400_pkg.move_node:main',
            'move_service_server = mg400_pkg.move_service_server:main',
        ],
    },
)
