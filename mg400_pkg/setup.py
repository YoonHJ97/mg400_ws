from setuptools import find_namespace_packages, setup

package_name = 'mg400_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_node = mg400_pkg.move_node:main',
            'move_service_server = mg400_pkg.move_service_server:main',
            'mg400_status_node = mg400_pkg.status_node:main',
            'nav_and_move = mg400_pkg.nav_and_move:main',
            'nav_and_return = mg400_pkg.nav_and_return:main',
            'test = mg400_pkg.test:main',
            'test_mg400 = mg400_pkg.test_mg400:main',
        ],
    },
)
