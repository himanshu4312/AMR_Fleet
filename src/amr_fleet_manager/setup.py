from setuptools import find_packages, setup

package_name = 'amr_fleet_manager'

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
    maintainer='himanshu',
    maintainer_email='himanshulohakare4@gmail.com',
    description='Central fleet manager node for AMR_Fleet multi-robot coordination.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fleet_manager_node = amr_fleet_manager.fleet_manager_node:main',
            'fleet_watchdog_node = amr_fleet_manager.fleet_watchdog_node:main',
        ],
    },
)
