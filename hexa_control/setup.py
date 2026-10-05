from setuptools import find_packages, setup

package_name = 'hexa_control'

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
    maintainer='suraj',
    maintainer_email='suraj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hexa_node = hexa_control.hexa_node:main',
            'dynamixels_driver = hexa_control.dynamixels_controller:main',
            'sim_relay = hexa_control.sim_relay:main',
        ],
    },
)
