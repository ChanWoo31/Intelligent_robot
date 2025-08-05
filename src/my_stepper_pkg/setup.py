from setuptools import find_packages, setup

package_name = 'my_stepper_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'rclpy',
                      'geometry_msgs',
                      'pyserial',
                      ],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = my_stepper_pkg.serial_bridge:main',
            'z_control = my_stepper_pkg.z_control:main',
            'packer_controller = my_stepper_pkg.packer_controller:main',
        ],
    },
)
