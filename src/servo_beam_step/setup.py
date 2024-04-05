from setuptools import find_packages, setup

package_name = 'servo_beam_step'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Costa',
    maintainer_email='lipemenezescosta@gmail.com',
    description='Servo-Beam step response to transfer function.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obtain_data = servo_beam_step.__init__:main'
        ],
    },
)
