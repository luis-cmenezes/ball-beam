from setuptools import find_packages, setup

package_name = 'pot_calibration'

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
    maintainer='Luis Costa',
    maintainer_email='lipemenezescosta@gmail.com',
    description='Potentiometer calibration package.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate = pot_calibration.__init__:main',
        ],
    },
)
