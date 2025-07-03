from setuptools import setup
from glob import glob
import os

package_name = 'newcastle_racing_ai'

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ernesto Casablanca',
    maintainer_email='casablancaernesto@gmail.com',
    description='Package developed by the Newcastle Racing AI team for the UK Formula AI competition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = newcastle_racing_ai.controller:main',
            'perception = newcastle_racing_ai.perception:main',
            'safety = newcastle_racing_ai.safety:main',
            'mission_control = newcastle_racing_ai.mission_control:main',
            'odometry = newcastle_racing_ai.odometry:main',
        ],
    },
)
