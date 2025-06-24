from setuptools import setup

package_name = 'capture_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'torchvision',
        'opencv-python',
        'ultralytics'
    ],
    zip_safe=True,
    maintainer='srking501',
    maintainer_email='alshadadiabdullah853@gmail.com',
    description='Capture Camera Example',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_camera_node = capture_camera.capture_camera_node:main',
            'find_paths = capture_camera.find_paths:main'
        ],
    },
)
