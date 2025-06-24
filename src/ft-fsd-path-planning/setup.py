from setuptools import setup, find_packages

package_name = 'ft-fsd-path-planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'icecream',
        'typing_extensions',
        'scikit-learn'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of the package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_node = fsd_path_planning.path_planning_node:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ]
)
