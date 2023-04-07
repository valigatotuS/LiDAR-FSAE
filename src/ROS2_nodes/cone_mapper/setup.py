from setuptools import setup

package_name = 'cone_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='valigatotus',
    maintainer_email='valentin.quevy@gmail.com',
    description='Cone mapper',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_mapper = cone_mapper.cone_mapper_node:main',
        ],
    },
)
