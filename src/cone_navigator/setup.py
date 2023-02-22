from setuptools import setup

package_name = 'cone_navigator'

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
    description='Cone navigator for the turtlebot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_navigator = cone_navigator.cone_navigator_node:main',
            'cone_visualizer = cone_navigator.cone_visualizer_node:main'
        ],
    },
)
