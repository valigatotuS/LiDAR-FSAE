from setuptools import setup

package_name = 'move_tb3'

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
    description='Moving the turtlebot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_tb3 = move_tb3.move_tb3:main',
        ],
    },
)
