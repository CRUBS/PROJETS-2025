from setuptools import setup

package_name = 'robot_accessories'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/allAccessories.launch.py'] )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crubs',
    maintainer_email='crubs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lcd_screen_prepare = robot_accessories.lcdPrepare:main',
            'lidar_led_prepare = robot_accessories.lidarLedsPrepare:main',
            'servos_prepare = robot_accessories.servosPrepare:main',
            'motors_prepare = robot_accessories.motorsPrepare:main',
            'sequence_reader = robot_accessories.sequenceRead:main',
'            clock  = robot_accessories.clock:main'
        ],
    },
)
