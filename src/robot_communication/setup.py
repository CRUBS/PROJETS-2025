from setuptools import setup

package_name = 'robot_communication'

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
    maintainer='crubs',
    maintainer_email='crubs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotI2C = robot_communication.robotI2C:main',
            'moteurI2C = robot_communication.moteurI2C:main',
            'interfaceI2C = robot_communication.interfaceI2C:main',
            'interfaceGPIO = robot_communication.interfaceGPIO:main',
            'testI2C = robot_communication.testI2C:main',
	    'interfacePAMI = robot_communication.interfacePAMI:main'

        ],
    },
)
