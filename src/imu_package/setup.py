from setuptools import find_packages, setup, os
import glob

package_name = 'imu_package'
submodules = 'imu_package/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
	    (os.path.join('share', package_name,'config'), glob.glob('config/*.yaml')),
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
		    'imu_main = imu_package.imu_main:main',
        	'wheel_listener = imu_package.wheel_listener:main',
        
        ],
    },
)
