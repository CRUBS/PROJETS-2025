from setuptools import find_packages, setup

package_name = 'asserposition'

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
    maintainer='crubs',
    maintainer_email='wacrenier.e2000822@etud.univ-ubs.fr',
    description='Asservissement en position du robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_publish = asserposition.cmd_vel_rampPublisher:main',
            'cmd_turn_publish = asserposition.cmd_vel_turnPublisher:main',
            'cmd_dist_publish = asserposition.cmd_dist_publisher:main',
            'cmd_dist_publish_dev = asserposition.cmd_dist_publisher_dev:main',
            'asservissement = asserposition.asservissement:main',
        ],
    },
)
