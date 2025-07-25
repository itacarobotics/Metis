from setuptools import find_packages, setup

package_name = 'deltarobot'

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
    maintainer='ostifede02',
    maintainer_email='ostifederico02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_controller_node = deltarobot.robot_controller:main",
            "task_scheduler_node = deltarobot.task_scheduler:main"
        ],
    },
)
