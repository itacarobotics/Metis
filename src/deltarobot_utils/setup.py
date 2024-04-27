from setuptools import find_packages, setup

package_name = 'deltarobot_utils'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gepetto_visualizer_node = deltarobot_utils.gepetto_visualizer:main",
            "telemetry_node = deltarobot_utils.telemetry:main",
        ],
    },
)
