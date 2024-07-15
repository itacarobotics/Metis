from setuptools import find_packages, setup

package_name = 'deltarobot_inputs'

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
            "gui_node = deltarobot_inputs.gui:main",
            "object_detection_node = deltarobot_inputs.object_detection:main",
            "gcode_parser_node = deltarobot_inputs.gcode_parser:main",
            "gamepad_controller_node = deltarobot_inputs.gamepad_controller:main",
        ],
    },
)
