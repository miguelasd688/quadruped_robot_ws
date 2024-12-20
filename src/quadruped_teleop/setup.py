from setuptools import find_packages, setup

package_name = 'quadruped_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ps5_controller.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguelasd',
    maintainer_email='miguel_ayuso@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_teleop = quadruped_teleop.joystick_teleop:main',
            'keyboard_controller = quadruped_teleop.keybord_teleop:main'
        ],
    },
)
