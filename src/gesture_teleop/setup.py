from setuptools import setup

package_name = 'gesture_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ik_commander = gesture_teleop.ik_commander:main',
            'gesture_control_node = gesture_teleop.gesture_control_node:main',
        ],
    },
)
