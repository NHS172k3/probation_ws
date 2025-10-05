from setuptools import setup

package_name = 'gate_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhs172003',
    maintainer_email='nhs172003@example.com',
    description='Gate navigator for drone',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = gate_navigator.navigator:main',
            'gate_detector = gate_navigator.gate_detector:main',
            'search_controller = gate_navigator.search_controller:main',
            'behavior_tree_navigator = gate_navigator.behavior_tree_navigator:main',
            'mode_setter = gate_navigator.mode_setter:main',
        ],
    },
)
