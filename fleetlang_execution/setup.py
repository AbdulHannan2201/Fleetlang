from setuptools import setup

package_name = 'fleetlang_execution'

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
    maintainer='hannan',
    maintainer_email='abdulhannan220105@gmail.com',
    description='Task executor and local navigation for FleetLang',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_executor_node = fleetlang_execution.task_executor_node:main',
        ],
    },
)
