from setuptools import setup

package_name = 'allocation'

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
    description='Task allocator for FleetLang',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_allocator_node = allocation.task_allocator_node:main',
        ],
    },
)
