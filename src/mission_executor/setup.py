from setuptools import setup

package_name = 'mission_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavan',
    maintainer_email='you@example.com',
    description='Validated JSON -> deterministic Nav2 executor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'executor_node = mission_executor.executor_node:main',
        ],
    },
)
