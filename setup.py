from setuptools import setup

package_name = 'custom_interfaces'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'task_server = custom_interfaces.task_server:main',
            'task_client = custom_interfaces.task_client:main',
            'task_publisher = custom_interfaces.task_publisher:main',
        ],
    },
)

