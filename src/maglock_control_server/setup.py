from setuptools import find_packages, setup

package_name = 'maglock_control_server'

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
    maintainer='Ntiaju Chukwuebuka Eric',
    maintainer_email='ntiajubukason@gmail.com',
    description='A package to handle the lock and unlock of an electromagnetic lock',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
