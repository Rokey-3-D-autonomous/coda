from setuptools import find_packages, setup

package_name = 'coda'

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
    maintainer='lhj',
    maintainer_email='hojun7889@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot1_node = coda.turtlebot1.turtlebot1_node:main',
            'server1_node = coda.server1.server1_node:main',
            'test_dispatch_publisher = coda.test.test_dispatch_publisher:main'
        ],
    },
)
