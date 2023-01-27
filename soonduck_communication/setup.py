from setuptools import setup

package_name = 'soonduck_communication'

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
    maintainer='seongmin',
    maintainer_email='sxj754@case.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication_publisher = soonduck_communication.communication_publisher:main',
            'communication_subscriber = soonduck_communication.communication_subscriber:main',
        ],
    },
)
