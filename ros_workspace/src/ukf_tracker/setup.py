from setuptools import find_packages, setup

package_name = 'ukf_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'tcp_sensor_driver',
                      'torch'],
    zip_safe=True,
    maintainer='',
    maintainer_email='abc@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ukf_main = ukf_tracker.ukf_main:main'
        ],
    },
)
