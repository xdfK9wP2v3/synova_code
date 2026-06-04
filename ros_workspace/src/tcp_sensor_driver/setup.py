from setuptools import find_packages, setup

package_name = 'tcp_sensor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='abc@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtk_main = tcp_sensor_driver.rtk_main:main',
            'adis_main = tcp_sensor_driver.adis_main:main',
            'mag_main = tcp_sensor_driver.mag_main:main',
            'lps22_main = tcp_sensor_driver.lps22_main:main',
            'lps28_main = tcp_sensor_driver.lps28_main:main',
            'sync_main = tcp_sensor_driver.sync_main:main',
            "rtcm_main = tcp_sensor_driver.rtcm_main:main",
        ],
    },
)
