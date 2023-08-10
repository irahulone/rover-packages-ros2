from setuptools import setup

package_name = 'gps_diff_rtk_ublox'

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
    maintainer='gps-cluster-i',
    maintainer_email='gps-cluster-i@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_run = gps_diff_rtk_ublox.gps_run:main'
        ],
    },
)
