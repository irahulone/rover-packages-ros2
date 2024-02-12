from setuptools import setup

package_name = 'perception_using_zed_depth'

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
    maintainer='vision-complex-ii',
    maintainer_email='vision-complex-ii@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_r = perception_using_zed_depth.ros_zed2i_right:main',
            'zed_l = perception_using_zed_depth.ros_zed2i_left:main',
            'combined_data = perception_using_zed_depth.combined_data:main',
        ],
    },
)
