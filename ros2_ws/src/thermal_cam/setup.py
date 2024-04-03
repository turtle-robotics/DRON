from setuptools import find_packages, setup

package_name = 'thermal_cam'

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
    maintainer='abel',
    maintainer_email='abel@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "thermal_cam_pub = thermal_cam.thermal_cam_pub:main",
        "thermal_cam_sub = thermal_cam.thermal_cam_sub:main"
        ],
    },
)
