from setuptools import find_packages, setup

package_name = 'temperature'

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
    maintainer='brussell03',
    maintainer_email='brussell03@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "temperature_pub = temperature.temp_pub:main",
            "temperature_sub = temperature.temp_sub:main"
        ],
    },
)
