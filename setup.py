from setuptools import find_packages, setup

package_name = 'turtle_dhruv'

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
    maintainer='ugkid',
    maintainer_email='ugkid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "basic = turtle_dhruv.basic : main",
        "spdcon = turtle_dhruv.spdcon : main",
        "spiral = turtle_dhruv.spiral : main",
        "special = turtle_dhruv.special : main",
        "name = turtle_dhruv.name : main",
        "circle = turtle_dhruv.circle : main"
        ],
    },
)
