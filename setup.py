from setuptools import find_packages, setup

package_name = 'intro_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch' , ['launch/giraff_robot.launch.py']),
        ('share/' + package_name + '/urdf' , ['urdf/giraff_robot.urdf']),
        ('share/' + package_name + '/rviz' , ['rviz/giraff_robot.rviz']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Introduction to Robotics Project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'giraff_robot = intro_robot.giraff_robot:main'
     ],
    },
)
