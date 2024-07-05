from setuptools import find_packages, setup

package_name = 'b3rb_ros_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayank',
    maintainer_email='mayankmahajan.x@nxp.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'vectors = b3rb_ros_line_follower.b3rb_ros_edge_vectors:main',
                'runner = b3rb_ros_line_follower.b3rb_ros_line_follower:main',
                'detect = b3rb_ros_line_follower.b3rb_ros_object_recog:main',
        ],
    },
)
