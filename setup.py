from setuptools import setup

package_name = 'gazebo2rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("lib/python3.8/site-packages/gazebo2rviz/scripts" , ['scripts2222/gazebo2tf_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arahami',
    maintainer_email='a.rahami88@gmail.com',
    description='connecting rviz to gazebo to display its topics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gazebo2tf = gazebo2rviz.scripts.gazebo2tf_node:main",
        ],
    },
)
