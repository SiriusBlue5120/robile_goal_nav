from setuptools import find_packages, setup

package_name = 'robile_goal_nav'

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
    maintainer='shinas',
    maintainer_email='shinasshaji12@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = robile_goal_nav.my_node:main',
            'send_goal = robile_goal_nav.send_goal:main',
            'send_goal_copy = robile_goal_nav.send_goal_copy:main',
        ],
    },
)
