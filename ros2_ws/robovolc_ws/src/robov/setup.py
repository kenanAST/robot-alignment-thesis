from setuptools import setup

package_name = 'robov'

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
    maintainer='kenan',
    maintainer_email='kenan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = robov.publisher_member_function:main',
                'listener = robov.subscriber_member_function:main',
                'commander = robov.commander:main',
        ],
    },
)
