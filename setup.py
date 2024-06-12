from setuptools import setup, find_packages

package_name = 'my_rviz2_plugin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['my_rviz2_plugin', 'my_rviz2_plugin.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='The my_rviz2_plugin package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_signals = my_rviz2_plugin.publish_signals:main'
        ],
    },
)

