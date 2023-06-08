from setuptools import setup

package_name = 'pp_controller'

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
    maintainer='user',
    maintainer_email='szymon.z.kwiatkowski@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_f1_tenth_controller = pp_controller.pure_pursuit_f1_tenth_controller:main',
            'validate_topic_sent.py = pp_controller.validate_topic_sent:main'
        ],
    },
)
