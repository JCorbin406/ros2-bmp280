from setuptools import find_packages, setup

package_name = 'bmp280'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bmp280.launch.py']),
        ('share/' + package_name + '/config', ['config/bmp280_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jcorbin',
    maintainer_email='jcorbin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bmp280_node = bmp280.bmp280_node:main',
        ],
    },
)
