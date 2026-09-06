from setuptools import find_packages, setup

package_name = 'cuadc_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fake_perception.launch.py']),
        ('share/' + package_name + '/config', ['config/fake_perception_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CUADC Team',
    maintainer_email='cuadc-team@users.noreply.github.com',
    description='CUADC 感知包骨架：假感知节点 + 契约校验器（真感知 M2 实现）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_perception_node = cuadc_perception.fake_perception_node:main',
            'check_vision_contract = cuadc_perception.check_vision_contract:main',
        ],
    },
)
