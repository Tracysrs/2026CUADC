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
        ('share/' + package_name + '/launch',
            ['launch/fake_perception.launch.py', 'launch/fake_recon.launch.py',
             'launch/hazard_recon.launch.py',
             'launch/bucket_perception.launch.py', 'launch/h_circle.launch.py']),
        ('share/' + package_name + '/config',
            ['config/fake_perception_params.yaml', 'config/hazard_recon_params.yaml',
             'config/bucket_perception_params.yaml', 'config/h_circle_params.yaml']),
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
            'fake_recon_node = cuadc_perception.fake_recon_node:main',
            'recon_viewer_node = cuadc_perception.recon_viewer_node:main',
            'hazard_recon_node = cuadc_perception.hazard_recon_node:main',
            'p04_odom_perception_node = cuadc_perception.p04_odom_perception_node:main',
            'scene_truth_perception_node = cuadc_perception.scene_truth_perception_node:main',
            'bucket_cv_perception_node = cuadc_perception.bucket_cv_perception_node:main',
            'bucket_perception_node = cuadc_perception.bucket_perception_node:main',
            'h_circle_node = cuadc_perception.h_circle_node:main',
        ],
    },
)
