from setuptools import find_packages, setup

package_name = 'demo_python_topic'

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
    maintainer='pjc',
    maintainer_email='285489754@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    # 用 extras_require 替代 tests_require
    extras_require={
        'test': ['pytest'],  # 测试依赖
    },
    entry_points={
        'console_scripts': [
            'novel_pub_node = demo_python_topic.novel_pub_node:main',
            'novel_sub_node = demo_python_topic.novel_sub_node:main'
        ],
    },
)
