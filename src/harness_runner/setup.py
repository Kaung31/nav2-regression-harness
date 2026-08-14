from setuptools import find_packages, setup

package_name = 'harness_runner'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_goal = harness_runner.send_goal:main',
            'gen_map = harness_runner.gen_map:main',
            'run_scenario = harness_runner.run_scenario:main',
            'preview = harness_runner.preview:main',
            'gen_scenarios = harness_runner.gen_scenarios:main',
            'batch_run = harness_runner.batch_run:main',
            'merge_shards = harness_runner.merge_shards:main',
        ],
    },
)
