from setuptools import find_packages, setup
from glob import glob

package_name = 'fault_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/fault_detector/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faisal-mazloum',
    maintainer_email='faisal@ingeniarius.pt',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = fault_detector.talker:main',
            'listener = fault_detector.listener:main',
            'gbdt_fault_detector = fault_detector.gbdt_fault_detector:main',
            'gbdt_fault_detector_metrics = fault_detector.gbdt_fault_detector_metrics:main',
            'lstm_fault_detector = fault_detector.lstm_fault_detector:main',
            'lstm_fault_detector_metrics = fault_detector.lstm_fault_detector_metrics:main',
            'heatmap = fault_detector.fault_heatmap:main',
        ],
    },
)
