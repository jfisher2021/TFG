from setuptools import find_packages, setup

package_name = 'my_python_pkg'

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
    maintainer='jfisherr',
    maintainer_email='j.fisher.2021@alumnos.urjc.es',
    description='Services and clients for TTS/STT in a virtual museum tour system',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_service = my_python_pkg.src.stt_service:main',
            'stt_client = my_python_pkg.src.stt_client:main',
            'tts_client = my_python_pkg.src.tts_client:main',
            'tts_service = my_python_pkg.src.tts_service:main',
        ],
    },
)
