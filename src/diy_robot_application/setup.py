from setuptools import find_packages, setup

package_name = 'diy_robot_application'

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
    maintainer='hephaestus',
    maintainer_email='hephaestus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    #define here executables you want to call from the commamnd line. This are usual your applications
    entry_points={                                                     
        'console_scripts': [                        
            'example = diy_robot_application.example:main',
            'printworldpos= diy_robot_application.printworldpos:main',   
        ],
    },
)
