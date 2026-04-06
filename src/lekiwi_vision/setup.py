from setuptools import setup

package_name = 'lekiwi_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['src.red_dot_detector'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kang',
    maintainer_email='kindsimple2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_dot_detector = src.red_dot_detector:main',
        ],
    },
)
