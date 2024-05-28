from setuptools import setup, find_packages

setup(
    name='PololuStepper',
    version='1.0',
    packages=find_packages(),
    install_requires=[
        'RPi.GPIO',  # Add any other dependencies you have here
    ],
    entry_points={
        'console_scripts': [
            # If you have any command-line scripts
        ],
    },
    author='Alex Lazarovich',
    author_email='alexlazarovch@gmail.com',
    description='Implementation of Pololu stepper motor drivers for Raspberry Pi, based on the Pololu library for Arduino.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/AlexLazarovich/PololuStepper',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)
