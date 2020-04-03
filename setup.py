from setuptools import setup

setup(
    name='pyobs-baader',
    version='0.9',
    description='pyobs component for Baader domes',
    author='Tim-Oliver Husser',
    author_email='thusser@uni-goettingen.de',
    packages=['pyobs_baader'],
    install_requires=[
        'pyserial'
    ]
)
