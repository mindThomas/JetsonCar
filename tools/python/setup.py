import setuptools

with open('requirements.txt', 'r') as file:
    requirements = [line.strip('\n') for line in file.readlines() if line != '' and line != '\n']

setuptools.setup(
    name='jetsoncar',
    version='0.0.1',
    description='JetsonCar RC project Python tools package',
    author='Thomas Jespersen',
    author_email='thomasj@tkjelectronics.dk',
    #package_dir={"": "src"},
    #packages=find_packages(where="src"),
    packages=[package for package in setuptools.find_packages()],
    install_requires=requirements,    
)