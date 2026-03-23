from setuptools import find_packages, setup

setup(
    name="motorgo-python",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["spidev", "gpiozero", "RPi.GPIO", "numpy", "imufusion"],
    entry_points={},
    author="Swapnil Pande",
    author_email="swapnil@everyflavor.bot",
    description="Python API for the Raspberry Pi + MotorGo line.",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/Every-Flavor-Robotics/pyplink",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
