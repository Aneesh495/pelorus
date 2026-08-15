from setuptools import setup, find_packages

setup(
    name="pelorus",
    version="0.8.0",
    description="High-performance real-time spatial perception and kinodynamic motion planning engine",
    author="Aneesh Krishna",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.20.0",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: MacOS :: MacOS X",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires=">=3.8",
)
