from setuptools import setup, find_packages

setup(
    name="NightEngine",
    version="0.1.0",
    author="cenfra",
    description="",
    long_description=open("README.org").read(),
    long_description_content_type="text/plain",
    url="https://github.com/cenfraGit/NightEngine",
    packages=find_packages(),
    python_requires=">=3.12",
    install_requires=[
        "glfw==2.8.0",
        "nodeenv==1.9.1",
        "numpy==2.2.3",
        "pybullet==3.2.7",
        "PyOpenGL==3.1.9",
        "PyOpenGL-accelerate==3.1.9",
        "pyright==1.1.396",
        "scipy==1.15.2",
        "typing_extensions==4.12.2",
    ],
    classifiers=[
        "License :: OSI Approved :: MIT License",
    ],
)
