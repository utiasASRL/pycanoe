import setuptools
from pathlib import Path

this_directory = Path(__file__).parent
with open(str(this_directory / "README.md"), encoding="utf-8") as f:
    long_description = f.read()

setuptools.setup(
    name="asrl-pycanoe",
    version="0.1.0",
    description="A toolkit for working with the Canoe dataset in Python",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Blerim Abdullai, Yubo Wang, Mia Thomas",
    # TODO: Set up email
    # TODO: Set up URL
    # author_email='canoe@robotics.utias.utoronto.ca',
    # url='https://github.com/utiasASRL/canoe-devkit',
    license="BSD",
    packages=setuptools.find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.21.0,<2",
        "opencv-python>=4.5.3.56",
        "matplotlib>=3.7",
        "PyYAML>=5.4.0",
        "open3d>=0.13.0",
    ],
)
