# Installation

The game engine is built in Python, using the Pygame and NumPy packages.

This package uses Python 3.7 or above.

# Anaconda

We provide an Anacoda YAML file to provide a clean python environment in which to run the game engine. 

1. Download miniconda or anaconda 
2. Save environment_gameai_yml.yaml (and the rest of the contents of this directory) to a directory of your choice
3. From a Anacoda command prompt, or shell with access to conda, run the following in the directory you saved the YAML file
```conda env create -f environment_gameai_yml.yaml```
4. Start the environment:
```source activate gameai```
or
```conda activate gameai```

# Test the basic game engine

```python runbasic.py```
