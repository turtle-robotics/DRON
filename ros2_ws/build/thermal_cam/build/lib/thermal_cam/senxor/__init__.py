import os
path = os.path.dirname(__file__)
version_file = open(os.path.join(path, 'VERSION'))
__version__ = version_file.read().strip()
