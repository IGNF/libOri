# libOri

Basic C++ library to read, write and use image orientation files.
Supported formats:
- [IGN/MATIS](http://recherche.ign.fr/labos/matis/accueilMATIS.php) .ori.xml
- [IGN/iTowns](http://github.com/itowns/) .json

Available models
- conic model with or without polynomial distortion
- spheric model

## Optional Dependencies
- [TinyXML](https://sourceforge.net/projects/tinyxml/) (.ori.xml support)
- [JSON](http://json.org/) (iTowns .json support) : ```sudo apt-get install libjsoncpp-dev```
- [PBRT](http://pbrt.org/) and [OpenCV](http://opencv.org/) (raytracing module)
