# A* Route Planner Project

Route Planner was developed as a final project for Udacity's C++ Developer nano-degree program. 
It uses A* path search to find shortest distance to the goal. The project uses OSM map data. 
Apart from coding the algorithm, I also gained in-depth knowledge of how lane segments, intersections and other map features are constructed from individual nodes. 

When executed, the program takes input co-ordinates (x,y) for start location and destination.
Consider a square with (0,0) being bottom left and (100,100) being top right. Enter any combinations of start/goal points within this limit. The code uses closest node available to user input for path search.  

A map with shortest path rendered is then presented to the user.

Here's an example with (10,10) as start point and (50, 50) as goal.
<img src="map2.png"/>

You can download map of areas you know and provide that .osm file as an argument to this program! Detailed steps for this are documented in the execution section below.  

## Dependencies for Running Locally
* cmake >= 3.11.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.4.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same instructions as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* IO2D
  * Installation instructions for all operating systems can be found [here](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md)
  * The steps for Linux should have following additional command from Debug folder : sudo make install
  * This library must be built in a place where CMake `find_package` will be able to find it

## Compiling and Running

### Compiling
To compile the project, first, create a `build` directory and change to that directory:
```
mkdir build && cd build
```
From within the `build` directory, then run `cmake` and `make` as follows:
```
cmake ..
make
```
### Running
The executable will be placed in the `build` directory. From within `build`, you can run the project as follows:
```
./OSM_A_star_search
```
Or to specify a map file:
```
./OSM_A_star_search -f ../<your_osm_file.osm>
```
 OSM map files for any area can be downloaded on Open Street Map [website](https://www.openstreetmap.org/export#map=15/37.4060/-122.1039). Search for locations and export!

## Testing

The testing executable is also placed in the `build` directory. From within `build`, you can run the unit tests as follows:
```
./test
```

