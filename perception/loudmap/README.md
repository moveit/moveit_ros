Loudmap - A fast roadmap free space projector
=======================================================================================

Authors: <>, Willow Garage, Copyright (C) 2013.
http://<>.com

Further Contributors:
* 

License for loudmap: [New BSD License](LICENSE.txt)


REQUIREMENTS
------------

* For only the loudmap library: cmake and a regular build environment (gcc)
* For HTML documentation: doxygen (optional)


Skip to WINDOWS for tips on compilation under Windows. You can install all dependencies on Ubuntu by running:

    sudo apt-get install cmake doxygen 
       

       
INSTALLATION
------------
 
Build the complete project by changing into the "build" directory 
and running cmake:

    mkdir build && cd build	
    cmake ..
	
Type `make` to compile afterwards. This will create all CMake
files cleanly in the `build` folder (Out-of-source build).
Executables will end up in `bin`, libraries in `lib`.


A debug configuration can be created by running:
	
    cmake -DCMAKE_BUILD_TYPE=Debug ..

in `build` or a different directory (e.g. `build-debug`).

You can install the library by running `make install`, though it 
is usually not necessary. Be sure to adjust `CMAKE_INSTALL_PREFIX` before.

The target `make test` executes the unit tests for the loudmap library,
if you are interested in verifying the functionality on your machine.


DOCUMENTATION
-------------

The documentation for the latest stable release is available online:
  http://<>/loudmap/doc/index.html

You can build the most current HTML-Documentation for your current
source with Doxygen by running `make docs` 
in the build directory. The documentation will end up in
`doc/html/index.html` in the main directory.


GETTING STARTED
---------------

<>

USE IN OTHER PROJECTS
---------------------

A CMake-project config is generated for LoudMap which allows it to be used from 
other CMake-Projects easily.

Point CMake to your loudmap installation so that it finds the file
loudmap/lib/cmake/loudmap/loudmap-config.cmake, e.g. by setting the environment
variable `loudmap_DIR`to the directory containing it.

Then add the following to your CMakeLists.txt:

    find_package(loudmap REQUIRED)
    include_directories(${LOUDMAP_INCLUDE_DIRS})
    link_libraries(${LOUDMAP_LIBRARIES})

In addition to this cmake-module we also provide a pkgconfig-file.

ECLIPSE PROJECT FILES
---------------------

Eclipse project files can be generated (with some limitations, see:
http://www.vtk.org/Wiki/Eclipse_CDT4_Generator) by running:

    cmake -G"Eclipse CDT4 - Unix Makefiles" ..
	
Import the project (existing project, root is the build folder, 
do not copy contents) into Eclipse afterwards. For full Eclipse
compatibility, it might be necessary to build in the main source
directory.


WINDOWS
-------

The loudmap library and tools can be compiled and used
under Windows although this has not been tested in-depth. 
Feedback is welcome.

To compile the library you need cmake (http://www.cmake.org).

### MinGW ###

1. Download the MinGW distribution (http://www.mingw.org)
2. Install C++ compiler and add MingGW/bin to your system PATH
3. Start the cmake-gui and set the code directory to the 
  library root (e.g. `/loudmap`)
4. Set the build directory to, e.g., `/loudmap/build`.
5. Press "Generate", select the appropriate generator, "MinGW Makefiles".
6. Start a command shell and "make" the project:

        loudmap> cd build
        loudmap/build> mingw32-make.exe
    

You can run the unit tests using ctest on the command prompt:

    loudmap/build> ctest.exe


### Microsoft Visual Studio 2010 ###

1. Start the cmake-gui and set the code directory to the 
  library root (e.g. `/loudmap`)
2. Set the build directory to, e.g., /loudmap/build.
3. Press "Generate", select the appropriate generator, e.g. "Visual Studio 10". 
      This generates a solution file loudmap.sln
4. Load this file and build the project


You can run the unit tests using ctest on the command prompt:

    loudmap/build> ctest.exe -C Release

