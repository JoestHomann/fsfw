<img align="center" src="./images/cmake.png" width="25%">

<sub><sup>Image taken from [Wikipedia](https://commons.wikimedia.org/wiki/File:Cmake.svg) 
and licensed under [Creative Commons 2.0](https://creativecommons.org/licenses/by/2.0/deed.en), 
no changes made</sup></sub>

CMake is a modern cross-platform build system which is able to generate
various build systems. It also features a dependency management system which
allows developers to operate on targets (e.g. compile target as a library, link
against a target) which allows better control of build properties compared
to tools like Make.

## Building with CMake

Generally, building software with CMake is a two-step process.
First, the build configuration is set up using the CMake build system or IDE project 
generators and then the software is built using the select build system or IDE.
CMake projects are generally built out-of-source which means that the files generated
during the build process are kept separately from the source tree. This generally involves
creating a build folder like `build-Debug` or `build-Release` and then performing all
steps inside that folder.

It is also possible to generate IDE project files with CMake. This is 
not recommended for Eclipse because the CDT generation is not very good. 
Instead, it is recommended to configure the build system once in the command line and then 
invoke the CMake build command from Eclipse.
Script files were supplied in the `buildsystem` folder to have a starting point.

It is also possible to generate Visual Studio files but this has not been tested extensively yet.

It is possible to perform the build configuration steps with the
`cmake-gui` or with the curses `ccmake` command line utility. This also provides a graphical displayed
of available options and variables.

## Build Configuration options in CMake

Call `cmake --help` to get a first overview of how the CMake build configuration
works. Generally, build options can be displayed by running following command:

```sh
cmake -LA <path-to-source>
```

The general form to configure a build system is to call this command
in the folder where the build system should be set up (this is generally not
in a separate folder to avoid pollution of the source tree).

The generators for the host system can be displayed with `cmake --help` as well
and are supplied with `-G` to the build configuration. 
Please note that the OSAL and architecture specific READMEs contain the 
explicit commands to configure the build systems correctly.

```sh
cmake -G <Build Generator> <Options and Defines> <path-to-source>
```

Following build configurations are possible by setting the `CMAKE_BUILD_TYPE`
string when configuring the build system. Supply `-DCMAKE_BUILD_TYPE=<option>` 
to do this:

1. `None`: No flags added
1. `Debug`: Default type if no build type is specified 
2. `RelWithDebInfo`: Release build, but debug symbols included 
3. `MinSizeRel`: Build for minimum size 
4. `Release`: Build for maximum speed

For more information, see the [CMake website](https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/Useful-Variables#compilers-and-tools)

The FSFW OSAL can be specified with the `OS_FSFW` define during build configuration
Supply `-DOS_FSFW=<option>` to the configuration to do this.
Possible options are:

1. `host`: Host OSAL, tested for Windows 10 and Linux (Ubuntu 20.04)
2. `linux`: Linux OSAL, tested for regular Linux (Ubuntu 20.04) and embedded Linux
3. `freertos`: FreeRTOS OSAL, example for the STM32H743ZI-Nucleo development board provided.
4. `rtems`: Currently, no example provided, but will be provided for STM32H743ZI-Nucleo board.
