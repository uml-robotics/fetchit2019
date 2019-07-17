# fetch_cpp

A C++ library for Fetch.

## Implemented APIs

Head interface ([Fetch docs](http://docs.fetchrobotics.com/api_overview.html#head-interface))

- [HeadClient.h](include/fetch_cpp/HeadClient.h)
- [PointHeadClient.h](include/fetch_cpp/PointHeadClient.h)

Torso lift controller ([Fetch docs](http://docs.fetchrobotics.com/api_overview.html#arm-and-torso))

- [TorsoLiftClient.h](include/fetch_cpp/TorsoLiftClient.h)

Gripper interface ([Fetch docs](http://docs.fetchrobotics.com/api_overview.html#gripper-interface))

- [FetchGripper.h](include/fetch_cpp/FetchGripper.h)

## Usage

Add `fetch_cpp` catkin package in `CMakeLists.txt`, :

```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  fetch_cpp # newly added
)
```

Add dependency in `package.xml`, append the following to similar tags:

```xml
<build_depend>fetch_cpp</build_depend>
<build_export_depend>fetch_cpp</build_export_depend> 
```

Note: The change in `package.xml` is needed in case `fetch_cpp` is not compiled.

In your C++ source file, e.g., :

```cpp
#include <fetch_cpp/FetchGripper.h>

FetchGripper gripper;
gripper.close();
```