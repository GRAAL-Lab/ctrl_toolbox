# Control Toolbox

`ctrl_toolbox` is a C++17 library collecting reusable control-oriented building blocks under the `ctb` namespace.

## What is included

1. `ctb::DigitalPID`
	1-D digital PID controller with anti-windup and configurable error function.
2. `ctb::VirtualFrame`
	Virtual-frame based smoothing for Cartesian pose/velocity references.
3. `ctb::ExtendedKalmanFilter`
	EKF framework with pluggable model and measurement components.
4. Utility helpers
	Angle wrapping, geodesy conversions (`LatLong`/local frames), and configuration helpers.

## Requirements

- CMake >= 3.2
- C++17 compiler
- `GeographicLib`
- `libconfig++`
- `rml` (Robotic Mathematical Library): <https://bitbucket.org/isme_robotics/rml/src>

On Debian/Ubuntu, install system dependencies with:

```bash
sudo apt-get update
sudo apt-get install -y cmake g++ libgeographiclib-dev libconfig++-dev
```

`rml` must be installed separately and discoverable by your compiler/linker.

## Build

From the project root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
```

To also compile the test executable:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTS=ON
cmake --build build -j
```

## Install

```bash
cmake --install build
```

The project exports a CMake package configuration (`ctrl_toolboxConfig.cmake`) under:

- `share/ctrl_toolbox/cmake`

## Use from another CMake project

```cmake
find_package(ctrl_toolbox REQUIRED)

add_executable(my_controller main.cpp)
target_link_libraries(my_controller PRIVATE ctrl_toolbox)
```

## Public headers

Use the umbrella include:

```cpp
#include <ctrl_toolbox/ctrl_toolbox.hpp>
```

or include individual modules, for example:

- `<ctrl_toolbox/pid/DigitalPID.h>`
- `<ctrl_toolbox/virtual_frame/VirtualFrame.h>`
- `<ctrl_toolbox/kalman_filter/ExtendedKalmanFilter.h>`
- `<ctrl_toolbox/HelperFunctions.h>`

## Minimal PID example

```cpp
#include <ctrl_toolbox/pid/DigitalPID.h>

int main()
{
	 ctb::PIDGains gains;
	 gains.Kp = 2.0;
	 gains.Ki = 0.4;
	 gains.Kd = 0.05;
	 gains.Kff = 0.0;
	 gains.N = 10.0;
	 gains.Tr = 0.2;

	 ctb::DigitalPID pid(gains, 0.01, 5.0);

	 const double reference = 1.0;
	 const double feedback = 0.8;
	 const double command = pid.Compute(reference, feedback);

	 (void)command;
	 return 0;
}
```

## Notes

- The library target is currently built as a shared library.
- Most APIs are defined in namespace `ctb`.
- The current repository includes a sample test executable in `test/ctrl_toolbox_test.cc`.

## License

This project is released under the MIT License. See [LICENSE.md](LICENSE.md).

## Maintainer

Maintained by [GRAAL Laboratory](https://www.graal.dibris.unige.it), University of Genoa (Italy).
