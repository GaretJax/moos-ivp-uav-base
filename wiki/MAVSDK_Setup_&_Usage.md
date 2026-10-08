
# MAVSDK Setup & Usage

> **Note:** This guide is **necessary** for building the MAVSDK library included as a submodule in moos-ivp-uav-base. MAVSDK is not built automatically with the main project. This guide also includes build steps in case you need to develop custom MAVSDK applications outside of the moos-ivp-uav-base project.

---

### Build Options 

#### Standalone build:

Clone from fork of official MAVSDK

**https:**

	git clone https://github.com/Scarter668/MAVSDK.git;
	git submodule update --init --recursive

**ssh**:

	git clone git@github.com:Scarter668/MAVSDK.git;
	git submodule update --init --recursive


**CHECKOUT** the branch `mavsdk-ardupilot`
**NOTE:** MAVSDK only supports Mavlink 2 which is backwards compatible with Mavlink 1


#### Build with moos

Read instructions [Installation_&_Setup](Installation_&_Setup.md)

### Building the library

[Official guide](https://mavsdk.mavlink.io/main/en/cpp/guide/build.html)

From the `moos-ivp-uav-base` repository root, run:

```bash
./build-mavlink.sh
./build.sh
```

`build-mavlink.sh` configures and builds the included MAVSDK submodule, including
its MAVLink dependency, and installs it into the project-local prefix
`MAVSDK/build/default/install`. It does not use `sudo` or modify `/usr/local`.
`build.sh` locates that prefix automatically, so no additional CMake options or
environment variables are required.

The MAVSDK build defaults to `Debug`. It can be overridden when necessary with
the `MAVSDK_BUILD_TYPE` environment variable, for example:

```bash
MAVSDK_BUILD_TYPE=Release ./build-mavlink.sh
```



### Alias (Optional - For VSCode Development)

The aliases in `scripts/setup_bash_aliases_moos.sh` call the same local build
script. The `bldm` alias builds MAVSDK, builds this project, and merges the
compile command databases for editor tooling; it does not require `sudo`.


### Building an example/app (Optional - For Learning MAVSDK)

Building and running MAVSDK example applications is useful for gaining hands-on experience with MAVSDK features and understanding how to interact with drones programmatically. This is **not required** for using moos-ivp-uav-base, but can help you understand the underlying MAVSDK library better.

This should be called in the folder where the .cpp`.cpp` file is defined (in MAVSDK):


Go to an example program: `cd examples/takeoff_and_land/`

```bash
cmake -Bbuild -H.;
cmake --build build -j8;
```

*Notice* the configure step `cmake -Bbuild -H.` that should be called first and only once

Summary:
```bash
cmake --build build -j8;
MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/<app> <args> 
```

where `<app>` is you MAVSDK app, eg. `fly_mission_ex`, `<arg>` is the argument of the application, eg. `udp://:14550`. To redirect the output to a file append the following `> output.log 2>&1`.

**Additional debug var:** *`MAVSDK_MESSAGE_HANDLER_DEBUGGING=1 `*

*EX:*

```bash
cmake --build build -j8; 
 MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/fly_mission_ex udp://:14550 > output.log 2>&1
```

or 
```bash
cmake --build build -j8;  MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/fly_mission_ex serial:///dev/ttySAC0:115200
```



