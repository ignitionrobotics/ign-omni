# Connect Samples for the Omniverse Connect SDK

These samples demonstrate some key concepts for writing Omniverse Connectors and Converters. The samples use the OpenUSD and [Omniverse Connect SDK](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk) to demonstrate how to author consistent and correct USD:

- [`Omni Asset Validator`](#omni-asset-validator) - A command line validation tool.
- [`Omni CLI`](#omni-cli) - A command line utility to manage files on a Nucleus server.
- [`HelloWorld (C++ and Python)`](#helloworld-c-and-python) - A sample program that shows how to connect to an Omniverse Nucleus server, create a USD stage, create a polygonal box, bind a material, add a light, save data to .usd file, create and edit a .live layer, and send/receive messages over a channel on Nucleus. This sample is provided in both C++ and Python to demonstrate the Omniverse APIs for each language.
- [`LiveSession (C++ and Python)`](#livesession-c-and-python) - A sample program that demonstrates how to create, join, merge, and participate in live sessions. This sample is provided in both C++ and Python to demonstrate the Omniverse APIs for each language.
- [`OmniUsdaWatcher (C++)`](#omniusdawatcher-c) - A live USD watcher that outputs a constantly updating USDA file on disk.
- [`OmniSimpleSensor (C++)`](#omnisimplesensor-c) - A C++ program that demonstrates how to connect external input (e.g sensor data) to a USD layer in Nucleus.

## How to Build

### Build and CI/CD Tools
The Samples repository uses the [Repo Tools Framework (`repo_man`)](https://docs.omniverse.nvidia.com/kit/docs/repo_man) to configure premake, packman, build and runtime dependencies, testing, formatting, and other tools. Packman is used as a dependency manager for packages like OpenUSD, the Omniverse Client Library, the Omniverse USD Resolver, the Omniverse Asset Validator, and other items. The Samples use the [Connect SDK's standard repoman and packman tooling](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/docs/details.html#repoman-and-packman) as templates for including and linking against USD, Omniverse Client, etc.  These can serve as an example for the build and runtime configuration that a customer's application might require.  Here's a list of interesting files:

- [premake5.lua](./premake5.lua) - the build configuration file for the samples
- [prebuild.toml](./prebuild.toml) - consumed by the repo build tools to specify where runtime dependencies should be copied
- _build/target-deps/omni_connect_sdk/release/dev/tools/premake/connect-sdk-public.lua - the Connect SDK's build configuration template file for including USD, Omniverse Client, the Connect SDK itself, and other libraries.
  - this file isn't available until dependencies are fetched
- [source/config/omni.connect.client.toml](./source/config/omni.connect.client.toml) - A configuration file for overriding settings for logging, crash reporting, etc.  Details for how to configure this file are documented in the [Connect SDK's Core and Client Settings](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/docs/settings-config.html#core-and-client-settings).

For details on choosing and installing Connect SDK build flavors, features, or versions, see the [install_sdk](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/docs/devtools.html#repo-install-sdk) tool documentation.

### Linux
This project requires "make" and "g++".

- Open a terminal.
- To obtain "make" type `sudo apt install make` (Ubuntu/Debian), or `yum install make` (CentOS/RHEL).
- For "g++" type `sudo apt install g++` (Ubuntu/Debian), or `yum install gcc-c++` (CentOS/RHEL).

Use the provided build script to download all other dependencies (e.g USD), create the Makefiles, and compile the code.

```bash
./repo.sh build
```

Use any of the `run_*.sh` scripts (e.g. `./run_hello_world.sh`) to execute each program with a pre-configured environment.

> Tip: If you prefer to manage the environment yourself, add `<samplesRoot>/_build/linux64-x86_64/release` to your `LD_LIBRARY_PATH`.

For commandline argument help, use `--help`
```bash
./run_hello_world.sh --help
```

### Windows
#### Building
Use the provided build script to download all dependencies (e.g USD), create the projects, and compile the code.
```bash
.\repo.bat build
```

Use any of the `run_*.bat` scripts (e.g. `.\run_hello_world.bat`) to execute each program with a pre-configured environment.

For commandline argument help, use `--help`
```bash
.\run_hello_world.bat --help
```

#### Building within the Visual Studio IDE

To build within the VS IDE, open `_compiler\vs2019\Samples.sln` in Visual Studio 2019.  The sample C++ code can then be tweaked, debugged, rebuilt, etc. from there.

> Note : If the Launcher installs the Connect Samples into the `%LOCALAPPDATA%` folder, Visual Studio will not "Build" properly when changes are made because there is something wrong with picking up source changes.  Do one of these things to address the issue:
>  - `Rebuild` the project with every source change rather than `Build`
>  - Copy the Connect Samples folder into another folder outside of `%LOCALAPPDATA%`
>  - Make a junction to a folder outside of %LOCALAPPDATA% and open the solution from there:
>    - `mklink /J C:\connect-samples %LOCALAPPDATA%\ov\pkg\connectsample-202.0.0`

#### Changing the MSVC Compiler [Advanced]

When `repo.bat build` is run, a version of the Microsoft Visual Studio Compiler and the Windows 10 SDK are downloaded and referenced by the generated Visual Studio projects.  If a user wants the projects to use an installed version of Visual Studio 2019 then run `repo.bat build --use-devenv`.  Note, the build scripts are configured to tell `premake` to generate VS 2019 project files.  Some plumbing is required to support other Visual Studio versions.


## Using the Connect SDK in an Application

See the [Connect SDK Getting Started docs](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/docs/getting-started.html#integrate-connect-sdk-and-build-a-connector) for a walkthrough of how use the Connect SDK and Open USD in your application.

## Sample Details

The samples listed are focused on these key concepts:
- Omniverse
    - Asset Validator
    - Carbonite
    - Connect SDK
        - Initialization
        - Live Session Workflow
        - Logging
    - Client Library
    - USD Resolver Plugin
- OpenUSD
    - USD Cameras
    - USD Display Names
    - USD Lights
    - USD Materials
    - USD Meshes
    - USD Prim Names
    - USD Primvars
    - USD Stages
    - USD Xforms

### Omni Asset Validator
A command line USD validation tool (`omni_asset_validator.bat|sh`).

The Omniverse Asset Validator is a Python framework to provide `Usd.Stage` validation based on the [USD ComplianceChecker](https://github.com/PixarAnimationStudios/OpenUSD/blob/release/pxr/usd/usdUtils/complianceChecker.py) (i.e. the same backend as the usdchecker commandline tool), with an aim to validate assets against Omniverse specific rules to ensure they run smoothly across all Omniverse products.

[Complete Asset Validator Documentation](https://docs.omniverse.nvidia.com/kit/docs/asset-validator/latest/index.html)

To get the supported command line arguments, run omni_asset_validator.bat|sh --help. For example, the `--fix` flag will automatically apply fixes to a stage if possible (not all validation failures are automatically repairable):

```bash
omni_asset_validator.bat|sh --fix omniverse://localhost/Users/test/helloworld.usd
```

The asset validator may also be run against stages in-memory. This is demonstrated in the Python version of the LiveSession example with the v option.

### Omni CLI
A command line utility to manage files on a Nucleus Server (`omnicli.bat|sh`).

This program was initially created to exercise most of the Omniverse Client Library API, but has grown to be a useful utility to interact with Nucleus servers.  Typing `help` will produce a menu that shows the many functions available.  Among the most useful are the move/copy functions which can transfer data to and from servers.

### HelloWorld (C++ and Python)
A sample program that creates a USD stage on a Nucleus server (`run_hello_world.bat|sh` or `run_py_hello_world.bat|sh`).

The sample demonstrates how to:

- Initialize the Connect SDK Core
- Connect to a Nucleus server - by default a [`localhost Nucleus Workstation`](https://docs.omniverse.nvidia.com/nucleus/latest/workstation/installation.html)
- Create a USD stage
- Create a physics scene to define simulation parameters
- Create a polygonal box and add it to the stage and make it a dynamic rigid
- Create a cube and add it to the stage and make it a dynamic rigid
- Create a quad and add it to the stage and make it a collider
- Upload an MDL material and its textures to an Omniverse server
- Create and bind a MDL and USD Preview Surface materials to the box
- Add a distant and dome light to the stage
- Add a skinned skeletal mesh quad
- Add a folder in Nucleus - An empty folder will be generated when first creating the `HelloWorld.usd`.
- Create Nucleus checkpoints
- Move and rotate the box with live updates
- Tweak skeletal mesh animation data with live updates
- Print verbose Omniverse logs
- Open an existing stage and find a mesh to do live edits
- Send and receive messages over a channel on an Omniverse server

### LiveSession (C++ and Python)
A sample program that demonstrates how to create, join, merge, and participate in live sessions (`run_live_session.bat|sh` or `run_py_live_session.bat|sh`).

A .live layer is used in the stage's session layer to contain the changes. An Omniverse channel is used to broadcast users and merge notifications to all clients, and a session config (TOML) file is used to determine the "owner" of the session.

The sample demonstrates how to:

- Initialize the Omniverse Resolver Plugin
- Display existing live sessions for a stage
- Connect to a live session
- Make xform changes to a mesh prim in the .live layer
- Rename a prim in the .live layer
- Display the owner of the live session
- Display the current connected users/peers in the session
- Emit a GetUsers message to the session channel
- Display the contents of the session config
- Validate a stage using the Omniverse Asset Validator (Python example only)
- Merge the changes from the .live session back to the root stage
- Respond (by exiting) when another user merges session changes back to the root stage

### OmniUSDAWatcher (C++)
The Omniverse USDA Watcher is a command line program that keeps an updated USDA file on local disk that maps to a Live USD layer (.live) resident on a Nucleus server (`run_omniUsdaWatcher.bat|sh`).

It takes two arguments, the USD layer to watch and the output USDA layer:

- Acceptable forms:
    - URL: `omniverse://localhost/Users/test/helloworld.live`
    - URL: `C:\USD\helloworld.usda`
    - A local file path using shell variables: `~\helloworld.usda`
    - A relative path based on the CWD of the program: `helloworld.usda`

Note: Since the version 200.0 releases of the Connect Sample (Client Library 2.x) only `.live` layers synchronize through Nucleus.  This tool will export any supported USD file format as USDA, but if you intend to watch live file edits it must be a `.live` layer.  For more information on where to find the `root.live` layer for Live Sessions, see the [Connect SDK Live Session Configuration File Utilities](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/api/group__livesessions.html#group__livesessions_1autotoc_md4).

The "watcher" demonstrates how to:

- Initialize Omniverse
    - Set the Omniverse Client log callback (using a lambda)
    - Set the Omniverse Client log level
    - Initialize the Omniverse Client library
    - Register a connection status callback
- Open the USD stage
- Create and register the Sdf layer reload, layer change, and USD notice listeners
- Subscribe to file changes with omniClientStatSubscribe
- Start a thread that loops, receiving live changes from other clients (if the specified layer is a ``.live`` layer)
    - When the stage is modified it's written out to the specified USDA
- The main thread loops on keyboard input, waiting for a 'q' or ESC
- Cleanup the callbacks (unsubscribe and revoke)
- Destroy the stage object
- Shutdown the Omniverse Client library

For example, to monitor the stage that the HelloWorld sample creates by default in "live" mode:

```bash
run_omniUsdaWatcher.bat|sh omniverse://localhost/Users/test/helloworld.live C:\USD\helloworld.usda
```

### OmniSimpleSensor (C++)
The Omniverse Simple Sensor example demonstrates how to connect external input (e.g sensor data) to a USD layer in Nucleus(`run_omniSimpleSensor.bat|sh`).

This could effectively be a large number of inputs that report current values for IoT sensors or could be locations from robots and a real-time synchronization of the data in the virtual world is desired.

It takes three arguments, the Nucleus server path, the number of inputs and a timeout value.

```bash
run_omniSimpleSensor.bat|sh  <server path> <number of inputs> <timeout>
```

- Acceptable forms
    - Server Path: `omniverse://localhost/Users/test` (a location on a Nucleus server)
    - Number of Inputs: `4` (integer value of 1 to any number)
    - Timeout: `-1 or 20`  (-1 is for infinity to run until killed, otherwise a number in seconds)

For example, `run_omniSimpleSensor.bat omniverse://localhost/Users/test 27 -1`

There are two parts to this project.

**OmniSimpleSensor** will build a USD with a number of boxes (meshes) on one layer.

- Initialize Omniverse
- Check for an existing `SimpleSensorExample.live` stage at the `<server path>` location, if it does not exist, create it
- Edit `SimpleSensorExample.live` at `<server path>`
- Build a simple array of box meshes, starting with /World/Box_0 then /World/Box_1 and so on
- Save the Live layer
- Destroy the stage object
- Shutdown the Omniverse Client library

**OmniSensorThread** is then started for each 'input' specified in the command line.

- Initialize Omniverse
- Open `SimpleSensorExample.live` at `<server path>`
- Find the box mesh in USD this process will change
- Create a worker thread to update the box's color every 300ms
- In the main loop, wait until the timeout occurs, then
    - Stop the worker thread
    - Destroy the stage object
    - Shutdown the Omniverse Client library

For example, if 6 inputs are specified then there will be 6 OmniSensorThread processes running, independently of each other. The OmniSensorThread will launch a thread that will update the color of its assigned box at a given frequency (300ms in the code, but this can be altered). When opening the `SimpleSensorExample.live` stage in USD Composer the boxes will change colors at regular intervals.

This project can easily be extended to change the transform of the boxes or to add some metadata with a custom string attached to the USD or do change visibility states.

Some things to note here:

- Using separate processes for each input allows the USD to be changed independently
- This example could be re-written to have one main process with many worker threads launched for each input. In this case there would need to be a mutex when writing data to same live layer. This will ensure that the writing to USD via the Omniverse Client Library resource is dedicated. Wrapping the mutex around the smallest bit of code writing to USD is recommended in this case to prevent thread starvation (especially when the frequency of input is high).
- Reading USD via Omniverse Client Library does not have this issue and multiple threads in the same process can read from a USD, even the same layer in USD, without a mutex.

## Issues with Self-Signed Certs
If the scripts from the Connect Sample fail due to self-signed cert issues, a possible workaround would be to do this:

Install python-certifi-win32 which allows the windows certificate store to be used for TLS/SSL requests:

```bash
tools\packman\python.bat -m pip install python-certifi-win32 --trusted-host pypi.org --trusted-host files.pythonhosted.org
```

## External Support

First search the existing [GitHub Issues](https://github.com/NVIDIA-Omniverse/connect-samples/issues) and the [Connect Samples Forum](https://forums.developer.nvidia.com/c/omniverse/connectors/sample) to see if anyone has reported something similar.

If not, create a new [GitHub Issue](https://github.com/NVIDIA-Omniverse/connect-samples/issues/new) or forum topic explaining your bug or feature request.

- For bugs, please provide clear steps to reproduce the issue, including example failure data as needed.
- For features, please provide user stories and persona details (i.e. who does this feature help and how does it help them).

Whether adding details to an existing issue or creating a new one, please let us know what companies are impacted.


## Licenses

The license for the samples is located in [LICENSE.md](./LICENSE.md).

Third party license notices for dependencies used by the samples are located in the [Connect SDK License Notices](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/docs/licenses.html).

## Documentation and learning resources for USD and Omniverse

[OpenUSD Docs - Creating Your First USD Stage](https://openusd.org/docs/Hello-World---Creating-Your-First-USD-Stage.html)

[OpenUSD API Docs](https://openusd.org/docs/api/index.html)

[OpenUSD User Docs](https://openusd.org/release/index.html)

[OpenUSD Tutorials and Examples](https://github.com/NVIDIA-Omniverse/USD-Tutorials-And-Examples)

[OpenUSD Code Samples](https://github.com/NVIDIA-Omniverse/OpenUSD-Code-Samples)

[NVIDIA OpenUSD Docs](https://developer.nvidia.com/usd)

[Omniverse Connect SDK Docs](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk)

[Omniverse Client Library Docs](https://docs.omniverse.nvidia.com/kit/docs/client_library)

[Omniverse USD Resolver Docs](https://docs.omniverse.nvidia.com/kit/docs/usd_resolver)
