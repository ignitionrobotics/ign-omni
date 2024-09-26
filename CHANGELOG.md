205.0
-----
Release: May 2024

Connect SDK 1.0.0 release

* Samples
    * These samples were changed to use Connect SDK functions to create consistent and correct USD and to use the SDK to interact with the Omniverse platform:
        * HelloWorld
        * LiveSession
        * SimpleSensor
    * The OmniUsdReader was renamed to UsdTraverse and now uses the Connect SDK and has a new "Getting Started" walkthrough in the SDK docs
    * The build and runtime configuration was revamped to use the Connect SDK premake dependencies
* Omniverse Connect SDK
    * [Version 1.0.0](https://docs.omniverse.nvidia.com/kit/docs/connect-sdk/1.0.0/index.html)
* Omniverse Carbonite SDK
    * [167.3 Changes](https://docs.omniverse.nvidia.com/kit/docs/carbonite/167.3/CHANGES.html)
* Omniverse USD Resolver
    * [1.42.3 Changes](https://docs.omniverse.nvidia.com/kit/docs/usd_resolver/1.42.3/docs/changes.html)
* Omniverse Client Library
    * [2.47.1 Changes](https://docs.omniverse.nvidia.com/kit/docs/client_library/2.47.1/docs/changes.html)
* Omniverse Asset Validator
    * [0.11.1 Changes](https://docs.omniverse.nvidia.com/kit/docs/asset-validator/0.11.1/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)
* Omniverse Transcoding
    * [0.1.0 About](https://docs.omniverse.nvidia.com/kit/docs/omni-transcoding/0.1.0/index.html)

204.2
-----
Release: February 2024

Connect SDK Alpha 0.7.0 release

* Samples
    * Switch OpenUSD version to 23.11
    * Use the new OMNICONNECTCORE_INIT macro to initialize a Connector that uses OMNI logging [OM-119286]
    * Prune xform and prim utilities from Samples [OM-109464]
    * Use Connect SDK to define PBR materials and textures [OM-109462]
* Omniverse Connect SDK
    * [Version 0.7.0](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/connect-sdk/0.7.0/index.html)
* Omniverse Carbonite SDK
    * [160.8 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/carbonite/160.8/CHANGES.html)
* Omniverse USD Resolver
    * [1.42.1 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.42.1/docs/changes.html)
* Omniverse Client Library
    * [2.45.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.45.0/docs/changes.html)
* Omniverse Asset Validator
    * [0.11.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator/0.11.0/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)

204.1
-----
Release: December 2023

Connect SDK Alpha 0.6.0 release

* Samples
    * Switch OpenUSD version to 23.08
    * Add display name and valid prim name examples [OM-106962]
    * Update live samples with the SDK's LiveSession interface [OM-62756]
    * Use the SDK's defineXform function [OM-109461]
    * Use the SDK's definePolyMesh function [OM-OM-109460]
    * Add calls to the SDK's isUriWritable function [OM-109463]
    * Add version resource information to the generated executable files on Windows [OM-108758]
    * Import the SDK's connect-defaults.toml for common repo configuration [OM-113276]
    * Adjust packaging by removing prebuild samples for distribution
* Connect SDK
    * [Version 0.6.0](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/connect-sdk/0.6.0/index.html)
* Omniverse USD Resolver
    * [1.40.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.40.0/docs/changes.html)
* Omniverse Client Library
    * [2.38.12 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.38.12/docs/changes.html)
* Omniverse Asset Validator
    * [0.10.1 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator/0.10.1/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)

204.0
-----
Release: September 2023

Connect SDK Alpha 0.5.0 release

* Samples
    * HelloWorld (c++) : Added an optional UI to select the export folder (enabled via --gui or -g)
* Connect SDK
    * [Version 0.5.0](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/connect-sdk/0.5.0/index.html)

203.1
-----
Released: November 2023

* Samples
    * Add a Linux Makefile for OmniUsdReader
    * Find a new prim to move when the prim being moved by 't' is no longer valid [OMFP-2976]
    * Set the client retry count lower so accessing an invalid server returns faster [OMFP-2975]
    * Modify the client connection status callback to handle the multiple possible successful enums
    * Build the packaged binaries on Linux with an older version of GCC to support more versions [OMFP-2974]
    * Add a check in HelloWorld to exit early if it can't find the "resources" folder [OMPM-913]
    * Set the Visual Studio working directory debug settings from premake [OMPM-913]
    * Print the live session options before asking for user input the first time [OMPM-913]
    * Build both release and debug configs by default with `repo build`, `-r` or `-d` may be used to modify this [OMPM-913]

* Omniverse USD Resolver
    * [1.34.3 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.34.3/docs/changes.html)
* Omniverse Client Library
    * [2.38.9 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.38.9/docs/changes.html)
* Omniverse Asset Validator
    * [0.8.1 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator/0.8.1/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)


203.0
-----

* Samples
    * Update USD version to 22.11
    * Update Python version to 3.10
    * Add a skinned skeletal mesh example with live updates
    * Fix 21.08 include guard for UsdPhysics
    * Add channel join/send/leave commands to OmniCLI
    * Use a mesh rather than a cube to test the invalid extents since a cube always returns an extents attribute
    * Set the correct render type for the shader output attributes
    * Use an installed OmniPBR rather than including a very old version
* Omniverse USD Resolver
    * [1.27.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.27.0/docs/changes.html)
* Omniverse Client Library
    * [2.32.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.32.0/docs/changes.html)
* Omniverse Asset Validator
    * [0.6.1 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator/0.6.1/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)


202.0
-----

* Samples
    * Update the repo build tools to modern versions (OM-77486)
    * Add a test script for all programs ("repo test" or tests/test_all.[bat,sh])
    * Allow Windows users to use their installed build tools with "repo build --use-devenv" (OM-74840)
    * Change defaultPrim handling to be less hardcoded and more correct
    * Add a function to determine if any prims exist in the root layer before merging (OM-64707)
    * Add a note in README.md about Visual Studio failing to iterate on builds (OM-78756)
    * Add a prop payload and reference to a HelloWorld (OM-35218)
    * Modify an OmniPBR MDL parameter for a referenced prop
    * Set the defaultPrim to kind=assembly, all others to kind=component
    * Correctly set the USD Preview Surface bias, scale, and sourceColorSpace
    * Correctly apply the MaterialBindingAPI to the box mesh prim
    * Change OmniCLI to overwrite destination folders for copy and move (OM-80541)
* Omniverse USD Resolver
    * [1.20.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.20.0/docs/changes.html)
* Omniverse Client Library
    * [2.23.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.23.0/docs/changes.html)
* Omniverse Asset Validator
    * [0.2.2 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator/0.2.2/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)


201.0
-----

* Samples
    * Added omni_asset_validator.bat|sh which uses the [Asset Validator](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator) to find issues in USD layer file URIs
    * Added 'v' validate option for pyLiveSession to demonstrate the Asset Validator on an in-memory stage.
    * Add a C++ UsdLuxLight compatibility class to help with schema and API upgrade issues (OM-71861)
        * Rename `omniLiveSessionLib` to `omniUtilsLib`, move `cppUtils` to `omniUtilsLib/include`
    * Adjust light values from the samples
    * Cleanup the #includes in some files and remove some cruft from copy/paste
    * Create extents attributes on all meshes in HelloWorld (so we don't fail the asset validation checks)
    * Add example usage of TfMakeValidIdentifier() so prim names are valid (OM-66072)
    * Modify the build config and source to support 2022-era USD versions (OM-53579)
    * Don't throw an exception when the user name is not valid in the Python sample
    * Update msvc (OM-42026)
    * Prevent "Core::unregister callback called after shutdown" in samples by clearing the client log callback
    * Report the update frequency and run duration in the simple sensor example
    * Support reusing the live stage in the simple sensor example rather than deleting it every run
    * Make the Materials parent prim a Scope
    * Make Fieldstone.mdl use the OmniPBR.mdl that ships with the sample (OM-73976)
    * Handle the 22.08 UsdGeomPrimvarsAPI changes
* Omniverse USD Resolver
    * [1.18.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.18.0/docs/changes.html)
* Omniverse Client Library
    * [2.21.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.21.0/docs/changes.html)
* Omniverse Asset Validator
    * [0.2.1 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/asset-validator/0.2.1/source/extensions/omni.asset_validator.core/docs/CHANGELOG.html)

200.1
-----

* Samples
    * Fix the "MESSAGE" message type and custom "message" key to be case correct
    * Ensure that we flush live layer clears with omniClientLiveProcess
    * Fix the usd_resolver post build copy step in UsdReader example
* Omniverse USD Resolver
    * [1.11.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.11.0/docs/changes.html)

* Omniverse Client Library
    * [2.15.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.15.0/docs/changes.html)


200.0
-----

* Samples
    * Modify live editing behavior to use a ".live" extension
    * Add a text communication mode between clients using message channels in HelloWorld
    * Added thorough C++ USD transform handling with the xformUtils namespace
    * Change the default stage path from /localhost/Users/test to /localhost/Users/_connected_user_name (OM-53886)
* Omniverse USD Resolver
    * [1.9.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/usd_resolver/1.9.0/docs/changes.html)

* Omniverse Client Library
    * [2.13.0 Changes](http://omniverse-docs.s3-website-us-east-1.amazonaws.com/client_library/2.13.0/docs/changes.html)

104.1
-----

* Samples
    * Added thorough Python USD transform handling with the xform_utils.py module
    * Added a better logging system in the Python hello world example
    * Formatted the help text in OmniCLI to be spaced properly and to include arguments
    * Added omniSimpleSensor, a simple example of simulating sensor data pushed into a USD
    * Added omniSensorThread, this is the separate working thread to change a USD
    * Added physics to HelloWorld sample to showcase UsdPhysics usage.
    * Added the file hash to the stat output in OmniCLI
    * Added "cver" command to print client version
* Omniverse Client Library
    * 1.17.4
    * OM-47554: Downgrade some "Error" messages during auth to "Verbose"
        * We attempt to connect using 3 different methods, and it's normal for 2 of them to fail. This avoids confusing users with error messages for a connection that ultimately succeeds.
        * OM-48252: Lower required "list2" version, to allow connecting to servers running Nucleus 112.0.
    * 1.17.3
        * CC-357: Fixed a deadlock that could occur when a python file status callback is being unregistered on one thread while another thread is simultaneously trying to call that file status callback
        * CC-367: Allow stat & list using cloudfront.net URLs
        * OM-45178: Print extended error message when Token.subscribe fails
        * OM-45887: Enable extra connection logging
        * CC-384: Remove support for Nucleus 107, 109, and 110
        * CC-366: Update OpenSSL to avoid a security vulnerability
    * 1.17.2
        * CC-32: Fixed a crash that could happen on disconnect
        * OM-43009: Removed "stop" from list of required capabilities, to discover newer servers which don't advertise that capability
        * CC-236: Add support for checkpoint change notification
        * CC-245: Fixed a deadlock that could occur rarely when connection failed.
    * 1.17.1
        * CC-228: subscription based authentication using 'nonce'
        * CC-231: Discover API server using minimum required capabilities rather than all capabilites
        * CC-229: Fixed a case where the client library would not connect to Nucleus securely
    * 1.17.0
        * CC-7: Add shutdown guard to omniclient::Core


103.2
-----
* Samples
    * Change the live rotations from ZYX to XYZ and all xform ops to double3
* Omniverse Client Library
    * 1.16.0
        * OM-39826: Prevent copying of channels from nucleus servers to other providers
        * OM-38687: Fix crash when shutdown with an outstanding stat subscription
        * OM-37095: Use omniClientWait in blocking python binding functions
        * OM-38761: Fix ResolveSubscribe to handle more invalid search paths
        * OM-39746: Support stat('/') on S3 buckets. S3 API does not support stat() on root, but we can fill in the gaps
    * 1.15.0
        * OM-39614: Fixed a case where pings would not report a connection error
        * OM-36524: update connection library to prevent using Nucleus Cache when accessing localhost servers
        * OM-37061: Fix crash if a request is started after it is stopped
        * OM-34916: Map MountExistsUnderPath error to ErrorNotSupported
        * OM-38761: Fixed StatImpl::beginWatch to always call the provided callback, even when the uri to watch is invalid.
        * OM-39367: Removed "fast locked updates" because it's fundamentally broken.
            * Removes omniUsdLiveLock & omniUsdLiveUnlock
        * OM-23042: If the relativePath starts with "./" then don't use search paths
    * 1.14.0
        * OM-38721: Added function omniClientGetLocalFile which returns the local filename for a URL
            * If the input URL is already a local file, it is returned without error
            * If the remote file has not yet been downloaded to cache, it is downloaded before returning
            * Python bindings are "get_local_file" "get_local_file_async" and "get_local_file_with_callback"
        * OM-38816: Added environment variable OMNI_DEPLOYMENT which can override the deployment sent to discovery
        * OM-37061: Early exit from callback if cacheEntry weak_ptr is expired, correct creation of cacheEntry ownership
    * 1.13.25
        * OM-34145: Fix omniClientCopy to not infinitely copy when copying a directory into a subdirectory of itself.
    * 1.13.24
        * OM-38028: Update Brotli, OpenSSL, and libcurl versions
    * 1.13.23
        * OM-37701: Fix FetchToLocalResolvedPath to work with SdfFileFormat arguments
    * 1.13.22
        * OM-37276: Use latest idl.cpp to pickup SSL cert directory location fixes
    * 1.13.21
        * OM-36064 & OM-36306: Fix crash in listSubscribe on disconnect
    * 1.13.20
        * OM-37054: Fix incorrect search order according to PBR specification
        * OM-36511: Add python bindings set_authentication_message_box_callback & authentication_cancel


103.1
-----
* Samples
    * Added omniUsdReader, a very very simple program for build config demonstration that opens a stage and traverses it, printing all of the prims
    * Added omniUsdaWatcher, a live USD watcher that outputs a constantly updating USDA file on disk
    * Updated the nv-usd library to one with symbols so the Visual Studio Debug Visualizers work properly
* Omniverse Client Library
    * Still using 1.13.19

102.1
-----
* Samples
    * OM-31648: Add a windows build tool configuration utility if the user wants to use an installed MSVC and the Windows SDK
    * Add a dome light with texture to the stage
    * OM-35991: Modify the MDL names and paths to reduce some code redundancy based on a forum post
    * Add Nucleus checkpoints to the Python sample
    * Avoid writing Nucleus checkpoints when live mode is enabled, this isn't supported properly
    * OM-37005: Fix a bug in the Python sample batch file if the sample was installed in a path with spaces
    * Make the /Root prim the `defaultPrim`
    * Update Omniverse Client Library to 1.13.19
* Omniverse Client Library
    * 1.13.19
        * OM-36925: Fix omniClientMakeRelative("omni://host/path/", "omni://host/path/");
    * 1.13.18
        * OM-25931: Fixed some issues around changing and calling the log callback to reduce hangs.
        * OM-36755: Fixed possible use-after-delete issue with set_log_callback (Python).
    * 1.13.17
        * OM-34879: Hard-code "mdl" as "not a layer" to work around a problem that happens if the "usdMdl" plugin is loaded
    * 1.13.16
        * OM-36756: Fix crash that could happen if two threads read a layer at the exact same time.
    * 1.13.15
        * OM-35235: Fix various hangs by changing all bindings to release the GIL except in very specific cases.
    * 1.13.14
        * OM-34879: Fix hang in some circumstances by delaying USD plugin registration until later
        * OM-33732: Remove USD diagnostic delegate
    * 1.13.13
        * OM-36256: Fixed S3 provider from generating a bad AWS signature when Omni Cache is enabled
    * 1.13.12
        * OM-20572: Fixed setAcls
    * 1.13.11
        * OM-35397: Fixed a bug that caused Linux's File Watcher Thread to peg the CPU in some cases.
    * 1.13.10
        * OM-32244: Fixed a very rare crash that could occur when reading a local file that another process has locked
    * 1.13.9
        * OM-35050: Fixed problem reloading a non-live layer after it's been modified.
    * 1.13.8
        * OM-34739: Fix regression loading MDLs introduced in     * 1.13.3
        * OM-33949: makeRelativeUrl prepends "./" to relative paths
        * OM-34752: Make sure local paths are always using "" inside USD on Windows
    * 1.13.7
        * OM-34696: Fixed bug when S3 + cloudfront + omni cache are all used
    * 1.13.6
        * OM-33914: Fixed crash when accessing http provider from mulitple threads simultaneously
    * 1.13.5
        * OM-26039: Fixed "Restoring checkpoint while USD stage is opened live wipes the content"
        * OM-33753: Fixed "running massive amounts of live edits together causes massive amounts of checkpoints"
        * OM-34432: Fixed "[Create] It will lose all data or hang Create in live session"
            * These were all the same underlying issue: When a layer is overwritten in live mode it was cleared and set as 'dirty' which would cause the next "Save()" (which happens every frame in live mode) to save the cleared layer back to the Omniverse server.
    * 1.13.4
        * OM-31830: omniClientCopy() with HTTP/S3 provider as source
        * OM-33321: Use Omni Cache 2.4.1+ new reverse proxy feature for HTTPS caching
    * 1.13.3
        * OM-33483: Don't crash when trying to save a layer that came from a mount
        * OM-27233: Support loading non-USD files (abc, drc, etc)
        * OM-4613 & OM-34150: Support saving usda files as ascii
            * Note this change means live updates no longer work with usda files (though they technically never did -- it would silently convert them to usdc files).

101.1
-----
* Add Linux package for the Omniverse Launcher
* Add a python 3 Hello World sample
* Update the Omniverse Client Library to 1.13.2
* Update to Python 3.7
* Add a Nucleus Checkpoint example
* Add the ability to create/access a USD stage on local disk in the Hello World sample

100.2
-----
* Update the Omniverse Client Library fix an issue with overlapping file writes

100.1
-----
* First release
* HelloWorld sample that demonstrates how to:
    * connect to an Omniverse server
    * create a USD stage
    * create a polygonal box and add it to the stage
    * upload an MDL material and its textures to an Omniverse server
    * bind an MDL and USD Preview Surface material to the box
    * add a light to the stage
    * move and rotate the box with live updates
    * print verbose Omniverse logs
    * open an existing stage and find a mesh to do live edits
* OmniCLI sample that exercises most of the Omniverse Client Library API
