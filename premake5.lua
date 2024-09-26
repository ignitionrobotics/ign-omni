-- Shared build scripts from repo_build package
repo_build = require("omni/repo/build")
repo_build.root = os.getcwd()

-- Shared build scripts for use in client Connectors
connect_build = require(path.replaceextension(os.matchfiles("_build/target-deps/omni_connect_sdk/*/dev/tools/premake/connect-sdk-public.lua")[1], ""))

local targetDepsDir = "_build/target-deps"

workspace "Samples"
    connect_build.setup_workspace({
        msvc_version = "14.29.30133",
        winsdk_version = "10.0.18362.0",
    })

    -- override some connect_build settings
    -- install exeutables and libraries in the main target_build_dir
    target_bin_dir = target_build_dir
    target_lib_dir = target_build_dir

    configurations { "debug", "release" }
    platforms { "x86_64" }
    architecture "x86_64"

    -- common dir name to store platform specific files
    local platform = "%{cfg.system}-%{cfg.platform}"

    local targetName = _ACTION
    local workspaceDir = "_compiler/"..targetName

    local gazeboInstallDir = "../../install"

    print('gazeboInstallDir', gazeboInstallDir)

    local targetDir = "_build/"..platform.."/%{cfg.buildcfg}"

    -- adding dependencies
    filter { "system:linux" }
        linkoptions { '-Wl,--disable-new-dtags -Wl,-rpath,../../../_build/target-deps/nv_usd/%{cfg.buildcfg}/lib:../../../_build/target-deps/omni_client_library/%{cfg.buildcfg}:../../../_build/target-deps/python/lib:' }
        includedirs { targetDepsDir.."/nv_usd/%{cfg.buildcfg}/include", targetDepsDir.."/omni_client_library/include", targetDepsDir.."/python/include/python3.7m",
                      gazeboInstallDir.."/include",
                      gazeboInstallDir.."/include/gz/math7",
                      gazeboInstallDir.."/include/gz/utils2",
                      gazeboInstallDir.."/include/gz/transport13",
                      gazeboInstallDir.."/include/gz/msgs10",
                      gazeboInstallDir.."/include/gz/cmake3",
                      gazeboInstallDir.."/include/gz/common5",
                      gazeboInstallDir.."/include/gz/sdformat14" }
        libdirs { targetDepsDir.."/nv_usd/%{cfg.buildcfg}/lib", targetDepsDir.."/omni_client_library/%{cfg.buildcfg}", targetDepsDir.."/python/lib",
        gazeboInstallDir.."/lib" }
    filter { "configurations:debug" }
        defines { "DEBUG", "NOMINMAX" }
        optimize "Off"
        runtime "Debug"
    filter { "configurations:release" }
        defines { "NDEBUG", "NOMINMAX" }
        optimize "On"
        runtime "Release"
    filter {}

    location (workspaceDir)
    targetdir (targetDir)

    -- symbolspath ("_build/"..targetName.."/symbols/%{cfg_buildcfg}/%{prj.name}.pdb")
    objdir ("_build/intermediate/"..platform.."/%{prj.name}")
    symbols "On"
    exceptionhandling "On"
    rtti "On"
    staticruntime "Off"
    cppdialect "C++17"

    filter { "system:linux" }
        buildoptions {"-D_MSC_VER=0 -D_GLIBCXX_USE_CXX11_ABI=0 -Wno-deprecated-declarations -Wno-deprecated -Wno-unused-variable -pthread -lstdc++fs"}
    filter {}


function sample(projectName, sourceFolder)
    project(projectName)
    kind "ConsoleApp"
    optimize "Size"
    intrinsics "off"
    inlining "Explicit"
    flags { "NoManifest", "NoIncrementalLink", "NoPCH" }

    includedirs { "source/common/include" }

    -- setup all paths, links, and carb dependencies to enable omni_connect_core
    connect_build.use_omni_client()
    connect_build.use_omni_resolver()
    connect_build.use_usd({
        "arch",
        "gf",
        "kind",
        "pcp",
        "plug",
        "sdf",
        "tf",
        "usd",
        "usdGeom",
        "usdLux",
        "usdPhysics",
        "usdShade",
        "usdSkel",
        "usdUtils",
        "vt",
    })
    connect_build.use_connect_core()
    filter { "system:linux" }
        links { "ar","arch","gf","js","kind","pcp","plug","sdf","tf","trace","usd","usdGeom", "vt","work","usdShade","usdLux","omniclient","python3.7m","boost_python37", "pthread", "stdc++fs",
                "gz-math7", "gz-utils2", "gz-common5", "gz-common5-graphics", "gz-transport13", "gz-msgs10", "protobuf", "protoc", "sdformat14" }
    filter {}
    location (workspaceDir.."/%{prj.name}")
    files { "source/"..sourceFolder.."/**.*" }
    filter { "system:windows" }
        links { "shlwapi" }
    filter {}

    connect_build.executable({
        name = projectName,
        sources = { "source/"..sourceFolder.."/**.*" },
    })
end


sample("gazebo-omniverse1", "gazebo_live")
