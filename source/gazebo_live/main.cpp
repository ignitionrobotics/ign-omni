/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "GetOp.hpp"
#include "OmniverseConnect.hpp"
#include "Scene.hpp"
#include "SetOp.hpp"
#include "ThreadSafe.hpp"

#include <gz/common/Console.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/StringUtils.hh>

#include <gz/utils/cli.hh>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include <string>

using namespace gz::omniverse;

constexpr double kTargetFps = 60;
constexpr std::chrono::duration<double> kUpdateRate(1 / kTargetFps);

int main(int argc, char* argv[])
{
  CLI::App app("gz omniverse connector");

  std::string destinationPath;
  app.add_option("-p,--path", destinationPath,
                 // clang-format off
                 "Location of the omniverse stage. e.g. \"omniverse://localhost/Users/gz/stage.usd\"")
      // clang-format on
      ->required();
  std::string worldName;
  gz::omniverse::Simulator simulatorPoses{
    gz::omniverse::Simulator::gz};
  app.add_option("-w,--world", worldName, "Name of the gz world")
      ->required();

  std::map<std::string, gz::omniverse::Simulator> map{
    {"gz", gz::omniverse::Simulator::gz},
    {"isaacsim", gz::omniverse::Simulator::IsaacSim}};
  app.add_option("--pose", simulatorPoses, "Which simulator will handle the poses")
      ->required()
      ->transform(CLI::CheckedTransformer(map, CLI::ignore_case));;
  app.add_flag_callback("-v,--verbose",
                        []() { gz::common::Console::SetVerbosity(4); });

  CLI11_PARSE(app, argc, argv);

  std::string ignGazeboResourcePath;
  auto systemPaths = gz::common::systemPaths();
  gz::common::env("GZ_GAZEBO_RESOURCE_PATH", ignGazeboResourcePath);
  for (const auto& resourcePath :
       gz::common::Split(ignGazeboResourcePath, ':'))
  {
    systemPaths->AddFilePaths(resourcePath);
  }

  // Connect with omniverse
  if (!StartOmniverse())
  {
    ignerr << "Not able to start Omniverse" << std::endl;
    return -1;
  }

  // Open the USD model in Omniverse
  const std::string stageUrl = [&]()
  {
    auto result = CreateOmniverseModel(destinationPath);
    if (!result)
    {
      ignerr << result.Error() << std::endl;
      exit(-1);
    }
    return result.Value();
  }();

//  omniUsdLiveSetModeForUrl(stageUrl.c_str(),
//                           OmniUsdLiveMode::eOmniUsdLiveModeEnabled);

  PrintConnectedUsername(stageUrl);

  Scene scene(worldName, stageUrl, simulatorPoses);
  if (!scene.Init())
  {
    return -1;
  };

  auto lastUpdate = std::chrono::steady_clock::now();
  // don't spam the console, show the fps only once a sec
  auto nextShowFps =
      lastUpdate.time_since_epoch() + std::chrono::duration<double>(1);

  while (true)
  {
    std::this_thread::sleep_for((lastUpdate + kUpdateRate) -
                                std::chrono::steady_clock::now());
    auto now = std::chrono::steady_clock::now();
    if (now.time_since_epoch() > nextShowFps)
    {
      double curFps =
          1 / std::chrono::duration<double>(now - lastUpdate).count();
      nextShowFps = now.time_since_epoch() + std::chrono::duration<double>(1);
      igndbg << "fps: " << curFps << std::endl;
    }
    lastUpdate = now;

    scene.Save();
   // omniUsdLiveProcess();
    omniClientLiveProcess();
  }

  return 0;
}
