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

#include "FUSDLayerNoticeListener.hpp"

#include <memory>
#include <string>

#include <gz/transport/Node.hh>

namespace gz
{
namespace omniverse
{
class FUSDLayerNoticeListener::Implementation
{
public:
  std::shared_ptr<ThreadSafe<pxr::UsdStageRefPtr>> stage;
  std::string worldName;
  gz::transport::Node node;
};

FUSDLayerNoticeListener::FUSDLayerNoticeListener(
  std::shared_ptr<ThreadSafe<pxr::UsdStageRefPtr>> &_stage,
  const std::string& _worldName)
    : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->stage = _stage;
  this->dataPtr->worldName = _worldName;
}

void FUSDLayerNoticeListener::HandleGlobalLayerReload(
  const pxr::SdfNotice::LayerDidReloadContent& /*n*/)
{
  gzdbg << "HandleGlobalLayerReload called" << std::endl;
}

// Print some interesting info about the LayerNotice
void FUSDLayerNoticeListener::HandleRootOrSubLayerChange(
    const class pxr::SdfNotice::LayersDidChangeSentPerLayer& _layerNotice,
    const pxr::TfWeakPtr<pxr::SdfLayer>& _sender)
{
  auto iter = _layerNotice.find(_sender);
  for (auto & changeEntry : iter->second.GetEntryList())
  {
    const pxr::SdfPath& sdfPath = changeEntry.first;

    if (changeEntry.second.flags.didRemoveNonInertPrim)
    {
      gz::msgs::Entity req;
      req.set_name(sdfPath.GetName());
      req.set_type(gz::msgs::Entity::MODEL);

      gz::msgs::Boolean rep;
      bool result;
      unsigned int timeout = 5000;
      bool executed = this->dataPtr->node.Request(
        "/world/" + this->dataPtr->worldName + "/remove",
        req, timeout, rep, result);
      if (executed)
      {
        if (rep.data())
        {
          gzdbg << "Model was removed [" << sdfPath.GetName() << "]"
                 << std::endl;
          this->dataPtr->stage->Lock()->RemovePrim(sdfPath);
        }
        else
        {
          gzerr << "Error model was not removed [" << sdfPath.GetName()
                 << "]" << std::endl;
        }
      }
      gzmsg << "Deleted " << sdfPath.GetName() << std::endl;
    }
    else if (changeEntry.second.flags.didAddNonInertPrim)
    {
      gzmsg << "Added" << sdfPath.GetName() << std::endl;
    }
  }
}

}
}