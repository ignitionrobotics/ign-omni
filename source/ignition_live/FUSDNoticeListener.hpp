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
#ifndef IGNITION_OMNIVERSE_FUSDNOTICELISTENER_HPP
#define IGNITION_OMNIVERSE_FUSDNOTICELISTENER_HPP

#include "GetOp.hpp"

#include <ignition/common/Console.hh>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/notice.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>

#include <ignition/transport/Node.hh>

namespace ignition
{
namespace omniverse
{
class FUSDNoticeListener : public pxr::TfWeakBase
{
 public:
  FUSDNoticeListener(
    std::shared_ptr<ThreadSafe<pxr::UsdStageRefPtr>> _stage,
    const std::string &_worldName)
      : stage(_stage), worldName(_worldName)
  {
  }

  void ParseCube(const pxr::UsdPrim &_prim, std::string &_stringSDF)
  {
    double size;
    auto variant_cylinder = pxr::UsdGeomCube(_prim);
    variant_cylinder.GetSizeAttr().Get(&size);

    double x, y, z;

    // TODO(ahcorde): Fix scale
    x = size;  // * _scale.X(),
    y = size;  // * _scale.Y(),
    z = size;  // * _scale.Z()

    _stringSDF +=
        "\t\t\t<visual name='" + _prim.GetPath().GetName() + "_visual'>\n";
    _stringSDF += "\t\t\t\t<geometry>\n";
    _stringSDF += "\t\t\t\t\t<box>\n";
    _stringSDF += "\t\t\t\t\t\t<size>" + std::to_string(x) + " " +
                  std::to_string(y) + " " + std::to_string(z) + "</size>\n";
    _stringSDF += "\t\t\t\t\t</box>\n";
    _stringSDF += "\t\t\t\t</geometry>\n";
    _stringSDF += "\t\t\t</visual>\n";

    _stringSDF += "\t\t\t<collision name='" + _prim.GetPath().GetName() +
                  "_collision'>\n";
    _stringSDF += "\t\t\t\t<geometry>\n";
    _stringSDF += "\t\t\t\t\t<box>\n";
    _stringSDF += "\t\t\t\t\t\t<size>" + std::to_string(x) + " " +
                  std::to_string(y) + " " + std::to_string(z) + "</size>\n";
    _stringSDF += "\t\t\t\t\t</box>\n";
    _stringSDF += "\t\t\t\t</geometry>\n";
    _stringSDF += "\t\t\t</collision>\n";
  }

  void ParseCylinder(const pxr::UsdPrim &_prim, std::string &_stringSDF)
  {
    auto variant_cylinder = pxr::UsdGeomCylinder(_prim);
    double radius;
    double height;
    variant_cylinder.GetRadiusAttr().Get(&radius);
    variant_cylinder.GetHeightAttr().Get(&height);

    _stringSDF +=
        "\t\t\t<visual name='" + _prim.GetPath().GetName() + "_visual'>\n";
    _stringSDF += "\t\t\t\t<geometry>\n";
    _stringSDF += "\t\t\t\t\t<cylinder>\n";
    _stringSDF +=
        "\t\t\t\t\t\t<radius>" + std::to_string(radius) + "</radius>\n";
    _stringSDF +=
        "\t\t\t\t\t\t<height>" + std::to_string(height) + "</height>\n";
    _stringSDF += "\t\t\t\t\t</cylinder>\n";
    _stringSDF += "\t\t\t\t</geometry>\n";
    _stringSDF += "\t\t\t</visual>\n";

    _stringSDF += "\t\t\t<collision name='" + _prim.GetPath().GetName() +
                  "_collision'>\n";
    _stringSDF += "\t\t\t\t<geometry>\n";
    _stringSDF += "\t\t\t\t\t<cylinder>\n";
    _stringSDF +=
        "\t\t\t\t\t\t<radius>" + std::to_string(radius) + "</radius>\n";
    _stringSDF +=
        "\t\t\t\t\t\t<height>" + std::to_string(height) + "</height>\n";
    _stringSDF += "\t\t\t\t\t</cylinder>\n";
    _stringSDF += "\t\t\t\t</geometry>\n";
    _stringSDF += "\t\t\t</collision>\n";
  }

  void ParseSphere(const pxr::UsdPrim &_prim, std::string &_stringSDF)
  {
    double radius;
    auto variant_sphere = pxr::UsdGeomSphere(_prim);
    variant_sphere.GetRadiusAttr().Get(&radius);
    _stringSDF +=
        "\t\t\t<visual name='" + _prim.GetPath().GetName() + "_visual'>\n";
    _stringSDF += "\t\t\t\t<geometry>\n";
    _stringSDF += "\t\t\t\t\t<sphere>\n";
    _stringSDF +=
        "\t\t\t\t\t\t<radius>" + std::to_string(radius) + "</radius>\n";
    _stringSDF += "\t\t\t\t\t</sphere>\n";
    _stringSDF += "\t\t\t\t</geometry>\n";
    _stringSDF += "\t\t\t</visual>\n";

    _stringSDF += "\t\t\t<collision name='" + _prim.GetPath().GetName() +
                  "_collision'>\n";
    _stringSDF += "\t\t\t\t<geometry>\n";
    _stringSDF += "\t\t\t\t\t<sphere>\n";
    _stringSDF +=
        "\t\t\t\t\t\t<radius>" + std::to_string(radius) + "</radius>\n";
    _stringSDF += "\t\t\t\t\t</sphere>\n";
    _stringSDF += "\t\t\t\t</geometry>\n";
    _stringSDF += "\t\t\t</collision>\n";
  }

  void CreateSDF(std::string &_stringSDF, const pxr::UsdPrim &_prim)
  {
    if (!_prim)
      return;
    auto children = _prim.GetChildren();
    for (const pxr::UsdPrim &childPrim : children)
    {
      igndbg << childPrim.GetPath().GetText() << "\n";
      if (!childPrim)
        continue;
      if (childPrim.IsA<pxr::UsdGeomSphere>())
      {
        ParseSphere(childPrim, _stringSDF);
      }
      else if (childPrim.IsA<pxr::UsdGeomCylinder>())
      {
        ParseCylinder(childPrim, _stringSDF);
      }
      else if (childPrim.IsA<pxr::UsdGeomCube>())
      {
      }
      else
      {
        CreateSDF(_stringSDF, childPrim);
      }
    }
  }

  void Handle(const class pxr::UsdNotice::ObjectsChanged &ObjectsChanged)
  {
    for (const pxr::SdfPath &objectsChanged : ObjectsChanged.GetResyncedPaths())
    {
      ignmsg << "Resynced Path: " << objectsChanged.GetText() << std::endl;
      auto stage = this->stage->Lock();
      auto modelUSD = stage->GetPrimAtPath(objectsChanged);
      if (modelUSD)
      {
        std::string strPath = objectsChanged.GetText();
        if (strPath.find("_link") != std::string::npos
           || strPath.find("_visual") != std::string::npos
           || strPath.find("geometry") != std::string::npos) {
          return;
        }
      	std::string sdfString = std::string("<sdf version='1.7'>\n");

      	sdfString += std::string("\t<model name='") +
      		modelUSD.GetPath().GetName() + std::string("'>\n");

      	sdfString += "\t\t<pose>" +
          std::to_string(3) + " " +
          std::to_string(3) + " " +
          std::to_string(0.5) + " " +
          std::to_string(0) + " " +
          std::to_string(0) + " " +
          std::to_string(0) + "</pose>\n";

      	sdfString += "\t\t<link name='" + modelUSD.GetPath().GetName() + "_link'>\n";

      	CreateSDF(sdfString, modelUSD);

      	sdfString += "\t\t</link>\n";

      	sdfString += std::string("\t</model>\n</sdf>\n");

      	std::cerr << sdfString;

      	// Prepare the input parameters.
      	ignition::msgs::EntityFactory req;
      	req.set_sdf(sdfString);
      	req.set_name(modelUSD.GetPath().GetName());
      	req.set_allow_renaming(false);

        ignition::msgs::Boolean rep;
        bool result;
        unsigned int timeout = 5000;
        bool executed = node.Request(
          "/world/" + this->worldName + "/create",
          req, timeout, rep, result);
      	if (executed)
        {
      		if (rep.data())
      		{
      			std::cerr << "Model was inserted [" << modelUSD.GetPath().GetName()
                      << "]" << '\n';
      		}
      		else
      		{
      			std::cerr << "Error model was not inserted" << '\n';
      		}
      	}
      }
    }

    ignition::msgs::Pose_V req;

    {
      // Right now, there is no way to get the joint angle from Issac Sim
      // We can get the rotation of the link


      // this loop checks all paths to find revolute joints
      // if there is some, we get the body0 and body1 and calculate the
      // joint angle.

      auto stage = this->stage->Lock();
      auto range = pxr::UsdPrimRange::Stage(*stage);
      for (auto const &prim : range)
      {
        std::string primType = prim.GetPrimTypeInfo().GetTypeName().GetText();
        if (primType == std::string("PhysicsRevoluteJoint"))
        {
          auto attrTargetPos = prim.GetAttribute(
              pxr::TfToken("drive:angular:physics:targetPosition"));

          auto relBody0 = prim.GetRelationships();
          std::vector<ignition::math::Quaterniond> qVector;
          for (auto & rel : relBody0)
          {
            pxr::SdfPathVector paths;
            rel.GetTargets(&paths);
            for (auto p: paths)
            {
              auto modelUSD = stage->GetPrimAtPath(p);
              auto xform = pxr::UsdGeomXformable(modelUSD);
              auto transforms = GetOp(xform);
              qVector.emplace_back(
                transforms.rotQ.GetReal(),
                transforms.rotQ.GetImaginary()[0],
                transforms.rotQ.GetImaginary()[1],
                transforms.rotQ.GetImaginary()[2]);
              // std::cerr << "pxr::TfStringify(p) " << pxr::TfStringify(p) << '\n';
            }
          }

          double angle1, angle2, angleDiff;
          ignition::math::Vector3d axis1, axis2, axisDiff;
          qVector[0].ToAxis(axis1, angle1);
          qVector[1].ToAxis(axis2, angle2);
          axis2.Normalize();
          axis1.Normalize();
          auto qdiff = qVector[1] - qVector[0];
          qdiff.ToAxis(axisDiff, angleDiff);
          //
          // std::cerr << "axis1 " << axis1 << " " << qVector[0] << '\n';
          // std::cerr << "axis2 " << axis2 << " " << qVector[1] << '\n';
          // std::cerr << "axisDiff " << axisDiff << " " << qdiff << '\n';
          // std::cerr << acos(axisDiff.Dot(ignition::math::Vector3d(0, 0, 1))) << '\n';
          // auto attrBody0 = prim.GetAttribute(
          //     pxr::TfToken("physics:body1"));
          float pos = acos(axisDiff.Dot(ignition::math::Vector3d(0, 0, 1)));
          if (attrTargetPos)
          {
            // Subscribe to commands
            std::string topic = transport::TopicUtils::AsValidTopic(
              std::string("/model/") + std::string("panda") +
              std::string("/joint/") + prim.GetPath().GetName() +
              std::string("/0/cmd_pos"));

            auto pub = revoluteJointPublisher.find(topic);
            if (pub == revoluteJointPublisher.end())
            {
              revoluteJointPublisher[topic] =
                this->node.Advertise<msgs::Double>(topic);
            }
            else
            {
              msgs::Double cmd;
              cmd.set_data(pos);
              pub->second.Publish(cmd);
            }
          }
          else
          {
            prim.CreateAttribute(
              pxr::TfToken("drive:angular:physics:targetPosition"),
              pxr::SdfValueTypeNames->Float, false).Set(0.0f);
          }
        }
      }
    }

    for (const pxr::SdfPath &objectsChanged :
        ObjectsChanged.GetChangedInfoOnlyPaths())
    {
      if (std::string(objectsChanged.GetText()) == "/")
        continue;
      auto stage = this->stage->Lock();
      igndbg << "path " << objectsChanged.GetText() << std::endl;
      auto modelUSD = stage->GetPrimAtPath(objectsChanged.GetParentPath());
      auto property = modelUSD.GetPropertyAtPath(objectsChanged);
      std::string strProperty = property.GetBaseName().GetText();
      if (strProperty == "radius")
      {
        double radius;
        auto attribute = modelUSD.GetAttributeAtPath(objectsChanged);
        attribute.Get(&radius);
      }
      if (strProperty == "translate")
      {
        auto xform = pxr::UsdGeomXformable(modelUSD);

        auto transforms = GetOp(xform);
        auto currentPrim = modelUSD;
        ignition::math::Quaterniond q(
          transforms.rotXYZ[0],
          transforms.rotXYZ[1],
          transforms.rotXYZ[2]);
        if (currentPrim.GetName() == "geometry")
        {
          currentPrim = currentPrim.GetParent();
          auto visualXform = pxr::UsdGeomXformable(currentPrim);
          auto visualOp = GetOp(visualXform);
          transforms.position += visualOp.position;
          ignition::math::Quaterniond qX, qY, qZ;
          ignition::math::Angle angleX(IGN_DTOR(visualOp.rotXYZ[0]));
          ignition::math::Angle angleY(IGN_DTOR(visualOp.rotXYZ[1]));
          ignition::math::Angle angleZ(IGN_DTOR(visualOp.rotXYZ[2]));
          qX = ignition::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
          qY = ignition::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
          qZ = ignition::math::Quaterniond(0, 0, angleZ.Normalized().Radian());
          q = ((q * qX) * qY) * qZ;
          transforms.scale = pxr::GfVec3f(
            transforms.scale[0] * visualOp.scale[0],
            transforms.scale[1] * visualOp.scale[1],
            transforms.scale[2] * visualOp.scale[2]);
        }
        auto currentPrimName = currentPrim.GetName().GetString();
        int substrIndex = currentPrimName.size() - std::string("_visual").size();
        if (substrIndex >= 0 && substrIndex < currentPrimName.size())
        {
          if (currentPrimName.substr(substrIndex).find("_visual") !=
            std::string::npos)
          {
            currentPrim = currentPrim.GetParent();
            auto linkXform = pxr::UsdGeomXformable(currentPrim);
            auto linkOp = GetOp(linkXform);
            transforms.position += linkOp.position;
            ignition::math::Quaterniond qX, qY, qZ;
            ignition::math::Angle angleX(IGN_DTOR(linkOp.rotXYZ[0]));
            ignition::math::Angle angleY(IGN_DTOR(linkOp.rotXYZ[1]));
            ignition::math::Angle angleZ(IGN_DTOR(linkOp.rotXYZ[2]));
            qX = ignition::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
            qY = ignition::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
            qZ = ignition::math::Quaterniond(0, 0, angleZ.Normalized().Radian());
            q = ((q * qX) * qY) * qZ;
            transforms.scale = pxr::GfVec3f(
              transforms.scale[0] * linkOp.scale[0],
              transforms.scale[1] * linkOp.scale[1],
              transforms.scale[2] * linkOp.scale[2]);
          }
        }
        currentPrimName = currentPrim.GetName().GetString();
        substrIndex = currentPrimName.size() - std::string("_link").size();
        if (substrIndex >= 0 && substrIndex < currentPrimName.size())
        {
          if (currentPrimName.substr(substrIndex).find("_link") !=
              std::string::npos)
          {
            currentPrim = currentPrim.GetParent();
            auto modelXform = pxr::UsdGeomXformable(currentPrim);
            auto modelOp = GetOp(modelXform);
            transforms.position += modelOp.position;
            ignition::math::Quaterniond qX, qY, qZ;
            ignition::math::Angle angleX(IGN_DTOR(modelOp.rotXYZ[0]));
            ignition::math::Angle angleY(IGN_DTOR(modelOp.rotXYZ[1]));
            ignition::math::Angle angleZ(IGN_DTOR(modelOp.rotXYZ[2]));
            qX = ignition::math::Quaterniond(angleX.Normalized().Radian(), 0, 0);
            qY = ignition::math::Quaterniond(0, angleY.Normalized().Radian(), 0);
            qZ = ignition::math::Quaterniond(0, 0, angleZ.Normalized().Radian());
            q = ((q * qX) * qY) * qZ;
            transforms.scale = pxr::GfVec3f(
              transforms.scale[0] * modelOp.scale[0],
              transforms.scale[1] * modelOp.scale[1],
              transforms.scale[2] * modelOp.scale[2]);
          }
        }

        auto poseMsg = req.add_pose();
        poseMsg->set_name(currentPrim.GetName());

        poseMsg->mutable_position()->set_x(transforms.position[0]);
        poseMsg->mutable_position()->set_y(transforms.position[1]);
        poseMsg->mutable_position()->set_z(transforms.position[2]);

        poseMsg->mutable_orientation()->set_x(q.X());
        poseMsg->mutable_orientation()->set_y(q.Y());
        poseMsg->mutable_orientation()->set_z(q.Z());
        poseMsg->mutable_orientation()->set_w(q.W());

        // float angle = 2 * acos(q.W());
        // float x = q.X() / sqrt(1-q.W()*q.W());
        // float y = q.Y() / sqrt(1-q.W()*q.W());
        // float z = q.Z() / sqrt(1-q.W()*q.W());
        //
        // std::cerr << "qXAxis " << q.XAxis() << '\n';
        // std::cerr << "qYAxis " << q.YAxis() << '\n';
        // std::cerr << "qZAxis " << q.ZAxis() << '\n';
        // std::cerr << "angle " << angle << " (" << x << ", " << y << ", " << z << ")" << '\n';

      }
    }
    if (req.pose_size() > 0)
    {
      bool result;
      ignition::msgs::Boolean rep;
      unsigned int timeout = 500;
      bool executed = this->node.Request(
        "/world/" + this->worldName + "/set_pose_vector",
        req, timeout, rep, result);
      if (executed)
      {
        if (!result)
          ignerr << "Service call failed" << std::endl;
      }
      else
        ignerr << "Service call timed out" << std::endl;
    }
  }

  std::shared_ptr<ThreadSafe<pxr::UsdStageRefPtr>> stage;
  std::string worldName;
  std::unordered_map<std::string, transport::Node::Publisher> revoluteJointPublisher;

  /// \brief Ignition communication node.
  public: transport::Node node;
};
}  // namespace omniverse
}  // namespace ignition

#endif
