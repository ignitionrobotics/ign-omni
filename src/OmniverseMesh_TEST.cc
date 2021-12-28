/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "OmniverseMesh.hh"

#include <gtest/gtest.h>

#include "test/SceneTest.hh"

namespace ignition::rendering::omni::test {

TEST_F(SceneTest, PointsMesh) {
  ignition::common::SubMesh subMesh{"test_sub_mesh"};
  subMesh.AddVertex(0, 0, 0);
  subMesh.AddVertex(0, 1, 0);
  subMesh.AddVertex(1, 1, 0);
  subMesh.AddVertex(1, 0, 0);
  subMesh.AddIndex(0);
  subMesh.AddIndex(1);
  subMesh.AddIndex(2);
  subMesh.AddIndex(3);

  ignition::common::Mesh mesh;
  mesh.SetName("test_mesh");
  mesh.AddSubMesh(subMesh);

  MeshDescriptor meshDesc{&mesh};
  auto ovMesh = OmniverseMesh::Make(0, "test_mesh", this->scene, meshDesc);
  this->rootVisual->AddGeometry(ovMesh);

  auto it = this->rootVisual->Prim().GetChildren();
  std::vector<pxr::UsdPrim> children{it.begin(), it.end()};
  EXPECT_EQ(1, children.size());
  EXPECT_EQ("Mesh", children[0].GetTypeName());
}

}  // namespace ignition::rendering::omni::test
