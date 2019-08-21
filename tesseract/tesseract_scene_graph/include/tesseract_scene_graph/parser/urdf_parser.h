/**
 * @file graph.h
 * @brief A basic scene graph using boost
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_SCENE_GRAPH_URDF_PARSER_H
#define TESSERACT_SCENE_GRAPH_URDF_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <urdf_parser/urdf_parser.h>
#include <tesseract_geometry/geometries.h>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/mesh_parser.h>

namespace tesseract_scene_graph
{
using ResourceLocatorFn = std::function<std::string(const std::string&)>;

inline Eigen::Isometry3d urdfPose2Eigen(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}

inline JointType convert(const int& joint_type)
{
  switch (joint_type)
  {
    case urdf::Joint::FIXED:
    {
      return JointType::FIXED;
    }
    case urdf::Joint::PLANAR:
    {
      return JointType::PLANAR;
    }
    case urdf::Joint::FLOATING:
    {
      return JointType::FLOATING;
    }
    case urdf::Joint::REVOLUTE:
    {
      return JointType::REVOLUTE;
    }
    case urdf::Joint::PRISMATIC:
    {
      return JointType::PRISMATIC;
    }
    case urdf::Joint::CONTINUOUS:
    {
      return JointType::CONTINUOUS;
    }
    default:
    {
      return JointType::UNKNOWN;
    }
  }
}
inline JointCalibration::Ptr convert(const urdf::JointCalibrationConstSharedPtr& calibration)
{
  if (calibration == nullptr)
    return nullptr;

  JointCalibration::Ptr c(new JointCalibration());
  c->reference_position = calibration->reference_position;
  c->rising = (calibration->rising == nullptr) ? 0 : *(calibration->rising);
  c->falling = (calibration->falling == nullptr) ? 0 : *(calibration->falling);
  return c;
}

inline JointDynamics::Ptr convert(const urdf::JointDynamicsConstSharedPtr& dynamics)
{
  if (dynamics == nullptr)
    return nullptr;

  JointDynamics::Ptr d(new JointDynamics());
  d->damping = dynamics->damping;
  d->friction = dynamics->friction;
  return d;
}

inline JointLimits::Ptr convert(const urdf::JointLimitsConstSharedPtr& limits)
{
  if (limits == nullptr)
    return nullptr;

  JointLimits::Ptr l(new JointLimits());
  l->lower = limits->lower;
  l->upper = limits->upper;
  l->effort = limits->effort;
  l->velocity = limits->velocity;
  return l;
}

inline JointMimic::Ptr convert(const urdf::JointMimicConstSharedPtr& mimic)
{
  if (mimic == nullptr)
    return nullptr;

  JointMimic::Ptr m(new JointMimic());
  m->offset = mimic->offset;
  m->multiplier = mimic->multiplier;
  m->joint_name = mimic->joint_name;
  return m;
}

inline JointSafety::Ptr convert(const urdf::JointSafetyConstSharedPtr& safety)
{
  if (safety == nullptr)
    return nullptr;

  JointSafety::Ptr s(new JointSafety());
  s->soft_upper_limit = safety->soft_upper_limit;
  s->soft_lower_limit = safety->soft_lower_limit;
  s->k_position = safety->k_position;
  s->k_velocity = safety->k_velocity;
  return s;
}

inline Inertial::Ptr convert(const urdf::InertialConstSharedPtr& inertial)
{
  if (inertial == nullptr)
    return nullptr;

  Inertial::Ptr i(new Inertial());
  i->origin = urdfPose2Eigen(inertial->origin);
  i->mass = inertial->mass;
  i->ixx = inertial->ixx;
  i->ixy = inertial->ixy;
  i->ixz = inertial->ixz;
  i->iyy = inertial->iyy;
  i->iyz = inertial->iyz;
  i->izz = inertial->izz;
  return i;
}

inline std::vector<tesseract_geometry::Geometry::Ptr> convert(const urdf::GeometryConstSharedPtr& geometry,
                                                              ResourceLocatorFn locator,
                                                              bool visual = true)
{
  std::vector<tesseract_geometry::Geometry::Ptr> g;

  if (geometry == nullptr)
    return g;

  switch (geometry->type)
  {
    case urdf::Geometry::BOX:
    {
      const urdf::Box& ug = static_cast<const urdf::Box&>(*geometry);
      g.push_back(tesseract_geometry::Geometry::Ptr(new tesseract_geometry::Box(ug.dim.x, ug.dim.y, ug.dim.z)));
      break;
    }
    case urdf::Geometry::SPHERE:
    {
      const urdf::Sphere& ug = static_cast<const urdf::Sphere&>(*geometry);
      g.push_back(tesseract_geometry::Geometry::Ptr(new tesseract_geometry::Sphere(ug.radius)));
      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      const urdf::Cylinder& ug = static_cast<const urdf::Cylinder&>(*geometry);
      g.push_back(tesseract_geometry::Geometry::Ptr(new tesseract_geometry::Cylinder(ug.radius, ug.length)));
      break;
    }
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh& ug = static_cast<const urdf::Mesh&>(*geometry);
      std::vector<tesseract_geometry::Mesh::Ptr> meshes;

      if (visual)
        meshes = createMeshFromPath<tesseract_geometry::Mesh>(
            locator(ug.filename), Eigen::Vector3d(ug.scale.x, ug.scale.y, ug.scale.z), true, true);
      else
        meshes = createMeshFromPath<tesseract_geometry::Mesh>(
            locator(ug.filename), Eigen::Vector3d(ug.scale.x, ug.scale.y, ug.scale.z), true, false);

      assert(!meshes.empty());
      g.insert(g.end(), meshes.begin(), meshes.end());
      break;
    }
  }

  assert(!g.empty());
  return g;
}

/**
 * @brief Convert a urdf material to tesseract material
 * @param material The urdf material
 * @return Tesseract Material
 */
inline Material::Ptr convert(const urdf::MaterialConstSharedPtr& material)
{
  if (material == nullptr)
    return nullptr;

  Material::Ptr m(new Material(material->name));
  m->color(0) = static_cast<double>(material->color.r);
  m->color(1) = static_cast<double>(material->color.g);
  m->color(2) = static_cast<double>(material->color.b);
  m->color(3) = static_cast<double>(material->color.a);
  m->texture_filename = material->texture_filename;
  return m;
}

/**
 * @brief Convert a urdf visual to tesseract visual
 * @param visual The urdf visual
 * @return A tesseract visual object
 */
inline Visual::Ptr convert(const urdf::VisualConstSharedPtr& visual, ResourceLocatorFn locator)
{
  if (visual == nullptr)
    return nullptr;

  Visual::Ptr v(new Visual());
  v->name = visual->name;
  v->origin = urdfPose2Eigen(visual->origin);
  v->material = convert(visual->material);
  std::vector<tesseract_geometry::Geometry::Ptr> geometries = convert(visual->geometry, locator, true);
  v->geometry = geometries[0];

  return v;
}

/**
 * @brief Convert a urdf collision to tesseract collision
 * @param collision The urdf collision
 * @return A vector of tesseract collision object. When loading mesh files it can contain multiple meshes.
 */
inline std::vector<Collision::Ptr> convert(const urdf::CollisionConstSharedPtr& collision, ResourceLocatorFn locator)
{
  if (collision == nullptr)
    return std::vector<Collision::Ptr>();

  std::vector<Collision::Ptr> cv;
  std::string name = collision->name;
  Eigen::Isometry3d pose = urdfPose2Eigen(collision->origin);
  std::vector<tesseract_geometry::Geometry::Ptr> geometries = convert(collision->geometry, locator, false);

  if (geometries.size() == 1)
  {
    Collision::Ptr c(new Collision());
    c->name = name;
    c->origin = pose;
    c->geometry = geometries[0];
    cv.push_back(c);
  }
  else
  {
    int i = 0;
    for (auto g : geometries)
    {
      Collision::Ptr c(new Collision());
      c->name = name + "_" + std::to_string(i);
      c->origin = pose;
      c->geometry = g;
      cv.push_back(c);
      ++i;
    }
  }

  return cv;
}

inline SceneGraph::Ptr parseURDFModel(const urdf::ModelInterfaceSharedPtr& urdf_model, ResourceLocatorFn locator)
{
  if (urdf_model == nullptr || locator == nullptr)
    return nullptr;

  SceneGraph::Ptr g(new SceneGraph());

  // Populate Links
  std::vector<urdf::LinkSharedPtr> urdf_links;
  urdf_model->getLinks(urdf_links);
  for (const auto& link : urdf_links)
  {
    Link tlink(link->name);
    tlink.inertial = convert(link->inertial);
    if (link->visual_array.size() > 0)
    {
      for (const auto& v : link->visual_array)
        tlink.visual.push_back(convert(v, locator));
    }
    if (link->collision_array.size() > 0)
    {
      for (const auto& c : link->collision_array)
      {
        std::vector<Collision::Ptr> cv = convert(c, locator);
        tlink.collision.insert(tlink.collision.end(), cv.begin(), cv.end());
      }
    }

    g->addLink(tlink);
  }

  // Populate Joints
  for (const auto& jp : urdf_model->joints_)
  {
    const urdf::Joint& joint = *(jp.second);
    Joint tjoint(joint.name);
    tjoint.type = convert(joint.type);
    tjoint.axis = Eigen::Vector3d(joint.axis.x, joint.axis.y, joint.axis.z);
    tjoint.child_link_name = joint.child_link_name;
    tjoint.parent_link_name = joint.parent_link_name;
    tjoint.parent_to_joint_origin_transform = urdfPose2Eigen(joint.parent_to_joint_origin_transform);
    tjoint.dynamics = convert(joint.dynamics);
    tjoint.limits = convert(joint.limits);
    tjoint.safety = convert(joint.safety);
    tjoint.calibration = convert(joint.calibration);
    tjoint.mimic = convert(joint.mimic);

    g->addJoint(tjoint);
  }

  g->setRoot(urdf_model->getRoot()->name);
  g->setName(urdf_model->getName());

  return g;
}

inline SceneGraph::Ptr parseURDFString(const std::string& urdf_xml_string, ResourceLocatorFn locator)
{
  return parseURDFModel(urdf::parseURDF(urdf_xml_string), locator);
}

inline SceneGraph::Ptr parseURDFFile(const std::string& path, ResourceLocatorFn locator)
{
  std::ifstream ifs(path);
  std::string urdf_xml_string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return parseURDFString(urdf_xml_string, locator);
}

}  // namespace tesseract_scene_graph

#endif
