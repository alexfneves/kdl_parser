#ifndef KDL_PARSER__URDF_HPP_
#define KDL_PARSER__URDF_HPP_


#include "kdl/tree.hpp"
#include "urdf/model.h"
#include "urdf/urdfdom_compatibility.h"

namespace kdl_parser {

KDL::Vector toKdl(urdf::Vector3 v);
KDL::Rotation toKdl(urdf::Rotation r);
KDL::Frame toKdl(urdf::Pose p);
KDL::Joint toKdl(urdf::JointSharedPtr jnt);
KDL::RigidBodyInertia toKdl(urdf::InertialSharedPtr i);
bool addChildrenToTree(urdf::LinkConstSharedPtr root, KDL::Tree & tree);

}


#endif  // KDL_PARSER__URDF_HPP_
