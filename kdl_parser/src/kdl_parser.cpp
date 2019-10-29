/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include "kdl_parser/kdl_parser.hpp"

#include <string>
#include <vector>

#include "urdf.hpp"
#include "sdf.hpp"

namespace kdl_parser
{


bool treeFromFile(const std::string & file, KDL::Tree & tree, MODEL_FORMAT format)
{
  if (format == MODEL_AUTO)
    format = guessFormatFromFilename(file);

  switch (format)
  {
    case MODEL_URDF:
    {
      TiXmlDocument urdf_xml;
      urdf_xml.LoadFile(file);
      return treeFromXml(&urdf_xml, tree);
    }
    case MODEL_SDF:
    {
      std::ifstream file_stream(file.c_str());
      std::string str((std::istreambuf_iterator<char>(file_stream)),
        std::istreambuf_iterator<char>());
      return treeFromString(str, tree, MODEL_SDF);
    }
    default:
    {
      fprintf(stderr, "cannot load file of type %s", formatNameFromID(format));
      return false;
    }
  }
}

bool treeFromParam(const std::string & param, KDL::Tree & tree)
{
  fprintf(stderr, "treeFromParam currently not implemented.\n");
  return false;
  /*
  urdf::Model robot_model;
  if (!robot_model.initParam(param)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
  */
}

bool treeFromString(const std::string & xml, KDL::Tree & tree, MODEL_FORMAT format)
{
  if (format == MODEL_AUTO)
    format = guessFormatFromString(xml);

  switch (format)
  {
    case MODEL_URDF:
    {
      TiXmlDocument urdf_xml;
      urdf_xml.Parse(xml.c_str());
      return treeFromXml(&urdf_xml, tree); 
    }
    case MODEL_SDF:
    {
      sdf::SDFPtr sdf(new sdf::SDF);

      if (!sdf::init(sdf)){
        fprintf(stderr, "unable to initialize sdf.");
        return false;
      }

      if (!sdf::readString(xml, sdf)){
        fprintf(stderr, "unable to read xml string.");
        return false;
      }

      if (!sdf->Root()->HasElement("model")){
        fprintf(stderr, "the <model> tag not exists");
        return false;
      }

      treeFromSdfModel(sdf->Root()->GetElement("model"), tree);
      return true;
    }
    default:
    {
      fprintf(stderr, "cannot load file of type %s", formatNameFromID(format));
      return false;
    }
  }
}

bool treeFromXml(TiXmlDocument * xml_doc, KDL::Tree & tree)
{
  urdf::Model robot_model;
  if (!robot_model.initXml(xml_doc)) {
    fprintf(stderr, "Could not generate robot model\n");
    return false;
  }
  return treeFromUrdfModel(robot_model, tree);
}


bool treeFromUrdfModel(const urdf::ModelInterface & robot_model, KDL::Tree & tree)
{
  if (!robot_model.getRoot()) {
    return false;
  }

  tree = KDL::Tree(robot_model.getRoot()->name);

  // warn if root link has inertia. KDL does not support this
  if (robot_model.getRoot()->inertial) {
    fprintf(stderr, "The root link %s has an inertia specified in the URDF, but KDL does not "
      "support a root link with an inertia.  As a workaround, you can add an extra "
      "dummy link to your URDF.\n", robot_model.getRoot()->name.c_str());
  }

  //  add all children
  for (size_t i = 0; i < robot_model.getRoot()->child_links.size(); i++) {
    if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree)) {
      return false;
    }
  }

  return true;
}

bool treeFromSdfModel(const sdf::ElementPtr& sdf_model, KDL::Tree& tree)
{
  std::string model_name = sdf_model->Get<std::string>("name");
  std::cout << "Loading SDF model" << std::endl << std::endl;
  auto links = loadLinks(sdf_model, "");
  links.toKdlTree(tree, model_name);
  std::cout << std::endl << "Finished loading SDF model" << std::endl << std::endl;

  return true;
}

}  // namespace kdl_parser
