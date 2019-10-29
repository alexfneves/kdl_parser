#ifndef KDL_PARSER__SDF_HPP_
#define KDL_PARSER__SDF_HPP_


#include <kdl/tree.hpp>
#include <sdf/sdf.hh>

using ignition::math::Vector3d;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;

namespace kdl_parser {


/**
 * convert <axis> element to KDL::Vector
 */
KDL::Vector toKdl(Vector3d axis);

/**
 * convert Quaternion to KDL::Rotation
 */
KDL::Rotation toKdl(Quaterniond rot);

/**
 * convert <pose> element to KDL::Frame
 */
KDL::Frame toKdl(Pose3d pose);

/**
 * convert <joint> element to KDL::Joint
 */
KDL::Joint toKdl(std::string name, std::string type, KDL::Frame pose, KDL::Vector axis);



class Joint
{
    std::string name(std::vector<std::string> & name_vec)
    {
        if (name_vec.size() < 1)
            throw std::logic_error("Can't extract name of joint because variable is empty");
        std::string ret = name_vec[0];
        for (auto it = name_vec.begin() + 1; it != name_vec.end(); it++)
            ret += "::" + *it;
        return ret;
    }

public:
    std::vector<std::string> joint_name_;
    std::string type_;
    KDL::Frame pose_;
    KDL::Vector axis_;

    std::string name() {return name(joint_name_);}
    KDL::Joint toKdlJoint(){return toKdl(name(), type_, pose_, axis_);}
};

class Link
{
    std::string name(std::vector<std::string> & name_vec)
    {
        if (name_vec.size() < 1)
            throw std::logic_error("Can't extract name of link because variable is empty");
        std::string ret = name_vec[0];
        for (auto it = name_vec.begin() + 1; it != name_vec.end(); it++)
            ret += "::" + *it;
        return ret;
    }

public:
    sdf::ElementPtr sdf_;
    std::vector<std::string> link_name_;
    std::vector<std::string> parent_link_name_;
    KDL::Frame frame_;
    Joint joint_;

    std::string name() {return name(link_name_);}
    std::string parent_name() {return name(parent_link_name_);}
};

class Links
{
public:
    std::vector<Link> data;
    void append(Links & links)
    {
        for (auto & l : links.data)
            data.push_back(l);
    }
    void indent(std::string name)
    {
        for (auto & l : data)
        {
            l.link_name_.insert(l.link_name_.begin(), name);
            l.parent_link_name_.insert(l.parent_link_name_.begin(), name);
            if (l.joint_.joint_name_.size() > 0)
                std::cout << "indenting joint " << l.joint_.name() << " with " << name << " with link " << l.name() << std::endl;
            else
                std::cout << "indenting joint without name with " << name << " with link " << l.name() << std::endl;
            l.joint_.joint_name_.insert(l.joint_.joint_name_.begin(), name);
            std::cout << "the result is " << l.joint_.name() << std::endl;
        }
    }
    Link* find(std::string link_name)
    {
        for (auto & l : data)
            if (l.name() == link_name)
                return &l;
        return nullptr;
    }
    void toKdlTree(KDL::Tree & tree, std::string parent, bool root = true);
};



/**
 * check if the SDF element is a static element
 */
bool sdfIsModelStatic(const sdf::ElementPtr sdf_model);

/**
 * convert <inertial> element to KDL::RigidBodyInertia
 */
KDL::RigidBodyInertia sdfInertiaToKdl(sdf::ElementPtr sdf);

/*
 * extract joint data
 */
void sdfExtractJointData(sdf::ElementPtr sdf_joint,
                         std::string& joint_name,
                         std::string& joint_type,
                         KDL::Frame& joint_pose,
                         KDL::Vector& joint_axis,
                         bool& use_parent_model_frame);

/*
 * get the pose of a link
 */
KDL::Frame getLinkPose(sdf::ElementPtr sdf_link);

Links loadLinks(const sdf::ElementPtr& sdf_model, std::string link_name, KDL::Frame offset = KDL::Frame(), bool indent = false);

}

#endif  // KDL_PARSER__SDF_HPP_
