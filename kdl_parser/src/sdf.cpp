
#include "sdf.hpp"

using namespace std;
using ignition::math::Vector3d;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;

namespace kdl_parser {

KDL::Vector toKdl(Vector3d axis)
{
    return KDL::Vector(axis.X(), axis.Y(), axis.Z());
}

KDL::Rotation toKdl(Quaterniond rot)
{
    return KDL::Rotation::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W());
}
KDL::Frame toKdl(Pose3d pose)
{
    KDL::Vector position = toKdl(pose.Pos());
    KDL::Rotation rotation = toKdl(pose.Rot());
    return KDL::Frame(rotation, position);
}

KDL::Joint toKdl(std::string name, std::string type, KDL::Frame pose, KDL::Vector axis)
{
    if (type == "revolute"){
        return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::RotAxis);
    }
    else if (type == "prismatic"){
        return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::TransAxis);
    }
    else if (type == "fixed"){
        return KDL::Joint(name, KDL::Joint::None);
    }
    else
        throw runtime_error("cannot handle joint type " + type);

}

void Links::toKdlTree(KDL::Tree & tree, std::string parent, bool root)
{
    for (auto & l : data)
    {
        KDL::Joint joint;
        auto kdl_parent = parent;
        if (l.parent_link_name_.size() == 0)
        {
            if (root)
            {
                joint = ((KDL::Joint)(KDL::Joint::None));
                kdl_parent = "root";
            }
            else
                continue;
        }
        else if (l.parent_name() == parent)
            joint = l.joint_.toKdlJoint();
        else
            continue;
        KDL::RigidBodyInertia I;
        if (l.sdf_->HasElement("inertial"))
            I = sdfInertiaToKdl(l.sdf_->GetElement("inertial"));
        auto new_segment_frame = l.frame_;
        auto parent_obj = this->find(parent);
        if (parent_obj)
            new_segment_frame = this->find(parent)->frame_.Inverse()*new_segment_frame;
        KDL::Segment segment(l.name(), joint, new_segment_frame, I);
        tree.addSegment(segment, kdl_parent);
        toKdlTree(tree, l.name(), false);
    }
}

bool sdfIsModelStatic(const sdf::ElementPtr sdf_model)
{
    sdf::ElementPtr elementStatic = sdf_model->GetElement("static");
    if (elementStatic)
        return elementStatic->Get<bool>();
    else
        return false;
}

KDL::RigidBodyInertia sdfInertiaToKdl(sdf::ElementPtr sdf)
{
    KDL::Frame pose = toKdl(sdf->GetElement("pose")->Get<Pose3d>());
    double mass;

    if (sdf->HasElement("mass")){
        mass = sdf->GetElement("mass")->Get<double>();
        if (sdf->HasElement("inertia")){

            sdf::ElementPtr sdf_inertia = sdf->GetElement("inertia");

            const char *tags[] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz" };

            double ixx, ixy, ixz, iyy, iyz, izz;
            double *ptr[] = {&ixx, &ixy, &ixz, &iyy, &iyz, &izz};

            for (int i = 0; i < 6; i++){
                (*ptr[i]) = sdf_inertia->GetElement(tags[i])->Get<double>();
            }

            return pose.M * KDL::RigidBodyInertia(mass, pose.p,
                KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz));
        }
    }

    return KDL::RigidBodyInertia();
}

void sdfExtractJointData(sdf::ElementPtr sdf_joint,
                         std::string& joint_name,
                         std::string& joint_type,
                         KDL::Frame& joint_pose,
                         KDL::Vector& joint_axis,
                         bool& use_parent_model_frame)
{
    if (sdf_joint->HasAttribute("name")){
        joint_name = sdf_joint->Get<std::string>("name");
    }

    if (sdf_joint->HasAttribute("type")){
        joint_type = sdf_joint->Get<std::string>("type");
    }

    if (sdf_joint->HasElement("pose")){
        joint_pose = toKdl(sdf_joint->GetElement("pose")->Get<Pose3d>());
    }

    if (sdf_joint->HasElement("axis")){
        sdf::ElementPtr sdf_axis = sdf_joint->GetElement("axis");

        joint_axis = toKdl(sdf_joint->GetElement("axis")->GetElement("xyz")->Get<Vector3d>());

        if (sdf_axis->HasElement("use_parent_model_frame")){
            use_parent_model_frame = sdf_axis->GetElement("use_parent_model_frame")->Get<bool>();
        }

    }
}

KDL::Frame getLinkPose(sdf::ElementPtr sdf_link)
{
    if (sdf_link->HasElement("pose"))
        return toKdl(sdf_link->GetElement("pose")->Get<Pose3d>());
    else
        return KDL::Frame();
}

Links loadLinks(const sdf::ElementPtr& sdf_model, std::string model_name, KDL::Frame offset, bool indent)
{
    std::cout << "Recursive on model " << model_name << std::endl;
    auto root_2_model = getLinkPose(sdf_model);

    Links links;

    if (sdf_model->HasElement("model")) {
        sdf::ElementPtr internal_sdf_model = sdf_model->GetElement("model");
        while (internal_sdf_model) {
            std::string internal_model_name = internal_sdf_model->Get<std::string>("name");

            KDL::Frame internal_model_pose;
            if (internal_sdf_model->HasElement("pose")) {
                internal_model_pose = toKdl(internal_sdf_model->GetElement("pose")->Get<Pose3d>());
                std::cout << "Got internal_model_pose of model " << internal_model_name << " " <<
                    internal_model_pose.p[0] << " " << internal_model_pose.p[1] << " " <<
                    internal_model_pose.p[2] << " " << std::endl;
            }

            auto internal_links = loadLinks(internal_sdf_model, internal_model_name,
                internal_model_pose, true);
            links.append(internal_links);
            internal_sdf_model = internal_sdf_model->GetNextElement("model");
        }
    }

    if (sdf_model->HasElement("link")) {
        sdf::ElementPtr link_elem = sdf_model->GetElement("link");
        while (link_elem) {
            Link link;
            link.link_name_.push_back(link_elem->Get<std::string>("name"));
            link.sdf_ = link_elem;
            link.frame_ = offset*getLinkPose(link_elem);
            std::cout << "Link " << link.name() << " has pose " << " " << link.frame_.p[0] <<
                " " << link.frame_.p[1] << " " << link.frame_.p[2] << std::endl;
            links.data.push_back(link);
            link_elem = link_elem->GetNextElement("link");
        }
    }

    if (sdf_model->HasElement("joint")) {
        sdf::ElementPtr joint_elem = sdf_model->GetElement("joint");
        while (joint_elem) {
            auto child_name = joint_elem->GetElement("child")->Get<std::string>();
            auto child_link_ptr = links.find(child_name);
            if (child_link_ptr == nullptr)
                throw std::logic_error("Couldn't find child " + child_name + " of joint " +
                    joint_elem->Get<std::string>("name"));

            auto parent_name = joint_elem->GetElement("parent")->Get<std::string>();
            auto parent_link_ptr = links.find(parent_name);
            if (parent_link_ptr == nullptr)
                throw std::logic_error("Couldn't find parent " + parent_name + " of joint " +
                    joint_elem->Get<std::string>("name"));

            child_link_ptr->parent_link_name_ = parent_link_ptr->link_name_;

            // getting KDL::Joint
            {
                std::string joint_type;
                std::string joint_name;
                KDL::Vector joint_axis;
                KDL::Frame joint_2_child;
                bool use_parent_model_frame = false;
                sdfExtractJointData(joint_elem, joint_name, joint_type, joint_2_child, joint_axis,
                    use_parent_model_frame);

                KDL::Frame child_2_model;
                if (child_link_ptr->sdf_->HasElement("pose")) {
                    child_2_model = toKdl(child_link_ptr->sdf_->GetElement("pose")->Get<Pose3d>());
                }
                auto parent_2_model = getLinkPose(parent_link_ptr->sdf_);
                KDL::Frame child_2_parent = parent_2_model.Inverse() * child_2_model;
                KDL::Frame joint_2_parent = child_2_parent * joint_2_child;

                if (use_parent_model_frame){
                    KDL::Frame joint_2_model = joint_2_child * child_2_model;
                    joint_axis = joint_2_model.M.Inverse() * joint_axis;
                }
                child_link_ptr->joint_.joint_name_.push_back(joint_name);
                child_link_ptr->joint_.type_ = joint_type;
                child_link_ptr->joint_.pose_ = joint_2_parent;
                child_link_ptr->joint_.axis_ = joint_axis;
            }


            joint_elem = joint_elem->GetNextElement("joint");
        }
    }

    if (indent)
        links.indent(model_name);

    std::cout << std::endl << "Links:" << std::endl;
    for (auto & l : links.data)
    {
        std::cout << "link " << l.name();
        if (l.parent_link_name_.size() == 0)
            std::cout << " without parent" << std::endl;
        else
            std::cout << " with parent " << l.parent_name() << std::endl;
    }

    std::cout << std::endl << "Recursive end on model " << model_name << std::endl;
    return links;
}

}
