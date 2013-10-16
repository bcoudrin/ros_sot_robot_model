#include <limits>

#include <boost/bind.hpp>

#include <ros/ros.h>

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <jrl/dynamics/urdf/parser.hh>

#include "dynamic_graph_bridge/ros_init.hh"
#include <ros_sot_robot_model/ros_sot_robot_model.hh>

namespace dynamicgraph
{

RosSotRobotModel::RosSotRobotModel(const std::string& name)
    : Dynamic(name,false),
      jointsParameterName_("jrl_map"),
      ns_("sot_controller")
{
    std::string docstring;

    docstring =
            "\n"
            "  Load the robot model from the parameter server.\n"
            "\n"
            "  This is the recommended method.\n"
            "\n";
    addCommand("loadFromParameterServer", command::makeCommandVoid0(*this,&RosSotRobotModel::loadFromParameterServer,docstring));

    docstring =
            "\n"
            "  Load the robot model from an URDF file.\n"
            "\n";
    addCommand("loadUrdf", command::makeCommandVoid1(*this,&RosSotRobotModel::loadUrdf,docstring));

    docstring =
            "\n"
            "  Set the controller namespace."
            "\n";
    addCommand("setNamespace", command::makeCommandVoid1(*this,&RosSotRobotModel::setNamespace,docstring));

    docstring =
            "\n"
            "  Get current configuration of the robot.\n"
            "\n";
    addCommand ("curConf", new command::Getter<RosSotRobotModel,Vector> (*this,&RosSotRobotModel::curConf,docstring));

    docstring =
            "\n"
            "  Maps a link name in the URDF parser to actual robot link name.\n"
            "\n";
    addCommand ("addJointMapping", command::makeCommandVoid2(*this,&RosSotRobotModel::addJointMapping,docstring));

    docstring =
            "\n"
            "  Displays the chain between to joints.\n"
            "\n";
    addCommand ("displayChain", command::makeCommandVoid2(*this,&RosSotRobotModel::displayChain,docstring));

    docstring =
            "\n"
            "  Displays the the current transformation of a joint.\n"
            "\n";
    addCommand ("jointTransformation", command::makeCommandVoid1(*this,&RosSotRobotModel::jointTransformation,docstring));
}

RosSotRobotModel::~RosSotRobotModel()
{}

void RosSotRobotModel::loadUrdf (const std::string& filename)
{
    jrl::dynamics::urdf::Parser parser;

    std::map<std::string, std::string>::const_iterator it = specialJoints_.begin();
    for (;it!=specialJoints_.end();++it) {
        parser.specifyREPName(it->first, it->second);
    }

    m_HDR = parser.parse(filename);

    ros::NodeHandle nh(ns_);

    nh.setParam(jointsParameterName_, parser.JointsNamesByRank_);
}

void RosSotRobotModel::setNamespace (const std::string& ns)
{
    ns_ = ns;
}

void RosSotRobotModel::loadFromParameterServer()
{
    jrl::dynamics::urdf::Parser parser;

    std::map<std::string, std::string>::const_iterator it = specialJoints_.begin();
    for (;it!=specialJoints_.end();++it) {
        parser.specifyREPName(it->first, it->second);
    }

    rosInit (false);
    std::string robotDescription;
    ros::param::param<std::string> ("/robot_description", robotDescription, "");

    if (robotDescription.empty ())
        throw std::runtime_error("No model available as ROS parameter. Fail.");

    m_HDR = parser.parseStream (robotDescription);

    ros::NodeHandle nh(ns_);

    nh.setParam(jointsParameterName_, parser.JointsNamesByRank_);

}

namespace
{

vectorN convertVector(const ml::Vector& v)
{
    vectorN res (v.size());
    for (unsigned i = 0; i < v.size(); ++i)
        res[i] = v(i);
    return res;
}

ml::Vector convertVector(const vectorN& v)
{
    ml::Vector res;
    res.resize(v.size());
    for (unsigned i = 0; i < v.size(); ++i)
        res(i) = v[i];
    return res;
}

} // end of anonymous namespace.

Vector RosSotRobotModel::curConf() const
{

    // The first 6 dofs are associated to the Freeflyer frame
    // Freeflyer reference frame should be the same as global
    // frame so that operational point positions correspond to
    // position in freeflyer frame.

    XmlRpc::XmlRpcValue ffpose;
    ros::NodeHandle nh(ns_);
    std::string param_name = "ffpose";
    if (nh.hasParam(param_name)){
        nh.getParam(param_name, ffpose);
        ROS_ASSERT(ffpose.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(ffpose.size() == 6);
        for (int32_t i = 0; i < ffpose.size(); ++i)
        {
            ROS_ASSERT(ffpose[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }
    }
    else
    {
        ffpose.setSize(6);
        for (int32_t i = 0; i < ffpose.size(); ++i)
            ffpose[i] = 0.0;
    }

    if (!m_HDR )
        throw std::runtime_error ("no robot loaded");
    else {
        vectorN currConf = m_HDR->currentConfiguration();
        Vector res;
        res = convertVector(currConf);

        for (int32_t i = 0; i < ffpose.size(); ++i)
            res(i) = static_cast<double>(ffpose[i]);

        return res;
    }
}

void
RosSotRobotModel::addJointMapping(const std::string &link, const std::string &repName)
{
    specialJoints_[link] = repName;
}

void
RosSotRobotModel::displayChain(const std::string& fromJoint, const std::string &toJoint) {
    std::vector<CjrlJoint*> ljoints = m_HDR->jointVector();

    matrix4d tf;
    tf.setIdentity();
    std::vector<CjrlJoint*>::const_iterator it=ljoints.begin();
    for (;it!=ljoints.end();++it) {
        if ((*it)->getName() == toJoint) {
            std::cout << (*it)->getName() << " <-- ";
            tf = (*it)->currentTransformation() * tf;
            CjrlJoint *parent = (*it)->parentJoint();
            while (parent && parent->getName() != fromJoint) {
                std::cout << parent->getName() << " <-- ";
                tf = parent->currentTransformation() * tf;
                parent = parent->parentJoint();
            }
            if (parent) {
                tf = parent->currentTransformation() * tf;
                std::cout << parent->getName();
            }
            std::cout << std::endl;

            std::cout << tf << std::endl;
            break;
        }
    }
}

void
RosSotRobotModel::jointTransformation(const std::string& jointName) {
    std::vector<CjrlJoint*> ljoints = m_HDR->jointVector();

    std::vector<CjrlJoint*>::const_iterator it=ljoints.begin();
    for (;it!=ljoints.end();++it) {
        if ((*it)->getName() == jointName) {
            std::cout << (*it)->currentTransformation() << std::endl;
        }
    }
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosSotRobotModel, "RosSotRobotModel");
} // end of namespace dynamicgraph.
