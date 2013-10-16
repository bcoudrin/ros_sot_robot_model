#ifndef ROS_SOT_ROBOT_MODEL_HH
# define ROS_SOT_ROBOT_MODEL_HH
# include <list>
# include <string>

//# include <jrl/mal/boost.hh>
# include "jrl/mal/matrixabstractlayer.hh"

namespace ml = maal::boost;

# include <jrl/dynamics/dynamicsfactory.hh>

//# include <sot/core/flags.hh>
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
//# include <dynamic-graph/pool.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>
//# include <sot/core/exception-dynamic.hh>
# include <sot/core/matrix-homogeneous.hh>
# include <sot-dynamic/dynamic.h>
# include "XmlRpcValue.h"

namespace dynamicgraph
{
  class RosSotRobotModel;

  /// \brief This entity load either the current model available in
  /// the robot_description parameter or a specified file and provides
  /// various data such as body positions, jacobians, etc.
  ///
  /// This relies on jrl_dynamics_urdf to load the model and jrl-dynamics
  /// to realize the computation.
  class RosSotRobotModel : public sot::Dynamic
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:

    RosSotRobotModel(const std::string& n);

    virtual ~RosSotRobotModel();

    void loadUrdf(const std::string& filename);
    void setNamespace (const std::string& ns);
    void loadFromParameterServer();
    Vector curConf() const;

    void addJointMapping(const std::string& link, const std::string& repName);
    void displayChain(const std::string& fromJoint, const std::string &toJoint);
    void jointTransformation(const std::string& jointName);

  protected:

    unsigned getDimension () const
    {
      if (!m_HDR)
	throw std::runtime_error ("no robot loaded");
      return m_HDR->numberDof();
    }

  private:

    /// \brief Name of the parameter where the joints list will be published
    std::string jointsParameterName_;

    /// \brief Name of the controller namespace
    std::string ns_;

    /// \brief Special joints map for the parser
    std::map<std::string, std::string> specialJoints_;


  };
} // end of namespace dynamicgraph.

#endif //! ROS_SOT_ROBOT_MODEL_HH
