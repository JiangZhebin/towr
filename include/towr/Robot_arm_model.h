#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>

namespace towr{

class SingleRobotArmKinematicModel : public KinematicModel{
public:
  SingleRobotArmKinematicModel () : KinematicModel(1)
  {
    nominal_stance_.at(0) = Eigen::Vector3d(0,0,-0.1);
    max_dev_from_nominal_ << 0.5, 0.5, 1.3;

  }
  ~SingleRobotArmKinematicModel(){};
};

class SingleObjectDynamicModel : public SingleRigidBodyDynamics{
public:
  SingleObjectDynamicModel(): SingleRigidBodyDynamics(10,0.06667,0.06667,0.06667,0,0,0,1)

  {}
  ~SingleObjectDynamicModel(){};
};



class BipedRobotArmKinematicModel : public KinematicModel{
public:
  BipedRobotArmKinematicModel () : KinematicModel(2)
  {
    nominal_stance_.at(0) = Eigen::Vector3d(-0.1,0,-0.1);
    nominal_stance_.at(1) = Eigen::Vector3d(0.1,0,-0.1);
    max_dev_from_nominal_ << 0.5, 0.5, 1.3;

  }
  ~BipedRobotArmKinematicModel(){};
};

class BipedObjectDynamicModel : public SingleRigidBodyDynamics{
public:
  BipedObjectDynamicModel(): SingleRigidBodyDynamics(10,0.06667,0.06667,0.06667,0,0,0,2)

  {}
  ~BipedObjectDynamicModel(){};
};
//input radius of the sphere
class FourRobotArmKinematicModel : public KinematicModel{
public:
  FourRobotArmKinematicModel () : KinematicModel(4)
  {
    nominal_stance_.at(0) = Eigen::Vector3d(-0.1,0,0);
    nominal_stance_.at(1) = Eigen::Vector3d(0.1,0.0,0.0);
    nominal_stance_.at(2) = Eigen::Vector3d(0,-0.1,0);
    nominal_stance_.at(3) = Eigen::Vector3d(0,0.1,0);
    max_dev_from_nominal_ << 0.5, 0.5, 1.3;

  }
  ~FourRobotArmKinematicModel(){};
};

class FourObjectDynamicModel : public SingleRigidBodyDynamics{
public:
  FourObjectDynamicModel(): SingleRigidBodyDynamics(10,0.06667,0.06667,0.06667,0,0,0,4)

  {}
  ~FourObjectDynamicModel(){};
};
}


