#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <Ravelin/VectorNd.h>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/LinAlgd.h>
#include <stdio.h>
#include <fstream>
#include <sstream>

namespace gazebo
{
  // converts a Pose to a Gazebo Matrix4
  math::Matrix4 ToMatrix(const math::Pose& p)
  {
    math::Matrix3 m = p.rot.GetAsMatrix3();
    math::Matrix4 T;
    for (unsigned i=0; i< 3; i++)
      for (unsigned j=0; j< 3; j++)
        T[i][j] = m[i][j];

    for (unsigned i=0; i< 3; i++)
      T[i][3] = p.pos[i];

    // set bottom row of the matrix
    T[3][0] = T[3][1] = T[3][2] = 0.0;
    T[3][3] = 1.0;
    return T;
  }

  class JacobiansPlanarPlugin : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j1, _j2, _j3;

    public: JacobiansPlanarPlugin()
    {
    }

    // sets the arm to a particular configuration 
    private: void SetConfig(Ravelin::VectorNd& q)
    {
      // iterate over all joints
      for (unsigned i=0, k=0; i< _model->GetJoints().size(); i++)
      {
        physics::JointPtr joint = _model->GetJoints()[i];

        // iterate over all degrees-of-freedom in the joint
        for (unsigned j=0; j< joint->GetAngleCount(); j++)
        {
          // make sure that the joint is active
          if (joint->GetLowerLimit(j) >= joint->GetUpperLimit(j))
            continue;

          // set the joint angle
          joint->SetPosition(j, q[k]);
          k++;
        }
      }
    }

    // gets the Jacobian for the planar manipulator
    private: Ravelin::MatrixNd CalcJacobian(double theta1, double theta2, double theta3)
    {
      const math::Vector3 ORIGIN1(0.0, 0.0, 0.0);  // origin of frame 1
      const math::Vector3 ORIGIN2(0.0, 0.0, 0.0);  // origin of frame 2
      const math::Vector3 ORIGIN3(0.0, 0.0, 0.0);  // origin of frame 3 
      const math::Vector3 JointAxis(0.0, 0.0, 1.0); // All joints have axis in the vertical or z direction
      const unsigned X = 0, Y = 1, THETA = 2, J1 = 0, J2 = 1, J3 = 2;

      // temporary values
      const double INF = std::numeric_limits<double>::max();
      const double CHANGEME = INF;
      const math::Vector3 CHANGEME_VEC3(CHANGEME, CHANGEME, CHANGEME);
 
      // the poses (for you, the student, to set) 
      math::Pose _0P0x, _0xP1, _1P1x, _1xP2, _2P2x, _2xP3; 

      // TODO: compute the first transform: frame 0' to 0
	  _0P0x.rot.SetFromAxis(0,0,1,theta1);
      // TODO: compute the second transform: frame 1 to 0' 
      _0xP1.pos = math::Vector3(2,0,0);      
      // TODO: compute the third transform: frame 1' to 1
      _1P1x.rot.SetFromAxis(0,0,1,theta2);
      // TODO: compute the fourth transform: frame 2 to 1' 
	  _1xP2.pos = math::Vector3(1.333,0,0);
      // TODO: compute the fifth transform: frame 2' to 2 
      _2P2x.rot.SetFromAxis(0,0,1,theta3);      
      // TODO: compute the fourth transform: frame 3 to 2' 
	  _2xP3.pos = math::Vector3(0.8712,0,0);
      // convert all poses to transforms
      math::Matrix4 _0T0x = ToMatrix(_0P0x);
      math::Matrix4 _0xT1 = ToMatrix(_0xP1);
      math::Matrix4 _1T1x = ToMatrix(_1P1x);
      math::Matrix4 _1xT2 = ToMatrix(_1xP2);
      math::Matrix4 _2T2x = ToMatrix(_2P2x);
      math::Matrix4 _2xT3 = ToMatrix(_2xP3);

	  math::Matrix4 _0T1 = _0T0x * _0xT1; // transfrom from frame 1 to 0
	  math::Matrix4 _1T2 = _1T1x * _1xT2; // transfrom from frame 2 to 1
	  math::Matrix4 _2T3 = _2T2x * _2xT3; // transfrom from frame 3 to 2	  	  
      // position of the first joint is always (0,0,0)
      math::Vector3 p1(0.0, 0.0, 0.0);

      // TODO: compute the position of the second joint
      math::Vector3 p2 = _0T1 * ORIGIN1;

      // TODO: compute the position of the third joint
      math::Vector3 p3 =  _0T1 * _1T2 * ORIGIN2;

      // TODO: get the position of the manipulator end point
      math::Vector3 p = _0T1 * _1T2 * _2T3 * ORIGIN3;
      
      // setup the Jacobian: 3 degrees of freedom x 3 joints
      Ravelin::MatrixNd J(3,3);

      math::Vector3 Joint1 = JointAxis.Cross(p-p1);
      math::Vector3 Joint2 = JointAxis.Cross(p-p2);
      math::Vector3 Joint3 = JointAxis.Cross(p-p3);            

      J(X,J1) = Joint1[0];     J(X,J2) = Joint2[0];     J(X,J3) = Joint3[0]; 
      J(Y,J1) = Joint1[1];     J(Y,J2) = Joint2[1];     J(Y,J3) = Joint3[1]; 
      J(THETA,J1) = Joint1[2]; J(THETA,J2) = Joint2[2]; J(THETA,J3) = Joint3[2]; 

      return J;
    }

    public: void Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      _model = model;

      // store the pointer to the world
      _world = _model->GetWorld();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&JacobiansPlanarPlugin::OnUpdate, this, _1));

      // get the joints
      _j1 = model->GetJoint("joint_1");
      _j2 = model->GetJoint("joint_2");
      _j3 = model->GetJoint("joint_3");
    }

    // open up files for writing 
    public: void Init()
    {
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      Ravelin::VectorNd theta(3);

      // set the joint angles
      theta[0] = 0.0;  theta[1] = 0.0;  theta[2] = 0.0;
      SetConfig(theta);
      Ravelin::MatrixNd J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = 0.0;  theta[1] = 0.0;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = 0.0;  theta[1] = M_PI_2;  theta[2] = 0.0;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = 0.0;  theta[1] = M_PI_2;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = 0.0;  theta[2] = 0.0;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = 0.0;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = M_PI_2;  theta[2] = 0.0;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // compute the Jacobian for the next set of joint angles
      theta[0] = M_PI_2;  theta[1] = M_PI_2;  theta[2] = M_PI_2;
      SetConfig(theta);
      J = CalcJacobian(theta[0], theta[1], theta[2]);      
      std::cout << "joint angles: " << theta << std::endl;
      std::cout << "Jacobian: " << std::endl << J;

      // exit when we're done
      exit(-1);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(JacobiansPlanarPlugin)
}


