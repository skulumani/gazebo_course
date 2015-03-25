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

  class IKPlanarPlugin : public ModelPlugin
  {
    private: physics::WorldPtr _world;
    private: physics::ModelPtr _model;
    private: physics::JointPtr _j1, _j2, _j3;
    private: Ravelin::VectorNd _dq, _qdes;
    private: bool _do_position;   // if 'true' does IK on position, otherwise
                                  // uses orientation

    public: IKPlanarPlugin()
    {
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
          boost::bind(&IKPlanarPlugin::OnUpdate, this, _1));

      // get the joints
      _j1 = model->GetJoint("joint_1");
      _j2 = model->GetJoint("joint_2");
      _j3 = model->GetJoint("joint_3");

      // get the joint angles
      double theta1 = _j1->GetAngle(0).Radian();
      double theta2 = _j2->GetAngle(0).Radian();
      double theta3 = _j3->GetAngle(0).Radian();

      // set qdes
      _qdes.resize(3);
      _qdes[0] = theta1;
      _qdes[1] = theta2;
      _qdes[2] = theta3;
    }

    // open up files for writing 
    public: void Init()
    {
      // set the seed for randomization
      srand(time(NULL));
    }

    // solves J*dq = dx for dq
    private: void SolveJ(const Ravelin::MatrixNd& J, const Ravelin::VectorNd& dx, Ravelin::VectorNd& dq)
    {
      static Ravelin::LinAlgd LA;
      static Ravelin::MatrixNd A, U, V;
      static Ravelin::VectorNd S;

      A = J;
      dq = dx;
      LA.svd(A, U, S, V);
      LA.solve_LS_fast(U, S, V, dq);
    } 

    // does the Jacobian transpose method: dq = J'*dx
    private: void TransposeJ(const Ravelin::MatrixNd& J, const Ravelin::VectorNd& dx, Ravelin::VectorNd& dq)
    {
      J.transpose_mult(dx, dq);
    } 

    // computes the numerical gradient of 1/2 ||x_des - f(q)|| w.r.t. q 
    // (used for backtracking line search)
    private: Ravelin::VectorNd GradG(const math::Pose& target, double theta1, double theta2, double theta3)
    {
      const double DQ = 1e-6;

      // get the current end-effector pose
      double y = CalcOSDiff(FKin(theta1, theta2, theta3), target).GetLength();

      // compute the other end effector poses
      double y1 = CalcOSDiff(FKin(theta1+DQ, theta2, theta3), target).GetLength();
      double y2 = CalcOSDiff(FKin(theta1, theta2+DQ, theta3), target).GetLength();
      double y3 = CalcOSDiff(FKin(theta1, theta2, theta3+DQ), target).GetLength();

      // get the error between the poses
      Ravelin::VectorNd f(3);
      f[0] = y1 - y; 
      f[1] = y2 - y; 
      f[2] = y3 - y; 
      f *= 0.5 / DQ;
      return f;
    }

    // TODO: implement this using coord_frame_planar2.cpp from coord_frames
    // computes the forward kinematics function for the planar manipulator 
    private: math::Pose FKin(double theta1, double theta2, double theta3)
    {

     // setup the first pose
      math::Pose _0P0x, _0xP1, _1P1x, _1xP2, _2P2x, _2xP3, _3Pc; 
      math::Matrix4 _0T0x, _0xT1, _1T1x, _1xT2, _2T2x, _2xT3;
      _0P0x.rot.SetFromAxis(0,0,1,theta1);
       
      _0xP1.pos = math::Vector3(2,0,0);
      
      _1P1x.rot.SetFromAxis(0,0,1,theta2);
       
	  _1xP2.pos = math::Vector3(1.333,0,0);
      
      _2P2x.rot.SetFromAxis(0,0,1,theta3);
      
	  _2xP3.pos = math::Vector3(0.8712,0,0);
      	  
      // convert each pose to a Matrix4 
       _0T0x = ToMatrix(_0P0x);
       _0xT1 = ToMatrix(_0xP1);
       _1T1x = ToMatrix(_1P1x);
       _1xT2 = ToMatrix(_1xP2);
       _2T2x = ToMatrix(_2P2x);
       _2xT3 = ToMatrix(_2xP3);
      
      // get the manipulator end pose
      return (_0T0x * _0xT1 * _1T1x * _1xT2 * _2T2x * _2xT3).GetAsPose();
    }

    // TODO: implement this by copying the same function from Jacobians
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
	  _1xP2.pos = math::Vector3(4.0/3,0,0);
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
        
      // Not the whole Jacobian. Only rows 1,2, 6
      J(X,J1) = Joint1[0];     J(X,J2) = Joint2[0];     J(X,J3) = Joint3[0]; 
      J(Y,J1) = Joint1[1];     J(Y,J2) = Joint2[1];     J(Y,J3) = Joint3[1]; 
      J(THETA,J1) = JointAxis[2]; J(THETA,J2) = JointAxis[2]; J(THETA,J3) = JointAxis[2]; 

      return J;
    }

    // "wraps" angle on a differential to interval [-pi, pi]
    private: double WrapAngle(double x)
    {
    double x_out;
      // TODO: implement this might need a loop for large angles
      if (x > M_PI) 
        {
           x_out = x - 2.0*M_PI;  
        } 
      else if ( x < - M_PI)
        {
            x_out = x + 2.0*M_PI;
        } 
      else 
        {
            x_out = x;
        }
    return x_out;
    }

    // computes the operational space differential between two poses
    private: math::Vector3 CalcOSDiff(const math::Pose& xcurrent, const math::Pose& xdes)
    {
      const unsigned X = 0, Y = 1, Z = 2;
      const double INF = std::numeric_limits<double>::max();
      const double CHANGEME = INF;

      // get the transformation matrices for xcurrent and xdes
      math::Matrix4 Tcurrent = ToMatrix(xcurrent); 
      math::Matrix4 Tdes = ToMatrix(xdes); 

      // TODO: get the angles of rotation about z from Tcurrent and Tdes
      double theta_d = atan2(Tdes[1][0],Tdes[0][0]);
      double theta_c = atan2(Tcurrent[1][0],Tcurrent[0][0]);

      // TODO: compute the difference in angles
      double theta_diff = theta_d - theta_c;

      // wrap the differential to [-pi,pi]
      theta_diff = WrapAngle(theta_diff);

      // TODO: get the difference in position
      double x_diff = Tdes[0][3] - Tcurrent[0][3];
      double y_diff = Tdes[1][3] - Tcurrent[1][3]; 
 
      // construct the differential
      math::Vector3 dx(x_diff, y_diff, theta_diff);

      // zero the parts that we do not want to use for IK
      if (_do_position)
        dx.z = 0.0;
      else
        dx.x = dx.y = 0.0;

      return dx;
    }

    // does inverse kinematics using Resolved Motion Rate Control
    private: void DoIK(const math::Pose& target)
    {
      const double DX_TOL = 1e-4, LOCAL_MIN_TOL = 1e-8;
      double min_dx = std::numeric_limits<double>::max();

      // get the current joint angles
      double theta1 = _j1->GetAngle(0).Radian();
      double theta2 = _j2->GetAngle(0).Radian();
      double theta3 = _j3->GetAngle(0).Radian();

      // set the number of iterations and restarts 
      unsigned iter = 0, restarts = 0;

      // do the IK process
      while (true)
      {
        // TODO: get the current end-effector pose
        math::Pose x = FKin(theta1, theta2, theta3);// = FILL ME IN 

        // get the error between the current and desired poses
        math::Vector3 deltax = CalcOSDiff(x, target); 

        // compute the gradient of 1/2 * ||x_des - f(q)|| - we will use this
        // for the backtracking line search below
        Ravelin::VectorNd grad = GradG(target, theta1, theta2, theta3);

        // convert the error to a Ravelin vector
        Ravelin::VectorNd dx(3);
        dx[0] = deltax[0];
        dx[1] = deltax[1];
        dx[2] = deltax[2];

        // if there's hardly any error, quit
        const double DX_NORM = dx.norm();
        if (DX_NORM < DX_TOL)
          
          break;

        // record smallest dx
        min_dx = std::min(min_dx, DX_NORM);
        std::cout << "dx norm: " << DX_NORM << "  minimum dx observed: " << min_dx << std::endl;

        // TODO: get the Jacobian
        Ravelin::MatrixNd J = CalcJacobian(theta1,theta2, theta3);// = FILL ME IN 

        // TODO: "solve" J*dq = dx for _dq using SolveJ or TransposeJ
        //Ravelin::VectorNd _dq;
        //SolveJ(J, dx, _dq);  5 iters
	  TransposeJ(J,dx, _dq); // this one sucks
        // do backtracking search to determine value of t 
        const double ALPHA = 0.05, BETA = 0.5;
        double t = 1.0;

        // TODO: compute f(q + dq*t)
        math::Pose xprime = FKin(theta1+_dq[0]*t,theta2+_dq[1]*t,theta3+_dq[2]*t);// = FILL ME IN
        math::Vector3 deltax_prime = CalcOSDiff(xprime, target); 
        while (0.5*deltax_prime.GetLength() > 0.5*deltax.GetLength() + ALPHA*t*grad.dot(_dq))
        {
          // reduce t geometrically
          t*= BETA;

          // if t becomes too small (smaller than machine epsilon), quit
          if (t < std::numeric_limits<double>::epsilon())
            break;

          // TODO: recompute f(q + dq*t)
          // xprime = FILL ME IN 
            xprime = FKin(theta1+_dq[0]*t,theta2+_dq[1]*t,theta3+_dq[2]*t);
          // recompute deltax_prime
          deltax_prime = CalcOSDiff(xprime, target); 
        }

        // update change in q         
        _dq *= t;

        // if this block of code is triggered, we do not have a good descent
        // direction - we hit a local minimum; try again from a random starting
        // point 
        if (_dq.norm() < LOCAL_MIN_TOL)
        {
          std::cout << "-- hit a local minimum: norm dq (" << _dq.norm() << "), norm dx (" << DX_NORM <<") " << std::endl << " -- resetting joint angles and trying again" << std::endl;
          theta1 = (double) rand() / RAND_MAX * 2.0 * M_PI;
          theta2 = (double) rand() / RAND_MAX * 2.0 * M_PI;
          theta3 = (double) rand() / RAND_MAX * 2.0 * M_PI;
          iter = 0;
          restarts++;
          continue;
        }

        // TODO: update thetas using _dq
        theta1 = theta1 + _dq[0];
        theta2 = theta2 + _dq[1];
        theta3 = theta3 + _dq[2];

	  iter++;
      }

      // update qdes using the IK solution: this will allow the robot to go
      // to the IK solution (using the controller in OnUpdate(.)) 

      _qdes[0] = theta[0];
      _qdes[1] = theta[1];
      _qdes[2] = theta[2];


      // indicate IK solution found
      std::cout << "IK solution found after " << restarts << " restarts and " << iter << " iterations" << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double theta[3], dtheta[3], perr[3];
      double f[3];
      const double KP = 100.0, KD = 25.0; 
      static bool first_time = true;

      // setup the target
      if (first_time)
      {
        // determine whether to solve for position
        _do_position = true;

        // find the target pose using randomization 
        math::Pose target = FKin((double) rand() / RAND_MAX, (double) rand() / RAND_MAX, (double) rand() / RAND_MAX);
        std::cout << "Solving to target: " << target << std::endl;
        std::cout << "Solving for position? " << _do_position << std::endl;
        DoIK(target);
        first_time = false;
      }

      // get current joint angles 
      theta[0] = _j1->GetAngle(0).Radian();
      theta[1] = _j2->GetAngle(0).Radian();
      theta[2] = _j3->GetAngle(0).Radian();

      // get current joint velocities
      dtheta[0] = _j1->GetVelocity(0);
      dtheta[1] = _j2->GetVelocity(0);
      dtheta[2] = _j3->GetVelocity(0);

      // compute position error
      perr[0] = (_qdes[0] - theta[0]);
      perr[1] = (_qdes[1] - theta[1]);
      perr[2] = (_qdes[2] - theta[2]);
      
      // compute PD torques
      f[0] = KP*perr[0] - KD*dtheta[0];
      f[1] = KP*perr[1] - KD*dtheta[1];
      f[2] = KP*perr[2] - KD*dtheta[2];

      // apply PD torques
      _j1->SetForce(0, f[0]);
      _j2->SetForce(0, f[1]);
      _j3->SetForce(0, f[2]);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(IKPlanarPlugin)
}


