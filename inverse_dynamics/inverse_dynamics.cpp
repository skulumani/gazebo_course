// the inverse dynamics computation 
public: void CalcInvDyn(double theta1, double theta2, double dtheta1, double dtheta2, double ddtheta1_des, double ddtheta2_des, double& tau1, double& tau2)
{
  const double CHANGEME = 0.0/0.0;

  // setup link lengths and masses
  const double l1 = 1.0, l2 = 1.0;
  const double m1 = 1.0, m2 = 1.0;

  // set acceleration due to gravity 
  double g = 9.8;

  // TODO: put "C" code output from Mathematica here
  // tau1 = ...
  // tau2 = ...
  tau1 = ((m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(theta2)) * ddtheta1_des + (m2*l2^2 + m2*l1*l2*cos(theta2))*ddtheta2_des - (m2*l1*l2*(2*dtheta1+dtheta2)*sin(theta2))*dtheta2 + g*((m1+m2)*l1*sin(theta1)+m2*l2*sin(theta1+theta2));
  tau2 = (m2*l2^2 + m2*l1*l2*cos(theta2))*ddtheta1_des + m2*l2^2*ddtheta2_des + m2*l1*l2*dtheta1*sin(theta2)*dtheta1 + m2*l2*g*sin(theta1+theta2);
  }


