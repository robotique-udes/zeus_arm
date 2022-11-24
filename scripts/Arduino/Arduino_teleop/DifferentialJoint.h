#ifndef _DIFFJOINT
#define _DIFFJOINT

#include "Motor.h"
#include "Joint.h"
#include "Encoder.h"

// Max number of limitswitch
#define MAX_LIM 2


// JOINT CLASS
class DifferentialJoint : public Joint
{
  public:
    DifferentialJoint(SingleJoint* jointL, SingleJoint* jointR, double maxDiffInSpeed);

    void setVelSetpoint(double velSetpointJoint1, double velSetpointJoint2);

    void JointLoop();

    bool velDiffReached = false;

    
  private:
    SingleJoint* _jointL = NULL, *_jointR = NULL;
    double _maxDiffInSpeed;

    double _velSetpointJoint1 = 0.0, _velSetpointJoint2 = 0.0;

    void calculatVelSetpoints();

};


DifferentialJoint::DifferentialJoint(SingleJoint* joint1, SingleJoint* joint2, double maxDiffInSpeed)
  : _jointL(jointL), _jointR(jointR), _maxDiffInSpeed(maxDiffInSpeed)
{   

}

void DifferentialJoint::setOpenLoop() 
{ 
  if (_jointL != NULL && _jointR != NULL) {
    _jointL->setOpenLoop();
    _jointR->setOpenLoop();
  }
}


bool DifferentialJoint::isOpenLoop() 
{ 
  if (_jointL != NULL && _jointR != NULL)
    return _jointL->isOpenLoop(); 
  return true;
}


void DifferentialJoint::UpdateLastComm() 
{
  if (_jointL != NULL && _jointR != NULL)
  {
    _jointL->UpdateLastComm();
    _jointR->UpdateLastComm();
  }
}

void DifferentialJoint::calculatVelSetpoints()
{
  // adjust vel setpoints for either joint
  if (_jointL != NULL && _jointR != NULL && !velDiffReached)
  {
    if (velSetpointJoint1 != 0)
    {
      // this means we move the first joint (so same direction)
      _jointL->velSetpoint = velSetpointJoint1;
      _jointR->velSetpoint = velSetpointJoint1;
    }
    else
    {
      // this means we move joint2 (so opposite directions)
      _jointL->velSetpoint = velSetpointJoint2;
      _jointR->velSetpoint = -velSetpointJoint2;
    }
  }
  else
  {
    _velSetpointJoint1 = 0.0;
    _velSetpointJoint2 = 0.0;
  } 
}


void Joint::JointLoop()
{ 
  // Check if speed for each singleJoint is different and adjust command for each
  // (for now there is no pid adjustment, we just monitor de difference and stop everything if the difference is too much)
  if (abs())
    velDiffReached = true;

  calculatVelSetpoints();

  if (_jointL != NULL && _jointR != NULL)
  {
    _jointL->JointLoop();
    _jointR->JointLoop();
  }


}

#endif
