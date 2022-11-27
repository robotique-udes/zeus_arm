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

    void setOpenLoop();
    bool isOpenLoop();

    void BypassComm();
    void UpdateLastComm();
    void CheckForComm();
    void AxLimits();

    void calculatVelSetpoints();
    void JointLoop();

    bool velDiffReached = false;

    SingleJoint* _jointL = NULL, *_jointR = NULL;

    double _maxDiffInSpeed = 0;

    double _velSetpointJoint1 = 0.0, _velSetpointJoint2 = 0.0;

    int jointActive;

};


DifferentialJoint::DifferentialJoint(SingleJoint* joint1, SingleJoint* joint2, double maxDiffInSpeed)
  : Joint(0), _jointL(joint1), _jointR(joint2), _maxDiffInSpeed(maxDiffInSpeed)
{   
  jointActive = 1;
}

void DifferentialJoint::setVelSetpoint(double velSetpointJoint1, double velSetpointJoint2)
{
  _velSetpointJoint1 = velSetpointJoint1;
  _velSetpointJoint2 = velSetpointJoint2;
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

void DifferentialJoint::CheckForComm() 
{
  if (_jointL != NULL)
    _jointL->CheckForComm();
  if (_jointR != NULL)
    _jointR->CheckForComm();
}

void DifferentialJoint::BypassComm()
{
  _bypassComm = true;
  if (_jointL != NULL)
    _jointL->BypassComm();
  if (_jointR != NULL)
    _jointR->BypassComm();
}

void DifferentialJoint::AxLimits()
{
  // this should be done more ellegantly, but right now i assume 
  // the joint limits are only on the first joint 
  if (jointActive == 1)
  {
    for (int i = 0; i<_nLim; i++)
      if (_limits[i].swtch->get())
      {
        if (_velSetpointJoint1*_limits[i].dir > 0.0) //same sign
          setVelSetpoint(0, _velSetpointJoint2);
      }
  }
}

void DifferentialJoint::calculatVelSetpoints()
{
  // adjust vel setpoints for either joint
  if (_jointL != NULL && _jointR != NULL && !velDiffReached)
  {
    if (_velSetpointJoint1 != 0)
    {
      // this means we move the first joint (so same direction, inverse speed cmd since motors are faced togheter)
      _jointL->velSetpoint = _velSetpointJoint1;
      _jointR->velSetpoint = -_velSetpointJoint1;
      jointActive = 1;
    }
    else
    {
      // this means we move joint2 (so opposite directions)
      _jointL->velSetpoint = _velSetpointJoint2;
      _jointR->velSetpoint = _velSetpointJoint2;
      jointActive = 2;
    }
  }
  else
  {
    jointActive = 0;
    _velSetpointJoint1 = 0.0;
    _velSetpointJoint2 = 0.0;
  } 
}


void DifferentialJoint::JointLoop()
{ 
  // Check if speed for each singleJoint is different and adjust command for each
  // (for now there is no pid adjustment, we just monitor de difference and stop everything if the difference is too much)
  if (_maxDiffInSpeed != 0)
    if (abs(_jointL->actualVel - _jointR->actualVel) > _maxDiffInSpeed)
      velDiffReached = true;

  calculatVelSetpoints();

  AxLimits();
  if (!_bypassComm)
    CheckForComm();
  _bypassComm = false;


  if (_jointL != NULL && _jointR != NULL)
  {
    _jointL->JointLoop();
    _jointR->JointLoop();
  }


}

#endif
