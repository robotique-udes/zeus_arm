#ifndef _JOINTS
#define _JOINTS

#include "Motor.h"
#include "Limitswitch.h"

#include <PID_v1.h>

// Max number of limitswitch
#define MAX_LIM 2

// JOINT CLASS
class Joint
{
  public:
    Joint(unsigned int timePeriodCom);

    //Ax limit
    void setAxLimit(Limitswitch* swtch, int dir);

    // Comm (if lost comm sends velociy of 0)
    void UpdateLastComm();
    void CheckForComm();

    // Loop
    virtual void JointLoop();
    void setOpenLoop() { _closedLoopCtrl = false; }
    bool isOpenLoop() { return !_closedLoopCtrl; }
    
    //Goes from -1.0 to 1 if in opened-loop, else its in rad/s
    double velSetpoint = 0.0;

    void BypassComm() { _bypassComm = true; }

    double Debug();

  protected:
    bool _closedLoopCtrl = false;
    double _cmd = 0.0;
    
    void AxLimits();
    bool _bypassComm = false;


    unsigned long _timeLastCom = 0.0; //comm watchdog
    unsigned int _timePeriodCom;

    // Ax limit
    struct axLimit
    {
      Limitswitch* swtch;
      int dir;
    };
    axLimit _limits[MAX_LIM];
    int _nLim = 0;
    
};


Joint::Joint(unsigned int timePeriodCom)
  : _timePeriodCom(timePeriodCom)
{   
  velSetpoint = 0.0;

  _cmd = 0.0;
  _timeLastCom = 0.0;
  _closedLoopCtrl = false;
}


void Joint::setAxLimit(Limitswitch* swtch, int dir)
{
  if (_nLim < MAX_LIM)
  {
    _limits[_nLim] = {swtch, dir};
    _nLim += 1;
  }
}

void Joint::AxLimits()
{
  for (int i = 0; i<_nLim; i++)
    if (_limits[i].swtch->get())
    {
      if (_cmd*_limits[i].dir > 0.0) //same sign
        _cmd = 0.0;
    }
}

void Joint::UpdateLastComm()
{ 
  _timeLastCom = millis();
}

void Joint::CheckForComm()
{
  if (millis() - _timeLastCom > _timePeriodCom)
    _cmd = 0.0;
}

double Joint::Debug()
{
  return _cmd;
}

#endif
