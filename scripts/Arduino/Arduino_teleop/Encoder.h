#ifndef _ENCODERS
#define _ENCODERS

#include <ams_as5048b.h>

//********** ENCODER CLASS *************

class Encoder
{
  public:
    virtual void setup_enc();
    virtual void encoder_loop();
    
    virtual double get();
    virtual double getAngle();
    virtual void set_zero(double offset);

    int sign = 1;
  protected:
    double _offset = 0.0;
};


// -------- AMS encoder ------

class Encoder_ams : public Encoder
{
  public:
    Encoder_ams(int address, int frequence_update, int unit, bool switch_sign);
    virtual void setup_enc();
    virtual void encoder_loop();
    
    virtual double get();
    virtual double getAngle();
    virtual void set_zero(double offset);

  private:
    AMS_AS5048B _encoder;
    int _unit;
    long _time_last_update;
    int _freq_update;
};

Encoder_ams::Encoder_ams(int address, int frequence_update, int unit = U_RAD, bool switch_sign = false)
  : _encoder(address), _unit(unit), _time_last_update(millis()), _freq_update(frequence_update)
{
  if (switch_sign)
    sign = -1;
}

void Encoder_ams::setup_enc()
{
  _encoder.begin();
  this->set_zero(0);
}

void Encoder_ams::encoder_loop()
{
  // Must be call in loop or else moving average will not work
  if (millis() - _time_last_update > _freq_update)
  {
    _encoder.updateMovingAvgExp();
    _time_last_update = millis();
  }
}

double Encoder_ams::get()
{
  double pos = _encoder.getMovingAvgExp(_unit)*sign + _offset;
  if (isnan(pos))
    pos = 0.;
  return pos;
}

double Encoder_ams::getAngle()
{
  if (_unit == U_DEG) 
  {
    double posDeg = this->get();
    return fmod(posDeg, 360);
  }
  
  double posRad = this->get();
  return fmod(posRad, (2*M_PI));

}

void Encoder_ams::set_zero(double offset)
{
  _offset = offset;
  _encoder.setZeroReg();
}



// ------- Other encoder (polulu) --------

class Encoder_oth : public Encoder
{
  public:
    Encoder_oth(int channel_a, int channel_b, long counts_per_rev, bool switch_sign);
    virtual void setup_enc(){}
    virtual void encoder_loop(){}
    
    virtual double get();
    virtual double getAngle();
    
    virtual void set_zero(double offset);

    void modify_count();

  private:
    static Encoder_oth *instance;
    
    static void modify_count_ISR(); // function for interrupt (needs to be static)

    double pulse2pos(double counter);
    
    float _ratio;
    int _ch_a, _ch_b;
    double _counter;
};


Encoder_oth::Encoder_oth(int channel_a, int channel_b, long counts_per_rev, bool switch_sign = false)
  : _ratio((2*M_PI)/counts_per_rev), _ch_a(channel_a), _ch_b(channel_b)
{
  _counter = 0;
  
  pinMode(channel_a, INPUT_PULLUP);
  pinMode(channel_b, INPUT_PULLUP);

  // give pointer to instance so it can call function modify count
  instance = this;

  //Interrupt pin correct values 2, 3, 18, 19
  //attachInterrupt(digitalPinToInterrupt(channel_a), Encoder_oth::modify_count_ISR, RISING);

  if (switch_sign)
    sign = -1;
}


// Forward to non-static member function.
void Encoder_oth::modify_count_ISR()
{
  if (instance)
    instance->modify_count();
}

void Encoder_oth::modify_count()
{
  int b = digitalRead(_ch_b);
  
  if(b>0) _counter += 1*sign;
  else _counter -= 1*sign;
}

double Encoder_oth::get()
{
  return (pulse2pos(_counter)*sign + _offset);
}

double Encoder_oth::pulse2pos(double counter)
{
  return counter * _ratio;
}

double Encoder_oth::getAngle()
{  
  return fmod(get(), 2*M_PI);
}

void Encoder_oth::set_zero(double offset)
{
  _offset = offset;
  _counter = 0;
}

// to explain why you need to do that go see here
// https://forum.arduino.cc/t/using-an-interrupt-from-a-method-inside-a-class/486521

Encoder_oth* Encoder_oth::instance = NULL;

#endif
