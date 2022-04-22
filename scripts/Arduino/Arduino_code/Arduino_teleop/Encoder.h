#include <ams_as5048b.h>

//********** ENCODER CLASS *************

class Encoder
{
  public:
    virtual void setup_enc();
    virtual void encoder_loop();
    
    virtual double get();
    virtual void set_zero(double offset);
  protected:
    double _offset = 0.0;
};


// -------- AMS encoder ------

class Encoder_ams : public Encoder
{
  public:
    Encoder_ams(int address, int frequence_update, int unit);
    virtual void setup_enc();
    virtual void encoder_loop();
    
    virtual double get();
    virtual void set_zero(double offset);

  private:
    AMS_AS5048B _encoder;
    int _unit;
    long _time_last_update;
    int _freq_update;
};

Encoder_ams::Encoder_ams(int address, int frequence_update, int unit = U_RAD)
  : _encoder(address), _unit(unit), _time_last_update(millis()), _freq_update(frequence_update){}

void Encoder_ams::setup_enc()
{
  _encoder.begin();
  //this->set_zero(0);
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
  return _encoder.getMovingAvgExp(_unit) + _offset;
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
    Encoder_oth(int channel_a, int channel_b, long counts_per_rev);
    virtual void setup_enc(){}
    virtual void encoder_loop(){}
    
    virtual double get();
    virtual void set_zero(double offset);

  private:
    static Encoder_oth *instance;
    
    static void modify_count_ISR(); // function for interrupt (needs to be static)
    void modify_count();
    
    float _ratio;
    int _ch_a, _ch_b;
    int _counter;
};


Encoder_oth::Encoder_oth(int channel_a, int channel_b, long counts_per_rev)
  : _ratio((2*M_PI)/counts_per_rev), _ch_a(channel_a), _ch_b(channel_b)
{
  _counter = 0;
  
  pinMode(channel_a, INPUT_PULLUP);
  pinMode(channel_b, INPUT_PULLUP);

  // give pointer to instance so it can call function modify count
  instance = this;

  //Interrupt pin correct values 2, 3, 18, 19
  attachInterrupt(digitalPinToInterrupt(channel_a), Encoder_oth::modify_count_ISR, RISING);
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
  
  if(b>0) _counter++;
  else _counter--;
}

double Encoder_oth::get()
{
  double pos_rad = fmod((_counter * _ratio),(2*M_PI));
  //if (pos_rad >=M_PI)pos_rad -= 2*M_PI;
  //if (pos_rad >=M_PI)pos_rad += 2*M_PI;
  
  return (pos_rad + _offset);
}

void Encoder_oth::set_zero(double offset)
{
  _offset = offset;
  _counter = 0;
}

// to explain why you need to do that go see here
// https://forum.arduino.cc/t/using-an-interrupt-from-a-method-inside-a-class/486521

Encoder_oth* Encoder_oth::instance = NULL;
