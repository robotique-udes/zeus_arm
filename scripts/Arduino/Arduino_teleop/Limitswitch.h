
// LIMIT SWITCH CLASS

class Limitswitch
{
  public:
    Limitswitch(int channel, bool reverse);
    bool get();
  private:
    int _channel;
    bool _reverse;
};

Limitswitch::Limitswitch(int channel, bool reverse = false)
  : _channel(channel), _reverse(reverse)
{
  pinMode(channel, INPUT);
}

bool Limitswitch::get()
{
  bool state = digitalRead(_channel);
  if (_reverse) state = !state;
  return (state);
}




