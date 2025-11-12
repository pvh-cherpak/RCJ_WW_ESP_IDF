#include <iot_button.h>

class DribblerMicroswitch_t
{
private:
    button_handle_t microswitch;
public:
    DribblerMicroswitch_t(/* args */);
    void init(int pin, uint16_t hold_time, bool active_state);
    bool ballCatched();
    ~DribblerMicroswitch_t();
};



