#ifndef SOUND_METER_H
#define SOUND_METER_H

#include <stdint.h>

class SoundLevelMeter {
public:
    void begin();
    float measureLeq(uint16_t seconds);
};

#endif // SOUND_METER_H
