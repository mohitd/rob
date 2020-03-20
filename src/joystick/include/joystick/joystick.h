//

#pragma once

#include <iostream>
#include <limits>

namespace joystick {


struct JoystickValues {
    static constexpr short MAX_AXIS_VALUE = std::numeric_limits<short>::max();
    static constexpr short MIN_AXIS_VALUE = std::numeric_limits<short>::min();

    struct Axis {
        int x, y;
    };

    Axis axis_1;
};

class JoystickInterface {
public:
    JoystickInterface();
    ~JoystickInterface();

    bool Sample(JoystickValues& values);
private:
    int joystick_fd_;
};

}

