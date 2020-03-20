//

#include "joystick/joystick.h"

#include <fcntl.h>
#include <unistd.h>

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */
#define JS_EVENT_INIT   0x80    /* initial state of device */

namespace joystick {

// see https://www.kernel.org/doc/Documentation/input/joystick-api.txt
struct JoystickEvent {
    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;
};

JoystickInterface::JoystickInterface() {
    if ((joystick_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK)) < 0) {
        throw std::invalid_argument("Unable to open joystick!");
    }
}

JoystickInterface::~JoystickInterface() {
    close(joystick_fd_);
}

bool JoystickInterface::Sample(JoystickValues& values) {
    JoystickEvent js_event{};
    int bytes_read = read(joystick_fd_, &js_event, sizeof(js_event));
    if (bytes_read != sizeof(js_event)) {
        return false;
    }

    if ((js_event.type & JS_EVENT_AXIS) != 0) {
        if (js_event.number == 0) {
            values.axis_1.x = -js_event.value;
        } else if (js_event.number == 1) {
            values.axis_1.y = -js_event.value;
        }
    }

    return true;
}

}
