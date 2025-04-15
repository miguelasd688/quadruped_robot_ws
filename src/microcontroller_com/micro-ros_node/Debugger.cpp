#include "Debugger.h"


rcl_publisher_t* Debugger::debug_publisher = nullptr;
std_msgs__msg__String* Debugger::debug_msg = nullptr;

void Debugger::Init(rcl_publisher_t* publisher, std_msgs__msg__String* msg) {
    debug_publisher = publisher;
    debug_msg = msg;
}

void Debugger::Log(const char* message) {
    if (!message || !debug_publisher || !debug_msg) return;

    size_t len = strlen(message);
    if (len >= debug_msg->data.capacity) len = debug_msg->data.capacity - 1;

    memcpy(debug_msg->data.data, message, len);
    debug_msg->data.data[len] = '\0';
    debug_msg->data.size = len;

    rcl_publish(debug_publisher, debug_msg, NULL);
}