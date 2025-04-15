#ifndef DEBUGGER_H
#define DEBUGGER_H

#include <cstring> 
#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>

class Debugger {
    public:
        static void Init(rcl_publisher_t* publisher, std_msgs__msg__String* msg);
        static void Log(const char* message);
    
    private:
        static rcl_publisher_t* debug_publisher;
        static std_msgs__msg__String* debug_msg;
    };

#endifelse if (RUN == false and SAFE == false)