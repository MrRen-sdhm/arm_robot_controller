#ifndef _ROS_SERVICE_SetIO_h
#define _ROS_SERVICE_SetIO_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

static const char SETIO[] = "ur_msgs/SetIO";

  class SetIORequest : public ros::Msg
  {
    public:
      typedef int8_t _fun_type;
      _fun_type fun;
      typedef int8_t _pin_type;
      _pin_type pin;
      typedef float _state_type;
      _state_type state;
      enum { FUN_SET_DIGITAL_OUT =  1 };
      enum { FUN_SET_FLAG =  2 };
      enum { FUN_SET_ANALOG_OUT =  3 };
      enum { FUN_SET_TOOL_VOLTAGE =  4 };

    SetIORequest():
      fun(0),
      pin(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_fun;
      u_fun.real = this->fun;
      *(outbuffer + offset + 0) = (u_fun.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fun);
      union {
        int8_t real;
        uint8_t base;
      } u_pin;
      u_pin.real = this->pin;
      *(outbuffer + offset + 0) = (u_pin.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pin);
      union {
        float real;
        uint32_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_fun;
      u_fun.base = 0;
      u_fun.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fun = u_fun.real;
      offset += sizeof(this->fun);
      union {
        int8_t real;
        uint8_t base;
      } u_pin;
      u_pin.base = 0;
      u_pin.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pin = u_pin.real;
      offset += sizeof(this->pin);
      union {
        float real;
        uint32_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return SETIO; };
    const char * getMD5(){ return "144e866c50167f6d4b99948cba355b8c"; };

  };

  class SetIOResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetIOResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETIO; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetIO {
    public:
    typedef SetIORequest Request;
    typedef SetIOResponse Response;
  };

}
#endif
