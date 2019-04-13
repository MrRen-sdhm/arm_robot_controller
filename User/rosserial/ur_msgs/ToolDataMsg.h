#ifndef _ROS_ur_msgs_ToolDataMsg_h
#define _ROS_ur_msgs_ToolDataMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

  class ToolDataMsg : public ros::Msg
  {
    public:
      typedef int8_t _analog_input_range2_type;
      _analog_input_range2_type analog_input_range2;
      typedef int8_t _analog_input_range3_type;
      _analog_input_range3_type analog_input_range3;
      typedef float _analog_input2_type;
      _analog_input2_type analog_input2;
      typedef float _analog_input3_type;
      _analog_input3_type analog_input3;
      typedef float _tool_voltage_48v_type;
      _tool_voltage_48v_type tool_voltage_48v;
      typedef uint8_t _tool_output_voltage_type;
      _tool_output_voltage_type tool_output_voltage;
      typedef float _tool_current_type;
      _tool_current_type tool_current;
      typedef float _tool_temperature_type;
      _tool_temperature_type tool_temperature;
      typedef uint8_t _tool_mode_type;
      _tool_mode_type tool_mode;
      enum { ANALOG_INPUT_RANGE_CURRENT =  0 };
      enum { ANALOG_INPUT_RANGE_VOLTAGE =  1 };
      enum { TOOL_BOOTLOADER_MODE =  249 };
      enum { TOOL_RUNNING_MODE =  253 };
      enum { TOOL_IDLE_MODE =  255 };

    ToolDataMsg():
      analog_input_range2(0),
      analog_input_range3(0),
      analog_input2(0),
      analog_input3(0),
      tool_voltage_48v(0),
      tool_output_voltage(0),
      tool_current(0),
      tool_temperature(0),
      tool_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range2;
      u_analog_input_range2.real = this->analog_input_range2;
      *(outbuffer + offset + 0) = (u_analog_input_range2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->analog_input_range2);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range3;
      u_analog_input_range3.real = this->analog_input_range3;
      *(outbuffer + offset + 0) = (u_analog_input_range3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->analog_input_range3);
      offset += serializeAvrFloat64(outbuffer + offset, this->analog_input2);
      offset += serializeAvrFloat64(outbuffer + offset, this->analog_input3);
      union {
        float real;
        uint32_t base;
      } u_tool_voltage_48v;
      u_tool_voltage_48v.real = this->tool_voltage_48v;
      *(outbuffer + offset + 0) = (u_tool_voltage_48v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tool_voltage_48v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tool_voltage_48v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tool_voltage_48v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_voltage_48v);
      *(outbuffer + offset + 0) = (this->tool_output_voltage >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tool_output_voltage);
      union {
        float real;
        uint32_t base;
      } u_tool_current;
      u_tool_current.real = this->tool_current;
      *(outbuffer + offset + 0) = (u_tool_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tool_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tool_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tool_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_current);
      union {
        float real;
        uint32_t base;
      } u_tool_temperature;
      u_tool_temperature.real = this->tool_temperature;
      *(outbuffer + offset + 0) = (u_tool_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tool_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tool_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tool_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_temperature);
      *(outbuffer + offset + 0) = (this->tool_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tool_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range2;
      u_analog_input_range2.base = 0;
      u_analog_input_range2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->analog_input_range2 = u_analog_input_range2.real;
      offset += sizeof(this->analog_input_range2);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range3;
      u_analog_input_range3.base = 0;
      u_analog_input_range3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->analog_input_range3 = u_analog_input_range3.real;
      offset += sizeof(this->analog_input_range3);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->analog_input2));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->analog_input3));
      union {
        float real;
        uint32_t base;
      } u_tool_voltage_48v;
      u_tool_voltage_48v.base = 0;
      u_tool_voltage_48v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tool_voltage_48v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tool_voltage_48v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tool_voltage_48v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tool_voltage_48v = u_tool_voltage_48v.real;
      offset += sizeof(this->tool_voltage_48v);
      this->tool_output_voltage =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->tool_output_voltage);
      union {
        float real;
        uint32_t base;
      } u_tool_current;
      u_tool_current.base = 0;
      u_tool_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tool_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tool_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tool_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tool_current = u_tool_current.real;
      offset += sizeof(this->tool_current);
      union {
        float real;
        uint32_t base;
      } u_tool_temperature;
      u_tool_temperature.base = 0;
      u_tool_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tool_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tool_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tool_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tool_temperature = u_tool_temperature.real;
      offset += sizeof(this->tool_temperature);
      this->tool_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->tool_mode);
     return offset;
    }

    const char * getType(){ return "ur_msgs/ToolDataMsg"; };
    const char * getMD5(){ return "404fc266f37d89f75b372d12fa94a122"; };

  };

}
#endif