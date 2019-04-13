#ifndef _ROS_ur_msgs_IOStates_h
#define _ROS_ur_msgs_IOStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"

namespace ur_msgs
{

  class IOStates : public ros::Msg
  {
    public:
      uint32_t digital_in_states_length;
      typedef ur_msgs::Digital _digital_in_states_type;
      _digital_in_states_type st_digital_in_states;
      _digital_in_states_type * digital_in_states;
      uint32_t digital_out_states_length;
      typedef ur_msgs::Digital _digital_out_states_type;
      _digital_out_states_type st_digital_out_states;
      _digital_out_states_type * digital_out_states;
      uint32_t flag_states_length;
      typedef ur_msgs::Digital _flag_states_type;
      _flag_states_type st_flag_states;
      _flag_states_type * flag_states;
      uint32_t analog_in_states_length;
      typedef ur_msgs::Analog _analog_in_states_type;
      _analog_in_states_type st_analog_in_states;
      _analog_in_states_type * analog_in_states;
      uint32_t analog_out_states_length;
      typedef ur_msgs::Analog _analog_out_states_type;
      _analog_out_states_type st_analog_out_states;
      _analog_out_states_type * analog_out_states;

    IOStates():
      digital_in_states_length(0), digital_in_states(NULL),
      digital_out_states_length(0), digital_out_states(NULL),
      flag_states_length(0), flag_states(NULL),
      analog_in_states_length(0), analog_in_states(NULL),
      analog_out_states_length(0), analog_out_states(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->digital_in_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->digital_in_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->digital_in_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->digital_in_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->digital_in_states_length);
      for( uint32_t i = 0; i < digital_in_states_length; i++){
      offset += this->digital_in_states[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->digital_out_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->digital_out_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->digital_out_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->digital_out_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->digital_out_states_length);
      for( uint32_t i = 0; i < digital_out_states_length; i++){
      offset += this->digital_out_states[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->flag_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flag_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->flag_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->flag_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flag_states_length);
      for( uint32_t i = 0; i < flag_states_length; i++){
      offset += this->flag_states[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->analog_in_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->analog_in_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->analog_in_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->analog_in_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->analog_in_states_length);
      for( uint32_t i = 0; i < analog_in_states_length; i++){
      offset += this->analog_in_states[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->analog_out_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->analog_out_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->analog_out_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->analog_out_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->analog_out_states_length);
      for( uint32_t i = 0; i < analog_out_states_length; i++){
      offset += this->analog_out_states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t digital_in_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      digital_in_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      digital_in_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      digital_in_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->digital_in_states_length);
      if(digital_in_states_lengthT > digital_in_states_length)
        this->digital_in_states = (ur_msgs::Digital*)realloc(this->digital_in_states, digital_in_states_lengthT * sizeof(ur_msgs::Digital));
      digital_in_states_length = digital_in_states_lengthT;
      for( uint32_t i = 0; i < digital_in_states_length; i++){
      offset += this->st_digital_in_states.deserialize(inbuffer + offset);
        memcpy( &(this->digital_in_states[i]), &(this->st_digital_in_states), sizeof(ur_msgs::Digital));
      }
      uint32_t digital_out_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      digital_out_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      digital_out_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      digital_out_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->digital_out_states_length);
      if(digital_out_states_lengthT > digital_out_states_length)
        this->digital_out_states = (ur_msgs::Digital*)realloc(this->digital_out_states, digital_out_states_lengthT * sizeof(ur_msgs::Digital));
      digital_out_states_length = digital_out_states_lengthT;
      for( uint32_t i = 0; i < digital_out_states_length; i++){
      offset += this->st_digital_out_states.deserialize(inbuffer + offset);
        memcpy( &(this->digital_out_states[i]), &(this->st_digital_out_states), sizeof(ur_msgs::Digital));
      }
      uint32_t flag_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      flag_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      flag_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      flag_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->flag_states_length);
      if(flag_states_lengthT > flag_states_length)
        this->flag_states = (ur_msgs::Digital*)realloc(this->flag_states, flag_states_lengthT * sizeof(ur_msgs::Digital));
      flag_states_length = flag_states_lengthT;
      for( uint32_t i = 0; i < flag_states_length; i++){
      offset += this->st_flag_states.deserialize(inbuffer + offset);
        memcpy( &(this->flag_states[i]), &(this->st_flag_states), sizeof(ur_msgs::Digital));
      }
      uint32_t analog_in_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      analog_in_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      analog_in_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      analog_in_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->analog_in_states_length);
      if(analog_in_states_lengthT > analog_in_states_length)
        this->analog_in_states = (ur_msgs::Analog*)realloc(this->analog_in_states, analog_in_states_lengthT * sizeof(ur_msgs::Analog));
      analog_in_states_length = analog_in_states_lengthT;
      for( uint32_t i = 0; i < analog_in_states_length; i++){
      offset += this->st_analog_in_states.deserialize(inbuffer + offset);
        memcpy( &(this->analog_in_states[i]), &(this->st_analog_in_states), sizeof(ur_msgs::Analog));
      }
      uint32_t analog_out_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      analog_out_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      analog_out_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      analog_out_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->analog_out_states_length);
      if(analog_out_states_lengthT > analog_out_states_length)
        this->analog_out_states = (ur_msgs::Analog*)realloc(this->analog_out_states, analog_out_states_lengthT * sizeof(ur_msgs::Analog));
      analog_out_states_length = analog_out_states_lengthT;
      for( uint32_t i = 0; i < analog_out_states_length; i++){
      offset += this->st_analog_out_states.deserialize(inbuffer + offset);
        memcpy( &(this->analog_out_states[i]), &(this->st_analog_out_states), sizeof(ur_msgs::Analog));
      }
     return offset;
    }

    const char * getType(){ return "ur_msgs/IOStates"; };
    const char * getMD5(){ return "0a5c7b73e3189e9a2caf8583d1bae2e2"; };

  };

}
#endif