#ifndef _ROS_std_msgs_UInt16MultiArray_h
#define _ROS_std_msgs_UInt16MultiArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/MultiArrayLayout.h"

namespace std_msgs
{

  class UInt16MultiArray : public ros::Msg
  {
    public:
      typedef std_msgs::MultiArrayLayout _layout_type;
      _layout_type layout;
      uint32_t data_length;
      typedef uint16_t _data_type;
      _data_type st_data;
      _data_type * data;

    UInt16MultiArray():
      layout(),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->layout.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->layout.deserialize(inbuffer + offset);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint16_t*)realloc(this->data, data_lengthT * sizeof(uint16_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint16_t) (*(inbuffer + offset)));
      this->st_data |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint16_t));
      }
     return offset;
    }

    virtual const char * getType(){ return "std_msgs/UInt16MultiArray"; };
    virtual const char * getMD5(){ return "52f264f1c973c4b73790d384c6cb4484"; };

  };

}
#endif
