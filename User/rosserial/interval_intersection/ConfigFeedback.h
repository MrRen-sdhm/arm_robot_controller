#ifndef _ROS_interval_intersection_ConfigFeedback_h
#define _ROS_interval_intersection_ConfigFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace interval_intersection
{

  class ConfigFeedback : public ros::Msg
  {
    public:

    ConfigFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "interval_intersection/ConfigFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif