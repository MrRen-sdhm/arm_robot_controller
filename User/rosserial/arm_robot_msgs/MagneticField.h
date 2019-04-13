#ifndef _ROS_arm_robot_msgs_MagneticField_h
#define _ROS_arm_robot_msgs_MagneticField_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace arm_robot_msgs
{

  class MagneticField : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _magnetic_field_type;
      _magnetic_field_type magnetic_field;

    MagneticField():
      header(),
      magnetic_field()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->magnetic_field.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType(){ return "arm_robot_msgs/MagneticField"; };
    virtual const char * getMD5(){ return "300c3173930958ac0ced7a90bc6c44a2"; };

  };

}
#endif
