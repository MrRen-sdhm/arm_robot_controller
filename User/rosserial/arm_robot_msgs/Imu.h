#ifndef _ROS_arm_robot_msgs_Imu_h
#define _ROS_arm_robot_msgs_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace arm_robot_msgs
{

  class Imu : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;

    Imu():
      header(),
      angular_velocity(),
      linear_acceleration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType(){ return "arm_robot_msgs/Imu"; };
    virtual const char * getMD5(){ return "464450fcea736fec565d7aac952efb90"; };

  };

}
#endif
