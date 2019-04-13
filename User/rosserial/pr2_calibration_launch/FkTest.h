#ifndef _ROS_SERVICE_FkTest_h
#define _ROS_SERVICE_FkTest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_calibration_launch
{

static const char FKTEST[] = "pr2_calibration_launch/FkTest";

  class FkTestRequest : public ros::Msg
  {
    public:
      typedef const char* _root_type;
      _root_type root;
      typedef const char* _tip_type;
      _tip_type tip;
      uint32_t joint_positions_length;
      typedef float _joint_positions_type;
      _joint_positions_type st_joint_positions;
      _joint_positions_type * joint_positions;

    FkTestRequest():
      root(""),
      tip(""),
      joint_positions_length(0), joint_positions(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_root = strlen(this->root);
      varToArr(outbuffer + offset, length_root);
      offset += 4;
      memcpy(outbuffer + offset, this->root, length_root);
      offset += length_root;
      uint32_t length_tip = strlen(this->tip);
      varToArr(outbuffer + offset, length_tip);
      offset += 4;
      memcpy(outbuffer + offset, this->tip, length_tip);
      offset += length_tip;
      *(outbuffer + offset + 0) = (this->joint_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_positions_length);
      for( uint32_t i = 0; i < joint_positions_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_positions[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_root;
      arrToVar(length_root, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_root; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_root-1]=0;
      this->root = (char *)(inbuffer + offset-1);
      offset += length_root;
      uint32_t length_tip;
      arrToVar(length_tip, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tip; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tip-1]=0;
      this->tip = (char *)(inbuffer + offset-1);
      offset += length_tip;
      uint32_t joint_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_positions_length);
      if(joint_positions_lengthT > joint_positions_length)
        this->joint_positions = (float*)realloc(this->joint_positions, joint_positions_lengthT * sizeof(float));
      joint_positions_length = joint_positions_lengthT;
      for( uint32_t i = 0; i < joint_positions_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_positions));
        memcpy( &(this->joint_positions[i]), &(this->st_joint_positions), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return FKTEST; };
    const char * getMD5(){ return "708e14f98ff72822d3442bcaef9c218d"; };

  };

  class FkTestResponse : public ros::Msg
  {
    public:
      uint32_t pos_length;
      typedef float _pos_type;
      _pos_type st_pos;
      _pos_type * pos;
      uint32_t rot_length;
      typedef float _rot_type;
      _rot_type st_rot;
      _rot_type * rot;

    FkTestResponse():
      pos_length(0), pos(NULL),
      rot_length(0), rot(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_length);
      for( uint32_t i = 0; i < pos_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pos[i]);
      }
      *(outbuffer + offset + 0) = (this->rot_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rot_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rot_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rot_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rot_length);
      for( uint32_t i = 0; i < rot_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rot[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t pos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pos_length);
      if(pos_lengthT > pos_length)
        this->pos = (float*)realloc(this->pos, pos_lengthT * sizeof(float));
      pos_length = pos_lengthT;
      for( uint32_t i = 0; i < pos_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_pos));
        memcpy( &(this->pos[i]), &(this->st_pos), sizeof(float));
      }
      uint32_t rot_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rot_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rot_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rot_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rot_length);
      if(rot_lengthT > rot_length)
        this->rot = (float*)realloc(this->rot, rot_lengthT * sizeof(float));
      rot_length = rot_lengthT;
      for( uint32_t i = 0; i < rot_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_rot));
        memcpy( &(this->rot[i]), &(this->st_rot), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return FKTEST; };
    const char * getMD5(){ return "e2248c2f30c5f3e010ed2e9434015c6e"; };

  };

  class FkTest {
    public:
    typedef FkTestRequest Request;
    typedef FkTestResponse Response;
  };

}
#endif
