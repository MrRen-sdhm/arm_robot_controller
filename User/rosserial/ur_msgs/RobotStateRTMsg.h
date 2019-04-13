#ifndef _ROS_ur_msgs_RobotStateRTMsg_h
#define _ROS_ur_msgs_RobotStateRTMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

  class RobotStateRTMsg : public ros::Msg
  {
    public:
      typedef float _time_type;
      _time_type time;
      uint32_t q_target_length;
      typedef float _q_target_type;
      _q_target_type st_q_target;
      _q_target_type * q_target;
      uint32_t qd_target_length;
      typedef float _qd_target_type;
      _qd_target_type st_qd_target;
      _qd_target_type * qd_target;
      uint32_t qdd_target_length;
      typedef float _qdd_target_type;
      _qdd_target_type st_qdd_target;
      _qdd_target_type * qdd_target;
      uint32_t i_target_length;
      typedef float _i_target_type;
      _i_target_type st_i_target;
      _i_target_type * i_target;
      uint32_t m_target_length;
      typedef float _m_target_type;
      _m_target_type st_m_target;
      _m_target_type * m_target;
      uint32_t q_actual_length;
      typedef float _q_actual_type;
      _q_actual_type st_q_actual;
      _q_actual_type * q_actual;
      uint32_t qd_actual_length;
      typedef float _qd_actual_type;
      _qd_actual_type st_qd_actual;
      _qd_actual_type * qd_actual;
      uint32_t i_actual_length;
      typedef float _i_actual_type;
      _i_actual_type st_i_actual;
      _i_actual_type * i_actual;
      uint32_t tool_acc_values_length;
      typedef float _tool_acc_values_type;
      _tool_acc_values_type st_tool_acc_values;
      _tool_acc_values_type * tool_acc_values;
      uint32_t tcp_force_length;
      typedef float _tcp_force_type;
      _tcp_force_type st_tcp_force;
      _tcp_force_type * tcp_force;
      uint32_t tool_vector_length;
      typedef float _tool_vector_type;
      _tool_vector_type st_tool_vector;
      _tool_vector_type * tool_vector;
      uint32_t tcp_speed_length;
      typedef float _tcp_speed_type;
      _tcp_speed_type st_tcp_speed;
      _tcp_speed_type * tcp_speed;
      typedef float _digital_input_bits_type;
      _digital_input_bits_type digital_input_bits;
      uint32_t motor_temperatures_length;
      typedef float _motor_temperatures_type;
      _motor_temperatures_type st_motor_temperatures;
      _motor_temperatures_type * motor_temperatures;
      typedef float _controller_timer_type;
      _controller_timer_type controller_timer;
      typedef float _test_value_type;
      _test_value_type test_value;
      typedef float _robot_mode_type;
      _robot_mode_type robot_mode;
      uint32_t joint_modes_length;
      typedef float _joint_modes_type;
      _joint_modes_type st_joint_modes;
      _joint_modes_type * joint_modes;

    RobotStateRTMsg():
      time(0),
      q_target_length(0), q_target(NULL),
      qd_target_length(0), qd_target(NULL),
      qdd_target_length(0), qdd_target(NULL),
      i_target_length(0), i_target(NULL),
      m_target_length(0), m_target(NULL),
      q_actual_length(0), q_actual(NULL),
      qd_actual_length(0), qd_actual(NULL),
      i_actual_length(0), i_actual(NULL),
      tool_acc_values_length(0), tool_acc_values(NULL),
      tcp_force_length(0), tcp_force(NULL),
      tool_vector_length(0), tool_vector(NULL),
      tcp_speed_length(0), tcp_speed(NULL),
      digital_input_bits(0),
      motor_temperatures_length(0), motor_temperatures(NULL),
      controller_timer(0),
      test_value(0),
      robot_mode(0),
      joint_modes_length(0), joint_modes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      *(outbuffer + offset + 0) = (this->q_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->q_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->q_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->q_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q_target_length);
      for( uint32_t i = 0; i < q_target_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->q_target[i]);
      }
      *(outbuffer + offset + 0) = (this->qd_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->qd_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->qd_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->qd_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qd_target_length);
      for( uint32_t i = 0; i < qd_target_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->qd_target[i]);
      }
      *(outbuffer + offset + 0) = (this->qdd_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->qdd_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->qdd_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->qdd_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qdd_target_length);
      for( uint32_t i = 0; i < qdd_target_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->qdd_target[i]);
      }
      *(outbuffer + offset + 0) = (this->i_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_target_length);
      for( uint32_t i = 0; i < i_target_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->i_target[i]);
      }
      *(outbuffer + offset + 0) = (this->m_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->m_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->m_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->m_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m_target_length);
      for( uint32_t i = 0; i < m_target_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->m_target[i]);
      }
      *(outbuffer + offset + 0) = (this->q_actual_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->q_actual_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->q_actual_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->q_actual_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q_actual_length);
      for( uint32_t i = 0; i < q_actual_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->q_actual[i]);
      }
      *(outbuffer + offset + 0) = (this->qd_actual_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->qd_actual_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->qd_actual_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->qd_actual_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qd_actual_length);
      for( uint32_t i = 0; i < qd_actual_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->qd_actual[i]);
      }
      *(outbuffer + offset + 0) = (this->i_actual_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_actual_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_actual_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_actual_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_actual_length);
      for( uint32_t i = 0; i < i_actual_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->i_actual[i]);
      }
      *(outbuffer + offset + 0) = (this->tool_acc_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tool_acc_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tool_acc_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tool_acc_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_acc_values_length);
      for( uint32_t i = 0; i < tool_acc_values_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tool_acc_values[i]);
      }
      *(outbuffer + offset + 0) = (this->tcp_force_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tcp_force_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tcp_force_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tcp_force_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tcp_force_length);
      for( uint32_t i = 0; i < tcp_force_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tcp_force[i]);
      }
      *(outbuffer + offset + 0) = (this->tool_vector_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tool_vector_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tool_vector_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tool_vector_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_vector_length);
      for( uint32_t i = 0; i < tool_vector_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tool_vector[i]);
      }
      *(outbuffer + offset + 0) = (this->tcp_speed_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tcp_speed_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tcp_speed_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tcp_speed_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tcp_speed_length);
      for( uint32_t i = 0; i < tcp_speed_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tcp_speed[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->digital_input_bits);
      *(outbuffer + offset + 0) = (this->motor_temperatures_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_temperatures_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_temperatures_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_temperatures_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_temperatures_length);
      for( uint32_t i = 0; i < motor_temperatures_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->motor_temperatures[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->controller_timer);
      offset += serializeAvrFloat64(outbuffer + offset, this->test_value);
      offset += serializeAvrFloat64(outbuffer + offset, this->robot_mode);
      *(outbuffer + offset + 0) = (this->joint_modes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_modes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_modes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_modes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_modes_length);
      for( uint32_t i = 0; i < joint_modes_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_modes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      uint32_t q_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      q_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      q_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      q_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->q_target_length);
      if(q_target_lengthT > q_target_length)
        this->q_target = (float*)realloc(this->q_target, q_target_lengthT * sizeof(float));
      q_target_length = q_target_lengthT;
      for( uint32_t i = 0; i < q_target_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_q_target));
        memcpy( &(this->q_target[i]), &(this->st_q_target), sizeof(float));
      }
      uint32_t qd_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      qd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      qd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      qd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->qd_target_length);
      if(qd_target_lengthT > qd_target_length)
        this->qd_target = (float*)realloc(this->qd_target, qd_target_lengthT * sizeof(float));
      qd_target_length = qd_target_lengthT;
      for( uint32_t i = 0; i < qd_target_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_qd_target));
        memcpy( &(this->qd_target[i]), &(this->st_qd_target), sizeof(float));
      }
      uint32_t qdd_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      qdd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      qdd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      qdd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->qdd_target_length);
      if(qdd_target_lengthT > qdd_target_length)
        this->qdd_target = (float*)realloc(this->qdd_target, qdd_target_lengthT * sizeof(float));
      qdd_target_length = qdd_target_lengthT;
      for( uint32_t i = 0; i < qdd_target_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_qdd_target));
        memcpy( &(this->qdd_target[i]), &(this->st_qdd_target), sizeof(float));
      }
      uint32_t i_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_target_length);
      if(i_target_lengthT > i_target_length)
        this->i_target = (float*)realloc(this->i_target, i_target_lengthT * sizeof(float));
      i_target_length = i_target_lengthT;
      for( uint32_t i = 0; i < i_target_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_i_target));
        memcpy( &(this->i_target[i]), &(this->st_i_target), sizeof(float));
      }
      uint32_t m_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      m_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      m_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      m_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->m_target_length);
      if(m_target_lengthT > m_target_length)
        this->m_target = (float*)realloc(this->m_target, m_target_lengthT * sizeof(float));
      m_target_length = m_target_lengthT;
      for( uint32_t i = 0; i < m_target_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_m_target));
        memcpy( &(this->m_target[i]), &(this->st_m_target), sizeof(float));
      }
      uint32_t q_actual_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      q_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      q_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      q_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->q_actual_length);
      if(q_actual_lengthT > q_actual_length)
        this->q_actual = (float*)realloc(this->q_actual, q_actual_lengthT * sizeof(float));
      q_actual_length = q_actual_lengthT;
      for( uint32_t i = 0; i < q_actual_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_q_actual));
        memcpy( &(this->q_actual[i]), &(this->st_q_actual), sizeof(float));
      }
      uint32_t qd_actual_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      qd_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      qd_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      qd_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->qd_actual_length);
      if(qd_actual_lengthT > qd_actual_length)
        this->qd_actual = (float*)realloc(this->qd_actual, qd_actual_lengthT * sizeof(float));
      qd_actual_length = qd_actual_lengthT;
      for( uint32_t i = 0; i < qd_actual_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_qd_actual));
        memcpy( &(this->qd_actual[i]), &(this->st_qd_actual), sizeof(float));
      }
      uint32_t i_actual_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_actual_length);
      if(i_actual_lengthT > i_actual_length)
        this->i_actual = (float*)realloc(this->i_actual, i_actual_lengthT * sizeof(float));
      i_actual_length = i_actual_lengthT;
      for( uint32_t i = 0; i < i_actual_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_i_actual));
        memcpy( &(this->i_actual[i]), &(this->st_i_actual), sizeof(float));
      }
      uint32_t tool_acc_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tool_acc_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tool_acc_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tool_acc_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tool_acc_values_length);
      if(tool_acc_values_lengthT > tool_acc_values_length)
        this->tool_acc_values = (float*)realloc(this->tool_acc_values, tool_acc_values_lengthT * sizeof(float));
      tool_acc_values_length = tool_acc_values_lengthT;
      for( uint32_t i = 0; i < tool_acc_values_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tool_acc_values));
        memcpy( &(this->tool_acc_values[i]), &(this->st_tool_acc_values), sizeof(float));
      }
      uint32_t tcp_force_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tcp_force_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tcp_force_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tcp_force_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tcp_force_length);
      if(tcp_force_lengthT > tcp_force_length)
        this->tcp_force = (float*)realloc(this->tcp_force, tcp_force_lengthT * sizeof(float));
      tcp_force_length = tcp_force_lengthT;
      for( uint32_t i = 0; i < tcp_force_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tcp_force));
        memcpy( &(this->tcp_force[i]), &(this->st_tcp_force), sizeof(float));
      }
      uint32_t tool_vector_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tool_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tool_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tool_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tool_vector_length);
      if(tool_vector_lengthT > tool_vector_length)
        this->tool_vector = (float*)realloc(this->tool_vector, tool_vector_lengthT * sizeof(float));
      tool_vector_length = tool_vector_lengthT;
      for( uint32_t i = 0; i < tool_vector_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tool_vector));
        memcpy( &(this->tool_vector[i]), &(this->st_tool_vector), sizeof(float));
      }
      uint32_t tcp_speed_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tcp_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tcp_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tcp_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tcp_speed_length);
      if(tcp_speed_lengthT > tcp_speed_length)
        this->tcp_speed = (float*)realloc(this->tcp_speed, tcp_speed_lengthT * sizeof(float));
      tcp_speed_length = tcp_speed_lengthT;
      for( uint32_t i = 0; i < tcp_speed_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tcp_speed));
        memcpy( &(this->tcp_speed[i]), &(this->st_tcp_speed), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->digital_input_bits));
      uint32_t motor_temperatures_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_temperatures_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_temperatures_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_temperatures_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_temperatures_length);
      if(motor_temperatures_lengthT > motor_temperatures_length)
        this->motor_temperatures = (float*)realloc(this->motor_temperatures, motor_temperatures_lengthT * sizeof(float));
      motor_temperatures_length = motor_temperatures_lengthT;
      for( uint32_t i = 0; i < motor_temperatures_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_motor_temperatures));
        memcpy( &(this->motor_temperatures[i]), &(this->st_motor_temperatures), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->controller_timer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->test_value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->robot_mode));
      uint32_t joint_modes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_modes_length);
      if(joint_modes_lengthT > joint_modes_length)
        this->joint_modes = (float*)realloc(this->joint_modes, joint_modes_lengthT * sizeof(float));
      joint_modes_length = joint_modes_lengthT;
      for( uint32_t i = 0; i < joint_modes_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_joint_modes));
        memcpy( &(this->joint_modes[i]), &(this->st_joint_modes), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "ur_msgs/RobotStateRTMsg"; };
    const char * getMD5(){ return "ce6feddd3ccb4ca7dbcd0ff105b603c7"; };

  };

}
#endif