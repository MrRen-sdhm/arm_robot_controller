#ifndef _ROS_trajectory_msgs_JointTrajectoryPoint_h
#define _ROS_trajectory_msgs_JointTrajectoryPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <array>

#include "ros/msg.h"
#include "ros/duration.h"

//#include "dmabuffer_uart.hpp"

namespace trajectory_msgs
{

class JointTrajectoryPoint : public ros::Msg
{
public:
    // 最多包含7个关节
    static const int max_length = 7;
    
    uint32_t positions_length;

    typedef float _positions_type;
    std::array<_positions_type, max_length> positions;

    // 禁用速度 加速度 力信息
    static const uint32_t velocities_length = 0;

    static const uint32_t accelerations_length = 0;

    static const uint32_t effort_length = 0;
    
    typedef ros::Duration _time_from_start_type;
    _time_from_start_type time_from_start;

    JointTrajectoryPoint() : positions_length(0),
//                             velocities_length(0), velocities(NULL),
//                             accelerations_length(0), accelerations(NULL),
//                             effort_length(0), effort(NULL),
                             time_from_start()
    {
    }
    
    // 移动幅值
//    JointTrajectoryPoint& operator=(JointTrajectoryPoint&& rhs) {
//        if (positions) {
//            free(positions);
//            positions = NULL;
//        }
//        if (velocities) {
//            free(velocities);
//            velocities = NULL;
//        }
//        if (positions) {
//            free(positions);
//            positions = NULL;
//        }
//        
//        positions_length = rhs.positions_length;
//        rhs.positions_length = 0;
//        positions = rhs.positions;
//        rhs.positions = NULL;
//        
//        velocities_length = rhs.velocities_length;
//        rhs.velocities_length = 0;
//        velocities = rhs.velocities;
//        rhs.velocities = NULL;
//        
//        accelerations_length = rhs.accelerations_length;
//        rhs.accelerations_length = 0;
//        accelerations = rhs.accelerations;
//        rhs.accelerations = NULL;
//        
//        effort_length = rhs.effort_length;
//        rhs.effort_length = 0;
//        effort = rhs.effort;
//        rhs.effort = NULL;
//        
//        time_from_start = rhs.time_from_start;
//        
//        return *this;
//    }

    virtual int serialize(unsigned char *outbuffer) const
    {
        int offset = 0;
        *(outbuffer + offset + 0) = (this->positions_length >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (this->positions_length >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (this->positions_length >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (this->positions_length >> (8 * 3)) & 0xFF;
        offset += sizeof(this->positions_length);
        for (uint32_t i = 0; i < positions_length; i++)
        {
            offset += serializeAvrFloat64(outbuffer + offset, this->positions[i]);
        }
        *(outbuffer + offset + 0) = (velocities_length >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (velocities_length >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (velocities_length >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (velocities_length >> (8 * 3)) & 0xFF;
        offset += sizeof(velocities_length);
//        for (uint32_t i = 0; i < velocities_length; i++)
//        {
//            offset += serializeAvrFloat64(outbuffer + offset, velocities[i]);
//        }
        *(outbuffer + offset + 0) = (accelerations_length >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (accelerations_length >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (accelerations_length >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (accelerations_length >> (8 * 3)) & 0xFF;
        offset += sizeof(accelerations_length);
//        for (uint32_t i = 0; i < accelerations_length; i++)
//        {
//            offset += serializeAvrFloat64(outbuffer + offset, accelerations[i]);
//        }
        *(outbuffer + offset + 0) = (effort_length >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (effort_length >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (effort_length >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (effort_length >> (8 * 3)) & 0xFF;
        offset += sizeof(effort_length);
//        for (uint32_t i = 0; i < effort_length; i++)
//        {
//            offset += serializeAvrFloat64(outbuffer + offset, effort[i]);
//        }
        *(outbuffer + offset + 0) = (this->time_from_start.sec >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (this->time_from_start.sec >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (this->time_from_start.sec >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (this->time_from_start.sec >> (8 * 3)) & 0xFF;
        offset += sizeof(this->time_from_start.sec);
        *(outbuffer + offset + 0) = (this->time_from_start.nsec >> (8 * 0)) & 0xFF;
        *(outbuffer + offset + 1) = (this->time_from_start.nsec >> (8 * 1)) & 0xFF;
        *(outbuffer + offset + 2) = (this->time_from_start.nsec >> (8 * 2)) & 0xFF;
        *(outbuffer + offset + 3) = (this->time_from_start.nsec >> (8 * 3)) & 0xFF;
        offset += sizeof(this->time_from_start.nsec);
        return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
        int offset = 0;
        
        // 读取position长度
        uint32_t positions_lengthT = ((uint32_t)(*(inbuffer + offset)));
        positions_lengthT |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        positions_lengthT |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        positions_lengthT |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->positions_length);
        positions_length = positions_lengthT;

        // 读取position值
        for (uint32_t i = 0; i < positions_length; i++)
        {
            if (i < max_length) {
                // 位置在有效范围内, 读取数据
                offset += deserializeAvrFloat64(inbuffer + offset, &(this->positions[i]));
            } else {
                // 位置超出有效范围, 直接+8
                offset += 8;
            }
        }
        
        // 限制长度
        if (positions_length > max_length) {
            positions_length = max_length;
        }
        
        // 跳过速度字段
        uint32_t velocities_lengthT = ((uint32_t)(*(inbuffer + offset)));
        velocities_lengthT |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        velocities_lengthT |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        velocities_lengthT |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->velocities_length);
        offset += velocities_lengthT * 8;
        
        // 跳过加速度字段
        uint32_t accelerations_lengthT = ((uint32_t)(*(inbuffer + offset)));
        accelerations_lengthT |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        accelerations_lengthT |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        accelerations_lengthT |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->accelerations_length);
        offset += accelerations_lengthT * 8;
        
        // 跳过力字段
        uint32_t effort_lengthT = ((uint32_t)(*(inbuffer + offset)));
        effort_lengthT |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        effort_lengthT |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        effort_lengthT |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->effort_length);
        offset += effort_lengthT * 8;
        
        this->time_from_start.sec = ((uint32_t)(*(inbuffer + offset)));
        this->time_from_start.sec |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        this->time_from_start.sec |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        this->time_from_start.sec |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->time_from_start.sec);
        
        this->time_from_start.nsec = ((uint32_t)(*(inbuffer + offset)));
        this->time_from_start.nsec |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
        this->time_from_start.nsec |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
        this->time_from_start.nsec |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
        offset += sizeof(this->time_from_start.nsec);
        return offset;
    }

    virtual const char *getType() { return "trajectory_msgs/JointTrajectoryPoint"; };
    virtual const char *getMD5() { return "f3cd1e1c4d320c79d6985c904ae5dcd3"; };
};

} // namespace trajectory_msgs
#endif
