#ifndef _ROS_interval_intersection_ConfigAction_h
#define _ROS_interval_intersection_ConfigAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "interval_intersection/ConfigActionGoal.h"
#include "interval_intersection/ConfigActionResult.h"
#include "interval_intersection/ConfigActionFeedback.h"

namespace interval_intersection
{

  class ConfigAction : public ros::Msg
  {
    public:
      typedef interval_intersection::ConfigActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef interval_intersection::ConfigActionResult _action_result_type;
      _action_result_type action_result;
      typedef interval_intersection::ConfigActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    ConfigAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "interval_intersection/ConfigAction"; };
    const char * getMD5(){ return "daf4f3168538603bba58f6ea42d51314"; };

  };

}
#endif