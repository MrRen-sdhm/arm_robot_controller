#ifndef _ROS_pr2_tilt_laser_interface_GetSnapshotAction_h
#define _ROS_pr2_tilt_laser_interface_GetSnapshotAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_tilt_laser_interface/GetSnapshotActionGoal.h"
#include "pr2_tilt_laser_interface/GetSnapshotActionResult.h"
#include "pr2_tilt_laser_interface/GetSnapshotActionFeedback.h"

namespace pr2_tilt_laser_interface
{

  class GetSnapshotAction : public ros::Msg
  {
    public:
      typedef pr2_tilt_laser_interface::GetSnapshotActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef pr2_tilt_laser_interface::GetSnapshotActionResult _action_result_type;
      _action_result_type action_result;
      typedef pr2_tilt_laser_interface::GetSnapshotActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GetSnapshotAction():
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

    const char * getType(){ return "pr2_tilt_laser_interface/GetSnapshotAction"; };
    const char * getMD5(){ return "a02433255d6053a289c4e30e1bb6194d"; };

  };

}
#endif