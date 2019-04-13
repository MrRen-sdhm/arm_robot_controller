#include "action_server.hpp"

namespace hustac {

//static uint32_t action_test_time_start = 0;
//    
//static void on_goal(ActionServer<actionlib::TestAction>& action, const actionlib::TestGoal& goal) {
//    printf("Goal recved: %d\n", goal.goal);
//    action_test_time_start = HAL_GetTick();
//    action.setAccepted();
//}

//static void on_cancel(ActionServer<actionlib::TestAction>& action) {
//    printf("Cancel recved\n");
//    action_test_time_start = 0;
//    action.setCanceled();
//}

//void action_test_update() {
//    if (action_test_time_start && HAL_GetTick() - action_test_time_start > 2000) {
//        printf("Goal aborted\n");
//        action_test_time_start = 0;
//        action_server_test.setAborted();
//    }
//}
    
//ActionServer<actionlib::TestAction> action_server_test("action_test", on_goal, on_cancel);
    
}