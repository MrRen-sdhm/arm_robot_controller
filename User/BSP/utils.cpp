#include "utils.hpp"

namespace hustac {

// 以下都是100ms的定时器
static SoftTimerMS<100, true> timer_ms;
static SoftTimerNS<100000000, true> timer_ns;
static SoftTimerFloat<100000000, true> timer_float;
static SoftTimerDouble<100000000, true> timer_double;
static SoftTimerMS<100, false> timer_ms2;
static SoftTimerNS<100000000, false> timer_ns2;
static SoftTimerFloat<100000000, false> timer_float2;
static SoftTimerDouble<100000000, false> timer_double2;
static SoftTimerMS<0, true> timer_ms3;
static SoftTimerNS<0, true> timer_ns3;
static SoftTimerFloat<0, true> timer_float3;
static SoftTimerDouble<0, true> timer_double3;
static SoftTimerMS<0, false> timer_ms4;
static SoftTimerNS<0, false> timer_ns4;
static SoftTimerFloat<0, false> timer_float4;
static SoftTimerDouble<0, false> timer_double4;
    
static void f() {
//    timer_ms.set_interval(1); // 错误, 模板参数interval!=0, 不可以自定义interval
    timer_ms3.set_interval(1);
}
    
static QueueUnsafe<int, 1> queue1;
static QueueSafe<int, 1> queue2;
    
}