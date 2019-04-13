#pragma once

#include <cstring>
#include <array>
#include <list>
#include <numeric>
#include <utility>
#include <algorithm>

namespace hustac {

class BaseMotor {
public:
    static constexpr float invalid_output = std::numeric_limits<float>::signaling_NaN();
    BaseMotor() = default;
    explicit BaseMotor(const char *name)
        : name_(name) {}
    const char *name() { return name_; };
    bool operator==(const BaseMotor &rhs) { return strcmp(name_, rhs.name_) == 0; }
    
    virtual int init() { return -1; }
    virtual int start() { return -1; }
    virtual int stop() { return -1; }
    virtual bool started() { return false; }
    
    virtual std::pair<float, float> limit() {
        return std::make_pair(std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN());
    }
    
    virtual float goal_position() { return std::numeric_limits<float>::signaling_NaN(); }
    virtual int goal_position(float) { return -1; }
    
    virtual float current_position() { return std::numeric_limits<float>::signaling_NaN(); }
    virtual float current_velocity() { return std::numeric_limits<float>::signaling_NaN(); }
    virtual float current_effort() { return std::numeric_limits<float>::signaling_NaN(); }
    
    virtual int spin_once() { return -1; };

protected:
    const char *name_ = "";
};

class EmergencyStoppable {
public:
    static std::list<EmergencyStoppable *> all_emergency_stoppable_;
    static const auto all_emergency_stoppable() {
        return all_emergency_stoppable_;
    }
    static int all_emergency_stop(bool val) {
        int ret = 1;
        for (auto p : all_emergency_stoppable_) {
            int r = p->emergency_stop(val);
            if (r != 1) {
                ret = r;
            }
        }
        return ret;
    }
    static bool all_emergency_stop() {
        for (auto p : all_emergency_stoppable_)
            if (!p->emergency_stop())
                return false;
        return true;
    }
private:
    bool emergency_stop_ = false;
public:
    EmergencyStoppable() {
        all_emergency_stoppable_.push_back(this);
    }
    virtual ~EmergencyStoppable() {
        auto it = std::find(all_emergency_stoppable_.begin(), all_emergency_stoppable_.end(), this);
        if (it != all_emergency_stoppable_.end())
            all_emergency_stoppable_.erase(it);
    }
    bool emergency_stop() {
        return emergency_stop_;
    }
    int emergency_stop(bool val) {
        if (emergency_stop_ ^ val) {
            emergency_stop_ = val;
            on_emergency_stop_changed(val);
            return 0;
        } else {
            return 1;
        }
    }
protected:
    virtual void on_emergency_stop_changed(bool value) {};
};

}
