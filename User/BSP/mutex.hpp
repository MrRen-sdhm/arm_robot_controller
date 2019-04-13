//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_MUTEX_HPP
#define ARM_ROBOT_F427_V3_0_MUTEX_HPP

#include <mutex>

#include "cmsis_os.h"

namespace hustac {

// BasicLockable
class InterruptLock {
public:
    void lock() {
        pri_ = taskENTER_CRITICAL_FROM_ISR();
    }
    void unlock() {
        taskEXIT_CRITICAL_FROM_ISR(pri_);
    }
protected:
    decltype(taskENTER_CRITICAL_FROM_ISR()) pri_ = 0;
};

using InterruptLockGuard = std::lock_guard<InterruptLock>;

// BasicLockable
class KernelLock {
public:
    void lock() {
        vTaskSuspendAll();
    }
    void unlock() {
        xTaskResumeAll();
    }
};

using KernelLockGuard = std::lock_guard<KernelLock>;

// BasicLockable
class Mutex {
public:
    Mutex()
        : mutex_id_(osMutexCreate(osMutex(mutex_))) {
        if (mutex_id_ == NULL) {
            throw std::runtime_error("osMutexCreate() failed");
        }
    }
    Mutex &operator=(const Mutex &) = delete;
    void lock() {
        if (osMutexWait(mutex_id_, osWaitForever) != osOK) {
            throw std::runtime_error("osMutexWait() failed");
        }
    }
    void unlock() {
        if (osMutexRelease(mutex_id_) != osOK) {
            throw std::runtime_error("osMutexRelease() failed");
        }
    }
    ~Mutex() {
        if (mutex_id_ != NULL) {
            osMutexRelease(mutex_id_);
            osMutexDelete(mutex_id_);
        }
    }
protected:
    osMutexDef(mutex_);
    osMutexId mutex_id_;
};

using MutexLockGuard = std::lock_guard<Mutex>;

class RecursiveMutex {
public:
    RecursiveMutex();
    void lock();
    void unlock();
    ~RecursiveMutex();
};

}

#endif //ARM_ROBOT_F427_V3_0_MUTEX_HPP
