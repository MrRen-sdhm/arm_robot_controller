#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include <control_msgs/FollowJointTrajectoryGoal.h>

#include "crc.h"
#include "stm32f4xx_hal_crc.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#include "dmabuffer_uart.hpp"

#define R2D(rad) ((rad) / M_PI * 180)
#define D2R(deg) ((deg)*M_PI / 180)

namespace hustac {

template <size_t joint_count>
struct JointTolerance {
    std::array<float, joint_count> positions;
    //        std::array<float, joint_count> velocities;
    //        std::array<float, joint_count> accelerations;
};

// 一个轨迹点
template <size_t joint_count>
struct TrajectoryPoint {
    std::array<float, joint_count> positions;
    //        std::array<float, joint_count> velocities;
    //        std::array<float, joint_count> accelerations;
    //        std::array<float, joint_count> effort;
    uint64_t time_nsec; // 相对于轨迹跟踪开始的时间偏移
};

// 轨迹
template <size_t joint_count, size_t max_trajectory_points>
struct Trajectory {
    static const int joint_count_ = joint_count;
    /* 路径容差值, 小于0表示无限制 */
    JointTolerance<joint_count> path_tol;
    /* 最终位置容差值, 小于0表示无限制 */
    JointTolerance<joint_count> goal_tol;
    uint64_t goal_time_tolerance; // 结束时间容差值
    // 待执行的轨迹
    std::array<TrajectoryPoint<joint_count>, max_trajectory_points> points;
    // 待执行的轨迹长度
    size_t points_length = 0;

    uint32_t dummy = 0;

    void print() {
        if (points_length <= 0) {
            printf("[TRAJ Empty]\n");
            return;
        }
        printf("[TRAJ length=%d (%.3f s)]\n", points_length, (points[points_length - 1].time_nsec - points[0].time_nsec) / 1e9f);
        
        printf("[TRAJ]: path_tol=[");
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f", R2D(path_tol.positions[i]));
        }
        printf("]\n");
        
        printf("[TRAJ]: goal_tol=[");
        for (int i = 0; i < joint_count_; i++) {
            if (i > 0) {
                printf(",");
            }
            printf("%.2f", R2D(goal_tol.positions[i]));
        }
        printf("]\n");
        
        printf("[TRAJ]: goal_time_tolerance=%d ms\n", (int)(goal_time_tolerance / 1000000));
        do {
            auto& p = points[0];
            printf("[TRAJ]: begin (%d ms)=[", int(p.time_nsec / 1000000));
            for (int i = 0; i < joint_count_; i++) {
                if (i > 0) {
                    printf(",");
                }
                printf("%.2f", R2D(p.positions[i]));
            }
            printf("]\n");
        } while (0);
        
        do {
            auto& p = points[points_length - 1];
            printf("[TRAJ]: end (%d ms)=[", int(p.time_nsec / 1000000));
            for (int i = 0; i < joint_count_; i++) {
                if (i > 0) {
                    printf(",");
                }
                printf("%.2f", R2D(p.positions[i]));
            }
            printf("]\n");
        } while (0);
    }

    // 数据结构
    // uint32_t size
    // uint32_t size_crc
    // uint32_t[sizeof(*this)/4] content
    // uint32_t content_crc

    // 保存到flash
    // 可以保存在8个位置 (page), 每个page为128kB
    // page0的地址为0x08100000
    // page=0 -> Sector[12,16]共128kB
    // page=1 -> Sector17共128kB
    // page=2 -> Sector18共128kB
    // page=3 -> Sector19共128kB
    // page=4 -> Sector20共128kB
    // page=5 -> Sector21共128kB
    // page=6 -> Sector22共128kB
    // page=7 -> Sector23共128kB
    int save(int page) {
        if (page < 0 || page >= 8) {
            return -1;
        }
        uint32_t err  = 0;
        uint32_t addr = 0x08100000 + page * 0x20000;
        uint32_t* p   = (uint32_t*)this;
        // calc crc
        uint32_t size        = sizeof(*this);
        uint32_t size_crc    = HAL_CRC_Calculate(&hcrc, &size, 1);
        uint32_t content_crc = HAL_CRC_Calculate(&hcrc, p, size / 4);
        int ret              = 0;
        // unlock
        if (HAL_FLASH_Unlock() != HAL_OK) {
            return -2;
        }
        // erase
        FLASH_EraseInitTypeDef erase_init;
        erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase_init.Banks        = 1;
        erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        if (page == 0) {
            erase_init.Sector    = 12;
            erase_init.NbSectors = 5;
        } else {
            erase_init.Sector    = 16 + page;
            erase_init.NbSectors = 1;
        }
        uint32_t dummy;
        if (HAL_FLASHEx_Erase(&erase_init, &dummy) != HAL_OK) {
            ret = -3;
            goto EXIT_CLEAR;
        }
        // program
        // program size
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, size) != HAL_OK) {
            ret = -4;
            goto EXIT_CLEAR;
        }
        addr += 4;
        // program size_crc
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, size_crc) != HAL_OK) {
            ret = -5;
            goto EXIT_CLEAR;
        }
        addr += 4;
        // program content
        size /= 4;
        while (size--) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *p) != HAL_OK) {
                ret = -6;
                goto EXIT_CLEAR;
            }
            addr += 4;
            p++;
        }
        // program content_crc
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, content_crc) != HAL_OK) {
            ret = -7;
            goto EXIT_CLEAR;
        }
        addr += 4;
    EXIT_CLEAR:
        // lock
        if (HAL_FLASH_Lock() != HAL_OK) {
            ret = -8;
        }
        return ret;
    }

    // 从flash读取
    int load(int page) {
        if (page < 0 || page >= 8) {
            return -1;
        }
        uint32_t addr     = 0x08100000 + page * 0x20000;
        uint32_t* p_flash = (uint32_t*)addr;
        // check size
        uint32_t size     = p_flash[0];
        uint32_t size_crc = p_flash[1];
        if (HAL_CRC_Calculate(&hcrc, &size, 1) != size_crc) {
            return -2;
        }
        if (size != sizeof(*this)) {
            return -3;
        }
        // check content
        uint32_t content_crc = p_flash[2 + size / 4];
        if (HAL_CRC_Calculate(&hcrc, p_flash + 2, size / 4) != content_crc) {
            return -4;
        }
        // copy
        p_flash         = p_flash + 2;
        uint32_t* p_ram = (uint32_t*)this;
        size /= 4;
        while (size--) {
            *p_ram = *p_flash;
            p_flash++;
            p_ram++;
        }
        dummy = 0;

        return 0;
    }
};

template <size_t joint_count, size_t max_trajectory_points>
class TrajectoryParser {
public:
    const std::array<char*, joint_count>& joint_names;

    const control_msgs::FollowJointTrajectoryGoal& msg;

    Trajectory<joint_count, max_trajectory_points>& traj;

    std::array<int, joint_count> id_to_traj_index;

    TrajectoryParser(decltype(joint_names) _joint_names, decltype(msg) _msg, decltype(traj) _trajecoty)
        : joint_names(_joint_names)
        , msg(_msg)
        , traj(_trajecoty) {
    }

    // 从关节名称列表中获取每个关节ID对应的索引号
    // 成功返回0, 出错返回-1
    int _parse_id_to_index() {
        const char* const* names = msg.trajectory.joint_names;
        size_t length            = msg.trajectory.joint_names_length;

        if (length < joint_count) {
            // 消息中的关节数量不足
            printf("not enough joint names\n");
            return -1;
        }

        std::fill_n(id_to_traj_index.begin(), joint_count, -1);
        for (int i = 0; i < length; i++) {
            for (int id = 1; id <= joint_count; id++) {
                if (strcmp(names[i], joint_names[id - 1]) == 0) {
                    id_to_traj_index[id - 1] = i;
                }
            }
        }
        for (int id = 1; id <= joint_count; id++) {
            if (id_to_traj_index[id - 1] < 0) {
                // 消息中的关节名称错误
                printf("bad joint names\n");
                return -2;
            }
        }
        return 0;
    }

    // 检查轨迹指令的结构
    // 若指令结构异常, 返回-N
    // 否则返回可以导入轨迹长度
    int check_trajectory_struct() {
        int ret = _parse_id_to_index();
        if (ret != 0) {
            // 消息中的关节名称错误
            return ret;
        }
        // 检查数据
        for (int i = 0; i < msg.trajectory.points_length; i++) {
            const trajectory_msgs::JointTrajectoryPoint& p = msg.trajectory.points[i];
            if (p.positions_length != p.positions_length) {
                // 位置字段不符
                printf("joint_names_length != positions_length\n");
                return -3;
            }
        }
        // 指令正常
        return msg.trajectory.points_length;
    }

    // 检查轨迹数据
    // 若关节限制为{0, 0}, 则该关节视为环形关节
    // 若轨迹数据点包含NaN或Inf等非正常数据, 则返回-1
    // 若轨迹数据点超出关节范围 joint_limits, 返回-2
    // 若相邻的两个轨迹之间的距离大于阈值 max_delta, 返回-3
    // 若时间非递增, 返回-4
    // 若第一个轨迹点的时间戳非0, 返回-5
    int check_trajectory_value(const std::array<std::pair<float, float>, joint_count>& joint_limits, float max_delta) {
        // 检查数据
        for (int i = 0; i < msg.trajectory.points_length; i++) {
            const trajectory_msgs::JointTrajectoryPoint& p = msg.trajectory.points[i];

            if (i == 0 && !p.time_from_start.isZero()) {
                printf("first_point.time_from_start != 0\n");
                return -5;
            }

            for (int id = 1; id <= joint_count; id++) {

                bool is_circular = false;
                if (joint_limits[id - 1] == std::pair<float, float>(0, 0)) {
                    is_circular = true;
                }

                float pos = p.positions[id_to_traj_index[id - 1]];
                if (!std::isfinite(pos)) {
                    // 位置指令中包含无效数据
                    printf("illegal pos: %f\n", pos);
                    return -1;
                }
                if (is_circular) {
                    // 取余 (IEEE余数: abs(余数)<=abs(除数/2) )
                    pos = std::remainder(pos, 2 * M_PI);
                } else {
                    if (pos > joint_limits[id - 1].second || pos < joint_limits[id - 1].first) {
                        // 轨迹超出范围
                        printf("pos out of range: %f\n", pos);
                        return -2;
                    }
                }

                if (i < msg.trajectory.points_length - 1) {
                    const trajectory_msgs::JointTrajectoryPoint& p_next = msg.trajectory.points[i + 1];
                    if (p_next.time_from_start <= p.time_from_start) {
                        printf("time_from_start not increase: p %ld >= p_next %ld ms\n", p.time_from_start.toNSec(), p_next.time_from_start.toNSec());
                        return -4;
                    }
                    float pos_next = p_next.positions[id_to_traj_index[id - 1]];
                    if (is_circular) {
                        pos_next = std::remainder(pos_next, 2 * M_PI);
                    }
                    float delta = pos_next - pos;
                    if (is_circular) {
                        delta = std::remainder(delta, 2 * M_PI);
                    }
                    if (std::abs(delta) > max_delta) {
                        printf("pos delta step too large: %f > %f\n", std::abs(delta), max_delta);
                        return -3;
                    }
                }
            }
        }
        return 0;
    }

    // 从JointTrajectory消息中拷贝轨迹数据, 并将环形关节的位置设定值限制为 -pi 到 pi
    // id_to_index由_check_trajectory计算得到
    void copy_trajectory(const std::array<std::pair<float, float>, joint_count>& joint_limits) {
        // 拷贝数据
        for (int i = 0; i < msg.trajectory.points_length; i++) {
            const trajectory_msgs::JointTrajectoryPoint& p = msg.trajectory.points[i];

            traj.points[i].time_nsec = (uint64_t)p.time_from_start.sec * 1000000000 + p.time_from_start.nsec;

            for (int id = 1; id <= joint_count; id++) {
                float pos = p.positions[id_to_traj_index[id - 1]];

                if (joint_limits[id - 1] == std::pair<float, float>(0, 0)) {
                    // 环形关节, 取余 (IEEE余数: abs(余数)<=abs(除数/2) )
                    pos = std::remainder(pos, 2 * M_PI);
                }
                traj.points[i].positions[id - 1] = pos;
            }
        }
        traj.points_length = msg.trajectory.points_length;
    }

    // 拷贝容差值
    void _copy_tolerance(const control_msgs::JointTolerance* tols, size_t length, const JointTolerance<joint_count>& default_tols, JointTolerance<joint_count>& tols_out) {
        // 建立关节ID到列表序号的索引
        std::array<int, joint_count> id_to_index;
        std::fill_n(id_to_index.begin(), joint_count, -1);
        for (int i = 0; i < length; i++) {
            for (int id = 1; id <= joint_count; id++) {
                if (strcmp(tols[i].name, joint_names[id - 1]) == 0) {
                    id_to_index[id - 1] = i;
                }
            }
        }
        // 检查并拷贝数据
        for (int i = 0; i < joint_count; i++) {
            if (id_to_index[i] < 0) {
                // 没有给定容差值, 使用默认值
                tols_out.positions[i] = default_tols.positions[i];
            } else {
                const control_msgs::JointTolerance& tol = tols[id_to_index[i]];
                if (tol.position < 0) {
                    // 给定值小于0, 容差值无限制
                    tols_out.positions[i] = -1;
                } else if (tol.position == 0 || !std::isfinite(tol.position)) {
                    // 给定值为0, 或给定值为异常数, 使用默认容差值
                    tols_out.positions[i] = default_tols.positions[i];
                } else {
                    // 使用给定容差值
                    tols_out.positions[i] = tol.position;
                }
            }
        }
    }

    void copy_tolerance(const JointTolerance<joint_count>& default_path_tol, const JointTolerance<joint_count>& default_goal_tol) {
        _copy_tolerance(msg.path_tolerance, msg.path_tolerance_length, default_path_tol, traj.path_tol);
        _copy_tolerance(msg.goal_tolerance, msg.goal_tolerance_length, default_goal_tol, traj.goal_tol);
        traj.goal_time_tolerance = msg.goal_time_tolerance.toNSec();
    }
};
}

#undef R2D
#undef D2R
