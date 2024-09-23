/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */
#include "stddef.h"
#include "cia402device.h"

#define CIA402_COMMAND(CMD)                     CIA402_CONTROLWORD_##CMD##_COMMAND
#define CIA402_MASK(CMD)                        CIA402_CONTROLWORD_##CMD##_MASK
#define IS_CIA402_COMMAND(controlword, CMD)     (controlword & CIA402_MASK(CMD)) == CIA402_COMMAND(CMD)


void cia402_initialize(cia402_axis_t * axis, uint16_t * statusword, uint16_t * ALstatus) {
    axis->ALstatus = ALstatus;                   // 将传入的 ALstatus 指针赋值给轴的 ALstatus 成员
    axis->statusword = statusword;               // 将传入的 statusword 指针赋值给轴的 statusword 成员
    axis->state = NOT_READY_TO_SWITCH_ON;        // 初始化轴的状态为“未准备好开启”
    axis->transition = NO_TRANSITION;             // 初始化状态机的转换为“无转换”
    axis->flags.config_allowed = 0;              // 设置允许配置标志为 0（不允许配置）
    axis->flags.axis_func_enabled = 0;           // 设置轴功能启用标志为 0（功能未启用）
    axis->flags.hv_power_applied = 0;           // 设置高压电源应用标志为 0（高压电源未应用）
    axis->flags.brake_applied = 0;               // 设置刹车应用标志为 0（刹车未应用）
    axis->prevflags = axis->flags;               // 保存当前标志状态到 prevflags，以便后续比较
}


/**
 * @brief 处理伺服驱动轴的状态机逻辑。
 *
 * 此函数根据给定的控制字和轴的当前状态，更新轴的状态、状态字和过渡状态。
 * 状态机遵循 CiA402 规范，支持多种状态和过渡。
 *
 * @param axis 指向需要更新的轴的指针，包含轴的当前状态、状态字和标志。
 * @param controlword 控制字，包含用于状态转换的命令。
 *
 * @return 无返回值。状态和状态字直接在轴结构中更新。
 *
 * 状态定义：
 * - NOT_READY_TO_SWITCH_ON: 轴未准备好开启。
 * - SWITCH_ON_DISABLED: 开关被禁用。
 * - READY_TO_SWITCH_ON: 轴准备好可以开启。
 * - SWITCHED_ON: 轴已开启。
 * - OPERATION_ENABLED: 轴已启用操作。
 * - QUICK_STOP_ACTIVE: 快速停止状态激活。
 * - FAULT_REACTION_ACTIVE: 故障反应状态激活。
 * - FAULT: 发生故障。
 *
 * 状态过渡定义：
 * - NO_TRANSITION: 无状态过渡。
 * - 各种状态到状态的转换，如 NOT_READY_TO_SWITCH_ON 到 SWITCH_ON_DISABLED。
 */
void cia402_state_machine(cia402_axis_t * axis, uint16_t controlword) {
    *(axis->statusword) = 0x0000;  // 清空状态字
    axis->transition = NO_TRANSITION;  // 初始化过渡状态
    axis->prevflags = axis->flags;  // 保存之前的标志

    switch (axis->state)
    {
    case NOT_READY_TO_SWITCH_ON:
        /**
         * @brief 处理未准备好开启状态。
         *
         * 在此状态下，轴未准备好进行任何操作。只有在 AL 状态为 OP 时，
         * 轴才会过渡到开关禁用状态。
         */
        if (*(axis->ALstatus) == AL_STATUS_OP) {
            // 过渡 1: 从未准备好到开关禁用
            axis->state = SWITCH_ON_DISABLED;
            *(axis->statusword) |= SWITCH_ON_DISABLED;
            axis->transition = NOT_READY_TO_SWITCH_ON_TO_SWITCH_ON_DISABLED;
        } else {
            *(axis->statusword) |= NOT_READY_TO_SWITCH_ON; // 保持当前状态
        }
        break;

    case SWITCH_ON_DISABLED:
        /**
         * @brief 处理 开关禁用状态。
         *
         * 在此状态下，轴的电源未施加，无法进行操作。
         * 只有在接收到 SHUTDOWN 命令或 AL 状态为 OP 时，才会过渡到准备好开启状态。
         */
        if (IS_CIA402_COMMAND(controlword, SHUTDOWN) || *(axis->ALstatus) == AL_STATUS_OP) {
            // 过渡 2: 从开关禁用到准备好开启
            axis->state = READY_TO_SWITCH_ON;
            *(axis->statusword) |= READY_TO_SWITCH_ON;
            axis->transition = SWITCH_ON_DISABLED_TO_READY_TO_SWITCH_ON;
        } else {
            *(axis->statusword) |= SWITCH_ON_DISABLED; // 保持当前状态
        }
        break;

    case READY_TO_SWITCH_ON:
        /**
         * @brief 处理准备好开启状态。
         *
         * 在此状态下，轴已准备好进行操作。可以通过控制字进行状态转换。
         * 如果接收到 SWITCH_ON 命令，则过渡到已开启状态。
         */
        if (IS_CIA402_COMMAND(controlword, DISABLE_VOLTAGE)) {
            // 过渡 7: 从准备好开启到开关禁用
            axis->state = SWITCH_ON_DISABLED;
            *(axis->statusword) |= SWITCH_ON_DISABLED;
            axis->transition = READY_TO_SWITCH_ON_TO_SWITCH_ON_DISABLED;
        } else if (IS_CIA402_COMMAND(controlword, SWITCH_ON)) {
            // 过渡 3: 从准备好开启到已开启
            axis->state = SWITCHED_ON;
            *(axis->statusword) |= SWITCHED_ON;
            axis->transition = READY_TO_SWITCH_ON_TO_SWITCHED_ON;
    
            if (IS_CIA402_COMMAND(controlword, SWITCH_ON_ENABLE)) {
                // 在一个命令中进行过渡 3 + 4: 从准备好开启到操作启用
                axis->state = OPERATION_ENABLED;
                *(axis->statusword) |= OPERATION_ENABLED;
                axis->transition = READY_TO_SWITCH_ON_TO_OPERATION_ENABLED;
            }
        } else {
            *(axis->statusword) |= READY_TO_SWITCH_ON; // 保持当前状态
        }
        break;

    case SWITCHED_ON:
        /**
         * @brief 处理已开启状态。
         *
         * 在此状态下，轴已成功开启。可以接收命令以启用操作或进行其他状态转换。
         */
        if (IS_CIA402_COMMAND(controlword, SHUTDOWN)) {
            // 过渡 6: 从已开启到准备好开启
            axis->state = READY_TO_SWITCH_ON;
            *(axis->statusword) |= READY_TO_SWITCH_ON;
            axis->transition = SWITCHED_ON_TO_READY_TO_SWITCH_ON;
        } else if (IS_CIA402_COMMAND(controlword, ENABLE_OPERATION)) {
            // 过渡 4: 从已开启到操作启用
            axis->state = OPERATION_ENABLED;
            *(axis->statusword) |= OPERATION_ENABLED;
            axis->transition = SWITCHED_ON_TO_OPERATION_ENABLED;
        } else if (IS_CIA402_COMMAND(controlword, DISABLE_VOLTAGE)) {
            // 过渡 10: 从已开启到开关禁用
            axis->state = SWITCH_ON_DISABLED;
            *(axis->statusword) |= SWITCH_ON_DISABLED;
            axis->transition = SWITCHED_ON_TO_SWITCH_ON_DISABLED;
        } else {
            *(axis->statusword) |= SWITCHED_ON; // 保持当前状态
        }
        break;

    case OPERATION_ENABLED:
        /**
         * @brief 处理操作启用状态。
         *
         * 在此状态下，轴可以执行操作。可以接收命令以禁用操作或进行其他状态转换。
         */
        if (IS_CIA402_COMMAND(controlword, DISABLE_OPERATION)) {
            // 过渡 5: 从操作启用到已开启
            axis->state = SWITCHED_ON;
            *(axis->statusword) |= SWITCHED_ON;
            axis->transition = OPERATION_ENABLED_TO_SWITCHED_ON;
        }
        else if (IS_CIA402_COMMAND(controlword, SHUTDOWN)) {
            // 过渡 8: 从操作启用到准备好开启
            axis->state = READY_TO_SWITCH_ON;
            *(axis->statusword) |= READY_TO_SWITCH_ON;
            axis->transition = OPERATION_ENABLED_TO_READY_TO_SWITCH_ON;
        }
        else if (IS_CIA402_COMMAND(controlword, DISABLE_VOLTAGE)
            || *(axis->ALstatus) != AL_STATUS_OP) { // 连接丢失
            // 过渡 9: 从操作启用到开关禁用
            axis->state = SWITCH_ON_DISABLED;
            *(axis->statusword) |= SWITCH_ON_DISABLED;
            axis->transition = OPERATION_ENABLED_TO_SWITCH_ON_DISABLED;
        }
        else if (IS_CIA402_COMMAND(controlword, QUICK_STOP)) {
            // 过渡 11: 从操作启用到快速停止激活
            axis->state = QUICK_STOP_ACTIVE;
            *(axis->statusword) |= QUICK_STOP_ACTIVE;
            axis->transition = OPERATION_ENABLED_TO_QUICK_STOP_ACTIVE;
        }
        else {
            *(axis->statusword) |= OPERATION_ENABLED; // 保持当前状态
        }
        break;

    case QUICK_STOP_ACTIVE:
        /**
         * @brief 处理快速停止激活状态。
         *
         * 在此状态下，轴正在进行快速停止。可以接收命令以禁用电压或启用操作。
         */
        if (IS_CIA402_COMMAND(controlword, DISABLE_VOLTAGE)) {
            // 过渡 12: 从快速停止激活到开关禁用
            axis->state = SWITCH_ON_DISABLED;
            *(axis->statusword) |= SWITCH_ON_DISABLED;
            axis->transition = QUICK_STOP_ACTIVE_TO_SWITCH_ON_DISABLED;
        }
        else if (IS_CIA402_COMMAND(controlword, ENABLE_OPERATION)) {
            // 过渡 16: 从快速停止激活到操作启用（不推荐）
            *(axis->statusword) |= QUICK_STOP_ACTIVE; // 保持当前状态
            break;
            axis->state = OPERATION_ENABLED;
            *(axis->statusword) |= OPERATION_ENABLED;
            axis->transition = QUICK_STOP_ACTIVE_TO_OPERATION_ENABLED;
        } else {
            *(axis->statusword) |= QUICK_STOP_ACTIVE; // 保持当前状态
        }
        break;

    case FAULT_REACTION_ACTIVE:
        /**
         * @brief 处理故障反应激活状态。
         *
         * 在此状态下，轴检测到故障并进入故障反应模式。将自动转到故障状态。
         */
        axis->state = FAULT;
        *(axis->statusword) |= FAULT;
        axis->transition = FAULT_REACTION_ACTIVE_TO_FAULT;
        break;

    case FAULT:
        /**
         * @brief 处理故障状态。
         *
         * 在此状态下，轴处于故障状态。可以接收命令以重置故障。
         */
        if (IS_CIA402_COMMAND(controlword, FAULT_RESET)) {
            // 过渡 15: 从故障到开关禁用
            axis->state = SWITCH_ON_DISABLED;
            *(axis->statusword) |= SWITCH_ON_DISABLED;
            axis->transition = FAULT_TO_SWITCH_ON_DISABLED;
        } else {
            *(axis->statusword) |= FAULT; // 保持当前状态
        }
        break;

    default:
        // 处理意外状态，重置为未准备好开启
        axis->state = NOT_READY_TO_SWITCH_ON;
        *(axis->statusword) |= NOT_READY_TO_SWITCH_ON;
        axis->transition = NO_TRANSITION;
        axis->flags.config_allowed    = 0; // 禁止配置
        axis->flags.axis_func_enabled = 0; // 禁止轴功能
        axis->flags.hv_power_applied  = 0; // 高压未施加
        axis->flags.brake_applied     = 0; // 刹车未施加
        break;
    }

    // 根据当前状态更新轴的标志
    switch (axis->state)
    {
    case SWITCH_ON_DISABLED:
    case READY_TO_SWITCH_ON:
        axis->flags.config_allowed    = 1; // 允许配置
        axis->flags.axis_func_enabled = 0; // 禁止轴功能
        axis->flags.hv_power_applied  = 0; // 高压未施加
        axis->flags.brake_applied     = 1; // 刹车施加
        break;    
    
    case SWITCHED_ON:
        axis->flags.config_allowed    = 1; // 允许配置
        axis->flags.axis_func_enabled = 0; // 禁止轴功能
        axis->flags.hv_power_applied  = 1; // 高压施加
        axis->flags.brake_applied     = 1; // 刹车施加
        break;
    
    case OPERATION_ENABLED:
    case QUICK_STOP_ACTIVE:
        axis->flags.config_allowed    = 0; // 禁止配置
        axis->flags.axis_func_enabled = 1; // 允许轴功能
        axis->flags.hv_power_applied  = 1; // 高压施加
        axis->flags.brake_applied     = 0; // 刹车未施加
        break;

    case FAULT:
        axis->flags.config_allowed    = 1; // 允许配置
        axis->flags.axis_func_enabled = 0; // 禁止轴功能
        axis->flags.hv_power_applied  = 0; // 高压未施加
        axis->flags.brake_applied     = 0; // 刹车未施加
        break;

    default:
        break;
    }
}


