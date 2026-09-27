#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/* 上电初始化：配置 CAN 过滤器、启动 CAN 与 1kHz 控制定时器
 * 在 main.c 中全部 MX_XXX_Init() 执行完后调用一次 */
void ControlTaskInit(void);

/* 1kHz 控制入口，在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用 */
void MainTask(void);

/* CAN 接收回调，在 HAL_CAN_RxFifo0MsgPendingCallback 中调用 */
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

/* 切换工作模式：0=位置正弦跟踪，1=速度正弦跟踪 */
void ControlTaskSetMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLTASK_H */