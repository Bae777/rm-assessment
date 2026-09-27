/**
 * @file    ControlTask.cpp
 * @brief   RM 电控考核：GM6020 正弦波位置/速度跟踪 + 看门狗保护
 *
 * 控制周期：1 kHz（TIM6 周期中断中调用 MainTask）
 * 通信：CAN 1 Mbps
 * 保护：IWDG 独立看门狗，在 1 kHz 控制中断中喂狗
 */

#include "ControlTask.h"
#include "Gm6020.h"
#include "Pid.h"

extern "C" {
#include "can.h"
#include "tim.h"
#include "iwdg.h"
}

#include <cmath>
#include <cstring>

/* ==================== 可调参数 ==================== */

#define MOTOR_ID            1U          /* 电机拨码 ID (1~7) */

/* 工作模式：每次上电默认位置模式 */
#define MODE_POSITION       0
#define MODE_SPEED          1
static volatile uint8_t g_mode = MODE_POSITION;

/* ---- 位置正弦参数 ---- */
static const float POS_AMP   = 1.57f;       /* 幅值 (rad)  ≈ 90° */
static const float POS_FREQ  = 0.5f;        /* 频率 (Hz) */

/* ---- 速度正弦参数 ---- */
static const float SPD_AMP   = 20.0f;       /* 幅值 (rad/s) */
static const float SPD_FREQ  = 0.5f;        /* 频率 (Hz) */

/* ---- PID 参数（需实测整定）---- */
static const float POS_KP = 20.0f, POS_KI = 0.0f, POS_KD = 0.20f;
static const float SPD_KP = 15.0f, SPD_KI = 2.0f, SPD_KD = 0.00f;

/* ---- 输出限幅：GM6020 电压给定范围 ±25000 ---- */
static const int16_t VOLT_LIMIT = 25000;

/* ---- 温度保护阈值 ---- */
static const float TEMP_LIMIT = 80.0f;

/* ==================== 内部对象 ==================== */

static Gm6020 g_motor(MOTOR_ID);

static Pid g_pidPos(POS_KP, POS_KI, POS_KD,
                    (float)VOLT_LIMIT, -(float)VOLT_LIMIT);
static Pid g_pidSpd(SPD_KP, SPD_KI, SPD_KD,
                    (float)VOLT_LIMIT, -(float)VOLT_LIMIT);

static uint8_t g_txBuf[8];
static float   g_phase = 0.0f;              /* 正弦相位累加器 */

/* 调试观测用 */
volatile float g_refValue  = 0.0f;          /* 当前目标值 */
volatile float g_fdbValue  = 0.0f;          /* 当前反馈值 */
volatile int16_t g_voltage = 0;             /* 当前输出电压 */

/* ==================== 内部函数 ==================== */

/**
 * @brief  正弦波发生器（相位累加）
 * @param  amp   幅值
 * @param  freq  频率 (Hz)
 * @param  Ts    控制周期 (s)
 * @retval 当前时刻的正弦值
 * @note   用相位累加而非 sin(2πft)，保证周期精确、无累积漂移
 */
static float SineRef(float amp, float freq, float Ts)
{
    const float TWO_PI = 6.28318530718f;

    g_phase += TWO_PI * freq * Ts;
    if (g_phase > TWO_PI) {
        g_phase -= TWO_PI;
    }

    return amp * sinf(g_phase);
}

/* ==================== 对外接口 ==================== */

/**
 * @brief  控制任务初始化
 * @note   在所有 MX_XXX_Init() 之后调用一次
 */
void ControlTaskInit(void)
{
    /* ---------- 1. 配置 CAN 过滤器：只接收本电机的反馈帧 ---------- */
    CAN_FilterTypeDef filter = {0};
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)((0x204u + MOTOR_ID) << 5);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (uint16_t)(0x7FFu << 5);
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    /* ---------- 2. 启动 CAN 与接收中断 ---------- */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* ---------- 3. 启动 1 kHz 控制定时器 ---------- */
    HAL_TIM_Base_Start_IT(&htim6);

    /* ---------- 4. IWDG 由 CubeMX 生成的 MX_IWDG_Init() 启动 ---------- */

    g_phase = 0.0f;
    memset(g_txBuf, 0, sizeof(g_txBuf));
}

/**
 * @brief  1 kHz 控制任务（在 TIM6 周期中断中调用）
 */
void MainTask(void)
{
    const float Ts = 0.001f;                /* 1 kHz */

    float ref = 0.0f;
    float fdb = 0.0f;
    float out = 0.0f;

    if (g_mode == MODE_POSITION) {
        /* -------- 位置正弦跟踪 -------- */
        ref = SineRef(POS_AMP, POS_FREQ, Ts);
        fdb = g_motor.angle();              /* 累计角度 (rad) */
        out = g_pidPos.calc(ref, fdb);
    } else {
        /* -------- 速度正弦跟踪 -------- */
        ref = SineRef(SPD_AMP, SPD_FREQ, Ts);
        fdb = g_motor.vel();                /* 转速 (rad/s) */
        out = g_pidSpd.calc(ref, fdb);
    }

    g_refValue = ref;
    g_fdbValue = fdb;

    /* 过温保护：超限则输出置零 */
    if (g_motor.temp() > TEMP_LIMIT) {
        out = 0.0f;
        g_pidPos.reset();
        g_pidSpd.reset();
    }

    /* -------- 输出电压 -------- */
    g_motor.setVoltage((int16_t)out);
    g_voltage = (int16_t)out;

    /* -------- 组装并发送 CAN 控制帧 -------- */
    memset(g_txBuf, 0, sizeof(g_txBuf));
    g_motor.encode(g_txBuf);

    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.StdId = g_motor.txId();        /* 0x1FF 或 0x2FF */
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;

    uint32_t txMailbox = 0;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, g_txBuf, &txMailbox);

    /* -------- 喂狗 ----------
     * 在控制中断中喂狗：只要 1 kHz 控制回路停止运行，
     * IWDG 将在约 800 ms 后复位系统，避免电机保持最后电压值失控。
     */
    HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @brief  CAN 接收回调（在 HAL_CAN_RxFifo0MsgPendingCallback 中调用）
 * @param  std_id  标准帧标识符
 * @param  data    8 字节数据
 */
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
    if (std_id == g_motor.rxId()) {         /* 0x204 + MOTOR_ID */
        g_motor.decode(data);
    }
}

/**
 * @brief  切换工作模式（可由按键调用）
 */
void ControlTaskSetMode(uint8_t mode)
{
    g_mode  = mode;
    g_phase = 0.0f;
    g_pidPos.reset();
    g_pidSpd.reset();
}