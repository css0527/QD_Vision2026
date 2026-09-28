/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static USARTInstance *vision_usart_instance;

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

#if defined(VISION_USE_UART) || defined(VISION_USE_VCP)
/**
 * @brief 校验视觉帧并更新 pitch、yaw。
 * @param frame 完整的接收帧。
 * @param frame_len 实际收到的帧长，单位为字节。
 */
static void DecodeVisionFrame(uint8_t *frame, uint16_t frame_len)
{
    float values[VISION_RECV_FLOAT_COUNT];

    if (frame_len != VISION_RECV_SIZE
        || !get_protocol_info(frame, frame_len, (uint8_t *)values, (uint16_t)sizeof(values))) {
        return;
    }
    recv_data.pitch = values[0];
    recv_data.yaw = values[1];
    if (vision_daemon_instance != NULL) {
        DaemonReload(vision_daemon_instance);
    }
}

/**
 * @brief 将电控数据按约定的 15 个 float 顺序打包。
 * @param buffer 输出缓冲区。
 * @param capacity 输出缓冲区容量，单位为字节。
 * @param length 实际帧长的输出地址。
 * @return 1 表示打包成功，0 表示缓冲区容量或参数无效。
 */
static uint8_t PackVisionSend(uint8_t *buffer, uint16_t capacity, uint16_t *length)
{
    const float values[VISION_SEND_FLOAT_COUNT] = {
        send_data.yaw,
        send_data.pitch,
        send_data.roll,
        send_data.control_id,
        send_data.game_progress,
        send_data.current_HP,
        send_data.current_base_hp,
        send_data.allow_fire_amount,
        send_data.current_outpost_hp,
        send_data.current_enemy_base_hp,
        send_data.current_enemy_outpost_hp,
        send_data.hero_x,
        send_data.hero_y,
        send_data.cmd_x,
        send_data.cmd_y
    };

    return get_protocol_send_data(values, VISION_SEND_FLOAT_COUNT, buffer, capacity, length);
}
#endif

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"



/**
 * @brief 在串口接收回调中校验固定长度的视觉帧。
 * @note 底层串口驱动应在完整接收 VISION_RECV_SIZE 字节后调用此函数。
 */
static void DecodeVision(void)
{
    DecodeVisionFrame(vision_usart_instance->recv_buff, VISION_RECV_SIZE);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf = {0};
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10, 
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 将当前电控数据发送给视觉模块。
 * @note DMA 发送期间应由底层串口驱动避免再次占用发送缓冲区。
 */
void VisionSend(void)
{
    // 发送缓冲区必须持续存在，直到 DMA 发送结束。
    static uint8_t send_buff[VISION_SEND_SIZE];
    uint16_t tx_len = 0;

    if (vision_usart_instance == NULL
        || !PackVisionSend(send_buff, VISION_SEND_SIZE, &tx_len)) {
        return;
    }
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA);
}

/**
 * @brief 向视觉发送裁判系统数据发送
 */
void RefereeSetAltitude(float current_HP,float game_progress)
{
    send_data.current_HP = current_HP;
    send_data.game_progress = game_progress;
    send_data.control_id = 100;
}
/**
    *@发给视觉裁判系统2部分
 */
void RefereeSetAltitude2(float cb_hp, float ala, float co_hp, float ceb_hp, float ceo_hp, float hero_x, float hero_y,float cmd_x,float cmd_y)
{
    send_data.current_base_hp = cb_hp;           // 己方基地血量
    send_data.allow_fire_amount = ala;           // 允许发射的子弹数量
    send_data.current_outpost_hp = co_hp;        // 当前前哨站血量
    // send_data.current_enemy_sentry_hp = ces_hp;  // 当前敌方哨兵血量
    send_data.current_enemy_base_hp = ceb_hp;    // 当前敌方基地血量
    send_data.current_enemy_outpost_hp = ceo_hp; // 当前敌方前哨站血量
    send_data.hero_x = hero_x;                   // 英雄X坐标
    send_data.hero_y = hero_y;                   // 英雄Y坐标
    send_data.cmd_x = cmd_x;
    send_data.cmd_y = cmd_y;
}
#endif // VISION_USE_UART

#ifdef VISION_USE_VCP
 #include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    DecodeVisionFrame(vis_recv_buff, recv_len);
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend(void)
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    uint16_t tx_len = 0;

    if (vis_recv_buff == NULL
        || !PackVisionSend(send_buff, VISION_SEND_SIZE, &tx_len)) {
        return;
    }
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
