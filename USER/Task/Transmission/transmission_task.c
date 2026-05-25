/**
  ******************************************************************************
  * @file    transmission_task.c
  * @author  Liu JiaJun(187353224@qq.com) and RockZhang(2431952330@qq.com)
  * @version V2.0.0
  * @date    2025-10-16
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include "transmission_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include "msg_freertos.h"
#include "robot_task.h"
#include "usbd_cdc_if.h"
#include "cmd_task.h"

#define MAX_USB_BUF_LEN     60
#define HEAD_BUF_LEN        4       // 帧头长度（0-3字节:帧头0xFF、地址、命名ID、数据长度）
#define CRC_BUF_LEN         2       // 帧头长度（0-3字节:帧头0xFF、地址、命名ID、数据长度）

#define USB_INS_DATA_LEN    36      // 数据部分长度（9个int32_t）
#define INS_BUF_LEN         (USB_INS_DATA_LEN+HEAD_BUF_LEN+CRC_BUF_LEN)      // 总长度：4(帧头)+36(数据)+2(校验)

#define USB_CMD_CHASSIS_DATA_LEN    12      // 数据部分长度（3个int32_t）
#define CMD_CHASSIS_BUF_LEN         (USB_CMD_CHASSIS_DATA_LEN+HEAD_BUF_LEN+CRC_BUF_LEN)      // 总长度：4(帧头)+12(数据)+2(校验)

#define USB_ARM_JOINTS_DATA_LEN    51      // 数据部分长度（12个int32_t+3个uint8_t）
#define USB_ARM_JOINTS_POS_DATA_LEN    24
#define USB_ARM_JOINTS_VEL_DATA_LEN    24
#define ARM_JOINTS_BUF_LEN         (USB_ARM_JOINTS_DATA_LEN+HEAD_BUF_LEN+CRC_BUF_LEN)      // 总长度：4(帧头)+49(数据)+2(校验)

#define USB_VOICE_CONTROL_LEN    24
#define VOICE_CONTROL_BUF_LEN         (HEAD_BUF_LEN+USB_VOICE_CONTROL_LEN+CRC_BUF_LEN)      // ??????4(??)+24(????)+2(У??)

#define HEADER_SOF 0xFF

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static struct ins_msg transmission_subscribe_ins_data;
static struct cmd_chassis_msg receive_pc_cmd_chassis_data;
static struct pc_cmd_arm_msg receive_pc_cmd_arm_msg_data;
static struct pc_cmd_voice_control_msg receive_pc_cmd_voice_control_data;
static dm_arm_feedback_msg_t transmission_subscribe_arm_feedback_data;

static uint16_t receive_pc_keyboard_data;
static publisher_t *pc_cmd_arm_topic_publish;
static publisher_t *pc_cmd_chassis_topic_publish;
static publisher_t *pc_cmd_voice_control_publisher;
static publisher_t *nuc_keyboard_publisher;
static subscriber_t *subscribe_ins_topic;
static subscriber_t *subscribe_cmd_chassis_topic;
static subscriber_t *subscribe_arm_feedback_topic;

static uint8_t usb_txbuffer[MAX_USB_BUF_LEN] = {0};

extern uint8_t USB_Received_Data[APP_RX_DATA_SIZE];//接收usb数据缓冲区
extern uint32_t USB_Received_Len;
extern volatile uint8_t USB_Data_Ready_Flag;


static uint8_t Rx_data[64];
static uint8_t Data_len = 0;
static uint32_t num_count = 0;
static uint32_t sum_check = 0;
static uint32_t addr_check = 0;


static void transmission_topic_pub_init(void);
static void transmission_topic_sub_init(void);
static void transmission_topic_pub_push(void);
static void transmission_topic_sub_pull(void);

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t transmission_task_dwt = 0;   // 毫秒监测
static float transmission_task_dt = 0;       // 线程实际运行时间dt
static float transmission_task_delta = 0;    // 监测线程运行时间
static float transmission_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

void InsDataPack(void);
void CmdChassisDataPack(void);
void set_data(uint8_t rx_byte);
void UnpackPCData(void);
void ArmJointDataPack(void);

/* -------------------------------- 线程入口 ------------------------------- */
void TransmissionTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
    transmission_topic_sub_init();
    transmission_topic_pub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
    transmission_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        transmission_task_delta = dwt_get_time_ms() - transmission_task_start_dt;
        transmission_task_start_dt = dwt_get_time_ms();
        transmission_task_dt = dwt_get_delta(&transmission_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */

/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        transmission_topic_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        //接收上位机数据并解析
        if (USB_Data_Ready_Flag == 1)//虚拟串口接收到数据，内置函数直接将标志位设为1
        {
            UnpackPCData();
            USB_Received_Len = 0;
            USB_Data_Ready_Flag = 0;
        }
/* --------------------------------在此处添加要发给上位机的打包函数 --------------------------------- */
        //InsDataPack();
        //vTaskDelay(1);
        ArmJointDataPack();//发给上位机
        vTaskDelay(1);
/* --------------------------------在此处添加要发给上位机的打包函数 --------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        transmission_topic_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(18);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static void transmission_topic_pub_init(void)
{
    pc_cmd_chassis_topic_publish = pub_register("pc_cmd_chassis",sizeof(struct cmd_chassis_msg));
    pc_cmd_arm_topic_publish = pub_register("pc_cmd_arm_pub",sizeof(struct pc_cmd_arm_msg));
    pc_cmd_voice_control_publisher =pub_register("voice_control_pub",sizeof(struct pc_cmd_voice_control_msg));
    nuc_keyboard_publisher =pub_register("nuc_keyboard_data",sizeof(uint16_t));
}

static void transmission_topic_sub_init(void)
{
    subscribe_ins_topic = sub_register("ins_pub", sizeof(struct ins_msg));
    subscribe_cmd_chassis_topic = sub_register("cmd_ch_pub", sizeof(struct cmd_chassis_msg));
    subscribe_arm_feedback_topic = sub_register("dm_arm_feedback_pub", sizeof(dm_arm_feedback_msg_t));
}

void uppack_pc_cmd_chassis_data(void)
{
    receive_pc_cmd_chassis_data.vx = (float)((Rx_data[4] << 0) |  // byte3占 31-24位
                                             (Rx_data[5] << 8) |  // byte2占 23-16位
                                             (Rx_data[6] << 16)  |  // byte1占 15-8位
                                             (Rx_data[7] << 24)) / 10000.0f;    // byte0占 7-0位
    receive_pc_cmd_chassis_data.vy = (float)((Rx_data[8] << 0) |  // byte3占 31-24位
                                             (Rx_data[9] << 8) |  // byte2占 23-16位
                                             (Rx_data[10] << 16)  |  // byte1占 15-8位
                                             (Rx_data[11] << 24)) / 10000.0f;    // byte0占 7-0位
    receive_pc_cmd_chassis_data.vw = (float)((Rx_data[24] << 0) |  // byte3占 31-24位
                                             (Rx_data[25] << 8) |  // byte2占 23-16位
                                             (Rx_data[26] << 16)  |  // byte1占 15-8位
                                             (Rx_data[27] << 24)) / 10000.0f;    // byte0占 7-0位
}

void uppack_keyboard_data(void)
{
    receive_pc_keyboard_data = (uint16_t)((Rx_data[4] << 0) | (Rx_data[5] << 8) );  // byte3占 31-24位

}

void uppack_pc_cmd_arm_data(void)
{
    //  /10000.0f与上位机约定好，因为传输float麻烦
    receive_pc_cmd_arm_msg_data.joint_pos[0] = (float)((Rx_data[4] << 0) |  // byte3占 31-24位
                                                       (Rx_data[5] << 8) |  // byte2占 23-16位
                                                       (Rx_data[6] << 16)  |  // byte1占 15-8位
                                                       (Rx_data[7] << 24)) /10000.0f;    // byte0占 7-0位
    receive_pc_cmd_arm_msg_data.joint_pos[1] = (float)((Rx_data[8] << 0) |  // byte3占 31-24位
                                                       (Rx_data[9] << 8) |  // byte2占 23-16位
                                                       (Rx_data[10] << 16)  |  // byte1占 15-8位
                                                       (Rx_data[11] << 24)) /10000.0f;    // byte0占 7-0位
    receive_pc_cmd_arm_msg_data.joint_pos[2] = (float)((Rx_data[12] << 0) |  // byte3占 31-24位
                                                       (Rx_data[13] << 8) |  // byte2占 23-16位
                                                       (Rx_data[14] << 16)  |  // byte1占 15-8位
                                                       (Rx_data[15] << 24))/10000.0f;    // byte0占 7-0位
    receive_pc_cmd_arm_msg_data.joint_pos[3] = (float)((Rx_data[16] << 0) |  // byte3占 31-24位
                                                       (Rx_data[17] << 8) |  // byte2占 23-16位
                                                       (Rx_data[18] << 16)  |  // byte1占 15-8位
                                                       (Rx_data[19] << 24)) /10000.0f;    // byte0占 7-0位
    receive_pc_cmd_arm_msg_data.joint_pos[4] = (float)((Rx_data[20] << 0) |  // byte3占 31-24位
                                                       (Rx_data[21] << 8) |  // byte2占 23-16位
                                                       (Rx_data[22] << 16)  |  // byte1占 15-8位
                                                       (Rx_data[23] << 24)) /10000.0f;    // byte0占 7-0位
    receive_pc_cmd_arm_msg_data.joint_pos[5] = (float)((Rx_data[24] << 0) |  // byte3占 31-24位
                                                       (Rx_data[25] << 8) |  // byte2占 23-16位
                                                       (Rx_data[26] << 16)  |  // byte1占 15-8位
                                                       (Rx_data[27] << 24)) /10000.0f;    // byte0占 7-0位
    receive_pc_cmd_arm_msg_data.gripper_ctrl = (Rx_data[28] << 0) ;
    receive_pc_cmd_arm_msg_data.control_state = (Rx_data[29] << 0);
    receive_pc_cmd_arm_msg_data.pc_ctrl_process_state = (int8_t)(Rx_data[30] << 0);
}


void uppack_pc_cmd_voice_control(void)
{
    receive_pc_cmd_voice_control_data.vx = (float)((Rx_data[4] << 0) |  // byte3占 31-24位
                                                   (Rx_data[5] << 8) |  // byte2占 23-16位
                                                   (Rx_data[6] << 16)  |  // byte1占 15-8位
                                                   (Rx_data[7] << 24)) / 10000.0f;    // byte0占 7-0位
    receive_pc_cmd_voice_control_data.vy = (float)((Rx_data[8] << 0) |  // byte3? 31-24λ
                                                   (Rx_data[9] << 8) |  // byte2? 23-16λ
                                                   (Rx_data[10] << 16)  |  // byte1? 15-8λ
                                                   (Rx_data[11] << 24)) / 10000.0f;    // byte0? 7-0λ
    receive_pc_cmd_voice_control_data.vw = (float)((Rx_data[24] << 0) |  // byte3? 31-24λ
                                                   (Rx_data[25] << 8) |  // byte2? 23-16λ
                                                   (Rx_data[26] << 16)  |  // byte1? 15-8λ
                                                   (Rx_data[27] << 24)) / 10000.0f;    // byte0? 7-0λ
}
static void transmission_topic_pub_push(void)
{
    pub_push_msg(pc_cmd_chassis_topic_publish,&receive_pc_cmd_chassis_data);
    pub_push_msg(pc_cmd_arm_topic_publish,&receive_pc_cmd_arm_msg_data);
    pub_push_msg(pc_cmd_voice_control_publisher,&receive_pc_cmd_voice_control_data);
    pub_push_msg(nuc_keyboard_publisher,&receive_pc_keyboard_data);
}

static void transmission_topic_sub_pull(void)
{
    sub_get_msg(subscribe_ins_topic, &transmission_subscribe_ins_data);
    sub_get_msg(subscribe_arm_feedback_topic, &transmission_subscribe_arm_feedback_data);
}


/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

/* -------------------------------- 线程间通讯数据包相关 ------------------------------- */

void InsDataPack()//目前上位机用不到下位机陀螺仪
{
    // 初始化缓冲区，避免脏数据
    memset(usb_txbuffer, 0, MAX_USB_BUF_LEN);

    // 填充帧头信息
    usb_txbuffer[0] = 0xFF;  // 帧头
    usb_txbuffer[1] = 0x05;  // 地址
    usb_txbuffer[2] = 0x13;  // 命名ID
    usb_txbuffer[3] = USB_INS_DATA_LEN;  // 数据长度

    const float scale = 10000.0f;
    int32_t chassis_imu_eul_yaw = (int32_t)(transmission_subscribe_ins_data.yaw * scale);
    int32_t chassis_imu_eul_pitch = (int32_t)(transmission_subscribe_ins_data.pitch * scale);
    int32_t chassis_imu_eul_roll = (int32_t)(transmission_subscribe_ins_data.roll * scale);
    int32_t chassis_imu_angle_x = (int32_t)(transmission_subscribe_ins_data.gyro[0] * scale);
    int32_t chassis_imu_angle_y = (int32_t)(transmission_subscribe_ins_data.gyro[1] * scale);
    int32_t chassis_imu_angle_z = (int32_t)(transmission_subscribe_ins_data.gyro[2] * scale);
    int32_t chassis_imu_accel_x = (int32_t)(transmission_subscribe_ins_data.accel[0] * scale);
    int32_t chassis_imu_accel_y = (int32_t)(transmission_subscribe_ins_data.accel[1] * scale);
    int32_t chassis_imu_accel_z = (int32_t)(transmission_subscribe_ins_data.accel[2] * scale);

    uint32_t offset = HEAD_BUF_LEN;  // 从帧头后开始
    memcpy(&usb_txbuffer[offset], &chassis_imu_eul_yaw, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_eul_pitch, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_eul_roll, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_angle_x, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_angle_y, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_angle_z, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_accel_x, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_accel_y, sizeof(int32_t)); offset += sizeof(int32_t);
    memcpy(&usb_txbuffer[offset], &chassis_imu_accel_z, sizeof(int32_t)); offset += sizeof(int32_t);

    // 计算校验码（覆盖0到最后一个数据字节）
    sum_check = 0;
    addr_check = 0;
    for (int i = 0; i < (HEAD_BUF_LEN + USB_INS_DATA_LEN); i++) {
        sum_check += usb_txbuffer[i];
        addr_check += sum_check;
    }
    usb_txbuffer[offset++] = sum_check & 0xFF;
    usb_txbuffer[offset] = addr_check & 0xFF;

    // 发送数据，检查返回值确保发送成功
    if (CDC_Transmit_HS(usb_txbuffer, INS_BUF_LEN) != USBD_OK) {
    }
}





void UnpackPCData(void)
{
    uint8_t rx_data_state = STEP_HEADER_SOF;
    Data_len = 0;
    sum_check = 0;
    addr_check = 0;
    memset(Rx_data, 0, sizeof(Rx_data));

    for (num_count = 0; num_count < USB_Received_Len; num_count++)
    {
        uint8_t curr_byte = USB_Received_Data[num_count];

        if (curr_byte == HEADER_SOF && rx_data_state == STEP_HEADER_SOF) // 等待帧头
        {
            set_data(curr_byte);
            rx_data_state = STEP_ADDRESS;
        }
        else if (curr_byte == 0x05 && rx_data_state == STEP_ADDRESS) // 等待地址//工程车固定ID号为0x05
        {
            set_data(curr_byte);
            rx_data_state = STEP_ID;
        }
        else if (rx_data_state == STEP_ID)
        {
            set_data(curr_byte);
            rx_data_state = STEP_LEN;
        }
        else if (rx_data_state == STEP_LEN)
        {
            if (Data_len == 3)//前三个帧头数量无误
            {
                set_data(curr_byte);
            }
            else
            {
                // 已接收数据字节数 = 总接收字节数 - 帧头长度（4字节）
                if ((Data_len - HEAD_BUF_LEN) <= Rx_data[3])//Rx_data[3]为数据包的长度
                {
                    set_data(curr_byte);
                }
                // 数据段接收完成，进入校验阶段
                if ((Data_len - HEAD_BUF_LEN) == Rx_data[3])
                {
                    rx_data_state = STEP_DATA;
                }
            }
            if (Data_len >= sizeof(Rx_data))//接收到的数据长度异常,抛弃
            {
                rx_data_state = STEP_HEADER_SOF;
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
            }
        }
        else if (rx_data_state == STEP_DATA) // 校验sum_check
        {
            if ((sum_check & 0xFF) == curr_byte)
            {
                rx_data_state = STEP_SC;
            }
            else
            {
                rx_data_state = STEP_HEADER_SOF;
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
            }
        }
        else if (rx_data_state == STEP_SC) // 校验addr_check
        {
            if ((addr_check & 0xFF) == curr_byte)
            {
                rx_data_state = STEP_HEADER_SOF;//数据校验成功，reset相关变量
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;

/* --------------------------------在此处添加要从上位机接收的id以及解包函数 --------------------------------- */

                if(Rx_data[2] == 0x12)//导航数据包
                {
                    uppack_pc_cmd_chassis_data();
                }
                else if(Rx_data[2] == 0x14)//键盘数据包
                {
                    uppack_keyboard_data();
                }
                else if(Rx_data[2] == 0x21)//机械臂数据包
                {
                    uppack_pc_cmd_arm_data();
                }

/* --------------------------------在此处添加要从上位机接收的id以及解包函数 --------------------------------- */

            }
            else
            {
                rx_data_state = STEP_HEADER_SOF;//数据校验失败，reset相关变量
                Data_len = 0;
                sum_check = 0;
                addr_check = 0;
            }
        }
        else
        {
            rx_data_state = STEP_HEADER_SOF;//数据解包失败，reset相关变量
            Data_len = 0;
            sum_check = 0;
            addr_check = 0;
        }
    }
}


void set_data(uint8_t rx_byte)
{
    if (Data_len < sizeof(Rx_data))
    {
        Rx_data[Data_len] = rx_byte;
        Data_len++;
        sum_check += rx_byte;
        addr_check += sum_check;
    }
    else
    {
        Data_len = 0;
        sum_check = 0;
        addr_check = 0;
    }
}



void ArmJointDataPack(void) {
    // 初始化缓冲区，避免脏数据
    memset(usb_txbuffer, 0, MAX_USB_BUF_LEN);

    // 填充帧头信息
    usb_txbuffer[0] = 0xFF;  // 帧头
    usb_txbuffer[1] = 0x05;  // 地址
    usb_txbuffer[2] = 0x20;  // 命名ID
    usb_txbuffer[3] = USB_ARM_JOINTS_DATA_LEN;  // 数据长度

    // 将关节数据填入缓冲区
    for (int i = 0; i < 6; i++) {
        int32_t joint_position = (int32_t) (transmission_subscribe_arm_feedback_data.joint[i].pos_rad * 10000.0f);
        memcpy(&usb_txbuffer[HEAD_BUF_LEN + i * 4], &joint_position, sizeof(int32_t));  // 填充关节位置
    }
    for (int i = 0; i < 6; i++) {
        int32_t joint_velocity = (int32_t) (transmission_subscribe_arm_feedback_data.joint[i].vel_rad_s * 10000.0f);
        memcpy(&usb_txbuffer[HEAD_BUF_LEN + USB_ARM_JOINTS_POS_DATA_LEN + i * 4], &joint_velocity, sizeof(int32_t));  // 填充关节位置
    }

    // 添加 gripper_state 和 auto_state
    memcpy(&usb_txbuffer[HEAD_BUF_LEN + USB_ARM_JOINTS_POS_DATA_LEN + USB_ARM_JOINTS_VEL_DATA_LEN ], &transmission_subscribe_arm_feedback_data.gripper_state, sizeof(int8_t));
    usb_txbuffer[HEAD_BUF_LEN + USB_ARM_JOINTS_POS_DATA_LEN + USB_ARM_JOINTS_VEL_DATA_LEN + sizeof(int8_t) ] = 1; //不为1的话上位机不能初始化成功，所以强制发送0，目前不影响正常车的运动
    memcpy(&usb_txbuffer[HEAD_BUF_LEN + USB_ARM_JOINTS_POS_DATA_LEN + USB_ARM_JOINTS_VEL_DATA_LEN + 2 * sizeof(int8_t) ], &transmission_subscribe_arm_feedback_data.auto_ctrl_mode, sizeof(int8_t));
    // 计算校验码（覆盖0到最后一个数据字节）
    sum_check = 0;
    addr_check = 0;
    for (int i = 0; i < (HEAD_BUF_LEN + USB_ARM_JOINTS_DATA_LEN); i++) {
        sum_check += usb_txbuffer[i];
        addr_check += sum_check;
    }
    usb_txbuffer[HEAD_BUF_LEN + USB_ARM_JOINTS_DATA_LEN ] = sum_check & 0xFF;
    usb_txbuffer[HEAD_BUF_LEN + USB_ARM_JOINTS_DATA_LEN + sizeof(int8_t)] = addr_check & 0xFF;

    // 发送数据
    if (CDC_Transmit_HS(usb_txbuffer, ARM_JOINTS_BUF_LEN) != USBD_OK) {
        // 发送失败处理
    }
}


/* -------------------------------- 线程间通讯数据包相关 ------------------------------- */