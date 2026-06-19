#include "bsp_fdcan.h"
#include "dj_motor.h"
#include "dm_motor_ctrl.h"
#include "DMmotor_task.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

static FDCAN_TxHeaderTypeDef  tx_message;

uint8_t CAN_send(FDCAN_HandleTypeDef *can, uint32_t send_id, uint8_t data[])
{
    // fdcanx_send_data(can, send_id, data, 8);
    // uint32_t send_mail_box;
    tx_message.Identifier=send_id;
    tx_message.IdType=FDCAN_STANDARD_ID;
    tx_message.TxFrameType=FDCAN_DATA_FRAME;
    tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch=FDCAN_BRS_ON;//FDCAN_BRS_OFF;
    tx_message.FDFormat=FDCAN_FD_CAN;//FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker=0;
//    if(len<=8)
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
//    if(len==12)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
//    if(len==16)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
//    if(len==20)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
//    if(len==24)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
//    if(len==32)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
//    if(len==48)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
//    if(len==64)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
    /* HAL API*/
    if(HAL_FDCAN_AddMessageToTxFifoQ(can, &tx_message, data)!=HAL_OK)
        return 1;
    return 0;
}



/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{

    can_filter_init();

    HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);

    // 开启关节电机读取中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);

    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);

}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter1;
    //过滤器1负责FIFO0
    fdcan_filter1.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter1.FilterIndex = 0;                                  //滤波器索引
    fdcan_filter1.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0
    fdcan_filter1.FilterID1 = 0x00;
    fdcan_filter1.FilterID2 = 0x00;

    FDCAN_FilterTypeDef fdcan_filter2;
    //过滤器2负责FIFO1
    fdcan_filter2.IdType = FDCAN_STANDARD_ID;                       //标准ID
    fdcan_filter2.FilterIndex = 0;                                  //滤波器索引
    fdcan_filter2.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;           //过滤器0关联到FIFO0
    fdcan_filter2.FilterID1 = 0x00;
    fdcan_filter2.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter1); 		 				  //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
    //HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);

    HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter1); 		 				  //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
   // HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);


    HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter1); 		 				  //接收ID2
    //拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    //HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);

}
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
    uint32_t nom_brp=0, nom_seg1=0, nom_seg2=0, nom_sjw=0;
    uint32_t dat_brp=0, dat_seg1=0, dat_seg2=0, dat_sjw=0;

    /*	nominal_baud = 80M/brp/(1+seg1+seg2)
        sample point = (1+seg1)/(1+seg1+seg2)
        sjw :	1-128
        seg1:	2-256
        seg2: 	2-128
        brp :	1-512  */
    if(mode == CAN_CLASS)
    {
        switch (baud)
        {
            case CAN_BR_125K: 	nom_brp=4 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
            case CAN_BR_200K: 	nom_brp=2 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
            case CAN_BR_250K: 	nom_brp=2 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
            case CAN_BR_500K: 	nom_brp=1 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
            case CAN_BR_1M:		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; break; // sample point 75%
        }
        dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10;	// 数据域默认1M
        hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    }
    /*	data_baud	 = 80M/brp/(1+seg1+seg2)
        sample point = (1+seg1)/(1+seg1+seg2)
        sjw :	1-16
        seg1:	1-32
        seg2: 	2-16
        brp :	1-32  */
    if(mode == CAN_FD_BRS)
    {
        switch (baud)
        {
            case CAN_BR_2M: 	dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10; break;	// sample point 75%
            case CAN_BR_2M5: 	dat_brp=1 ; dat_seg1=25; dat_seg2=6 ; dat_sjw=6 ; break;	// sample point 81.25%
            case CAN_BR_3M2: 	dat_brp=1 ; dat_seg1=19; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 80%
            case CAN_BR_4M: 	dat_brp=1 ; dat_seg1=14; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 75%
            case CAN_BR_5M:		dat_brp=1 ; dat_seg1=13; dat_seg2=2 ; dat_sjw=2 ; break;	// sample point 87.5%
        }
        nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; // 仲裁域默认1M
        hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    }

    HAL_FDCAN_DeInit(hfdcan);

    hfdcan->Init.NominalPrescaler = nom_brp;
    hfdcan->Init.NominalTimeSeg1  = nom_seg1;
    hfdcan->Init.NominalTimeSeg2  = nom_seg2;
    hfdcan->Init.NominalSyncJumpWidth = nom_sjw;

    hfdcan->Init.DataPrescaler = dat_brp;
    hfdcan->Init.DataTimeSeg1  = dat_seg1;
    hfdcan->Init.DataTimeSeg2  = dat_seg2;
    hfdcan->Init.DataSyncJumpWidth = dat_sjw;

    HAL_FDCAN_Init(hfdcan);
}


/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;

    if(len<=8)
        pTxHeader.DataLength = len;
    else if(len==12)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    else if(len==16)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    else if(len==20)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    else if(len==24)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    else if(len==32)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    else if(len==48)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    else if(len==64)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;

    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK)
        return 1;//发送
    return 0;
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive_FIFO0(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;
    if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;
        if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
            len = 12;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
            len = 16;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
            len = 20;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
            len = 24;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
            len = 32;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
            len = 48;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
            len = 64;

        return len;//接收数据
    }
    return 0;
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive_FIFO1(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;
    if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &pRxHeader, buf)==HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;
        if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
            len = 12;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
            len = 16;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
            len = 20;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
            len = 24;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
            len = 32;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
            len = 48;
        else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
            len = 64;

        return len;//接收数据
    }
    return 0;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    //!!!利用这个循环就要确保数据能在接收后被处理掉，不然就会死循环
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        if (hfdcan == &hfdcan1)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
            dji_motor_rx_callback(rx_header.Identifier, rx_data);
        }
        else if (hfdcan == &hfdcan2)
        {
            fdcan2_process_callback();//234567号电机
        }
        else if (hfdcan == &hfdcan3)
        {
            fdcan3_process_callback();//1号电机
        }
    }
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {

    }
}



void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if(ErrorStatusITs & FDCAN_IR_BO)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
    if(ErrorStatusITs & FDCAN_IR_EP)
    {
        MX_FDCAN1_Init();
        bsp_can_init();
    }
}


/**
************************************************************************
* @brief:      	fdcan2_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN2接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void fdcan2_process_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    // 取出一帧fdcan2中FIFO 0的原始数据帧，开始解包函数
    fdcanx_receive_FIFO0(&hfdcan2, &rec_id, rx_data);//取出接收到的数据帧
    // 获取电机ID号
    rec_id = (rx_data[0]) & 0x0F;
    switch (rec_id) {
//        case 0x01: {
//            // 调用关节电机解包函数，解出位置速度和力矩的值，并存入到对应的结构体中
//            dm_motor_fbdata(&motor[Motor1], rx_data);
//            break;
//        }
//        case 0x02: {
//            // 调用关节电机解包函数，解出位置速度和力矩的值，并存入到对应的结构体中
//            dm_motor_fbdata(&motor[Motor2], rx_data);
//            break;
//        }
        case 0x03: {
            // 调用关节电机解包函数，解出位置速度和力矩的值，并存入到对应的结构体中
            dm_motor_fbdata(&motor[Motor3], rx_data);
            break;
        }

        case 0x04: {
            dm_motor_fbdata(&motor[Motor4], rx_data);
            break;
        }

        case 0x05: {
            dm_motor_fbdata(&motor[Motor5], rx_data);
            break;
        }

        case 0x06: {
            dm_motor_fbdata(&motor[Motor6], rx_data);
            break;
        }

        case 0x07: {//夹爪注重于力矩控制故不需要位置控制
            dm_motor_fbdata(&motor[Motor7], rx_data);
            break;
        }
        default:
            break;
    }
}


void fdcan3_process_callback(void)
{
    uint16_t rec_id;
    uint8_t rx_data[8] = {0};
    fdcanx_receive_FIFO0(&hfdcan3, &rec_id, rx_data);
    rec_id = (rx_data[0]) & 0x0F;

    if(rec_id == 0x01)//处理一号电机数据
    {
        dm_motor_fbdata(&motor[Motor1], rx_data);
    }
    else if(rec_id == 0x02)//处理一号电机数据
    {
        dm_motor_fbdata(&motor[Motor2], rx_data);
    }
}



