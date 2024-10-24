#include "drv_can.h"

/**
 * @brief can初始化(默认can1分配滤波器0和1,can2分配滤波器13和14,均使能FIFO0和FIFO1)
 * @param hcan can类型句柄指针
*/
void CAN_Init(CAN_HandleTypeDef *hcan){
	HAL_CAN_Start (hcan);
	__HAL_CAN_ENABLE_IT (hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	__HAL_CAN_ENABLE_IT (hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
	if(hcan->Instance == CAN1)
    {
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }																																																																																																			
}
/**
 * @brief 发送can报文
 * @param hcan can类型句柄指针
 * @param ID 报文ID(标识符)
 * @param DATA 发送数据数组的指针
 * @param Length 发送数据的长度(数组长度)
 * @retval 发送成功返回HAL_OK
*/
uint8_t CAN_SEND_Data(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t *DATA,uint16_t Length){
	CAN_TxHeaderTypeDef tx_header;
	uint32_t used_mailbox;
	assert_param(hcan!=NULL);
	tx_header.StdId =ID;
	tx_header.ExtId =0;
	tx_header.IDE =0;
	tx_header.RTR =0;
	tx_header.DLC =Length;
	return(HAL_CAN_AddTxMessage(hcan,&tx_header,DATA,&used_mailbox));
}
/**
 * @brief 发送can报文
 * @param hcan can类型句柄指针
 * @param ID 报文ID(标识符)
 * @param DATA 发送数据数组的指针
 * @param Length 发送数据的长度(数组长度)
 * @retval 发送成功返回HAL_OK
*/
void CAN_SendData_3508(int16_t motorl,int16_t motor2,int16_t motor3,int16_t motor4)
{    
	CAN_TxHeaderTypeDef hcan1_tx;
	uint8_t DATA[8];	
	uint32_t mailbox;
	
	hcan1_tx.DLC = 0x08;               //date_CAN1数据每一个的长度
	hcan1_tx.IDE = CAN_ID_STD;         //决定用标准(11位)还是扩展ID(29位)的标识符，即帧类型 
	hcan1_tx.StdId = 0x200;            //标识符，电调说明书上有
	hcan1_tx.RTR = CAN_RTR_DATA;       //CAN_RTR_DATA表示数据帧，实际传输数据；CAN_RTR_REMOTE远程帧，请求远程节点发送数据

	DATA[0] = motorl >> 8;
	DATA[1] = motorl & 0xFF;
	DATA[2] = motor2 >> 8;
	DATA[3] = motor2 & 0xFF;
	DATA[4] = motor3 >> 8;
	DATA[5] = motor3 & 0xFF;
	DATA[6] = motor4 >> 8;
	DATA[7] = motor4 & 0xFF;
	
	HAL_CAN_AddTxMessage(&hcan1,&hcan1_tx,DATA,&mailbox);	//此函数可以添加多个数去缓冲区，课设置优先级；而HAL_CAN_Transmit直接发送到CAN总线上
}
/**
 * @brief 配置滤波器，FIFO，掩码和接收的报文ID
*/
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan,uint8_t object_param,uint32_t ID,uint32_t mask_ID){
	CAN_FilterTypeDef can_filter_init;
	assert_param(hcan!=NULL);
	if((object_param &0x02)){
	 	can_filter_init.FilterIdHigh =ID<<3<<16;																				
	 	can_filter_init.FilterIdLow =ID<<3|((object_param &0x03))<<1;
		can_filter_init.FilterMaskIdHigh =mask_ID <<3<<16;
		can_filter_init.FilterMaskIdLow =mask_ID<<3|((object_param &0x03))<<1; 
	}
	else{
		can_filter_init.FilterIdHigh =ID<<5;																				
	  	can_filter_init.FilterIdLow =ID<<3|((object_param &0x03))<<1;
		can_filter_init.FilterMaskIdHigh =mask_ID <<5;
		can_filter_init.FilterMaskIdLow =mask_ID<<3|((object_param &0x03))<<1; 
	}
	
	can_filter_init.FilterBank =object_param >>3;
	can_filter_init.FilterFIFOAssignment=((object_param >>2)&0x01);
	can_filter_init.FilterActivation =ENABLE ;
	can_filter_init.FilterMode =CAN_FILTERMODE_IDMASK;
	can_filter_init.FilterScale =CAN_FILTERSCALE_32BIT;
	can_filter_init.SlaveStartFilterBank =14;
	HAL_CAN_ConfigFilter (hcan,&can_filter_init);
}

void can_remote(uint8_t sbus_buf[],uint8_t can_send_id)
{
	CAN_TxHeaderTypeDef can_remote_message;
	uint32_t remote_mail_box = (uint32_t)CAN_TX_MAILBOX1; //邮箱2
	can_remote_message.StdId = can_send_id;
	can_remote_message.IDE = CAN_ID_STD;
	can_remote_message.RTR = CAN_RTR_DATA;
	can_remote_message.DLC = 0x08;
	HAL_CAN_AddTxMessage(&hcan1,&can_remote_message,sbus_buf,&remote_mail_box);
}

