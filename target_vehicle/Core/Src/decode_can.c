#include "decode_can.h"
#include "rc.h"

uint8_t DATA[8];	
uint8_t can_rx_data[32];  // ���ڴ洢���յ�������
CanData decoded_data = {0};
CanData motor1 = {0};
CanData motor2 = {0};
CanData motor3 = {0};
CanData motor4 = {0};

//���������ص���ĺ���
CanData decode_can_data(uint8_t data[8]) 
{
	decoded_data.last_ecd = decoded_data.ecd; 
	
	// ����ת�ӻ�е�Ƕ�
	decoded_data.rotor_angle = ((uint16_t)data[0] << 8) | data[1];
    
  // ����ת��ת��
  decoded_data.RPM = ((int16_t)data[2] << 8) | data[3];
    
  // ����ʵ��ת�ص���
  decoded_data.torque_current = ((int16_t)data[4] << 8) | data[5];
    
  // �������¶�
  decoded_data.motor_temperature = data[6];
	
	decoded_data.ecd = decoded_data.rotor_angle;

  if (decoded_data.ecd - decoded_data.last_ecd > 4096)
		decoded_data.total_round--;
	else if (decoded_data.ecd - decoded_data.last_ecd < -4096)
		decoded_data.total_round++;
	
	decoded_data.total_angle = decoded_data.total_round * 360.0 + (decoded_data.ecd/8192.0)*360.0; 
  decoded_data.total_distance = (decoded_data.total_round + decoded_data.ecd/8192.0)*2*PI*R;       // �������ڵ��ܾ��룬�з��򣬵�λm
  // ���ؽ����Ľṹ��
  return decoded_data;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{  CAN_RxHeaderTypeDef rxHeader;  // ���ڴ洢����ͷ��Ϣ
  
  // ȷ�� CAN1 ��������
  if (hcan->Instance == CAN1)
  {
    // ��ȡ���յ�������
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, can_rx_data) == HAL_OK)
    {
      switch (rxHeader.StdId)
			{
				case 0x201:
					motor1 = decode_can_data(can_rx_data);
				  break;
				
				case 0x202:
					motor2 = decode_can_data(can_rx_data);
				  break;
				
				case 0x203:
					motor3 = decode_can_data(can_rx_data);
				  break;
				
				case 0x204:
					motor4 = decode_can_data(can_rx_data);
				  break;
				
				default:
					break;
			}
    }
  }
};
