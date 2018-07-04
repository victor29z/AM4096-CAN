#ifndef __CAN_USER_H__
#define __CAN_USER_H__

/* Micro defines
--------------------------------------------*/
#define CAN_BCST_ID	0x300
#define DATA_SPACE_BASE	0x800FC00
#define DATA_OFFSET_CANID	0
#define CAN_MSG_UPLOAD	0x01
#define CAN_MSG_SETID	0x02
#define CAN_MSG_SETEMY	0x03


/* Private prototypes
--------------------------------------------*/
void CAN_init(void);
void Set_Can_id(unsigned int id);
void Can_Msg_Process(void);
void Can_Send_Temp(unsigned int temp, unsigned char mag,unsigned int id);










#endif
