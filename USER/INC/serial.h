#ifndef __SERIAL_H__
#define __SERIAL_H__

#define SERIAL_BUFF_MAX	100

#define CONSOLE_STAT_IDLE	0x00
#define CONSOLE_STAT_MENU	0x01
#define CONSOLE_STAT_EMI	0x02
#define CONSOLE_STAT_TEMP	0x03




void Show_Menu(void);
void Console(void);


#endif
