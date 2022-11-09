#ifndef _BUTTON_H_
#define _BUTTON_H_

#define BUTTON_ONOFF_PIN                GPIO_NUM_25
#define BUTTON_BOOT_PIN                 GPIO_NUM_0
#define TIME_DOWN_SET                   3000

void button_task(void *param);

#endif


