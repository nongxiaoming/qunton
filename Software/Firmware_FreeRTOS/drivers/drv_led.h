#ifndef __DRV_LED_H__
#define __DRV_LED_H__


#ifdef __cplusplus
extern "C" {
#endif

void LED_GPIO_Init(void);
void led_on(unsigned  int id);
void led_off(unsigned  int id);
	
#ifdef __cplusplus
}
#endif

#endif
