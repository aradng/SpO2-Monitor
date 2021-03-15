#include <stdint.h>
#include "stm32f10x.h"			//change include to match mcu

#define	RES_pin		15
#define	PD_pin		14
#define	REF_pin		12
#define GAIN_pin	11
#define	DCEN_pin	10

class dac
{
	typedef enum{
		normal	= 0 ,
		imp1k	= 1 ,
		imp100k	= 2 ,
		tri	= 3
	}pd_mode;
public:
	GPIO_TypeDef*	GPIO_SS;
	GPIO_TypeDef*	GPIO_SCK;
	GPIO_TypeDef*	GPIO_SDI;
	uint16_t Pin_SS;
	uint16_t Pin_SCK;
	uint16_t Pin_SDI;
	uint16_t value = 0;
	bool gain	= 0;
	bool ref	= 0;
	bool dcen	= 0;
	uint8_t pd = normal;
	void update(void);
	void reset(void);
	dac(GPIO_TypeDef* GPIO_SS, uint16_t  Pin_SS, GPIO_TypeDef* GPIO_SCK , uint16_t  Pin_SCK, GPIO_TypeDef* GPIO_SDI , uint16_t  Pin_SDI):
		GPIO_SS(GPIO_SS),
		GPIO_SCK(GPIO_SCK),
		GPIO_SDI(GPIO_SDI),
		Pin_SS(Pin_SS),
		Pin_SCK(Pin_SCK),
		Pin_SDI(Pin_SDI)
	{}
	private :
		void spi_w(uint8_t byte);
		void dac_w(uint8_t cmd , uint16_t data);
		void dac_wu(uint16_t data);
		void dac_ctrl(uint8_t PD ,uint8_t REF ,uint8_t GAIN ,uint8_t DCEN);
};