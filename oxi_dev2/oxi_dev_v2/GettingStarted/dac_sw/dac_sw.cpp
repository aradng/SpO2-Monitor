#include "dac_sw.h"

//spi send bytes
void dac::spi_w(uint8_t byte)
{
  for (int i = 0x80; i != 0x00; i >>= 1) 
  {
  	GPIO_ResetBits(this->GPIO_SCK , this->Pin_SCK);
  	GPIO_WriteBit(this->GPIO_SDI, this->Pin_SDI, byte & i?Bit_SET:Bit_RESET);
  	GPIO_SetBits(this->GPIO_SCK , this->Pin_SCK);
  }
}

//send command
void dac::dac_w(uint8_t cmd , uint16_t data)
{
  uint8_t buf[3];
  buf[0] = (data >> 12) | (cmd << 4);
  buf[1] = (data >> 4);
  buf[2] = ((data << 4) & 0xF0);
  for(int i = 0 ; i < 3 ; i++)
    spi_w(buf[i]);
}

//write and update
void dac::dac_wu(uint16_t data)
{
  dac_w(0b0011 , data);
}

//control commands
void dac::dac_ctrl(uint8_t PD ,uint8_t REF ,uint8_t GAIN ,uint8_t DCEN)
{
  dac_w(0b0100 , PD << PD_pin | REF << REF_pin | GAIN << GAIN_pin | DCEN << DCEN_pin );
}

//update dac
void dac::update(void)
{
	GPIO_ResetBits(this->GPIO_SS , this->Pin_SS);
	dac_ctrl(this->pd , this->ref , this->gain , this->dcen);
	GPIO_SetBits(this->GPIO_SS , this->Pin_SS);

	GPIO_ResetBits(this->GPIO_SS , this->Pin_SS);			//test if ss cycle is needed
	dac_wu(this->value);
	GPIO_SetBits(this->GPIO_SS , this->Pin_SS);
}

//reset
void dac::reset(void)
{
  dac_w(0b0100 , 1 << RES_pin);
}