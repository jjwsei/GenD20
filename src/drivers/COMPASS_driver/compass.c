#include <asf.h>
#include "compass.h"

struct i2c_master_module i2c_master_mod;

int16_t Xaxis = 0;
int16_t Yaxis = 0;
int16_t Zaxis = 0;

void CompassInit(struct i2c_master_module i2c_mast)
{
        uint8_t retry_count = 0;
        struct i2c_master_packet packet;
	uint8_t data[2] = {0};
	i2c_master_mod = i2c_mast;

        data[0] = CONFIG_REGISTER_A;
        data[1] = 0x70;

        packet.address = COMPASS_ADDRESS;
        packet.data_length = 2;
        packet.data = data;
 
        while(i2c_master_write_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
        {
           if(++retry_count > 200)
              break;
        }

        data[0] = CONFIG_REGISTER_B;
        data[1] = 0xA0;

        packet.data = data;
 
        retry_count = 0;
        while(i2c_master_write_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
        {
           if(++retry_count > 200)
              break;
        }

        data[0] = MODE_REGISTER;
        data[1] = 0;

        packet.data = data;
 
        retry_count = 0;
        while(i2c_master_write_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
        {
           if(++retry_count > 200)
              break;
        }
}

int16_t CompassGetXData(void)
{
        return Xaxis;
}

int16_t CompassGetYData(void)
{
        return Yaxis;
}

int16_t CompassGetZData(void)
{
        return Zaxis;
}

void CompassGetData(void)
{
        uint8_t retry_count = 0;
        struct i2c_master_packet packet;
	uint8_t data[2] = {0};
        bool dataReady = 0;
        bool timeout_flag = 0;

        // look for data ready
        data[0] = STATUS_REGISTER;
        packet.address = COMPASS_ADDRESS;
        packet.data_length = 1;
        packet.data = data; 

        while(i2c_master_write_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
        {
           if(++retry_count > 200)
           {
              timeout_flag = 1;
              break;
           }
        }

        while(dataReady == 0 && timeout_flag == 0)
        {
           retry_count = 0;
           while(i2c_master_read_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
           {
              if(++retry_count > 200)
              {
                 timeout_flag = 1;
                 break;
              }
              delay_ms(1);
           }
           dataReady = packet.data[0] & 0x01;
         } 

        if(timeout_flag == 0)
        {
           data[0] = DATA_X_MSB_REGISTER;
           packet.data = data; 

           retry_count = 0;
           while(i2c_master_write_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
           {
              if(++retry_count > 200)
              {
                 timeout_flag = 1;
                 break;
              }
           }

           packet.data_length = 6;
           retry_count = 0;
           while(i2c_master_read_packet_wait(&i2c_master_mod, &packet) != STATUS_OK)
           {
              if(++retry_count > 200)
                 break;
           }

	   Xaxis = packet.data[1];
           Xaxis <<= 8;
           Xaxis |= packet.data[0];
           Yaxis = packet.data[3];
           Yaxis <<= 8;
           Yaxis |= packet.data[2];
           Zaxis = packet.data[5];
           Zaxis <<= 8;
           Zaxis |= packet.data[4];
        }
        else
        {
	   Xaxis = 0;
           Yaxis = 0;
           Zaxis = 0;
        }
}
