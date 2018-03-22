/*
  Generic SAMD20 Project 
*/

#include <asf.h>
#include <math.h>
#include <stdio.h>
#include "adc_driver.h"
#include "lcd.h"
#include "compass.h"

#define LED_0_PIN 		PIN_PA14
#define BUTTON_0_PIN 		PIN_PA15
#define BUTTON_0_EIC_LINE       15
#define MAX_RX_BUFFER_LENGTH   	10
#define MAX_BUFFER_LENGTH	255
#define DATA_LENGTH 		8
#define SLAVE_ADDRESS 		0x12
#define BUF_LENGTH 		20
#define SLAVE_SELECT_PIN 	PIN_PA05
#define PWM_MODULE		TC6

#define TWI_SDA_PIN_MUX         PINMUX_PA08D_SERCOM2_PAD0   
#define TWI_SCL_PIN_MUX         PINMUX_PA09D_SERCOM2_PAD1;  

typedef enum {
   IDLE,
   GOT_ZERO,
   GOT_AA,
   GOT_LENGTH
} serial_states;

// USART
void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);
void configure_usart(void);
void configure_usart_callbacks(void);
uint8_t serial_state = IDLE;
uint16_t data_length = 0;
uint8_t index_count = 0;
uint8_t message[100] = {0};
uint8_t length_byte = 1;

// EXINT
void configure_extint_channel(void);
void configure_extint_callbacks(void);
void extint_detection_callback(void);

// I2C
void i2c_write_complete_callback(struct i2c_master_module *const module);
void configure_i2c(void);
void configure_i2c_callbacks(void);

// SPI
void configure_spi_master_callbacks(void);
void configure_spi_master(void);
static void callback_spi_master( struct spi_module *const module);

// TC
void configure_tc(void);
void configure_tc_callbacks(void);
void tc_callback_to_change_duty_cycle(struct tc_module *const module_inst);

// RTC
void rtc_match_callback(void);
void configure_rtc_callbacks(void);
void configure_rtc_calendar(void);

// EEPROM
void configure_eeprom(void);

// BOD
static void configure_bod(void);

struct usart_module usart_instance;

bool data_ready_flag = 0;
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
static int8_t wr_buffer[256] = {0};
static uint8_t wr_buffer_reversed[DATA_LENGTH] = {
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};
static uint8_t rd_buffer[DATA_LENGTH];
struct i2c_master_packet wr_packet;
struct i2c_master_packet rd_packet;
struct i2c_master_module i2c_master_instance;

static uint8_t spi_wr_buffer[BUF_LENGTH] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13
};
static uint8_t spi_rd_buffer[BUF_LENGTH];
struct spi_module spi_master_instance;
struct spi_slave_inst slave;
volatile bool transrev_complete_spi_master = false;

struct tc_module tc_instance;

struct rtc_module rtc_instance;
struct rtc_calendar_alarm_time alarm;
uint8_t rtc_alarm_flag = 0;

bool button_flag = 0;

int main (void)
{
   struct port_config config_port_pin;

   struct rtc_calendar_time prev_time;
   struct rtc_calendar_time time;
   rtc_calendar_get_time_defaults(&time);
   time.year   = 2018;
   time.month  = 3;
   time.day    = 8;
   time.hour   = 7;
   time.minute = 12;
   time.second = 50;
   prev_time = time;

   uint8_t page_data[EEPROM_PAGE_SIZE];

   system_init();
   delay_init();

   ADCCreate();
	
   port_get_config_defaults(&config_port_pin);
   config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
   port_pin_set_config(LED_0_PIN, &config_port_pin);
	
   configure_usart();
   configure_usart_callbacks();

   configure_extint_channel();
   configure_extint_callbacks();

   //configure_i2c();
   //configure_i2c_callbacks();

   configure_spi_master();
   configure_spi_master_callbacks();

   configure_tc();
   configure_tc_callbacks();

   configure_rtc_calendar();
   configure_rtc_callbacks();
   rtc_calendar_set_time(&rtc_instance, &time);

//   configure_eeprom();

   configure_bod();
/*
   eeprom_emulator_read_page(0, page_data);
   page_data[0] = !page_data[0];
   eeprom_emulator_write_page(0, page_data);
   eeprom_emulator_commit_page_buffer();
   page_data[1]=0x1;
   eeprom_emulator_write_page(0, page_data);
*/
   system_interrupt_enable_global();

   //CompassInit(i2c_master_instance);

   //LCDSetCursor(0,ROW1);
   //LCDClear();

   uint8_t buff[8] = {0, 0xAA, 0, 0x03, 0x04, 0, 0x07, 0xF2};

   while(1)
   {
        
	//usart_read_buffer_job(&usart_instance,
	//	(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
  
	usart_read_job(&usart_instance, (uint8_t *)rx_buffer);
        if(data_ready_flag == 1)
        {
           data_ready_flag = 0;
           switch(serial_state)
           {
              case IDLE:
                 if(rx_buffer[0] == 0x00)
                   serial_state = GOT_ZERO;
                 break;
              case GOT_ZERO:
                 if(rx_buffer[0] == 0xAA)
                   serial_state = GOT_AA;
                   data_length = 0;
                 break;
              case GOT_AA:
                 if(length_byte == 1) 
                 {
                    ++length_byte;
                    data_length = rx_buffer[0] << 8;
                 }
                 else
                 {
                    length_byte = 1;
                    data_length |= rx_buffer[0];
                    for(index_count = 0; index_count < data_length; index_count++)
                      message[index_count] = 0;
                    index_count = 0;
                    serial_state = GOT_LENGTH;
                 }
                 break;
              case GOT_LENGTH:
                 message[index_count] = rx_buffer[0];
                 if(index_count == data_length)
                   serial_state = IDLE;
                 else
                   ++index_count;
                 break;
              default:
                 break;
           }
        }      
        if(button_flag == 1)
        {
           button_flag = 0;
           //LCDClear();
           usart_write_buffer_job(&usart_instance, buff, 8);
        }
/*        rtc_calendar_get_time(&rtc_instance, &time);
        if(time.second != prev_time.second)
        {
	   LCDSetCursor(0,ROW1);
           LCDprintf("%d:%d:%d", time.hour, time.minute, time.second);
           prev_time = time;
        }

	usart_read_buffer_job(&usart_instance,
			(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);

        if(rtc_alarm_flag == 1)
        {
            rtc_alarm_flag = 0;
            sprintf(wr_buffer, "ADC = %u at %d:%d:%d\n\r", ADCGetSample(), time.hour, time.minute, time.second);

            CompassGetData();
            int16_t magneticX = (CompassGetXData());// / 390) * 100;
            int16_t magneticY = (CompassGetYData());// / 390) * 100;
            int16_t magneticZ = (CompassGetZData());// / 390) * 100;
            sprintf(wr_buffer, "%d ,%d, %d\n\r",  magneticX, magneticY, magneticZ);
            usart_write_buffer_job(&usart_instance, wr_buffer, strlen(wr_buffer));

	    int16_t heading = atan2(magneticY, magneticX);
            sprintf(wr_buffer, "Heading = %d\n\r", (heading * 100));
            usart_write_buffer_job(&usart_instance, wr_buffer, strlen(wr_buffer));
        }
*/
/*	spi_select_slave(&spi_master_instance, &slave, true);
	spi_transceive_buffer_job(&spi_master_instance, spi_wr_buffer,spi_rd_buffer,BUF_LENGTH);
	while (!transrev_complete_spi_master) {
	}
	transrev_complete_spi_master = false;
	spi_select_slave(&spi_master_instance, &slave, false);
*/
   }
}

// **** USART ****
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate    = 115200;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3 ;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;

	while (usart_init(&usart_instance,
			SERCOM4, &config_usart) != STATUS_OK) {
	}

	usart_enable(&usart_instance);
}

// **** EXTINT ****
void configure_extint_channel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = PIN_PA15;
	config_extint_chan.gpio_pin_mux       = PINMUX_PA15A_EIC_EXTINT15;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &config_extint_chan);
}

// **** I2C ****
void configure_i2c(void)
{
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	config_i2c_master.buffer_timeout = 65535;
        config_i2c_master.pinmux_pad0 = TWI_SDA_PIN_MUX;  
        config_i2c_master.pinmux_pad1 = TWI_SCL_PIN_MUX;  

	while(i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master)     \
			!= STATUS_OK);

	i2c_master_enable(&i2c_master_instance);

        //LCDInit(20, 4, LCD_5x8DOTS, i2c_master_instance);
}

// **** SPI ****
void configure_spi_master(void)
{
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;
	spi_slave_inst_get_config_defaults(&slave_dev_config);

	slave_dev_config.ss_pin = SLAVE_SELECT_PIN;
	spi_attach_slave(&slave, &slave_dev_config);
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_E;
	config_spi_master.pinmux_pad0 = PINMUX_PA04D_SERCOM0_PAD0;
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	config_spi_master.pinmux_pad2 = PINMUX_PA06D_SERCOM0_PAD2;
	config_spi_master.pinmux_pad3 = PINMUX_PA07D_SERCOM0_PAD3;

	spi_init(&spi_master_instance, SERCOM0, &config_spi_master);

	spi_enable(&spi_master_instance);
}

// **** TC ****
void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);

	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc.counter_16_bit.compare_capture_channel[0] = 0xFFFF;

	config_tc.pwm_channel[0].enabled = true;
	config_tc.pwm_channel[0].pin_out = PIN_PB02F_TC6_WO0;
	config_tc.pwm_channel[0].pin_mux = MUX_PB02F_TC6_WO0;

	tc_init(&tc_instance, PWM_MODULE, &config_tc);

	tc_enable(&tc_instance);
}

// **** RTC ****
void configure_rtc_calendar(void)
{
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);

	alarm.time.year      = 2018;
	alarm.time.month     = 3;
	alarm.time.day       = 8;
	alarm.time.hour      = 7;
	alarm.time.minute    = 13;
	alarm.time.second    = 0;

	config_rtc_calendar.clock_24h = true;
	config_rtc_calendar.alarm[0].time = alarm.time;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_YEAR;

	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);

	rtc_calendar_enable(&rtc_instance);
}

// **** EEPROM ****
void configure_eeprom(void)
{
	enum status_code error_code = eeprom_emulator_init();

	if (error_code == STATUS_ERR_NO_MEMORY) {
		while (true) {
                    delay_ms(100);
                    port_pin_toggle_output_level(LED_0_PIN);
                    printf("Error\n\r");
		}
	}
	else if (error_code != STATUS_OK) {
		eeprom_emulator_erase_memory();
		eeprom_emulator_init();
	}
}

// **** BOD ****
static void configure_bod(void)
{
	struct bod_config config_bod33;
	bod_get_config_defaults(&config_bod33);
	config_bod33.action = BOD_ACTION_INTERRUPT;
	/* BOD33 threshold level is about 3.2V */
	config_bod33.level = 48;
	bod_set_config(BOD_BOD33, &config_bod33);
	bod_enable(BOD_BOD33);

	SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
}

// Callbacks
void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
			usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
			usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

void configure_extint_callbacks(void)
{
	extint_register_callback(extint_detection_callback,
			BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
}

void configure_i2c_callbacks(void)
{
	i2c_master_register_callback(&i2c_master_instance, i2c_write_complete_callback,
			I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	i2c_master_enable_callback(&i2c_master_instance,
			I2C_MASTER_CALLBACK_WRITE_COMPLETE);
}

void configure_spi_master_callbacks(void)
{
	spi_register_callback(&spi_master_instance, callback_spi_master,
			SPI_CALLBACK_BUFFER_TRANSCEIVED);
	spi_enable_callback(&spi_master_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
}

void configure_tc_callbacks(void)
{
	tc_register_callback(
			&tc_instance,
			tc_callback_to_change_duty_cycle,
			TC_CALLBACK_CC_CHANNEL0);

	tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

void configure_rtc_callbacks(void)
{
	rtc_calendar_register_callback(
			&rtc_instance, rtc_match_callback, RTC_CALENDAR_CALLBACK_ALARM_0);
	rtc_calendar_enable_callback(&rtc_instance, RTC_CALENDAR_CALLBACK_ALARM_0);
}



void usart_read_callback(struct usart_module *const usart_module)
{
        data_ready_flag = 1;
}

void usart_write_callback(struct usart_module *const usart_module)
{

}

void extint_detection_callback(void)
{
	bool pin_state = port_pin_get_input_level(BUTTON_0_PIN);
	port_pin_set_output_level(LED_0_PIN, pin_state);
        button_flag = 1;
}

void i2c_write_complete_callback(
		struct i2c_master_module *const module)
{
	i2c_master_read_packet_job(&i2c_master_instance,&rd_packet);
}

static void callback_spi_master( struct spi_module *const module)
{
	transrev_complete_spi_master = true;
}

void tc_callback_to_change_duty_cycle(
		struct tc_module *const module_inst)
{
	static uint16_t i = 0;

	i += 128;
	tc_set_compare_value(module_inst, TC_COMPARE_CAPTURE_CHANNEL_0, i + 1);
}

void rtc_match_callback(void)
{
	alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;

	alarm.time.second += 1;
	alarm.time.second = alarm.time.second % 60;

	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);
         
        rtc_alarm_flag = 1;
}

void SYSCTRL_Handler(void)
{
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET;
		eeprom_emulator_commit_page_buffer();
	}
}

