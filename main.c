// Packet structure (13 bytes)
// message[0]: DeviceID0
// message[1]: DeviceID1
// message[2]: DeviceID2
// message[3]: DeviceID3
// message[4]: DeviceID4
// message[5]: DeviceID5
// message[6]: DeviceID6
// message[7]: DeviceID7

// message[8]: Counter
// message[9]: Temp0
// message[10]: Temp1
// message[11]: Test
// message[12]: Relayed

//////////////////////////////////////////////////////////////

// Revision TN003: channel to 120, attempts to 3, report attempts in Counter (tx_attempts declared as global)
// Revision TN004: rebuilt as fresh project 10/3/17
// Revision TN005: added Pin0 wake-up for Tempv1 sensors


////////////////////// Includes and defines ///////////////////

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include "nrf51.h"
#include "nrf_delay.h"
#include "twi_master.h"
#include "nrf_gpio.h"

#define TMP102_ADDRESS 0x90 

#define LED_RED        5
#define LED_AMB        4
#define LED_GRN        3

#define SW_TEST   0





/////////////////////////// Globals ///////////////////////////

static uesb_payload_t tx_payload, rx_payload;
bool ackd;
bool awake;
static uint32_t retries;

// Packet structure constants
const int DeviceID0=0;
const int DeviceID1=1;
const int DeviceID2=2;
const int DeviceID3=3;
const int DeviceID4=4;
const int DeviceID5=5;
const int DeviceID6=6;
const int DeviceID7=7;

const int Counter=8;
const int Temp0=9;
const int Temp1=10;
const int Test=11;
const int Relayed=12;




/////////////////////////// Support routines /////////////////////

void uesb_event_handler()
{
    static uint32_t rf_interrupts;
		static uint32_t tx_attempts;
	
    uesb_get_clear_interrupts(&rf_interrupts);
    
    if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK)
    {   
		ackd=true;
		// flash green LED if switch 2 set to ON
		if (nrf_gpio_pin_read(SW_TEST)==0) {	
			nrf_gpio_pin_set(LED_GRN);
			nrf_delay_ms(25);
			nrf_gpio_pin_clear(LED_GRN);			
		}
	}
    
    if(rf_interrupts & UESB_INT_TX_FAILED_MSK)
    {
        uesb_flush_tx();
		// flash red LED if switch 2 set to ON
		if (nrf_gpio_pin_read(SW_TEST)==0) {	
			nrf_gpio_pin_set(LED_RED);
			nrf_delay_ms(25);
			nrf_gpio_pin_clear(LED_RED);
		}
    }
    
    if(rf_interrupts & UESB_INT_RX_DR_MSK)
    {
        uesb_read_rx_payload(&rx_payload);
        NRF_GPIO->OUTCLR = 0xFUL << 8;
        NRF_GPIO->OUTSET = (uint32_t)((rx_payload.data[2] & 0x0F) << 8);
    }
    
    uesb_get_tx_attempts(&tx_attempts);
    retries=tx_attempts;
		NRF_GPIO->OUTCLR = 0xFUL << 12;
    NRF_GPIO->OUTSET = (tx_attempts & 0x0F) << 12;

}

void RTC1_IRQHandler(void)
{
// This handler will be run after wakeup from system ON (RTC wakeup)
	if(NRF_RTC1->EVENTS_COMPARE[0])
	{
		NRF_RTC1->EVENTS_COMPARE[0] = 0;
		awake=true;
		NRF_RTC1->TASKS_CLEAR = 1;
	}
}

void GPIOTE_IRQHandler(void)
{
    // This handler will be run after wakeup from system ON (GPIO wakeup)
    if(NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;
				awake=true;
    }
}

/////////////////////////// Main  /////////////////////////////////

int main(void)
{
	
	// Declarations and initiations
	uint8_t write_buffer[3];
	uint8_t data_buffer[2];
	
	
			
	// Set LED pins to high drive
	NRF_GPIO->PIN_CNF[LED_RED] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
																| (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
																| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
																| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
																| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	NRF_GPIO->PIN_CNF[LED_AMB] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
																| (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
																| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
																| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
																| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	NRF_GPIO->PIN_CNF[LED_GRN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
																| (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
																| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
																| (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
																| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	
	// Set switches as inputs pulled high
	//nrf_gpio_cfg_input(SW_LED,NRF_GPIO_PIN_PULLUP);
	//nrf_gpio_cfg_input(SW_TEST,NRF_GPIO_PIN_PULLUP);
	
	// Configure SW_TEST with SENSE enabled so that CPU is enabled (exit System-On low power mode) when pressing Button 0
  nrf_gpio_cfg_sense_input(SW_TEST, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	
	// Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
  NVIC_EnableIRQ(GPIOTE_IRQn);
	
	// Put power up high for testing purposes
	uesb_set_tx_power(UESB_TX_POWER_4DBM);

	// Set receive address
	uint8_t rx_addr_p0[] = {0xD2, 0xF0, 0xF0, 0xF0, 0xF0};
	uint8_t rx_addr_p1[] = {0xE1, 0xF0, 0xF0, 0xF0, 0xF0};
	uint8_t rx_addr_p2   = 0x66;	

	// Start the high-frequency clock - we'll need it for the radio
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);	

	// Configure the radio
	uesb_config_t uesb_config       = UESB_DEFAULT_CONFIG;
	uesb_config.rf_channel          = 120;
	uesb_config.crc                 = UESB_CRC_16BIT;
	uesb_config.retransmit_count    = 3;
	uesb_config.retransmit_delay    = 500;
	uesb_config.dynamic_ack_enabled = true;
	uesb_config.protocol            = UESB_PROTOCOL_ESB_DPL;
	uesb_config.bitrate             = UESB_BITRATE_1MBPS;
	uesb_config.event_handler       = uesb_event_handler;
	
	uesb_init(&uesb_config);
	uesb_set_address(UESB_ADDRESS_PIPE0, rx_addr_p0);
	uesb_set_address(UESB_ADDRESS_PIPE1, rx_addr_p1);
	uesb_set_address(UESB_ADDRESS_PIPE2, &rx_addr_p2);

	// Load the payload with chip ID
	tx_payload.length  = 13;
	tx_payload.pipe    = 0;
	tx_payload.data[DeviceID0] = NRF_FICR->DEVICEID[0] & 255;
	tx_payload.data[DeviceID1] = (NRF_FICR->DEVICEID[0] >> 8) &255;
	tx_payload.data[DeviceID2] = (NRF_FICR->DEVICEID[0] >> 16) &255;;
	tx_payload.data[DeviceID3] = (NRF_FICR->DEVICEID[0] >> 24) &255;;

	tx_payload.data[DeviceID4] = NRF_FICR->DEVICEID[1] & 255;
	tx_payload.data[DeviceID5] = (NRF_FICR->DEVICEID[1] >> 8) &255;
	tx_payload.data[DeviceID6] = (NRF_FICR->DEVICEID[1] >> 16) &255;;
	tx_payload.data[DeviceID7] = (NRF_FICR->DEVICEID[1] >> 24) &255;;
	
	tx_payload.data[Relayed] = 0;
	
	// Configure RTC clock for SYSTEM ON sleep
	
	// Use internal 32kHz RC
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos;
	
	// Start the 32 kHz clock, and wait for the start up to complete
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while(NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
	
	// Configure the RTC to run at 5 second intervals, and make sure COMPARE0 generates an interrupt (this will be the wakeup source)
	NRF_RTC1->PRESCALER = 0;
	NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk; 
	NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk; 
	NRF_RTC1->CC[0] = 32768;
	NVIC_EnableIRQ(RTC1_IRQn);
					
	// Configure the RAM retention parameters
	NRF_POWER->RAMON = POWER_RAMON_ONRAM0_RAM0On   << POWER_RAMON_ONRAM0_Pos
									 | POWER_RAMON_ONRAM1_RAM1Off  << POWER_RAMON_ONRAM1_Pos
									 | POWER_RAMON_OFFRAM0_RAM0Off << POWER_RAMON_OFFRAM0_Pos
									 | POWER_RAMON_OFFRAM1_RAM1Off << POWER_RAMON_OFFRAM1_Pos;		 


	// Initialise I2C
	twi_master_init(); 

	// Prepare data that will put TMP sensor into shutdown
	write_buffer[0]=1; // i.e. Configuration register
	write_buffer[1]=1; // MSB? 1=SHUTDOWN
	write_buffer[2]=0; // LSB? 

	if(twi_master_transfer(TMP102_ADDRESS , write_buffer, 3, TWI_ISSUE_STOP)){
	}
	
	// Turn I2C peripheral off (it uses the high-frequency clock, which we will need to turn off later)
	NRF_TWI1->ENABLE=0;
	NRF_TWI1->ENABLE          = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
	
	// Prepare data that will trigger a one-shot conversion from the tmp sensor
	write_buffer[0]=1; // i.e. Configuration register
	write_buffer[1]=128; // MSB? 128=OS bit
	write_buffer[2]=0; // LSB? 
	
	
///////////////////////// Main loop //////////////////////////////////

	while (true)
    {
        ackd=false;

				// Set switches as inputs pulled high
				//nrf_gpio_cfg_input(SW_LED,NRF_GPIO_PIN_PULLUP);
				//nrf_gpio_cfg_input(SW_TEST,NRF_GPIO_PIN_PULLUP);

			
			
				// -------------- SENSE ---------------------

				twi_master_init(); 		
				// Trigger a one-shot conversion from the temperature sensor
				write_buffer[0]=1; // i.e. Configuration register
				write_buffer[1]=129; // MSB 128=OS bit, 1=SD bit
				write_buffer[2]=0; // LSB 	
				if(twi_master_transfer(TMP102_ADDRESS , write_buffer, 3, TWI_ISSUE_STOP)){
				}			
	
				// Wait for OS bit to be re-set to 1 to signal end of conversion
				data_buffer[0]=0;
				while (data_buffer[0] && 128!=128) {
					if (twi_master_transfer(TMP102_ADDRESS | TWI_READ_BIT, data_buffer, 2, TWI_ISSUE_STOP)) {
					}				
				}
				
				// Switch to temperature register (from config register)
				write_buffer[0]=0; // i.e. Temperature register
				write_buffer[1]=0; // MSB? 
				write_buffer[2]=0; // LSB? 	
				if(twi_master_transfer(TMP102_ADDRESS , write_buffer, 3, TWI_ISSUE_STOP)){
				}			
				
				// Read the temperature and place in the payload ready to send
				if (twi_master_transfer(TMP102_ADDRESS | TWI_READ_BIT, data_buffer, 2, TWI_ISSUE_STOP)) {
							tx_payload.data[Temp0] = data_buffer[0];		
							tx_payload.data[Temp1] = data_buffer[1];				
				}

				// Prepare data that will put TMP sensor into shutdown
				write_buffer[0]=1; // i.e. Configuration register
				write_buffer[1]=1; // MSB? 1=SHUTDOWN
				write_buffer[2]=0; // LSB? 

				if(twi_master_transfer(TMP102_ADDRESS , write_buffer, 3, TWI_ISSUE_STOP)){
				}
				
				// Disable TWI ready for sleep

				NRF_TWI1->ENABLE= TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
				
				// -------------- TRANSMIT ---------------------
			
				if(uesb_write_tx_payload(&tx_payload) == UESB_SUCCESS)
				{
						tx_payload.data[Counter]++;
						tx_payload.data[Counter]=(uint8_t)retries;
				}

			
				nrf_delay_ms(10);

				// set switches to NOPULL before going to sleep
				//nrf_gpio_cfg_input(SW_LED, NRF_GPIO_PIN_NOPULL);
				//nrf_gpio_cfg_input(SW_TEST, NRF_GPIO_PIN_NOPULL);

				
				// --------------- SLEEP ------------------------
				
				// define length of sleep according to position of SW_TEST switch

				NRF_RTC1->PRESCALER = 0;
				NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk; 
				NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk; 
				
				nrf_gpio_cfg_input(SW_TEST,NRF_GPIO_PIN_PULLUP); // we need to pull up before testing state of pin
				if (nrf_gpio_pin_read(SW_TEST)==0) {	
					
						// we're in test mode......
						nrf_gpio_cfg_input(SW_TEST,NRF_GPIO_PIN_NOPULL); // set pin as standard input, no pull-up, so it doesn't interrupt sleep
						NRF_RTC1->CC[0] = 32768; // 1 second
						tx_payload.data[Test] = 1;  // set test flag in payload
				}
				else
				{		
						// we're not in test mode ....
	//					nrf_gpio_cfg_sense_input(SW_TEST, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW); // set pin as sense so that it will interrupt if pressed
						nrf_gpio_cfg_sense_input(SW_TEST, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW); // set pin as sense so that it will interrupt if pressed

						NRF_RTC1->CC[0] = 32768*60*5;  // 5 minutes
						tx_payload.data[Test] = 0;  // test flag in payload not set
				}				

				NVIC_EnableIRQ(RTC1_IRQn);				

				
				// Debug option: set switch pins to float? NRF_GPIO_PIN_NOPULL
				
				
				// Start the RTC timer
				NRF_RTC1->TASKS_START = 1;

				// shut down the high-frequency clock
				NRF_CLOCK->TASKS_HFCLKSTOP = 1;
				
				awake=false;
				while(!awake)
				{
					// Enter System ON sleep mode
					__WFE();  
					// Make sure any pending events are cleared
					__SEV();
					__WFE();                
				}
					
				// Stop and clear the RTC timer
				NRF_RTC1->TASKS_STOP = 1;
				NRF_RTC1->TASKS_CLEAR = 1;	

				// Restart the hi-frequency clock
				NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
				NRF_CLOCK->TASKS_HFCLKSTART = 1;
				while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);	
				
				
    }
}

