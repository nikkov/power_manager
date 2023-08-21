/*
	��� ������ IR-��������� ���� �:
	https://chipenable.ru/index.php/projects-avr/181-modul-priemnika-ik-signalov.html
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#ifndef F_CPU
#define F_CPU 128000.0f
#endif
#include <util/delay.h>
#define TIM_PRE  64.0f
/* �������� ������������ ������� 7,82 ���� � ������� */

/* ���������� ���, � �������� ��������� TSOP */
#define PDU_INPUT_PIN    2
/* ���������� ���, � �������� ��������� ��������� */
#define LED_OUTPUT_PIN    0
/* ���������� ���, � �������� ���������� ������ */
#define BUTTON_INPUT_PIN   5
/* ���������� ���, � �������� ��������� POWER_EN */
#define POWER_EN_OUTPUT_PIN    3
/* ���������� ���, � �������� ��������� POWER_OFF */
#define POWER_OFF_INPUT_PIN    4
/* ���������� ���, � �������� ���������� ���� ��������� ������� */
#define POWER_RELAY_OUTPUT_PIN   1

#define PDU_PORTX  PORTB
#define PDU_DDRX   DDRB

//������������ ��������
//#define START_IMP_TH     (uint8_t)((12.0f*F_CPU/1000.0f)/(TIM_PRE))
//#define START_IMP_MAX    (uint8_t)((15.0f*F_CPU/1000.0f)/(TIM_PRE))
//#define REPEAT_IMP_MIN    (uint8_t)((8.0f*F_CPU/1000.0f)/(TIM_PRE))
//#define BIT_IMP_MAX       (uint8_t)((3.0f*F_CPU/1000.0f)/(TIM_PRE))
//#define BIT_IMP_TH        (uint8_t)((1.5f*F_CPU/1000.0f)/(TIM_PRE))

#define START_IMP_TH     (uint8_t)25
#define START_IMP_MAX    (uint8_t)30
#define REPEAT_IMP_MIN    (uint8_t)16
#define BIT_IMP_MAX       (uint8_t)6
#define BIT_IMP_TH        (uint8_t)3

// ������ ����� packet,
// ������ ������ ����� - ����� � �������,
// ����� ���� - ���������� ��������
#define MAX_SIZE    4
// �������� ������
#define PACK_ADR    0
// �������� �������
#define PACK_COM    2
// �������� ����� ��������
//#define PACK_REPEAT 4


// ��������� ����������
typedef enum command_e {LED_OFF, LED_ON, LED_FAST_BLINK, LED_SLOW_BLINK} command_t;
// ���������, ������� ��������� curState
typedef enum state_pdu {PDU_IDLE, PDU_RECEIVE} state_pdu_t;
// ��������� � ������ ��������������� ������ ���
typedef enum state_programm { ST_EXIT, ST_PROG_WAIT, ST_PROG } state_programm_t;

// ��������� � ���������� ������
typedef enum state_main { ST_STANDBY, ST_POWER_ON, ST_WAIT_LOAD, ST_WORKING, ST_POWER_OFF, ST_WAIT_SHUTDOWN, ST_SHUTDOWN } state_main_t;

typedef enum button_code { CODE_NONE, CODE_PRESS, CODE_LONG } button_code_t;

// ������� ����� � ���������� ���������
typedef struct state_flag_struct
{
	uint8_t cmd_receive_ok : 1;
	uint8_t cmd_repeat : 1;
	uint8_t led_flag : 1;
} state_flag_t;

// ������� ����� PDU
static volatile state_flag_t stateFlag;
// ��������� PDU
static state_pdu_t curState = PDU_IDLE;

// �����, ��������� �� PDU
static uint8_t packet[MAX_SIZE];
volatile uint8_t  packetRepeat;

// ���� �������� ������������ ������� � ������� ���������
volatile uint8_t overflowTimerCounter = 0;
// ������� �������� �������� ������� �� ������ ���������� �� ���
volatile uint8_t currentTimerValue = 0;

// �������� ����� �������� ���������� ��������� � ����� ����������������
#define NUM_REPEAT_MIN  24
// ������ ��� ����������� ���� ������� �� ���
uint8_t command[1] EEMEM;

/* ��������� �������� ���, �������� � ����������� ���������� */
static inline void PDU_Handler(void)
{
	static uint8_t prevCounter = 0;
	static uint8_t currentCounter = 0;
	static uint8_t data = 0;
	static uint8_t countBit = 0;
	static uint8_t countByte = 0;
	uint8_t countersDiff;
	
	prevCounter = currentCounter;
	currentCounter = currentTimerValue;
	
	if (currentCounter > prevCounter)
	countersDiff = currentCounter - prevCounter;
	else
	countersDiff = currentCounter + (255 - prevCounter);
	
	switch(curState)
	{
		// ���� ��������� �������
		case PDU_IDLE:
		if (countersDiff < START_IMP_MAX)
		{
			if (countersDiff > START_IMP_TH)
			{
				// ��������� �������
				data = 0;
				countBit = 0;
				countByte = 0;
				packetRepeat = 0;
				curState = PDU_RECEIVE;
			}
			else
			{
				if((countersDiff > REPEAT_IMP_MIN) && (countersDiff < START_IMP_TH))
				{
					// ������� �������
					stateFlag.cmd_repeat = 1;
					packetRepeat++;
				}
			}
		}
		break;
		// ����� �������
		case PDU_RECEIVE:
		if (countersDiff < BIT_IMP_MAX) {
			if (countersDiff > BIT_IMP_TH) {
				// ������� 1
				data |=  0x80;
			}
			countBit++;
			if (countBit == 8) {
				packet[countByte] = data;
				countBit = 0;
				data = 0;
				countByte++;
				if (countByte >= (MAX_SIZE - 1)) {
					// ��������� ������� � ���������������� �����
					/*				   
					if((packet[0] ^ packet[1]) == 0xFF &&
					(packet[2] ^ packet[3]) == 0xFF) {
					state_flag.cmd_receive_ok = 1;
					}
					else
					state_flag.cmd_receive_ok = 0;
					*/
					stateFlag.cmd_receive_ok = 1;
					curState = PDU_IDLE;
					break;
				}
			}
			data = data >> 1;
		}
		else {
			packetRepeat = 0;
			curState = PDU_IDLE;
		}
		break;
		
		default:
		break;
	}
}
// ��������� ���������, �������� �� ��� ������� ��� 0
uint8_t PDU_GetCom(void)
{
	if (stateFlag.cmd_receive_ok)
	{
		stateFlag.cmd_receive_ok = 0;
		return packet[PACK_COM];
	}
	return 0;
}

// ��������� ���������� �������� ���������, �������� �� ��� �������
uint8_t PDU_GetNumRepeat(void)
{
	if (stateFlag.cmd_repeat)
	{
		stateFlag.cmd_repeat = 0;
		return packetRepeat;
	}
	return 0;
}

// ������� ����������� ��� ����� ���
uint8_t PDU_Led(command_t com)
{
	static uint8_t count = 0;
	static uint8_t period = 0;
	
	switch(com)
	{
		case LED_FAST_BLINK:
		{
			if (stateFlag.led_flag)
			{
				stateFlag.led_flag = 0;
				PDU_PORTX ^= (1 << LED_OUTPUT_PIN);
				++count;
			}
		}
		break;
		case LED_SLOW_BLINK:
		{
			if (stateFlag.led_flag)
			{
				stateFlag.led_flag = 0;
				++period;
				if(period == 6)
				{
					period = 0;
					PDU_PORTX ^= (1 << LED_OUTPUT_PIN);
				}
				++count;
			}
		}
		break;
		case LED_ON:
		{
			PDU_PORTX |= (1 << LED_OUTPUT_PIN);
		}
		break;
		case LED_OFF:
		{
			PDU_PORTX &= ~(1 << LED_OUTPUT_PIN);
			count = 0;
		}
		break;
	}

	return count;
}



// ���������� ���������� �� �������
ISR(TIMER0_OVF_vect)
{
	++overflowTimerCounter;
	stateFlag.led_flag = 1;
}

// ���������� ���������� �� ��������� ��
ISR(INT0_vect)
{
	currentTimerValue = TCNT0;
	PDU_Handler();
}

// ���������� ���������� �� ������, ����� ��� ������ �� ������
ISR(PCINT0_vect)
{
}


inline void start_timer()
{
	// ������ �������
	TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
}

inline void stop_timer()
{
	// ��������� ������� ����� ����������
	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
}


void init()
{
	stateFlag.cmd_receive_ok  = 0;
	stateFlag.cmd_repeat  = 0;
	stateFlag.led_flag = 0;
	
	// ����������� ������-����� � ������������� ���������� ��������
	PDU_DDRX  &= ~((1 << PDU_INPUT_PIN) | (1 << BUTTON_INPUT_PIN) | (1 << POWER_OFF_INPUT_PIN));
	PDU_PORTX |=    ((1 << PDU_INPUT_PIN) | (1 << BUTTON_INPUT_PIN) | (1 << POWER_OFF_INPUT_PIN));
	
	// ����������� �����-������ � ������������� 0
	PDU_DDRX  |=  (1 << LED_OUTPUT_PIN) | (1 << POWER_EN_OUTPUT_PIN) | (1 << POWER_RELAY_OUTPUT_PIN);
	PDU_PORTX &= ~((1 << LED_OUTPUT_PIN) | (1 << POWER_RELAY_OUTPUT_PIN));
	// ������ �� shutdown � 1
	PDU_PORTX |= (1 << POWER_EN_OUTPUT_PIN);
	
	start_timer();
	// ��������� ���������� �� ������������ �������
	TIMSK |= (1 << TOIE0);
	
	// ��������� ���������� �� ������������ ������ ��� ������ �� ������
	PCMSK |= (1 << BUTTON_INPUT_PIN) | (1 << POWER_OFF_INPUT_PIN);
	
	//������������ �� ������� ������
	MCUCR |= (1 << ISC01) | (0 << ISC00);
	//��������� ������� ���������� �� int0 �� ����� PDU � PCINT �� ������
	GIMSK |= (1 << INT0) | (1 << PCIE);
	
	ACSR  |=  (1 << ACD);
}

// ������� � ����� ���
void sleep()
{
	sleep_enable();
	stop_timer();
	sleep_cpu();
	start_timer();
	sleep_disable();
}

// ���������������� ������ ���
void set_command()
{
	state_programm_t state = ST_PROG_WAIT;
	uint8_t command_power;
	uint8_t data = 0;
	// ������� � ����� ����������������
	state = ST_PROG_WAIT;
	while(state != ST_EXIT)
	{
		data = PDU_GetCom();
		switch(state)
		{
			// ���������, ������������� �� ������������ ����� ������ � ����� ����������������. � ���� ������ ������ �� ������ ����������
			case ST_PROG_WAIT:
			if (data != 0)
			{
				PDU_Led(LED_OFF);
				state = ST_EXIT;
			}
			else
			// ���� ������ �� ���������� ���� ����� ���������, �� ��������� ������ � ��������� � ����� ����������������
			if (PDU_Led(LED_FAST_BLINK) > NUM_REPEAT_MIN)
			{
				PDU_Led(LED_ON);
				state = ST_PROG;
			}
			break;
			
			// ������� ��� �������
			case ST_PROG:
			if (data != 0)
			{
				command_power = data;
				if (command_power != eeprom_read_byte(&command[0]))
				eeprom_write_byte(&command[0], command_power);
				PDU_Led(LED_OFF);
				state = ST_EXIT;
			}
			break;
			
			default:
			state = ST_EXIT;
			break;
		}
	}
}


int main()
{
	uint8_t cmd = 0;
	
  	uint8_t current_time = 0;
	uint8_t period = 0;
 
	uint8_t current_sost_button = 0;
	uint8_t prev_sost_button = 0;
	uint8_t time_press_button = 0;
   
	uint8_t time_prev_timeout = 0;
	uint16_t timeout = 0;
	
	uint8_t time_sleep_enable = 0;
	uint8_t sleep_enable = 0;
	
	button_code_t button_sost = CODE_NONE;
	state_main_t state = ST_STANDBY;
	
	uint8_t command_power;
	
	init();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	
	// ���� ��� ��������� ������ ������� ������, �� ��������� � ����� ���������������� ������ ��
	if((PINB & (1 << BUTTON_INPUT_PIN)) == 0)
	set_command();
	
	// ������ ��� ������� ��� �� EEPROM
	command_power = eeprom_read_byte(&command[0]);
	
	// ������� ����
	while(1)
	{
		button_sost = CODE_NONE;
		if((PINB & (1 << BUTTON_INPUT_PIN)) == 0) // ������ ������
		{
			sleep_enable = 0;
			if(current_sost_button == 0)
			{
				// ����� ���������
				current_sost_button = 1;
				time_press_button = overflowTimerCounter;
			}
			else
			{
				// ������ ��� ��� ������
				current_time = overflowTimerCounter;
				if (current_time  >= time_press_button)
				period = current_time - time_press_button;
				else
				period = current_time + (255 - time_press_button);
				
				if(period > 38)
				button_sost = CODE_LONG;
				else
				if(period > 0)
				{
					if(prev_sost_button == 0)
					{
						prev_sost_button = 1;
						button_sost = CODE_PRESS;
					}
				}
			}
		}
		else
		{
			prev_sost_button = 0;
			current_sost_button = 0;
			cmd = PDU_GetCom();
			if(cmd != 0 && cmd == command_power)
			{
				button_sost = CODE_PRESS;
				sleep_enable = 0;
			}
			else
				sleep_enable = 1;
		}
		
		switch(state)
		{
			case ST_STANDBY:
				if(button_sost == CODE_PRESS) // ������ ������ ��� �������� ������� �� ���
					state = ST_POWER_ON;
			break;
			case ST_POWER_ON:
				// ������� ����
				PORTB |= 1 << POWER_RELAY_OUTPUT_PIN;
				state = ST_WAIT_LOAD;
				sleep_enable = 0;
				timeout = 0;
				time_prev_timeout = overflowTimerCounter;
			break;
			case ST_WAIT_LOAD:
				if(PINB & (1 << POWER_OFF_INPUT_PIN))
				{
					PDU_Led(LED_ON);
					state = ST_WORKING;
				}
				else
				{
					if(button_sost == CODE_LONG)
					state = ST_SHUTDOWN;
					else
					{
						current_time = overflowTimerCounter;
						if (current_time  >= time_prev_timeout)
							period = current_time - time_prev_timeout;
						else
							period = current_time + (255 - time_prev_timeout);
						
						time_prev_timeout = overflowTimerCounter;
						timeout += period;
						// ���� �� 2 ������ �� ��������� ������������� ��������, �� ��������� �������
						if(timeout > 940)
						{
							state = ST_SHUTDOWN;
						}
						else
							PDU_Led(LED_SLOW_BLINK);
					}
				}
				sleep_enable = 0;
			break;
			case ST_WORKING:
				if(button_sost == CODE_PRESS)				// ������ ������ ����������
				{
					state = ST_POWER_OFF;
				}
				if(!(PINB & (1 << POWER_OFF_INPUT_PIN)))		// ������ ������ POWER_OFF_INPUT_PIN, �.�. ������ � shutdown
				{
					state = ST_SHUTDOWN;
				}
			break;
			case ST_POWER_OFF:
				// ����� ������� �� shutdown ����������, �������� 0
				PORTB &= ~(1 << POWER_EN_OUTPUT_PIN);
				_delay_ms(50);
				// ������� ��������� �� ������� �� ��������� ���������� ������ ����������
				state = ST_WAIT_SHUTDOWN;
				sleep_enable = 0;
				timeout = 0;
				time_prev_timeout = overflowTimerCounter;
			break;
			case ST_WAIT_SHUTDOWN:
				if(button_sost == CODE_LONG ||  !(PINB & (1 << POWER_OFF_INPUT_PIN)))
				{
					state = ST_SHUTDOWN;
				}
				else
				{
					current_time = overflowTimerCounter;
					if (current_time  >= time_prev_timeout)
						period = current_time - time_prev_timeout;
					else
						period = current_time + (255 - time_prev_timeout);
					time_prev_timeout = overflowTimerCounter;
					timeout += period;
					// ���� �� 2 ������ �� ��������� ������������� ����������, �� ��������� �������
					if(timeout > 940)
					{
					   state = ST_SHUTDOWN;
					}
					else
						PDU_Led(LED_SLOW_BLINK);
				}
				// ������ ������ �� shutdown � 1
				PORTB |= (1 << POWER_EN_OUTPUT_PIN);
				sleep_enable = 0;
			break;
			case ST_SHUTDOWN:
				// �������� ����
				PORTB &= ~(1 << POWER_RELAY_OUTPUT_PIN);
				// ������� ���������
				PDU_Led(LED_OFF);
				// � �� �������� �������
				state = ST_STANDBY;
			break;
			default:
			break;
		}
		if(sleep_enable)
		{
			current_time = overflowTimerCounter;
			if (current_time  >= time_sleep_enable)
			period = current_time - time_sleep_enable;
			else
			period = current_time + (255 - time_sleep_enable);
			if(period > 15)
			{
				sleep();
				sleep_enable = 0;
				time_sleep_enable = overflowTimerCounter;
			}
		}
		else
			time_sleep_enable = overflowTimerCounter;
	}
	return 0;
}
