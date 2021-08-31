/* pump_core.c

This file provides the core operations for the PMS132 peristaltic pump.

The system is governed by a state machine where the state is represented as
different screen pages on the LCD. On each page, a sub-state machine is implemented
that allows the user to switch modes. 

The screen pages include: 
1. System State, 
2. Flow Rate,
3. Volume, 
4. Flow/Volume Mode Selection, 
5. Config: uL / rev, 
6. Config: steps / rev,
7. Return to System State.

The modes are: 
1. Menu Mode, 
2. Edit Mode,
3. Value Change Mode. 

Menu Mode allows the user to change pages or start the pump.
Edit Mode allows the user to select which digit/property to change in the menu.
Value Change Mode allows the user to increment or decrement the selected digit.


NOTE: 

	Do to ROM limitations, the stepper timing is based off of an 8-bit timer instead
	of the 11-bit PWM generator in order to allow for EEPROM usage. This reduces the 
	accuracy in velocity and may lead to issues at very high RPM. These potential
	velocity issues can	largely be mitigated by using larger ID tubing. EEPROM usage 
	was determined to be more valuable for the quality of life of the user.

THIS IMPLEMENTATION DOES NOT IMPACT THE PRECISION OF VOLUME DISPENSED - ONLY VELOCITY. 


ROM Used : 
RAM Used : 

This software is licensed under GPLv3 <http://www.gnu.org/licenses/>.
Any modifications or distributions have to be licensed under GPLv3.
No warranty of any kind and copyright holders cannot be held liable.
Licensees cannot remove copyright notices.

Copyright (c) 2021 Robert R. Puccinelli
*/

#include "../Padauk-Peripherals/system_settings.h"
#include "../Padauk-Peripherals/pdk_lcd.h"
#include "../Padauk-Peripherals/pdk_math.h"
#include "../Padauk-Peripherals/pdk_stepper.h"
#include "../Padauk-Peripherals/pdk_button.h"
#include "../Padauk-Peripherals/pdk_eeprom.h"


//=====================//
// HARDWARE PARAMETERS //
//=====================//

// Button pin assignments
#DEFINE active_inputs      button_active_b
#DEFINE start_button       PB.0
#DEFINE select_button      PB.1
#DEFINE rotary_input1      PB.2
#DEFINE rotary_input2      PB.3

// Default values on first initialization
#DEFINE def_steps_per_rev  800
#DEFINE def_ul_per_rev     230
#DEFINE def_ul_per_min     500
#DEFINE def_volume         500
#DEFINE def_direction      1

// EEPROM addresses for settings
#DEFINE ADDR_SAVED         0x0
#DEFINE ADDR_STEPS_REV     0x4
#DEFINE ADDR_UNITS_REV     0x8
#DEFINE ADDR_VOLUME        0x12
#DEFINE ADDR_VELOCITY      0x16
#DEFINE ADDR_DIR           0x20


//====================//
// SYSTEM DEFINITIONS //
//====================//

// LCD Command / Location Definitions
LCD_CURSOR_ON  => LCD_DISP_F + LCD_DISP_ON + LCD_DISP_CURSOR_ON
LCD_CURSOR_OFF => LCD_DISP_F + LCD_DISP_ON + LCD_DISP_CURSOR_OFF
REV_ENTRY      => LCD_ENTRY_F | LCD_ENTRY_DEC_DDRAM | LCD_ENTRY_DDRAM_SHIFT
FWD_ENTRY      => LCD_ENTRY_F | LCD_ENTRY_INC_DDRAM | LCD_ENTRY_DDRAM_SHIFT
LCD_END        => LCD_WIDTH + LCD_L2 - 1
RETURN_COL     => LCD_WIDTH - 1

// EEPROM first save
#DEFINE EEPROM_INIT_VAL 132

// Enumerations for state machine, mode, and eeprom operations
ENUM {MENU_MODE, EDIT_MODE, VALUE_MODE};
ENUM {HOME_PAGE, FLOW_PAGE, VOL_PAGE, MODE_PAGE, UNITS_PAGE, EXIT_PAGE};
ENUM {INIT, STEPS_REV, UNITS_REV, VOL, VEL, DIR};


//==================//
// SYSTEM VARIABLES //
//==================//

// General Purpose
STATIC WORD  temp_data     = 0;
STATIC DWORD temp_data2    = 0;

// State machine
STATIC BYTE  curr_state    = MENU_MODE;
STATIC BYTE  next_state    = MENU_MODE;
STATIC BYTE  curr_screen   = HOME_PAGE;
STATIC BYTE  next_screen   = HOME_PAGE;

// LCD
STATIC BYTE  col_index     = 0;
STATIC BYTE  col_data_s    = 0;
STATIC DWORD input_data    = 0;
STATIC BYTE  output_char   = 0;
STATIC BYTE  line_buffer   [LCD_WIDTH];

// EEPROM
STATIC BYTE  eeprom_buff   [5];

// Flags
STATIC BYTE  pump_flags     = 0;
STATIC BIT   start_flag     : pump_flags.?;
STATIC BIT   select_flag    : pump_flags.?;
STATIC BIT   shift_flag     : pump_flags.?;
STATIC BIT   shift_r_flag   : pump_flags.?;
STATIC BIT   update_display : pump_flags.?;
STATIC BIT   dir_sign       : pump_flags.?;
STATIC BIT   init_flag      : pump_flags.?;


//==================//
// STATIC FUNCTIONS //
//==================//

// LCD OPERATIONS
static void Display_Digit(void)
{
	switch (output_char)
	{
		case 1  : output_char = LCD_1;
				  break;
		case 2  : output_char = LCD_2;
				  break;
		case 3  : output_char = LCD_3;
				  break;
		case 4  : output_char = LCD_4;
				  break;
		case 5  : output_char = LCD_5;
				  break;
		case 6  : output_char = LCD_6;
				  break;
		case 7  : output_char = LCD_7;
				  break;
		case 8  : output_char = LCD_8;
				  break;
		case 9  : output_char = LCD_9;
				  break;
		default : output_char = LCD_0;
				  break;
	}
	lcd_trx_byte = output_char;
	LCD_Write_Byte();
}


static void Display_Digits(void)
{
	temp_data$0   = 0;
	temp_data$1   = LCD_WIDTH - 1 - col_data_s;

	math_dividend = input_data;
	math_divisor  = 10;

	while(math_dividend > 0)
	{
		eword_divide();
		math_dividend = math_quotient;
		output_char = math_remainder;
		Display_Digit();
		temp_data$0++;
	}

	while(temp_data$0 < temp_data$1)
	{
		output_char = 0;
		Display_Digit();
		temp_data$0++;
	}
}


static void Change_Dir_Sign(void)
{
	if (dir_sign)
	{
		lcd_trx_byte = LCD_minus;
		LCD_Write_Byte();
		dir_sign = 0;
	}
	else
	{
		lcd_trx_byte = LCD_plus;
		LCD_Write_Byte();
		dir_sign = 1;
	}
}


static void Change_Value(void)
{
	temp_data$0 = RETURN_COL - col_index - 1;
	temp_data$1 = temp_data$0;
	math_dividend = input_data;
	math_divisor = 10;

	do 
	{
		eword_divide();
		math_dividend = math_quotient;
	}
	while(temp_data$0--);
	output_char = math_remainder;

	if (shift_r_flag) 
	{
		output_char++;
		if (output_char > 9) output_char = 0;
	}
	else
	{
		if (output_char == 0) output_char = 9;
		else output_char--;
	}

	// Reconstruct new value since LCD cannot be read
	math_mult_a = math_quotient;
	math_mult_b = 10;
	word_multiply();

	temp_data2 = math_product + output_char;

	math_mult_a = 1;
	math_mult_b = 10;
	math_product = 1;

	while(temp_data$1)
	{
		math_mult_a = math_product;
		word_multiply();
		temp_data$1--;
	}

	math_mult_a = math_product;
	math_mult_b = temp_data2;
	word_multiply();

	temp_data2 = math_product;

	math_divisor = math_mult_a;
	math_dividend = input_data;
	eword_divide();

	input_data = temp_data2 + math_remainder;

	// Update digit on display
	Display_Digit();
}


static void Clear_Line_Buffer (void)
{
	temp_data2$0 = 16;
	temp_data = line_buffer;
	while (temp_data2$0--)
	{
		*temp_data = LCD_space;
		temp_data++;
	}
}

static void Write_Line_Buffer (void)
{
	temp_data2$0 = 16;
	temp_data = line_buffer;
	while (temp_data2$0--)
	{
		lcd_trx_byte = *temp_data++;
		LCD_Write_Byte();
	}
}


static void Write_Data_Line (void)
{
	lcd_trx_byte = LCD_END;
	LCD_Address_Set();
	lcd_command = 1;
	lcd_trx_byte = REV_ENTRY;
	LCD_Write_Byte();
	if(next_screen == HOME_PAGE)
	{
		if (stepper_dist_mode)
		{
			lcd_trx_byte = LCD_L;
			LCD_Write_Byte();
			lcd_trx_byte = LCD_U;
			LCD_Write_Byte();
		}
		else
		{
			lcd_trx_byte = LCD_N;
			LCD_Write_Byte();
			lcd_trx_byte = LCD_I;
			LCD_Write_Byte();
			lcd_trx_byte = LCD_M;
			LCD_Write_Byte();
			lcd_trx_byte = LCD_slash;
			LCD_Write_Byte();
			lcd_trx_byte = LCD_L;
			LCD_Write_Byte();
			lcd_trx_byte = LCD_U;
			LCD_Write_Byte();
		}
	}
	else
	{
		lcd_trx_byte = LCD_return;
		LCD_Write_Byte();
	}
	Display_Digits();
	lcd_command = 1;
	lcd_trx_byte = FWD_ENTRY;
	LCD_Write_Byte();
	if (next_screen == FLOW_PAGE)
	{
		lcd_trx_byte = LCD_space;
		LCD_Write_Byte();
		if (dir_sign) lcd_trx_byte = LCD_plus;
		else lcd_trx_byte = LCD_minus;
		LCD_Write_Byte();
	}
	lcd_trx_byte = LCD_END;
	LCD_Address_Set();
}


static void Render_Screen(void)
{
	LCD_Clear();
	switch (next_screen)
	{
		case HOME_PAGE : 
			col_data_s = LCD_WIDTH - 1;
			Clear_Line_Buffer();
			line_buffer[0]  = LCD_P;
			line_buffer[1]  = LCD_U;
			line_buffer[2]  = LCD_M;
			line_buffer[3]  = LCD_P;

			if(stepper_is_moving)
			{
				line_buffer[6] = LCD_O;
				line_buffer[7] = LCD_N;
			}
			else
			{
				line_buffer[5] = LCD_O;
				line_buffer[6] = LCD_F;
				line_buffer[7] = LCD_F;
			}

			if (stepper_dist_mode)
			{
				
				input_data = stepper_units_per_run;

				line_buffer[10]  = LCD_V;
				line_buffer[11]  = LCD_O;
				line_buffer[12]  = LCD_L;
				line_buffer[13]  = LCD_U;
				line_buffer[14]  = LCD_M;
				line_buffer[15]  = LCD_E;
			}
			else
			{
				input_data = stepper_units_per_min;

				line_buffer[12]  = LCD_F;
				line_buffer[13]  = LCD_L;
				line_buffer[14]  = LCD_O;
				line_buffer[15]  = LCD_W;
			}
			break;

		case FLOW_PAGE :
			col_data_s = 9;
			input_data = stepper_units_per_min;

			Clear_Line_Buffer();
			line_buffer[0]  = LCD_U;
			line_buffer[1]  = LCD_L;
			line_buffer[2]  = LCD_slash;
			line_buffer[3]  = LCD_M;
			line_buffer[4]  = LCD_I;
			line_buffer[5]  = LCD_N;
			break;

		case VOL_PAGE :
			col_data_s = 6;
			input_data = stepper_units_per_run;

			Clear_Line_Buffer();
			line_buffer[0]  = LCD_V;
			line_buffer[1]  = LCD_O;
			line_buffer[2]  = LCD_L;
			line_buffer[3]  = LCD_para_l;
			line_buffer[4]  = LCD_U;
			line_buffer[5]  = LCD_L;
			line_buffer[6]  = LCD_para_r;
			break;

		case MODE_PAGE :
			Clear_Line_Buffer();
			if (!stepper_dist_mode) line_buffer[0] = LCD_return;
			line_buffer[1]  = LCD_F;
			line_buffer[2]  = LCD_L;
			line_buffer[3]  = LCD_O;
			line_buffer[4]  = LCD_W;
			Write_Line_Buffer();

			lcd_trx_byte = LCD_L2;
			LCD_Address_Set();

			Clear_Line_Buffer();
			if (stepper_dist_mode) line_buffer[0] = LCD_return;
			line_buffer[1]  = LCD_V;
			line_buffer[2]  = LCD_O;
			line_buffer[3]  = LCD_L;
			line_buffer[4]  = LCD_U;
			line_buffer[5]  = LCD_M;
			line_buffer[6]  = LCD_E;
			break;

		case UNITS_PAGE :
			col_data_s = 10;
			input_data = stepper_units_per_rev;

			Clear_Line_Buffer();
			line_buffer[0]  = LCD_U;
			line_buffer[1]  = LCD_L;
			line_buffer[2]  = LCD_slash;
			line_buffer[3]  = LCD_R;
			line_buffer[4]  = LCD_E;
			line_buffer[5]  = LCD_V;
			break;

		case EXIT_PAGE:			
			Clear_Line_Buffer();
			line_buffer[0]  = LCD_E;
			line_buffer[1]  = LCD_X;
			line_buffer[2]  = LCD_I;
			line_buffer[3]  = LCD_T;
			line_buffer[5]  = LCD_return;
	}
	Write_Line_Buffer();
	if ((next_screen != MODE_PAGE) && (next_screen != EXIT_PAGE)) Write_Data_Line();
	
}


// EEPROM OPERATIONS
static void Save_Settings(void)
{
	// Not enough ROM to switch case the saves
	eeprom_buff[1] = ADDR_SAVED;
	eeprom_buff[2] = EEPROM_INIT_VAL;
	EEPROM_Write();

	eeprom_buff[1] = ADDR_DIR;
	if (stepper_dir) eeprom_buff[2] = 1;
	else eeprom_buff[2] = 0;
	EEPROM_Write();

	eeprom_buff[1] = ADDR_UNITS_REV;
	eeprom_buff[2] = stepper_units_per_rev$0;
	eeprom_buff[3] = stepper_units_per_rev$1;
	EEPROM_Write();

	eeprom_buff[1] = ADDR_VELOCITY;
	eeprom_buff[2] = stepper_units_per_min$0;
	eeprom_buff[3] = stepper_units_per_min$1;
	EEPROM_Write();

	eeprom_buff[1] = ADDR_VOLUME;
	eeprom_buff[2] = stepper_units_per_run$0;
	eeprom_buff[3] = stepper_units_per_run$1;
	eeprom_buff[4] = stepper_units_per_run$2;
	EEPROM_Write();
}

static void Read_Settings(void)
{
	if (init_flag) eeprom_buff[1] = ADDR_SAVED; 
	else
	{
		EEPROM_Read();
		if (eeprom_buff[2])  stepper_dir = 1;
		else stepper_dir = 0;

		eeprom_buff[1] = ADDR_UNITS_REV;
		EEPROM_Read();
		stepper_units_per_rev$0 = eeprom_buff[2];
		stepper_units_per_rev$1 = eeprom_buff[3];

		eeprom_buff[1] = ADDR_VELOCITY;
		EEPROM_Read();
		stepper_units_per_min$0 = eeprom_buff[2];
		stepper_units_per_min$1 = eeprom_buff[3];

		eeprom_buff[1] = ADDR_VOLUME;
		EEPROM_Read();
		stepper_units_per_run$0 = eeprom_buff[2];
		stepper_units_per_run$1 = eeprom_buff[3];
		stepper_units_per_run$2 = eeprom_buff[4];
	}
}


// STEPPER OPERATIONS
static void Check_And_Store_Value(void)
{
	switch (curr_screen)
	{

		case UNITS_PAGE :
			if (input_data > 0xFFFF) input_data = 0xFFFF;
			stepper_units_per_rev = input_data;
			break;

		case VOL_PAGE :
			if (input_data > 0xFFFFFF) input_data = 0xFFFFFF;
			stepper_units_per_run = input_data;
			break;

		case FLOW_PAGE :
			if (input_data > 0xFFFF) input_data = 0xFFFF;
			stepper_units_per_min = input_data;

			if (dir_sign) stepper_dir = 1;
			else stepper_dir = 0;
			break;
	}
	Save_Settings();
}


// BUTTON OPERATIONS
static void Process_Inputs(void)
{
	temp_data$0  = active_inputs & _FIELD(start_button);
	temp_data$1  = active_inputs & _FIELD(select_button);
	temp_data2$0 = active_inputs & _FIELD(rotary_input1);
	temp_data2$1 = active_inputs & _FIELD(rotary_input2);
	active_inputs = 0 ;

	if (temp_data$0)  start_flag  = 1;
	if (temp_data$1)  select_flag = 1;
	if (temp_data2$0)
	{
		shift_r_flag = 0;
		shift_flag  = 1;
		if (temp_data2$1)  shift_r_flag = 1;
	}
}


// STATE OPERATIONS
static void Change_Next_Screen(void)
{
	if (shift_r_flag)
	{
		next_screen++;
		if (next_screen > EXIT_PAGE) next_screen = FLOW_PAGE;
	}
	else
	{
		next_screen--;
		if (next_screen == 0) next_screen = EXIT_PAGE;
	}
	update_display = 1;
}


// MODE OPERATIONS
static void Operation_Menu(void)
{
	if (select_flag) 
	{
		if(curr_screen == MODE_PAGE && !stepper_is_moving)
		{
			if (stepper_dist_mode) stepper_dist_mode = 0;
			else stepper_dist_mode = 1;
			update_display = 1;
		}

		else
		{
			next_state = EDIT_MODE;
			col_index = RETURN_COL;
			lcd_command = 1;
			lcd_trx_byte = LCD_CURSOR_ON;
			LCD_Write_Byte();
		}
	}
	elseif (shift_flag) Change_Next_Screen();
}


static void Operation_Value(void)
{
	if (select_flag) next_state = EDIT_MODE;
	elseif (shift_flag) Change_Value();
}


static void Operation_Edit(void)
{
	if (select_flag)
	{
		if (col_index == RETURN_COL)
		{
			next_state = MENU_MODE;
			lcd_command = 1;
			lcd_trx_byte = LCD_CURSOR_OFF;
			LCD_Write_Byte();
			update_display = 1;
		}
		elseif ((curr_screen == FLOW_PAGE) && (col_index == col_data_s)) Change_Dir_Sign();
		else next_state = VALUE_MODE;
	}
	elseif (shift_flag)
	{
		if (shift_r_flag) col_index++;
		else col_index--;

		if (col_index > RETURN_COL) col_index = RETURN_COL;
		elseif (col_index < col_data_s) col_index = col_data_s;
	}
}


//===================//
// PROGRAM FUNCTIONS //
//===================//

void Pump_Initialize(void)
{
	lcd_device_addr = LCD_DRIVER;
	LCD_Initialize();
	Stepper_Initialize();
	Button_Initialize();
	EEPROM_Initialize();

	
	stepper_steps_per_rev = def_steps_per_rev;
	stepper_units_per_rev = def_ul_per_rev;
	stepper_units_per_run = def_volume;
	stepper_units_per_min = def_ul_per_min;
	stepper_dir = def_direction;

	init_flag = 1;
	eeprom_buff[0] = 4;
	Read_Settings();

	if (eeprom_buff[2] == EEPROM_INIT_VAL)
	{
		init_flag = 0;
		Read_Settings();
	}
	else Save_Settings();

	if (stepper_dir) dir_sign = 1;
	else dir_sign = 0;

	Render_Screen();
}


void Pump_State_Machine(void)
{
	Button_Poll();
	Process_Inputs();
	next_screen = curr_screen;
	switch (curr_screen)
	{
		case HOME_PAGE :
			if (select_flag)
			{
				next_screen = FLOW_PAGE;
				update_display = 1;
			}
			break;

		case EXIT_PAGE :
			if (select_flag)
			{
				next_screen = HOME_PAGE;
				update_display = 1;
			}
			elseif (shift_flag) Change_Next_Screen();
			break;

		default : //VOL_PAGE, UNITS_PAGE, STEPS_PAGE, FLOW_PAGE, MODE_PAG
			switch (curr_state)
			{
				case MENU_MODE :
					Operation_Menu();
					break;

				case EDIT_MODE :
					Operation_Edit();
					break;

				case VALUE_MODE :
					Operation_Value();
					break;
			}
	}

	// Update Stepper Settings
	if (next_state == MENU_MODE && curr_state == EDIT_MODE) Check_And_Store_Value();

	// Update Stepper State
	if (start_flag && stepper_is_moving)
	{
		Stepper_Stop();
	}
	elseif (start_flag && !stepper_is_moving && (next_state == MENU_MODE))
	{
		Stepper_Set_Dir();
		Stepper_Set_Vel();
		Stepper_Enable();
		Stepper_Start();
		update_display = 1;
	}
	if (!stepper_is_moving && stepper_enabled){
		Stepper_Disable();
		update_display = 1;
	}

	// Update Display
	if (update_display) Render_Screen();
	if (next_state != MENU_MODE) 
	{
		lcd_trx_byte = LCD_L2 + col_index;
		LCD_Address_Set();
	}

	// Update Indices
	curr_screen = next_screen;
	curr_state  = next_state;

	// Clear flags
	start_flag  = 0;
	select_flag = 0;
	shift_flag  = 0;
	update_display = 0;

	if (!stepper_is_moving)
	{
		CLKMD = 0xF4;
		CLKMD.4 = 0;
		STOPEXE;
		CLKMD.4 = 1;
		CLKMD = 0x14;
	}
}