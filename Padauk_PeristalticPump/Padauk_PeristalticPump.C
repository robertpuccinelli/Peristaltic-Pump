#include "../Padauk-Peripherals/system_settings.h"
#include "pump_core.h"
#include "../Padauk-Peripherals/pdk_button.h"
#include "../Padauk-Peripherals/pdk_stepper.h"

void	FPPA0 (void)
{
	.ADJUST_IC	SYSCLK=IHRC/4 Vdd=5.0V	//	SYSCLK=IHRC/4
	ENGINT;

	Pump_Initialize();

	while (1)
	{
		Pump_State_Machine();
	}
}


void	Interrupt (void)
{
	pushaf;

	if (Intrq.BTN_INTR)     Button_Debounce_Interrupt();
	if (Intrq.STEPPER_INTR) Stepper_Dist_Mode_Interrupt();

	popaf;
}

