/***************************************************************************//**
 * @file
 * @brief Application specific overrides of weak functions defined as part of
 * the test application.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_timer.h"
#include "bsp.h"
#include "gpiointerrupt.h"
#include "hal_common.h"

enum mode_e{
	CONFORT = 0,
	CONFORT1,
	CONFORT2,
	ECO,
	HORSGEL,
	ARRET,
};

TIMER_TypeDef *timer0 = 0x40018000;

void set_arret_horsgel(enum mode_e m)
{
	timer0->CC[2].CCV = 1;
	timer0->TOP = 1;

	uint32_t zc = GPIO_PinInGet(gpioPortC, 11);

	switch(m)
	{
	  case HORSGEL:
		timer0->CNT = zc ? 0 : 1;
		break;
	  default:
	    timer0->CNT = zc ? 1 : 0;
	    break;
	}

  uint32_t zcstate = GPIO_PinInGet(gpioPortC, 11);

  /* Check for race condition in case a ZC transition has just occurred */
  if (zcstate == zc)
	 return;

  switch(m)
  {
    case HORSGEL:
	  timer0->CNT = zcstate ? 0 : 1;
	  break;
	default:
	  timer0->CNT = zcstate ? 1 : 0;
	  break;
  }
}

int main(void)
{
  // Initialize the chip
  CHIP_Init();

  // Initialize the system clocks and other HAL components
  halInit();

  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);

  USTIMER_Init();
  TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;

  /* Stop Timer */
  timer0->CMD = 2;

  init.enable = 0;
  init.clkSel = timerClkSelCC1;


  TIMER_Init(timer0, &init);

  timer0->CNT = 0;
  timer0->CC[1].CTRL = TIMER_CC_CTRL_ICEDGE_BOTH;
  timer0->CC[2].CTRL = TIMER_CC_CTRL_MODE_PWM ;

  timer0->ROUTELOC0 = _TIMER_ROUTELOC0_CC0LOC_LOC16 | (_TIMER_ROUTELOC0_CC1LOC_LOC15 << 8) |  (_TIMER_ROUTELOC0_CC2LOC_LOC13 << 16);
  timer0->ROUTEPEN =  TIMER_ROUTEPEN_CC1PEN | TIMER_ROUTEPEN_CC2PEN;

  // Pilot wire on CC2 out on PC10
  // ZC on CC1 and CC0 on PC11
  GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortC, 11, gpioModeInput, 0);

  GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 0);

  timer0->CMD = 1;

  while (1) {
	  USTIMER_Delay(200000);
	  enum mode_e m = rand() % 2 ? HORSGEL:CONFORT1;
      uint32_t zstate;

	  switch(m)
	  {
		  case HORSGEL:
		  case ARRET:
		  set_arret_horsgel(m);
		  break;
		  case ECO:
		  /* Always 1 */
		  timer0->CC[2].CCV = 2;
		  timer0->TOP = 1;
		  timer0->CNT = 0;
		  break;
		  case CONFORT:
		  /* Always 0 */
		  timer0->CNT = 0;
		  timer0->CC[2].CCV = 0;
		  timer0->TOP = 1;
		  break;
		  case CONFORT1:
		  /* Always 0 */
		  timer0->CC[2].CCV = 24;
		  timer0->TOP = 28;
		  timer0->CNT = 0;
			 break;
		  case CONFORT2:
		  /* Always 0 */
		  timer0->CC[2].CCV = 12;
		  timer0->TOP = 14;
		  timer0->CNT = 0;
		  break;
	  }
  }
}
