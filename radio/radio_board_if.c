#include "radio_board_if.h"
#include "rtdevice.h"
#include "pin_config.h"

void RF_Switch_Init(void)
{
    rt_pin_mode(RF_SW1_PIN,PIN_MODE_OUTPUT);
    rt_pin_write(RF_SW1_PIN,PIN_LOW);
    rt_pin_mode(RF_SW2_PIN,PIN_MODE_OUTPUT);
    rt_pin_write(RF_SW2_PIN,PIN_LOW);
    rt_pin_mode(TCXO_PWR_PIN,PIN_MODE_OUTPUT);
    rt_pin_write(TCXO_PWR_PIN,PIN_HIGH);
}

int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
  int32_t retcode = 0;
  switch (Config)
  {
    case RBI_SWITCH_OFF:
    {
        /* Turn off switch */
        rt_pin_write(RF_SW1_PIN,PIN_LOW);
        rt_pin_write(RF_SW2_PIN,PIN_LOW);
        break;
    }
    case RBI_SWITCH_RX:
    {
        /*Turns On in Rx Mode the RF Switch */
        rt_pin_write(RF_SW1_PIN,PIN_LOW);
        rt_pin_write(RF_SW2_PIN,PIN_HIGH);
        break;
    }
    case RBI_SWITCH_RFO_LP:
    {
        /*Turns On in Tx Low Power the RF Switch */
        rt_pin_write(RF_SW1_PIN,PIN_HIGH);
        rt_pin_write(RF_SW2_PIN,PIN_LOW);
        break;
    }
    case RBI_SWITCH_RFO_HP:
    {
        /*NO HIGH POWER MODE */
        break;
    }
    default:
      break;
  }
  return retcode;
}

int32_t RBI_GetTxConfig(void)
{
  int32_t retcode = RBI_CONF_RFO_LP;
  return retcode;
}

int32_t RBI_IsTCXO(void)
{
  int32_t retcode = RADIO_CONF_TCXO_SUPPORTED;
  return retcode;
}

int32_t RBI_IsDCDC(void)
{
  int32_t retcode = RADIO_CONF_DCDC_SUPPORTED;
  return retcode;
}

int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef Config)
{
    int32_t ret = 0;
    if(Config == RBI_RFO_LP_MAXPOWER)
    {
        ret = RADIO_CONF_RFO_LP_MAX_15_dBm;
    }
    else
    {
        ret = RADIO_CONF_RFO_HP_MAX_22_dBm;
    }
    return ret;
}
