// This is firmware for the Gun Driver Board

#include "A35975_250.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_64 & PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


TYPE_GLOBAL_DATA_A35975_250 global_data_A35975_250;



#define FPGA_ID     				0x0040  // used to verify GD FPGA communications

// watch dog related
#define WATCHDOG_1V_KICK        	16000
#define WATCHDOG_3V_KICK        	48000
#define WATCHDOG_1V_FEEDBACK 		1000
#define WATCHDOG_3V_FEEDBACK 		3000

#define WATCHDOG_ERR_MAX     	    5
#define WATCHDOG_FEEDBACK_MARGIN    200

#define EF_SET_MIN					7518  /* 1V */
#define EG_SET_MIN                  150   /* -79V */
#define EK_RD_MIN_FOR_GRID_ON       900	  /* min 5kv to turn grid on */			    


int main(void) {
  control_state = STATE_START_UP;

  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  switch (global_data_A35975_250.control_state) {

  case STATE_STARTUP:
    InitializeA35975();
    DisableHighVoltage();
    DisableHeater();
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    global_data_A35975_250.start_up_counter = 0;
    global_data_A35975_250.led_flash_counter = 0;
    break;


  case STATE_WAIT_FOR_CONFIG:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A35975_250.control_state == STATE_WAIT_FOR_CONFIG) {
      DoA35975_250();
      DoStartupLEDs();
      if ((global_data_A35975_250.led_flash_counter >= LED_STARTUP_FLASH_TIME) && (_CONTROL_NOT_CONFIGURED == 0)) {
	global_data_A35975_250.control_state = STATE_HEATER_RAMP_UP;
      }
    }
    break;


    // DPARKER the function that sets the heater voltage must set the "heater_voltage_target", not the "set_point"
  case STATE_HEATER_RAMP_UP:
    global_data_A35975_250.analog_output_heater_voltage.set_point = 0;
    global_data_A35975_250.start_up_counter++;
    DisableHighVoltage();
    EnableHeater();
    while (global_data_A35975_250.control_state == STATE_HEATER_RAMP_UP) {
      DoA35975_250();
      DoHeaterRampUp();
      if (global_data_A35975_250.analog_output_heater_voltage.set_point >= global_data_A35975_250.heater_voltage_target) {
	global_data_A35975_250.control_state = STATE_HEATER_RAMP_UP_DONE;
      }
      if (_FAULT_PIC_HEATER_TURN_OFF) {
	global_data_A35975_250.control_state = STATE_FAULT_HEATER_OFF;
      }
    }
    break;
    

  case STATE_HEATER_RAMP_UP_DONE:
    DisableHighVoltage();
    while (global_data_A35975_250.control_state == STATE_HEATER_RAMP_UP_DONE) {
      global_data_A35975_250.analog_output_heater_voltage.set_point = global_data_A35975_250.heater_voltage_target;
      if (!_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	global_data_A35975_250.control_state = STATE_POWER_SUPPLY_RAMP_UP;
      }
      if (CheckFault()) {
	global_data_A35975_250.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

    
#define GUN_DRIVER_POWER_SUPPLY_STATUP_TIME  500 // 5 seconds

  case STATE_POWER_SUPPLY_RAMP_UP:
    EnableHighVoltage();
    power_supply_ramp_up_counter = 0;
    while (global_data_A35975_250.control_state == STATE_POWER_SUPPLY_RAMP_UP) {
      global_data_A35975_250.analog_output_heater_voltage.set_point = global_data_A35975_250.heater_voltage_target;
      if (power_supply_startup_up_counter >= GUN_DRIVER_POWER_SUPPLY_STATUP_TIME) {
	global_data_A35975_250.control_state = STATE_HV_ON;
      }
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	global_data_A35975_250.control_state = STATE_HEATER_RAMP_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A35975_250.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

  case STATE_HV_ON:
    while (global_data_A35975_250.control_state == STATE_HV_ON) {
      global_data_A35975_250.analog_output_heater_voltage.set_point = global_data_A35975_250.heater_voltage_target;
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	global_data_A35975_250.control_state = STATE_HEATER_RAMP_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A35975_250.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

  case STATE_FAULT_HEATER_ON:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A35975_250.control_state == STATE_FAULT_HEATER_ON) {

    }
    break;


  case STATE_FAULT_HEATER_OFF:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A35975_250.control_state == STATE_FAULT_HEATER_OFF) {
    }
    break;

  case STATE_FAULT_HEATER_FAILURE:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A35975_250.control_state == STATE_FAULT_HEATER_FAILURE) {
    }
    break;

  }
}


void DoA35975_250(void) {
  

  ETMCanSlaveDoCan();

  if (_T2IF) {

    // Run once every 10ms


    // Update to counter used to flash the LEDs at startup
    global_data_A35975_250.led_flash_counter++;
    if (global_data_A35975_250.led_flash_counter >= LED_STARTUP_FLASH_TIME) {
      global_data_A35975_250.led_flash_counter = LED_STARTUP_FLASH_TIME;
    }



  }
}




/*
  Based on the control state only a subset of faults are checked
  There are four fault checking functions that are called based upon the operation state
*/


void UpdateFaultsHeaterRampUp(void) {
  /*
    This updates all faults related to the heater ramping up
  */
}

void UpdateFaultsHeaterOn(void) {
  /*
    This includes all faults in UpdateFaultsHeaterRampUp as well as 
    faults that are checked once the heater has finished ramping up
  */
}


void UpdateFaultsPowerSupplyRampUp(void) {
  /*
    All faults in UpdateFaultsHeaterOn as well as
    faults that are checked when the power supplies are ramping up
  */
}


void UpdateFaultsPowerSupplyOn(void) {
  /*
    All faults in UpdateFaultsPowerSupplyRampUp as well as
    all other remaining faults
  */

}

void EnableHeater(void) {

}

void DisableHeater(void) {

}

void EnableHighVoltage(void) {
  /*
    Set the HVPS reference
    Set the grid top reference // DPARKER is this voltage always fixed?
    Set the HVPS enable control voltage
    Set the grid top enable control voltage
    Set the trigger enable control voltage
  */
}

void DisableHighVoltage(void) {
/*
    Set the HVPS reference to zero
    Set the grid top reference to zero // DPARKER is this voltage always fixed?
    Clear the HVPS enable control voltage
    Clear the grid top enable control voltage
    Clear the trigger enable control voltage
*/
}

void ResetFPGA(void) {

}



void ADCConfigure(void) {
  /*
    Configure for read of all channels + temperature with 8x (or 16x) Averaging
  */
}

void ADCStartAcquisition(void) {
  /* 
     Start the acquisition process
  */
}

void ADCReadResults(void) {
  /*
    Read all the results of the 16 Channels + temp sensor
    16 bits per channel
    17 channels
    272 bit message
    Approx 350us (counting processor overhead)
  */
}


void DACWriteAllChannels(void) {
  /*
    Write all 8 channels to the DAC.
    Message will be 24 bits per channel.
    8 channels 
    192 bit message.
    At 1 mbit a full message is approx 250us (counting processor overhead)
  */

}

void ReadFPGA(void) {
  /*
    Reads 32 bits from the FPGA
  */
}


void DoHeaterRampUp(void) {
  if (heater_ramp_counter >= 100) {
    // We only update the ramp up once per second durring the ramp
    heater_ramp_counter = 0;

    // If the current is less than the max ramp up current, then increase the heater program voltage
    if (global_data_A35975_250.analog_input_heater_current.reading_scaled_and_calibrated < MAX_HEATER_CURRENT_DURING_RAMP_UP) {
      global_data_A35975_250.analog_output_heater_voltage.set_point += HEATER_RAMP_UP_INCREMENT;
      if (global_data_A35975_250.analog_output_heater_voltage.set_point > global_data_A35975_250.heater_voltage_target) {
	global_data_A35975_250.analog_output_heater_voltage.set_point = global_data_A35975_250.heater_voltage_target;
      }
    }
  }  
}

void InitializeA35975(void) {
 
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;

  
  // Init analog scaling for CAN bus
#ifdef USE_ENGINEERING_UNIT_ON_GUN_DRIVER
  CAN_scale_table[CAN_RD_EK  ].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAL_EK_RD/CAN_EK_SCALE);
  CAN_scale_table[CAN_RD_IKP ].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAL_IKP_RD/CAN_IKP_SCALE);
  CAN_scale_table[CAN_RD_EF  ].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAL_EF_RD/CAN_EF_SCALE);
  CAN_scale_table[CAN_RD_IF  ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_IF_RD/CAN_IF_SCALE);
  CAN_scale_table[CAN_RD_EG  ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EG_RD/CAN_EG_SCALE);
  CAN_scale_table[CAN_RD_EC  ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EC_RD/CAN_EC_SCALE);
  CAN_scale_table[CAN_RD_TEMP].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_TEMP_RD/CAN_TEMP_SCALE);

  CAN_scale_table[CAN_RD_EKSET ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EKSET/CAN_EKSET_SCALE);
  CAN_scale_table[CAN_RD_EFSET ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EFSET/CAN_EFSET_SCALE);
  CAN_scale_table[CAN_RD_EGSET ].fixed_scale = MACRO_DEC_TO_CAL_FACTOR_2(CAL_EGSET/CAN_EGSET_SCALE);
  CAN_scale_table[CAN_SET_EKSET].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAN_EKSET_SCALE/CAL_EKSET);
  CAN_scale_table[CAN_SET_EFSET].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAN_EFSET_SCALE/CAL_EFSET);
  CAN_scale_table[CAN_SET_EGSET].fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(CAN_EGSET_SCALE/CAL_EGSET);
#endif 

  // --------- BEGIN IO PIN CONFIGURATION ------------------

  // Initialize Ouput Pin Latches BEFORE setting the pins to Output
  PIN_CS_DAC_ENABLE = !OLL_CS_DAC_ENABLE;
  PIN_CS_ADC_ENABLE = !OLL_CS_ADC_ENABLE;
  PIN_CS_AUX_ENABLE = !OLL_CS_AUX_ENABLE;
	  
  // LOGIC Output Pins
  PIN_OPT_GD_READY  = !OLL_OPT_GD_READY;
  PIN_CAN_HV_ENABLE = !OLL_CAN_HV_ENABLE;
  PIN_CAN_PULSETOP_ENABLE = !OLL_CAN_PULSETOP_ENABLE;
	  
  PIN_CAN_TRIGGER_ENABLE = !OLL_CAN_TRIGGER_ENABLE;
	  

  // MCP4822 DAC Output Pins
  PIN_CS_MCP4822_ENABLE = !OLL_CS_MCP4822_ENABLE;
  PIN_LDAC_MCP4822_ENABLE = !OLL_LDAC_MCP4822_ENABLE;

	  
  // UART TX enable
  PIN_RS422_DE = !OLL_RS422_DE_ENABLE_RS422_DRIVER;


  // LED Indicator Output Pins
  PIN_LED_24DC_OK = OLL_LED_ON;  
  PIN_LED_LAST_PULSE_GOOD = !OLL_LED_ON;
  PIN_LED_GD_READY = !OLL_LED_ON;  
	  
  PIN_LED_HV_ENABLE = !OLL_LED_ON;  
  PIN_LED_AC_ON = !OLL_LED_ON;  
  //  PIN_LED_LAST_PULSE_FAIL = !OLL_LED_ON;

  PIN_LED_WARMUP = !OLL_LED_ON;
  PIN_LED_SUM_FAULT = !OLL_LED_ON;

  // ---- Configure the dsPIC ADC Module Analog Inputs------------ //
  ADPCFG = 0xFFFF;             // all are digital I/O
 
  // Initialize all I/O Registers
  TRISA = A35975_TRISA_VALUE;
  TRISB = A35975_TRISB_VALUE;
  TRISC = A35975_TRISC_VALUE;
  TRISD = A35975_TRISD_VALUE;
  TRISF = A35975_TRISF_VALUE;
  TRISG = A35975_TRISG_VALUE;

  // Config SPI1 for Gun Driver
  ConfigureSPI(ETM_SPI_PORT_1, A35975_SPI1CON_VALUE, 0, A35975_SPI1STAT_VALUE, SPI_CLK_1_MBIT, FCY_CLK);  
  

  // ---------- Configure Timers ----------------- //

  // Configure TMR1
  T1CON = A35975_T1CON_VALUE;
  PR1 = A35975_PR1_VALUE;  
  TMR1 = 0;
  _T1IF = 0;

  // Initialize TMR2
  PR2   = A35975_PR2_VALUE;
  TMR2  = 0;
  _T2IF = 0;
  _T2IP = 5;
  T2CON = A35975_T2CON_VALUE;

  ResetFPGA();
  
  faults_reg_system_control = 0;
  faults_reg_software = 0;	 
  faults_reg_digi_from_gd_fpgaid = 0;
  

  // Initialize the Can module
  ETMCanSlaveInitialize(FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RG12, 4);
  ETMCanSlaveLoadConfiguration(35975, 250, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);



  /* DPARKER move to led pulse state
  for (n = 0; n < 2; n++) {
    PIN_LED_24DC_OK 		 = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_LAST_PULSE_GOOD  = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_GD_READY         = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_HV_ENABLE        = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_AC_ON            = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_LAST_PULSE_FAIL  = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_WARMUP           = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_SUM_FAULT        = n? !OLL_LED_ON : OLL_LED_ON;
    __delay32(600000);
    ClrWdt();
  }

  */

  // LED Indicator Output Pins, keep 24DC on
  PIN_LED_24DC_OK = OLL_LED_ON;  
}






unsigned char watch_dog_timer = 0;
unsigned long read_cycles = 0;
unsigned char gd_read_ptr;  // cycle the readbacks from gd fpga board
unsigned int sec_count = 0;


void Do10msTicToc(void) {
  unsigned long temp;
  unsigned int i, bit_value;
  

  // update one value from gd fpga board, watchdog channel is handled by watchdog kicking
  if (gd_read_ptr < 12)	{	// don't care ch9, 13, 14
    temp = ReadAdcChannel(gd_read_ptr);

    if (gd_read_ptr <= 8) {
      analog_reads[ gd_read_ptr ].read_cur = temp;
      analog_reads[ gd_read_ptr ].read_cnt++;
      CheckAnalogLimits(gd_read_ptr);
    } else if (gd_read_ptr >= 10 && gd_read_ptr <= 11) {	// only care about wdog and arc faults 
      i = gd_read_ptr - 9;
      if (temp <= 2500) {
	faults_from_ADC |= (1 << i);
      } else {
	faults_from_ADC &= ~(1 << i);
      }
      
      if (digi_reads[i].action_code < 99) {
	digi_reads[i].state = (temp > 2500);  
	if (temp <= 2500) {
	  DoFaultRecord(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i)); // mapped to fpgaID bit 1 & 2
	}
      }
    }
  } else {
    temp = ReadFPGAID(); 
    // check if the ID read is valid
    if ((temp & 0x03c0) == (FPGA_ID & 0x03c0)) {  // check FPGA major version only.  0x40 for the low byte, two bits stay 0 for the 2nd low byte, xxxxxx00 01000000
      temp >>= 16;
      fpga_ASDR = (unsigned int)temp;
      // handle digi faults from fpgaid
      for (i = 0; i < 16; i++) {
	bit_value = (temp & (1 << i)) > 0;
	if (digi_reads[ i + DIGI_ID_ARC_COUNT].action_code < 99) {
	  digi_reads[ i + DIGI_ID_ARC_COUNT].state = bit_value;
	  if (bit_value) {
	    DoFaultRecord(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i));
	  } else if (digi_reads[ i + DIGI_ID_ARC_COUNT].action_code == 0) {   
	    DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i));
	  }
	}
      }
      PIN_LED_AC_ON = OLL_LED_ON;    
    } else { // declare a fault
      PIN_LED_AC_ON = !OLL_LED_ON;    
      DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_FPGAID);
    }
  }
  
  if (gd_read_ptr == 8)	{
    gd_read_ptr = 9; // bypass ch9
  } else if (gd_read_ptr == 11) {
    gd_read_ptr = 14; // bypass ch12, 13, 14
  } 
  
  gd_read_ptr = (gd_read_ptr + 1) & 0x000f;	 // 0 to 15 for gd_read_ptr
  
  if (!gd_read_ptr) {
    read_cycles++;
  }
  
  if (_T2IF) {
    _T2IF = 0;
    //10ms roll has occured
    sec_count++;
    if (sec_count > 200) {
      read_cycles_in_2s = read_cycles;
      read_cycles = 0;
      sec_count = 0;
    }
        
    if ((control_state & 0x7f) > STATE_READY_FOR_HEATER) {
      if (_CONTROL_CAN_COM_LOSS)	{
	// declare fault, turn all off
	DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_CAN_TIMEOUT);
	_FAULT_CAN_COMMUNICATION_LATCHED = 1;
      }
    }

    watch_dog_timer++;
    if (watch_dog_timer >= 3) {
      DoFpgaWatchdog();  // tickle the watchdog every 30ms
      watch_dog_timer = 0;
    }
    
    // record debug data for CAN bus  
    /*
    local_debug_data.debug_0 = analog_reads[ANA_RD_EK].read_cur;
    local_debug_data.debug_1 = analog_reads[ANA_RD_IKA].read_cur;
    local_debug_data.debug_2 = analog_reads[ANA_RD_IKP].read_cur;
    local_debug_data.debug_3 = analog_reads[ANA_RD_EF].read_cur;
 
    local_debug_data.debug_4 = analog_reads[ANA_RD_IF].read_cur;;
    local_debug_data.debug_5 = analog_reads[ANA_RD_EG].read_cur;;
    local_debug_data.debug_6 = analog_reads[ANA_RD_EC].read_cur;;
    local_debug_data.debug_7 = analog_reads[ANA_RD_TEMP].read_cur;;
    local_debug_data.debug_8 = control_state;
    local_debug_data.debug_9 = htd_timer_in_100ms;
    

    local_debug_data.debug_D = analog_sets[ANA_SET_EK].ip_set;
    local_debug_data.debug_E = analog_sets[ANA_SET_EF].ip_set;
    local_debug_data.debug_F = analog_sets[ANA_SET_EG].ip_set;
    */
    local_debug_data.debug_0 = global_data_A35975_250.watchdog_count_error;
    

   
    if (ekuv_timeout_10ms) {
      if (ekuv_timeout_10ms < 1000) {
	ekuv_timeout_10ms++;
      }
    }
    
   
    if (ek_ref_changed_timer_10ms) {
      ek_ref_changed_timer_10ms--;
    }

    if (ef_ref_changed_timer_10ms) {
      ef_ref_changed_timer_10ms--;
    }
    
    if (eg_ref_changed_timer_10ms) {
      eg_ref_changed_timer_10ms--;
    }

    if (htr_OVOC_rest_delay_timer_10ms) {
      htr_OVOC_rest_delay_timer_10ms--; 
    }

    
    if ((_FAULT_REGISTER &  0x04 /*_FAULT_GD_SW_HTR_OVOC */) && (htr_OVOC_count == 0)) {
      if (_SYNC_CONTROL_RESET_ENABLE  && ((faults_reg_software & FAULTS_SW_EFOV_IFOC) == 0))
	_FAULT_REGISTER = _FAULT_REGISTER & (~0x04/* _FAULT_GD_SW_HTR_OVOC */);
    }
    
    
    led_pulse_count = ((led_pulse_count + 1) & 0b00111111);	 // 640ms
    if (led_pulse_count == 0) {
      // 10ms * 16 counter has ocurred
      if (PIN_LED_LAST_PULSE_GOOD) {
	PIN_LED_LAST_PULSE_GOOD = 0;
      } else {
	PIN_LED_LAST_PULSE_GOOD = 1;
      }  
    }
  } 
}

/*


/////////////////////////////////////////////////////////////////////////
// DoFpgaWatchdog() 
// send a watchdog DAC to fpga board 
//
static void DoFpgaWatchdog(void)
{
  static unsigned char kicked_1v = 0;
  static unsigned error_count = 0;
  int temp, counts;
    
    
  counts = kicked_1v? WATCHDOG_1V_FEEDBACK : WATCHDOG_3V_FEEDBACK;
  temp = (int)ReadAdcChannel(15);
    
#ifndef DEMO
  if ((temp < (counts - WATCHDOG_FEEDBACK_MARGIN)) || (temp > (counts + WATCHDOG_FEEDBACK_MARGIN))) {
    error_count++;
    if (error_count >= WATCHDOG_ERR_MAX) {            	
      DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_FPGA_WATCHDOG_ERR);
    } 
  }
  else {
    error_count = 0;
  }
#endif
    
  if (kicked_1v) {
    kicked_1v = 0;
    counts = WATCHDOG_3V_KICK;    
  }
  else {
    kicked_1v = 1;
    counts = WATCHDOG_1V_KICK;
  }
    
  SendWatchdogRef(counts);													
    	 
     
}

