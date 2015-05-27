#include "A35975.h"


// CONTROL FAULT REGISTER, 3 main types
unsigned int faults_reg_system_control;
unsigned int faults_reg_software;	    
unsigned int faults_reg_digi_from_gd_fpgaid;

TYPE_ANALOG_SETS analog_sets[ANALOG_SET_SIZE] = {	  
  {0,  0, 0, 0,   0xffff, 0,   0},
  {1,  0, 0, 0,   0xffff, 0,   0},
  {2,  0, 0, 0,   0xffff, 0,   0},
  {3,  0, 0, 0,   0xffff, 0,   0},
  {4,  0, 0, 0,   0xffff, 0,   0},
  {5,  0, 0, 0,   0xffff, 0,   0},
  {6,  0, 0, 0,   0xffff, 0,   0},
  {7,  0, 0, 0,   0xffff, 0,   0},
};

TYPE_ANALOG_READS analog_reads[ANALOG_READ_SIZE] = {	 
  {0,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEk  }, 
  {1,  0, 0, 0,    0, 0xfff, 0,  0, 0, 0        }, 
  {2,  0, 0, 0,    0, 0xfff, 0,  0, 0, 0        }, 
  {3,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEf  }, 
  {4,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultIf  }, 
  {5,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEg  }, 
  {6,  0, 0, 0,    0, 0xfff, 0,  0, 0, FaultEc  }, 
  {7,  0, 0, 0,    0, 0xfff, 0,  0, 0, Fault24v }, 
  {8,  0, 0, 0,    0, 0xfff, 0,  0, 0, 0        }, 
};

TYPE_CAN_SCALE_TABLE CAN_scale_table[CAN_SCALE_TABLE_SIZE] = {	 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0},
  
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
  {0, 0}, 
};


TYPE_DIGI_READS digi_reads[FAULT_SIZE] = {	
  {0, 0,   1, 0,  1,   0,    0, 99},	// htr/warmup fault
  {0, 0,   1, 1,  1,   0,    1, 1},	// fpga watchdog fault
  {0, 0,   1, 2,  1,   0,    0, 2},	// arc fault
  {0, 0,   1, 3,  1,   0,    0, 99},	// overtemp fault
  {0, 0,   1, 4,  1,   0,    0, 99},	// pw/duty fault
  {0, 0,   1, 5,  1,   0,    0, 99},	// bias or top fault
														   
  {0, 0,   0, 0,  1,   0,    0, 99},  // arc count > 0 			   
  {0, 0,   0, 1,  1,   0,    0, 99}, 	// arc HV inh active			   
  {0, 0,   0, 2,  1,   0,    0, 99},  // heater volt < 4.5V
  {0, 0,   0, 3,  1,   0,    0, 99},	// max temp > 65c
  {0, 0,   0, 4,  1,   0,    1, 1},	// max temp > 75c
  {0, 0,   0, 5,  1,   0,    0, 99},	// pulse width limiting
  {0, 0,   0, 6,  1,   0,    0, 2},	// prf fault
  {0, 0,   0, 7,  1,   0,    0, 2},	// current pw fault

  {0, 0,   0, 8,  1,   0,    1, 1},	// grid module hw fault
  {0, 0,   0, 9,  1,   0,    1, 1},	// grid module o/v fault
  {0, 0,   0, 10, 1,   0,    1, 1},	// grid module u/v fault
  {0, 0,   0, 11, 1,   0,    1, 1},	// grid module biasV fault
  {0, 0,   0, 12, 1,   0,    0, 99},	// hv regulation fault
  {0, 0,   0, 13, 1,   0,    0, 99},	// dip sw1 on
  {0, 0,   0, 14, 1,   0,    0, 99},	// test mode switch on
  {0, 0,   0, 15, 1,   0,    0, 99},	// local mode switch on
};




unsigned char control_state;
unsigned char last_control_state = STATE_START_UP;	
unsigned char system_byte;

unsigned int led_pulse_count;
unsigned int htd_timer_in_100ms;
unsigned int software_skip_warmup = 0;

unsigned int fpga_ASDR;
unsigned int faults_from_ADC;

unsigned long read_cycles_in_2s = 0;	// how fast is the updating speed

unsigned int ekuv_timeout_10ms = 0;

unsigned int ek_ref_changed_timer_10ms = 0;   // mask Ek faults when ref is changed
unsigned int ef_ref_changed_timer_10ms = 0;   // mask Ef, If faults when ref is changed
unsigned int eg_ref_changed_timer_10ms = 0;   // mask Eg fault when ref is changed

unsigned char htr_OVOC_count = 0;   // for auto-reset htr OVOC feature
unsigned int  htr_OVOC_rest_delay_timer_10ms = 0;	  // after OVOC fault, rest for a few seconds before turning htr on
unsigned char htr_OVOC_auto_reset_disable = 0;        // if other system fault happens, disable htr OVOC auto-reset

//	BUFFER64BYTE uart1_input_buffer;
//	BUFFER64BYTE uart1_output_buffer;

volatile unsigned int lvdinterrupt_counter = 0;

volatile unsigned int _PERSISTENT last_known_action;
volatile unsigned int _PERSISTENT last_osccon;

unsigned int _PERSISTENT processor_crash_count;
unsigned int previous_last_action;

unsigned long hw_version_data;


unsigned char sdo_reset_cmd_active;	  // logic resumes only when reset isn't active
unsigned char sdo_logic_reset;        // a separate cmd to reset fault
unsigned char sdo_htd_timer_reset; 

unsigned char sdo_htr_enable;
unsigned char sdo_hv_bypass;
unsigned char sdo_hv_enable;
unsigned char sdo_pulsetop_enable;
unsigned char sdo_trig_enable;  





// This is firmware for the Gun Driver Board

_FOSC(ECIO & CSW_FSCM_OFF); 
//_FWDT(WDT_OFF);  // 1 Second watchdog timer
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


//static void SetupAdc(void);
static unsigned ReadAdcChannel(unsigned chan);
static void SetDacChannel(unsigned chan, unsigned setvalue);
static unsigned long ReadFPGAID(void);
static void DoFpgaWatchdog(void);

void InitializeA35975(void);
void DoStateMachine(void);
void DoA35975(void);



//#define FPGA_ID     				0x1840  // used to verify GD FPGA communications
#define FPGA_ID     				0x0040  // used to verify GD FPGA communications
//#define FPGA_ID     				0x0880  // used to verify GD FPGA communications

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

  
  // ---- Configure the dsPIC ADC Module ------------ //
  ADPCFG = 0xffff;             // all are digital I/O

  control_state = STATE_START_UP;
   
  InitializeA35975();  

  T1CONbits.TON = 1;

  analog_sets[ANA_SET_EK].ip_set = 0;
  analog_sets[ANA_SET_EF].ip_set = 0;
  analog_sets[ANA_SET_EG].ip_set = 0; 
  _CONTROL_NOT_CONFIGURED = 1;
  _CONTROL_NOT_READY = 1;
  
  _STATUS_GD_HTR_NOT_READY = 1;
  _STATUS_GD_HTR_NOT_ENABLED = 1;
  _STATUS_GD_HV_NOT_ENABLED = 1;
  _STATUS_GD_TOP_NOT_ENABLED = 1;
  _STATUS_GD_TRIG_NOT_ENABLED = 1;


  while (1) {
  
    DoStateMachine();
    Do10msTicToc();  // Execute 10mS timed functions if the 10ms Timer has rolled
    ETMCanSlaveDoCan();

  }
}



void DoStateMachine(void) {
  static unsigned int delay_counter; 
  static unsigned int ef_ref_step;
  
#ifdef LOOPBACK_TEST
  unsigned char txData[8] = {1, 2, 3, 4, 7, 8, 5, 6};
#endif
  
  
  if ((control_state & 0x7f) > STATE_SYSTEM_HV_OFF) {
    
#ifndef TEST_MODE_BYP_FIBER_OFF
    if (PIN_OPT_HV_ENABLE1_INPUT != ILL_OPT_HV_ENABLE) {
      LogHvControl(0);
    }
    else if ((PIN_OPT_TRIG_ENABLE_INPUT != ILL_OPT_TRIG_ENABLE) && ((control_state & 0x7f) > STATE_SYSTEM_TRIG_OFF)) {
      LogTrigControl(0);    	
    }  
#endif
  }  	
  
  
  if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
    _STATUS_GD_HV_DISABLE = 1;
    if ((control_state & 0x7f) > STATE_SYSTEM_HV_OFF)  LogHvControl(0);

  }
  else
    _STATUS_GD_HV_DISABLE	= 0;

  
  switch(control_state) {
    
  case STATE_START_UP:

    system_byte = 0;
    
    analog_reads[ANA_RD_IF].read_f_hi = IF_READ_MAX;
    analog_reads[ANA_RD_EC].read_f_lo = 2150; // 120/0.05555
    analog_reads[ANA_RD_24V].read_f_hi = 3960; // 3600 + 360, 10% more 24V
    analog_reads[ANA_RD_24V].read_f_lo = 3240; // 3600 - 360, 10% less 24V     
    delay_counter = 0;    

#ifdef LOOPBACK_TEST
    PutResponseToBuffer(8, &txData[0]);
#endif
    control_state = STATE_START_UP2;
    break;

  case STATE_START_UP2:
    if (_T1IF) {
      // 100ms Timer over flow 
      _T1IF = 0;
      delay_counter++;
    }
    if (delay_counter > 10) {// 1s
      analog_reads[ANA_RD_EC ].read_m_lo = 1;
      analog_reads[ANA_RD_24V].read_m_lo = 1;
      analog_reads[ANA_RD_24V].read_m_hi = 1;
        
      delay_counter = 0;
      control_state = STATE_SYSTEM_HTR_OFF;
    }
    break;
    
  case STATE_SYSTEM_HTR_OFF:
    if (_CONTROL_NOT_CONFIGURED == 0) control_state = STATE_READY_FOR_HEATER;
    break;

  case STATE_READY_FOR_HEATER:
    system_byte |= SYS_BYTE_LOGIC_READY;
    htd_timer_in_100ms = 100; // SYSTEM_WARM_UP_TIME;
    ef_ref_step = 0;
    software_skip_warmup = 1; /* no local htd for linac gun driver */
    delay_counter = 0;
    
    // waiting htr on command from canopen
    if (analog_sets[ANA_SET_EF].ip_set >= EF_SET_MIN && htr_OVOC_rest_delay_timer_10ms == 0) {
      LogicHeaterControl(1);

      htr_OVOC_auto_reset_disable = 0;
    }

    break;

  case STATE_HEATER_STARTUP:
    if (_T1IF) {	 // runs every 100ms
      _T1IF = 0; 
      if (htd_timer_in_100ms) htd_timer_in_100ms--;
        
                    
      // heater ramp up
      if (ef_ref_step == 0) {
	ef_ref_step = 120; // around 512 steps for max Ef, (EF_SET_MAX >> 9);
	if (ef_ref_step < 10) ef_ref_step = 10;
	analog_sets[ANA_SET_EF].ip_set_alt = ef_ref_step;
	analog_sets[ANA_SET_EF].ip_set_flag = 1;
      }
      else if (analog_sets[ANA_SET_EF].ip_set_flag) {	                      
	if (analog_reads[ANA_RD_IF].read_cur <= IF_READ_MAX_90P) {            
	  analog_sets[ANA_SET_EF].ip_set_alt += ef_ref_step;
	  if (analog_sets[ANA_SET_EF].ip_set_alt >= analog_sets[ANA_SET_EF].ip_set) {
	    analog_sets[ANA_SET_EF].ip_set_alt = analog_sets[ANA_SET_EF].ip_set;
	    analog_sets[ANA_SET_EF].ip_set_flag = 0;
	  }   
	}
	if (htd_timer_in_100ms == 0) // 10s passed, check whether htr is shorted
	  {
	    if (analog_sets[ANA_SET_EF].ip_set_alt < EF_SET_MIN)
	      {  // htr is shorted
		FaultIf(2);	// call OC handler
		break;
	      }
	  }
                
	if (analog_sets[ANA_SET_EF].ip_set_alt >= EF_SET_MIN)
	  {
	    if (delay_counter < 200) 
	      {
		delay_counter++;
		if (delay_counter == 100)
		  {  // 10s after Ef > 1V without OV/OC, only set once at 10s delay time
	                    	
		    _STATUS_GD_HTR_NOT_READY = 0;
		    system_byte |= SYS_BYTE_HTR_WARMUP;        
		    PIN_LED_WARMUP = !OLL_LED_ON;
	                    
		  }
	      }
	  }
	/*
	  else if (analog_reads[ANA_RD_IF].read_cur > IF_READ_MAX_95P) {
	  if (analog_sets[ANA_SET_EF].ip_set_alt > ef_ref_step) {
	  analog_sets[ANA_SET_EF].ip_set_alt -= ef_ref_step; 
	  }
	  }
	*/
      }
        
           
      if (analog_sets[ANA_SET_EF].ip_set_flag) {
	SendHeaterRef(analog_sets[ANA_SET_EF].ip_set_alt);
      }
      else {
	analog_sets[ANA_SET_EF].ip_set_flag = 0;
	SendHeaterRef(analog_sets[ANA_SET_EF].ip_set);
	// send htr ref and htr on cmd
	//     htd_timer_in_100ms = SYSTEM_WARM_UP_TIME;
	// 	PIN_LED_WARMUP = OLL_LED_ON;
	//   analog_reads[ANA_RD_EF].read_m_lo = 1;
	htd_timer_in_100ms = 100; // 10s for heater to be stable
	control_state = STATE_WARM_UP;
            
			 
      }
    }       
    break;

  case STATE_WARM_UP:
    if (_T1IF) {
      if (htd_timer_in_100ms) htd_timer_in_100ms--;
      _T1IF = 0;
    }
	
    if (!htd_timer_in_100ms) {
      control_state = STATE_SYSTEM_HV_OFF;
        
      htr_OVOC_count = 0;        
      // Enable EfUV, IFOC
      analog_reads[ANA_RD_EF].read_m_lo = 1;
            
      software_skip_warmup = 0;
      _STATUS_GD_HTR_NOT_READY = 0;
        
      system_byte |= SYS_BYTE_HTR_WARMUP;        
      PIN_LED_WARMUP = !OLL_LED_ON;
    } 

    break;
    
  case STATE_SYSTEM_HV_OFF:
    // wait canopen HV on command
    if (sdo_hv_bypass) {
      control_state = STATE_SYSTEM_PULSETOP_OFF;
       
    }
#ifndef TEST_MODE_BYP_FIBER_OFF
    else if (PIN_OPT_HV_ENABLE1_INPUT == ILL_OPT_HV_ENABLE && _SYNC_CONTROL_RESET_ENABLE == 0) 
#else
    else if (_SYNC_CONTROL_RESET_ENABLE == 0)
#endif        
      control_state = STATE_READY_FOR_HV;
    break;
    
  case STATE_READY_FOR_HV:
    // wait canopen HV on command
#ifndef TEST_MODE_BYP_FIBER_OFF
    if (PIN_OPT_HV_ENABLE1_INPUT == ILL_OPT_HV_ENABLE && _SYNC_CONTROL_RESET_ENABLE == 0)
#else
    if (_SYNC_CONTROL_RESET_ENABLE == 0)
#endif        
      {
	system_byte |= SYS_BYTE_LOGIC_READY;
	delay_counter = 0;
	
	if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV == 0) LogHvControl(1);  // follow ethernet board to turn hv on
      } else {
      control_state = STATE_SYSTEM_HV_OFF;
      system_byte &= ~SYS_BYTE_LOGIC_READY;
    }
    
    break;
    
  case STATE_HV_STARTUP:
    
    
    if (_T1IF) {
      // 100ms Timer over flow 
      _T1IF = 0;
      delay_counter++;
    }
    
    if (delay_counter > 10) {
      delay_counter = 0;
      control_state = STATE_HV_ON;
    }
    
    break;
    
    
  case STATE_HV_ON:
    if (_T1IF) {
      // 100ms Timer over flow 
      _T1IF = 0;
    	delay_counter++;
    }
    
    if (delay_counter > 10) {
      delay_counter = 0;
      system_byte |= SYS_BYTE_HV_DRIVEUP;
      
      analog_reads[ANA_RD_EK].read_m_lo = 1;
      
	control_state = STATE_SYSTEM_PULSETOP_OFF;
    }
    
    break;
    
  case STATE_SYSTEM_PULSETOP_OFF:
    control_state = STATE_READY_FOR_PULSETOP;
    break;
    
  case STATE_READY_FOR_PULSETOP:
    // wait for pulse on cmd
    system_byte |= SYS_BYTE_LOGIC_READY;
    delay_counter = 0;
    
    // check Ek >= 5kv before grid on
    if (analog_reads[ANA_RD_EK].read_cur >= EK_RD_MIN_FOR_GRID_ON && analog_sets[ANA_SET_EG].ip_set >= EG_SET_MIN)  
      LogPulsetopControl(1);
    
    break;
    
  case STATE_PULSETOP_STARTUP:
    
    
    if (_T1IF) {
      // 100ms Timer over flow 
      _T1IF = 0;
      delay_counter++;
    }
    
    if (delay_counter > 10) {
      delay_counter = 0;
      control_state = STATE_PULSETOP_ON;
    }
    
    break;
    
  case STATE_PULSETOP_ON:
    
    // send out top ref and top on cmd
    if (_T1IF) {
      // 100ms Timer over flow 
      _T1IF = 0;
      delay_counter++;
    }
    
    if (delay_counter > 2) {
      delay_counter = 0;        
      
	control_state = STATE_SYSTEM_TRIG_OFF;
    }
    
    break;
    
  case STATE_SYSTEM_TRIG_OFF:
    
#ifndef TEST_MODE_BYP_FIBER_OFF
    if (PIN_OPT_TRIG_ENABLE_INPUT == ILL_OPT_TRIG_ENABLE)
#endif     
      control_state = STATE_READY_FOR_TRIG;
    
    break;
    
    
  case STATE_READY_FOR_TRIG:
    // wait for trig on cmd
#ifndef TEST_MODE_BYP_FIBER_OFF
    if (PIN_OPT_TRIG_ENABLE_INPUT == ILL_OPT_TRIG_ENABLE) 
#else
    if (1)     
#endif
      {     
	system_byte |= SYS_BYTE_LOGIC_READY;
	delay_counter = 0;
	
	LogTrigControl(1);  // turn trig on automatically
      } else {
      system_byte &= ~SYS_BYTE_LOGIC_READY;
      control_state = STATE_SYSTEM_TRIG_OFF;
    }
    break;
    
  case STATE_TRIG_STARTUP:
    
    // send out top ref and top on cmd
    if (_T1IF) {
      // 100ms Timer over flow 
      _T1IF = 0;
      delay_counter++;
    }
    
    if (delay_counter > 2) {
      delay_counter = 0;
      control_state = STATE_TRIG_ON;
    }
    
    break;
  
  case STATE_TRIG_ON:
    // highest state, can be turned off by user or a fault
    PIN_LED_GD_READY = OLL_LED_ON;
    PIN_OPT_GD_READY  = OLL_OPT_GD_READY;
    PIN_CAN_TRIGGER_ENABLE = OLL_CAN_TRIGGER_ENABLE; 

    _CONTROL_NOT_READY = 0;
    break;

        
  case STATE_FAULT_COLD_FAULT:
    if (htr_OVOC_auto_reset_disable == 0 && htr_OVOC_count > 0) {
      htr_OVOC_rest_delay_timer_10ms = 500;
      sdo_logic_reset = 1;  // htr OVOC autoreset
    }
    else if (_SYNC_CONTROL_RESET_ENABLE && htr_OVOC_count == 0)
      sdo_logic_reset = 1;
  
    if (sdo_logic_reset) {
      sdo_logic_reset = 0;
      DoFaultClear(FAULTS_TYPE_SYSTEM_CONTROL,   0xffff);
      DoFaultClear(FAULTS_TYPE_SOFTWARE,     0xffff);
      DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, 0xffff);
        
      // send reset to fpga board
      ResetFPGA();
      // clear fault status on CAN
      _FAULT_REGISTER = _FAULT_REGISTER & 0x04; // _FAULT_GD_SW_HTR_OVOC; //FAULTS_SW_EFOV_IFOC;

      PIN_LED_SUM_FAULT = !OLL_LED_ON;
        
        
      control_state = STATE_START_UP;
    }
    break;
  case STATE_FAULT_HOT_FAULT:
    // waiting for reset cmd
    if (_SYNC_CONTROL_RESET_ENABLE)
      sdo_logic_reset = 1;

    if (sdo_logic_reset) {
      sdo_logic_reset = 0;
      DoFaultClear(FAULTS_TYPE_SYSTEM_CONTROL,   0xffff);
      DoFaultClear(FAULTS_TYPE_SOFTWARE,         0xffff);
      DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, 0xffff);
        
      // clear fault status on CAN
      _FAULT_REGISTER = 0;

      if (analog_reads[ANA_RD_EF].read_m_lo > 1) analog_reads[ANA_RD_EF].read_m_lo = 1;
      if (analog_reads[ANA_RD_EG].read_m_hi > 1) analog_reads[ANA_RD_EG].read_m_hi = 1;
      if (analog_reads[ANA_RD_EK].read_m_hi > 1) analog_reads[ANA_RD_EK].read_m_hi = 1;
      if (analog_reads[ANA_RD_EK].read_m_lo > 1) analog_reads[ANA_RD_EK].read_m_lo = 1;


      PIN_LED_SUM_FAULT = !OLL_LED_ON;        
      if (last_control_state < 0x80) control_state = last_control_state;
      else                           control_state = STATE_START_UP;
    }     
    break;
    

  default:
    // DPARKER throw an ERROR
    _FAULT_GD_SYS_FAULTS = 1;
    DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_LOGIC_STATE);
    break;
  }
    
  if (control_state < 0x80) last_control_state = control_state;
    
}
  
void InitializeA35975(void) {
  unsigned int n;

 
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

 
  // Initialize all I/O Registers
  TRISA = A35975_TRISA_VALUE;
  TRISB = A35975_TRISB_VALUE;
  TRISC = A35975_TRISC_VALUE;
  TRISD = A35975_TRISD_VALUE;
  TRISF = A35975_TRISF_VALUE;
  TRISG = A35975_TRISG_VALUE;

  // config SPI1 for Gun Driver
  //ConfigureSPI(1, (A35975_SPI1CON_VALUE & A35975_SPI1CON_CLOCK), 0, A35975_SPI1STAT_VALUE, 1000000, FCY_CLK);  

  ConfigureSPI(ETM_SPI_PORT_1, ETM_DEFAULT_SPI_CON_VALUE, ETM_DEFAULT_SPI_CON2_VALUE, ETM_DEFAULT_SPI_STAT_VALUE, SPI_CLK_1_MBIT, FCY_CLK);  
  

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

  ResetAllFaults();

  // Initialize the Can module
  ETMCanSlaveInitialize(FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RG12, 4);
  ETMCanSlaveLoadConfiguration(35975, 250, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);


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

  // LED Indicator Output Pins, keep 24DC on
  PIN_LED_24DC_OK = OLL_LED_ON;  
      

}







/////////////////////////////////////////////////////////////////////////
// LogicHeaterControl() 
// turn on/off heater 
//
void LogicHeaterControl(unsigned char turnon)
{
    
  if (turnon) {
    if (control_state == STATE_READY_FOR_HEATER) {
      SetDacChannel(ANA_SET_HTR_ON, 0xffff);
      control_state = STATE_HEATER_STARTUP; 
            
      analog_sets[ ANA_SET_HTR_ON ].ip_set = 0xffff;

      system_byte |= SYS_BYTE_HTR_ON;
      system_byte &= ~SYS_BYTE_LOGIC_READY; 
            
      htd_timer_in_100ms = 100;  //SYSTEM_WARM_UP_TIME;
      PIN_LED_WARMUP = OLL_LED_ON; 
            
      _STATUS_GD_HTR_NOT_ENABLED = 0;
            
      analog_reads[ANA_RD_EF].read_m_hi = 1;
      analog_reads[ANA_RD_IF].read_m_hi = 1;
            
    }
        
  }
  else {
    LogHvControl(0);
    SetDacChannel(ANA_SET_HTR_ON,      0);

    analog_sets[ ANA_SET_HTR_ON      ].ip_set = 0;   
        
    SendHeaterRef(0);   
        
    PIN_LED_WARMUP = !OLL_LED_ON;
    htd_timer_in_100ms = SYSTEM_WARM_UP_TIME;
                
    sdo_hv_bypass = 0;  // reset hv bypass when htr is off
        
    analog_reads[ANA_RD_EF].read_m_hi = 0;
    analog_reads[ANA_RD_EF].read_m_lo = 0;
    analog_reads[ANA_RD_IF].read_m_hi = 0;
    analog_reads[ANA_RD_IF].read_m_lo = 0;

    last_control_state =  STATE_START_UP;

    if (control_state & 0x80)
      control_state = STATE_FAULT_COLD_FAULT;
    else
      control_state = STATE_START_UP;
            
    system_byte &= ~(SYS_BYTE_HTR_ON | SYS_BYTE_HTR_WARMUP);
    _STATUS_GD_HTR_NOT_ENABLED = 1;
    _STATUS_GD_HTR_NOT_READY = 1;

  }     

}
/////////////////////////////////////////////////////////////////////////
// LogHvControl() 
// turn on/off hv 
//
void LogHvControl(unsigned char turnon)
{
  if (turnon) {
    if (control_state == STATE_READY_FOR_HV) {
            
      // send out hv ref and hv on cmd
      SendHvRef(analog_sets[ANA_SET_EK].ip_set);

      SetDacChannel(ANA_SET_HV_ON, 0xffff);
      PIN_CAN_HV_ENABLE = OLL_CAN_HV_ENABLE;
      PIN_LED_HV_ENABLE = OLL_LED_ON;

      control_state = STATE_HV_STARTUP;       

      analog_sets[ ANA_SET_HV_ON ].ip_set = 0xffff;   
            
      analog_reads[ANA_RD_EK].read_m_hi = 1;   

      system_byte |= SYS_BYTE_HV_ON;
      system_byte &= ~SYS_BYTE_LOGIC_READY;  
            
      _STATUS_GD_HV_NOT_ENABLED = 0;
    }
        
  }
  else {
    LogPulsetopControl(0);
    SetDacChannel(ANA_SET_HV_ON,       0);
        
    analog_sets[ ANA_SET_HV_ON       ].ip_set = 0;      

    system_byte &= ~(SYS_BYTE_HV_ON | SYS_BYTE_HV_DRIVEUP);
    PIN_LED_HV_ENABLE = !OLL_LED_ON;
    PIN_CAN_HV_ENABLE = !OLL_CAN_HV_ENABLE;
        
    analog_reads[ANA_RD_EK].read_m_hi = 0;   
    analog_reads[ANA_RD_EK].read_m_lo = 0;   
        
    ekuv_timeout_10ms = 0;

    if ((last_control_state > STATE_SYSTEM_HV_OFF) && (last_control_state < 0x80)) last_control_state =  STATE_SYSTEM_HV_OFF;

    if (control_state & 0x80)
      control_state = STATE_FAULT_HOT_FAULT;
    else if (control_state > STATE_SYSTEM_HV_OFF)
      control_state = STATE_SYSTEM_HV_OFF;
        
    _STATUS_GD_HV_NOT_ENABLED = 1;    
        
  }     

}

/////////////////////////////////////////////////////////////////////////
// LogPulsetopControl() 
// turn on/off pulsetop 
//
void LogPulsetopControl(unsigned char turnon)
{
  if (turnon) {
    if (control_state == STATE_READY_FOR_PULSETOP) {
        
      // send out top ref and top on cmd
      SendPulsetopRef(analog_sets[ANA_SET_EG].ip_set);

      SetDacChannel(ANA_SET_PULSETOP_ON, 0xffff);
            
      PIN_CAN_PULSETOP_ENABLE = OLL_CAN_PULSETOP_ENABLE;

      control_state = STATE_PULSETOP_STARTUP;       

      analog_sets[ ANA_SET_PULSETOP_ON ].ip_set = 0xffff;      

      analog_reads[ANA_RD_EG].read_m_hi = 1;   

      system_byte |= SYS_BYTE_PULSETOP_ON;
      system_byte &= ~SYS_BYTE_LOGIC_READY;  
            
      _STATUS_GD_TOP_NOT_ENABLED = 0;   
    }
        
  }
  else {
    LogTrigControl(0);
    SetDacChannel(ANA_SET_PULSETOP_ON, 0);

    analog_sets[ ANA_SET_PULSETOP_ON ].ip_set = 0;      

    system_byte &= ~SYS_BYTE_PULSETOP_ON;
    PIN_CAN_PULSETOP_ENABLE = !OLL_CAN_PULSETOP_ENABLE;

        
    if ((last_control_state > STATE_SYSTEM_PULSETOP_OFF) && (last_control_state < 0x80)) last_control_state = STATE_SYSTEM_PULSETOP_OFF;

    if (control_state & 0x80)
      control_state = STATE_FAULT_HOT_FAULT;
    else if (control_state > STATE_SYSTEM_PULSETOP_OFF)
      control_state = STATE_SYSTEM_PULSETOP_OFF;
        
    _STATUS_GD_TOP_NOT_ENABLED = 1;   
  }     

}

/////////////////////////////////////////////////////////////////////////
// LogTrigControl() 
// turn on/off trig 
//
void LogTrigControl(unsigned char turnon)
{
    
  if (turnon) {
    if (control_state == STATE_READY_FOR_TRIG) {
      SetDacChannel(ANA_SET_TRIG_ON, 0xffff);
      control_state = STATE_TRIG_STARTUP;       

      analog_sets[ ANA_SET_TRIG_ON ].ip_set = 0xffff;      

      analog_reads[ANA_RD_EG].read_m_hi = 1;  // make sure faults are enabled 
      analog_reads[ANA_RD_EK].read_m_hi = 1;   
      analog_reads[ANA_RD_EK].read_m_lo = 1;   

      system_byte |= SYS_BYTE_TRIG_ON;
      system_byte &= ~SYS_BYTE_LOGIC_READY; 
      _STATUS_GD_TRIG_NOT_ENABLED = 0;    
    }
        
  }
  else {
    SetDacChannel(ANA_SET_TRIG_ON, 0);

    analog_sets[ ANA_SET_TRIG_ON     ].ip_set = 0;      

    system_byte &= ~(SYS_BYTE_TRIG_ON | SYS_BYTE_LOGIC_READY);
    PIN_LED_GD_READY = !OLL_LED_ON;
    PIN_OPT_GD_READY = !OLL_OPT_GD_READY;        
    PIN_CAN_TRIGGER_ENABLE = !OLL_CAN_TRIGGER_ENABLE;
        
    _CONTROL_NOT_READY = 1; 

    if ((last_control_state > STATE_SYSTEM_TRIG_OFF) && (last_control_state < 0x80)) last_control_state = STATE_SYSTEM_TRIG_OFF;

    if (control_state & 0x80)
      control_state = STATE_FAULT_HOT_FAULT;
    else if (control_state > STATE_SYSTEM_TRIG_OFF)
      control_state = STATE_SYSTEM_TRIG_OFF;
        
    _STATUS_GD_TRIG_NOT_ENABLED = 1;    
  }     

}

/////////////////////////////////////////////////////////////////////////
// SendHeaterRef() 
//  
//
void SendHeaterRef(unsigned int bits)
{	
  // need to check max, min
  SetDacChannel(ANA_SET_EF, bits);

}

/////////////////////////////////////////////////////////////////////////
// SendHvRef() 
//  
//
void SendHvRef(unsigned int bits)
{	
  // need to check max, min
  SetDacChannel(ANA_SET_EK, bits);

}

/////////////////////////////////////////////////////////////////////////
// SendPulsetopRef() 
//  
//
void SendPulsetopRef(unsigned int bits)
{	
  // need to check max, min
  SetDacChannel(ANA_SET_EG, bits);

}
/////////////////////////////////////////////////////////////////////////
// SendWatchdogRef() 
//  
//
void SendWatchdogRef(unsigned int bits)
{	
  // need to check max, min
  SetDacChannel(ANA_SET_WDOG, bits);
  analog_sets[ ANA_SET_WDOG ].ip_set = bits;  // for debug
    
}







void Do10msTicToc(void) {

  static unsigned char gd_read_ptr;  // cycle the readbacks from gd fpga board
  static unsigned long read_cycles = 0;
  static unsigned char watch_dog_timer = 0;

  static unsigned int sec_count = 0;

  unsigned long temp;
  unsigned int i, bit_value;
  
  /*
    Certain functions need to happen at regular interval for the system to work

    Thyratron PIDs - The gain and phase of the PID control loop is a function of it's execution frequency therefor it must be updated at a regular interval
    Analog Filters - The filter response is function of the execution frequency so they must be executed at a regular interval

    DAC updates - The DAC must be regularly.  Durring HV ON this should happen AFTER a pulse so that the SPI bus is not corrupted by EMI
    If the state is not in HV_ON or the system is pulsing at a very low freqeuncy, DAC updates must be handeled by this function.
    
    Calculating the PRF

    Other timing functions like flashing LEDs
  */

  last_known_action = LAST_ACTION_DO_10MS;


  ClrWdt();
  
  // update one value from gd fpga board, watchdog channel is handled by watchdog kicking
  if (gd_read_ptr < 12)	{	// don't care ch9, 13, 14
   	 
    temp = ReadAdcChannel(gd_read_ptr);

    if (gd_read_ptr <= 8) {
      analog_reads[ gd_read_ptr ].read_cur = temp;
      analog_reads[ gd_read_ptr ].read_cnt++;
      CheckAnalogLimits(gd_read_ptr);
    }    
    else if (gd_read_ptr >= 10 && gd_read_ptr <= 11) {	// only care about wdog and arc faults 
      i = gd_read_ptr - 9;
      if (temp <= 2500)   faults_from_ADC |= (1 << i);
      else				faults_from_ADC &= ~(1 << i);
            
      if (digi_reads[i].action_code < 99) {
	digi_reads[i].state = (temp > 2500);
                
	if (temp <= 2500) {
	  DoFaultRecord(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i)); // mapped to fpgaID bit 1 & 2
	}
      }
    }
     
  }
  else {
     
    temp = ReadFPGAID(); 
    // check if the ID read is valid
    if ((temp & 0x03c0) == (FPGA_ID & 0x03c0)) {  // check FPGA major version only.  0x40 for the low byte, two bits stay 0 for the 2nd low byte, xxxxxx00 01000000
      temp >>= 16;

      fpga_ASDR = (unsigned int)temp;
      // handle digi faults from fpgaid
      for (i = 0; i < 16; i++)
	{
	  bit_value = (temp & (1 << i)) > 0;
	  if (digi_reads[ i + DIGI_ID_ARC_COUNT].action_code < 99) {
	    digi_reads[ i + DIGI_ID_ARC_COUNT].state = bit_value;
	    if (bit_value)
	      DoFaultRecord(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i));
	    else if (digi_reads[ i + DIGI_ID_ARC_COUNT].action_code == 0)    
	      DoFaultClear(FAULTS_TYPE_DIGI_FROM_FPGAID, (1 << i));
	  }
               
	}
         
      PIN_LED_AC_ON = OLL_LED_ON;    
        
    }
    else { // declare a fault
      PIN_LED_AC_ON = !OLL_LED_ON;    
      DoFaultRecord(FAULTS_TYPE_SYSTEM_CONTROL, FAULTS_SYS_FPGAID);
    }
     
  }
  
  if (gd_read_ptr == 8)	{
    gd_read_ptr = 9; // bypass ch9
  }
  else if (gd_read_ptr == 11) {
    gd_read_ptr = 14; // bypass ch12, 13, 14
  } 
  
  gd_read_ptr = (gd_read_ptr + 1) & 0x000f;	 // 0 to 15 for gd_read_ptr
  
  if (!gd_read_ptr)  read_cycles++;	 
  
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
    if (watch_dog_timer >= 3) 
      {
	DoFpgaWatchdog();  // tickle the watchdog every 30ms
	watch_dog_timer = 0;
      }
    
    // record debug data for CAN bus    
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

   
    if (ekuv_timeout_10ms) {
      if (ekuv_timeout_10ms < 1000) ekuv_timeout_10ms++;
    }
        
   
    if (ek_ref_changed_timer_10ms) ek_ref_changed_timer_10ms--;
    if (ef_ref_changed_timer_10ms) ef_ref_changed_timer_10ms--;
    if (eg_ref_changed_timer_10ms) eg_ref_changed_timer_10ms--;

    if (htr_OVOC_rest_delay_timer_10ms) htr_OVOC_rest_delay_timer_10ms--; 

    
    if ((_FAULT_REGISTER &  0x04 /*_FAULT_GD_SW_HTR_OVOC */) && (htr_OVOC_count == 0)) {
      if (_SYNC_CONTROL_RESET_ENABLE  && ((faults_reg_software & FAULTS_SW_EFOV_IFOC) == 0))
	  	   
	_FAULT_REGISTER = _FAULT_REGISTER & (~0x04/* _FAULT_GD_SW_HTR_OVOC */);
    }
        
        
    led_pulse_count = ((led_pulse_count + 1) & 0b00111111);	 // 640ms
    if (led_pulse_count == 0) {
      // 10ms * 16 counter has ocurred
      // Flash the LED - NOTE "PIN_MAIN_CONTACTOR_CLOSE = !PIN_MAIN_CONTACTOR_CLOSE" was causing any changes made in Port F durring interrupt to be overwritten
      if (PIN_LED_LAST_PULSE_GOOD) {
	PIN_LED_LAST_PULSE_GOOD = 0;
      } else {
	PIN_LED_LAST_PULSE_GOOD = 1;
      }  
      
    }
    
    
  } 
}



#if 0 // copied to ReadAdcChannel function
/////////////////////////////////////////////////////////////////////////
// SetupGDadc() setup the ADC chip on the Gun Driver board
// 
//
static void SetupAdc(void)
{
  unsigned int temp;
 
  temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
  temp |= PIN_CS_ADC_ENABLE_BIT;
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

 
  //   temp = 0x78;  // clock mode 11(single conv), Ref mode 10 (internal, always on), no differential inputs 
  temp = 0x74;  // clock mode 11(single conv), Ref mode 01 (external single ended), no differential inputs 
  //   temp <<= 8; // high 8 bit out

  //  	spiSend((~mode) & 0xFF00); 
  temp = ~temp; 
  SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

  temp = LATD & 0x1fff; // clear all CS bits
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

		
  return;    
}
#endif


#ifdef DEMO
/////////////////////////////////////////////////////////////////////////
// CalculateDemoAdc() 
//  
//
static unsigned CalculateDemoAdc(unsigned chan)
{
  unsigned ret = 0;
  double temp = 0;
  static double little_change = -0.002; 
    
   
    
  switch (chan) {
  case ANA_RD_EK:
    if ((control_state & 0x7f) >= STATE_HV_STARTUP) {
      temp = (double)analog_sets[ANA_SET_EK].ip_set * (-0.0003333);	// ref v
      temp += temp * little_change;
      temp /= -0.005555;
    }
    break;
  case ANA_RD_IKA:
    if ((control_state & 0x7f) >= STATE_HV_STARTUP) {
      temp = 202.2;
      temp += temp *little_change;            
      temp = temp/0.001667;
    }
    break;
  case ANA_RD_IKP:
    if ((control_state & 0x7f) >= STATE_HV_STARTUP) {
      temp = 602.5;
      temp += temp * little_change;            
      temp = temp/0.277;
    }
    break;
  case ANA_RD_EF:
    //	if ((control_state & 0x7f) >= STATE_HEATER_STARTUP) {
    temp = analog_sets[ANA_SET_EF].ip_set_flag? analog_sets[ANA_SET_EF].ip_set_alt : analog_sets[ANA_SET_EF].ip_set;
    temp *= 0.000133;	// ref v
    temp += temp * little_change;            
    temp /= 0.00222;
    //    }
    break;
  case ANA_RD_IF:
    if ((control_state & 0x7f) >= STATE_HEATER_STARTUP) {
      temp = 2.5;
      temp += temp * little_change;
      temp /= 1.667e-3;
    }
    break;
  case ANA_RD_EG:
    if ((control_state & 0x7f) < STATE_PULSETOP_STARTUP) {
      temp = 80;
      temp /= 0.1111;
    }           
    else {
      temp = (double)analog_sets[ANA_SET_EG].ip_set * 0.00666 - 80;  // -80 offset
      temp += temp * little_change;            
      temp += 80;
      temp /= 0.1111;
    }
    break;
  case ANA_RD_EC:
    temp = -150.5;
    temp += temp * little_change;
    temp /= -55.55e-3;
    break;
  case ANA_RD_24V:
    temp = 24;
    temp += temp * little_change;            
    temp /= 0.00666;            
    break;
  case ANA_RD_TEMP:
    temp = 40;
    temp += temp * little_change; 
    temp /= 0.0133;           
    break;
     
  default:
    temp = 0xfff; // set high for no fault
    break;
  }

  if (temp > 0xfff) temp = 0xfff;
  else if (temp < 0) temp = 0;
    
  ret = (unsigned)(temp + 0.5);

  little_change += 0.001;
  if (little_change > 0.002) little_change = -0.002; // change between -0.002 to 0.002
  return (ret);
}
#endif
/////////////////////////////////////////////////////////////////////////
// ReadAdcChannel() 
// read ADC one channel 
//
static unsigned ReadAdcChannel(unsigned chan)
{
  unsigned int temp, ret, adc_val1, adc_val2;
 
  // enable ADC chip select
  temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
  temp |= PIN_CS_ADC_ENABLE_BIT;
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

  // always reset ADC mode first
  temp = 0x10;  // clock mode 11(single conv), Ref mode 01 (external single ended), no differential inputs 
  temp = ~temp; 
  SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

  // always setup ADC mode second
  temp = 0x74;  // clock mode 11(single conv), Ref mode 01 (external single ended), no differential inputs 
  temp = ~temp; 
  SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

  // send out conv command
  temp = 0x80 | (chan << 3) | 0x06; // conv enable, no scan, no temp measurement
  temp = ~temp;
  ret = SendAndReceiveSPI(temp & 0x00ff, ETM_SPI_PORT_1);     // send the setup byte out

  // get the msb
  adc_val1 = ~SendAndReceiveSPI(0xffff, ETM_SPI_PORT_1) & 0x00ff;
  adc_val2 = ~SendAndReceiveSPI(0xffff, ETM_SPI_PORT_1) & 0x00ff;
    
  // clear all CS bits
  temp = LATD & 0x1fff; 
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate


  ret = ((adc_val1 << 8) | adc_val2) & 0x0fff;

#ifdef DEMO
  // calculate the channel readback
  ret = CalculateDemoAdc(chan);
#endif    

  return (ret); 
}


/////////////////////////////////////////////////////////////////////////
// SetDacChannel() 
// set DAC one channel, used 8 bit SPI instead of 16bit, otherwise,
// SPI would get a reset when bit config is changed.
//
static void SetDacChannel(unsigned chan, unsigned setvalue)
{
  unsigned int temp, ret;
 
  if (chan >= 8) return;
  if (setvalue > 65535) setvalue = 65535;
    
  // enable DAC chip select
  temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
  temp |= PIN_CS_DAC_ENABLE_BIT;
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

  temp = 0x30 | chan; // conv enable, no scan, no temp measurement
  //   temp <<= 8;  // 8 bit out
  temp = ~temp;
  ret = SendAndReceiveSPI(temp, ETM_SPI_PORT_1);     // send the setup byte out

  // set setvalue out
  temp = (setvalue >> 8) & 0x00ff;
  temp = ~temp;
  ret  = SendAndReceiveSPI(temp, ETM_SPI_PORT_1);	   // high setbyte sent out first
  temp = setvalue & 0x00ff;
  temp = ~temp;
  ret  = SendAndReceiveSPI(temp, ETM_SPI_PORT_1);
    
  // clear all CS bits
  temp = LATD & 0x1fff; 
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);   // Wait for the cable select signal to propagate

  ret = ret; // used		
  return;    
}

/////////////////////////////////////////////////////////////////////////
// ReadFPGAID() 
// read FPGA ID, high 16 bits are fault information, low 16 bits are rev information. 
// used 8 bit SPI instead of 16bit, otherwise, SPI would get a reset when bit config is changed.
//
static unsigned long ReadFPGAID(void)
{
  unsigned i;
  unsigned int temp;
  unsigned long ret = 0;
 
  // enable Aux chip select
  temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
  temp |= PIN_CS_AUX_ENABLE_BIT;
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

  for (i = 0; i < 4; i++)
    {
      ret <<= 8;
      ret |= (~SendAndReceiveSPI(0xffff, ETM_SPI_PORT_1) & 0x00ff);
    }    
  // clear all CS bits
  temp = LATD & 0x1fff; 
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

#ifdef DEMO
  ret = FPGA_ID;
#endif       

  return (ret);

}

/////////////////////////////////////////////////////////////////////////
// ResetFPGA() 
// send a reset to FPGA board 
//
void ResetFPGA(void)
{
  unsigned int temp;
 
  // enable ADC chip select
  temp = LATD & 0x1fff; // D13 for DAC, D14 for ADC, D15 for Aux CS
  temp |= PIN_CS_ALL_ENABLE_BITS;
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

  // clear all CS bits
  temp = LATD & 0x1fff; 
  LATD = temp;
  __delay32(DELAY_PULSE_CABLE_SELECT_PROP_DELAY);        // Wait for the cable select signal to propagate

  return;
}

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


/////////////////////////////////////////////////////////////////////////
// SetEk() 
// Change Ek Ref  
//
void SetEk(unsigned set_value)
{
  static unsigned last_set_value = 0;
    
  if (set_value > EK_SET_MAX)  set_value = EK_SET_MAX;
     
  if (last_set_value != set_value) 
    { 
      last_set_value = set_value;
      analog_sets[ANA_SET_EK].ip_set = set_value;
      SetEkLimits();
      SendHvRef(analog_sets[ANA_SET_EK].ip_set);
               
    }

}


/////////////////////////////////////////////////////////////////////////
// SetEg() 
// Change Pulse Top Ref  
//
void SetEg(unsigned set_value)
{
  static unsigned last_set_value = 0;

  if (set_value > EG_SET_MAX)  set_value = EG_SET_MAX;
                
  if (last_set_value != set_value) 
    { 
      last_set_value = set_value;
      analog_sets[ANA_SET_EG].ip_set = set_value;
      SetEgLimits();        
      SendPulsetopRef(analog_sets[ANA_SET_EG].ip_set);
      if (set_value < EG_SET_MIN && (system_byte & SYS_BYTE_PULSETOP_ON) > 0)
	LogPulsetopControl(0);
    }
            
}


/////////////////////////////////////////////////////////////////////////
// SetEf() 
// Change Ef Ref  
//
void SetEf(unsigned set_value)
{
  static unsigned last_set_value = 0;
  if (set_value > EF_SET_MAX)  set_value = EF_SET_MAX;
                
  if (last_set_value != set_value) { 
    last_set_value = set_value;
    analog_sets[ANA_SET_EF].ip_set = set_value;
    SetEfLimits();
    if (set_value >= EF_SET_MIN) {
      if ((control_state & 0x7f) >= STATE_WARM_UP) // send htr ref directly if htr dly started.
	SendHeaterRef(analog_sets[ANA_SET_EF].ip_set);
    }
    else if ((system_byte & SYS_BYTE_HTR_ON) > 0) 
      LogicHeaterControl(0);  // turn off heater if ref < 1V
  }
            
}

/////////////////////////////////////////////////////////////////////////
// unsigned char AreAllReferencesConfigured() 
// Change Ef Ref  
//
unsigned char AreAnyReferenceNotConfigured(void)
{
  unsigned char ret = 0;
    
  if (analog_sets[ANA_SET_EK].ip_set == 0) ret = 1;
  else if (analog_sets[ANA_SET_EF].ip_set == 0) ret = 1;
  else if (analog_sets[ANA_SET_EG].ip_set == 0) ret = 1;
    
  return (ret);
}




// ------------------------------- from faults.c







void LoadFaultMaskRegisters(void);
void WriteToEventLog(unsigned char fault_register, unsigned int fault_bit);



/*
  FAULT EVALUATION
  
  10_ms_tic_toc
  Once every 10ms, the internal ADC is read and faults are updated

  After Every Pulse

  DPARKER give a big description of each fault here - How the input is measured, how the data is filtered, how the fault is generated (delay if included)

*/


/*
  In each State, Each fault input can do one of the following . . .

  1) It can be totally ignored (This is the default behavior - If the warning mask & the fault mask are NOT set)
  2) It can cause a fault  (This is set by the fault MASK)
  There are no "warm" or "cold" faults.  Any fault in state warm_ready or hv_on will go to state warm fault.
  If the fault is still active in the warm state fault then it will move to cold fault.
  3) It can cause a latched warning (This is set the warning MASK)
  NOTE: the warning register is independent of the fault mask.  To be latched as a warning, the fault input MUST BE IN THE WARNING REGISTER
  4) Special state at startup to handle board level failures . . . ????
*/

/*  
    Fault Log
    
    Each entry in the fault log contains the following information
    4 bits for fault register bit
    2 bits for fault register select
    2 bits for the calling state (HV_ON, WARM_READY, FAULT, STARTUP)
*/

/*
  For each state there are three fault masks for each fault register.
  Warning Ignore Mask - This shows which fault inputs will NOT generate a warning (this is mainly for debugging and and is used to filter out non-events, if a fault is in the warning ignore mask and in the warm or cold fault mask it will still generate a fault)
  Faul Mask - this shows which fault inputs will generate a fault in the current state


*/

/*

  Status_Register -> This register shows the current states of all the Inputs.  It IS NOT LATCHED.
  Fault_Register -> This register latches any fault input that matches the fault_mask for the current state.
  Warning_Latch_Regisiter -> Any fault input that matches the fault_mask or the warning_mask will be latched in this register.

  How faults are checked . . .
  
  Every time through a control loop, 50us->500us depending upon the state, all faults registeres are updated and checked.
  Faults like magnetron_heater_over_current - The limit is compared to the filtered data in RAM every time, even though the filtered data in ram is only update once every 10mS
  The fault status register is then compared to the masks to generate the warnings and faults.

*/

/* ----------------------FAULT MANAGEMENT ----------------------*/

/*
  There are lots of Fault Conditions, see A34335.h for a list of fault registeres and faults.
  For a given state a particular Input may
  1) No Action - But set a non latching status bit indicating the state of the fault input
  2) No Action - But set a latching "warning" bit 
  3) Generate a fault Condition - This will change the state to warm fault (or cold fault if the fault is globably defined as a cold fault)
 

  How are faults handeled . . .
  There a 3 data storage locations for each fault register.
  Status_Register -> This register shows the current states of all the Inputs.  It IS NOT LATCHED.
  Latch_Regisiter -> This is a latched version of the the status register.
  Fault_Register  -> If the Input is a Fault (it matches the warm_fault OR cold_fault mask) it is latched into the Fault Register
  
  How external data and faults are processesed . . . 

  Data is read from the external ADCs and inernal ADCs on a regular 10ms interval or after a pulse (for pulse data types).
  This inerval is scheduled to occur directly proceding a pulse if possible (pulse rate > 100 Hz).
  At PRF less than 100Hz (or in any other state) the readings will take place at opproximate 10ms intervals
  ADC values are filtered with software RC/glitch filters when they are read.
 
 
  Faults are evaluated durring the 10ms_tic_toc the occurs once every (approximatly) 10mS
  The following steps occur
  1) Status registers are reset to zero
  2) The input condition is tested, if it is a "logical fault" then (record_this_xxxxx_fault) is called which does the following
  a) Sets the appropriate bit in the status register
  b) If the bit matches the fault_mask, 
  ^ The appropriate bit in the fault register is set
  ^ The fault is added to the error log - TO BE IMPLEMENTED
  c) If the bit matches the warning_mask, the appropriate bit in the warning register is set 
  
  STEP 2 is repeated for all fault conditions.

  Some faults can not be tested durring the 10ms TicToc.
  These faults will have record_this_xxxxx_fault called when the fault is checked.
  These faults are . . .
  FAULT_HV_LAMBDA_EOC_TIMEOUT - this is evaluated in the TMR1 interrupt and set there.


  After all the faults have been tested and the fault registers updated the following action occurs
  
  + A cold shutdown occurs
  + A warm shutdown occurs
  
  + The fault is added to the Log
  + For *some* important faults, the fault counter is incremented.

  + The fault is added to the Log
  + For *some* important faults, the fault counter is incremented.
  
*/

/////////////////////////////////////////////////////////////////////////
// DoFaultAction() 
// take actions	when faulting
// 1: htr off, 2: hv off, 3: pulsetop off, 4: trig off, otherwise, no action
//
void DoFaultAction(unsigned char type, unsigned char disable_htr_auto_reset) {
  switch (type) {
  case 1:
    LogicHeaterControl(0);
    if (disable_htr_auto_reset) {
      htr_OVOC_auto_reset_disable = 1;
      //		htr_OVOC_count = 0;
    }
    //   if (control_state < 0x80) last_control_state = control_state;
    control_state = STATE_FAULT_COLD_FAULT;
    break;
  case 2:
    LogHvControl(0);
    //    if (control_state < 0x80) last_control_state = control_state;
    control_state = STATE_FAULT_HOT_FAULT;
    break;
  case 3:
    LogPulsetopControl(0);
    //    if (control_state < 0x80) last_control_state = control_state;
    control_state = STATE_FAULT_HOT_FAULT;
    break;
  case 4:
    LogTrigControl(0);
    //   if (control_state < 0x80) last_control_state = control_state;
    control_state = STATE_FAULT_HOT_FAULT;
    break;
  default:
    break;
  }
   
}   
/////////////////////////////////////////////////////////////////////////
// DoFaultRecord() 
// record fault and take actions
//
void DoFaultRecord(unsigned int fault_type, unsigned int fault_bit) {
   
  unsigned idx;		    
																		     
  switch (fault_type) {
    												  
  case FAULTS_TYPE_SYSTEM_CONTROL:    
    if ((faults_reg_system_control & fault_bit) == 0) {
      faults_reg_system_control |= fault_bit;	
      // action
      DoFaultAction(1, 1);
            
      if (fault_bit & (FAULTS_SYS_FPGAID |	FAULTS_SYS_FPGA_WATCHDOG_ERR)) 
	_FAULT_GD_FPGA_COMM_LOST = 1;
    }			   	
    break;												   
	
  case FAULTS_TYPE_SOFTWARE:
    if ((faults_reg_software & fault_bit) == 0) {
      faults_reg_software |= fault_bit;	
      // action
    }			   	
    break;
	
  case FAULTS_TYPE_DIGI_FROM_FPGAID:
#ifndef TEST_BYP_FPGA_FAULTS
    if ((faults_reg_digi_from_gd_fpgaid & fault_bit) == 0) {
      faults_reg_digi_from_gd_fpgaid |= fault_bit;
      // action
      if (fault_bit == 0x0002 || fault_bit == 0x0004) {  // wdog and arc faults from ADC
	if (fault_bit > 2)  _FAULT_GD_FPGA_ARC_FAULT = 1;
	else				 _FAULT_GD_FPGA_COMM_LOST = 1;
                 
	idx = (fault_bit >> 1);
	DoFaultAction(digi_reads[idx].action_code, 1);
      }
      else { 
	for (idx = 0; idx < 15; idx++) {
	  if (fault_bit == (1 << idx)) break;
	}
	if (idx < 15) {	// found the index
	  idx += DIGI_ID_ARC_COUNT;
	  switch (idx) {
	  case DIGI_ID_TEMP_75C:
	    _FAULT_GD_FPGA_TEMP_75C = 1;
	    break;
	  case DIGI_ID_PRF:
	  case DIGI_ID_CURR_PW:
	    _FAULT_GD_FPGA_PULSE_FAULT = 1;
	    break;
 
	  case DIGI_ID_GRID_HW:
	  case DIGI_ID_GRID_OV:
	  case DIGI_ID_GRID_UV:
	  case DIGI_ID_BIAS_V:
	    _FAULT_GD_FPGA_GRID_FAULT = 1;
	    break;

	  default:
	    break;

	  }

	  DoFaultAction(digi_reads[idx].action_code, 1);
	}
      }
    }
#endif        			   	
    break;
	
  default:
    break;
  }        
		
  if (faults_reg_system_control || faults_reg_software || (faults_reg_digi_from_gd_fpgaid & FPGAID_FAULTS_MASK))	{
    PIN_LED_SUM_FAULT = OLL_LED_ON; 
    system_byte |= SYS_BYTE_FAULT_ACTIVE; 
    _FAULT_GD_SUM_FAULT = 1;       
  }     
  else {
    PIN_LED_SUM_FAULT = !OLL_LED_ON;  
    system_byte &= ~SYS_BYTE_FAULT_ACTIVE;
    _FAULT_GD_SUM_FAULT = 0;        
  }
        
}		
/////////////////////////////////////////////////////////////////////////
// DoFaultClear() 
// clear fault and let logic continue
//
void DoFaultClear(unsigned int fault_type, unsigned int fault_bit) {		    
																		     
  switch (fault_type) {
    												  
  case FAULTS_TYPE_SYSTEM_CONTROL:
    faults_reg_system_control &= ~fault_bit;	
    break;												   
	
  case FAULTS_TYPE_SOFTWARE:
    faults_reg_software &= ~fault_bit;	
    break;
	
  case FAULTS_TYPE_DIGI_FROM_FPGAID:
    faults_reg_digi_from_gd_fpgaid &= ~fault_bit;
    break;
	
  default:
    break;
  } 
    
  if (faults_reg_system_control || faults_reg_software || (faults_reg_digi_from_gd_fpgaid & FPGAID_FAULTS_MASK))	{
    PIN_LED_SUM_FAULT = OLL_LED_ON;
    system_byte |= SYS_BYTE_FAULT_ACTIVE;        
    _FAULT_GD_SUM_FAULT = 1;       
  }      
  else {
    PIN_LED_SUM_FAULT = !OLL_LED_ON;      
    system_byte &= ~SYS_BYTE_FAULT_ACTIVE;        
    _FAULT_GD_SUM_FAULT = 0;       
  }
		
}		


/////////////////////////////////////////////////////////////////////////
// CheckAnalogLimits() check analog over/under limits
// 
//
void CheckAnalogLimits(unsigned index) {

  /* test for foo. */
  if (index < ANALOG_READ_SIZE) {
    /* test if over the max */
    if (analog_reads[index].read_m_hi &&
	(analog_reads[index].read_cur > analog_reads[index].read_f_hi))	{

      if (analog_reads[index].read_m_lo > 1)  /*  if low was in service */
	analog_reads[index].read_m_lo = 1;   /* reset it               */

      /* faulted once already - trip fault this time */
      if (analog_reads[index].read_m_hi == 3){

	analog_reads[index].read_m_hi = 2;       /* set in service mode */

	if (analog_reads[index].fault_vect)      /* test for vector */
	  (analog_reads[index].fault_vect)(2); /* call if so send hi signal*/

	if (analog_reads[index].read_m_lo == 2)  /*  if low was in service */
	  analog_reads[index].read_m_lo = 1;   /* reset it               */

      }

      /* if not masked and not in service */
      if (analog_reads[index].read_m_hi == 1){

	analog_reads[index].read_m_hi = 3;       /* set one free trip */

	return;

      }

    }

    /* test if under the min */
    else if (analog_reads[index].read_m_lo &&
	     (analog_reads[index].read_cur < analog_reads[index].read_f_lo)) {

      if (analog_reads[index].read_m_hi > 1)  /*  if hi was in service */
	analog_reads[index].read_m_hi = 1;   /* reset it               */

      /* if TRIP + HAD ONE FREE ALREADY */
      if (analog_reads[index].read_m_lo == 3){

	analog_reads[index].read_m_lo = 2;       /* set in service mode */

	if (analog_reads[index].fault_vect)      /* test for vector */
	  (analog_reads[index].fault_vect)(1); /* call if so send lo signal*/

	if (analog_reads[index].read_m_hi == 2)  /*  if hi was in service */
	  analog_reads[index].read_m_hi = 1;   /* reset it               */

      }

      /* if not masked and not in service */
      if (analog_reads[index].read_m_lo == 1){

	analog_reads[index].read_m_lo = 3;       /* set ONE FREE TRIP mode */

	return;
      }

    }
		    
    /* test for clear                   */
    /* could be 2 TRIP or 3 - one time  */
    /* one free fault added             */
    else if ((analog_reads[index].read_m_hi > 1) || (analog_reads[index].read_m_lo > 1)) {

      /* log event - reset happened after only one reading */
      if ((analog_reads[index].read_m_hi == 3) || (analog_reads[index].read_m_lo == 3)) 
	analog_reads[index].events++;

      if (analog_reads[index].fault_vect)      /* test for vector */
	(analog_reads[index].fault_vect)(0); /* call if so send lo signal*/

      if (analog_reads[index].read_m_hi)
	analog_reads[index].read_m_hi = 1;     /* reset in-services - cleared or masked */

      if (analog_reads[index].read_m_lo)
	analog_reads[index].read_m_lo = 1;         

    }

  }  // index < ANALOG_READ_SIZE


}

/////////////////////////////////////////////////////////////////////////
// FaultEk() 
//  
//
void FaultEk(unsigned state) {

  unsigned char fault_just_happened = 0;
    
  switch (state) {
    
  case 1: // under
    if (ekuv_timeout_10ms < 220) {
      if (!ekuv_timeout_10ms) ekuv_timeout_10ms++;
      if (analog_reads[ANA_RD_EK].read_m_lo > 1) analog_reads[ANA_RD_EK].read_m_lo = 1;
      break; // no faulting this time
    }
    if (ek_ref_changed_timer_10ms) {
      if (analog_reads[ANA_RD_EK].read_m_lo == 2) analog_reads[ANA_RD_EK].read_m_lo = 1;
      if (analog_reads[ANA_RD_EK].read_m_hi == 2) analog_reads[ANA_RD_EK].read_m_hi = 1;
      break; // no faulting this time, armed for next time     
    }
    else {
      _FAULT_GD_SW_EK_UV = 1;
      fault_just_happened = 1;
    }
    break;
      	  
  case 2: // over, and under pass through
    if (ek_ref_changed_timer_10ms) {
      if (analog_reads[ANA_RD_EK].read_m_lo == 2) analog_reads[ANA_RD_EK].read_m_lo = 1;
      if (analog_reads[ANA_RD_EK].read_m_hi == 2) analog_reads[ANA_RD_EK].read_m_hi = 1;
      break; // no faulting this time, armed for next time     
    }
    else {
      _FAULT_GD_SW_EK_OV = 1; 
      fault_just_happened = 1;
    }    
    break;
      
  default:
    ekuv_timeout_10ms = 0;
    break;
      
  }
  if (fault_just_happened) {
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EKOV_EKUV);
    DoFaultAction(2, 0);
  }

}
/////////////////////////////////////////////////////////////////////////
// FaultEf() 
//  
//
void FaultEf(unsigned state) {

  switch (state) {
    
  case 1: // under
#ifndef TEST_BYP_FPGA_FAULTS
    if (ef_ref_changed_timer_10ms) {
      if (analog_reads[ANA_RD_EF].read_m_lo == 2) analog_reads[ANA_RD_EF].read_m_lo = 1;
      break; // no faulting this time, armed for next time     
    }
    _FAULT_GD_SW_HTR_UV = 1; // QQ: not fault, warning only?
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EFUV);
    DoFaultAction(2, 0);
#endif
    break;
  case 2: // over
#ifndef TEST_BYP_FPGA_FAULTS
    if (ef_ref_changed_timer_10ms) {
      if (analog_reads[ANA_RD_EF].read_m_hi == 2) analog_reads[ANA_RD_EF].read_m_hi = 1;
      break; // no faulting this time, armed for next time     
    }
    _FAULT_GD_SW_HTR_OVOC = 1;
    htr_OVOC_count++;
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EFOV_IFOC);
    DoFaultAction(1, htr_OVOC_count >= 5? 1 : 0);
      
#endif
    break;
  default:
    break;
      
  }
    
}
/////////////////////////////////////////////////////////////////////////
// FaultIf() 
//  
//
void FaultIf(unsigned state) {

  switch (state) {
    
  case 1: // under
    break;
  case 2: // over
#ifndef TEST_BYP_FPGA_FAULTS
    if (ef_ref_changed_timer_10ms) {
      if (analog_reads[ANA_RD_IF].read_m_hi == 2) analog_reads[ANA_RD_IF].read_m_hi = 1;
      break; // no faulting this time, armed for next time     
    }
    _FAULT_GD_SW_HTR_OVOC = 1;
    htr_OVOC_count++;
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EFOV_IFOC);
    DoFaultAction(1, htr_OVOC_count >= 5? 1 : 0);
#endif
    break;
  default:
    break;
      
  }
    
}
/////////////////////////////////////////////////////////////////////////
// FaultEg() 
//  
//
void FaultEg(unsigned state) {

  switch (state) {
    
  case 1: // under
    break;
  case 2: // over
    if (eg_ref_changed_timer_10ms) {
      if (analog_reads[ANA_RD_EG].read_m_hi == 2) analog_reads[ANA_RD_EG].read_m_hi = 1;
      break; // no faulting this time, armed for next time     
    }
    _FAULT_GD_SW_GRID_OV = 1;
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_EGOV);
    DoFaultAction(1, 1);
    break;
  default:
    break;
      
  }
    
}
/////////////////////////////////////////////////////////////////////////
// FaultEc() 
//  
//
void FaultEc(unsigned state) {

  switch (state) {
    
  case 1: // under
    _FAULT_GD_SW_BIAS_UV = 1;
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_ECUV);
    DoFaultAction(1, 1);
    break;
  case 2: // over
    break;
  default:
    break;
      
  }
    
}
/////////////////////////////////////////////////////////////////////////
// Fault24v() 
//  
//
void Fault24v(unsigned state) {

  switch (state) {
    
  case 1: // under
  case 2: // over
    _FAULT_GD_SW_24V_FAULT = 1;
    DoFaultRecord(FAULTS_TYPE_SOFTWARE, FAULTS_SW_24V);
    DoFaultAction(2, 0);
    break;
  default:
    break;
      
  }
    
}

/////////////////////////////////////////////////////////////////////////
// SetEfLimits() 
//  
//
void SetEfLimits(void) {

  double value, temp;
    
  value = (double)analog_sets[ANA_SET_EF].ip_set * 0.000133;
  temp = value * 0.2;
  if (temp < 0.2) temp = 0.2;
  temp += value;
  temp /= 0.00222;
  if (temp > EF_READ_MAX) temp = EF_READ_MAX;
  analog_reads[ANA_RD_EF].read_f_hi = (unsigned)temp;
    

  temp = value * 0.15;
  if (temp < 0.2) temp = 0.2;
  value -= temp;
  if (value < 0) value = 0;
  value /= 0.00222;
  analog_reads[ANA_RD_EF].read_f_lo = (unsigned)value;
    
  ef_ref_changed_timer_10ms = 220;  // 2.2s

}

/////////////////////////////////////////////////////////////////////////
// SetEkLimits() 
//  
//
void SetEkLimits(void) {

  double value, offset, temp;
    
  value = (double)analog_sets[ANA_SET_EK].ip_set * 0.0003333;
  offset = value * 0.1;
  if (offset < 0.2) offset = 0.2; // min 200v
 
  temp = value + offset;
  temp /= 0.005555;
  analog_reads[ANA_RD_EK].read_f_hi = (unsigned)temp;
    
  temp = value - offset;
  if (temp < 0) temp = 0;
  temp /= 0.005555;
  analog_reads[ANA_RD_EK].read_f_lo = (unsigned)temp;
    
  ek_ref_changed_timer_10ms = 220;  // 2.2s
    
}
/////////////////////////////////////////////////////////////////////////
// SetEgLimits() 
//  
//
void SetEgLimits(void) {

  double value;
    
  value = (double)analog_sets[ANA_SET_EG].ip_set * 0.00666;
  value += 10;
  value /= 0.1111;
  analog_reads[ANA_RD_EG].read_f_hi = (unsigned)value;

  eg_ref_changed_timer_10ms = 220;  // 2.2s    

}

// DPARKER update pulse fault must be called at 10ms Interval or the sections that count "out of range" counts will be arbitrary time lengths

void UpdateFaults(void) {
#if 0
  // See h file for documentation
  unsigned int temp_u16int;

  // The status registers are not latched so they are reset to zero each time the faults are evaluated
  faults_magnetron_status_reg = 0;  
  faults_high_voltage_status_reg = 0;
  faults_thyratron_status_reg = 0;
  faults_control_board_status_reg = 0;
  
  
  // Load the fault masks for the current state
  LoadFaultMaskRegisters();
  
  //------------------------- START MAGNETRON FAULTS ------------------------------//
  
  // Check External Magnetron Heater Over Voltage Latch
  if (PIN_FILAMENT_OV_LATCH == ILL_FILAMENT_OV_FAULT) {
    RecordThisMagnetronFault(FAULT_HW_MAGNETRON_FILAMENT_OV);
  } 
  
  // Check that the magnetron heater voltage ADC reading has exceed fixed value set Config.h 
  if (ps_filament.v_adc_reading > ps_filament.v_adc_over_abs) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_OV_HARD_LIMIT);
  }
  
  // Check that the magnetron heater voltage ADC reading is not greater than X% of its program point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckOverVoltFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_OV);
  }
  
  // Check that the magnetron heater voltage ADC reading is not less than X% of its program point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckUnderVoltFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_UV);
  }
  
  // Check that the magnetron heater current ADC reading is not greater than X% of its expected point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckOverCurrentFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_OC);
  }
  
  
  // Check that the magnetron heater current ADC reading is not less than X% of its expected point (set in Config.h)
  // It must remain outside this range for at least v_out_of_range_count before a fault is generated
  if (CheckUnderCurrentFault(&ps_filament)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_FILAMENT_UC);
  }
  
  
  // Check External Magnetron Magnet Current Out of Range Latch
  if (PIN_MAGNET_CURRENT_OOR_LATCH == ILL_MAGNET_CURRENT_OOR_FAULT) {
    RecordThisMagnetronFault(FAULT_HW_MAGNETRON_MAGNET_COOR);
  }
  
  // Check that the magnetron magnet current ADC reading has no exceeded fixed value set Config.h 
  if (ps_magnet.i_adc_reading > ps_magnet.i_adc_over_abs) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_OC_HARD_LIMIT);
  }
  
  // Check that the magnetron magnet current ADC reading is not greater than X% of its program point (set in Config.h)
  if (CheckOverCurrentFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_OC);
  }
  
  // Check that the magnetron magnet current ADC reading is not less than X% of its program point (set in Config.h)
  if (CheckUnderCurrentFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_UC);
  }
  
  // Check that the magnetron magnet voltage ADC reading is not greater than X% of its expected value (set in Config.h)
  if (CheckOverVoltFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_OV);
  }
  
  // Check that the magnetron magnet voltage ADC reading is not less than X% of its expected value (set in Config.h)
  if (CheckUnderVoltFault(&ps_magnet)) {
    RecordThisMagnetronFault(FAULT_MAGNETRON_MAGNET_UV);
  }
  
  
  //------------------------- START HIGH VOLTAGE FAULTS ------------------------------//
  
  // THE ARC FAULTS ARE CURRENTLY SET IN UpdatePulseData()
  // UpdatesPulseData() is called after every pulse.
  
  // Check the digital fault outputs from from HV LAMBDA -
  // The sum fault line must be set for HV_LAMBDA_SUM_FAULT_COUNTER_TRIP_POINT consective readings before a trip will occur
  
  
  
  if (PIN_HV_LAMBDA_SUM_FAULT == ILL_HV_LAMBDA_SUM_FAULT_FAULTED) {
    // Record the sum fault and check the other fault lines
    RecordThisHighVoltageFault(FAULT_HV_LAMBDA_SUM_FAULT);
    
    temp_u16int = MCP23017ReadSingleByte(&U64_MCP23017, MCP23017_REGISTER_GPIOA);
    if (temp_u16int >= 0xFA00) {
      global_debug_counter.i2c_bus_error++;
    } else {
      U64_MCP23017.input_port_a_in_ram = temp_u16int & 0xFF;
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_OVER_TEMP) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_OVER_TEMP);
      }
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_INTERLOCK_FAULT) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_INTERLOCK_FAULT);
      }
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_LOAD_FLT) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_LOAD_FAULT);
      }
      if (U64_MCP23017.input_port_a_in_ram & BIT_INPUT_HV_LAMBDA_PHASE_LOSS) {
	RecordThisHighVoltageFault(FAULT_HV_LAMBDA_PHASE_LOSS);
      }
    }
  }
  
  /*
    FAULT_LAMBDA_EOC_TIMEOUT is checked/set by the TMR1 Interrupt
  */


  // DPARKER these vpeak readings . . . do we need them?  do they serve any purpose????
  /*

  // Check that the lambda vpeak ADC reading is not greater than X% of its set point (set in Config.h)
  // It must remain outside this range for at least HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (hv_lambda_vpeak_adc_reading > hv_lambda_vpeak_adc_over_trip_point) {
  hv_lambda_vpeak_over_voltage_count++;
  } else if (hv_lambda_vpeak_over_voltage_count >= 1) {
  hv_lambda_vpeak_over_voltage_count--;
  }
  if (hv_lambda_vpeak_over_voltage_count > HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT) {
  RecordThisHighVoltageFault(FAULT_HV_LAMBDA_VPEAK_OVER_VOLTAGE);
  }

  // Check that the lambda vpeak ADC reading is not less than X% of its set point (set in Config.h)
  // It must remain outside this range for at least HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (hv_lambda_vpeak_adc_reading < hv_lambda_vpeak_adc_under_trip_point) {
  hv_lambda_vpeak_under_voltage_count++;
  } else if (hv_lambda_vpeak_under_voltage_count >= 1) {
  hv_lambda_vpeak_under_voltage_count--;
  }
  if (hv_lambda_vpeak_under_voltage_count > HV_LAMBDA_VPEAK_MAX_OUT_OF_RANGE_COUNT) {
  RecordThisHighVoltageFault(FAULT_HV_LAMBDA_VPEAK_UNDER_VOLTAGE);
  }
  */
  
  //------------------------- START THYRATRON FAULTS ------------------------------//
 
  // Check that the thyratron cathode heater voltage ADC reading has exceed fixed value set Config.h 
  if (ps_thyr_cathode_htr.v_adc_reading > ps_thyr_cathode_htr.v_adc_over_abs) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_OV_HARD_LIMIT);
  }
    
  // Check that the thyratron heater voltage ADC reading is not greater than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated  
  if (CheckOverVoltFault(&ps_thyr_cathode_htr)) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_OV);
  }

  // Check that the thyratron heater voltage ADC reading is not less than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (CheckUnderVoltFault(&ps_thyr_cathode_htr)) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_UV);
  }
    
  // Check if the 4-20ma Driver has reported a fault
  if (PIN_4_20_DRVR_FLT == ILL_4_20_DRIVER_FAULT) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_DRVR_FLT);
  }

  // Check to see if the control loop has saturated.
  // If it has that means that the SCR controller is no longer responding and it should be shut down
  temp_u16int = thyratron_cathode_heater_PID.controlOutput;
  if (temp_u16int & 0x8000) {
    temp_u16int = 0x0000;
  }
  temp_u16int = temp_u16int << 1;
  if (temp_u16int >= THYRATRON_DAC_SATURATED) {
    RecordThisThyratronFault(FAULT_THYR_CATHODE_HEATER_CONTROL_SAT);
  }
  


  // Check if the thyratron reservoir ADC reading has exceed fixed value set Config.h 
  if (ps_thyr_reservoir_htr.v_adc_reading > ps_thyr_reservoir_htr.v_adc_over_abs) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_OV_HARD_LIMIT);
  }


  // Check that the thyratron reservoir voltage ADC reading is not greater than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated  
  if (CheckOverVoltFault(&ps_thyr_reservoir_htr)) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_OV);
  }

  // Check that the thyratron reservoir voltage ADC reading is not less than X% of its program point (set in Config.h)
  // It must remain outside this range for at least THYRATRON_HEATER_MAX_OUT_OF_RANGE_COUNT before a fault is generated
  if (CheckUnderVoltFault(&ps_thyr_reservoir_htr)) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_UV);
  }

  // Check if the 4-20ma Driver has reported a fault
  if (PIN_4_20_DRVR_FLT == ILL_4_20_DRIVER_FAULT) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_DRVR_FLT);
  }
  
  // Check to see if the control loop has saturated.
  // If it has that means that the SCR controller is no longer responding and it should be shut down
  temp_u16int = thyratron_reservoir_heater_PID.controlOutput;
  if (temp_u16int & 0x8000) {
    temp_u16int = 0x0000;
  }
  temp_u16int = temp_u16int << 1;
  if (temp_u16int >= THYRATRON_DAC_SATURATED) {
    RecordThisThyratronFault(FAULT_THYR_RESER_HEATER_CONTROL_SAT);
  }


  //------------------------- START CONTROL BOARD FAULTS ------------------------------//


  // Check that the lambda supply powered up
  if (PIN_HV_LAMBDA_POWER_UP == ILL_PIN_HV_LAMBDA_DID_NOT_POWER_UP) {
    RecordThisControlBoardFault(FAULT_LAMBDA_OFF);
  }
  
  // Check to see if digital interlock 1 is open  
  if (PIN_INTERLOCK_1 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_1);
  }
  
  // Check to see if digital interlock 2 is open  
  if (PIN_INTERLOCK_2 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_2);
  }
  
  // Check to see if digital interlock 3 is open  
  if (PIN_INTERLOCK_3 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_3);
  }
  
  // Check to see if digital interlock 4 is open  
  if (PIN_INTERLOCK_4 == ILL_INTERLOCK_OPEN) {
    RecordThisControlBoardFault(FAULT_DIGITAL_INTERLOCK_4);
  }
#endif
}

void ResetPulseLatches(void) {
#if 0
  PIN_PULSE_LATCH_RESET = OLL_PULSE_LATCH_RESET;
  __delay32(DELAY_PULSE_LATCH_RESET);
  PIN_PULSE_LATCH_RESET = !OLL_PULSE_LATCH_RESET;
#endif
}


void ResetHWLatches(void) {

#if 0
  // Reset the latches
  PIN_LATCH_RESET = OLL_RESET_LATCH;
  __delay32(DELAY_LATCH_RESET);
  PIN_LATCH_RESET = !OLL_RESET_LATCH;
#endif
}

void ResetAllFaults(void) {

  ResetFPGA();
  
  faults_reg_system_control = 0;
  faults_reg_software = 0;	 
  faults_reg_digi_from_gd_fpgaid = 0;
  
  control_state = STATE_START_UP;
 
 
}







void LoadFaultMaskRegisters(void) {
}



void RecordThisMagnetronFault(unsigned int fault_bit) {  
}





void WriteToEventLog(unsigned char fault_register, unsigned int fault_bit) {
  // DPARKER this function should write to the event log in ram and update the fault counter in RAM
  // These values are moved to EEPROM later on  . . . like programmed later on
}



unsigned int CheckFaultActive(void) {
  // return (faults_control_board_fault_reg | faults_thyratron_fault_reg | faults_magnetron_fault_reg | faults_high_voltage_fault_reg);
  return (0);
}


unsigned int CheckColdFaultActive(void) {
  unsigned int temp = 0;
  return temp;
}




