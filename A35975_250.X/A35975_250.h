/*
  -------------------------------------------
  This file contains configuration data specific to the A35975-000
  
  Dan Parker
  2012-06-09

  --------------------------------------------
*/

#ifndef __A35975_250_H
#define __A35975_250_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <spi.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"
//#include "faults.h"

#define FCY_CLK                    10000000


// --------- Resource Summary  -----------------
/*
  Hardware Module Resource Usage

  CAN2   - Used/Configured by ETM CAN (optical CAN) - DPARKER NEED TO RECOMPILE LIBRARY TO USE CAN2
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by Gun Driver

  Timer1 - Used to generate 100ms Timer - DPARKER this apears to be used by multiple 100ms timing operations
  Timer2 - Used for 10msTicToc (route this over to Timer)

  ADC Module - Not used

*/




// --------- Compile Time Options -----------------

//#define DEMO   /* simulator for CAN protocol */

#define TEST_MODE_BYP_FIBER_OFF        1    /* don't turn off hv or trig because fiber off, for test only */

#define USE_ENGINEERING_UNIT_ON_GUN_DRIVER    1   /* use engineering units for all parameters on CAN */

//#define TEST_BYP_FPGA_FAULTS       1

//#define TEST_STATE_LOG             1

//#define TEST_SIMULATOR    1


//#define LOOPBACK_TEST    1

//#define ENABLE_STANDARD_CANOPEN     1   // enable heartbeat, standard command set





// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*
  RA15 - optical HV enable1
  RA14 - optical HV enable2
  RA13 - test pulse good2
  RA12 - test pulse good1
  
  RB15 - CS DAC Enable 

  RB11 - optical Trig enable
  RB9  - optical test pulse enable


*/

//   ------------------  Digital Output Pins ---------------
/*

  
 */

//					         fedcba9876543210
#define A35975_TRISA_VALUE 0b1111000000000000 
#define A35975_TRISB_VALUE 0b1000101000000000 
#define A35975_TRISC_VALUE 0b0000000000000000 
#define A35975_TRISD_VALUE 0b0000000000000000 
#define A35975_TRISF_VALUE 0b0000000010000101 
#define A35975_TRISG_VALUE 0b0000000000000001

//------------------- GUN Driver Interface I/O ------------------------- //
// FPGA Output Pins
#define PIN_CS_DAC_ENABLE                    _LATD13		   
#define OLL_CS_DAC_ENABLE                    1

#define PIN_CS_ADC_ENABLE                    _LATD14		   
#define OLL_CS_ADC_ENABLE                    1

#define PIN_CS_AUX_ENABLE                    _LATD15		   
#define OLL_CS_AUX_ENABLE                    1

#define PIN_CS_DAC_ENABLE_BIT                0x2000
#define PIN_CS_ADC_ENABLE_BIT                0x4000
#define PIN_CS_AUX_ENABLE_BIT                0x8000
#define PIN_CS_ALL_ENABLE_BITS               0xe000


// LOGIC Output Pins
#define PIN_OPT_GD_READY                     _LATA10	   
#define OLL_OPT_GD_READY                     1

#define PIN_CAN_HV_ENABLE                    _LATB12	   
#define OLL_CAN_HV_ENABLE                    1

#define PIN_CAN_PULSETOP_ENABLE              _LATB10	   
#define OLL_CAN_PULSETOP_ENABLE              1

#define PIN_CAN_TRIGGER_ENABLE               _LATB8	   
#define OLL_CAN_TRIGGER_ENABLE               1


//#define PIN_OPT_CAN_XMIT_OUT                 _LATG1		   
//#define OLL_OPT_CAN_XMIT_ON                  1


// LOGIC Input Pins
//#define PIN_CS_DAC_ENABLE_INPUT            _RB15		   // was designed for DAC output 


#ifndef DEMO
#define PIN_OPT_HV_ENABLE1_INPUT             _RA15	   
#define ILL_OPT_HV_ENABLE                    1

#define PIN_OPT_HV_ENABLE2_INPUT             _RA14	   // A14 and A15 inputs are tied together

#define PIN_OPT_TRIG_ENABLE_INPUT            _RB11
#define ILL_OPT_TRIG_ENABLE                  1

#define PIN_OPT_TEST_PULSE_ENABLE_INPUT       _RB9	   
#define ILL_OPT_TEST_PULSE_ENABLE             1

#else
#define PIN_OPT_HV_ENABLE1_INPUT             1	   
#define ILL_OPT_HV_ENABLE                    1

#define PIN_OPT_HV_ENABLE2_INPUT             _RA14	   // A14 and A15 inputs are tied together

#define PIN_OPT_TRIG_ENABLE_INPUT              1
#define ILL_OPT_TRIG_ENABLE                    1

#define PIN_OPT_TEST_PULSE_ENABLE_INPUT        1	   
#define ILL_OPT_TEST_PULSE_ENABLE              1
#endif

#define PIN_TEST_PULSE_GOOD1_INPUT           _RA12
#define ILL_TEST_PULSE_GOOD                  1

#define PIN_TEST_PULSE_GOOD2_INPUT           _RA13	   // A12 and A13 inputs are tied together



// MCP4822 DAC Output Pins
#define PIN_CS_MCP4822_ENABLE                _LATB14	   
#define OLL_CS_MCP4822_ENABLE                0

#define PIN_LDAC_MCP4822_ENABLE              _LATB13	   
#define OLL_LDAC_MCP4822_ENABLE              0



// UART TX enable
#define PIN_RS422_DE                         _LATF4
#define OLL_RS422_DE_ENABLE_RS422_DRIVER     1




// LED Indicator Output Pins
#define OLL_LED_ON                            0

#define PIN_LED_24DC_OK                      _LATG14	   

#define PIN_LED_LAST_PULSE_GOOD              _LATG12	   

#define PIN_LED_GD_READY                     _LATG13	   

#define PIN_LED_HV_ENABLE                    _LATG15   

#define PIN_LED_AC_ON                        _LATC1   

#define PIN_LED_LAST_PULSE_FAIL              _LATC2  
//#define TRIS_PIN_LED_LAST_PULSE_FAIL         _TRISC2		   
//#define OLL_LED_LAST_PULSE_FAIL              0

#define PIN_LED_WARMUP                       _LATC3   

#define PIN_LED_SUM_FAULT                    _LATC4   


// -----------------------  END IO PIN CONFIGURATION ------------------------ //


/* ------------------------------ CLOCK AND TIMING CONFIGURATION ------------------------- */
//#define FCY_CLK                    10000000      // 29.495 MHz   defined in ETM CAN
//#define FCY_CLK_MHZ                10.000        // 29.495 MHz   defined in ETM CAN

#define UART1_BAUDRATE             124000        // U1 Baud Rate
#define I2C_CLK                    100000        // Target I2C Clock frequency of 100KHz



// -------------------------------------------- INTERNAL MODULE CONFIGURATION --------------------------------------------------//


/*
  --- SPI1 Port --- 
  This SPI port is used to connect with the gun driver
  The prescales of 16:1 and 1:1 will generate a clock = Fcy/16.  In this case 1.843MHz
  This must be slower to compensate for the 2x delay across the optocoupler 200ns with filtering in one direction, 80ns (without filtering) in the other direction
  Minimum clock period is therefore 280ns + holdtime + margins
*/
//#define A35975_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
//#define A35975_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   
//#define A35975_SPI1CON_CLOCK (SEC_PRESCAL_4_1 & PRI_PRESCAL_4_1)
//#define A35975_SPI2CON_CLOCK (SEC_PRESCAL_2_1 & PRI_PRESCAL_1_1)



/*
  --- Timer1 Setup ---
  Period of 100mS
*/
#define A35975_T1CON_VALUE  (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT)
#define A35975_PR1_ROLL_US  100000      // 100mS
#define A35975_PR1_VALUE    ((FCY_CLK/1000000)*A35975_PR1_ROLL_US/64)


/*
  --- Timer2 Setup ---
  Period of 10mS
*/
#define A35975_T2CON_VALUE     (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define A35975_PR2_VALUE_US    10000   // 10mS
#define A35975_PR2_VALUE       ((FCY_CLK/1000000)*A35975_PR2_VALUE_US/8)

 
// ---- Hard Coded Delays ---- //
#define DELAY_TCY_10US                          100       // 10us us at 10MHz Clock

#define DELAY_PULSE_CABLE_SELECT_PROP_DELAY_US  1        // 1uS
// This delay must be longer than the propogation delay on the Isolated DAC Cable Select Line
#define DELAY_PULSE_CABLE_SELECT_PROP_DELAY     ((FCY_CLK/1000000)*DELAY_PULSE_CABLE_SELECT_PROP_DELAY_US)



// ----------- Data Structures ------------ //

#define ANALOG_SET_SIZE   8
#define ANALOG_READ_SIZE  10


// analog set pointers
enum {
  ANA_SET_EK = 0,
  ANA_SET_EG,
  ANA_SET_EF,
  ANA_SET_HV_ON,
  ANA_SET_HTR_ON,
  ANA_SET_PULSETOP_ON,
  ANA_SET_TRIG_ON,
  ANA_SET_WDOG,
};




extern void FaultEk(unsigned state);
extern void FaultEf(unsigned state);
extern void FaultIf(unsigned state);
extern void FaultEg(unsigned state);
extern void FaultEc(unsigned state);
extern void Fault24v(unsigned state);

extern void SetEfLimits(void);
extern void SetEkLimits(void);
extern void SetEgLimits(void);

extern void SetEf(unsigned set_value);
extern void SetEk(unsigned set_value);
extern void SetEg(unsigned set_value);

extern unsigned char AreAnyReferenceNotConfigured(void);

// analog read pointers
enum {
  ANA_RD_EK = 0,
  ANA_RD_IKA,
  ANA_RD_IKP,
  ANA_RD_EF,
  ANA_RD_IF,
  ANA_RD_EG, // grid V
  ANA_RD_EC, // bias V
  ANA_RD_24V, // 24DC
  ANA_RD_TEMP, // temperature
   
  ANA_RD_HTR_WARMUP,
  ANA_RD_WATCHDOG,
  ANA_RD_ARC,
  ANA_RD_OT,
  ANA_RD_PW_DUTY,
  ANA_RD_BIASFLT,
  ANA_RD_DA_FDBK,
};



#ifdef USE_ENGINEERING_UNIT_ON_GUN_DRIVER 

#define CAN_SCALE_TABLE_SIZE  13
// analog read pointers
enum {
  CAN_RD_EK = 0,
  //   CAN_RD_IKA,
  CAN_RD_IKP,
  CAN_RD_EF,
  CAN_RD_IF,
  CAN_RD_EG, // grid V
  CAN_RD_EC, // bias V
  //  CAN_RD_24V, // 24DC
  CAN_RD_TEMP, // temperature
   
  CAN_RD_EKSET,
  CAN_RD_EFSET,
  CAN_RD_EGSET,
  CAN_SET_EKSET,
  CAN_SET_EFSET,
  CAN_SET_EGSET,
   
};
/*
  Name 	      |   cal factor | offset|	CAN Interface |	CAN Unit/bit |	CAN Range|	CAN Scaling| ScaleFactor |Scale Offset
  --------------------------------------------------------------------------------------------------------------------------
  EK_RD		  |	  0.005555	 | 0	 |	1 V/bit	      |  0.001		 |	65.535	 |	5.555	   | 22753 		 | 0
  IKA_RD		  |	  0.001667	 | 0	 |	1 mA/bit	  |  0.001		 |	65.535	 |	1.667	   | 54624 		 | 0
  IKP_RD		  |	  0.277		 | 0	 |	100 mA/bit    |    0.1		 |	6553.5	 |	2.77	   | 11345 		 | 0
  EF_RD		  |	  0.00222	 | 0	 |	1 mV/bit	  |  0.001		 |	65.535	 |	2.22	   | 9093  		 | 0
  IF_RD		  |	  0.001667	 | 0	 |	10 mA/bit	  |  0.001		 |	65.535	 |	1.667	   | 54624 		 | 0
  EG_RD		  |	  0.1111	 | 80	 |   100 mV/bit	  |    0.1		 |	6553.5	 |	1.111	   | 36405 		 | 0
  EC_RD		  |	  0.05555	 | 0	 |	100 mV/bit    |    0.1		 |	6553.5	 |	0.5555	   | 18202 		 | 0
  TEMP_RD		  |	  0.0133	 | 0	 |	0.01 C/bit    |   0.01		 |	655.35	 |	1.33	   | 43581 		 | 0
  |				 |		 |				  |				 |		  	 |			   |	   		 |	
  Ekset bits-val|	  0.0003333	 | 0	 |	1 V/bit	      |  0.001		 |	65.535	 |	0.3333	   | 10921 		 | 0
  Ekset val-bits|				 |		 |				  |				 |		  	 |	3.00030003 | 12289 		 | 0
  Efset bits-val|	  0.000133	 | 0	 |	10 mV/bit	  |  0.001		 |	65.535	 |	0.133	   | 4358  		 | 0
  Efset val-bits|				 |		 |				  |				 |		  	 |	7.518796992| 30796 		 | 0
  Egset bits-val|	  0.00666	 | 80	 |   100 mV/bit	  |    0.1		 |	6553.5	 |	0.0666	   | 2182  		 | 0
  Egset val-bits|				 |		 |				  |				 |			 |	15.01501502| 61501 		 | 0  


  Note:  Scale offset for EG read/set is handled by GUI.
			  
*/			  

#define CAL_EK_RD    0.005555
#define CAL_IKA_RD   0.001667
#define CAL_IKP_RD   0.277
#define CAL_EF_RD    0.00222
#define CAL_IF_RD    0.001667
#define CAL_EG_RD    0.1111
#define CAL_EC_RD	 0.05555
#define CAL_TEMP_RD	 0.0133

#define CAL_EKSET    0.0003333
#define CAL_EFSET    0.000133
#define CAL_EGSET    0.00666

#define CAN_EK_SCALE     0.001
#define CAN_IKA_SCALE 	 0.001	
#define CAN_IKP_SCALE 	 0.1	
#define CAN_EF_SCALE 	 0.001	
#define CAN_IF_SCALE 	 0.001	
#define CAN_EG_SCALE 	 0.1	
#define CAN_EC_SCALE 	 0.1	
#define CAN_TEMP_SCALE 	 0.01	
								
#define CAN_EKSET_SCALE  0.001
#define CAN_EFSET_SCALE  0.001
#define CAN_EGSET_SCALE  0.1
					 
					 




#endif /* USE_ENGINEERING_UNIT_ON_GUN_DRIVER */




#define FAULT_SIZE  22  /* 6 ADC + 16 FPGA_ID */

//#define DIGI_ADC_HTR_FLT_WARMUP    0
#define DIGI_ADC_FPGA_WATCHDOG     1
#define DIGI_ADC_ARC      		   2
//#define DIGI_ADC_TEMP			   3
//#define DIGI_ADC_PW_DUTY  		   4
//#define DIGI_ADC_BIAS_TOP          5

#define DIGI_ID_ARC_COUNT          6
//#define DIGI_ID_ARC_HV_INHIBIT     7
//#define DIGI_ID_EF_LESS_4p5V       8
//#define DIGI_ID_TEMP_65C		   9
#define DIGI_ID_TEMP_75C		   10
#define DIGI_ID_PW_LIMITING        11
#define DIGI_ID_PRF			       12
#define DIGI_ID_CURR_PW		       13

#define DIGI_ID_GRID_HW		  	   14
#define DIGI_ID_GRID_OV		       15
#define DIGI_ID_GRID_UV		       16
#define DIGI_ID_BIAS_V			   17
//#define DIGI_ID_HV_REGULATION      18
#define DIGI_ID_DIP_SW             19
#define DIGI_ID_TEST_MODE		   20
#define DIGI_ID_LOCAL_MODE         21
 





/*
  --- LOGIC  STATE DEFINITIONS ---
  See flow diagram for more information
  DPARKER add flow diagram doc number
*/
#define STATE_START_UP                       0x06
#define STATE_START_UP2                      0x08

#define STATE_SYSTEM_HTR_OFF          		 0x10
#define STATE_READY_FOR_HEATER 				 0x12
#define STATE_HEATER_STARTUP   				 0x14
#define STATE_WARM_UP                        0x16

#define STATE_SYSTEM_HV_OFF                  0x20
#define STATE_READY_FOR_HV		     		 0x22
#define STATE_HV_STARTUP                     0x24
#define STATE_HV_ON                          0x26

#define STATE_SYSTEM_PULSETOP_OFF            0x30
#define STATE_READY_FOR_PULSETOP    		 0x32
#define STATE_PULSETOP_STARTUP               0x34
#define STATE_PULSETOP_ON		             0x36

#define STATE_SYSTEM_TRIG_OFF                0x40
#define STATE_READY_FOR_TRIG		         0x42
#define STATE_TRIG_STARTUP	                 0x44
#define STATE_TRIG_ON	                     0x46


#define STATE_FAULT_COLD_FAULT               0x80 
#define STATE_FAULT_HOT_FAULT                0xA0 // 0x80 + 0x20


/* 
   --- SYSTEM STATE BYTE DEFINITIONS --- 
*/
#define SYS_BYTE_HTR_ON						 0x01
#define SYS_BYTE_LOGIC_READY  				 0x02
#define SYS_BYTE_HV_ON						 0x04
#define SYS_BYTE_PULSETOP_ON    		     0x08

#define SYS_BYTE_TRIG_ON					 0x10
#define SYS_BYTE_FAULT_ACTIVE                0x20
#define SYS_BYTE_HTR_WARMUP				     0x40  /* htr off or warmup */
#define SYS_BYTE_HV_DRIVEUP				     0x80

/*
  --- Public Functions ---
*/

extern void DoStateMachine(void);

extern void LogicHeaterControl(unsigned char turnon);
extern void LogHvControl(unsigned char turnon);
extern void LogPulsetopControl(unsigned char turnon);
extern void LogTrigControl(unsigned char turnon);

extern void SendHeaterRef(unsigned int bits);
extern void SendHvRef(unsigned int bits);
extern void SendPulsetopRef(unsigned int bits);
extern void SendWatchdogRef(unsigned int bits);

extern void Do10msTicToc(void);
extern void ResetFPGA(void);


#define SYSTEM_WARM_UP_TIME      3000 /* 100ms Units  //DPARKER this is way to short */
#define EF_READ_MAX              2838 /*  -6.3/-0.00222 */
#define IF_READ_MAX              1051 /*  1.75/0.001666 */
//#define IF_READ_MAX_95P          1824 // 95% of IF_MAX
//#define IF_READ_MAX_85P          1633 // 85% of IF_MAX

#define IF_READ_MAX_90P           945 /* 90% of IF_MAX */

#define EF_SET_MAX              47369 /*  -6.3/-0.000133  */
#define EG_SET_MAX              33033 /* 140V, 0.00666    */
#define EK_SET_MAX              60060 /* -20kV/-0.000333  */
 


#define _ISRFASTNOPSV __attribute__((interrupt, shadow, no_auto_psv)) 
#define _ISRNOPSV __attribute__((interrupt, no_auto_psv)) 


/*
  --- Gobal Variales ---
*/
extern unsigned char control_state;
extern unsigned char last_control_state;
extern unsigned char system_byte;

extern unsigned int led_pulse_count;
extern unsigned int htd_timer_in_100ms;
extern unsigned int software_skip_warmup;	

extern unsigned int fpga_ASDR;
extern unsigned int faults_from_ADC;

extern unsigned long read_cycles_in_2s;
 
extern unsigned int ekuv_timeout_10ms;

extern unsigned int ek_ref_changed_timer_10ms;   // mask Ek faults when ref is changed
extern unsigned int ef_ref_changed_timer_10ms;   // mask Ef, If faults when ref is changed
extern unsigned int eg_ref_changed_timer_10ms;   // mask Eg fault when ref is changed

extern unsigned char htr_OVOC_count;                  // for auto-reset htr OVOC feature
extern unsigned int  htr_OVOC_rest_delay_timer_10ms;  // after OVOC fault, rest for a few seconds before turning htr on
extern unsigned char htr_OVOC_auto_reset_disable;     // if other system fault happens, disable htr OVOC auto-reset

//	extern BUFFER64BYTE uart1_input_buffer;
//	extern BUFFER64BYTE uart1_output_buffer;

extern volatile unsigned int lvdinterrupt_counter;

extern volatile unsigned int _PERSISTENT last_known_action;
extern volatile unsigned int _PERSISTENT last_osccon;

extern unsigned int _PERSISTENT processor_crash_count;
extern unsigned int previous_last_action;

extern  unsigned long hw_version_data;


extern signed int ps_magnet_config_ram_copy[16];
extern unsigned long EE_address_ps_magnet_config_in_EEPROM;



#define LAST_ACTION_DEFAULT_INT                        0xFABC
#define LAST_ACTION_CLEAR_LAST_ACTION                  0x0000
//#define LAST_ACTION_ADC_INTERRUPT                      0x0001
#define LAST_ACTION_LVD_INT                            0x0002
//#define LAST_ACTION_T1_INT                             0x0003
//#define LAST_ACTION_INT1_INT                           0x0004
//#define LAST_ACTION_UPDATE_IO_EXPANDER                 0x0005
#define LAST_ACTION_FILTER_ADC                         0x0006
#define LAST_ACTION_READ_ISOLATED_ADC                  0x0007
//#define LAST_ACTION_DO_THYRATRON_PID                   0x0008
#define LAST_ACTION_DO_10MS                            0x0009
#define LAST_ACTION_UPDATE_DAC_ALL                     0x000A
//#define LAST_ACTION_POST_PULSE_PROC                    0x000B
#define LAST_ACTION_HV_ON_LOOP                         0x000C
#define LAST_ACTION_OSC_FAIL                           0x000D
#define LAST_ACTION_ADDRESS_ERROR                      0x000E
#define LAST_ACTION_STACK_ERROR                        0x000F
#define LAST_ACTION_MATH_ERROR                         0x0010


// CAN bus related variables

extern unsigned char sdo_reset_cmd_active;	  // logic resumes only when reset isn't active
extern unsigned char sdo_logic_reset;        // a separate cmd to reset fault
extern unsigned char sdo_htd_timer_reset; 

extern unsigned char sdo_htr_enable;
extern unsigned char sdo_hv_bypass;
extern unsigned char sdo_hv_enable;
extern unsigned char sdo_pulsetop_enable;
extern unsigned char sdo_trig_enable;  




#define _STATUS_GD_HV_DISABLE                           _STATUS_0	
#define _STATUS_GD_HTR_NOT_READY                        _STATUS_1
#define _STATUS_GD_TRIG_NOT_ENABLED                     _STATUS_2
#define _STATUS_GD_TOP_NOT_ENABLED                      _STATUS_3
#define _STATUS_GD_HV_NOT_ENABLED    				    _STATUS_4
#define _STATUS_GD_HTR_NOT_ENABLED                      _STATUS_5	

//#define _STATUS_GD_FPGA_DIP_SWITCH                      _STATUS_5
//#define _STATUS_GD_FPGA_WIDTH_LIMITING                  _STATUS_6
//#define _STATUS_GD_FPGA_ARC_WARNING                     _STATUS_7

#define _FAULT_GD_SUM_FAULT                             _FAULT_0
#define _FAULT_GD_FPGA_COMM_LOST                        _FAULT_1
#define _FAULT_GD_SW_HTR_OVOC                           _FAULT_2
#define _FAULT_GD_SW_BIAS_UV                            _FAULT_3
#define _FAULT_GD_SW_EK_OV                              _FAULT_4
#define _FAULT_GD_SW_EK_UV                              _FAULT_5
#define _FAULT_GD_SW_GRID_OV                            _FAULT_6
#define _FAULT_GD_FPGA_TEMP_75C                         _FAULT_7

#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_8

#define _FAULT_GD_FPGA_ARC_FAULT                        _FAULT_9
#define _FAULT_GD_FPGA_PULSE_FAULT                      _FAULT_A
#define _FAULT_GD_FPGA_GRID_FAULT                       _FAULT_B

#define _FAULT_GD_SW_HTR_UV                             _FAULT_C
#define _FAULT_GD_SW_24V_FAULT                          _FAULT_D
#define _FAULT_GD_SYS_FAULTS                            _FAULT_E
//#define _FAULT_GD_SYS_FAULTS                            _FAULT_F



typedef struct {
  unsigned int channel;       /* DAC channel 							   */
  unsigned int ip_set;        /* current setting (0-ffff)		           */
  unsigned int ip_set_alt;    /* static alt setting if flag              */
  unsigned int ip_set_flag;   /* static alt setting if varptr goes true  */
				  
  unsigned int ip_set_max;    /* factory maximum setting (0-ffff) 	   */
  unsigned int ip_set_min;    /* factory minimum setting (0-ffff) 	   */

  unsigned int need_update; 

} TYPE_ANALOG_SETS;
extern TYPE_ANALOG_SETS analog_sets[ANALOG_SET_SIZE];


typedef struct {
  unsigned int channel;	/* ADC channel 							       */
  unsigned int read_cur;  /* current reading                             */

  unsigned int read_cnt;  /* how many readings taken                     */
  unsigned int read_err;  /* how many errors                             */

  unsigned int read_f_lo; /* low end fault level                         */
  unsigned int read_f_hi; /* high end fault level                        */
  unsigned int read_m_lo; /* lo end mask   0 off   1 on   2 trip         */
  unsigned int read_m_hi; /* hi end mask   0 off   1 on   2 trip         */
  unsigned int events;
  void (*fault_vect)(unsigned state);

} TYPE_ANALOG_READS;
extern TYPE_ANALOG_READS analog_reads[ANALOG_READ_SIZE];



typedef struct {
  // -------- These are used to calibrate and scale the ADC Reading to Engineering Units ---------
  unsigned int fixed_scale;
  signed int   fixed_offset;

} TYPE_CAN_SCALE_TABLE;

extern TYPE_CAN_SCALE_TABLE CAN_scale_table[CAN_SCALE_TABLE_SIZE];



typedef struct {
  unsigned int state;                 /* fault input state                */
  unsigned int mask;                  /* 0 = disabled  -  1 = enabled     */

  unsigned int from_adc;              /* 1: from ADC, 0: from FPGA ID     */
  unsigned int bits;                   /* bit position */
  unsigned int fre;                   /* how many events have to happen   */
  /* 0 = do next. 1 = one free        */
  unsigned int left;                  /* how many events left.            */

  unsigned int fault_latched;         /* fault was latched, need send RESET to FPGA board  */
  unsigned int action_code;           /* 0: no action, update the bit, 1: htr off, 2: hv off, 3: pulsetop off, 4: trig off, 99: ignore the bit */
} TYPE_DIGI_READS;

extern TYPE_DIGI_READS digi_reads[FAULT_SIZE];




typedef struct {
  unsigned int spi1_bus_error;
  unsigned int spi2_bus_error;
  unsigned int external_adc_false_trigger;
  unsigned int LTC2656_write_error;
  unsigned int setpoint_not_valid;
  unsigned int scale16bit_saturation;
  unsigned int reversescale16bit_saturation;
} TYPE_DEBUG_COUNTER;

extern TYPE_DEBUG_COUNTER global_debug_counter;


// CONTROL FAULT REGISTER, 3 main types
extern unsigned int faults_reg_system_control;
extern unsigned int faults_reg_software;	    
extern unsigned int faults_reg_digi_from_gd_fpgaid;

#define FPGAID_FAULTS_MASK                        0x0FD6

// main fault type definitions
#define FAULTS_TYPE_SYSTEM_CONTROL                1
#define FAULTS_TYPE_SOFTWARE                      2
#define FAULTS_TYPE_DIGI_FROM_FPGAID              3

// system fault bits
#define FAULTS_SYS_FPGAID                         0x0001
#define FAULTS_SYS_CAN_TIMEOUT                    0x0002
#define FAULTS_SYS_FPGA_WATCHDOG_ERR              0x0004

#define FAULTS_SYS_LOGIC_STATE                    0x0010
#define FAULTS_SYS_ILLEGAL_INTERRUPT			  0x0020


#define FAULTS_SW_EFOV_IFOC  					  0x0001  // Ef > (120% or .2V) ref, or If > Ifmax
#define FAULTS_SW_EFUV       					  0x0002  // Ef < (85% or .2V) ref
#define FAULTS_SW_ECUV      					  0x0004  // abs(Ec) < 120V
#define FAULTS_SW_EKOV_EKUV  					  0x0008  // Ek 10% higher or lower

#define FAULTS_SW_EGOV      					  0x0010  // Eg > (ref + 10)
#define FAULTS_SW_24V        					  0x0020  // out of 10% 24V range





#define NO_FAULTS  0x0000
#define ALL_FAULTS 0x1111    



void DoFaultRecord(unsigned int fault_type, unsigned int fault_bit);
void DoFaultClear(unsigned int fault_type, unsigned int fault_bit);

extern void CheckAnalogLimits(unsigned index);
extern void DoFaultAction(unsigned char type, unsigned char disable_htr_auto_reset);



void UpdateFaults(void);
/*
  This function updates all faults that are checked on a periodic basis.
  This is all faults EXCEPT for the pulse faults (these are checked after each pulse)
  It is called by Do10msTicToc() once every 10 ms
  What this function does
  * Loads the fault/warning masks for the current state
  * Checks all the Magnetron Faults Inputs and sets Status/Warning/Fault registers
  * Checks all the HV Lambda Fault Inputs and sets Status/Warning/Fault registers
  * Checks all the Thyratron Fault Inputs and sets Status/Warning/Fault registers
  * Checks all the Control Board Faults and sets Status/Warning/Fault registers

  */


void UpdatePulseData(unsigned char mode);
/*
  This function updates all the pulse faults and is called after each pulse.
  * Looks for an arc. 
  */

void ResetHWLatches(void);

void ResetAllFaults();
// DPARKER need to write function

unsigned int CheckStartupFailed(void);

unsigned int CheckFaultActive(void);

unsigned int CheckColdFaultActive(void);





void RecordThisMagnetronFault(unsigned int fault_bit);
void RecordThisHighVoltageFault(unsigned int fault_bit);
void RecordThisThyratronFault(unsigned int fault_bit);
void RecordThisControlBoardFault(unsigned int fault_bit);


void ResetPulseLatches(void);



#endif
