#include <std.h>
#include <log.h>
#include <clk.h>
#include <gbl.h>
#include <bcache.h>

#include <mem.h> // MEM_alloc calls
#include <que.h> // QUE functions
#include <sem.h> // Semaphore functions
#include <sys.h> 
#include <tsk.h> // TASK functions
#include <math.h> 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <c6x.h> // register defines


#include "projectinclude.h"
#include "c67fastMath.h" // sinsp,cossp, tansp
#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "evmomapl138_spi.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "LCDprintf.h"
#include "ladar.h"
#include "xy.h"
#include "MatrixMath.h"

#define FEETINONEMETER 3.28083989501312

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;

volatile uint32_t index;

extern float enc1;  // Left motor encoder
extern float enc2;  // Right motor encoder
extern float enc3;
extern float enc4;
extern float adcA0;  // ADC A0 - Gyro_X -400deg/s to 400deg/s  Pitch
extern float adcB0;  // ADC B0 - External ADC Ch4 (no protection circuit)
extern float adcA1;  // ADC A1 - Gyro_4X -100deg/s to 100deg/s  Pitch
extern float adcB1;  // ADC B1 - External ADC Ch1
extern float adcA2;  // ADC A2 -	Gyro_4Z -100deg/s to 100deg/s  Yaw
extern float adcB2;  // ADC B2 - External ADC Ch2
extern float adcA3;  // ADC A3 - Gyro_Z -400deg/s to 400 deg/s  Yaw
extern float adcB3;  // ADC B3 - External ADC Ch3
extern float adcA4;  // ADC A4 - Analog IR1
extern float adcB4;  // ADC B4 - USONIC1
extern float adcA5;  // ADC A5 -	Analog IR2
extern float adcB5;  // ADC B5 - USONIC2
extern float adcA6;  // ADC A6 - Analog IR3
extern float adcA7;  // ADC A7 - Analog IR4
extern float compass;
extern float switchstate;

float vref = 0;
float turn = 0;

int tskcount = 0;
char fromLinuxstring[LINUX_COMSIZE + 2];
char toLinuxstring[LINUX_COMSIZE + 2];

float LVvalue1 = 0;
float LVvalue2 = 0;
int new_LV_data = 0;

int newnavdata = 0;
float newvref = 0;
float newturn = 0;

extern sharedmemstruct *ptrshrdmem;

float x_pred[3][1] = {{0},{0},{0}};					// predicted state

//more kalman vars
float B[3][2] = {{1,0},{1,0},{0,1}};			// control input model
float u[2][1] = {{0},{0}};			// control input in terms of velocity and angular velocity
float Bu[3][1] = {{0},{0},{0}};	// matrix multiplication of B and u
float z[3][1];							// state measurement
float eye3[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// 3x3 identity matrix
float K[3][3] = {{1,0,0},{0,1,0},{0,0,1}};		// optimal Kalman gain
#define ProcUncert 0.0001
#define CovScalar 10
float Q[3][3] = {{ProcUncert,0,ProcUncert/CovScalar},
				 {0,ProcUncert,ProcUncert/CovScalar},
				 {ProcUncert/CovScalar,ProcUncert/CovScalar,ProcUncert}};	// process noise (covariance of encoders and gyro)
#define MeasUncert 1
float R[3][3] = {{MeasUncert,0,MeasUncert/CovScalar},
				 {0,MeasUncert,MeasUncert/CovScalar},
				 {MeasUncert/CovScalar,MeasUncert/CovScalar,MeasUncert}};	// measurement noise (covariance of LADAR)
float S[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance
float S_inv[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance matrix inverse
float P_pred[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// predicted covariance (measure of uncertainty for current position)
float temp_3x3[3][3];				// intermediate storage
float temp_3x1[3][1];				// intermediate storage
float ytilde[3][1];					// difference between predictions

// deadreckoning
float vel1 = 0,vel2 = 0;
float vel1old = 0,vel2old = 0;
float enc1old = 0,enc2old = 0;

// SETTLETIME should be an even number and divisible by 3
#define SETTLETIME 6000
int settlegyro = 0;
float gyro_zero = 0;
float gyro_angle = 0;
float old_gyro = 0;
float gyro_drift = 0;
float gyro = 0;
int gyro_degrees = 0;
float gyro_radians = 0.0;
float gyro_x = 0,gyro_y = 0;
float gyro4x_gain = 1;

extern float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
extern float newLADARangle[LADAR_MAX_DATA_SIZE];		// in degrees
float LADARdistance[LADAR_MAX_DATA_SIZE];
float LADARangle[LADAR_MAX_DATA_SIZE];
extern pose ROBOTps;
extern pose LADARps;
extern float newLADARdataX[LADAR_MAX_DATA_SIZE];
extern float newLADARdataY[LADAR_MAX_DATA_SIZE];
float LADARdataX[LADAR_MAX_DATA_SIZE];
float LADARdataY[LADAR_MAX_DATA_SIZE];
extern int newLADARdata;

// Optitrack Variables
int trackableIDerror = 0;
int firstdata = 1;
volatile int new_optitrack = 0;
volatile float previous_frame = -1;
int frame_error = 0;
volatile float Optitrackdata[OPTITRACKDATASIZE];
pose OPTITRACKps;
float temp_theta = 0.0;
float tempOPTITRACK_theta = 0.0;
volatile int temp_trackableID = -1;
int trackableID = -1;
int errorcheck = 1;


//data for the ball following
int blue_x = 0;
int blue_y = 0;
int blue_num_pix = 0;
int blue_num_obj = 0;
int new_blue_x = 0;
int new_blue_y = 0;
int new_blue_num_pix = 0;
int new_blue_num_obj = 0;
int red_x = 0;
int red_y = 0;
int red_num_pix = 0;
int red_num_obj = 0;
int new_red_x = 0;
int new_red_y = 0;
int new_red_num_pix = 0;
int new_red_num_obj = 0;

float Kp_ball = 0.05;
float blue_error = 0;
float red_error = 0;
float follow_ref = 0;

float blue_dist = 0.0;
float red_dist = 0.0;

extern int new_coordata;

// Path Planning Variables
// Format Coordinates as "variable"_"coord frame"_"object"
// H represents heuristic
// G represents movement cost
// F represents sum of G and H

#define X_GRID_SIZE 8
#define Y_GRID_SIZE 11
#define H_COST 1
#define G_MOVE_COST 1
#define G_TURN_COST 1
#define HITS_THRESHOLD 40
#define REC_CNT_THRESHOLD 300

// Structure Declerations
typedef struct tile {
	int x;
	int y;
	int map;
	int hits;
	int recognized_cnt;
	float h;
	float g;
	float f;
	int link;
	int open;
	int closed;
} tile;

typedef struct map_point {
	float x;
	float y;
} map_point;

// A* Semaphore
extern SEM_Obj SEM_a_star;

// Global Variable Declarations
// old way Dan Block found as a mistake
//tile grid[X_GRID_SIZE * Y_GRID_SIZE - 1];
tile grid[X_GRID_SIZE * Y_GRID_SIZE];
map_point target_points[9];
map_point waypoints[50];
int current_target = 0;	// skip first target point
int current_waypoint = -1;
int flag_new_path_calculating = 0;
int flag_new_path = 0;
int flag_bounds_error = 0;
int obst_grid = 0;
int x_grid_obst = 0;
int y_grid_obst = 0;
int old_map = 0;
int new_map = 0;

// initialize course map
int course_map[X_GRID_SIZE * Y_GRID_SIZE] = 	// initialize grid map with course walls
	{1,	1,	1,	1,	1,	1,	1,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	1,	1,	0,	0,	1,	1,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	0,	0,	0,	0,	0,	0,	1,
	1,	1,	1,	1,	1,	1,	1,	1};

// A* Variables
int x_grid_robot = 0;
int y_grid_robot = 0;
int x_grid_target = 0;
int y_grid_target = 0;
int robot_grid = 0;
int target_grid = 0;
int flag_path_found = 0;
int task_current = 0;
int adjacent = 0;
int	link_found = 0;
int current_link = 0;
int flag_list_started = 0;
int flag_error_invalid_obstacle = 0;
int flag_error_no_path_found = 0;
int old_current = 0;
int robot_direction = 0;
int current_direction = 0;
int adjacent_direction = 0;
float old_f = 0;
float new_g = 0;
float new_f = 0;
int already_passed_six = 0;

// "Shared" A* Variables
long a_star_cnt = 0;
int num_links = 0;
float x_world_robot = 0;
float y_world_robot = 0;
float theta_world_robot = 0;
float x_world_target = 0;
float y_world_target = 0;


pose UpdateOptitrackStates(pose localROBOTps, int * flag);


void ComWithLinux(void) {

	int i = 0;
	TSK_sleep(100);

	while(1) {

		BCACHE_inv((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
		
		if (GET_DATA_FROM_LINUX) {

			if (newnavdata == 0) {
				newvref = ptrshrdmem->Floats_to_DSP[0];
				newturn = ptrshrdmem->Floats_to_DSP[1];
				newnavdata = 1;
			}

			CLR_DATA_FROM_LINUX;

		}

		if (GET_LVDATA_FROM_LINUX) {

			if (ptrshrdmem->DSPRec_size > 256) ptrshrdmem->DSPRec_size = 256;
				for (i=0;i<ptrshrdmem->DSPRec_size;i++) {
					fromLinuxstring[i] = ptrshrdmem->DSPRec_buf[i];
				}
				fromLinuxstring[i] = '\0';

				if (new_LV_data == 0) {
					sscanf(fromLinuxstring,"%f%f",&LVvalue1,&LVvalue2);
					new_LV_data = 1;
				}

			CLR_LVDATA_FROM_LINUX;

		}

		if ((tskcount%6)==0) {
			if (GET_LVDATA_TO_LINUX) {

				// Default
				// ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"1.0 1.0 1.0 1.0");
				// you would do something like this
				// ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",var1,var2,var3,var4);

				// ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f "
				// "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f ",
				// ROBOTps.x,
				// ROBOTps.y,
				// 255.0,
				// 255.0,
				// course_map[0], course_map[1], course_map[2], course_map[3], course_map[4], course_map[5], course_map[6], course_map[7],
				// course_map[8], course_map[9], course_map[10], course_map[11], course_map[12], course_map[13], course_map[14], course_map[15],
				// course_map[16], course_map[17], course_map[18], course_map[19], course_map[20], course_map[21], course_map[22], course_map[23],
				// course_map[24], course_map[25], course_map[26], course_map[27], course_map[28], course_map[29], course_map[30], course_map[31],
				// course_map[32], course_map[33], course_map[34], course_map[35], course_map[36], course_map[37], course_map[38], course_map[39],
				// course_map[40], course_map[41], course_map[42], course_map[43], course_map[44], course_map[45], course_map[46], course_map[47],
				// course_map[48], course_map[49], course_map[50], course_map[51], course_map[52], course_map[53], course_map[54], course_map[55],
				// course_map[56], course_map[57], course_map[58], course_map[59], course_map[60], course_map[61], course_map[62], course_map[63],
				// course_map[64], course_map[65], course_map[66], course_map[67], course_map[68], course_map[69], course_map[70], course_map[71],
				// course_map[72], course_map[73], course_map[74], course_map[75], course_map[76], course_map[77], course_map[78], course_map[79],
				// course_map[80], course_map[81], course_map[82], course_map[83], course_map[84], course_map[85], course_map[86], course_map[87]
				// );

				ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f "
				"%d %d %d %d %d %d "
				"%d %d %d %d %d %d "
				"%d %d %d %d %d %d "
				"%d %d %d %d %d %d "
				"%d %d %d %d %d %d "
				"%d %d %d %d %d %d ",
				ROBOTps.x,
				ROBOTps.y,
				255.0,
				255.0,
				grid[38].map,  grid[37].map,  grid[36].map,  grid[35].map, grid[34].map, grid[33].map,
				grid[46].map,  grid[45].map,  grid[44].map,  grid[43].map, grid[42].map, grid[41].map,
				grid[54].map,  grid[53].map,  grid[52].map,  grid[51].map, grid[50].map, grid[49].map,
				grid[62].map,  grid[61].map,  grid[60].map,  grid[59].map, grid[58].map, grid[57].map,
				grid[70].map,  grid[69].map,  grid[68].map,  grid[67].map, grid[66].map, grid[65].map,
				grid[78].map,  grid[77].map,  grid[76].map,  grid[75].map, grid[74].map, grid[73].map
				);

				for (i=0;i<ptrshrdmem->DSPSend_size;i++) {
					ptrshrdmem->DSPSend_buf[i] = toLinuxstring[i];
				}

				// Flush or write back source
				BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

				CLR_LVDATA_TO_LINUX;

			}
		}
		
		if (GET_DATAFORFILE_TO_LINUX) {

			// This is an example write to scratch
			// The Linux program SaveScratchToFile can be used to write the
			// ptrshrdmem->scratch[0-499] memory to a .txt file.
//			for (i=100;i<300;i++) {
//				ptrshrdmem->scratch[i] = (float)i;
//			}

			// Flush or write back source
//			BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
//
//			CLR_DATAFORFILE_TO_LINUX;

			// First make sure all scratch elements are zero
			for (i=0;i<500;i++) {
				ptrshrdmem->scratch[i] = 0;
			}
			// Write LADARdataX to scratch
			for (i=0;i<228;i++) {
				ptrshrdmem->scratch[i] = LADARdataX[i];
			}
			// Write LADARdataY to scratch
			for (i=0;i<228;i++) {
				ptrshrdmem->scratch[228+i] = LADARdataY[i];
			}
			// Flush or write back source
			BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

			CLR_DATAFORFILE_TO_LINUX;
		}

		tskcount++;
		TSK_sleep(40);
	}


}


/*
 *  ======== main ========
 */
Void main()
{

	int i = 0;

	// unlock the system config registers.
	SYSCONFIG->KICKR[0] = KICK0R_UNLOCK;
	SYSCONFIG->KICKR[1] = KICK1R_UNLOCK;

	SYSCONFIG1->PUPD_SEL |= 0x10000000;  // change pin group 28 to pullup for GP7[12/13] (LCD switches)

	// Initially set McBSP1 pins as GPIO ins
	CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[1], 0x88888880);  // This is enabling the McBSP1 pins

	CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
	SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13 
	CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
	SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13


	//Rick added for LCD DMA flagging test
	GPIO_setDir(GPIO_BANK0, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setOutput(GPIO_BANK0, GPIO_PIN8, OUTPUT_HIGH);

	GPIO_setDir(GPIO_BANK0, GPIO_PIN0, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN1, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN2, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN3, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN4, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK0, GPIO_PIN5, GPIO_INPUT);  
	GPIO_setDir(GPIO_BANK0, GPIO_PIN6, GPIO_INPUT);

	GPIO_setDir(GPIO_BANK7, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN9, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN10, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN11, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN12, GPIO_INPUT);
	GPIO_setDir(GPIO_BANK7, GPIO_PIN13, GPIO_INPUT); 

	GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_HIGH);  
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN9, OUTPUT_HIGH);
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN10, OUTPUT_HIGH);
	GPIO_setOutput(GPIO_BANK7, GPIO_PIN11, OUTPUT_HIGH);  

	CLRBIT(SYSCONFIG->PINMUX[13], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[13], 0x88888811); //Set GPIO 6.8-13 to GPIOs and IMPORTANT Sets GP6[15] to /RESETOUT used by PHY, GP6[14] CLKOUT appears unconnected

	#warn GP6.15 is also connected to CAMERA RESET This is a Bug in my board design Need to change Camera Reset to different IO.

	GPIO_setDir(GPIO_BANK6, GPIO_PIN8, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN9, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN10, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN11, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN12, GPIO_OUTPUT);
	GPIO_setDir(GPIO_BANK6, GPIO_PIN13, GPIO_INPUT);   


   // on power up wait until Linux has initialized Timer1
	while ((T1_TGCR & 0x7) != 0x7) {
	  for (index=0;index<50000;index++) {}  // small delay before checking again

	}

	USTIMER_init();
	
	// Turn on McBSP1
	EVMOMAPL138_lpscTransition(PSC1, DOMAIN0, LPSC_MCBSP1, PSC_ENABLE);

    // If Linux has already booted It sets a flag so no need to delay
    if ( GET_ISLINUX_BOOTED == 0) {
    	USTIMER_delay(4*DELAY_1_SEC);  // delay allowing Linux to partially boot before continuing with DSP code
    }
	   
	// init the us timer and i2c for all to use.
	I2C_init(I2C0, I2C_CLK_100K);
	init_ColorVision();	
	init_LCD_mem(); // added rick

	EVTCLR0 = 0xFFFFFFFF;
	EVTCLR1 = 0xFFFFFFFF;
	EVTCLR2 = 0xFFFFFFFF;
	EVTCLR3 = 0xFFFFFFFF;	

	init_DMA();
	init_McBSP();

	init_LADAR();

	CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
	SETBIT(SYSCONFIG->PINMUX[1], 0x22222220);  // This is enabling the McBSP1 pins

	CLRBIT(SYSCONFIG->PINMUX[5], 0x00FF0FFF);
	SETBIT(SYSCONFIG->PINMUX[5], 0x00110111);  // This is enabling SPI pins

	CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
	SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13 
	CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
	SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13

	init_LCD();
    
	LADARps.x = 3.5/12; // 3.5/12 for front mounting
	LADARps.y = 0;
	LADARps.theta = 1;  // not inverted

	OPTITRACKps.x = 0;
	OPTITRACKps.y = 0;
	OPTITRACKps.theta = 0;

	for(i = 0;i<LADAR_MAX_DATA_SIZE;i++)
	{ LADARdistance[i] = LADAR_MAX_READING; } //initialize all readings to max value.

	// ROBOTps will be updated by Optitrack during gyro calibration
	// TODO: specify the starting position of the robot
	ROBOTps.x = 0;			//the estimate in array form (useful for matrix operations)
	ROBOTps.y = 0;
	ROBOTps.theta = 0;  // was -PI: need to flip OT ground plane to fix this
	x_pred[0][0] = ROBOTps.x; //estimate in structure form (useful elsewhere)
	x_pred[1][0] = ROBOTps.y;
	x_pred[2][0] = ROBOTps.theta;

	// flag pins
	GPIO_setDir(IMAGE_TO_LINUX_BANK, IMAGE_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(OPTITRACKDATA_FROM_LINUX_BANK, OPTITRACKDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATA_TO_LINUX_BANK, DATA_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATA_FROM_LINUX_BANK, DATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(DATAFORFILE_TO_LINUX_BANK, DATAFORFILE_TO_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(LVDATA_FROM_LINUX_BANK, LVDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
	GPIO_setDir(LVDATA_TO_LINUX_BANK, LVDATA_TO_LINUX_FLAG, GPIO_OUTPUT);


	CLR_OPTITRACKDATA_FROM_LINUX;  // Clear = tell linux DSP is ready for new Opitrack data
	CLR_DATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new data
	CLR_DATAFORFILE_TO_LINUX;  // Clear = linux not requesting data
	SET_DATA_TO_LINUX;  // Set = put float array data into shared memory for linux
	SET_IMAGE_TO_LINUX;  // Set = put image into shared memory for linux
	CLR_LVDATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new LV data
	SET_LVDATA_TO_LINUX;  // Set = put LV char data into shared memory for linux

    // clear all possible EDMA 
	EDMA3_0_Regs->SHADOW[1].ICR = 0xFFFFFFFF;

	for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
		// initialize grid array
		grid[i].x = i % X_GRID_SIZE;	// assign every grid space a x and y value
		grid[i].y = i / X_GRID_SIZE;
		grid[i].map = course_map[i];	// initialize map
        grid[i].hits = 0;
		grid[i].h = 0;
		grid[i].g = 0;
		grid[i].f = 1000;
		grid[i].link = i;	// link every grid space to itself
		grid[i].open = 0;
		grid[i].closed = 0;
		grid[i].recognized_cnt = 0;
	}

	// initialize waypoints array
	for (i = 0; i < 50; i++) {
		waypoints[i].x = 0;
		waypoints[i].y = 0;
	}

	// initialize target points
	target_points[0].x = 0;			target_points[0].y = -1;	// starting point
	target_points[1].x = -5;		target_points[1].y = -3;	// point 1
	target_points[2].x = 3;			target_points[2].y = 7; 	// point 2
	target_points[3].x = -3;		target_points[3].y = 7; 	// point 3
	target_points[4].x = 5;			target_points[4].y = -3; 	// point 4
	target_points[5].x = 0;			target_points[5].y = 11; 	// point 5
	target_points[6].x = -2;		target_points[6].y = -4; 	// blue chute
	target_points[7].x = 2;			target_points[7].y = -4; 	// orange chute
	target_points[8].x = 0;			target_points[8].y = 0; 	// finishing point
}
	

long timecount= 0;
int whichled = 0;
// This SWI is Posted after each set of new data from the F28335
void RobotControl(void) {

	int newOPTITRACKpose = 0;
	int i = 0;

	if (0==(timecount%1000)) {
		switch(whichled) {
		case 0:
			SETREDLED;
			CLRBLUELED;
			CLRGREENLED;
			whichled = 1;
			break;
		case 1:
			CLRREDLED;
			SETBLUELED;
			CLRGREENLED;
			whichled = 2;
			break;
		case 2:
			CLRREDLED;
			CLRBLUELED;
			SETGREENLED;
			whichled = 0;
			break;
		default:
			whichled = 0;
			break;
		}
	}
	
	// get data from ColorVision.c
	if (new_coordata) {
		blue_x = new_blue_x;
		blue_y = new_blue_y;
		blue_num_pix = new_blue_num_pix;
		blue_num_obj = new_blue_num_obj;

		red_x = new_red_x;
		red_y = new_red_y;
		red_num_pix = new_red_num_pix;
		red_num_obj = new_red_num_obj;

		new_coordata = 0;
	}

	if (GET_OPTITRACKDATA_FROM_LINUX) {

		if (new_optitrack == 0) {
			for (i=0;i<OPTITRACKDATASIZE;i++) {
				Optitrackdata[i] = ptrshrdmem->Optitrackdata[i];
			}
			new_optitrack = 1;
		}

		CLR_OPTITRACKDATA_FROM_LINUX;

	}

	if (new_optitrack == 1) {
		OPTITRACKps = UpdateOptitrackStates(ROBOTps, &newOPTITRACKpose);
		new_optitrack = 0;
	}

	// using 400deg/s gyro
	gyro = adcA3*3.0/4096.0;
	if (settlegyro < SETTLETIME) {
		settlegyro++;
		if (settlegyro < (SETTLETIME/3)) {
			// do nothing
		} else if (settlegyro < (2*SETTLETIME/3)) {
			gyro_zero = gyro_zero + gyro/(SETTLETIME/3);
		} else {
			gyro_drift += (((gyro-gyro_zero) + old_gyro)*.0005)/(SETTLETIME/3);
			old_gyro = gyro-gyro_zero;
		}
		if(settlegyro%500 == 0) {
			LCDPrintfLine(1,"Cal Gyro -- %.1fSecs", (float)(SETTLETIME - settlegyro)/1000.0 );
			LCDPrintfLine(2,"");
		}

		newOPTITRACKpose = 0;

		SetRobotOutputs(0,0,0,0,0,0,0,0,0,0);
	}
	else {

		gyro_angle = gyro_angle - ((gyro-gyro_zero) + old_gyro)*.0005 + gyro_drift; 
		old_gyro = gyro-gyro_zero;
		gyro_radians = (gyro_angle * (PI/180.0)*400.0*gyro4x_gain);

		// Kalman filtering
		vel1 = (enc1 - enc1old)/(193.0*0.001);	// calculate actual velocities
		vel2 = (enc2 - enc2old)/(193.0*0.001);
		if (fabsf(vel1) > 10.0) vel1 = vel1old;	// check for encoder roll-over should never happen
		if (fabsf(vel2) > 10.0) vel2 = vel2old;
		enc1old = enc1;	// save past values
		enc2old = enc2;
		vel1old = vel1;
		vel2old = vel2;

		// Step 0: update B, u
		B[0][0] = cosf(ROBOTps.theta)*0.001;
		B[1][0] = sinf(ROBOTps.theta)*0.001;
		B[2][1] = 0.001;
		u[0][0] = 0.5*(vel1 + vel2);	// linear velocity of robot
		u[1][0] = (gyro-gyro_zero)*(PI/180.0)*400.0*gyro4x_gain;	// angular velocity in rad/s (negative for right hand angle)

		// Step 1: predict the state and estimate covariance
		Matrix3x2_Mult(B, u, Bu);					// Bu = B*u
		Matrix3x1_Add(x_pred, Bu, x_pred, 1.0, 1.0); // x_pred = x_pred(old) + Bu
		Matrix3x3_Add(P_pred, Q, P_pred, 1.0, 1.0);	// P_pred = P_pred(old) + Q
		// Step 2: if there is a new measurement, then update the state
		if (1 == newOPTITRACKpose) {
			newOPTITRACKpose = 0;
			z[0][0] = OPTITRACKps.x;	// take in the LADAR measurement
			z[1][0] = OPTITRACKps.y;
			// fix for OptiTrack problem at 180 degrees
			if (cosf(ROBOTps.theta) < -0.99) {
				z[2][0] = ROBOTps.theta;
			}
			else {
				z[2][0] = OPTITRACKps.theta;
			}
			// Step 2a: calculate the innovation/measurement residual, ytilde
			Matrix3x1_Add(z, x_pred, ytilde, 1.0, -1.0);	// ytilde = z-x_pred
			// Step 2b: calculate innovation covariance, S
			Matrix3x3_Add(P_pred, R, S, 1.0, 1.0);							// S = P_pred + R
			// Step 2c: calculate the optimal Kalman gain, K
			Matrix3x3_Invert(S, S_inv);
			Matrix3x3_Mult(P_pred,  S_inv, K);								// K = P_pred*(S^-1)
			// Step 2d: update the state estimate x_pred = x_pred(old) + K*ytilde
			Matrix3x1_Mult(K, ytilde, temp_3x1);
			Matrix3x1_Add(x_pred, temp_3x1, x_pred, 1.0, 1.0);
			// Step 2e: update the covariance estimate   P_pred = (I-K)*P_pred(old)
			Matrix3x3_Add(eye3, K, temp_3x3, 1.0, -1.0);
			Matrix3x3_Mult(temp_3x3, P_pred, P_pred);
		}	// end of correction step
	
		// set ROBOTps to the updated and corrected Kalman values.
		ROBOTps.x = x_pred[0][0];
		ROBOTps.y = x_pred[1][0];
		ROBOTps.theta = x_pred[2][0];

		// calculate new path if target is reached
		if ((flag_new_path_calculating == 0) && (current_waypoint == -1)) {
			flag_new_path = 1;
			current_target++;
		}

//		// obstacle detection
//		for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
//			old_map = grid[i].map;
//			if ((ROBOTps.y > 0) && (grid[i].hits > HITS_THRESHOLD)) {
//				grid[i].map = 1;
//			}
//			new_map = grid[i].map;
//			// calculate new path if new obstacle is detected
//			if ((flag_new_path_calculating == 0) && (new_map != old_map)) {
//				flag_new_path = 1;
//			}
//		}

		// obstacle detection
		for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
			if ((ROBOTps.y > 0) && (grid[i].hits > HITS_THRESHOLD)) {
				grid[i].recognized_cnt++;
			}

			// calculate new path if new obstacle is detected
			if ((flag_new_path_calculating == 0) && (grid[i].recognized_cnt == REC_CNT_THRESHOLD)) {
				flag_new_path = 1;
				grid[i].map = 1;
				grid[i].recognized_cnt++;
			}
		}

		// plan new robot path
		if(flag_new_path && (flag_new_path_calculating == 0)) {
			flag_new_path_calculating = 1;
			flag_new_path = 0;
			SEM_post (&SEM_a_star);
		}

		// stop robot movement if A* is running or if robot has reached finishing point
		if ((flag_new_path_calculating) || (current_target > 8)) {
			vref = 0;
			turn = 0;
		}
		else {
			if (current_waypoint == -1) {
				flag_bounds_error = 1;
			}
			// uses xy code to step through an array of positions
			if (!flag_bounds_error && xy_control(&vref, &turn, 2.0, ROBOTps.x, ROBOTps.y, waypoints[current_waypoint].x, waypoints[current_waypoint].y, ROBOTps.theta, 0.25, 0.75)) {
				current_waypoint--; // "remove" waypoint from stack
			}
		}
		
		if ((newLADARdata == 1)) {
			newLADARdata = 0;
			for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
				grid[i].hits = 0;
			}
			for (i=0;i<228;i++) {

				LADARdistance[i] = newLADARdistance[i];
				LADARangle[i] = newLADARangle[i];
				LADARdataX[i] = newLADARdataX[i];
				LADARdataY[i] = newLADARdataY[i];

				x_grid_obst = floorf(LADARdataX[i]/2.0) + 4;
				y_grid_obst = floorf(LADARdataY[i]/2.0) + 4;

				obst_grid = x_grid_obst + (y_grid_obst * X_GRID_SIZE);

				if (obst_grid < (X_GRID_SIZE * Y_GRID_SIZE) && (fabsf(turn) < 0.5)) {
					grid[obst_grid].hits++;
				}
			}
		}


		// calculate blue error
		blue_error = follow_ref - blue_x;
		red_error = follow_ref - red_x;


		// calculate blue golf ball distance
		if ((-0.0098*blue_y + 0.37) != 0) {
			blue_dist = 1/(-0.0098*blue_y + 0.37);
		}
		if ((-0.0098*red_y + 0.37) != 0) {
			red_dist = 1/(-0.0098*red_y + 0.37);
		}
	    vref = 1.0;
	    if (blue_num_obj >= 1 && red_num_obj >= 1){
	    	if (blue_dist >= red_dist){
	    		turn = Kp_ball * blue_error;
	    	}
	    	else{
	    		turn = Kp_ball * red_error;
	    	}
	    }
	    else if (blue_num_obj >= 1){
	    	turn = Kp_ball * blue_error;
	    }
	    else if (red_num_obj >= 1){
	    	turn = Kp_ball * red_error;
	    }



		if ((timecount%200)==0) {

			LCDPrintfLine(1,"X%.0f Y%.0f T%.0f", x_world_robot, y_world_robot, theta_world_robot);
			LCDPrintfLine(2,"A*%d WX%.0f WY%.0f CT%d", flag_new_path_calculating, waypoints[current_waypoint].x, waypoints[current_waypoint].y, current_target);
		}

		SetRobotOutputs(vref,turn,0,0,0,0,0,0,0,0);

		timecount++;
	}
}

pose UpdateOptitrackStates(pose localROBOTps, int * flag) {

	pose localOPTITRACKps;

	// Check for frame errors / packet loss
	if (previous_frame == Optitrackdata[OPTITRACKDATASIZE-1]) {
		frame_error++;
	}
	previous_frame = Optitrackdata[OPTITRACKDATASIZE-1];

	// Set local trackableID if first receive data
	if (firstdata){
		//trackableID = (int)Optitrackdata[OPTITRACKDATASIZE-1]; // removed to add new trackableID in shared memory
		trackableID = Optitrackdata[OPTITRACKDATASIZE-2];
		firstdata = 0;
	}

	// Check if local trackableID has changed - should never happen
	if (trackableID != Optitrackdata[OPTITRACKDATASIZE-2]) {
		trackableIDerror++;
		// do some sort of reset(?)
	}

	// Save position and yaw data
	if (isnan(Optitrackdata[0]) != 1) {  // this checks if the position data being received contains NaNs
		// check if x,y,yaw all equal 0.0 (almost certainly means the robot is untracked)
		if ((Optitrackdata[0] != 0.0) && (Optitrackdata[1] != 0.0) && (Optitrackdata[2] != 0.0)) {
			// save x,y
			// adding 2.5 so everything is shifted such that optitrack's origin is the center of the arena (while keeping all coordinates positive)
			// This was the old way for Optitrack coordinates
			//localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
			//localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

			// This is the new coordinates for Motive
			localOPTITRACKps.x = -1.0*Optitrackdata[0]*FEETINONEMETER; 
			localOPTITRACKps.y = Optitrackdata[1]*FEETINONEMETER+4.0;

			// make this a function
			temp_theta = fmodf(localROBOTps.theta,(float)(2*PI));//(theta[trackableID]%(2*PI));
			tempOPTITRACK_theta = Optitrackdata[2];
			if (temp_theta > 0) {
				if (temp_theta < PI) {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA > 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta > (PI/2)) {
							// THETA > 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA > 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA > 0, kal in QIII, OT in QIII
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta > (3*PI/2)) {
							// THETA > 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA > 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = (floorf((localROBOTps.theta)/((float)(2.0*PI))))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			} else {
				if (temp_theta > -PI) {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA < 0, kal in QIII/IV, OT in QIII/IV
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta < (-PI/2)) {
							// THETA < 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA < 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA < 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta < (-3*PI/2)) {
							// THETA < 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA < 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			}
			*flag = 1;
		}
	}
	return localOPTITRACKps;
}

// A* Path Planning Algorithm
void a_star (void) {
    int i = 0;

	while (1) {
		SEM_pend (&SEM_a_star, SYS_FOREVER);
		a_star_cnt++;

		// inputs to A* algorithm
		x_world_robot = ROBOTps.x;
		y_world_robot = ROBOTps.y;
		theta_world_robot = ROBOTps.theta;
		x_world_target = target_points[current_target].x;
		y_world_target = target_points[current_target].y;

		// reinitialize variables
		x_grid_robot = 0;
		y_grid_robot = 0;
		x_grid_target = 0;
		y_grid_target = 0;
		i = 0;
		robot_grid = 0;
		target_grid = 0;
		flag_path_found = 0;
		task_current = 0;
		adjacent = 0;
		link_found = 0;
		current_link = 0;
		flag_list_started = 0;
		flag_error_invalid_obstacle = 0;
		flag_error_no_path_found = 0;
		old_current = 0;
		robot_direction = 0;
		current_direction = 0;
		adjacent_direction = 0;
		old_f = 0;
		new_g = 0;
		new_f = 0;
		num_links = 0;

		// reinitialize grid array
		for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
			grid[i].h = 0;
			grid[i].g = 0;
			grid[i].f = 1000;
			grid[i].link = i;	// link every grid space to itself
			grid[i].open = 0;
			grid[i].closed = 0;
		}

		// convert theta to a value between -PI and PI
		if (theta_world_robot > PI) {
			theta_world_robot = theta_world_robot - 2.0*PI*floorf((theta_world_robot+PI)/(2.0*PI));
		} else if (theta_world_robot < -PI) {
			theta_world_robot = theta_world_robot - 2.0*PI*ceilf((theta_world_robot-PI)/(2.0*PI));
		} else {
			theta_world_robot = theta_world_robot;
		}

		// calculate robot in grid coordinate frame
		x_grid_robot = floorf(x_world_robot/2.0) + 4;
		y_grid_robot = floorf(y_world_robot/2.0) + 4;
		// calculate target in grid coordinate frame
		x_grid_target = floorf(x_world_target/2.0) + 4;
		y_grid_target = floorf(y_world_target/2.0) + 4;
		// calculate robot grid space
		robot_grid = x_grid_robot + (y_grid_robot * X_GRID_SIZE);
		// calculate target grid space
		target_grid = x_grid_target + (y_grid_target * X_GRID_SIZE);

		// report error if obstacle is in robot grid space
		if (grid[robot_grid].map) {
			flag_error_invalid_obstacle = 1;
		}
		// report error if obstacle is in target grid space
		if (grid[target_grid].map) {
			flag_error_invalid_obstacle = 1;
		}

		if (flag_error_invalid_obstacle == 0) {
			// calculate h values using Manhattan distance
			for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
				grid[i].h = H_COST*(abs(grid[i].x - x_grid_target) + abs(grid[i].y - y_grid_target));
			}

			// calculate robot current direction
			// current direction is right
			if ((theta_world_robot <= PI/4) && (theta_world_robot >= -PI/4)) {
				robot_direction = 1;
			}
			// current direction is up
			else if (theta_world_robot <= 3*PI/4) {
				robot_direction = X_GRID_SIZE;
			}
			// current direction is down
			else if (theta_world_robot >= -3*PI/4) {
				robot_direction = -X_GRID_SIZE;
			}
			// current direction is left
			else {
				robot_direction = -1;
			}

			task_current = robot_grid;
			if (flag_error_invalid_obstacle == 0) {
				while (flag_path_found == 0) {
					for (i = 0; i < 4; i++) {
						// add grid space to closed list and remove from open list
						grid[task_current].closed = 1;
						grid[task_current].open = 0;

						// check all 4 adjacent grid spaces
						// note that edge cases do not need to be checked for because the map has a 1 grid space buffer around entire outside
						switch (i) {
							// check grid space to right
							case 0:
								adjacent = task_current + 1;
								break;
							// check grid space to left
							case 1:
								adjacent = task_current - 1;
								break;
							// check grid space above
							case 2:
								adjacent = task_current + X_GRID_SIZE;
								break;
							// check grid space below
							case 3:
								adjacent = task_current - X_GRID_SIZE;
								break;
						}

						// find current grid link direction
						current_direction = task_current - grid[task_current].link;
						if (task_current == robot_grid) {
							current_direction = robot_direction;
						}
						// find adjacent grid direction
						adjacent_direction = adjacent - task_current;

						// check if adjacent grid space is target grid space
						if (adjacent == target_grid) {
							flag_path_found = 1;
							i = 4;
						}

						// check if adjacent grid space is on closed list or obstacle
						if ((grid[adjacent].closed || grid[adjacent].map) == 0) {
							grid[adjacent].open = 1;	// add grid space to open list

							// check if adjacent grid space is on open list, calculate G and F value, and assign parent
							if (grid[adjacent].open) {
								old_f = grid[adjacent].f;
								new_g = grid[task_current].g + G_MOVE_COST;
								// add extra movement cost for turning
								if (adjacent_direction != current_direction) {
									new_g += G_TURN_COST;
								}
								new_f = grid[adjacent].h + new_g;
								// if new f value lower than old f value
								if (new_f < old_f) {
									grid[adjacent].g = new_g;
									grid[adjacent].f = new_f;
									grid[adjacent].link = task_current;
								}
							}
						}
					}

					// determine next grid space to be analyzed based on lowest F value
					old_current = task_current;
					for (i = 0; i < (X_GRID_SIZE * Y_GRID_SIZE); i++) {
						// start with first grid space on open list
						if (grid[i].open) {
							if (flag_list_started == 0) {
								task_current = i;
								flag_list_started = 1;
							}
							// calculate grid space with lowest F value
							if (grid[i].f < grid[task_current].f) {
								task_current = i;
							}
						}
					}
					flag_list_started = 0;

					// report error if no path is found
					if (task_current == old_current) {
						flag_error_no_path_found = 1;
					}
				}

				if (flag_error_no_path_found == 0) {
					// calculate number of links in planned path
					current_link = target_grid;
					link_found = 0;
					while (link_found == 0) {
						if (grid[current_link].link == current_link) {
							link_found = 1;
						}
						else {
							num_links++;
							current_link = grid[current_link].link;
						}
					}

					// update waypoints array with new path
					// and target grid space unless target point is point 5
					waypoints[0].x = x_world_target;
					waypoints[0].y = y_world_target;

					// disregard target grid space unless target point is point 5
					if (current_target == 5) {
						waypoints[1].x = (grid[target_grid].x - 4)*2.0 + 1;
						waypoints[1].y = (grid[target_grid].y - 4)*2.0 + 1;
						i = 2;
						num_links++;
					}
					else {
						i = 1;
					}

					current_link = grid[target_grid].link;
					for (; i < num_links; i++) {
						waypoints[i].x = (grid[current_link].x - 4)*2.0 + 1;
						waypoints[i].y = (grid[current_link].y - 4)*2.0 + 1;
						if (i < (num_links - 1)) {
							current_link = grid[current_link].link;
						}
					}

					// disregard robot grid space unless target point is point 6 and the planned path is to the right
					if ((current_target == 6) && ((current_link == 68) || (current_link == 77)) && (already_passed_six == 0)) {
						waypoints[num_links].x = (grid[76].x - 4)*2.0 + 1;
						waypoints[num_links].y = (grid[76].y - 4)*2.0 + 1;
						num_links++;
						already_passed_six = 1;
					}
				}
			}
		}

		flag_new_path_calculating = 0;
		current_waypoint = num_links - 1;
	}
}

