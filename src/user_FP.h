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

#define NUM_PIX_THRES_BLUE 25
#define NUM_PIX_THRES_ORANGE 25
//flags to make sure we track the same ball
#define NO_BALL 0
#define BLUE_FLAG 1
#define ORANGE_FLAG 2
int ball_track_flag = NO_BALL;

float Kp_ball = 0.05;
float blue_error = 0;
float red_error = 0;
float blue_follow_ref = 0;
float red_follow_ref = 35.0;

float blue_dist = 0.0;
float red_dist = 0.0;

int time_dump = 1500;
int time_ball_down = 100;
int time_ball_up = 0;

//vars and declarations for ball collection mechanism
#define BLUE_OPEN 6.5
#define BLUE_CLOSE 10.75
#define ORANGE_OPEN 8
#define ORANGE_CLOSE 4
#define OPEN_DELAY 600
float orange_door = ORANGE_OPEN;
float blue_door = BLUE_OPEN;
float K_ball_turn = 0.05;

//basic avoidance vars
float avg_right_LADAR;
float avg_left_LADAR;
float turn_thresh = 400.0;
float turn_right;
float turn_left;
int n = 1;

int blue_detected = 0;
int orange_detected = 0;
float blue_ball_array[6] = {-11.0, 0.0, -11.0, 0.3, -11.0, 0.6};
float orange_ball_array[6] = {-11.0, 0.9, -11.0, 1.2, -11.0, 1.5};
int framecount = 0;
int dec_ball=0;

extern int new_coordata;

int ball_debounce = 0;
int ball_collected = 0;

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
#define REC_CNT_THRESHOLD 3

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
tile grid[X_GRID_SIZE * Y_GRID_SIZE];
map_point target_points[8];
map_point waypoints[50];
int current_target = -1;
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
int already_passed_five = 0;

// "Shared" A* Variables
long a_star_cnt = 0;
int num_links = 0;
float x_world_robot = 0;
float y_world_robot = 0;
float theta_world_robot = 0;
float x_world_target = 0;
float y_world_target = 0;

/* Begin State Machine State Declarations */
#define PATH_NAV 0
#define BALL_NAV 1
#define BALL_DUMP_BLUE 2
#define BALL_DUMP_ORANGE 3
char nav_state = PATH_NAV;
/* End State Machine State Declarations */

/* Begin Audio Feedback Declarations */
#define AF_SPEECH 1
#define AF_SOUND_FILE 2
void play_speech(char * words);
void play_sound_file(char * filename);

char * catchphrases[50] = {
	"You say tomayto, I say tomato. You say potato, I also say potato."
};
/* End Audio Feedback Declarations */
