#ifndef _BT4_GSENSOR_H_
#define _BT4_GSENSOR_H_

//走步运动传感器
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

#define TIMEWINDOW_MIN    15  	//时间窗，单位20ms
#define TIMEWINDOW_MAX    150		//时间窗，单位20ms
#define REGULATION	      3			//认为找到稳定规律所需要的步数
#define INVALID		        3			//认为失去稳定规律所需要的步数

#define VPP_MIN 250


#define ADC_MIM_VALUE (-4096)
#define ADC_MAX_VALUE 4095

#define MMA8452Q_AXIS_X          0
#define MMA8452Q_AXIS_Y          1
#define MMA8452Q_AXIS_Z          2
#define MMA8452Q_AXES_NUM        3
#define MMA8452Q_DATA_LEN        6

#define MMA8452Q_SUCCESS 0
typedef signed   char   int8;
typedef unsigned char   uint8;

typedef signed   short  int16;
typedef unsigned short  uint16;

typedef signed   long   int32;
typedef unsigned long   uint32;

typedef unsigned char   bool;

typedef uint8           halDataAlign_t;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif

/**/
/*----------------------------------------------------------------------------*/

#define BT4_SYNC1 (0)
#define BT4_SYNC2 (1)
#define BT4_CMD   (2)
#define BT4_DATA1 (3)
#define BT4_DATA2 (4)
#define BT4_DATA3 (5)
#define BT4_DATA4 (6)
#define BT4_DATA5 (7)
#define BT4_DATA6 (8)

#define BT4_DATA7 (9)
#define BT4_DATA8 (10)
#define BT4_DATA9 (11)
#define BT4_DATA10 (12)

struct scale_factor{
    uint8  whole;
    uint8  fraction;
};

struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};

/*当前值处于哪个区间*/
#define STATE_IDLE 0
#define STATE_LOW 1
#define STATE_HIGH 2

/*找到一步或没找到的原因*/
#define FIND_IDLE 0
#define FIND_OK 1
#define WIN_TOO_SMALL 2
#define WIN_TOO_LARGE 3
#define FIND_NOT_REACH_REGULATION 4



/*峰值检测返回值*/
#define FIND_NGT_PEAK 1
#define FIND_POST_PEAK 2
#define FIND_NO_PEAK

#define SUCESS 0
#define ERR_PEAK_DET_SIZE_SMALL 0x80


void gsensor_read();
void gsensor_init();
void gsensor_update();
void step_init();
void step_counter();

extern int16 g_acc[MMA8452Q_AXES_NUM]; 	//运动传感器保存的数据
#endif
