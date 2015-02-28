#include <string.h>
#include<stdio.h>
#include "gsensor_drv.h"

/**/
uint32 g_DebugCnt = 0;

static struct data_resolution mma8452q_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   /*+/-2g  in 12-bit resolution:  3.9 mg/LSB*/
    {{ 2, 0}, 512},   /*+/-4g  in 12-bit resolution:  7.8 mg/LSB*/
    {{3, 9},  256},   /*+/-8g  in 12-bit resolution: 15.6 mg/LSB*/
    {{ 15, 6}, 64},   /*+/-2g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 31, 3}, 32},   /*+/-4g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 62, 5}, 16},   /*+/-8g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/            
};

/*----------------------------------------------------------------------------*/
extern FILE *g_FileRes;

static int MMA8452Q_CheckDeviceID()
{
	
	return MMA8452Q_SUCCESS;
}

static int MMA8452Q_SetPowerMode( bool enable)
{
	
	return MMA8452Q_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
//set detect range

static int MMA8452Q_SetDataFormat(uint8 dataformat)
{


	return MMA8452Q_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MMA8452Q_SetBWRate(uint8 bwrate)
{
	
	
	return MMA8452Q_SUCCESS;    
}

/*----------------------------------------------------------------------------*/

static int MMA8452Q_SetDataResolution(uint8 dataresolution)
{
	return 0;
}

/*----------------------------------------------------------------------------*/

static int MMA8452Q_ResetCalibration()
{

	return 0;    
}

/*----------------------------------------------------------------------------*/

static int MMA8452Q_Init(int reset_cali)
{
	
	return MMA8452Q_SUCCESS;
}

/*----------------------------------------------------------------------------*/



int16 g_acc[MMA8452Q_AXES_NUM]; 	//运动传感器保存的数据

static int mma8452q_read_data(int16 data[MMA8452Q_AXES_NUM])
{  
#if 0
	uint8 buf[MMA8452Q_DATA_LEN] = {0};
	int err = 0;
	
	err = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DATAX0, buf, 2); 
	if( err ) {
		return err;
	}

	err = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DATAY0, &buf[MMA8452Q_AXIS_Y*2], 2); 
	if( err ) {
		return err;
	}

	err = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DATAZ0, &buf[MMA8452Q_AXIS_Z*2], 2); 
	if( err ) {
		return err;
	}	

	data[MMA8452Q_AXIS_X] = (int16)((buf[MMA8452Q_AXIS_X*2] << 8) | (buf[MMA8452Q_AXIS_X*2+1]));
	data[MMA8452Q_AXIS_Y] = (int16)((buf[MMA8452Q_AXIS_Y*2] << 8) | (buf[MMA8452Q_AXIS_Y*2+1]));
	data[MMA8452Q_AXIS_Z] = (int16)((buf[MMA8452Q_AXIS_Z*2] << 8) | (buf[MMA8452Q_AXIS_Z*2+1]));
   
	data[MMA8452Q_AXIS_X] = data[MMA8452Q_AXIS_X]>>4;
	data[MMA8452Q_AXIS_Y] = data[MMA8452Q_AXIS_Y]>>4;
	data[MMA8452Q_AXIS_Z] = data[MMA8452Q_AXIS_Z]>>4;
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/

static int mma8452q_read()
{


	return 0;
}

/*********************************************************************/
/*********************************************************************/


//ADC采样通道



uint32 STEPS;	//总步数

uint8 itemp,jtemp;								
uint8 _bad_flag[3];									
uint8  sampling_counter;								
int16 _adresult[3];									
int16 _max[3]={0,0,0};
int16 _min[3]={1000,1000,1000};
int16 _dc[3]={500,500,500};


int16 _vpp[3]={30,30,30};	
int16  _precision[3]={5,5,5};	
int16 _old_fixed[3];
int16 _new_fixed[3];


uint16 Interval=0;		//记录时间间隔数
uint8 TempSteps=0;		//记步缓存
uint8 InvalidSteps=0;	//无效步缓存
uint8 ReReg=2;			//记录是否重新开始寻找规律
						//	2-新开始
						//	1-已经开始，但是还没有找到规律
						//	0-已经找到规律
int16 highTh[3] = {0};
int16 lowTh[3] = {0};
uint8 currState[3] = {STATE_IDLE};
uint8 preState[3] = {STATE_IDLE};
uint8 findStepFlag[3] = {0};
uint8 chanSelFlag[3]={FALSE};
int16 vppSum,highThSum,lowThSum,maxSum, minSum, dcSum, adResSum, precisionSum,badFlagSum;
int16 preStateSum, currStateSum, oldDataSum, newDataSum;
int16 adc[2];
int16 adcfltOut[2];
#if 0
void lps_flt(int16 in, int16* out)
{
    int16 B[] = {8036, 8036};
    int16 A[] = {16384,-16696};
    int8 i;
    
	/*a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
							 - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
	*/
    /*Refresh input data, 0 is the oldest data*/
	adc[0] = adc[1];
	adc[1] = in;

    /*Refresh output data buf, 0 is the oldest data*/
	adcfltOut[0] = adcfltOut[1];

	/*filter*/
    adcfltOut[1]= ((int32)(B[0]) * adc[1] + (int32)(B[1]) * adc[0] - (int32)A[1] * adcfltOut[0]) >> 15;

    *out = adcfltOut[1];
     
	return;
}

uint8 peak_det(int16 pData, uint8 size)
{
    uint8 midIdx;
    uint8 i;
    
    if(size < 3)
    {
    	return ERR_PEAK_DET_SIZE_SMALL;
    }

    midIdx = size >> 2;

    /*Find positive peak*/
	for(i = 1; i <= midIdx; i ++)
	{
	    if(pData[i] < pData[i-1])
	    {
	        break;
	    }
	}

	if(i != (midIdx + 1))
	{
	    goto findNgtPeak;
	}

	for(i = midIdx + 1; i < size; i++)
	{
	    if(pData[i] > pData[i-1])
	    {
	        break;
	    }
	}

	if(i != (midIdx + 1))
	{
	    goto findNgtPeak;
	}

    return FIND_POST_PEAK;
    
findNgtPeak:
   	for(i = 1; i <= midIdx; i ++)
	{
	    if(pData[i] > pData[i-1])
	    {
	        break;
	    }
	}

	if(i != (midIdx + 1))
	{
	   return FIND_NO_PEAK;
	}

	for(i = midIdx + 1; i < size; i++)
	{
	    if(pData[i] < pData[i-1])
	    {
	        break;
	    }
	}

	if(i != (midIdx + 1))
	{
	    return FIND_NO_PEAK;
	}
     
	return FIND_NGT_PEAK;
}
#endif
/*------------------------------------------------------------------------------------------------------------------------
*Name: 		TimeWindow()
*Function:	实现"时间窗"算法,认为只有在有效"时间窗"内的记步才有效,而且起始时需要连续出现有效步才认为开始
*Input:		void
*Output: 	void
*------------------------------------------------------------------------------------------------------------------------*/
uint8 TimeWindow()
{
    uint8 ret = FIND_IDLE;
    
	if(ReReg==2)		//如果是新开始的第一步，直接在记步缓存中加1
	{
		TempSteps++;
		Interval=0;
		ReReg=1;
		InvalidSteps=0;	
		ret = FIND_NOT_REACH_REGULATION;
	}
	else				//如果不是新开始的第一步
	{
		if((Interval>=TIMEWINDOW_MIN)&&(Interval<=TIMEWINDOW_MAX))	//如果时间间隔在有效的时间窗内
		{
			InvalidSteps=0;	
			if(ReReg==1)					//如果还没有找到规律
			{
				TempSteps++;				//记步缓存加1
				if(TempSteps>=REGULATION)	//如果记步缓存达到所要求的规律数
				{
					ReReg=0;				//已经找到规律
					STEPS=STEPS+TempSteps;	//更新显示
					TempSteps=0;
					ret = FIND_OK;
				}
				else
				{
				    ret = FIND_NOT_REACH_REGULATION;
                }
				Interval=0;
			}
			else if(ReReg==0)				//如果已经找到规律，直接更新显示
			{
				STEPS++;
				TempSteps=0;
				Interval=0;
				ret = FIND_OK;
			}
		}
		else if(Interval<TIMEWINDOW_MIN)	//如果时间间隔小于时间窗下限
		{	
			if(ReReg==0)					//如果已经找到规律
			{
				if(InvalidSteps<255) 	InvalidSteps++;	//无效步缓存加1
				if(InvalidSteps>=INVALID)				//如果无效步达到所要求的数值，则重新寻找规律
				{	
					InvalidSteps=0;
					ReReg=1;
					TempSteps=0;
					Interval=0;
				}
				else					    //否则，只丢弃这一次的记步，但是继续记步，不需要重新寻找规律
				{
					Interval=0;
				}
			}
			else if(ReReg==1)				//如果还没有找到规律，则之前的寻找规律过程无效，重新寻找规律
			{
				InvalidSteps=0;	
				ReReg=1;
				TempSteps=0;
				Interval=0;
			}

			ret = WIN_TOO_SMALL;
		}
		else if(Interval>TIMEWINDOW_MAX)	//如果时间间隔大于时间窗上限，记步已经间断，重新寻找规律
		{
			InvalidSteps=0;	
			ReReg=1;						
			TempSteps=0;
			Interval=0;

			ret = WIN_TOO_LARGE;
		}
	}		

	return ret;
}
void updateStep()
{
	if(ReReg == 1)
	{
		STEPS=STEPS+TempSteps;	//更新显示
		TempSteps = 0;
	}
	else if(ReReg==0)				//如果已经找到规律，直接更新显示
    {
    	STEPS++;
    }
    
	return;
}
/*------------------------------------------------------------------------------------------------------------------------
*Name: 		step_counter()
*Function:	实现Pedometer计步的基本算法.
*Input:		void
*Output: 	void
*------------------------------------------------------------------------------------------------------------------------*/
void step_counter()
{
	uint8 i,j,tmp,idx1,idx2;
    uint8 sortChanId[3],chanSelNum;
	uint8 findStepFlagTmp, findStepFlag;
     
	

    g_DebugCnt ++;
	if(g_DebugCnt == 91)
	{
		g_DebugCnt = g_DebugCnt;
	}
	Interval++;
	
	//----------------------------------------------ADC采样----------------------//
	for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
	{
		_adresult[X_CHANNEL] = g_acc[X_CHANNEL];//;0;//g_acc[X_CHANNEL];
		_adresult[Y_CHANNEL] = g_acc[Y_CHANNEL];
		_adresult[Z_CHANNEL] = g_acc[Z_CHANNEL];
		
		if (_adresult[jtemp]>_max[jtemp])    {_max[jtemp]=_adresult[jtemp];}
		if (_adresult[jtemp]<_min[jtemp])    {_min[jtemp]=_adresult[jtemp];}
	}
	

    sampling_counter=sampling_counter+1;
	
	//----------------------------------计算动态门限和动态精度-----------------------//
    if (sampling_counter==50)
    {               
        sampling_counter=0;
				
        for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
        {
          _vpp[jtemp]=_max[jtemp]-_min[jtemp];
          _dc[jtemp]=_min[jtemp]+(_vpp[jtemp]>>1);
          
          highTh[jtemp] = _dc[jtemp] + ((_max[jtemp] - _dc[jtemp]) >> 1);
          lowTh[jtemp] = _dc[jtemp] - ((_dc[jtemp] - _min[jtemp]) >> 1);

          _bad_flag[jtemp]=0;
          badFlagSum = 0;

          //调整门限参数
          if (_vpp[jtemp]>=2000) 
          {
            _precision[jtemp]=1800; 
          } 
          else if (_vpp[jtemp]>=VPP_MIN)
          {            
            _precision[jtemp] = (_vpp[jtemp]*12)>>4;
          } 
          else { 
            _precision[jtemp]=100;
            _bad_flag[jtemp]=1;
          }

         
        }
#if 0
         /*排序确定3个通道的vpp大小*/
         for(i = X_CHANNEL;i <= Z_CHANNEL;i++)
         {
             sortChanId[i] = i;
         }

        for(i = X_CHANNEL;i <= Z_CHANNEL-1;i++)
        {
            for(j = X_CHANNEL;j <=Z_CHANNEL-i;j++)
            {
                idx1 = sortChanId[j];
                idx2 = sortChanId[j+1];
            	if(_vpp[idx1] < _vpp[idx2])
            	{
            	    tmp = sortChanId[j];
            	    sortChanId[j] = sortChanId[j+1];
                	sortChanId[j+1] = tmp;
               	}
        	}
        }

		memset(chanSelFlag,FALSE,sizeof(chanSelFlag));
        /*Vpp 最大的channel一定被选上*/
        idx1 = sortChanId[0];
        idx2 = sortChanId[1];
        chanSelFlag[idx1] = TRUE;
        
        /*如果vpp次大的channel和最大的channel接近，则也选上*/
        if(_vpp[idx2] > (_vpp[idx1] * 5 >> 3)  )
        {
            chanSelFlag[idx2] = TRUE;
            chanSelNum = 2;
        }
 #endif
        
        //简化处理，按正常佩戴方式，固定选择Y通道
        sortChanId[0] = Y_CHANNEL;
        chanSelFlag[Y_CHANNEL] = TRUE;
        chanSelNum = 1;
        if(_dc[X_CHANNEL] < 950)
        {
        	_bad_flag[Y_CHANNEL] = 1;
        }

		vppSum = 0;
		highThSum = 0;
		lowThSum = 0;
		precisionSum = 0;

        /*计算融合后的门限参数*/
        if(1 == chanSelNum)
        {
          idx1 = sortChanId[0];
          vppSum = _vpp[idx1];
          highThSum = highTh[idx1];
          lowThSum = lowTh[idx1];
          precisionSum = _precision[idx1];
          if(_bad_flag[idx1] == 1)
          {
              badFlagSum = 1;
          }
        
        }
        else if(2 == chanSelNum)
        {
          idx1 = sortChanId[0];
          idx2 = sortChanId[1];
          /*融合后参数计算暂时根据经验数据，今后再优化*/
          maxSum = _max[idx1] + _max[idx2];
          minSum = _min[idx1] +  _max[idx2] - (_vpp[idx2] >> 1);
          vppSum = maxSum - minSum;
          precisionSum = _precision[idx1] + (_precision[idx2] >> 1) ;          

          dcSum = minSum + (vppSum>>1);
          highThSum = dcSum + ((maxSum - dcSum) >> 1);
          lowThSum = dcSum - ((dcSum - minSum) >> 1);
      
          if((_bad_flag[idx1] == 1)||(_bad_flag[idx2] == 1))
          {
              badFlagSum = 1;
          }
        
        }

  	  for(jtemp = X_CHANNEL; jtemp <= Z_CHANNEL; jtemp++)
  	  {
  	  	_max[jtemp]= ADC_MIM_VALUE;
  	  	_min[jtemp]=ADC_MAX_VALUE;
  
  	  }
	}	
	//--------------------------线性移位寄存器--------------------------------------
    adResSum = 0;

	for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
	{  
	    if(chanSelFlag[jtemp] == TRUE)
	    {
	         adResSum += _adresult[jtemp];
	    }
	}

	if(adResSum > highThSum)
	{
		 preStateSum = currStateSum;
		 currStateSum = STATE_HIGH;
		/*从一个状态切换到另外一个状态，则寄存器值移位*/ 
		if(preStateSum!= currStateSum)
		{
			oldDataSum=  newDataSum;
		}
		/*记录正最大值*/
		if(adResSum > newDataSum)
		{
			 newDataSum = adResSum;
		}
	}
	else if(adResSum < lowThSum)
	{
	    preStateSum = currStateSum;
		currStateSum = STATE_LOW;
		/*从一个状态切换到另外一个状态，则寄存器值移位*/ 
		if(preStateSum!= currStateSum)
		{
			oldDataSum=  newDataSum;
		}
		/*记录负最大值*/
		if(adResSum < newDataSum)
		{
			newDataSum = adResSum;
		}
	}
		
	findStepFlagTmp= FALSE;
	if(((oldDataSum - newDataSum)> precisionSum) 
	    ||((newDataSum - oldDataSum)> precisionSum))
	{
	     if(Interval>TIMEWINDOW_MIN)
		 {
		 findStepFlagTmp = TRUE;
		 oldDataSum =  newDataSum;
		 }
	} 

    findStepFlag= FIND_IDLE;
	//---------------------- 动态门限判决 ----------------------------------
	if(( findStepFlagTmp == TRUE)&&(badFlagSum != 1))
	{
	    findStepFlag = TimeWindow();
	}
	else if(Interval >= TIMEWINDOW_MAX)
	{
	     /*如果超过一定时间间隔仍然没找到下一步，则重新进入计步初始态*/
	    InvalidSteps=0;	
		ReReg=1;						
		TempSteps=0;
		Interval=0;
	}
	fprintf(g_FileRes, "Cnt:%4d,interval:%4d, STEPS:%3d\\
	        findTmp:%d,findFlag:%d,\\
	        vppSum:%5d,oldDataSum:%5d,newDataSum:%5d,precisionSum:%5d, adResSum:%7d,\\
			chanSelFlag:%d,%d,%d,\\
	        badFlagSum:%d,ReReg:%d,TempSteps:%2d\\
	        currStateSum:%d,preStateSum:%d,\\
		    old:%5d,%5d,%5d,new:%5d,%5d,%5d,\\
		   highThSum:%6d,lowThSum:%6d, maxSum:%6d, minSum:%6d, dcSum:%6d,\\
	       highTh:%6d,%6d,%6d,lowTh:%6d,%6d,%6d,\\
		    bad:%5d,%5d,%5d,_vpp:%6d,%6d,%6d,\\
			ad:%5d,%5d,%5d,\\
            dc:%5d,%5d,%5d,max:%5d,%5d,%5d,min:%5d,%5d,%5d,\\
            precision:%5d,%5d,%5d,\\
            InvalidSteps:%4d\n",
	      g_DebugCnt, Interval,STEPS,
		findStepFlagTmp,findStepFlag,  
		vppSum,	oldDataSum, newDataSum, precisionSum, adResSum,
		chanSelFlag[X_CHANNEL],chanSelFlag[Y_CHANNEL],chanSelFlag[Z_CHANNEL],	
		badFlagSum, ReReg, TempSteps,
		currStateSum,preStateSum,
	      _old_fixed[X_CHANNEL],_old_fixed[Y_CHANNEL],_old_fixed[Z_CHANNEL],
	      _new_fixed[X_CHANNEL],_new_fixed[Y_CHANNEL],_new_fixed[Z_CHANNEL],
		highThSum, lowThSum,maxSum, minSum, dcSum,  
	       highTh[X_CHANNEL], highTh[Y_CHANNEL],highTh[Z_CHANNEL],
	        lowTh[X_CHANNEL], lowTh[Y_CHANNEL],lowTh[Z_CHANNEL],
	      _bad_flag[X_CHANNEL],_bad_flag[Y_CHANNEL],_bad_flag[Z_CHANNEL],
	      _vpp[X_CHANNEL],_vpp[Y_CHANNEL],_vpp[Z_CHANNEL],
		_adresult[X_CHANNEL],_adresult[Y_CHANNEL],_adresult[Z_CHANNEL],
	      _dc[X_CHANNEL], _dc[Y_CHANNEL],_dc[Z_CHANNEL],
	      _max[X_CHANNEL], _max[Y_CHANNEL],_max[Z_CHANNEL],
	      _min[X_CHANNEL], _min[Y_CHANNEL],_min[Z_CHANNEL],
	      _precision[X_CHANNEL], _precision[Y_CHANNEL],_precision[Z_CHANNEL],
	      InvalidSteps);
	
}

//计步初始化
void step_init()
{
	sampling_counter=0;
	STEPS=0;	
	memset(_new_fixed, 0, sizeof(_new_fixed));
	memset(highTh, 0, sizeof(highTh));
	memset(lowTh, 0, sizeof(lowTh));	
	memset(_bad_flag,1,sizeof(_bad_flag));
	memset(chanSelFlag,FALSE,sizeof(chanSelFlag));
	memset(_max,ADC_MIM_VALUE,sizeof(_max));
	memset(_min,ADC_MAX_VALUE,sizeof(_min));

	   

	preStateSum = STATE_IDLE;
	currStateSum = STATE_IDLE;
	oldDataSum = 0;
	newDataSum = 0;
	precisionSum = 4095;
}

/*********************************************************************/
/*********************************************************************/

//gsensor初始化
void gsensor_init()
{
	MMA8452Q_Init(1);
	
	step_init();
}

//主机读取结果
void gsensor_read()
{
	
}

//GSensor定期更新处理
void gsensor_update()
{
	mma8452q_read();


	step_counter();
	
}


