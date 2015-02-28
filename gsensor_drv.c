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



int16 g_acc[MMA8452Q_AXES_NUM]; 	//�˶����������������

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


//ADC����ͨ��



uint32 STEPS;	//�ܲ���

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


uint16 Interval=0;		//��¼ʱ������
uint8 TempSteps=0;		//�ǲ�����
uint8 InvalidSteps=0;	//��Ч������
uint8 ReReg=2;			//��¼�Ƿ����¿�ʼѰ�ҹ���
						//	2-�¿�ʼ
						//	1-�Ѿ���ʼ�����ǻ�û���ҵ�����
						//	0-�Ѿ��ҵ�����
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
*Function:	ʵ��"ʱ�䴰"�㷨,��Ϊֻ������Ч"ʱ�䴰"�ڵļǲ�����Ч,������ʼʱ��Ҫ����������Ч������Ϊ��ʼ
*Input:		void
*Output: 	void
*------------------------------------------------------------------------------------------------------------------------*/
uint8 TimeWindow()
{
    uint8 ret = FIND_IDLE;
    
	if(ReReg==2)		//������¿�ʼ�ĵ�һ����ֱ���ڼǲ������м�1
	{
		TempSteps++;
		Interval=0;
		ReReg=1;
		InvalidSteps=0;	
		ret = FIND_NOT_REACH_REGULATION;
	}
	else				//��������¿�ʼ�ĵ�һ��
	{
		if((Interval>=TIMEWINDOW_MIN)&&(Interval<=TIMEWINDOW_MAX))	//���ʱ��������Ч��ʱ�䴰��
		{
			InvalidSteps=0;	
			if(ReReg==1)					//�����û���ҵ�����
			{
				TempSteps++;				//�ǲ������1
				if(TempSteps>=REGULATION)	//����ǲ�����ﵽ��Ҫ��Ĺ�����
				{
					ReReg=0;				//�Ѿ��ҵ�����
					STEPS=STEPS+TempSteps;	//������ʾ
					TempSteps=0;
					ret = FIND_OK;
				}
				else
				{
				    ret = FIND_NOT_REACH_REGULATION;
                }
				Interval=0;
			}
			else if(ReReg==0)				//����Ѿ��ҵ����ɣ�ֱ�Ӹ�����ʾ
			{
				STEPS++;
				TempSteps=0;
				Interval=0;
				ret = FIND_OK;
			}
		}
		else if(Interval<TIMEWINDOW_MIN)	//���ʱ����С��ʱ�䴰����
		{	
			if(ReReg==0)					//����Ѿ��ҵ�����
			{
				if(InvalidSteps<255) 	InvalidSteps++;	//��Ч�������1
				if(InvalidSteps>=INVALID)				//�����Ч���ﵽ��Ҫ�����ֵ��������Ѱ�ҹ���
				{	
					InvalidSteps=0;
					ReReg=1;
					TempSteps=0;
					Interval=0;
				}
				else					    //����ֻ������һ�εļǲ������Ǽ����ǲ�������Ҫ����Ѱ�ҹ���
				{
					Interval=0;
				}
			}
			else if(ReReg==1)				//�����û���ҵ����ɣ���֮ǰ��Ѱ�ҹ��ɹ�����Ч������Ѱ�ҹ���
			{
				InvalidSteps=0;	
				ReReg=1;
				TempSteps=0;
				Interval=0;
			}

			ret = WIN_TOO_SMALL;
		}
		else if(Interval>TIMEWINDOW_MAX)	//���ʱ��������ʱ�䴰���ޣ��ǲ��Ѿ���ϣ�����Ѱ�ҹ���
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
		STEPS=STEPS+TempSteps;	//������ʾ
		TempSteps = 0;
	}
	else if(ReReg==0)				//����Ѿ��ҵ����ɣ�ֱ�Ӹ�����ʾ
    {
    	STEPS++;
    }
    
	return;
}
/*------------------------------------------------------------------------------------------------------------------------
*Name: 		step_counter()
*Function:	ʵ��Pedometer�Ʋ��Ļ����㷨.
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
	
	//----------------------------------------------ADC����----------------------//
	for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
	{
		_adresult[X_CHANNEL] = g_acc[X_CHANNEL];//;0;//g_acc[X_CHANNEL];
		_adresult[Y_CHANNEL] = g_acc[Y_CHANNEL];
		_adresult[Z_CHANNEL] = g_acc[Z_CHANNEL];
		
		if (_adresult[jtemp]>_max[jtemp])    {_max[jtemp]=_adresult[jtemp];}
		if (_adresult[jtemp]<_min[jtemp])    {_min[jtemp]=_adresult[jtemp];}
	}
	

    sampling_counter=sampling_counter+1;
	
	//----------------------------------���㶯̬���޺Ͷ�̬����-----------------------//
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

          //�������޲���
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
         /*����ȷ��3��ͨ����vpp��С*/
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
        /*Vpp ����channelһ����ѡ��*/
        idx1 = sortChanId[0];
        idx2 = sortChanId[1];
        chanSelFlag[idx1] = TRUE;
        
        /*���vpp�δ��channel������channel�ӽ�����Ҳѡ��*/
        if(_vpp[idx2] > (_vpp[idx1] * 5 >> 3)  )
        {
            chanSelFlag[idx2] = TRUE;
            chanSelNum = 2;
        }
 #endif
        
        //�򻯴��������������ʽ���̶�ѡ��Yͨ��
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

        /*�����ںϺ�����޲���*/
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
          /*�ںϺ����������ʱ���ݾ������ݣ�������Ż�*/
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
	//--------------------------������λ�Ĵ���--------------------------------------
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
		/*��һ��״̬�л�������һ��״̬����Ĵ���ֵ��λ*/ 
		if(preStateSum!= currStateSum)
		{
			oldDataSum=  newDataSum;
		}
		/*��¼�����ֵ*/
		if(adResSum > newDataSum)
		{
			 newDataSum = adResSum;
		}
	}
	else if(adResSum < lowThSum)
	{
	    preStateSum = currStateSum;
		currStateSum = STATE_LOW;
		/*��һ��״̬�л�������һ��״̬����Ĵ���ֵ��λ*/ 
		if(preStateSum!= currStateSum)
		{
			oldDataSum=  newDataSum;
		}
		/*��¼�����ֵ*/
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
	//---------------------- ��̬�����о� ----------------------------------
	if(( findStepFlagTmp == TRUE)&&(badFlagSum != 1))
	{
	    findStepFlag = TimeWindow();
	}
	else if(Interval >= TIMEWINDOW_MAX)
	{
	     /*�������һ��ʱ������Ȼû�ҵ���һ���������½���Ʋ���ʼ̬*/
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

//�Ʋ���ʼ��
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

//gsensor��ʼ��
void gsensor_init()
{
	MMA8452Q_Init(1);
	
	step_init();
}

//������ȡ���
void gsensor_read()
{
	
}

//GSensor���ڸ��´���
void gsensor_update()
{
	mma8452q_read();


	step_counter();
	
}


