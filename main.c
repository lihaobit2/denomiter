#include <stdio.h>
#include "Gsensor_drv.h"

#define MAX_IN_DATA_NUM 2048

int16 g_in[MMA8452Q_AXES_NUM][MAX_IN_DATA_NUM] = {0};
FILE *g_FileRes;

int main()
{
    uint32 i, j, ulDataNum;
    FILE *fp;

	fp = fopen("data.txt","r");
	if(fp == NULL)
	{
		printf("File open error!");
		
		return 1;
	}



	i = 0;
	while(!feof(fp))
	{
		fscanf(fp, "%d %d %d",&g_in[X_CHANNEL][i],&g_in[Y_CHANNEL][i], &g_in[Z_CHANNEL][i]);
		for(j = 1; j < 2; j++)
		{
			g_in[X_CHANNEL][i+j] = g_in[X_CHANNEL][i];
			g_in[Y_CHANNEL][i+j] = g_in[Y_CHANNEL][i];
			g_in[Z_CHANNEL][i+j] = g_in[Z_CHANNEL][i];
		}
		
		i += 2;
		
		if(i > MAX_IN_DATA_NUM - 8)
		{
		    printf("data size large than buf!\n");
			break;
		}

		
	}
    ulDataNum = i;
	fclose(fp);

    g_FileRes = fopen("result.txt","w");
	if(feof(fp))
	{
		return 0;
	}	
	
	gsensor_init();
	for(i = 0;i < ulDataNum; i ++)
	{
    	g_acc[X_CHANNEL] = g_in[X_CHANNEL][i];
		g_acc[Y_CHANNEL] = g_in[Y_CHANNEL][i];
	    g_acc[Z_CHANNEL] = g_in[Z_CHANNEL][i];


		step_counter();
	}

    fclose(g_FileRes);
	

	return 0;
}
