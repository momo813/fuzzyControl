#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define MAX(x, y) (((x) > (y)) ? (x):(y))
#define MIN(x, y) (((x) < (y)) ? (x):(y))
/* Output P controller membership function initialization */
float p_ze[] = {1, 0.75, 0.5, 0.25, 0, 0, 0, 0, 0, 0};
float p_s[] = {0, 0, 0.25, 0.5, 0.75, 1, 0.75, 0.5, 0.25, 0};
float p_pm[] = {0, 0, 0, 0, 0, 0, 0, 0.25, 0.5, 0.75, 1};
float p_output;

/* Output I controller membership function initialization */
float i_ze[] = {1, 0.75, 0.5, 0.25, 0, 0, 0, 0, 0, 0};
float i_s[] = {0, 0, 0.25, 0.5, 0.75, 1, 0.75, 0.5, 0.25, 0};
float i_pm[] = {0, 0, 0, 0, 0, 0, 0, 0.25, 0.5, 0.75, 1};
float i_output;

/*fuzzy-p control output function*/
void CNTL_OUT(float error)
{
	int i;
	float area_p, area_i, moment_p, moment_i;
	float map_p, map_i, v_p, v_i;
	float u_er;
	float *u_p, *u_i;
	//memory alloc
	u_p = (float *) calloc(11, sizeof(float));
	u_i = (float *) calloc(11, sizeof(float));
	
/****       Rule one      ****/
/*      If input is ZE       */
	u_er = 0;
	if ((error>=0) & (error<=.4))
	u_er = (float)(0.4-error)/0.4;
	printf("rule1: %f\n",u_er);

/*      kp output is PM     */
	if (u_er!=0){
	  for(i=0; i<11; i++)
	  u_p[i] = MAX(u_p[i], MIN(u_er,p_pm[i]));
	}	
	
/*      ki output is ZE*/
	if (u_er!=0){
	  for(i=0;i<11;i++)
	  u_i[i] = MAX(u_i[i], MIN(u_er, i_ze[i]));
	}

/****        Rule two    ******/
/*      If input is S       */
	u_er = 0;
	if ((error>=0.1) & (error<=0.5))
	u_er = (float)(error-0.1)/0.4;

	if ((error>0.5) & (error<=0.9))
	u_er = (float)(0.9-error)/0.4;

	printf("rule2 : %f\n", u_er);

/*      kp output is PM    */
	if (u_er!=0){
	  for(i=0;i<11;i++)
	  u_p[i] = MAX(u_p[i], MIN(u_er, p_pm[i]));
	}
	
/*      ki output is ZE    */
	if (u_er!=0){
	  for (i=0;i<11;i++)
	  u_i[i] = MAX(u_i[i], MIN(u_er, i_ze[i]));
	}

/****      Rule 3    *****/
/*      If input is PM    */
	u_er = 0.0;
	if ((error>=0.6) & (error<=1.0))
	u_er = (float)(1.0-error)/0.4;

	printf("rule3: %f\n", u_er);

/*      then kp output is PM  */
	if (u_er!=0){
	  for(i=0;i<11;i++)
	  u_p[i] = MAX(u_p[i], MIN(u_er, p_pm[i]));	
	}

/*      ki output is PM   */
	if (u_er!=0){
	  for(i=0;i<11; i++)
	  u_i[i] = MAX(u_i[i], MIN(u_er, i_pm[i]));
	}

/****     defuzzification      ****/
	/* initial the parameters */
	area_p = 0.0;
	moment_p= 0.0;
	area_i = 0.0;
	moment_i = 0.0;
	v_p = 0.0;
	v_i = 0.0;
	/* init end */
	for (i=0;i<11;i++){
	  map_p = u_p[i];
	  map_i = u_i[i];
	  printf("i=%d, map_p = %f, map_i=%f\n", i, map_p, map_i);
	  area_p += map_p;
	  area_i += map_i;
	  moment_p += map_p * v_p;
	  moment_i += map_i * v_i;
	  v_p += 0.2;
	  v_i += 0.01;	
	}
	printf("\narea_p: %f,  moment_p: %f\n",area_p, moment_p);
	printf("\narea_i: %f,  moment_i: %f\n",area_i, moment_i);

	/* avoid den is ZERO */
	if (area_p == 0){
	  p_output = 2;
	}
	else {
	  p_output = moment_p/area_p;
	}
	
	if (area_i == 0){
	  i_output = 0.1;
	}
	else {
	  i_output = moment_i/area_i;
	}
	free(u_p);
	free(u_i);
}

int main(void){
	float error;
	error = 0.7;
	CNTL_OUT(error);
	printf("\nerror: %f,  p_output: %f,  i_output: %f\n",
	error, p_output, i_output);	
	return 0;
}

