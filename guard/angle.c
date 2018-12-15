#include "angle.h"
#include "math.h"
#define M_E 2.718281828f


uint16_t test_counter = 0;


/**
  * @note   Optimize
  * @brief  ���е��绬������������Ż���ԽǶȼ��㣬
  * @brief  ������ﵽĿ��Ƕȵ���Сƫת�Ƕ�
  * @param  float angle_now:�˿̽Ƕ�
  * @param  float angle_ref:Ŀ��Ƕ�
  * @retval float ������ĽǶȣ���Χ��-180.0 ~ +180.0
  * @note   
  */
float Angle_Optimize(float angle_now, float angle_ref)
{
  float temp_now, temp_ref;
  temp_now = angle_now - (int)(angle_now / 360) * 360;
  temp_ref = angle_ref - (int)(angle_ref / 360) * 360;
  if (temp_ref - temp_now > 180)
    return (temp_ref - temp_now - 360.0f);
  else if (temp_ref - temp_now < -180)
    return (temp_ref - temp_now + 360.0f);
  else
    return (temp_ref - temp_now);
}
/**
  * @note   modified
  * @brief  SIGMOIDƽ������  y = 1/(1+z^(-t))
  * @param  Angle_Ref Ŀ�����
  * @param  Angle_Last ��ʼ����
  * @param  ƽ�����̳���(����)
  * @retval ƽ����ĽǶ�
  */
float SmoothSigmoid(float Angle_Ref, int16_t smooth_time, Smooth_t *obj)
{
					test_counter++;
	if (obj->SmoothMode == SMOOTH_SINGLE)
	{
		if (obj->Updata_Flag == 1) //����ƽ����ʼ����λ
		{
			obj->Angle_Last = obj->Angle_Mark;
			obj->Angle_Mark = Angle_Ref;
			obj->t = -5;
			obj->Updata_Flag = 0;
		}
	}
	else
	{
		if (Angle_Ref != obj->Angle_Mark) //�Ƕȷ����仯 ����ƽ��
		{
			obj->Angle_Last = obj->Angle_Mark;
			obj->Angle_Mark = Angle_Ref;
			obj->t = -5;
		}
	}
	float output = obj->Angle_Last + (Angle_Ref - obj->Angle_Last) * (1.0f / (1 + pow(M_E, -(obj->t))));
	if (obj->t > 5)
		output = Angle_Ref;
	else
		obj->t += (10.0f / smooth_time);
	return output;
}
/**
  * @brief  �ǶȻ��㣬�����-180~180
  * @param  float angle:������Ƕ�
  * @retval  float angle:�����Ƕ�
  * @note   
  */
float Angle_Process(float angle)
{
	angle=angle-360*(int16_t)(angle/360);
    if(angle>180)
    	angle-=360;
	return angle;
}
