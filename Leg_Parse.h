/************************************************************************
* �ļ�����Leg_Parse.h
* ��  �ܣ�ʵ�ֵ���վ�ϴ����μ������󺽶νṹ�嶨�塢���ν�����������
* ��  �ߣ�lwh
* ��  �ڣ�2016.11.23
*************************************************************************/
#ifndef _LEG_PARSE_H
#define _LEG_PARSE_H

#include "JacTypeDef.h"

#define LINE 0            //����ֱ�ߺ���
#define ARC  1            //����Բ������

typedef struct            //����ǰ��������Ϣ
{
	double *Lat;
	double *Lon;
	//double *H;
	UINT16 Num_to_Parse;
}Original_Leg_Def;

typedef struct             //������ĵ���С����
{
	double Lat_Start;
	double Lon_Start;
	//double H_Start;
	double Lat_End;
	double Lon_End;
	//double H_End;
	UINT8 Type_Leg;       //С�������ͣ�0Ϊֱ�ߺ��Σ�1ΪԲ������
	double Lat_Center;
	double Lon_Center;
	double R;
	UINT8 Direct;         //ת�䷽��0ΪԲ������Բ��λ�ڵ�һ������࣬1ΪԲ������λ�ڵ�һ�����Ҳ�,�����Բ������
	double chi;
	double Del_Chi;      //Բ������ת��Բ�Ľ�
	UINT16 Nav_Index;       //��ʾ��С����λ���ĸ��󺽶�
	//UINT8  End_Task;
}Parsed_Single_Leg_Def;

typedef struct               //�������С���μ���
{
	Parsed_Single_Leg_Def *Parsed_Leg;
	UINT16 Num_Parsed;
}Parsed_Leg_Def;

typedef struct
{
	double Lat_Center;
	double Lon_Center;
	double Lat_Start;
	double Lon_Start;
	double Lat_End;
	double Lon_End;
	double R;
}Center_Struct;

/***********************************************************************************************
* ��������Leg_Parse
* ��  �ܣ��Ե���վ�ϴ����к�·��Ϣ���н���������ת�䴦�����Բ��
* ����ֵ��Parsed_Leg_Def���ͽṹ�壬���д�Ž�����ĺ�����Ϣ��������
* ��  ����Leg_Orig��Original_Leg_Def���ͽṹ�壬���д�ŵ���վ�ϴ���·�㾭γ�ȼ���·�����
		  R��       �趨ת��뾶�����������ں�·��ļн�������1�Ȼ�179��181��֮��ʱ���ת��
		  �뾶�����������崦�����λ��Center_Analysis������
************************************************************************************************/
Parsed_Leg_Def Leg_Parse(Original_Leg_Def Leg_Orig, double R);

/***********************************************************************************************
* ��������Center_Analysis
* ��  �ܣ��õ��������ں��μ��Բ��������Ϣ
* ����ֵ��Center_Struct�����а����������Բ��������Ϣ
* ��  ����lat0,lon0 �����ں����еĵ�һ�����㾭γ��
          lat1,lon1 �����ں����еĵڶ������㾭γ��
		  lat2,lon2 �����ں����еĵ��������㾭γ��
		  Bearing0,len0 �����ں����е�һ�����ε�ָ��ͳ���
		  Bearing1,len1 �����ں����еڶ������ε�ָ��ͳ���
		  R ����ת��뾶
************************************************************************************************/
Center_Struct Center_Analysis(double lat0, double lon0, double lat1, double lon1, double lat2, double lon2, double Bearing0, double Bearing1, double len0, double len1, double R);

/***********************************************************************************************
* ��������Get_Phi_Ref
* ��  �ܣ��õ���Ӧ���ε�phi_ref
* ����ֵ��phi_ref
* ��  ����Parsed_Leg��������ĺ�����Ϣ�ṹ��
          Index_Leg:  ��ǰС����������
		  Lat_Now:    �ɻ���ǰλ��γ��
		  Lon_Now:    �ɻ���ǰλ�þ���
		  Chi_FeedBack:�ɻ���ǰ�����
************************************************************************************************/
double Get_Phi_Ref(Parsed_Leg_Def Parsed_Leg, UINT16 Index_Leg, double Lat_Now, double Lon_Now, double Chi_FeedBack, double V_Feedback);

#endif