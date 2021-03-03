/************************************************************************
* 文件名：Leg_Parse.h
* 功  能：实现地面站上传航段及解析后航段结构体定义、航段解析函数定义
* 作  者：lwh
* 日  期：2016.11.23
*************************************************************************/
#ifndef _LEG_PARSE_H
#define _LEG_PARSE_H

#include "JacTypeDef.h"

#define LINE 0            //代表直线航段
#define ARC  1            //代表圆弧航段

typedef struct            //解析前各航点信息
{
	double *Lat;
	double *Lon;
	//double *H;
	UINT16 Num_to_Parse;
}Original_Leg_Def;

typedef struct             //解析后的单个小航段
{
	double Lat_Start;
	double Lon_Start;
	//double H_Start;
	double Lat_End;
	double Lon_End;
	//double H_End;
	UINT8 Type_Leg;       //小航段类型：0为直线航段，1为圆弧航段
	double Lat_Center;
	double Lon_Center;
	double R;
	UINT8 Direct;         //转弯方向：0为圆弧航段圆心位于第一航段左侧，1为圆弧航段位于第一航段右侧,仅针对圆弧航段
	double chi;
	double Del_Chi;      //圆弧航段转过圆心角
	UINT16 Nav_Index;       //表示该小航段位于哪个大航段
	//UINT8  End_Task;
}Parsed_Single_Leg_Def;

typedef struct               //解析后的小航段集合
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
* 函数名：Leg_Parse
* 功  能：对地面站上传所有航路信息进行解析，即将转弯处处理成圆弧
* 返回值：Parsed_Leg_Def类型结构体，其中存放解析后的航段信息及航段数
* 参  数：Leg_Orig：Original_Leg_Def类型结构体，其中存放地面站上传航路点经纬度及航路点个数
		  R：       设定转弯半径，当两条相邻航路间的夹角在正负1度或179到181度之间时这个转弯
		  半径不成立，具体处理过程位于Center_Analysis函数中
************************************************************************************************/
Parsed_Leg_Def Leg_Parse(Original_Leg_Def Leg_Orig, double R);

/***********************************************************************************************
* 函数名：Center_Analysis
* 功  能：得到两条相邻航段间的圆弧航段信息
* 返回值：Center_Struct类型中包含解析后的圆弧航段信息
* 参  数：lat0,lon0 两相邻航段中的第一个航点经纬度
          lat1,lon1 两相邻航段中的第二个航点经纬度
		  lat2,lon2 两相邻航段中的第三个航点经纬度
		  Bearing0,len0 两相邻航段中第一个航段的指向和长度
		  Bearing1,len1 两相邻航段中第二个航段的指向和长度
		  R 期望转弯半径
************************************************************************************************/
Center_Struct Center_Analysis(double lat0, double lon0, double lat1, double lon1, double lat2, double lon2, double Bearing0, double Bearing1, double len0, double len1, double R);

/***********************************************************************************************
* 函数名：Get_Phi_Ref
* 功  能：得到对应航段的phi_ref
* 返回值：phi_ref
* 参  数：Parsed_Leg：解析后的航段信息结构体
          Index_Leg:  当前小航段索引号
		  Lat_Now:    飞机当前位置纬度
		  Lon_Now:    飞机当前位置经度
		  Chi_FeedBack:飞机当前航向角
************************************************************************************************/
double Get_Phi_Ref(Parsed_Leg_Def Parsed_Leg, UINT16 Index_Leg, double Lat_Now, double Lon_Now, double Chi_FeedBack, double V_Feedback);

#endif