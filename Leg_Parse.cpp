#include "Leg_Parse.h"
#include "LaLonH.h"
#include "Saturation.h"

extern TELE_PID_PARA_DEF gtele_pid_para;
extern double UserData_Record[10];

Parsed_Leg_Def Leg_Parse(Original_Leg_Def Leg_Orig, double R)
{
	UINT16 Index_Waypoint, Index_Leg = 0;
	data_fj data_start,data_end;
	Center_Struct center;
	Parsed_Leg_Def Parsed_Leg_Ret;
	Parsed_Single_Leg_Def Parsed_Single_Leg[50];
	double D_Z_Center, D_Start_End;

	for(Index_Waypoint = 0; Index_Waypoint < Leg_Orig.Num_to_Parse - 2; Index_Waypoint++)
	{
		data_start = rhumbline_fj(Leg_Orig.Lat[Index_Waypoint],   Leg_Orig.Lon[Index_Waypoint], 
			                      Leg_Orig.Lat[Index_Waypoint+1], Leg_Orig.Lon[Index_Waypoint+1]);

		data_end   = rhumbline_fj(Leg_Orig.Lat[Index_Waypoint+1], Leg_Orig.Lon[Index_Waypoint+1], 
			                      Leg_Orig.Lat[Index_Waypoint+2], Leg_Orig.Lon[Index_Waypoint+2]);
		
		/* 解析后的直线航段 */
		if(Index_Leg == 0)
		{
			Parsed_Single_Leg[Index_Leg].Lat_Start  = Leg_Orig.Lat[Index_Waypoint];
			Parsed_Single_Leg[Index_Leg].Lon_Start  = Leg_Orig.Lon[Index_Waypoint];
		}
		else
		{
			Parsed_Single_Leg[Index_Leg].Lat_Start  = center.Lat_End;
			Parsed_Single_Leg[Index_Leg].Lon_Start  = center.Lon_End;
		}
		
		center  = Center_Analysis(Leg_Orig.Lat[Index_Waypoint],   Leg_Orig.Lon[Index_Waypoint], 
		                          Leg_Orig.Lat[Index_Waypoint+1], Leg_Orig.Lon[Index_Waypoint+1],
			                      Leg_Orig.Lat[Index_Waypoint+2], Leg_Orig.Lon[Index_Waypoint+2],
			                      data_start.chi,                 data_end.chi,
			                      data_start.length,              data_end.length,         R);

		Parsed_Single_Leg[Index_Leg].Lat_End    = center.Lat_Start;
		Parsed_Single_Leg[Index_Leg].Lon_End    = center.Lon_Start;
		Parsed_Single_Leg[Index_Leg].Type_Leg   = LINE;
		Parsed_Single_Leg[Index_Leg].Lat_Center = 0;
		Parsed_Single_Leg[Index_Leg].Lon_Center = 0;
		Parsed_Single_Leg[Index_Leg].R          = 0;
		Parsed_Single_Leg[Index_Leg].chi        = data_start.chi;
		Parsed_Single_Leg[Index_Leg].Del_Chi    = 0;
		Parsed_Single_Leg[Index_Leg].Direct     = 0;
		Parsed_Single_Leg[Index_Leg].Nav_Index  = Index_Waypoint;
		Index_Leg = Index_Leg + 1;

		/* 解析后的圆弧航段 */
		if(center.R > 0.5)
		{
			Parsed_Single_Leg[Index_Leg].Lat_Start  = center.Lat_Start;
			Parsed_Single_Leg[Index_Leg].Lon_Start  = center.Lon_Start;
			Parsed_Single_Leg[Index_Leg].Lat_End    = center.Lat_End;
			Parsed_Single_Leg[Index_Leg].Lon_End    = center.Lon_End;
			Parsed_Single_Leg[Index_Leg].Type_Leg   = ARC;
			Parsed_Single_Leg[Index_Leg].Lat_Center = center.Lat_Center;
			Parsed_Single_Leg[Index_Leg].Lon_Center = center.Lon_Center;
			Parsed_Single_Leg[Index_Leg].R          = center.R;
			Parsed_Single_Leg[Index_Leg].chi        = 0;
			D_Start_End = rhumbline_fj(center.Lat_Start, center.Lon_Start, center.Lat_End, center.Lon_End).length;
			Parsed_Single_Leg[Index_Leg].Del_Chi = acos((2 * pow(center.R,2) - pow(D_Start_End,2))/(2 * pow(center.R,2)));
			D_Z_Center = rhumbline_XTE(Leg_Orig.Lat[Index_Waypoint], Leg_Orig.Lon[Index_Waypoint], center.Lat_Center,center.Lon_Center,data_start.chi);
			if(D_Z_Center >= 0)     //圆心在左
			{
				Parsed_Single_Leg[Index_Leg].Direct = 0;
			}
			else
			{
				Parsed_Single_Leg[Index_Leg].Direct = 1;
			}
			Parsed_Single_Leg[Index_Leg].Nav_Index   = Index_Waypoint;
			Index_Leg = Index_Leg + 1;
		}

		///* 两段航线间夹角不满足圆弧转弯条件，此时无圆弧过渡 */
		//if(center.R <= 0.5)
		//{
		//	continue;
		//}
		if(Index_Waypoint + 3 == Leg_Orig.Num_to_Parse)
		{
			Parsed_Single_Leg[Index_Leg].Lat_Start  = center.Lat_End;
			Parsed_Single_Leg[Index_Leg].Lon_Start  = center.Lon_End;
			Parsed_Single_Leg[Index_Leg].Lat_End    = Leg_Orig.Lat[Leg_Orig.Num_to_Parse - 1];
			Parsed_Single_Leg[Index_Leg].Lon_End    = Leg_Orig.Lon[Leg_Orig.Num_to_Parse - 1];
			Parsed_Single_Leg[Index_Leg].Type_Leg   = LINE;
			Parsed_Single_Leg[Index_Leg].Lat_Center = 0;
			Parsed_Single_Leg[Index_Leg].Lon_Center = 0;
			Parsed_Single_Leg[Index_Leg].R          = 0;
			Parsed_Single_Leg[Index_Leg].chi        = data_end.chi;
			Parsed_Single_Leg[Index_Leg].Del_Chi    = 0;
			Parsed_Single_Leg[Index_Leg].Direct     = 0;
			Parsed_Single_Leg[Index_Leg].Nav_Index  = Index_Waypoint;
			Index_Leg = Index_Leg + 1;
		}
	}

	Parsed_Leg_Ret.Parsed_Leg = Parsed_Single_Leg;
	Parsed_Leg_Ret.Num_Parsed = Index_Leg;
	return(Parsed_Leg_Ret);
}

Center_Struct Center_Analysis(double lat0, double lon0, double lat1, double lon1, double lat2, double lon2, double Bearing0, double Bearing1, double len0, double len1, double R)
{
	data_zj point2, point3, point4;
	data_fj line1;
	Center_Struct center1;
	double Delta_Phi, Angle_1, C2, Flag_angle;

	if((fabs(Bearing1-Bearing0) - floor(fabs(Bearing1-Bearing0)/180)*180) > 30
		&& (fabs(Bearing1-Bearing0) - floor(fabs(Bearing1-Bearing0)/180)*180) < 150)   //排除正负1度和179度到181度，((fabs(Bearing1-Bearing0)-floor(fabs(Bearing1-Bearing0)/180)*180)的作用是将角度转化到0到180
	{
		Flag_angle = Bearing1 - Bearing0;

		if(Flag_angle>180)
			Delta_Phi = Bearing1 - Bearing0 - 180;
		else if(Flag_angle<-180)
			Delta_Phi = -(Bearing1 - Bearing0) - 180;
		else
			Delta_Phi = -fabs(Bearing1 - Bearing0) + 180;

		if ((Flag_angle>0 && Flag_angle<180) || Flag_angle<-180)
			Angle_1 = Bearing0 + 90;
		else 
			Angle_1 = Bearing0 + 270;

		if(Angle_1>360)
			Angle_1 = Angle_1 - 360;

		C2 = R/tan(fabs(Delta_Phi/2/180*3.14159265358979));
		point2 = rhumbline_zj(lat0, lon0, fabs(len0) - C2, Bearing0);
		point3 = rhumbline_zj(lat1, lon1, C2,        Bearing1);
		point4 = rhumbline_zj(point2.Lat, point2.Lon, R, Angle_1);
		center1.Lat_Center = point4.Lat;
		center1.Lon_Center = point4.Lon;
		center1.Lat_Start  = point2.Lat;
		center1.Lon_Start  = point2.Lon;
		center1.Lat_End    = point3.Lat;
		center1.Lon_End    = point3.Lon;
		center1.R          = R;
	}
	else
	{
		line1  = rhumbline_fj(lat0, lon0, lat1, lon1);
		point2 = rhumbline_zj(lat0, lon0, line1.length - 50, line1.chi);
		line1  = rhumbline_fj(lat1, lon1, lat2, lon2);
		point3 = rhumbline_zj(lat1, lon1, 50, line1.chi);
		center1.Lat_Center = 0;
		center1.Lon_Center = 0;
		center1.Lat_Start  = point2.Lat;
		center1.Lon_Start  = point2.Lon;
		center1.Lat_End    = point3.Lat;
		center1.Lon_End    = point3.Lon;
		center1.R          = 0;
	}

	return center1;
}

double Get_Phi_Ref(Parsed_Leg_Def Parsed_Leg, UINT16 Index_Leg, double Lat_Now, double Lon_Now, double Chi_FeedBack, double V_Feedback)
{
	double phi_ref;
	double E_Z,E_Chi,Chi_Ref,Chi_States,phi_g;
	data_fj data;

	switch(Parsed_Leg.Parsed_Leg[Index_Leg].Type_Leg)
	{
	case LINE:
		E_Z = rhumbline_XTE(Parsed_Leg.Parsed_Leg[Index_Leg].Lat_Start,Parsed_Leg.Parsed_Leg[Index_Leg].Lon_Start, Lat_Now, Lon_Now,Parsed_Leg.Parsed_Leg[Index_Leg].chi);
		UserData_Record[0] = E_Z;
		Chi_Ref = Zyw_2PiTOPi(Parsed_Leg.Parsed_Leg[Index_Leg].chi * (3.1415926535897 / 180));
		Chi_States = Zyw_2PiTOPi(Chi_FeedBack);
		E_Chi	   = Chi_Ref - Chi_States;
		//保证 Chi 在 -pi - pi 之间
		if( -180 * d2r < E_Chi && E_Chi< 180 * d2r)				
			E_Chi = E_Chi;
		if(E_Chi <= -180 * d2r)
			E_Chi = ( -360 * d2r - E_Chi ) * -1;
		if(E_Chi >= 180 * d2r)
			E_Chi = ( 360 * d2r - E_Chi ) * -1;

		//侧偏距限幅
		E_Z = Zyw_Saturation_Position( E_Z , 80 , -80 );							//侧偏距限幅  

		//if (fabs(E_Z) <= 2)
		//{
		//	Enable_Ki_Func(ID_EZ);
		//	Pid_Ek(ID_EZ, 0, Zyw_Saturation_Position( E_Z , 2 , -2 ));
		//	Get_Pid(ID_EZ);
		//	//PID.PID_Model[ID_EZ].Out = 0;
		//}
		//if (fabs(E_Z) > 2)
		//{
		//	PID.PID_Model[ID_EZ].Out = 0;
		//	Disable_Ki_Func(ID_EZ);
		//}

		

		//控制律计算
		//phi_ref = 0.4 * 2 * d2r * E_Z + 1.2 * 1.5 * (E_Chi);
		//phi_ref = PID.PID_Model[ID_EZ].Out + 0.96 * d2r * E_Z + 1.2 * E_Chi;
		//phi_ref =2*( PID.PID_Model[ID_EZ].Out + 0.2 * d2r * E_Z + 0.6 * E_Chi);
		phi_ref = PID.PID_Model[ID_EZ].Out + gtele_pid_para.kp_EZ * d2r * E_Z + gtele_pid_para.kp_Chi * E_Chi;
		phi_ref = Zyw_Saturation_Position(phi_ref , 35 * d2r , -35 * d2r );		//滚转角给指令限幅
		break;
	case ARC:
		data = rhumbline_fj(Parsed_Leg.Parsed_Leg[Index_Leg].Lat_Center,Parsed_Leg.Parsed_Leg[Index_Leg].Lon_Center,Lat_Now,Lon_Now);
		E_Z  = Parsed_Leg.Parsed_Leg[Index_Leg].R - fabs(data.length);
		UserData_Record[0] = E_Z;
		E_Z = Zyw_Saturation_Position( E_Z , 80 , -80 );
		switch(Parsed_Leg.Parsed_Leg[Index_Leg].Direct)
		{
		case 0:
			phi_g = -atan(V_Feedback*V_Feedback/9.8/Parsed_Leg.Parsed_Leg[Index_Leg].R);//-atan(V_Feedback*V_Feedback/9.8/(Parsed_Leg.Parsed_Leg[Index_Leg].R));
			//phi_ref = 1.0 * phi_g + 0.96 * d2r * E_Z;
			phi_ref = 1.0 * phi_g + gtele_pid_para.kp_EZ * d2r * E_Z;
			phi_ref = Zyw_Saturation_Position(phi_ref , 35 * d2r , -35 * d2r );			//滚转角给指令限幅
			break;
		case 1:
			phi_g = atan(V_Feedback*V_Feedback/9.8/Parsed_Leg.Parsed_Leg[Index_Leg].R);//atan(V_Feedback*V_Feedback/9.8/(Parsed_Leg.Parsed_Leg[Index_Leg].R));
			//phi_ref = 1.0 * phi_g - 0.96 * d2r * E_Z;
			phi_ref = 1.0 * phi_g - gtele_pid_para.kp_EZ * d2r * E_Z;
			phi_ref = Zyw_Saturation_Position(phi_ref , 35 * d2r , -35 * d2r );			//滚转角给指令限幅
			break;
		default:
			phi_ref = 0;
			break;
		}
		break;
	default:
		phi_ref = 0;
		break;
	}
	return(phi_ref);
}