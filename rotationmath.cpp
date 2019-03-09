#include "rotationmath.h"
//��x��������˳ʱ��Ϊ��
#define TUKEY_C 4.6851
/********************************************************************
created:	2013/09/16
created:	18:5:2010   20:13
author:		��Ծǿ	
purpose:	��׼���Ƕ�ʹ����[head+0��head+360]֮��
*********************************************************************/
double standardizeAngle(double a,double head = 0)												// [head,head + 360)
{ 
	if( a >= head )	
		a -= int( (a-head)/360 ) * 360;
	else			
		a += int( -(a-head)/360 + 1 ) * 360;
	return a;
}

/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:	��׼���Ƕ�ʹ����[head+0��head+360]֮��
*********************************************************************/
 myAngle Get_StandardizeAngle(myAngle angle,double head /*= 0*/)
 {
 	angle.ax = standardizeAngle(angle.ax,head);
 	angle.ay = standardizeAngle(angle.ay,head);
 	angle.az = standardizeAngle(angle.az,head);
 	return angle;
 }

/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ		
purpose:	ǰ�����ǡ�[-180,180)�м�ڶ����ǡ�[-90,90]
*********************************************************************/
myAngle Get_StandardizeAngle(myAngle angle)
{
	// 	if( !VM_ISDEGREE)
	// 		angle = Mrad2deg(angle);

	angle.ay = standardizeAngle(angle.ay,-90);
	if(angle.ay > 90)
	{
		angle.ax += 180;
		angle.ay  = 180 - angle.ay;
		angle.az += 180;
	}
	angle.ax = standardizeAngle(angle.ax,-180);
	angle.az = standardizeAngle(angle.az,-180);

	return angle;
}

/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:	x�ǡ�[-180,180), ˳ʱ��Ϊ�����õ���ת����
*********************************************************************/
cv::Mat Get_Rx(double ax)
{
	//�Ƕ�ת��Ϊ����
	double aax = ax*CV_PI/180.0;

	return (Mat_<float>(3,3) <<  	
		1,	       0,	        0,  
		0,	cos(aax),	-sin(aax),
		0,	sin(aax),	 cos(aax) );
}

/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ		
purpose:	y�ǡ�[-90,90), ˳ʱ��Ϊ�����õ���ת����
*********************************************************************/
cv::Mat Get_Ry(double ay)
{
	//�Ƕ�ת��Ϊ����
	double aay = ay*CV_PI/180.0;

	return (Mat_<float>(3,3) <<  	
		cos(aay),	0,	sin(aay),
		0,    1,	      0,
		-sin(aay),	0,	cos(aay));
}

/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ			
purpose:	z�ǡ�[-180,180), ˳ʱ��Ϊ�����õ���ת����
*********************************************************************/
cv::Mat Get_Rz(double az)
{
	//�Ƕ�ת��Ϊ����
	double aaz = az*CV_PI/180.0;

	return (Mat_<float>(3,3) <<  
		cos(aaz),	-sin(aaz),	0,
		sin(aaz),	 cos(aaz),	0,
		0,		   0,   1);
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��ŷ���ǵ���ת����ang - ŷ���ǣ�order-��ת˳��
*********************************************************************/
cv::Mat Get_R(const double* ang , int order)
{
	double ax = ang[0];
	double ay = ang[1];
	double az = ang[2];
	cv::Mat R;
	switch (order)
	{
		case 1:      //xyz
			R = Get_Rz(az)*Get_Ry(ay)*Get_Rx(ax);
			break;
		case 2:      //yzx
			R = Get_Rx(ax)*Get_Rz(az)*Get_Ry(ay);
			break;
		case 3:      //zxy
			R = Get_Ry(ay)*Get_Rx(ax)*Get_Rz(az);
			break;
		case 4:      //xzy
			R = Get_Ry(ay)*Get_Rz(az)*Get_Rx(ax);
			break;
		case 5:      //yxz
			R = Get_Rz(az)*Get_Rx(ax)*Get_Ry(ay);
			break;
		case 6:      //zyx
			R = Get_Rx(ax)*Get_Ry(ay)*Get_Rz(az);
			break;
		default:
			break;
	}
	return R;
}

cv::Mat Get_R(myAngle ang , int order)
{
	double angle[3] = {ang.ax, ang.ay, ang.az};
	return Get_R(angle, order);
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ����ת����ŷ���ǣ�myAngle - ŷ���ǣ�order-��ת˳��
*********************************************************************/
myAngle Get_EularAngle(const cv::Mat& R, int order)
{
	myAngle ang;
	switch (order)
	{
	case 1:      //xyz
		ang=Get_Angle_xyz(R);
		break;
	case 2:      //yzx
		ang=Get_Angle_yzx(R);
		if (fabs(ang.az)>90)
			ang=myAngle(ang.ax, -ang.ay, -ang.az);
		break;
	case 3:      //zxy
		ang=Get_Angle_zxy(R);
		if (fabs(ang.ax)>90)
			ang=myAngle(-ang.ax, -ang.ay, ang.az);
		break;
	case 4:      //xzy
		ang=Get_Angle_xzy(R);
		if (fabs(ang.az)>90)
			ang=myAngle(ang.ax, -ang.ay, -ang.az);
		break;
	case 5:      //yxz
		ang=Get_Angle_yxz(R);
		if (fabs(ang.ax)>90)
			ang=myAngle(-ang.ax, -ang.ay, ang.az);
		break;
	case 6:      //zyx
		ang=Get_Angle_zyx(R);
		break;
	default:
		break;
	}
	return ang;

}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��Ԫ��<->��ת����
*********************************************************************/
cv::Mat Get_RFromQuaternion(const cv::Mat& q)
{
	double q1 = q.at<float>(0, 0), q2 = q.at<float>(1, 0), q3 = q.at<float>(2, 0), q4 = q.at<float>(3, 0);
	double pR[9] = {
		q1*q1+q2*q2-q3*q3-q4*q4,	2*(q2*q3-q1*q4),					2*(q2*q4+q1*q3),
		2*(q2*q3+q1*q4),			q1*q1+q3*q3-q2*q2-q4*q4,	        2*(q3*q4-q1*q2),
		2*(q2*q4-q1*q3),			2*(q3*q4+q1*q2),			        q1*q1+q4*q4-q2*q2-q3*q3};
	cv::Mat AA(3, 3,CV_32F);
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			AA.at<float>(i, j) = pR[i*3+j];
		}
	}
		//cout << "AA: " << AA << endl;
	return AA;
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��Ԫ��<->��ת����
*********************************************************************/
cv::Mat Get_QuaternionFromR(const cv::Mat& R)
{
	//��Ԫ����ת��ת�ǵĹ�ϵ ˳ʱ����ת q = [ cos(��/2), sin(��/2) * N]
	// q1=cos(��/2) �� cos��= 2*q1^2 - 1 = (tr+1)/2 - 1 = (tr-1) / 2  �� ���ü���ʾ
	cv::Scalar rt = cv::trace(R);
	double q1     = sqrt(rt[0]+1) / 2.0; // ��һ���涨����0
	double pQ[4]  = {q1,                                         (R.at<float>(2,1)-R.at<float>(1,2))/(4*q1), 
		            (R.at<float>(0,2)-R.at<float>(2,0))/(4*q1),  (R.at<float>(1,0)-R.at<float>(0,1))/(4*q1)};
	cv::Mat AA(3, 3,CV_32F);
	for (int i=0; i<4; i++)
	{
		AA.at<float>(i, 0) = pQ[i];
		
	}
	//cout << "AA: " << AA << endl;
	return AA;
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��ת˳��x->y->z, �������������򿴹�ȥ˳ʱ��Ϊ��
*********************************************************************/
void EulerAngle2Rm(const double* angle , cv::Mat& R)
{
	double alpha = angle[0], beta = angle[1], gamma = angle[2];
	alpha =  alpha*CV_PI/180.0;
	beta  =  beta*CV_PI/180.0;
	gamma =  gamma*CV_PI/180.0;
	float sx = sin(alpha);
	float cx = cos(alpha);
	float sy = sin(beta);
	float cy = cos(beta);
	float sz = sin(gamma);
	float cz = cos(gamma);
	R.at<float>(0,0) = cy*cz; R.at<float>(0,1) = sx*sy*cz-cx*sz; R.at<float>(0,2) = cx*sy*cz+sx*sz;
	R.at<float>(1,0) = cy*sz; R.at<float>(1,1) = sx*sy*sz+cx*cz; R.at<float>(1,2) = cx*sy*sz-sx*cz;
	R.at<float>(2,0) = -sy;   R.at<float>(2,1) = sx*cy;          R.at<float>(2,2) = cx*cy;
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��ת˳��x->y->z, �������������򿴹�ȥ˳ʱ��Ϊ��
*********************************************************************/
double*  Rm2EulerAngle(const cv::Mat& rm)
{
	double R[3][3];
	double angle[3];
	for (int i=0; i<3;i++)
	{
		for (int j=0; j<3; j++)
		{
			R[i][j] = rm.at<float>(i, j);
		}
	}
	double Rz = atan2(R[1][0],R[0][0]); 
	double Sz = sin(Rz);
	double Cz = cos(Rz);
	double Ry = atan2(-R[2][0], (R[0][0]*Cz+R[1][0]*Sz));
	double Rx = atan2((R[0][2]*Sz-R[1][2]*Cz), (R[1][1]*Cz-R[0][1]*Sz));
	angle[0] = Rx/CV_PI*180;
	angle[1] = Ry/CV_PI*180;
	angle[2] = Rz/CV_PI*180;
	return angle;
}

myAngle  Rm2EulerAngle2(const cv::Mat& rm)
{
	double R[3][3];
	double angle[3];
	/*R�����ж�������*/
	if(cv::determinant(rm) < 0)
	{
		rm *= -1;
	}
	for (int i=0; i<3;i++)
	{
		for (int j=0; j<3; j++)
		{
			R[i][j] = rm.at<float>(i, j);
		}
	}
	double Rz = atan2(R[1][0],R[0][0]); 
	double Sz = sin(Rz);
	double Cz = cos(Rz);
	double Ry = atan2(-R[2][0], (R[0][0]*Cz+R[1][0]*Sz));
	double Rx = atan2((R[0][2]*Sz-R[1][2]*Cz), (R[1][1]*Cz-R[0][1]*Sz));
	angle[0] = Rx/CV_PI*180;
	angle[1] = Ry/CV_PI*180;
	angle[2] = Rz/CV_PI*180;
	
	return myAngle(angle[0], angle[1], angle[2]);
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��ת˳��x->y->z, �������������򿴹�ȥ˳ʱ��Ϊ�� 
            ���:y��[-90,90],x,z��[-180,180),����Ƕ�
*********************************************************************/
myAngle Get_Angle(const cv::Mat& R)
{
	double angle[3] = {0};	
	double x, y, z;

	/*R�����ж�������*/
	if(cv::determinant(R) < 0)
	{
		R *= -1;
	}
	y = asin( -R.at<float>(2,0) );	//[-90,90], cy >=0
	if( fabs(y*180/CV_PI-90 )<1e-6 )
	{
		z = 0;					// ������z=0
		x = atan( R.at<float>(0,1)/R.at<float>(0,2) );// [-90,90]
	}
	else if( fabs(y*180/CV_PI+90 )<1e-6 )
	{
		z = 0;					// ������z=0
		x = atan( R.at<float>(0,1) / R.at<float>(0,2) );// [-90,90]
	}
	else
	{
		x = atan2( R.at<float>(2,1) , R.at<float>(2,2) );   //[-180,180]

		z = atan2( R.at<float>(1,0) , R.at<float>(0,0) );   //[-180,180]
	}	
	angle[0] = x*180/CV_PI;
	angle[1] = y*180/CV_PI;
	angle[2] = z*180/CV_PI;

	/*double rm[3][3];
	double angle[3];
	for (int i=0; i<3;i++)
	{
		for (int j=0; j<3; j++)
		{
			rm[i][j] = R.at<float>(i, j);
		}
	}
	double Rz = atan(rm[1][0]/rm[0][0]); 
	double Sz = sin(Rz);
	double Cz = cos(Rz);
	double Ry = atan(-rm[2][0]/(rm[0][0]*Cz+rm[1][0]*Sz));
	double Rx = atan((rm[0][2]*Sz-rm[1][2]*Cz)/(rm[1][1]*Cz-rm[0][1]*Sz));
	angle[0] = Rx/CV_PI*180;
	angle[1] = Ry/CV_PI*180;
	angle[2] = Rz/CV_PI*180;*/
	return myAngle(angle[0], angle[1], angle[2]);	
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��ת˳��x->y->z, �������������򿴹�ȥ˳ʱ��Ϊ�� 
            ���:y��[-90,90],x,z��[-180,180),����Ƕ�
*********************************************************************/
myAngle Get_Angle_xyz(const cv::Mat& R)
{
	return Get_Angle(R);	
}

myAngle  Get_Angle_yzx(const cv::Mat& R)
{
	//       ��zw          ��yc
	//  ��xw�L��yw  ��zc���Lxc 

	cv::Mat Rpq = (Mat_<float>(3,3) <<   
		0, 1, 0,
		0, 0, 1,
		1, 0, 0);
	myAngle ang = Get_Angle( Rpq * R * Rpq.t());
	return Get_StandardizeAngle(myAngle(ang.az, ang.ax, ang.ay));
}
/********************************************************************
created:	2013/09/16
created:	2013/09/16   10:10
author:		��Ծǿ	
purpose:    ��ת˳��z->x->y, �������������򿴹�ȥ˳ʱ��Ϊ�� 
���:y��[-90,90],x,z��[-180,180),����Ƕ�
*********************************************************************/
myAngle Get_Angle_zxy(const cv::Mat& R)
{
	//       ��zw           ��xc
	//  ��xw�L��yw    �� yc�L��zc
	cv::Mat Rpq = (Mat_<float>(3,3) <<    
		0, 0, 1,
		1, 0, 0,
		0, 1, 0);
	myAngle ang = Get_Angle( Rpq * R * Rpq.t());
	return Get_StandardizeAngle(myAngle(ang.ay, ang.az, ang.ax));
}

/********************************************************************
����Ҫ����� ����,ȫ���Դ˺���Ϊ��
Xw' = Rww' Xw;
������Ҫ��xw zw yw��˳��ת���ĽǶȣ���Ҫ������xc yc zc,�Ȳ���������
xc        xw             1 0  0
yc = Rwc  yw ����ȻRwc = 0 0 -1 
zc        zw             0 1  0
c����ϵ����תcx cy cz��c'��w' 
Xw' = Rc'w' Rcc' Rwc Xw
Rww' = ~Rwc Rcc' Rwc
Ҳ���Ǵ����wx wz wy��Ӧcx cy cz
xw   1 0  0 xc                      wx   1      cx
yw = 0 0  1 xy����һ��yw��zw˳���  wz =   -1   cy
zw   0 -1 0 xz                      wy        1 cz
*********************************************************************/
myAngle Get_Angle_xzy(const cv::Mat& R)
{
	//       ��zw         ��2        ��yc
	//  ��xw�L��yw  ��3���L1  ��zc���Lxc   �� c��c' �� c'��w'
	cv::Mat Rpq = (Mat_<float>(3,3) <<
		1, 0, 0,
		0, 0, -1,
		0, 1, 0);
	/*Rww' = ~Rwc Rcc' Rwc��Rcc' = Rwc * Rww' * ~Rwc*/
	myAngle aq = Get_Angle( Rpq * R * Rpq.t());
	aq.ay = -aq.ay; 	//�Ƕ�λ�ö�Ӧ��ֻ��һ������
	return Get_StandardizeAngle(myAngle(aq.ax, aq.az, aq.ay));
}
// �����濪ʼ�������һ������ȥ
myAngle Get_Angle_yxz(const cv::Mat&  R)
{
	//       ��zw          ��xc
	//  ��xw�L��yw  ��yc�L��zc 
	cv::Mat Rpq = (Mat_<float>(3,3) <<
		0, 1, 0,
		1, 0, 0,
		0, 0, -1);
	myAngle ap = Get_Angle(Rpq * R * Rpq.t());
	ap.az = -ap.az;
	return Get_StandardizeAngle(myAngle(ap.ay, ap.ax, ap.az));
}
myAngle Get_Angle_zyx(const cv::Mat& R)
{
	//       ��zw     xc���Jzc
	//  ��xw�L��yw  ��   ��yc
	cv::Mat Rpq = (Mat_<float>(3,3) <<
		0, 0, 1,
		0, 1, 0,
		-1, 0, 0 );
	myAngle ap = Get_Angle(Rpq * R * Rpq.t());
	ap.az = -ap.az;
	return Get_StandardizeAngle(myAngle(ap.az, ap.ay, ap.ax));
}

myPosi Get_Posi(const cv::Mat& t)
{
	return myPosi(t.at<float>(0, 0), t.at<float>(1, 0), t.at<float>(2, 0));
}
cv::Mat Get_T(const myPosi posi)
{
	return (Mat_<float>(3, 1)<<posi.tx, posi.ty, posi.tz);
}

cv::Mat Get_T(double* posi)
{
	return Get_T(myPosi(posi[0], posi[1], posi[2]));
}