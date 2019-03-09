#include <queue>
#include "wrapImageRef.h"
#include "rotationmath.h"

extern "C"
{
#include "lsd.h"
};
#include "VPDetection.h"
using namespace std;
using namespace cv;
VPDetection detector;
// LSD line segment detection
void LineMerge(std::vector< std::vector<double> > &lines, float termTh, float distTh, float angTh);
void LineDetect(cv::Mat image, double thLength, std::vector<std::vector<double> > &lines)
{
	lines.clear();
	cv::Mat grayImage;
	if (image.channels() == 1)
		grayImage = image;
	else
		cv::cvtColor(image, grayImage, CV_BGR2GRAY);

	image_double imageLSD = new_image_double(grayImage.cols, grayImage.rows);
	unsigned char* im_src = (unsigned char*)grayImage.data;

	int xsize = grayImage.cols;
	int ysize = grayImage.rows;
	for (int y = 0; y < ysize; ++y)
	{
		for (int x = 0; x < xsize; ++x)
		{
			imageLSD->data[y * xsize + x] = im_src[y * xsize + x];
		}
	}

	ntuple_list linesLSD = lsd(imageLSD);
	free_image_double(imageLSD);

	int nLines = linesLSD->size;
	int dim = linesLSD->dim;
	std::vector<double> lineTemp(4);
	for (int i = 0; i < nLines; ++i)
	{
		double x1 = linesLSD->values[i * dim + 0];
		double y1 = linesLSD->values[i * dim + 1];
		double x2 = linesLSD->values[i * dim + 2];
		double y2 = linesLSD->values[i * dim + 3];

		double l = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
		if (l > thLength)
		{
			lineTemp[0] = x1;
			lineTemp[1] = y1;
			lineTemp[2] = x2;
			lineTemp[3] = y2;

			lines.push_back(lineTemp);
		}
	}
//    LineMerge(lines, 3.0, 20, 5.0);
	free_ntuple_list(linesLSD);
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

double angle_line(double x1, double y1, double x2, double y2)
{
	double ang = 180.0 * atan2(float(y1 - y2), float(x1 - x2)) / CV_PI;
	if (ang < 0)
	{
		ang += 180;
	}
	return ang;
}

void  FitLine2D(CvPoint2D32f* points, int count, int dist,
	void *param, float reps, float aeps, float* line)
{
	CvMat mat = cvMat(1, count, CV_32FC2, points);
	float _param = param != NULL ? *(float*)param : 0.f;
	assert(dist != CV_DIST_USER);
	cvFitLine(&mat, dist, _param, reps, aeps, line);
}

int minimal(double d1, double d2, double d3, double d4) {
	if (d1 <= d2 && d1 <= d3 && d1 <= d4) { return 1; }
	if (d2 <= d1 && d2 <= d3 && d2 <= d4) { return 2; }
	if (d3 <= d1 && d3 <= d2 && d3 <= d4) { return 3; }
	if (d4 <= d1 && d4 <= d2 && d4 <= d3) { return 4; }
	return 0;
}

double minimum(double d1, double d2, double d3, double d4) {
	if (d1 <= d2 && d1 <= d3 && d1 <= d4) { return d1; }
	if (d2 <= d1 && d2 <= d3 && d2 <= d4) { return d2; }
	if (d3 <= d1 && d3 <= d2 && d3 <= d4) { return d3; }
	if (d4 <= d1 && d4 <= d2 && d4 <= d3) { return d4; }
	return 0;
}

void LineMerge(std::vector< std::vector<double> > &lines, float termTh, float distTh, float angTh)
{
	vector<bool> flag;            //ֱ�ߺϲ���־
	vector<double> angles;            //ֱ�߽Ƕ�
	vector<double> lengths;            //ֱ�߳���
	for (int i = 0; i < lines.size(); i++)
	{
		flag.push_back(false);
		angles.push_back(angle_line(lines[i][0], lines[i][1], lines[i][2], lines[i][3]));
		lengths.push_back(distance(lines[i][0], lines[i][1], lines[i][2], lines[i][3]));
	}

	float angle = 0.0;
	float angle2 = 0.0;
	float angle3 = 0.0;
	float endpointdis11, endpointdis12, endpointdis21, endpointdis22, endpointdis, midpointdis;
	int pointpair;
	CvPoint2D32f point1, point2;
	//����ֱ�߼�ļн�,�Լ�ֱ�߼�ľ���
	for (int i = 0; i < lines.size(); i++)
	{
		int j;
		if (flag[i])
			continue;
		for (j = i + 1; j < lines.size(); j++)
		{
			//�ж�ֱ�߼н�
			if (flag[j])
				continue;
			angle = fabs(angles[i] - angles[j]);
			if (angle > 90)
			{
				angle = 180 - angle;
			}

			if ((angle < angTh))
			{
				//�ж϶˵����루p2-p3 or p1-p4��(p1,p2),(p3,p4)

				endpointdis11 = distance(lines[i][0], lines[i][1], lines[j][0], lines[j][1]);
				endpointdis12 = distance(lines[i][0], lines[i][1], lines[j][2], lines[j][3]);
				endpointdis21 = distance(lines[i][2], lines[i][3], lines[j][0], lines[j][1]);
				endpointdis22 = distance(lines[i][2], lines[i][3], lines[j][2], lines[j][3]);
				endpointdis = minimum(endpointdis11, endpointdis12, endpointdis21, endpointdis22);
				//���ؾ�������Ķ˵�����
				pointpair = minimal(endpointdis11, endpointdis12, endpointdis21, endpointdis22);
				midpointdis = distance((lines[i][0] + lines[i][2]) / 2.0, (lines[i][1] + lines[i][3]) / 2.0, (lines[j][0] + lines[j][2]) / 2.0, (lines[j][1] + lines[j][3]) / 2.0);
				if ((endpointdis < distTh/**MIN((*line_detected)[i].len,(*line_detected)[j].len)*/) && (midpointdis > (MAX(lengths[i] / 2, lengths[j] / 2))))
				{
					//�ж�������
					float a, b, c;
					CvPoint2D32f* edge = new CvPoint2D32f[4];
					float* line = new float[4];
					edge[0] = cvPoint2D32f(lines[i][0], lines[i][1]);
					edge[1] = cvPoint2D32f(lines[i][2], lines[i][3]);
					edge[2] = cvPoint2D32f(lines[j][0], lines[j][1]);
					edge[3] = cvPoint2D32f(lines[j][2], lines[j][3]);
					//����ĸ���
					FitLine2D(edge, 4, CV_DIST_L2, 0, 0.01, 0.01, line);
					a = -line[0];
					b = line[1];
					c = line[0] * line[3] - line[1] * line[2];
					if (line != NULL)
					{
						delete[] line;
						line = NULL;
					}
					if (edge != NULL)
					{
						delete[] edge;
						edge = NULL;
					}
					//����������
					float fiterror1 = fabs(lines[i][0] * b + lines[i][1] * a + c);
					float fiterror2 = fabs(lines[i][2] * b + lines[i][3] * a + c);
					float fiterror3 = fabs(lines[j][0] * b + lines[j][1] * a + c);
					float fiterror4 = fabs(lines[i][2] * b + lines[i][3] * a + c);
					float fiterror = (fiterror1 + fiterror2 + fiterror3 + fiterror4) / 4.0;

					if (fiterror < termTh)
					{
						//���˵�����ͶӰ�����ֱ��
						double X1, Y1, X2, Y2;
						if (pointpair == 1)
						{
							point1 = cvPoint2D32f(lines[i][2], lines[i][3]);
							point2 = cvPoint2D32f(lines[j][2], lines[j][3]);
						}
						if (pointpair == 2)
						{
							point1 = cvPoint2D32f(lines[i][2], lines[i][3]);
							point2 = cvPoint2D32f(lines[j][0], lines[j][1]);
						}
						if (pointpair == 3)
						{
							point1 = cvPoint2D32f(lines[i][0], lines[i][1]);
							point2 = cvPoint2D32f(lines[j][2], lines[j][3]);
						}
						if (pointpair == 4)
						{
							point1 = cvPoint2D32f(lines[i][0], lines[i][1]);
							point2 = cvPoint2D32f(lines[j][0], lines[j][1]);
						}
						if (point1.x < point2.x)
						{
							X1 = point1.y;
							Y1 = point1.x;
							X2 = point2.y;
							Y2 = point2.x;

						}
						else
						{
							X1 = point2.y;
							Y1 = point2.x;
							X2 = point1.y;
							Y2 = point1.x;

						}
						a = a / sqrt(a*a + b*b);
						b = b / sqrt(a*a + b*b);
						double p1_x = a*a*Y1 - a*b*X1 - b*c;
						double p1_y = b*b*X1 - a*b*Y1 - a*c;
						double p2_x = a*a*Y2 - a*b*X2 - b*c;
						double p2_y = b*b*X2 - a*b*Y2 - a*c;

						double x1 = p1_x;
						double y1 = p1_y;
						double x2 = p2_x;
						double y2 = p2_y;
						vector<double> li;
						li.push_back(x1);
						li.push_back(y1);
						li.push_back(x2);
						li.push_back(y2);
						lines.push_back(li);
						lengths.push_back(distance(x1, y1, x2, y2));
						angles.push_back(angle_line(x1, y1, x2, y2));
						flag.push_back(false);

						flag[i] = true;
						flag[j] = true;
						break;
					}

				}
			}
		}
	}
	std::vector< std::vector<double> > _lines;
	for (int i = 0; i < lines.size(); i++)
	{
		if (!flag[i])
			_lines.push_back(lines[i]);

	}
	flag.clear();
	angles.clear();
	lengths.clear();
	lines.clear();
	for (int i = 0; i < _lines.size(); i++)
	{
		if (lengths[i] < 30)
		{
			continue;
		}
		lines.push_back(_lines[i]);
	}
	_lines.clear();
}

void drawClusters(cv::Mat &img, std::vector<std::vector<double> > &lines, std::vector<std::vector<int> > &clusters)
{
	int cols = img.cols;
	int rows = img.rows;

	//draw lines
	std::vector<cv::Scalar> lineColors(3);
	lineColors[0] = cv::Scalar(0, 0, 255);
	lineColors[1] = cv::Scalar(0, 255, 0);
	lineColors[2] = cv::Scalar(255, 0, 0);

	for (int i = 0; i < lines.size(); ++i)
	{
		int idx = i;
		cv::Point pt_s = cv::Point(lines[idx][0], lines[idx][1]);
		cv::Point pt_e = cv::Point(lines[idx][2], lines[idx][3]);
		cv::Point pt_m = (pt_s + pt_e) * 0.5;

		cv::line(img, pt_s, pt_e, cv::Scalar(0, 0, 0), 2, CV_AA);
	}

	for (int i = 0; i < clusters.size(); ++i)
	{
		for (int j = 0; j < clusters[i].size(); ++j)
		{
			int idx = clusters[i][j];

			cv::Point pt_s = cv::Point(lines[idx][0], lines[idx][1]);
			cv::Point pt_e = cv::Point(lines[idx][2], lines[idx][3]);
			cv::Point pt_m = (pt_s + pt_e) * 0.5;

			cv::line(img, pt_s, pt_e, lineColors[i], 2, CV_AA);
		}
	}
}

void filterClusters(std::vector<std::vector<int> > &clusters, std::vector<cv::Point3d> &vps)
{
	typedef pair< int, int > cluster;
	priority_queue< cluster > clusters_num;
	std::vector<std::vector<int> > _clusters;
	std::vector<cv::Point3d>       _vps;
	for (int i = 0; i < clusters.size(); ++i)
	{
		clusters_num.push(cluster(clusters[i].size(), i));
	}
	for (int i = 0; i < MIN(clusters_num.size(), 2); ++i)
	{
		_clusters.push_back(clusters[clusters_num.top().second]);
		_vps.push_back(vps[clusters_num.top().second]);
		clusters_num.pop();
	}
	clusters.clear();
	vps.clear();
	clusters = _clusters;
	vps = _vps;
}

bool line2VPS(std::vector<std::vector<double> > &lines, std::vector<std::vector<int> > &clusters, std::vector<cv::Point3d> &vps)
{
	typedef pair< int, int > cluster;
	priority_queue< cluster > clusters_num;
	std::vector<std::vector<int> > _clusters;
	std::vector<cv::Point3d>       _vps;
	std::cout<<"clusters:"<<clusters.size()<<endl;
	for (int i = 0; i < clusters.size(); ++i)
	{
		clusters_num.push(cluster(clusters[i].size(), i));
	}
	int NN = 0;
	vector<double> angle;
	for (int i = 0; i < MIN(clusters_num.size(), 2); ++i)
	{
		_clusters.push_back(clusters[clusters_num.top().second]);
		_vps.push_back(vps[clusters_num.top().second]);
		clusters_num.pop();
	}
	for (int i = 0; i < _clusters.size(); ++i)
	{
		if (_clusters[i].size()>10)
		{
			NN++;
		}
	}
	clusters.clear();
	vps.clear();

	if(NN < 2) {
		//todo
		return false;
	}
	vps = _vps;
	clusters = _clusters;
//	typedef pair< double, int> lineSeg;
	for (int g = 0; g < clusters.size(); ++g) {
//		priority_queue< lineSeg > lineSegs;
		int m = clusters[g].size();
		cv::Mat A = cv::Mat::zeros(m, 3, CV_64F);
//		cv::Mat b = cv::Mat::zeros(m, 1, CV_64F);
		cv::Mat x;
		for (int l = 0; l < clusters[g].size(); ++l) {
			int index = clusters[g][l];
			cv::Mat_<double> p1 = ( cv::Mat_<double>(3, 1) << lines[index][0], lines[index][1], 1.0 );
			cv::Mat_<double> p2 = ( cv::Mat_<double>(3, 1) << lines[index][2], lines[index][3], 1.0 );

			cv::Mat para = p1.cross( p2 );
            A.at<double>(l, 0) = para.at<double>(0);
			A.at<double>(l, 1) = para.at<double>(1);
			A.at<double>(l, 2) = para.at<double>(2);

//			lineSegs.push(lineSeg(cv::norm(p1-p2), l));
		}
		//another method, do not care now, maybe use later if the number of lines is very big
//        std::vector<int> lis_para;
//		for (int i = 0; i < MIN(lineSegs.size(), 2); ++i)
//		{
//			lis_para.push_back(lineSegs.top().second);
//			lineSegs.pop();
//		}
		SVD::solveZ(A,x);

		vps[g].x = x.at<double>(0)/x.at<double>(2) - detector.pp.x;
		vps[g].y = x.at<double>(1)/x.at<double>(2) - detector.pp.y;
		vps[g].z = detector.f;
	}
    return  true;
}

//angThresh,threshold for rotation about x and y axis
//allRot = false , only using the rotation with z axis
//bShape = true  , keep the shap
bool wrapImagePerspective(cv::Mat &src, cv::Mat &dst, cv::Mat &H, cv::Mat &R, double angThresh, bool bShape, bool allRot, bool bDrawLis)
{
	// LSD line segment detection
	double thLength = 20.0;
	std::vector<std::vector<double> > lines;
	LineDetect(src, thLength, lines);
	if (lines.size()<40)
	{
		return  false;
	}
	// Camera internal parameters
	cv::Point2d pp(src.cols / 2, src.rows / 2);        // Principle point (in pixel)
													   //double f = 6.053 / 0.009, Focal length (in pixel)
	double f = 1.2*MAX(src.cols, src.rows);
	// Vanishing point detection
	std::vector<cv::Point3d> vps;              // Detected vanishing points
	std::vector<std::vector<int> > clusters;   // Line segment clustering results of each vanishing point

	detector.run(lines, pp, f, vps, clusters);

   	//filterClusters(clusters, vps);
	if (!line2VPS(lines, clusters, vps))
    {
        if (bDrawLis)
        {
            drawClusters(src, lines, clusters);
        }
        return  false;
    }
	if (vps.size()<2)
	{
        if (bDrawLis)
        {
            drawClusters(src, lines, clusters);
        }
		return false;
	}
	//
	if (bShape)
	{
		if (fabs(vps[0].x) < fabs(vps[1].x))
		{
			cv::Point3d tp = vps[1];
			vps[1] = vps[0];
			vps[0] = tp;
			std::vector<int> tp2 = clusters[1];
			clusters[1] = clusters[0];
			clusters[0] = tp2;
		}
		if (vps[0].x<0)
		{
			vps[0] = -vps[0];
		}
		if (vps[1].y < 0)
		{
			vps[1] = -vps[1];
		}
	}
	//calibration matrix of camera
	cv::Mat K = (cv::Mat_<float>(3, 3) << f, 0, pp.x, 0, f, pp.y, 0, 0, 1);
	//vp1 = KR[1,0,0]T=Kr1 vp2 = KR[0, 1, 0]T=Kr2
	//H = K(KR)-1 = KRTK-1
	cv::Mat K_inv = K.inv();
	cv::Mat v1 = (cv::Mat_<float>(3, 1) << vps[0].x, vps[0].y, vps[0].z);  //v1 = v1 / vps[0].z;
	cv::Mat v2 = (cv::Mat_<float>(3, 1) << vps[1].x, vps[1].y, vps[1].z);  //v2 = v2 / vps[1].z;
	cv::Mat r1 = v1.clone();   r1 = r1 / cv::norm(r1);
	cv::Mat r2 = v2.clone();   r2 = r2 / cv::norm(r2);
	cout << r1 << endl;
	cout << r2 << endl;

	//test the angle between two vectors
	double angle = acos(r1.dot(r2));
	angle =  angle*180.0/CV_PI;
	if (angle>90)
    {
	    angle = 180-angle;
    }
	std::cout<<"vector angle:"<<angle<<endl;
	if (angle<85)
    {
        if (bDrawLis)
        {
            drawClusters(src, lines, clusters);
        }
        return  false;
    }

//	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
	r1.copyTo(R.colRange(0, 1).rowRange(0, 3));
	r2.copyTo(R.colRange(1, 2).rowRange(0, 3));
	cv::Mat r3 = R.col(0).cross(R.col(1)); r3 = r3 / cv::norm(r3);
	r3.copyTo(R.colRange(2, 3).rowRange(0, 3));

	myAngle ang_rot = Get_EularAngle(R);
//	ang_rot.ax = ang_rot.ax > 0 ? MIN(ang_rot.ax, angThresh) : MAX(ang_rot.ax, -angThresh);
	ang_rot.ay = ang_rot.ay > 0 ? MIN(ang_rot.ay, angThresh) : MAX(ang_rot.ay, -angThresh);
    ang_rot.az = 0;
	R = allRot ? Get_R(ang_rot) : Get_R(myAngle(0, 0, ang_rot.az));

	//cout << R << endl;
	//cout << cv::determinant(R)<< endl;
	//homgraphy matrix from the from to the wrapped image
	H = K*R.t()*K_inv;

	//attach a perspective transform
	cv::Mat pt0 = H*(cv::Mat_<float>(3, 1) << 0, 0, 1);
	cv::Mat pt1 = H*(cv::Mat_<float>(3, 1) << src.cols, 0, 1);
	cv::Mat pt2 = H*(cv::Mat_<float>(3, 1) << src.cols, src.rows, 1);
	cv::Mat pt3 = H*(cv::Mat_<float>(3, 1) << 0, src.rows, 1);
	pt0 = pt0 / pt0.at<float>(2);
	pt1 = pt1 / pt1.at<float>(2);
	pt2 = pt2 / pt2.at<float>(2);
	pt3 = pt3 / pt3.at<float>(2);
	vector<cv::Point2f> pts;
	pts.push_back(cv::Point2f(pt0.at<float>(0), pt0.at<float>(1)));
	pts.push_back(cv::Point2f(pt1.at<float>(0), pt1.at<float>(1)));
	pts.push_back(cv::Point2f(pt2.at<float>(0), pt2.at<float>(1)));
	pts.push_back(cv::Point2f(pt3.at<float>(0), pt3.at<float>(1)));
	cv::Rect wrap_rect = cv::boundingRect(pts);
	
	cv::Size dsize(MAX(src.cols, src.rows), MAX(src.cols, src.rows)*wrap_rect.height / float(wrap_rect.width));
	if (MIN(dsize.width, dsize.height)<200)
	{
		return false;
	} 
	cv::Point2f pts0[] = { cv::Point2f(wrap_rect.x, wrap_rect.y), cv::Point2f(wrap_rect.x + wrap_rect.width, wrap_rect.y + 0), cv::Point2f(wrap_rect.x + wrap_rect.width, wrap_rect.y + wrap_rect.height), cv::Point2f(wrap_rect.x, wrap_rect.y + wrap_rect.height) };
	
	cv::Point2f pts1[] = { cv::Point2f(0, 0), cv::Point2f(dsize.width, 0), cv::Point2f(dsize.width, dsize.height), cv::Point2f(0, dsize.height) };
	cv::Mat H0 = cv::getPerspectiveTransform(pts0, pts1);
//	cout << wrap_rect << endl;
//	cout << dsize << endl;
//	cout << H0 << endl;
//	cout << src.size() << endl;
	cv::Mat H1;
	H0.convertTo(H1, CV_32F);
	H = H1*H;
	cv::warpPerspective(src, dst, H, dsize);
	if (bDrawLis)
	{
		drawClusters(src, lines, clusters);
	}
	return true;
}

void calImageRotation(cv::Mat &src, cv::Mat &R, double angThresh, bool allRot)
{
	// LSD line segment detection
	double thLength = 30.0;
	std::vector<std::vector<double> > lines;
	LineDetect(src, thLength, lines);
	// Camera internal parameters
	cv::Point2d pp(src.cols / 2, src.rows / 2);        // Principle point (in pixel)
													   //double f = 6.053 / 0.009, Focal length (in pixel)
	double f = 1.2*MAX(src.cols, src.rows);
	// Vanishing point detection
	std::vector<cv::Point3d> vps;              // Detected vanishing points
	std::vector<std::vector<int> > clusters;   // Line segment clustering results of each vanishing point
	VPDetection detector;
	detector.run(lines, pp, f, vps, clusters);
	filterClusters(clusters, vps);
	cv::Mat v1 = (cv::Mat_<float>(3, 1) << vps[0].x, vps[0].y, vps[0].z);  //v1 = v1 / vps[0].z;
	cv::Mat v2 = (cv::Mat_<float>(3, 1) << vps[1].x, vps[1].y, vps[1].z);  //v2 = v2 / vps[1].z;
	cv::Mat r1 = v1.clone();   r1 = r1 / cv::norm(r1);
	cv::Mat r2 = v2.clone();   r2 = r2 / cv::norm(r2);
	cout << r1 << endl;
	cout << r2 << endl;
	R = cv::Mat::eye(3, 3, CV_32F);
	r1.copyTo(R.colRange(0, 1).rowRange(0, 3));
	r2.copyTo(R.colRange(1, 2).rowRange(0, 3));
	cv::Mat r3 = R.col(0).cross(R.col(1)); r3 = r3 / cv::norm(r3);
	r3.copyTo(R.colRange(2, 3).rowRange(0, 3));

	myAngle ang_rot = Get_EularAngle(R);
	ang_rot.ax = ang_rot.ax > 0 ? MIN(ang_rot.ax, angThresh) : MAX(ang_rot.ax, -angThresh);
	ang_rot.ay = ang_rot.ay > 0 ? MIN(ang_rot.ay, angThresh) : MAX(ang_rot.ay, -angThresh);

	R = allRot ? Get_R(ang_rot) : Get_R(myAngle(0, 0, ang_rot.az));
}

void wrapImagePerspectiveH(cv::Mat &src, cv::Mat &dst, cv::Mat &R, cv::Mat &H)
{
	// Camera internal parameters
	cv::Point2d pp(src.cols / 2, src.rows / 2);        // Principle point (in pixel)
	//double f = 6.053 / 0.009, Focal length (in pixel)
	double f = 1.2*MAX(src.cols, src.rows);
	//calibration matrix of camera
	cv::Mat K = (cv::Mat_<float>(3, 3) << f, 0, pp.x, 0, f, pp.y, 0, 0, 1);
	//vp1 = KR[1,0,0]T=Kr1 vp2 = KR[0, 1, 0]T=Kr2
	//H = K(KR)-1 = KRTK-1
	cv::Mat K_inv = K.inv();
	H = K*R.t()*K_inv*H;
	//cout << H << endl;
	cv::Mat pt0 = H*(cv::Mat_<float>(3, 1) << 0, 0, 1);
	cv::Mat pt1 = H*(cv::Mat_<float>(3, 1) << src.cols, 0, 1);
	cv::Mat pt2 = H*(cv::Mat_<float>(3, 1) << src.cols, src.rows, 1);
	cv::Mat pt3 = H*(cv::Mat_<float>(3, 1) << 0, src.rows, 1);
	pt0 = pt0 / pt0.at<float>(2);
	pt1 = pt1 / pt1.at<float>(2);
	pt2 = pt2 / pt2.at<float>(2);
	pt3 = pt3 / pt3.at<float>(2);
	vector<cv::Point2f> pts;
	pts.push_back(cv::Point2f(pt0.at<float>(0), pt0.at<float>(1)));
	pts.push_back(cv::Point2f(pt1.at<float>(0), pt1.at<float>(1)));
	pts.push_back(cv::Point2f(pt2.at<float>(0), pt2.at<float>(1)));
	pts.push_back(cv::Point2f(pt3.at<float>(0), pt3.at<float>(1)));

	cv::Rect wrap_rect = cv::boundingRect(pts);
    cv::Size dsize(MIN(wrap_rect.width, src.cols*1.5), wrap_rect.height/float(wrap_rect.width)*MIN(wrap_rect.width, src.cols*1.5));
	cv::Point2f pts0[] = { cv::Point2f(wrap_rect.x, wrap_rect.y), cv::Point2f(wrap_rect.x + wrap_rect.width, wrap_rect.y + 0), cv::Point2f(wrap_rect.x + wrap_rect.width, wrap_rect.y + wrap_rect.height), cv::Point2f(wrap_rect.x, wrap_rect.y + wrap_rect.height) };
	cv::Point2f pts1[] = { cv::Point2f(0, 0), cv::Point2f(dsize.width, 0), cv::Point2f(dsize.width, dsize.height), cv::Point2f(0, dsize.height) };

	cv::Mat H0 = cv::getPerspectiveTransform(pts0, pts1);

	cv::Mat H1;
	H0.convertTo(H1, CV_32F);
    H = H1*H;
	cv::warpPerspective(src, dst, H, dsize);
}

void wrapImagePerspectiveR(cv::Mat &src, cv::Mat &dst, cv::Mat R)
{
	cv::Point2d pp(src.cols / 2, src.rows / 2);        // Principle point (in pixel)											   
	double f = 1.2*MAX(src.cols, src.rows);
	//calibration matrix of camera
	cv::Mat K = (cv::Mat_<float>(3, 3) << f, 0, pp.x, 0, f, pp.y, 0, 0, 1);
	cv::Mat K_inv = K.inv();
	cv::Mat H = K*R.t()*K_inv;
	//cout << H << endl;
	cv::Mat pt0 = H*(cv::Mat_<float>(3, 1) << 0, 0, 1);
	cv::Mat pt1 = H*(cv::Mat_<float>(3, 1) << src.cols, 0, 1);
	cv::Mat pt2 = H*(cv::Mat_<float>(3, 1) << src.cols, src.rows, 1);
	cv::Mat pt3 = H*(cv::Mat_<float>(3, 1) << 0, src.rows, 1);
	pt0 = pt0 / pt0.at<float>(2);
	pt1 = pt1 / pt1.at<float>(2);
	pt2 = pt2 / pt2.at<float>(2);
	pt3 = pt3 / pt3.at<float>(2);
	vector<cv::Point2f> pts;
	pts.push_back(cv::Point2f(pt0.at<float>(0), pt0.at<float>(1)));
	pts.push_back(cv::Point2f(pt1.at<float>(0), pt1.at<float>(1)));
	pts.push_back(cv::Point2f(pt2.at<float>(0), pt2.at<float>(1)));
	pts.push_back(cv::Point2f(pt3.at<float>(0), pt3.at<float>(1)));

	cv::Rect wrap_rect = cv::boundingRect(pts);
	cv::Point2f pts0[] = { cv::Point2f(wrap_rect.x, wrap_rect.y), cv::Point2f(wrap_rect.x + wrap_rect.width, wrap_rect.y + 0), cv::Point2f(wrap_rect.x + wrap_rect.width, wrap_rect.y + wrap_rect.height), cv::Point2f(wrap_rect.x, wrap_rect.y + wrap_rect.height) };
	cv::Point2f pts1[] = { cv::Point2f(0, 0), cv::Point2f(wrap_rect.width, 0), cv::Point2f(wrap_rect.width, wrap_rect.height), cv::Point2f(0, wrap_rect.height) };

	cv::Mat H0 = cv::getPerspectiveTransform(pts0, pts1);

	cv::Mat H1;
	H0.convertTo(H1, CV_32F);

	cv::warpPerspective(src, dst, H1*H, cv::Size(wrap_rect.width, wrap_rect.height));
}

//std::vector<cv::Point> trans_image_pts(cv::Point center, double angle, std::vector<cv::Point> pts)
//{
//	std::vector<cv::Point> pts_trans;
//	double aaz = angle*CV_PI/180.0; //deg2rad
//	cv::Mat Rz =  (Mat_<float>(2,2) <<
//		cos(aaz),	-sin(aaz),
//		sin(aaz),	 cos(aaz));
//	cv::Mat pt0 = (Mat_<float>(2,1) <<  center.x,	center.y);
//	for(int i = 0; i < pts.size(); i++)
//	{
//		cv::Mat pt = (Mat_<float>(2,1) <<  pts[i].x,	pts[i].y);
//		//move to center
//		pt = pt - center;
//		cv::Mat pt_tans = Rz*pt;
//		//move to image frame
//		pt_tans += pt0;
//		pts_trans.push_back(cv::Point(pt_tans.at<float>(0), pt_tans.at<float>(1)));
//
//	}
//
//	return pts_trans;
//}

