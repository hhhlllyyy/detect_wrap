//
// Created by gh on 19-2-16.
//

#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <wrapImageRef.h>
#include "curl/curl.h"
#include <sstream>
#include "json.hpp"
using json = nlohmann::json;
using namespace cv;
using namespace std;
std::vector<std::vector<cv::Point>> get_box_post(std::string url,std::string  img_name);

static size_t OnWriteData(void *buffer, size_t size, size_t nmemb,
                          void *lpVoid) {
    std::string *str = dynamic_cast<std::string *>((std::string *) lpVoid);
    if (nullptr == str || nullptr == buffer) {
        return -1;
    }

    char *pData = (char *) buffer;
    str->append(pData, size * nmemb);

    return nmemb;
}
std::vector<std::vector<cv::Point>> get_box_post(std::string url,std::string  img_name)
{
    std::vector<std::vector<cv::Point>> boxes;
    std::string response;

    CURL *curl = curl_easy_init();
    long respCode = 0;
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.data());

        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 50000L);
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, NULL);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, OnWriteData);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *) &response);

        struct curl_slist *headers = NULL;

        headers = curl_slist_append(headers, "cache-control: no-cache");

        headers = curl_slist_append(
                headers, "application/x-www-form-urlencoded;charset=utf-8");
        headers =
                curl_slist_append(headers,
                                  "Content-Type:multipart/form-data; "
                                  "boundary=----WebKitFormBoundary55dqueBxQGGV9lAD");

        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        struct curl_httppost *formpost = NULL;
        struct curl_httppost *lastptr = NULL;


        curl_formadd(&formpost, &lastptr,
                     CURLFORM_COPYNAME, "image",
                     CURLFORM_CONTENTTYPE, "image/jpeg",
                     CURLFORM_END);

        curl_slist *pOptionList = NULL;
        pOptionList = curl_slist_append(pOptionList, "Expect:");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, pOptionList);


        curl_formadd(&formpost, &lastptr, CURLFORM_COPYNAME, "image", CURLFORM_FILE, img_name.data(), CURLFORM_CONTENTTYPE,
                     "image/jpeg", CURLFORM_END);

        curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);


        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK) {
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &respCode);
        } else {
        }
        // LOGE(" code = %d",res);
        curl_easy_cleanup(curl);
        curl_formfree(formpost);

    }
    json obj_json = json::parse(response);
    auto err_code = obj_json.at("err_code").get<int>();

    int num_rows[1000] = {0};
    std::vector<int> boxes_row;
    if(err_code == 0)
    {
        auto result_array = obj_json.at("list");
        for (auto& element : result_array)
        {
            std::vector<cv::Point> pts_set;
            auto points = element.at("points");
            int  row = element.at("row").get<int>();
            if (row>=0&&row<1000)
            {
                num_rows[row]++;
            }
            boxes_row.push_back(row);
            for (auto& pt : points)
            {
                int x = pt.at("x").get<int>();
                int y = pt.at("y").get<int>();
                pts_set.push_back(cv::Point(x, y));
//                std::cout<<"x:"<<x<<"   y:"<<y<<std::endl;
            }
            boxes.push_back(pts_set);
        }
    }
    std::vector<std::vector<cv::Point>> _boxes;
    for (int i = 0; i < boxes.size(); ++i) {
        if (boxes_row[i]>=0&&boxes_row[i]<1000&&num_rows[boxes_row[i]]>2)
        {
            _boxes.push_back(boxes[i]);
        }
    }
    boxes = _boxes;
    cv::Mat imgdis = cv::imread(img_name, 1);
    for (int i = 0; i < boxes.size(); ++i) {
        cv::polylines(imgdis, boxes[i], true, cv::Scalar(255, 0, 0), 2);
        cv::circle(imgdis, boxes[i][0], 4, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("box", imgdis);

    std::cout << "response: " << response << std::endl;
    curl_global_cleanup();
    std::cout<<"boxsize:"<<boxes.size()<<std::endl;
    return  boxes;
}


#ifndef CLAMP
#define CLAMP(x,a,b) MAX((a), MIN((b), (x)))
#endif
cv::Rect get_rect_valid(cv::Point pt1, cv::Point pt2, int w, int h)
{
    int x0 = CLAMP(pt1.x, 0, w);
    int x1 = CLAMP(pt2.x, 0, w);
    int y0 = CLAMP(pt1.y, 0, h);
    int y1 = CLAMP(pt2.y, 0, h);
    return cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1));
}
cv::Rect get_boundingRect(std::vector<std::vector<cv::Point>> &boxes, cv::Size im, cv::Mat A)
{
    int minX = 10000, minY = 10000, maxX = 0, maxY = 0;
    for (int i=0; i<boxes.size(); i++)
    {
        for (int j=0; j<boxes[i].size(); j++)
        {
            cv::Mat pt = A*(cv::Mat_<float>(3, 1)<<boxes[i][j].x, boxes[i][j].y, 1);
            double x = pt.at<float>(0);
            double y = pt.at<float>(1);
            if (x<minX)
            {
                minX = x;
            }

            if (x > maxX)
            {
                maxX = x;
            }

            if (y < minY)
            {
                minY = y;
            }

            if (y >maxY)
            {
                maxY = y;
            }
        }
    }
    cv::Rect rec = get_rect_valid(cv::Point(minX, minY), cv::Point(maxX, maxY), im.width-1, im.height-1);
    return rec;
}

double calcLineDegree(const cv::Point2f& firstPt, const cv::Point2f& secondPt)
{
    double curLineAngle = 0.0f;
    if (secondPt.x - firstPt.x != 0)
    {
        curLineAngle = atan(static_cast<double>(firstPt.y - secondPt.y) / static_cast<double>(secondPt.x - firstPt.x));
//        if (curLineAngle < 0)
//        {
//            curLineAngle += CV_PI;
//        }
    }
    else
    {
        curLineAngle = secondPt.y - firstPt.y>0 ?CV_PI / 2.0f : -CV_PI / 2.0f ; //90度
    }
    return curLineAngle*180.0f/CV_PI;
}
//angle is the angle between short eage and x axis
double getRcDegree(std::vector<cv::Point> vertVect)
{
    double degree = 0.0f;
    //line 1
    const double firstLineLen = (vertVect[1].x - vertVect[0].x)*(vertVect[1].x - vertVect[0].x) +
                                (vertVect[1].y - vertVect[0].y)*(vertVect[1].y - vertVect[0].y);
    //line 2
    const double secondLineLen = (vertVect[2].x - vertVect[1].x)*(vertVect[2].x - vertVect[1].x) +
                                 (vertVect[2].y - vertVect[1].y)*(vertVect[2].y - vertVect[1].y);
    if (firstLineLen < secondLineLen)
    {
        degree = calcLineDegree(vertVect[0], vertVect[1]);
    }
    else
    {
        degree = calcLineDegree(vertVect[2], vertVect[1]);
    }
    return degree;
}

//cal the rotation angle
int test_box(std::vector<std::vector<cv::Point>> &boxes)
{
    int angle0 = -90;
    int angle1 =  90;
    int NB = 12;
    cv::Mat angle_hist = cv::Mat::zeros(NB+1, 1, CV_32F);
    double angle_detar = (angle1-angle0)/NB;
    std::vector<double > angles;
    for (int i = 0; i < NB; ++i) {
        angles.push_back(angle0+i*angle_detar);
    }
    angles.push_back(angle1);

    for (int i = 0; i < boxes.size(); ++i) {
//        cv::RotatedRect box = cv::minAreaRect(boxes[i]);
        double angle = getRcDegree(boxes[i]);
        for (int j = 0; j < NB+1; ++j) {
            if (angle>=angles[j]-angle_detar/2.0&&angle<angles[j]+angle_detar/2.0)
            {
                angle_hist.at<float>(j,0)++;
                break;
            }
        }
    }
    double max_val = 0, min_val = 0;
    cv::Point max_loc, min_loc;
    cv::minMaxLoc(angle_hist, &min_val, &max_val, &min_loc, &max_loc);
    cout<<"angle_hist:"<<angle_hist<<endl;
    if (max_val > boxes.size()/4.0)
    {
        return angles[max_loc.y];
    } else
    {
        return -1000;
    }
}

//cal the rotation angle
bool test_box_wrap(std::vector<std::vector<cv::Point>> &boxes)
{
    if (boxes.size()<20)
    {
        return false;
    }
    for (int i = 0; i < boxes.size(); ++i) {
        cv::RotatedRect box = cv::minAreaRect(boxes[i]);

    }
}

int rotateImage(Mat img,Mat & imgout, int degree,int border_value,cv::Mat &A)
{
    if(img.empty())
        return 1;
    degree = degree;//warpAffine默认的旋转方向是逆时针，所以加负号表示转化为顺时针
    double angle = degree  * CV_PI / 180.; // 弧度
    double a = sin(angle), b = cos(angle);
    int width = img.cols;
    int height = img.rows;
    int width_rotate = int(width * fabs(b)+height * fabs(a));
    int height_rotate = int(height * fabs(b)+width * fabs(a));
    if(width_rotate<=20||height_rotate<=20)
    {
        width_rotate = 20;
        height_rotate = 20;
    }
    //旋转数组map
    // [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]
    // [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]
    float map[6];
    Mat map_matrix = Mat(2, 3, CV_32F, map);
    // 旋转中心
    CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);
    CvMat map_matrix2 = map_matrix;
    cv2DRotationMatrix(center, degree, 1.0, &map_matrix2);//计算二维旋转的仿射变换矩阵
    map[2] += (width_rotate - width) / 2;
    map[5] += (height_rotate - height) / 2;
    A = map_matrix.clone();
    int chnnel =img.channels();
    if(chnnel == 3)
        warpAffine(img, imgout, map_matrix, Size(width_rotate, height_rotate), 1, 0, Scalar(border_value,border_value,border_value));
    else
        warpAffine(img, imgout, map_matrix, Size(width_rotate, height_rotate), 1, 0, border_value);
    return 0;
}

// return 0 do not need wrap
int imo_wrap_image_perspective_reg(std::string & img_name, cv::Mat &dstImage_roi, cv::Mat &dstImage, cv::Mat H_roi, cv::Mat H)
{
    std::vector<std::vector<cv::Point>> boxes = get_box_post("http://192.168.6.242:43010/detect", (char*)img_name.data());
    double start = cv::getTickCount();
    cv::Mat img = cv::imread(img_name, 1);
    double scale = 1.0;
    int    max_size = 1280;
    cv::Mat img_resize;
    if (img.cols>img.rows)
    {
        if (img.cols>max_size)
        {
            scale = img.cols/float(max_size);
            cv::resize(img, img_resize, cv::Size(max_size, max_size*img.rows/float(img.cols)));
        } else
        {
            img_resize = img.clone();
            scale = 1.0;
        }
    } else{
        if (img.rows>max_size)
        {
            scale = img.rows/float(max_size);
            cv::resize(img, img_resize, cv::Size(max_size*img.cols/float(img.rows), max_size));
        } else
        {
            img_resize = img.clone();
            scale = 1.0;
        }
    }

    int angle = test_box(boxes);
    angle = 0;
    if (angle>=-90)
    {
        cv::Mat img_rot;
        cv::Mat A = cv::Mat::zeros(2, 3, CV_32F);;
        cv::Mat I22 = cv::Mat::eye(2, 2, CV_32F);
        I22.copyTo(A(cv::Rect(0, 0, 2, 2)));
        rotateImage(img, img_rot, angle, 0, A);
        cv::Rect roi = get_boundingRect(boxes, cv::Size(img_rot.cols, img_rot.rows), A);
        bool bSucceed = false;
        cv::Rect roi_resize(roi.x/scale, roi.y/scale, roi.width/scale, roi.height/scale);
        cv::Mat img_roi_resize = img_resize(roi_resize).clone();
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        bSucceed = wrapImagePerspective(img_roi_resize, dstImage_roi, H_roi, R, 30, true, true, true);
        cv::imshow("line", img_roi_resize);

        cout<<"wrap time:"<<1000*(cv::getTickCount()-start)/(cv::getTickFrequency())<<"ms"<<endl;
        if (bSucceed)
        {
            //cv::imshow("dstImage_roi", dstImage_roi);
            cv::Mat H0 = cv::Mat::eye(3, 3, CV_32F);
            H0 = H0/scale;
            H_roi = H0.clone();
            H = H_roi.clone();
            cv::Mat img_roi = img_rot(roi).clone();
            wrapImagePerspectiveH(img_roi, dstImage_roi, R, H_roi);
            wrapImagePerspectiveH(img_rot, dstImage, R, H);
            return 1;
        }
        else
        {
            H = cv::Mat::eye(3, 3, CV_32F);
            H_roi = cv::Mat::eye(3, 3, CV_32F);
            dstImage = img_rot.clone();
            dstImage_roi = img_rot.clone();
            return 0;
        }
    } else{
        dstImage = img.clone();
        dstImage_roi = img.clone();
        H = cv::Mat::eye(3, 3, CV_32F);
        H_roi = cv::Mat::eye(3, 3, CV_32F);
        return  0;
    }
}
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
using namespace std;
int main(int argc, char ** argv) {
    const char* path = "../test/incorrect/";

    std::string path0 = path;
    vector<string> files;
    DIR* pDir;
    struct dirent* ptr;

    struct stat s;
    lstat(path, &s);

    if(!S_ISDIR(s.st_mode)){
        return 0;
    }

    if(!(pDir = opendir(path))){
        return 0;
    }
    int i = 0;
    std::string subFile;
    while((ptr = readdir(pDir)) != 0){
        subFile = ptr -> d_name;
        if(subFile == "." || subFile == "..")
            continue;
        if(subFile.find_last_of(".jpg")==string::npos&&subFile.find_last_of(".png")==string::npos)
            continue;
        subFile = path + subFile;
        files.push_back(subFile);
    }
    closedir(pDir);

    //get_box_post("http://192.168.6.242:43010/detect", "img.jpg");

    for (int i = 0; i < files.size(); ++i) {
        cv::Mat img_wrap , img_wrap_roi;
        cv::Mat H, H_roi;
        std::string img_name = files[i];
        std::cout<<img_name<<endl;
        imo_wrap_image_perspective_reg(img_name, img_wrap_roi, img_wrap, H_roi, H);
        cv::Mat img_wrap_roi_resize, img_wrap_resize;
        cv::resize(img_wrap, img_wrap_resize, cv::Size(img_wrap.cols/2, img_wrap.rows/2));
        cv::resize(img_wrap_roi, img_wrap_roi_resize, cv::Size(img_wrap_roi.cols/2, img_wrap_roi.rows/2));
        cv::imshow("wrap_roi", img_wrap_roi_resize);
        cv::imshow("wrap", img_wrap_resize);
        cv::waitKey(0);
        cv::destroyAllWindows();

    }

    return 1;
}