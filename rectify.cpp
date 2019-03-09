#include "rectify.h"
#include "opencv2/opencv.hpp"
#include "wrapImageRef.h"

using namespace cv;

int  imo_wrap_image_perspective(imo_image * srcImage, imo_image * dstImage){
	if(srcImage == nullptr || dstImage == nullptr 
		|| srcImage->buf == nullptr || dstImage->buf == nullptr) {
		return -1;
	}


	std::vector<uchar> tmp;
	tmp.resize(srcImage->len);

	memcpy(tmp.data(), srcImage->buf, srcImage->len);

	cv::Mat input_mat = cv::imdecode(tmp, CV_LOAD_IMAGE_COLOR);

//	cv::imshow("test", input_mat);
	//cv::waitKey(-1);

	if(input_mat.empty()) {
	    return -1;
	}

	cv::Mat image_wrapped ;
	cv::Mat H;
	wrapImagePerspective(input_mat, input_mat, H, 20.0, true, true);

//	cv::imshow("test2", image_wrapped);
//	cv::waitKey(-1);

	std::vector<uchar> tmp_output;

	cv::imencode(".jpg",input_mat, tmp_output );


	dstImage->len = tmp_output.size();
	dstImage->buf = new char[dstImage->len];

	memcpy(dstImage->buf, tmp_output.data(), tmp_output.size());

	return 0;


}

int free_image(imo_image * image){
    if(image == nullptr || image->buf == nullptr) {
        return -1;
    }

    delete [] image->buf;

    return 0;
}
