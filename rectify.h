#ifndef _AIMALL_SDK_INFRA_CORE_SDK_H_
#define _AIMALL_SDK_INFRA_CORE_SDK_H_


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int len;
	char * buf;
} imo_image;

/**
*dst内存由内部申请，需要主动释放
*/
int  imo_wrap_image_perspective(imo_image * srcImage, imo_image * dstImage);


int free_image(imo_image * image);



#ifdef __cplusplus
}
#endif

#endif //_AIMALL_SDK_INCLUDE_ENCTYPTION_DECODE_H_
