#ifndef RRTS_C2CPP_H
#define RRTS_C2CPP_H
#include <opencv2/highgui/highgui_c.h>
#ifdef __cplusplus
extern "C" {
#endif
void InitYOLO(const char* prefix, const char* datacfg, const char* cfg, const char* weights, const char* m_names, float thresh, bool enable_debug);
void RunYOLO(bool enable, IplImage* next_img, IplImage* result_img, int *object_num);
#ifdef __cplusplus
};
#endif

#endif //RRTS_C2CPP_H