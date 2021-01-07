#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "darknet.h"
#include "c2cpp.h"

//for cvRound error
int cvRound(double value) {return(ceil(value));}

#ifdef __cplusplus
extern "C" {
#endif

void InitYOLO(const char* prefix, const char* datacfg, const char* cfg, const char* weights, const char* m_names, float thresh, bool enable_debug) {
#ifndef GPU
  gpu_index = -1;
  printf("GPU  not defined~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~```");
#else
  printf("GPU defined-------------------------------------------------");
  if(gpu_index >= 0){
    cuda_set_device(gpu_index);
  }
#endif
  list *options = read_data_cfg(datacfg);
  int classes = option_find_int(options, "classes", 20);
  char *name_list_path = option_find_str(options, "names", m_names); 
  char *name_list = prefix;
  strcat(name_list, name_list_path);
  char **names = get_labels(name_list);
  demo_init(cfg, weights, thresh, names, classes, enable_debug);
}

void RunYOLO(bool enable, IplImage* next_img, IplImage* result_img, int *object_num){
  demo(enable, next_img, result_img, object_num);
}

#ifdef __cplusplus
};
#endif