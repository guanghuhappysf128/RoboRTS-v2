#ifndef MODULES_DRIVERS_CAMERA_MERCURE_DRIVER_H
#define MODULES_DRIVERS_CAMERA_MERCURE_DRIVER_H

#include "opencv2/opencv.hpp"
#include "../camera_param.h"
#include "image_proc.h"
#include "gx_api.h"
#include "fps.h"
#include "../camera_base.h"

#include "alg_factory/algorithm_factory.h"


namespace roborts_camera {

DX_PIXEL_COLOR_FILTER  __TransferFormat(int32_t nPixelFormat)
{
  DX_PIXEL_COLOR_FILTER nBayerType;
  switch(nPixelFormat)
  {
    case GX_PIXEL_FORMAT_BAYER_GR8:
      nBayerType = BAYERGR;
      break;
    case GX_PIXEL_FORMAT_BAYER_RG8:
      nBayerType = BAYERRG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GB8:
      nBayerType = BAYERGB;
      break;
    case GX_PIXEL_FORMAT_BAYER_BG8:
      nBayerType = BAYERBG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GR10:
      nBayerType = BAYERGR;
      break;
    case GX_PIXEL_FORMAT_BAYER_RG10:
      nBayerType = BAYERRG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GB10:
      nBayerType = BAYERGB;
      break;
    case GX_PIXEL_FORMAT_BAYER_BG10:
      nBayerType = BAYERBG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GR12:
      nBayerType = BAYERGR;
      break;
    case GX_PIXEL_FORMAT_BAYER_RG12:
      nBayerType = BAYERRG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GB12:
      nBayerType = BAYERGB;
      break;
    case GX_PIXEL_FORMAT_BAYER_BG12:
      nBayerType = BAYERBG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GR16:
      nBayerType = BAYERGR;
      break;
    case GX_PIXEL_FORMAT_BAYER_RG16:
      nBayerType = BAYERRG;
      break;
    case GX_PIXEL_FORMAT_BAYER_GB16:
      nBayerType = BAYERGB;
      break;
    case GX_PIXEL_FORMAT_BAYER_BG16:
      nBayerType = BAYERBG;
      break;
    default:
      nBayerType = NONE;
      break;
  }
  return nBayerType;
}

class MercureDriver: public CameraBase {
 public:
  explicit MercureDriver(CameraInfo camera_info);
  void LoadParam() ;
  void Init();
  void StartReadCamera(cv::Mat &img) override;
  void StopReadCamera();
  void SaveRawFile(void *pImgBuffer, size_t nWidth, size_t nHeight);
  ~MercureDriver();
 private:
  CameraParam camera_param_;
  cv::VideoWriter video_writer_;
  uint8_t* pu8_image_buf_;
  GX_DEV_HANDLE device_;
  GX_STATUS status_;
  // to accommendate api change
  // GX_FRAME_DATA *frame_data_;
  PGX_FRAME_BUFFER frame_data_;
  unsigned int frame_count_;
  CFps fps_;
  double frame_rate_;
  bool camera_initialized_;
  uint64_t lost_count_;
  uint64_t last_frame_id_;
};

// potentially need to add camera info
roborts_common::REGISTER_ALGORITHM(CameraBase, "mercure", MercureDriver, CameraInfo);

} //namespace roborts_camera


#endif //MODULES_DRIVERS_CAMERA_MERCURE_DRIVER_H
