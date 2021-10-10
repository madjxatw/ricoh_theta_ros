#ifndef EQUIRECTANGULAR2PERSPECECTIVE_EQUIREC2PERSPEC_H
#define EQUIRECTANGULAR2PERSPECECTIVE_EQUIREC2PERSPEC_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#ifdef OPENCV_VER_4
#include <opencv2/imgproc/imgproc_c.h>
#endif

class Equirec2Perspec
{
public:
  Equirec2Perspec();
  ~Equirec2Perspec();

  void setParams(const cv::Mat &inputImg, float FOV, int height, int width);

  void convert(const cv::Mat &inputImg, cv::Mat &outputImg, float FOV,
               float theta, float phi, int height, int width);

  void convert(const cv::Mat &inputImg, cv::Mat &outputImg,
               float theta, float phi);

private:
  void updateParams();
  float calcRadian(float degree) {return degree/180*PI_;}
  void equi2per(const cv::Mat &inputImage, cv::Mat &outputImage);

  int perHeight_, perWidth_;
  int equHeight_, equWidth_;
  float equCenterX_, equCenterY_;

  float theta_, phi_;
  float FOV_, wFOV_, hFOV_;
  float wAngle_, hAngle_;
  float wLen_, hLen_;
  float wInterval_, hInterval_;

  cv::Mat xMap_, yMap_, zMap_, dMap_;
  cv::Mat xyz_;
  float zAxisArray_[3] = {0.0f, 0.0f, 1.0f};
  cv::Mat zAxis_ = cv::Mat(cv::Size(3,1),CV_32F, zAxisArray_);

  bool isSetParam_;

  int radius_;
  float PI_;
};

#endif //EQUIRECTANGULAR2PERSPECECTIVE_EQUIREC2PERSPEC_H
