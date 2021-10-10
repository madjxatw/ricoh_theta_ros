#include "equirec2perspec/equirec2perspec.h"

Equirec2Perspec::Equirec2Perspec()
{
  radius_ = 18;
  PI_ = float(4.0 * std::atan(1.0));;
  isSetParam_ = false;
}

Equirec2Perspec::~Equirec2Perspec() {}

void Equirec2Perspec::setParams(const cv::Mat &inputImg,
                                float FOV, int height, int width)
{
  equHeight_ = inputImg.rows;
  equWidth_  = inputImg.cols;

  FOV_ = FOV;
  perHeight_ = height;
  perWidth_ = width;

  updateParams();

  isSetParam_ = true;
}

void Equirec2Perspec::convert(const cv::Mat &inputImage,
                                      cv::Mat &outputImage,
                                      float FOV, float theta, float phi,
                                      int height, int width)
{
  setParams(inputImage, FOV, height, width);
  theta_ = theta;
  phi_ = phi;
  equi2per(inputImage, outputImage);
}

void Equirec2Perspec::convert(const cv::Mat &inputImage,
                                      cv::Mat &outputImage,
                                      float theta, float phi)
{
  theta_ = theta;
  phi_ = phi;
  if (isSetParam_) {
    equi2per(inputImage, outputImage);
  }
  else {
    std::cout << "Please set parameters before call convert()\n";
    exit(1);
  }
}

void Equirec2Perspec::updateParams() {
  equCenterX_ = (equWidth_-1)  / 2.0f;
  equCenterY_ = (equHeight_-1) / 2.0f;

  wFOV_ = FOV_;
  hFOV_ = static_cast<float>(perHeight_) / perWidth_ * wFOV_;

  wAngle_ = (180 - wFOV_) / 2.0f;
  hAngle_ = (180 - hFOV_) / 2.0f;
  wLen_ = 2 * radius_ * sin(calcRadian(wFOV_/2)) / sin(calcRadian(wAngle_));
  hLen_ = 2 * radius_ * sin(calcRadian(hFOV_/2)) / sin(calcRadian(hAngle_));
  wInterval_ = wLen_ / (perWidth_ -1);
  hInterval_ = hLen_ / (perHeight_-1);

  xMap_ = cv::Mat(cv::Size(perWidth_, perHeight_), CV_32F);
  xMap_ = radius_;

  const int arraySize = perHeight_ * perWidth_;

  float yArray[arraySize];
  for (int ai=0; ai<arraySize; ai++)
    yArray[ai] = static_cast<float>(ai%perWidth_);

  yMap_ = cv::Mat(cv::Size(perWidth_, perHeight_), CV_32F, yArray);
  float cX = (perWidth_-1) / 2.0f;
  yMap_ -= cX;
  yMap_ *= wInterval_;

  float zArray[arraySize];

  for (int ai = 0; ai < arraySize; ai++)
    zArray[ai] = float(ai%perHeight_);

  zMap_ = cv::Mat(cv::Size(perHeight_, perWidth_), CV_32F, zArray);
  float cY = (perHeight_-1) / 2.0f;
  zMap_ -= cY;
  zMap_ *= -hInterval_;
  zMap_ = zMap_.t();
  dMap_ = cv::Mat(cv::Size(perWidth_, perHeight_), CV_32F);

  cv::sqrt(xMap_.mul(xMap_)+yMap_.mul(yMap_)+zMap_.mul(zMap_), dMap_);

  cv::Mat xyzX = radius_ * (xMap_/dMap_);
  cv::Mat xyzY = radius_ * (yMap_/dMap_);
  cv::Mat xyzZ = radius_ * (zMap_/dMap_);
  cv::Mat xy;

  cv::vconcat(xyzX.reshape(1,perHeight_), xyzY.reshape(1,perHeight_), xy);
  cv::vconcat(xy, xyzZ.reshape(1,perHeight_), xyz_);
}

void Equirec2Perspec::equi2per(const cv::Mat &inputImage, cv::Mat &outputImage)
{
  cv::Mat R1(cv::Size(3,3),CV_32F), R2(cv::Size(3,3), CV_32F);
  cv::Rodrigues(zAxis_ * calcRadian(theta_), R1);
  cv::Rodrigues(R1.col(1).t() * calcRadian(-phi_), R2);

  xyz_ = R1 * xyz_.reshape(1,3);
  xyz_ = (R2 * xyz_).t();

  cv::Mat lat = xyz_.col(2).t() / radius_;
  int liCols = lat.cols;

  for (int li = 0; li < liCols; li++)
    lat.at<float>(li) = asin(lat.at<float>(li));

  cv::Mat lon = cv::Mat::zeros(cv::Size(perWidth_*perHeight_,1), CV_32F);
  cv::Mat thetaMat = xyz_.col(1).t()/xyz_.col(0).t();
  int thetaMatCols = thetaMat.cols;

  for (int li = 0; li < thetaMatCols; li++)
    thetaMat.at<float>(li) = atan(thetaMat.at<float>(li));

  cv::Mat idx1 = xyz_.col(0).t();
  int idx1Cols = idx1.cols;

  for (int li = 0; li < idx1Cols; li++) {
    if (idx1.at<float>(li) > 0)
      idx1.at<float>(li) = 1;
    else
      idx1.at<float>(li) = 0;
  }

  cv::Mat idx2 = xyz_.col(1).t();
  int idx2Cols = idx1.cols;

  for (int li = 0; li < idx2Cols; li++) {
    if (idx2.at<float>(li) > 0)
      idx2.at<float>(li) = 1;
    else
      idx2.at<float>(li) = 0;
  }

  cv::Mat idx3 = ((1-idx1).mul(idx2));
  cv::Mat idx4 = ((1-idx1).mul(1-idx2));
  int idx3Cols = idx3.cols;
  int idx4Cols = idx4.cols;

  for (int i = 0; i < idx1Cols; i++) {
    if (idx1.at<float>(i) > 0)
      lon.at<float>(i) = thetaMat.at<float>(i);
  }

  for(int i=0; i < idx3Cols; i++){
    if(idx3.at<float>(i)>0){
      lon.at<float>(i) = thetaMat.at<float>(i) + PI_;
    }
  }

  for (int i = 0; i < idx4Cols; i++) {
    if (idx4.at<float>(i) > 0)
      lon.at<float>(i) = thetaMat.at<float>(i) - PI_;
  }

  lon =  lon.reshape(1, perHeight_) / PI_ * 180;
  lat = -lat.reshape(1, perHeight_) / PI_ * 180;
  lon = lon / 180 * equCenterX_ + equCenterX_;
  lat = lat /  90 * equCenterY_ + equCenterY_;

  outputImage = cv::Mat(cv::Size(perWidth_,perHeight_), inputImage.type());
  cv::remap(inputImage, outputImage, lon, lat, CV_INTER_CUBIC, CV_HAL_BORDER_WRAP);
}
