// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdexcept>
#include <frame.h>
#include <feature.h>
#include <point.h>
#include <boost/bind.hpp>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/performance_monitor.h>
// #include <fast/fast.h>

namespace lidar_selection {

int Frame::frame_counter_ = 0; 

Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img) :
    id_(frame_counter_++), 
    cam_(cam), 
    key_pts_(5), 
    is_keyframe_(false)
{
  initFrame(img);
}

Frame::~Frame()
{// 遍历了fts_容器中的所有元素，并对每个元素调用了一个lambda函数，该lambda函数将每个元素重置为默认状态
  std::for_each(fts_.begin(), fts_.end(), [&](FeaturePtr i){i.reset();});
}

void Frame::initFrame(const cv::Mat& img)
{
  // check image
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

  // Set keypoints to nullptr
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](FeaturePtr ftr){ ftr=nullptr; });
  
  ImgPyr ().swap(img_pyr_);
  img_pyr_.push_back(img);
  // Build Image Pyramid
  // frame_utils::createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
  // frame_utils::createImgPyramid(img, 5, img_pyr_); 
}

void Frame::setKeyframe()
{
  is_keyframe_ = true;
  setKeyPoints();
}

void Frame::addFeature(FeaturePtr ftr)
{
  fts_.push_back(ftr);
}

void Frame::setKeyPoints()
{
  for(size_t i = 0; i < 5; ++i)
        //   五个特征和关联的 3D 点，用于检测两个帧是否具有重叠的视野
        //  key_pts_：  Five features and associated 3D points 
        // which are used to detect if two frames have overlapping field of view.
    if(key_pts_[i] != nullptr)
      if(key_pts_[i]->point == nullptr)
        key_pts_[i] = nullptr;
    // 遍历fts_，检查当前元素指向的指针是否不为空，如果不为空，则调用checkKeyPoints函数对该元素进行处理
  std::for_each(fts_.begin(), fts_.end(), [&](FeaturePtr ftr){ if(ftr->point != nullptr) checkKeyPoints(ftr); });
}

void Frame::checkKeyPoints(FeaturePtr ftr)
{
  const int cu = cam_->width()/2;
  const int cv = cam_->height()/2;

  // center pixel
  if(key_pts_[0] == nullptr)
    key_pts_[0] = ftr;
// 传进来的特征点坐标到坐标轴最大距离小于 key_pts_[0] 的，则 key_pts_去当前传进来的点
  else if(std::max(std::fabs(ftr->px[0]-cu), std::fabs(ftr->px[1]-cv))
        < std::max(std::fabs(key_pts_[0]->px[0]-cu), std::fabs(key_pts_[0]->px[1]-cv)))
    key_pts_[0] = ftr; // 会动

  if(ftr->px[0] >= cu && ftr->px[1] >= cv)// 2
  {
    if(key_pts_[1] == nullptr)
      key_pts_[1] = ftr; // 不动 （原来）
    // PROBLEM_ME
    // 目的是什么呢，也不是找距离中心最小的点，而是找面积最大的点
    //   新进来的点围城面积大于 原来点围城面积 则将新传进来的点更新为  不动（原来）
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[1]->px[0]-cu) * (key_pts_[1]->px[1]-cv))
      key_pts_[1] = ftr;
  }

  if(ftr->px[0] >= cu && ftr->px[1] < cv) // 4
  {
    if(key_pts_[2] == nullptr)
      key_pts_[2] = ftr;
    // else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
    else if((ftr->px[0]-cu) * (cv-ftr->px[1])
          // > (key_pts_[2]->px[0]-cu) * (key_pts_[2]->px[1]-cv))
          > (key_pts_[2]->px[0]-cu) * (cv-key_pts_[2]->px[1]))
      key_pts_[2] = ftr;
  }

  if(ftr->px[0] < cu && ftr->px[1] < cv) //3
  {
    if(key_pts_[3] == nullptr)
      key_pts_[3] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[3]->px[0]-cu) * (key_pts_[3]->px[1]-cv))
      key_pts_[3] = ftr;
  }

  if(ftr->px[0] < cu && ftr->px[1] >= cv)   //1
  // if(ftr->px[0] < cv && ftr->px[1] >= cv)
  {
    if(key_pts_[4] == nullptr)
      key_pts_[4] = ftr;

    else if(cu-(ftr->px[0]) * (ftr->px[1]-cv) 
          > (cu-key_pts_[4]->px[0]) * (key_pts_[4]->px[1]-cv))      
      key_pts_[4] = ftr;
  }
}

void Frame::removeKeyPoint(FeaturePtr ftr)
{
  bool found = false;
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](FeaturePtr& i){
    if(i == ftr) {
      i = nullptr;
      found = true;
    }
  });
  if(found)
    setKeyPoints();
}

bool Frame::isVisible(const Vector3d& xyz_w) const
{
  Vector3d xyz_f = T_f_w_*xyz_w;

  if(xyz_f.z() < 0.0)
    return false; // point is behind the camera
  Vector2d px = f2c(xyz_f);

  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;
  return false;
}

/// Utility functions for the Frame class
namespace frame_utils {

void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;

  for(int i=1; i<n_levels; ++i)
  {//对每一层图像进行半采样操作，生成下一层图像
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);//CV_8U   0
    vk::halfSample(pyr[i-1], pyr[i]);
  }
}

//返回场景的深度信息
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  vector<double> depth_vec;
  depth_vec.reserve(frame.fts_.size());//为depth_vec容器预留足够的空间
  depth_min = std::numeric_limits<double>::max(); //获取特定数值类型的最大或最小值。
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)
  {
    if((*it)->point != nullptr) 
    {
      const double z = frame.w2f((*it)->point->pos_).z();
      depth_vec.push_back(z);
      depth_min = fmin(z, depth_min);
    }
  }
  if(depth_vec.empty())
  {
    cout<<"Cannot set scene depth. Frame has no point-observations!"<<endl;
    return false;
  }
  depth_mean = vk::getMedian(depth_vec);
  return true;
}

} // namespace frame_utils
} // namespace lidar_selection
