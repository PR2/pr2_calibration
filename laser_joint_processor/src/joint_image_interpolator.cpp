/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//! \author Vijay Pradeep / vpradeep@willowgarage.com

#include <opencv2/imgproc/imgproc.hpp>

#include <laser_joint_processor/joint_image_interpolator.h>
#include <ros/console.h>

using namespace laser_joint_processor;
using namespace std;

bool JointImageInterpolator::interp(const std::vector <geometry_msgs::Point>& points,
                                    cv::Mat_<cv::Vec2f> image, std::vector<float>& positions, std::vector<float>& velocities)
{
  const unsigned int N = points.size();

  // Allocate Maps
  cv::Mat_<float> map_x_mat(N, 1);
  cv::Mat_<float> map_y_mat(N, 1);

  // Set up maps
  for (unsigned int i=0; i<N; i++)
  {
    map_x_mat(i) = points[i].x;
    map_y_mat(i) = points[i].y;
  }

  // Allocate Destination Image
  cv::Mat_<cv::Vec2f> dest_mat(N, 1);

  // Perform the OpenCV interpolation
  cv::remap(image, dest_mat, map_x_mat, map_y_mat, cv::INTER_LINEAR, cv::BORDER_WRAP);

  // Copy results into output vectors
  positions.resize(N);
  velocities.resize(N);
  for (unsigned int i=0; i<N; i++)
  {
    positions[i]  = dest_mat(i)[0];
    velocities[i] = dest_mat(i)[1];
  }

  return true;
}


bool laser_joint_processor::interpSnapshot(const std::vector <geometry_msgs::Point>& points,
                                           const calibration_msgs::DenseLaserSnapshot& snapshot,
                                           std::vector<float>& angles,
                                           std::vector<float>& ranges)
{
  const unsigned int N = points.size();

  // Check to make sure points are in range
  for (unsigned int i=0; i<N; i++)
  {
    if (points[i].x < 0  ||
        points[i].x > snapshot.readings_per_scan-1 ||
        points[i].y < 0 ||
        points[i].y > snapshot.num_scans-1 )
    {
      return false;
    }
  }

  // Set up input image
  cv::Mat_<float> range_image(snapshot.num_scans, snapshot.readings_per_scan, const_cast<float*>(&snapshot.ranges[0]));

  // Allocate Maps
  cv::Mat_<float> map_x_mat(N, 1);
  cv::Mat_<float> map_y_mat(N, 1);

  // Set up maps
  for (unsigned int i=0; i<N; i++)
  {
    map_x_mat(i) = points[i].x;
    map_y_mat(i) = points[i].y;
  } 

  // Allocate Destination Image
  ranges.resize(N);
  cv::Mat_<float> ranges_mat(N, 1, &ranges[0]);

  // Perform the OpenCV interpolation
  cv::remap(range_image, ranges_mat, map_x_mat, map_y_mat, cv::INTER_LINEAR, cv::BORDER_WRAP);

  // Do angle interp manually
  angles.resize(N);
  for (unsigned int i=0; i<N; i++)
    angles[i] = snapshot.angle_min +  points[i].x*snapshot.angle_increment;

  return true;
}





