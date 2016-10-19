#include <aslam_demo/aslam/aslam.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace aslam {
AslamBase::AslamBase() {

}

void AslamBase::getFrontierCells(nav_msgs::OccupancyGrid& occupancy_grid,std::vector<std::pair<int,int> >& frontier_indices) {

  size_t height = occupancy_grid.info.height;
  size_t width = occupancy_grid.info.width;
  size_t row = 0, col = 0;
  auto data = occupancy_grid.data;
  std::vector<std::pair<int,int> > indices_offset;

  indices_offset.push_back(std::make_pair(-1,-1));
  indices_offset.push_back(std::make_pair(-1,0));
  indices_offset.push_back(std::make_pair(-1,1));
  indices_offset.push_back(std::make_pair(0,-1));
  indices_offset.push_back(std::make_pair(0,1));
  indices_offset.push_back(std::make_pair(1,-1));
  indices_offset.push_back(std::make_pair(1,0));
  indices_offset.push_back(std::make_pair(1,1));

  for (size_t i = 0;i < height*width;i++) {
    int current = data[i];
    if(!(current < 127)) continue;
    row = i/width;
    col = i%width;
    bool occupied_present = false,unknown_present = false;
    for(auto const iter: indices_offset) {
      size_t new_row = (row + iter.first);
      size_t new_col = (col + iter.second);
      if (new_row < 0 || new_row >= height || new_col < 0 || new_col >= width) continue;

      int value = data[height*new_row + new_col];
      if (value > 127) occupied_present = true;
      if (value == 127) unknown_present = true;

    }
    if(!occupied_present && unknown_present) frontier_indices.push_back(std::make_pair(row,col));
  }
}

void AslamBase::findFrontierClusters(std::vector<std::pair<int,int> >& frontier_indices) {
  int numPoints = frontier_indices.size();
  cv::Mat points(numPoints,2,CV_32FC2),labels,centers;
  int cluster_count = 3;
  int index = 0;
  for(auto const iter: frontier_indices) {
    points.at<float>(index,0) = (float)(iter.first);
    points.at<float>(index,1) = (float)(iter.second);
    index++;
  }

  ROS_INFO_STREAM("Point"<<points.size());
  double temp = cv::kmeans(points, cluster_count, labels,cv::TermCriteria(cv::TermCriteria::COUNT, 100000,0.01),30, cv::KMEANS_RANDOM_CENTERS , centers);
  ROS_INFO_STREAM("Temp"<<temp);
  ROS_INFO_STREAM("Centers"<<labels.size());
  for(int row = 0;row < numPoints;row++)
      ROS_INFO_STREAM("Points:"<<labels.at<int>(row));
}

AslamBase::~AslamBase() {

}
};
