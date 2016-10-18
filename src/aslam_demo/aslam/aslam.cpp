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
    if(!(current < 50 && current != -1)) continue;
    row = current/width;
    col = current%width;
    bool occupied_present = false,unknown_present = false;
    for(auto const iter: indices_offset) {
      size_t new_row = (row + iter.first);
      size_t new_col = (col + iter.second);
      if (new_row < 0 || new_row >= height || new_col < 0 || new_col >= width) continue;

      int value = data[height*new_row + new_col];
      if (value > 50) occupied_present = false;
      if (value == -1 ) unknown_present = true;

    }
    if(!occupied_present && unknown_present) frontier_indices.push_back(std::make_pair(row,col));
  }
}

void AslamBase::findFrontierClusters(std::vector<std::pair<int,int> >& frontier_indices) {
  cv::Mat points(frontier_indices.size(),1);
  for(auto const iter: frontier_indices) {
  }

}

};
