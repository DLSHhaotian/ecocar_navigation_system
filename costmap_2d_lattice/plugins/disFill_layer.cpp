
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <cstdlib>
#include <costmap_2d/disFill_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::disFillLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

disFillLayer::disFillLayer()
  : resolution_(0)
  , gridValue_max(250)
  , dsrv_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  disFill_access_ = new boost::recursive_mutex();
}

void disFillLayer::onInitialize()
{
    boost::unique_lock < boost::recursive_mutex > lock(*disFill_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;

    dynamic_reconfigure::Server<costmap_2d::disFillPluginConfig>::CallbackType cb = boost::bind(
        &disFillLayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::disFillPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }

  matchSize();
}

void disFillLayer::reconfigureCB(costmap_2d::disFillPluginConfig &config, uint32_t level)
{
  if (enabled_ != config.enabled) {
    enabled_ = config.enabled;
  }
  gridValue_max=(unsigned char)config.grid_value_max;
}

void disFillLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*disFill_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
}

void disFillLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
}


void disFillLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*disFill_access_);
  if (!enabled_)
    return;
  std::vector<CellIndex> freeCell;
  std::multimap<unsigned int,unsigned int> obsCell;
  std::multimap<unsigned int,unsigned int> obsCell_rotate;
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  double map_size_x=master_grid.getSizeInMetersX(), map_size_y=master_grid.getSizeInMetersY();
  //pattern is true if wide>height, false if wide<height
  //x:width y:height
  //The local map is set to change the size type with the change of the track
  //different traversal directions
  bool pattern= (map_size_x>map_size_y);
  if(!pattern) {
      for (int i = 0; i < size_y; ++i) {
          for (int j = 0; j < size_x; ++j) {/*
              if (master_array[i * size_x + j] == FREE_SPACE) {
                  freeCell.push_back(CellIndex(j, i));//(x,y)
              } else if (master_array[i * size_x + j] >= INSCRIBED_INFLATED_OBSTACLE) {
                  obsCell.insert(std::make_pair(i, j));//(key=y,value=x)
              }*/
              if (master_array[i * size_x + j] < INSCRIBED_INFLATED_OBSTACLE) {
                  freeCell.push_back(CellIndex(j, i));//(x,y)
              }
              else{
                  obsCell.insert(std::make_pair(i, j));//(key=y,value=x)
                  obsCell_rotate.insert(std::make_pair(j, i));//(key=x,value=y)
              }
          }
      }
  }
  else{
      for (int i = 0; i < size_y; ++i) {
          for (int j = 0; j < size_x; ++j) {
              if (master_array[i * size_x + j] == FREE_SPACE) {
                  freeCell.push_back(CellIndex(j, i));//(x,y)
              } else if (master_array[i * size_x + j] >= INSCRIBED_INFLATED_OBSTACLE) {
                  obsCell.insert(std::make_pair(j, i));//(key=x,value=y)
                  obsCell_rotate.insert(std::make_pair(i, j));//(key=y,value=x)
              }
          }
      }
  }
//Because the obstacle points are sparse, we need to fill the obstacle points
//ROS_INFO("START FILL");
  const int obs_num_col_road=18;
  const int obs_num_col_noise=6;
  const int road_width=9;
  double thresh_road_x=0.5*(size_x-road_width)+2;
  double thresh_road_y=0.5*(size_y-road_width)+2;
  std::vector<unsigned int> x_list_col;
  std::vector<unsigned int> y_list_col;
  if(!pattern){
      for(auto i_key=obsCell_rotate.begin();i_key!=obsCell_rotate.end();i_key=obsCell_rotate.upper_bound(i_key->first)){
          //ROS_INFO("%d",obsCell_rotate.count(i_key->first));
          if(obsCell_rotate.count(i_key->first)>obs_num_col_road&&(i_key->first<thresh_road_x||i_key->second>(size_x-thresh_road_x))){
              //ROS_INFO("FOUND COL TO FILL");
              std::pair<std::multimap<unsigned int,unsigned int>::iterator, std::multimap<unsigned int,unsigned int>::iterator> i_value = obsCell_rotate.equal_range(i_key->first);
              for (auto i = i_value.first; i != i_value.second; ++i)
              {
                  y_list_col.push_back(i->second);
              }
              int max_y=*(std::max_element(y_list_col.begin(),y_list_col.end()));
              int min_y=*(std::min_element(y_list_col.begin(),y_list_col.end()));
              //ROS_INFO("max: %d, min: %d",max_x,min_x);
              for(int i=min_y;i<=max_y;++i){
                  if(std::find(y_list_col.begin(),y_list_col.end(),i)==y_list_col.end()){
                      obsCell.insert(std::make_pair(i,i_key->first));//put the filled obstacle point into obscell
                      master_array[i * size_x + i_key->first]=LETHAL_OBSTACLE;
                      //ROS_INFO("FILL IN");
                  }
              }
              y_list_col.clear();
          }
      }
  }
  else{//for pattern1
      for(auto i_key=obsCell_rotate.begin();i_key!=obsCell_rotate.end();i_key=obsCell_rotate.upper_bound(i_key->first)){
          //ROS_INFO("%d",obsCell_rotate.count(i_key->first));
          if(obsCell_rotate.count(i_key->first)>obs_num_col_road&&(i_key->first<thresh_road_y||i_key->second>(size_y-thresh_road_y))){
              //ROS_INFO("FOUND COL TO FILL");
              std::pair<std::multimap<unsigned int,unsigned int>::iterator, std::multimap<unsigned int,unsigned int>::iterator> i_value = obsCell_rotate.equal_range(i_key->first);
              for (auto i = i_value.first; i != i_value.second; ++i)
              {
                  x_list_col.push_back(i->second);
              }
              int max_x=*(std::max_element(x_list_col.begin(),x_list_col.end()));
              int min_x=*(std::min_element(x_list_col.begin(),x_list_col.end()));
              //ROS_INFO("max: %d, min: %d",max_x,min_x);
              for(int i=min_x;i<=max_x;++i){
                  if(std::find(x_list_col.begin(),x_list_col.end(),i)==x_list_col.end()){
                      obsCell.insert(std::make_pair(i,i_key->first));//put the filled obstacle point into obscell
                      master_array[i_key->first * size_x + i]=LETHAL_OBSTACLE;
                      //ROS_INFO("FILL IN");
                  }
              }
              x_list_col.clear();
          }
      }
  }


  unsigned int x_free=freeCell.begin()->x_;
  unsigned int y_free=freeCell.begin()->y_;
  std::multimap<unsigned int,unsigned int>::iterator iter;
  bool flag_RoadPoint=true;
  if(!pattern){
      for(auto index=freeCell.begin();index!=freeCell.end();++index){
          x_free=index->x_;
          y_free=index->y_;
          flag_RoadPoint=haveTwoRoadSidePoints(obsCell,y_free,road_width/2);
          if(obsCell.count(y_free)!=0&&flag_RoadPoint){
              //ROS_INFO("just row");
              iter=obsCell.find(y_free);
              for(int k=0;k<obsCell.count(y_free);++k,++iter){//unsigned and signed, it is important
                  if(master_array[y_free * size_x + x_free]<(gridValue_max /(abs(static_cast<int>(x_free - iter->second))+1))){
                      master_array[y_free * size_x + x_free]=gridValue_max / (abs(static_cast<int>(x_free - iter->second))+1);
                  }
              }
          }
          else{//there is no obstacle in this row,but perhaps exits in the neighbour rows
              flag_RoadPoint=haveTwoRoadSidePoints(obsCell,y_free+1,road_width/2);
              if(obsCell.count(y_free+1)!=0&&flag_RoadPoint){
                  //ROS_INFO("just row+1");
                  iter=obsCell.find(y_free+1);
                  //master_array[y_free * size_x + x_free]=gridValue_max / abs(static_cast<int>(x_free - iter->second));
                  for(int k=0;k<obsCell.count(y_free+1);++k,++iter){
                      if(master_array[y_free * size_x + x_free]<(gridValue_max / (abs(static_cast<int>(x_free - iter->second))+1))){
                          master_array[y_free * size_x + x_free]=gridValue_max / (abs(static_cast<int>(x_free - iter->second))+1);
                      }
                  }
              }
              else{
                  flag_RoadPoint=haveTwoRoadSidePoints(obsCell,x_free-1,road_width/2);
                  if(obsCell.count(y_free-1)!=0&&flag_RoadPoint){
                      //ROS_INFO("just row-1");
                      iter=obsCell.find(y_free-1);
                      //master_array[y_free * size_x + x_free]=gridValue_max / abs(static_cast<int>(x_free - iter->second));
                      for(int k=0;k<obsCell.count(y_free-1);++k,++iter){
                          if(master_array[y_free * size_x + x_free]<(gridValue_max / (abs(static_cast<int>(x_free - iter->second))+1))){
                              master_array[y_free * size_x + x_free]=gridValue_max/ (abs(static_cast<int>(x_free - iter->second))+1);
                          }
                      }
                  }
              }
          }
      }
  }
  else{// map size changed
      for(auto index=freeCell.begin();index!=freeCell.end();++index){
          x_free=index->x_;
          y_free=index->y_;
          flag_RoadPoint=haveTwoRoadSidePoints(obsCell,x_free,road_width/2);
          if(obsCell.count(x_free)!=0&&flag_RoadPoint){
              //ROS_INFO("just row pattern2");
              iter=obsCell.find(x_free);
              for(int k=0;k<obsCell.count(x_free);++k,++iter){
                  //ROS_INFO("cellgrid: %u",gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1));
                  if(master_array[y_free * size_x + x_free]<(gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1))){
                      master_array[y_free * size_x + x_free]=gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1);
                  }
              }
          }
          else{
              flag_RoadPoint=haveTwoRoadSidePoints(obsCell,x_free+1,road_width/2);
              if(obsCell.count(x_free+1)!=0&&flag_RoadPoint){
                   //ROS_INFO("just +1 pattern2");
                  iter=obsCell.find(x_free+1);
                  for(int k=0;k<obsCell.count(x_free+1);++k,++iter){
                       //ROS_INFO("cellgrid: %u",gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1));
                      if(master_array[y_free * size_x + x_free]<(gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1))){
                          master_array[y_free * size_x + x_free]=gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1);
                      }
                  }
              }
              else{
                  flag_RoadPoint=haveTwoRoadSidePoints(obsCell,x_free-1,road_width/2);
                  if(obsCell.count(x_free-1)!=0&&flag_RoadPoint){
                       //ROS_INFO("just -1 pattern2");
                      iter=obsCell.find(x_free-1);
                      for(int k=0;k<obsCell.count(x_free-1);++k,++iter){
                           //ROS_INFO("cellgrid: %u",gridValue_max /static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1));
                          if(master_array[y_free * size_x + x_free]<(gridValue_max / static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1))){
                              master_array[y_free * size_x + x_free]=gridValue_max / static_cast<unsigned char>(abs(static_cast<int>(y_free - iter->second))+1);
                          }
                      }
                  }
              }
          }
      }
  }

//ROS_INFO("DISFILL LAYER FINISHED");
}
bool disFillLayer::haveTwoRoadSidePoints(std::multimap<unsigned int, unsigned int>& obsCell, unsigned int key, int size) {
    if(obsCell.count(key)==0||obsCell.count(key)==1){
        return false;
    }
    std::pair<std::multimap<unsigned int, unsigned int>::iterator, std::multimap<unsigned int, unsigned int>::iterator> i_value = obsCell.equal_range(key);
    std::vector<int> value_list;
    for (auto i = i_value.first; i != i_value.second; ++i) {
        value_list.push_back(i->second);
    }
    int max_x=*(std::max_element(value_list.begin(),value_list.end()));
    int min_x=*(std::min_element(value_list.begin(),value_list.end()));
    return  (size<(max_x-min_x));
}
}  // namespace costmap_2d
