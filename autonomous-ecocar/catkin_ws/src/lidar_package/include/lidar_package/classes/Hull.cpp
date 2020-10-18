#include "Hull.h"

float Hull::distToLineSegmentOnZPlane(pcl::PointXYZ * p, pcl::PointXYZ * l_1, pcl::PointXYZ * l_2){
  const float l_x = l_2->x - l_1->x;
  const float l_y = l_2->y - l_1->y;

  const float l_1_to_l_2_sq = l_x * l_x + l_y * l_y;
  const float p_to_l_1_sq = (p->x - l_1->x) * (p->x - l_1->x) + (p->y - l_1->y) * (p->y - l_1->y);
  if(l_1_to_l_2_sq == 0.0) return sqrt(p_to_l_1_sq);
  const float p_to_l_2_sq = (p->x - l_2->x) * (p->x - l_2->x) + (p->y - l_2->y) * (p->y - l_2->y);
  if(l_1_to_l_2_sq >= p_to_l_1_sq && l_1_to_l_2_sq >= p_to_l_2_sq){
    return fabs(l_y * p->x - l_x * p->y + l_2->x * l_1->y - l_2->y * l_1->x)/sqrt(l_1_to_l_2_sq);
  }else if(p_to_l_1_sq >= p_to_l_2_sq){
      return sqrt(p_to_l_2_sq);
  }else{
      return sqrt(p_to_l_1_sq);
  }
}
float Hull::distToLineSegmentOnZPlaneSqr(pcl::PointXYZ * p, pcl::PointXYZ * l_1, pcl::PointXYZ * l_2){
  const float l_x = l_2->x - l_1->x;
  const float l_y = l_2->y - l_1->y;

  const float l_1_to_l_2_sq = l_x * l_x + l_y * l_y;
  const float p_to_l_1_sq = (p->x - l_1->x) * (p->x - l_1->x) + (p->y - l_1->y) * (p->y - l_1->y);
  if(l_1_to_l_2_sq == 0.0) return (p_to_l_1_sq);
  const float p_to_l_2_sq = (p->x - l_2->x) * (p->x - l_2->x) + (p->y - l_2->y) * (p->y - l_2->y);
  if(l_1_to_l_2_sq >= p_to_l_1_sq && l_1_to_l_2_sq >= p_to_l_2_sq){
    const float temp = l_y * p->x - l_x * p->y + l_2->x * l_1->y - l_2->y * l_1->x;
    return temp*temp/l_1_to_l_2_sq;
  }else if(p_to_l_1_sq >= p_to_l_2_sq){
      return (p_to_l_2_sq);
  }else{
      return (p_to_l_1_sq);
  }
}

bool Hull::doSegmentsCross(pcl::PointXYZ * p1, pcl::PointXYZ * p2, pcl::PointXYZ * p3, pcl::PointXYZ * p4){
  // Seg 1: p1 -> p2
  // Seg 2: p3 -> p4
  // On the Z plane
  float g1 = (p1->x - p3->x)*(p4->y - p3->y) - (p1->y - p3->y)*(p4->x - p3->x);
  float g2 = (p2->x - p3->x)*(p4->y - p3->y) - (p2->y - p3->y)*(p4->x - p3->x);
  float f3 = (p3->x - p1->x)*(p2->y - p1->y) - (p3->y - p1->y)*(p2->x - p1->x);
  float f4 = (p4->x - p1->x)*(p2->y - p1->y) - (p4->y - p1->y)*(p2->x - p1->x);
  return (g1*g2 < 0 && f3*f4 < 0);
}


Hull::Hull(){
};

Hull::Hull(pcl::PointCloud<pcl::PointXYZ>::Ptr in_vertices, bool make_hull, Extreme in_extreme, ros::Time in_timestamp){
  // Make Hull using either an aldready done hull, or make it from points on plane
  timestamp = in_timestamp;
  // extreme
  if(make_hull){
    vertices = makeHull(in_vertices);
  }else{
    vertices = in_vertices;
  }
  if(in_extreme.isSet()) extreme = in_extreme;
  else updateExtreme();
};
Hull::Hull(ptr other_hull){ // Makes Copy
  timestamp = other_hull->timestamp;
  extreme = other_hull->extreme;
  vertices = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  * vertices = * other_hull->vertices;
};
void Hull::updateExtreme(){
  extreme.makeExtreme(vertices);
};
Hull Hull::copyHull(){ // Makes Copy (TODO: not properly tested)
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_vertices (new pcl::PointCloud<pcl::PointXYZ>);
  * new_vertices = * vertices;
  return Hull(new_vertices, false, extreme, timestamp);
};
int Hull::size(){
  return (int) vertices->size();
};
void Hull::clear(){
  vertices->clear();
  extreme.clear();
};
pcl::PointCloud<pcl::PointXYZ>::Ptr Hull::makeHull(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_projection_cloud){
  if(USE_OPT_SWINGING_ARM){
    return Hull::newSwingingArmAlgorithm(planar_projection_cloud, SWINGING_ARM_LENGT);
  }else{
    const size_t size = planar_projection_cloud->size();
    if(size > 3){
      if(CONCAVE_HULL_ALPHA == 0){ // Make Convex
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud (planar_projection_cloud);
        chull.reconstruct (*cloud_hull); 
        return cloud_hull;
      }else{ // Make Concave
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConcaveHull<pcl::PointXYZ> chull;
        chull.setAlpha (CONCAVE_HULL_ALPHA);
        chull.setInputCloud (planar_projection_cloud);
        chull.reconstruct (*cloud_hull); 
        return cloud_hull;
      }
    }else if(size > 0){
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      for (size_t i = 0; i < planar_projection_cloud->size(); i++)
      {
        cloud_hull->push_back(planar_projection_cloud->at(i));
      }
      return cloud_hull;
    } else{
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      ROS_WARN("Making hull of 0 points!"); // TODO: NEEDS TO RETURN SOMETHING!
      return cloud_hull;
    }
  }
};
bool Hull::compareToHull(Hull::ptr other_hull){
    float max_dist = COMPARE_HULL_MAX_DISTANCE;

    if(extreme.isNear(&other_hull->extreme, max_dist)){
      size_t this_hull_size = this->size();
      size_t other_hull_size = other_hull->size();
      size_t smallest_hull_size = (this_hull_size < other_hull_size) ? this_hull_size : other_hull_size;
      size_t min_fits = (smallest_hull_size < COMPARE_HULL_MIN_FITS) ? smallest_hull_size : COMPARE_HULL_MIN_FITS;
      float max_dist_sqr = max_dist * max_dist;
      size_t fits = 0;

      // Fit current Vertices to Lines on other hull
      for (size_t i = 0; i < this_hull_size; i++)
      {
        for (size_t j = 0; j < other_hull_size; j++)
        {
          size_t next_index = j + 1;
          if(next_index == other_hull_size) next_index = 0;
          float dist_sqr = distToLineSegmentOnZPlaneSqr(&vertices->at(i), &other_hull->vertices->at(j), &other_hull->vertices->at(next_index));
          if(max_dist_sqr >= dist_sqr){
            fits++;
            break;
          }
        }
        if(fits >= min_fits) return true;
      }

      // Fit current Lines to Vertices on other hull
      for (size_t i = 0; i < other_hull_size; i++)
      {
        for (size_t j = 0; j < this_hull_size; j++)
        {
          size_t next_index = j + 1;
          if(next_index == this_hull_size) next_index = 0;
          float dist_sqr = distToLineSegmentOnZPlaneSqr(&other_hull->vertices->at(i), &vertices->at(j), &vertices->at(next_index));
          if(max_dist_sqr >= dist_sqr){
            fits++;
            break;
          }
        }
        if(fits >= min_fits) return true;
      }
    }
    return false;
};
void Hull::printHull(){
  ROS_INFO("Hull:");
  for (size_t i = 0; i < vertices->size(); i++)
  {
    ROS_INFO("\tx = %f\ty = %f\tz = %f", vertices->at(i).x, vertices->at(i).y, vertices->at(i).z);
  }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr Hull::newSwingingArmAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float r){
  
  uint32_t n = cloud->size();
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ>);
  if(n < 4) return cloud;
  int count = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  // Find max y point (and min x if multiply)
  size_t initial_index = 0;
  for (size_t point_index = 0; point_index < n; point_index++)
  {
    if(cloud->at(point_index).y >= cloud->at(initial_index).y){
      if(cloud->at(point_index).y == cloud->at(initial_index).y && cloud->at(point_index).x > cloud->at(initial_index).x){
        continue;
      }else{
        initial_index = point_index;
      }
    }
  }

  size_t current_hull_point_index = initial_index;
  float prev_angle = 0; 
  float r_sqr = r*r;
  float r_sqrt_2 = r*sqrt(2.0);
  float r_sqrt_2_sqr = r_sqrt_2*r_sqrt_2;
  hull_points->push_back(cloud->at(initial_index));

  // int crossCount = 0;

  while(true)
  {
    size_t best_index = 0;
    float best_angle_diff = -100; // Impossibly bad
    float best_dist = -1; // Impossibly bad
    float best_new_angle = 0;

    // Find possible points for next hullpoint
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int n_neighbors = kdtree.radiusSearch (cloud->at(current_hull_point_index), r, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    if ( n_neighbors > 1 ){
      // Find possible linesegments that can be crossed
      std::vector<int> segment_p1_index;
      std::vector<int> segment_p2_index;
      int prev_point_is_near = true;
      for (int i = hull_points->size() - 3; i >= 0; i--)
      {
        bool current_point_is_near = distanceSqr(hull_points->at(i), hull_points->back()) < r_sqrt_2_sqr;
        if(prev_point_is_near || current_point_is_near){
          segment_p1_index.push_back(i+1);
          segment_p2_index.push_back(i);
        }
        prev_point_is_near = current_point_is_near;
      }
      // Find next hull point
      for (size_t neighbors_index = 0; neighbors_index < n_neighbors; neighbors_index++)
      {
        int pointcloud_index = pointIdxRadiusSearch.at(neighbors_index);
        if(pointcloud_index == current_hull_point_index) continue; // Skip current point
        float distance_sqr = pointRadiusSquaredDistance.at(neighbors_index);
        if(distance_sqr == 0) continue; // Skip if duplicate point

        float new_angle = atan2(cloud->at(pointcloud_index).y - cloud->at(current_hull_point_index).y, cloud->at(pointcloud_index).x - cloud->at(current_hull_point_index).x);
        float angle_diff = new_angle - prev_angle;

        // Wrap
        if(angle_diff >= (PI - 10*(PI/180.0))) angle_diff -= 2*PI; // Maximum 170 degrees counter clockwise
        else if(angle_diff < -(PI + 10*(PI/180.0))) angle_diff += 2*PI; // Maximum 170 degrees clockwise

        // Update if best
        if(best_angle_diff <= angle_diff){
          // Skip if same angle but less distance
          if(best_angle_diff == angle_diff && distance_sqr < best_dist) continue; 

          // Check if crossing line
          bool crossing = false;
          for (size_t i = 0; i < segment_p1_index.size(); i++)
          {
            if(doSegmentsCross(&hull_points->at(segment_p1_index.at(i)), &hull_points->at(segment_p2_index.at(i)), &hull_points->back(), &cloud->at(pointcloud_index))){
              // crossCount++;       //REMOVE
              crossing = true;
              // ROS_INFO("Crossing detected to point %d", (int) pointcloud_index);
              break;
            }
          }
          if(crossing) continue;

          // Update best
          best_index = pointcloud_index;
          best_new_angle = new_angle;
          best_angle_diff = angle_diff;
          best_dist = distance_sqr;
        }
      }
      if(best_index == initial_index) return hull_points;

      // Update hull
      current_hull_point_index = best_index;
      prev_angle = best_new_angle; 
      hull_points->push_back(cloud->at(best_index));
      count++;
      
      // Avoid infinite loops
      if(count >= 200 || count > n){
        hull_points->clear();
        return hull_points;
      }
    }else{
      
      return hull_points;
    }
  }
  return hull_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Hull::swingingArmAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float r){
  
  uint32_t n = cloud->size();
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ>);
  if(n < 4) return cloud;
  int count = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  // Find max y point (and min x if multiply)
  size_t initial_index = 0;
  for (size_t point_index = 0; point_index < n; point_index++)
  {
    if(cloud->at(point_index).y >= cloud->at(initial_index).y){
      if(cloud->at(point_index).y == cloud->at(initial_index).y && cloud->at(point_index).x > cloud->at(initial_index).x){
        continue;
      }else{
        initial_index = point_index;
      }
    }
  }

  size_t current_hull_point_index = initial_index;
  float prev_angle = 0; 
  float r_sqr = r*r;
  hull_points->push_back(cloud->at(initial_index));
  std::vector<float> angles_diffs;
  angles_diffs.push_back(0); // Initial turn angle

  while(true)
  {
    size_t best_index = 0;
    float best_angle_diff = -100; // Impossibly bad
    float best_dist = -1; // Impossibly bad
    float best_new_angle = 0;

    // Find previous turns to avoid crossing line
    int total_prev_diffs = angles_diffs.size();
    float total_prev_diff = angles_diffs.at(total_prev_diffs - 1);
    for (int i = total_prev_diffs - 3; i >= 0; i--)
    {
      float temp_dist = distanceSqr(cloud->at(current_hull_point_index), hull_points->at(i));
      if(temp_dist <= r_sqr){
        total_prev_diff += angles_diffs.at(i+1);
      }else{
        break;
      }
    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int n_neighbors = kdtree.radiusSearch (cloud->at(current_hull_point_index), r, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if ( n_neighbors > 1 ){
      for (size_t neighbors_index = 0; neighbors_index < n_neighbors; neighbors_index++){
        int pointcloud_index = pointIdxRadiusSearch.at(neighbors_index);
        if(pointcloud_index == current_hull_point_index) continue; // Skip current point
        float distance_sqr = pointRadiusSquaredDistance.at(neighbors_index);
        if(distance_sqr == 0) continue; // Skip if duplicate point

        float new_angle = atan2(cloud->at(pointcloud_index).y - cloud->at(current_hull_point_index).y, cloud->at(pointcloud_index).x - cloud->at(current_hull_point_index).x);
        float angle_diff = new_angle - prev_angle;

        // Wrap
        if(angle_diff >= (PI - 10*(PI/180.0))) angle_diff -= 2*PI; // Maximum 170 degrees counter clockwise
        else if(angle_diff < -(PI + 10*(PI/180.0))) angle_diff += 2*PI; // Maximum 170 degrees clockwise

        // Avoid crossing line
        if(angle_diff > 0){
          float total_diff = angle_diff + total_prev_diff;
          if(total_diff > PI){
            continue; 
          }
        }

        // Update if best
        if(best_angle_diff <= angle_diff){
          if(best_angle_diff == angle_diff && distance_sqr < best_dist) continue; // Skip if same angle but less distance
          best_index = pointcloud_index;
          best_new_angle = new_angle;
          best_angle_diff = angle_diff;
          best_dist = distance_sqr;
        }
      }

      // Check if ended
      if(best_index == initial_index) return hull_points;

      // Update hull
      current_hull_point_index = best_index;
      prev_angle = best_new_angle; 
      angles_diffs.push_back(best_angle_diff);
      hull_points->push_back(cloud->at(best_index));
      count++;
      
      // Avoid infinite loops
      if(count >= 200 || count > n){
        ROS_WARN("Swinging Arm Algorithm Error: Stuck!");
        hull_points->clear();
        return hull_points;
      }
    }else{
      ROS_WARN("Swinging Arm Algorithm Error: Can not find neighbors!");
      return hull_points;
    }
  }
  return hull_points;
}
