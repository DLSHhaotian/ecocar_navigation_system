#include "vlp16_importer.h"

/*
To use pointer, create using:
MyClass x; // This creates memory on the stack, which you could run out of. To avoid create in heap.
(this cleans memory on return).
Use & to pass as parameter (expect * input). Without it is copied.
To return a new class created in the local function, return x will return a copy. 
To return a pointer to a new class it needs to be created as pointer as below (MyClass *x = new MyClass;). 
But then remember to delete after use.

If created using pointer notatation:
MyClass *x = new MyClass; //Allocates memory on heap (for larger memory, but must be deleted.)
Clean by:
delete x;
 */ 



pcl::PointXYZ lidar2Point(uint16_t R, float cos_omega, float sin_omega, float cos_alpha, float sin_alpha){
  float dist = ((float) R) * DIST_TO_METERS;
  // alpha + 90 degrees to align forward along x-axis
  float x = dist * cos_omega * cos_alpha; 
  float y = -dist * cos_omega * sin_alpha;
  float z = dist * sin_omega;
  // Otherwise:
  // float x = dist * cos_omega * sin_alpha; 
  // float y = dist * cos_omega * cos_alpha;
  // float z = dist * sin_omega;
  return pcl::PointXYZ(x,y,z);
};
static float toDist(uint16_t R) {
  return ((float) R) * DIST_TO_METERS;
}


float Distance(pcl::PointXYZ * p){
  return sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
};


bool preFilter(uint16_t channel_index, uint16_t distance, float azimuth){
  // if(REMOVE_UPWARDS_DIRECTIONS && channel_index % 2 != 0) return false; // Remove upwards going LiDAR scans
  if(REMOVE_UPWARDS_DIRECTIONS && channel_index % 2 != 0 && channel_index != 1) return false; // Remove upwards going LiDAR scans

  static const float viewAngle = VALID_LIDAR_ANGLE * DEGREES_TO_RADIANS;
  if(distance <= VALID_MAX_DISTANCE * METERS_TO_DIST && distance >= VALID_MIN_DISTANCE * METERS_TO_DIST)
    if(
      (azimuth <=  viewAngle && azimuth >= -viewAngle) || 
      (azimuth >= 2*PI - viewAngle && azimuth <= 2*PI + viewAngle)
    ) return true;
  return false;
}

bool isWithinCircleSide(pcl::PointXYZ * point){
  float abs_x = fabs(point->x);
  if(abs_x > CIRCLE_SIDES_RADIUS) return false;
  float abs_y = fabs(point->y);
  if(abs_y >= CIRCLE_SIDES_DISTANCE) return true;
  float y_dist = abs_y - CIRCLE_SIDES_DISTANCE;
  if(abs_x*abs_x + y_dist*y_dist > CIRCLE_SIDES_RADIUS_SQR) return false;
  return true;
};

bool postFilter(pcl::PointXYZ * p){
    if(p->y <= -VALID_ABS_Y_MAX_DISTANCE || p->y >= VALID_ABS_Y_MAX_DISTANCE) return false;
    if(p->z <= VALID_Z_MIN_DISTANCE) return false;
    if(isWithinCircleSide(p)) return false;
    return true;
}

void importPoints(Package * point_package, pcl::PointCloud<pcl::PointXYZ>::Ptr points, odo_data * in_odo, bool applyFilter){
  const static float channel_altitude[] = {
    -0.26179938780, // 00: -15 degrees
    +0.01745329252, // 01: +01 degrees
    -0.22689280276, // 02: -13 degrees
    +0.05235987756, // 03: -03 degrees
    -0.19198621772, // 04: -11 degrees
    +0.08726646260, // 05: +05 degrees
    -0.15707963268, // 06: -09 degrees
    +0.12217304764, // 07: +07 degrees
    -0.12217304764, // 08: -07 degrees
    +0.15707963268, // 09: +09 degrees
    -0.08726646260, // 10: -05 degrees
    +0.19198621772, // 11: +11 degrees
    -0.05235987756, // 12: -03 degrees
    +0.22689280276, // 13: +13 degrees
    -0.01745329252, // 14: -01 degrees
    +0.26179938780  // 15: +15 degrees
  };
  const static float cos_omega[] = {
    (float) cos(channel_altitude[0]),
    (float) cos(channel_altitude[1]),
    (float) cos(channel_altitude[2]),
    (float) cos(channel_altitude[3]),
    (float) cos(channel_altitude[4]),
    (float) cos(channel_altitude[5]),
    (float) cos(channel_altitude[6]),
    (float) cos(channel_altitude[7]),
    (float) cos(channel_altitude[8]),
    (float) cos(channel_altitude[9]),
    (float) cos(channel_altitude[10]),
    (float) cos(channel_altitude[11]),
    (float) cos(channel_altitude[12]),
    (float) cos(channel_altitude[13]),
    (float) cos(channel_altitude[14]),
    (float) cos(channel_altitude[15]),
  };
  const static float sin_omega[] = {
    (float) sin(channel_altitude[0]),
    (float) sin(channel_altitude[1]),
    (float) sin(channel_altitude[2]),
    (float) sin(channel_altitude[3]),
    (float) sin(channel_altitude[4]),
    (float) sin(channel_altitude[5]),
    (float) sin(channel_altitude[6]),
    (float) sin(channel_altitude[7]),
    (float) sin(channel_altitude[8]),
    (float) sin(channel_altitude[9]),
    (float) sin(channel_altitude[10]),
    (float) sin(channel_altitude[11]),
    (float) sin(channel_altitude[12]),
    (float) sin(channel_altitude[13]),
    (float) sin(channel_altitude[14]),
    (float) sin(channel_altitude[15]),
  };
  uint16_t block_index;
  uint16_t channel_index;
  int azi_delta = 0;

  // Transform with odometry
  bool apply_odo = in_odo != 0;
  odo_data odo;
  if(apply_odo){
    odo = * in_odo;
  }

  for(block_index = 0; block_index < BLOCKS_PER_PACKAGE; block_index++){
    float alpha [2];
    float cos_alpha [2];
    float sin_alpha [2];

    alpha[0] = ((float)point_package->blocks[block_index].azimuth)/100.0 * DEGREES_TO_RADIANS;
    cos_alpha[0] = cos(alpha[0]);
    sin_alpha[0] = sin(alpha[0]);

    if(block_index != BLOCKS_PER_PACKAGE - 1){ //In case last block in package, else using the previous
      azi_delta = point_package->blocks[block_index + 1].azimuth - point_package->blocks[block_index].azimuth;
      if(azi_delta < 0) azi_delta += 36000; // for passing 359 to 0
    }
    alpha[1] = ((float)(point_package->blocks[block_index].azimuth + azi_delta/2)/100.0 * DEGREES_TO_RADIANS);
    cos_alpha[1] = cos(alpha[1]);
    sin_alpha[1] = sin(alpha[1]);

    for (size_t dataset_index = 0; dataset_index < CHANNEL_SETS_PER_BLOCK; dataset_index++){
      for(channel_index = 0; channel_index < CHANNELS; channel_index++){
        uint16_t current_distance = point_package->blocks[block_index].dataset[dataset_index].channels[channel_index].distance;
        // Pre Filter: raw data
        if(applyFilter && !preFilter(channel_index, current_distance, alpha[dataset_index])){
          continue;
        }
        
        // Calculate point
        pcl::PointXYZ temp = lidar2Point(
          current_distance,
          cos_omega[channel_index], sin_omega[channel_index], cos_alpha[dataset_index], sin_alpha[dataset_index]
        );

        // Post Filter: Position
        if(applyFilter && !postFilter(&temp)){
          continue;
        }

        if(apply_odo){
          float x = temp.x*odo.cos_orientation - temp.y*odo.sin_orientation + odo.x_pos;  
          float y = temp.x*odo.sin_orientation + temp.y*odo.cos_orientation + odo.y_pos; 
          float z =  temp.z;  
          temp.x = x;
          temp.y = y; 
          temp.z = z; 
        }
        points->push_back(temp);  
      } 
    }
  }
}

void importPackage(const velodyne_msgs::VelodyneScan &pkt, pcl::PointCloud<pcl::PointXYZ>::Ptr points, Odometry * odo_object, ros::Time * lidar_timestamp, bool applyFilter){
  int packages =  (int) pkt.packets.size();
  // Transform with odometry
  if (odo_object != 0)
  {
    ros::Time current_time = * lidar_timestamp - ros::Duration( ((float) packages) * TIME_BETWEEN_PACKETS);
    for(int package_index = 0; package_index < packages; package_index++){
      current_time = current_time + ros::Duration( ((float) package_index) * TIME_BETWEEN_PACKETS);
      odo_data odo = odo_object->getInterpolatedData(current_time);
      importPoints((Package *) &pkt.packets[package_index].data, points, &odo, applyFilter); 
    }
  }
  else
  {
    for(int package_index = 0; package_index < packages; package_index++){
      importPoints((Package *) &pkt.packets[package_index].data, points, 0, applyFilter); 
    }
  }
}

void importSimPoints(const sensor_msgs::PointCloud2 &pkt, pcl::PointCloud<pcl::PointXYZ>::Ptr points, Odometry * odo_object, ros::Time * lidar_timestamp, bool applyFilter){
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(pkt, pcl_pc);
  if(applyFilter){
    pcl::PointCloud<pcl::PointXYZ> temp_points;
    pcl::fromPCLPointCloud2(pcl_pc, temp_points);
    for (auto &point : temp_points){

      // Simulated Pre Filter: raw data 
      if(point.z >= 0) continue;
      if((point.x * point.x + point.y * point.y) > VALID_MAX_DISTANCE * VALID_MAX_DISTANCE) continue;
      if(atan2(point.y, point.x) > (VALID_LIDAR_ANGLE/180.0*PI)) continue;
      if(atan2(point.y, point.x) < -(VALID_LIDAR_ANGLE/180.0*PI)) continue;

      // Post Filter: Position
      if(!postFilter(&point)){
        continue;
      }
      points->push_back(point);  
    }
  }else{
    pcl::fromPCLPointCloud2(pcl_pc, *points);
  }
  if (odo_object != 0){
    odo_object->transformPoints(points);
  }
}

Node::Node(){
  id = 1;
  roughness = -1.0;
  type = 0;
}
Node::Node(uint32_t in_id){
  id = in_id;
  roughness = -1.0; 
  type = 0;
}
Node::Node(uint32_t * id_ptr){
  id = * id_ptr;
  * id_ptr = (* id_ptr) + 1;
  roughness = -1.0;
  type = 0;
}
Node::~Node(){
  // std::cout << "Node " << id << " destroyed!" << std::endl;
}
float Node::sqrDistanceTo(Node::ptr other_node) const{
  return  (other_node->p.x - p.x) * (other_node->p.x - p.x) + 
          (other_node->p.y - p.y) * (other_node->p.y - p.y) + 
          (other_node->p.z - p.z) * (other_node->p.z - p.z);
}
float Node::distanceTo(Node::ptr other_node) const{
  return sqrt(sqrDistanceTo(other_node));
}
void Node::printNode() const{
    std::cout << "Node " << id << " has:" << std::endl;
    if(upSet()) std::cout << "\tNeighbor up: Node " << Up()->id << std::endl;
    else std::cout << "\tNeighbor up: none" << std::endl;
    if(downSet()) std::cout << "\tNeighbor down: Node " << Down()->id << std::endl;
    else std::cout << "\tNeighbor down: none" << std::endl;
    if(leftSet()) std::cout << "\tNeighbor left: Node " << Left()->id << std::endl;
    else std::cout << "\tNeighbor left: none" << std::endl;
    if(rightSet()) std::cout << "\tNeighbor right: Node " << Right()->id << std::endl;
    else std::cout << "\tNeighbor right: none" << std::endl;
}


Graph::Graph(){
  timestamp = ros::Time::now();
  import_frontier.resize(CHANNELS);
}
Graph::~Graph(){
  // for (auto &node : nodes) node->clear();
  // std::cout << "Graph destroyed!" << std::endl;
}

void Graph::importNodesFromPackage(Package * point_package, odo_data * in_odo, bool applyFilter){
  const static float channel_altitude[] = {
    -0.26179938780, // 00: -15 degrees
    +0.01745329252, // 01: +01 degrees
    -0.22689280276, // 02: -13 degrees
    +0.05235987756, // 03: -03 degrees
    -0.19198621772, // 04: -11 degrees
    +0.08726646260, // 05: +05 degrees
    -0.15707963268, // 06: -09 degrees
    +0.12217304764, // 07: +07 degrees
    -0.12217304764, // 08: -07 degrees
    +0.15707963268, // 09: +09 degrees
    -0.08726646260, // 10: -05 degrees
    +0.19198621772, // 11: +11 degrees
    -0.05235987756, // 12: -03 degrees
    +0.22689280276, // 13: +13 degrees
    -0.01745329252, // 14: -01 degrees
    +0.26179938780  // 15: +15 degrees
  };
  const static uint16_t channel_ordered[] = {
    // Bottom up
    0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15
  };
  const static float cos_omega[] = {
    (float) cos(channel_altitude[0]),
    (float) cos(channel_altitude[1]),
    (float) cos(channel_altitude[2]),
    (float) cos(channel_altitude[3]),
    (float) cos(channel_altitude[4]),
    (float) cos(channel_altitude[5]),
    (float) cos(channel_altitude[6]),
    (float) cos(channel_altitude[7]),
    (float) cos(channel_altitude[8]),
    (float) cos(channel_altitude[9]),
    (float) cos(channel_altitude[10]),
    (float) cos(channel_altitude[11]),
    (float) cos(channel_altitude[12]),
    (float) cos(channel_altitude[13]),
    (float) cos(channel_altitude[14]),
    (float) cos(channel_altitude[15]),
  };
  const static float sin_omega[] = {
    (float) sin(channel_altitude[0]),
    (float) sin(channel_altitude[1]),
    (float) sin(channel_altitude[2]),
    (float) sin(channel_altitude[3]),
    (float) sin(channel_altitude[4]),
    (float) sin(channel_altitude[5]),
    (float) sin(channel_altitude[6]),
    (float) sin(channel_altitude[7]),
    (float) sin(channel_altitude[8]),
    (float) sin(channel_altitude[9]),
    (float) sin(channel_altitude[10]),
    (float) sin(channel_altitude[11]),
    (float) sin(channel_altitude[12]),
    (float) sin(channel_altitude[13]),
    (float) sin(channel_altitude[14]),
    (float) sin(channel_altitude[15]),
  };
  uint16_t block_index;
  uint16_t channel_index;
  int azi_delta = 0;
  uint32_t id_top = 0;
  // Transform with odometry
  bool apply_odo = in_odo != 0;
  odo_data odo;
  if(apply_odo){
    odo = * in_odo;
  }
  for(block_index = 0; block_index < BLOCKS_PER_PACKAGE; block_index++){
    float alpha [2];
    float cos_alpha [2];
    float sin_alpha [2];

    alpha[0] = ((float)point_package->blocks[block_index].azimuth)/100.0 * DEGREES_TO_RADIANS;
    cos_alpha[0] = cos(alpha[0]);
    sin_alpha[0] = sin(alpha[0]);

    if(block_index != BLOCKS_PER_PACKAGE - 1){ //In case last block in package, else using the previous
      azi_delta = point_package->blocks[block_index + 1].azimuth - point_package->blocks[block_index].azimuth;
      if(azi_delta < 0) azi_delta += 36000; // for passing 359 to 0
    }
    alpha[1] = ((float)(point_package->blocks[block_index].azimuth + azi_delta/2)/100.0 * DEGREES_TO_RADIANS);
    cos_alpha[1] = cos(alpha[1]);
    sin_alpha[1] = sin(alpha[1]);

    for (size_t dataset_index = 0; dataset_index < CHANNEL_SETS_PER_BLOCK; dataset_index++){
      for(channel_index = 0; channel_index < CHANNELS; channel_index++){
        uint16_t current_index = channel_ordered[channel_index];
        uint16_t current_distance = point_package->blocks[block_index].dataset[dataset_index].channels[current_index].distance;

        // if(current_index % 2 != 0) continue; // Allways remove top points
        if(!applyFilter & toDist(current_distance) <= 0.1) continue;
        // Pre Filter: raw data
        else if(applyFilter && !preFilter(current_index, current_distance, alpha[dataset_index])){
          import_frontier.at(current_index).reset();
          continue;
        }
        
        // Calculate point
        pcl::PointXYZ temp = lidar2Point(
          current_distance,
          cos_omega[current_index], sin_omega[current_index], cos_alpha[dataset_index], sin_alpha[dataset_index]
        );

        // Post Filter: Position
        if(applyFilter && !postFilter(&temp)){
          import_frontier.at(current_index).reset();
          continue;
        }

        if(apply_odo){
          float x = temp.x*odo.cos_orientation - temp.y*odo.sin_orientation + odo.x_pos;  
          float y = temp.x*odo.sin_orientation + temp.y*odo.cos_orientation + odo.y_pos; 
          float z =  temp.z;  
          temp.x = x;
          temp.y = y; 
          temp.z = z; 
        }

        // Make new node
        Node::ptr temp_node_ptr(new Node());
        temp_node_ptr->p = temp;
        temp_node_ptr->distance = current_distance;
        temp_node_ptr->channel = current_index;
        temp_node_ptr->type = 0;
        addNode(temp_node_ptr);

        // Check if "left" neighbor exists
        Node::ptr temp_left_neighbor = import_frontier.at(current_index);
        if(temp_left_neighbor){
          // Check if close enough
          if(toDist(abs(temp_left_neighbor->distance - current_distance))/toDist(current_distance) < 0.1){
            // Close enough
            // Other checks
            // Calculate roughness
            if(CALCULATE_ROUGHNESS){
              // Get 5 last if exist
              static const uint16_t window_size = ROUGHNESS_WINDOW_SIZE;
              std::vector< Node::ptr > window;
              bool valid_window = true;
              window.push_back(temp_node_ptr);
              window.push_back(temp_left_neighbor);
              for (size_t i = 2; i < window_size; i++)
              {
                if(window.back()->leftSet()){
                  window.push_back(window.back()->Left());
                }else{
                  valid_window = false;
                  break;
                }
              }
              // If exist, calculate roughness
              if(valid_window){
                float diff[window_size-1];
                float sum_diff = 0;
                for (size_t i = 0; i < window_size-1; i++)
                {
                  diff[i] = window.at(i)->distance - window.at(i+1)->distance;
                  sum_diff += diff[i];
                }
                float mean_diff = sum_diff/(window_size-1);
                float sd_diff = 0;
                for (size_t i = 0; i < window_size-1; i++)
                {
                  float temp = diff[i] - mean_diff;
                  sd_diff += temp*temp;
                }
                sd_diff = sqrt(sd_diff/(window_size-2));
                window.at((window_size-1)/2)->roughness = sd_diff;
              } 
            }
     
            // Make neighbor
            connectHorizontal(temp_left_neighbor, temp_node_ptr);
          }
        }
        // Update import_frontier
        import_frontier.at(current_index) = temp_node_ptr;

        static const float max_height_change = MAX_HEIGHT_CHANGE;
        static const float max_angle = MAX_SLOPE_VERTICAL_NEIGHBORS;
        static const float max_angle_change = MAX_SLOPE_CHANGE_VERTICAL_NEIGHBORS;
        if(channel_index == 0){
          if(temp_node_ptr->p.z <= TRUE_GROUND_HIGH_MAX && temp_node_ptr->p.z > TRUE_GROUND_HIGH_MIN && temp_node_ptr->distance <= TRUE_GROUND_MAX_DISTANCE*METERS_TO_DIST){
            temp_node_ptr->type |= NodeType::GROUND;
          }
        }else{
          Node::ptr temp_below_neighbor = import_frontier.at(channel_ordered[channel_index-1]);
          if(temp_below_neighbor){
            // Calculate slope
            temp_node_ptr->type |= NodeType::SLOPE_AVAILIBLE;
            float y = temp_node_ptr->p.z - temp_below_neighbor->p.z;
            float x = temp_node_ptr->distance*DIST_TO_METERS*cos_omega[temp_node_ptr->channel] - temp_below_neighbor->distance*DIST_TO_METERS*cos_omega[temp_below_neighbor->channel];
            float slope = atan2(y, x);
            temp_node_ptr->vertical_slope = slope;

            // Check if height change is small, aka low pass
            if(temp_below_neighbor->type & (NodeType::GROUND) && fabs(temp_node_ptr->p.z - temp_below_neighbor->p.z) <= max_height_change){
              temp_node_ptr->type |= (NodeType::GROUND);
              // temp_node_ptr->type |= (NodeType::SLOPE_CHANGE_AVAILIBLE); // TEST DEBUG REMOVE
              connectVertical(temp_node_ptr, temp_below_neighbor);
            }else{
              // Check slope
              if(slope < max_angle && slope > -max_angle ){  
                temp_node_ptr->type |= NodeType::SLOPE_CONFIRMED;
                connectVertical(temp_node_ptr, temp_below_neighbor);
                  // If has another  previous
                  if(temp_below_neighbor->downSet()){
                    temp_node_ptr->type |= NodeType::SLOPE_CHANGE_AVAILIBLE;
                    // Check slope change
                    if(fabs(temp_node_ptr->vertical_slope - temp_below_neighbor->vertical_slope) < max_angle_change){
                      temp_node_ptr->type |= (NodeType::SLOPE_CHANGE_CONFIRMED);

                      if(temp_below_neighbor->type & (NodeType::GROUND)){
                        temp_node_ptr->type |= (NodeType::GROUND);
                      }else{
                        // Needs neigbor approval
                        temp_node_ptr->type |= NodeType::NEIGBOR_APPROVAL_AVAILIBLE;
                      }
                    }else{
                      if(temp_below_neighbor->type & (NodeType::GROUND)){

                      }else{
                        temp_node_ptr->type |= NodeType::NEIGBOR_APPROVAL_AVAILIBLE;
                      }
                      // Failed change
                    }
                  }else{
                    // If Previous is ground, make ground
                    if(temp_below_neighbor->type & (NodeType::GROUND)){
                      // Make ground
                      temp_node_ptr->type |= NodeType::GROUND; // Second point, no change availible
                    }else{
                      temp_node_ptr->type |= NodeType::NEIGBOR_APPROVAL_AVAILIBLE;
                    }
                  }
              }else{
                // Too steep
              } 
            }
          }else{
            // Unchecked because missing previous 
            temp_node_ptr->type |= NodeType::UNCHECKED;
            temp_node_ptr->type |= NodeType::NEIGBOR_APPROVAL_AVAILIBLE;
          }
        }

        // Apply neighbor approvals: Apply left to right
        if(temp_node_ptr->type & NodeType::NEIGBOR_APPROVAL_AVAILIBLE){
          if(temp_node_ptr->leftSet() && (temp_node_ptr->Left()->type & (NodeType::GROUND | NodeType::NEIGBOR_APPROVAL ))){
            temp_node_ptr->type |= NodeType::NEIGBOR_APPROVAL; // Good enough for jazz
          }
        }
        // Apply neighbor approvals: Apply right to left
        if(temp_node_ptr->type & NodeType::GROUND){
          Node::ptr leftSearch = temp_node_ptr;
          while(leftSearch->leftSet()){
            // Not already approved
            if(leftSearch->Left()->type & (NodeType::GROUND | NodeType::NEIGBOR_APPROVAL)){
              break;
            }else{
              // Availible for approvel
              if(leftSearch->Left()->type & NodeType::NEIGBOR_APPROVAL_AVAILIBLE){
                leftSearch->Left()->type |= NodeType::NEIGBOR_APPROVAL;
                leftSearch = leftSearch->Left();
              }else{
                break;
              }
            }
          }
        }
      }
    }
  }
}

void Graph::importNodesFromScan(const velodyne_msgs::VelodyneScan &pkt, Odometry * odo_object, ros::Time * lidar_timestamp, bool applyFilter){
  for (auto &node : import_frontier) node.reset(); // Clear scan frontier
  int packages =  (int) pkt.packets.size();
  // Transform with odometry
  if (odo_object != 0)
  {
    ros::Time current_time = * lidar_timestamp - ros::Duration( ((float) packages) * TIME_BETWEEN_PACKETS);
    for(int package_index = 0; package_index < packages; package_index++){
      current_time = current_time + ros::Duration( ((float) package_index) * TIME_BETWEEN_PACKETS);
      odo_data odo = odo_object->getInterpolatedData(current_time);
      importNodesFromPackage((Package *) &pkt.packets[package_index].data, &odo, applyFilter); 
    }
  }
  else
  {
    for(int package_index = 0; package_index < packages; package_index++){
      importNodesFromPackage((Package *) &pkt.packets[package_index].data); 
    }
  }
}

void Graph::exportToList(pcl::PointCloud<pcl::PointXYZ>::Ptr pointListFrom, pcl::PointCloud<pcl::PointXYZ>::Ptr pointListTo){
  // List of lines for visualization
  for (auto &node : nodes){
    if(node->upSet()){
      pointListFrom->push_back(node->p);
      pointListTo->push_back(node->Up()->p);
    }
    if(node->rightSet()){
      pointListFrom->push_back(node->p);
      pointListTo->push_back(node->Right()->p);
    }
  }
}
void Graph::exportVerticalToList(pcl::PointCloud<pcl::PointXYZ>::Ptr pointListFrom, pcl::PointCloud<pcl::PointXYZ>::Ptr pointListTo){
  // List of lines for visualization
  for (auto &node : nodes){
    if(node->upSet()){
      pointListFrom->push_back(node->p);
      pointListTo->push_back(node->Up()->p);
    }
  }
}
void Graph::exportHorizontalToList(pcl::PointCloud<pcl::PointXYZ>::Ptr pointListFrom, pcl::PointCloud<pcl::PointXYZ>::Ptr pointListTo){
  // List of lines for visualization
  for (auto &node : nodes){
    if(node->rightSet()){
      pointListFrom->push_back(node->p);
      pointListTo->push_back(node->Right()->p);
    }
  }
}
void Graph::exportObstaclePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points) const{
  for (auto &node : nodes){
    if(node->type & (NodeType::GROUND | NodeType::NEIGBOR_APPROVAL)){
      if(ground_points){
        pcl::PointXYZ p = pcl::PointXYZ(); 
        p.x = node->p.x; 
        p.y = node->p.y; 
        p.z = node->p.z; 
        ground_points->push_back(p); 
      }
    }else{
      pcl::PointXYZ p = pcl::PointXYZ(); 
      p.x = node->p.x; 
      p.y = node->p.y; 
      p.z = node->p.z; 
      points->push_back(p); 
    }
  }
}
void Graph::exportToPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points){
  for (auto &node : nodes){
    pcl::PointXYZRGB p = pcl::PointXYZRGB(); 
    p.x = node->p.x; 
    p.y = node->p.y; 
    p.z = node->p.z; 

    if(node->type  & NodeType::GROUND){
      p.r = 0; 
      p.g = 255; 
      p.b = 0; 
    }else if (node->type & NodeType::NEIGBOR_APPROVAL_AVAILIBLE){
      p.r = 255; 
      p.g = 255; 
      p.b = 0;  
    }else{
      p.r = 255; 
      p.g = 0; 
      p.b = 0; 
    }

    points->push_back(p); 
  }
}
void Graph::exportToPoints2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points){
  for (auto &node : nodes){
    pcl::PointXYZRGB p = pcl::PointXYZRGB(); 
    p.x = node->p.x; 
    p.y = node->p.y; 
    p.z = node->p.z; 
    if(node->type & (NodeType::GROUND | NodeType::NEIGBOR_APPROVAL)){
      p.r = 0; 
      p.g = 255; 
      p.b = 0;  
    }else{
      p.r = 255; 
      p.g = 0; 
      p.b = 0; 
    }
    points->push_back(p); 
  }
}

void Graph::exportToPoints3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points){
  for (auto &node : nodes){
    pcl::PointXYZRGB p = pcl::PointXYZRGB(); 
    p.x = node->p.x; 
    p.y = node->p.y; 
    p.z = node->p.z; 
    if(node->roughness < 0){
      p.r = 0; 
      p.g = 0; 
      p.b = 255; 
    }else{
      float temp_roughness;
      static float max_roughness = 30.0;
      if(node->roughness > max_roughness) temp_roughness = max_roughness;
      else temp_roughness = node->roughness;
      p.r = temp_roughness/max_roughness * 255; 
      p.g = (1.0 - temp_roughness/max_roughness) * 255; 
      p.b = 0.0; 
    }
    points->push_back(p); 
  }
}
