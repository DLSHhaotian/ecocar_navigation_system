#include "point_handler.h"

Eigen::MatrixXf linear_regression_fit(Eigen::MatrixXf X, Eigen::VectorXf Y){
  return Eigen::MatrixXf ((X.transpose() * X).inverse()) * (X.transpose() * Y);
}

Plane RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points, const float max_dist, const int max_i, RANSAC_method method, Plane * plane){
  Plane bestFit;
  int mostFits = 0;
  int max_forced = max_dist*10;

  float best_err = -1;
  int n = points->size();
  int min_fits = n/10; // Min number of fittet point in a model

  Plane fit;
  int fittetPoints;
  float err;
  if(n <= 30) return bestFit;
  for(int i = 0; i < max_i; i++){
    // Make random selection
    int p1_index = rand()%(n);
    int p2_index, p3_index;
    do{
        p2_index = rand()%(n);
    }while (p1_index == p2_index);
    do{
        p3_index = rand()%(n);
    }while (p1_index == p3_index || p2_index == p3_index);

    // Fit plane
    fit = Plane(&points->at(p1_index), &points->at(p2_index), &points->at(p3_index));

    // Validate
    err = 0;
    fittetPoints = 0;

    //Update best
    switch (method)
    {
      case MOST_FITS:
        for(int point_index = 0; point_index < n; point_index++){
          float dist = fit.DistToPoint(&points->at(point_index));
            if(dist <= max_dist){
              fittetPoints++;
              err += dist; // Error per point
            }
        }
        if(fittetPoints >= min_fits && fittetPoints > mostFits){
          mostFits = fittetPoints;
          bestFit = fit;
        }
        break;

      case BEST_FIT:
        for(int point_index = 0; point_index < n; point_index++){
          float dist = fit.DistToPoint(&points->at(point_index));
            if(dist <= max_dist){
              fittetPoints++;
              err += dist; // Error per point
            }
        }
        if(fittetPoints >= min_fits){  // Best fits
          err = err/((float) fittetPoints);
          if(best_err == -1 || err < best_err){
            mostFits = fittetPoints; //not most, just the amount for best fit
            best_err = err;
            bestFit = fit;
          }
        }
        break;
      
      case NORMAL_VECTOR_FIT:
        err = PI/2 - fit.getAngleBetweenPlanes(plane); // Move up for more effeciency
        //ROS_INFO("\tINITIAL ANGLE: %f", err);
        if(err <= 0.1){
          for(int point_index = 0; point_index < n; point_index++){
            float dist = fit.DistToPoint(&points->at(point_index));
              if(dist <= max_dist){
                fittetPoints++;
                //err += dist; // Error per point
              }
          }
          //ROS_INFO("\t\tFITTET: %d", fittetPoints);
          if(fittetPoints >= 100){  // Best fits
            if(best_err == -1 || mostFits < fittetPoints){
              mostFits = fittetPoints; //not most, just the amount for best fit
              bestFit = fit;
              best_err = err;
            }
          }
        }else{
          i--; //set max
          max_forced--;
          if(max_forced >= 0) break;
        }
        break;
      case COLUMN_FIT:
        
      break;
      default:
        ROS_ERROR("WRONG RANSAC METHOD!");
        return bestFit;
      break;
        /* Default Code */
    }
  }
  bestFit.Normalize();

  //Debug
  if(DEBUG){
    switch (method)
    {
      case MOST_FITS:
        ROS_INFO("Most fittet points: %d\tOut of %d", mostFits, n);
        break;
      case BEST_FIT:
        if(best_err == -1) ROS_WARN("No FIT!");
        ROS_INFO("Best fit error: %f\tFor %d points out of %d", best_err, mostFits, n);
        break;
      case NORMAL_VECTOR_FIT:
        if(best_err == -1){
          ROS_WARN("No FIT!");
          return Plane();
        }
        ROS_INFO("Best angle error: %f\tFor %d points out of %d", best_err, mostFits, n);
        break;
      default:
        /* Default Code (Is unachieveable) */
        break;
    }
    ROS_INFO("\tRaw Slope:\tx = %f\ty = %f\toffset =%f", -bestFit.a/bestFit.c, -bestFit.b/bestFit.c, -bestFit.d/bestFit.c);
  }

  // Fit model
  float sq_err = 0;
  pcl::PointCloud<pcl::PointXYZ> inliers;
  for(int point_index = 0; point_index < n; point_index++){
      float dist = bestFit.DistToPoint(&points->at(point_index));
      if(dist <= max_dist/2.0){ // TODO: evaluate usability. - removes a few outliers by decreasing max distance
        inliers.push_back(points->at(point_index));
        // TODO: Return outliers so split function is not needed
        sq_err += dist * dist;
      }
  }

  if(DEBUG) ROS_INFO("Error before refit:\t%f", sq_err);


  Eigen::MatrixXf m = inliers.getMatrixXfMap();

  // Find beta vector using a*x + b*y + c = z
  m.row(2).swap(m.row(3));
  Eigen::MatrixXf result = linear_regression_fit(m.topRows(3).transpose(), m.bottomRows(1).transpose());
  Plane betterFit = Plane(result(0), result(1), result(2));
  betterFit.Normalize();

  if(DEBUG) ROS_INFO("\tSlope:\t\tx = %f\ty = %f\toffset =%f", -betterFit.a/betterFit.c, -betterFit.b/betterFit.c, -betterFit.d/betterFit.c);
  if(DEBUG) sq_err = betterFit.SqErrToPLC(&inliers);
  if(DEBUG) ROS_INFO("Error after refit 1:\t%f", sq_err);

  return bestFit;
}

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, float max_neighbor_dist, std::vector<Extreme> * output_dest){
  if(DEBUG) ROS_INFO("Segmentation - BEGIN");
  // Debug variables START
  int skip_count = 0;
  int no_skip_count = 0;
  // Debug variables END

  float max_neighbor_dist_sqr = max_neighbor_dist * max_neighbor_dist; 
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
  std::vector<Extreme> extremes;

  //While there are unchecked points left
  for(int unchecked_index = 0; unchecked_index < inputPoints->size(); unchecked_index++){
    // Pick new point
    pcl::PointXYZ * unchecked = &inputPoints->at(unchecked_index);
    int firstNeigborCloudIndex = -1;
    // Compare to all created clouds
    for(int checked_cloud_index = clouds.size() -1; checked_cloud_index >= 0 ; checked_cloud_index--){
      // Quick check before inspecting every point in cloud
      if(extremes.at(checked_cloud_index).isNear(unchecked, max_neighbor_dist)){
        if(DEBUG) no_skip_count++;
        // Check distance to each point in cloud
        for(int checked_point_index =  clouds.at(checked_cloud_index)->size() -1; checked_point_index >= 0; checked_point_index--){
          pcl::PointXYZ * checked = &clouds.at(checked_cloud_index)->at(checked_point_index);
          float x_diff = (unchecked->x - checked->x);
          if(x_diff > max_neighbor_dist) continue; // Check using infinity norm first
          float y_diff = (unchecked->y - checked->y);
          if(y_diff > max_neighbor_dist) continue; // Check using infinity norm first
          float z_diff = (unchecked->z - checked->z);
          if(z_diff > max_neighbor_dist) continue; // Check using infinity norm first

          float dist_sqr = x_diff * x_diff + y_diff * y_diff + z_diff * z_diff;
          if(dist_sqr <= max_neighbor_dist_sqr){
            if(firstNeigborCloudIndex < 0){
              // Found first neighbor, add to cloud
              firstNeigborCloudIndex = checked_cloud_index;
              clouds.at(checked_cloud_index)->push_back(* unchecked);
              // Update extremes
              extremes.at(checked_cloud_index).update(unchecked);
              break;
            }else{
              // Found yet another neighbor, merge clouds
              * clouds.at(checked_cloud_index) += * clouds.at(firstNeigborCloudIndex);
              clouds.erase(clouds.begin() + firstNeigborCloudIndex);
              // Merge extremes
              extremes.at(checked_cloud_index).merge(&extremes.at(firstNeigborCloudIndex));
              extremes.erase(extremes.begin() + firstNeigborCloudIndex);
              // Set new reference to merged index
              firstNeigborCloudIndex = checked_cloud_index;
              break;
            }
          }
        }
      }else if(DEBUG){
        // ROS_INFO("Skipped cloud");
        skip_count++;
      }
      
      if(DEBUG && firstNeigborCloudIndex < 0){
        ROS_INFO("\tDid not find any neighbors in cloud %d", checked_cloud_index);
      }
    }
    if(firstNeigborCloudIndex < 0){
      // No neighbors found, add point to own new cloud
      if(DEBUG) ROS_INFO("Did not find any neighbors at all!\n");
      pcl::PointCloud<pcl::PointXYZ>::Ptr newSegmentCloud (new pcl::PointCloud<pcl::PointXYZ>);
      newSegmentCloud->push_back(* unchecked);
      clouds.push_back(newSegmentCloud);

      // Create initial extremes
      extremes.push_back(Extreme(unchecked->x, unchecked->x, unchecked->y, unchecked->y, unchecked->z, unchecked->z));
    }
  }
  if(DEBUG) ROS_INFO("Found %d clouds.", (int) clouds.size());
  if(DEBUG) ROS_INFO("Skip count: %d\tNo skip count: %d", skip_count, no_skip_count);
  if(DEBUG) ROS_INFO("Segmentation - END");
  
  if(output_dest){
    * output_dest = extremes;
  }
  return clouds;
}

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > segmentation_old(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, float max_neighbor_dist){
  if(DEBUG) ROS_INFO("Segmentation - BEGIN");
  
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
 
  //While unchecked left
  for(int unchecked_index = 0; unchecked_index < inputPoints->size(); unchecked_index++){
    pcl::PointXYZ * unchecked = &inputPoints->at(unchecked_index);
    int firstNeigborCloudIndex = -1;
    // Pick new point
    // Go though all checked clouds/points
    for(int checked_cloud_index = 0; checked_cloud_index < clouds.size(); checked_cloud_index++){
      for(int checked_point_index = 0; checked_point_index < clouds.at(checked_cloud_index)->size(); checked_point_index++){
        // Check distance to checked points/clouds
        pcl::PointXYZ * checked = &clouds.at(checked_cloud_index)->at(checked_point_index);
        float dist = sqrt(
          (unchecked->x - checked->x)*(unchecked->x - checked->x) + 
          (unchecked->y - checked->y)*(unchecked->y - checked->y) + 
          (unchecked->z - checked->z)*(unchecked->z - checked->z)
        );
        if(dist <= max_neighbor_dist){
          if(firstNeigborCloudIndex >= 0){
            // Found yet another neighbor, merge clouds
            * clouds.at(firstNeigborCloudIndex) += * clouds.at(checked_cloud_index);
            clouds.erase(clouds.begin() + checked_cloud_index);
            checked_cloud_index--;
            break;
          }else{
            // Found first neighbor, add to cloud
            firstNeigborCloudIndex = checked_cloud_index;
            clouds.at(checked_cloud_index)->push_back(* unchecked);
            break;
          }
        }
      }
      if(DEBUG && firstNeigborCloudIndex < 0){
        ROS_INFO("\tDid not find any neighbors in cloud %d", checked_cloud_index);
      }
    }
    if(firstNeigborCloudIndex < 0){
      // No neighbors found, add point to own new cloud
      if(DEBUG) ROS_INFO("Did not find any neighbors at all!\n");
      pcl::PointCloud<pcl::PointXYZ>::Ptr newSegmentCloud (new pcl::PointCloud<pcl::PointXYZ>);
      newSegmentCloud->push_back(* unchecked);
      clouds.push_back(newSegmentCloud);
    }
  }
  if(DEBUG) ROS_INFO("Found %d clouds.", (int) clouds.size());
  if(DEBUG) ROS_INFO("Segmentation - END");
 
  return clouds;
}

void printCloud(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * clouds){
  ROS_INFO("%d clouds:", (int) clouds->size());
  for(int checked_cloud_index = 0; checked_cloud_index < clouds->size(); checked_cloud_index++){
      ROS_INFO("\tCloud %d has %d points.", checked_cloud_index, (int) clouds->at(checked_cloud_index)->size());  
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr applyVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float leafsize){
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (pointcloud);
  vg.setDownsampleAllData (false);
  vg.setLeafSize (leafsize, leafsize, leafsize);
  vg.filter (*newCloud);
  return newCloud; 
}
pcl::PointCloud<pcl::PointXYZ>::Ptr applyGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float leafsize){
  pcl::GridMinimum<pcl::PointXYZ> vg(leafsize);
  pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (pointcloud);
  vg.filter (*newCloud);
  return newCloud; 
}
pcl::PointCloud<pcl::PointXYZ>::Ptr projectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, Plane * plane){
  pcl::PointCloud<pcl::PointXYZ>::Ptr flat_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if(plane == 0){
    *flat_cloud = *pointcloud;
    for (size_t i = 0; i < flat_cloud->size(); i++)
    {
      flat_cloud->at(i).z = 0;
    }
  }else{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = plane->a;
    coefficients->values[1] = plane->b;
    coefficients->values[2] = plane->c;
    coefficients->values[3] = plane->d;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setModelCoefficients (coefficients);
    proj.setInputCloud (pointcloud);
    proj.filter (*flat_cloud);
  }

  return flat_cloud;
}

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > kdTreeSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, float max_neighbor_dist){
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
     // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (inputPoints);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (max_neighbor_dist); // 2cm
    ec.setMinClusterSize (3);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inputPoints);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (inputPoints->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clouds.push_back(cloud_cluster);
    }
    return clouds;
}

lidar_package::cloud cloudToMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  lidar_package::cloud cloud_msg;
  cloud_msg.vector_len = cloud->size();
  for (size_t j = 0; j < cloud_msg.vector_len; j++){
    lidar_package::point point;
    point.x = cloud->at(j).x;
    point.y = cloud->at(j).y;
    point.z = cloud->at(j).z;
    cloud_msg.vector_point.push_back(point);
  }
  return cloud_msg;
}

lidar_package::cloudsAndPlane cloudsAndPlaneToMsg(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * clouds, Plane * plane, Odometry * odo, ros::Time timestamp){
  lidar_package::cloudsAndPlane clouds_and_plane_msg;
  // int count = 0;
  odo_data odo_struct = odo->getInterpolatedData(timestamp);
  // plane->Normalize();
  clouds_and_plane_msg.a = plane->a;
  clouds_and_plane_msg.b = plane->b;
  clouds_and_plane_msg.c = plane->c;
  clouds_and_plane_msg.d = plane->d;
  clouds_and_plane_msg.x = odo_struct.x_pos;
  clouds_and_plane_msg.y = odo_struct.y_pos;
  clouds_and_plane_msg.z = odo_struct.z_pos;
  clouds_and_plane_msg.orientation = odo_struct.orientation;
  clouds_and_plane_msg.vector_len = clouds->size();
  clouds_and_plane_msg.timestamp = timestamp;
  for (size_t i = 0; i < clouds_and_plane_msg.vector_len; i++){
    clouds_and_plane_msg.vector_cloud.push_back(cloudToMsg(clouds->at(i)));
  }
  // ROS_INFO("Number og points on all: %d", count);
  return clouds_and_plane_msg;
}

float vectorLength(pcl::PointXYZ * p){
  return sqrt(p->x * p->x + p->y * p->y + p->z * p->z);
}
float distanceBetweenPoint(pcl::PointXYZ * p1, pcl::PointXYZ * p2){
  float p12x = p1->x - p2->x;
  float p12y = p1->y - p2->y;
  float p12z = p1->z - p2->z;
  return sqrt(p12x * p12x + p12y * p12y + p12z * p12z);
}
pcl::PointXYZ cross(pcl::PointXYZ * a, pcl::PointXYZ * b){
  pcl::PointXYZ cross_result;
  cross_result.x = a->y * b->z - a->z * b->y;
  cross_result.y = a->z * b->x - a->x * b->z;
  cross_result.z = a->x * b->y - a->y * b->x;
  return cross_result;
}
pcl::PointXYZ normalize(pcl::PointXYZ * p){
  float length = vectorLength(p);
  pcl::PointXYZ norm_result;
  norm_result.x = p->x / length;
  norm_result.y = p->y / length;
  norm_result.z = p->z / length;
  return norm_result;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr findCircleCenters(pcl::PointXYZ * p1, pcl::PointXYZ * p2, float R, Plane * plane){
  pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ q(p1->x - p2->x, p1->y - p2->y, p1->z - p2->z);
  float q_length = distanceBetweenPoint(p1, p2);

  // If points are too far, return empty center list
  if(q_length > R) return result; 

  float center_to_q_distance = sqrt(R*R - (q_length*q_length/4.0));
  pcl::PointXYZ plane_normal_vector = plane->getNormalVector();
  pcl::PointXYZ center_to_q_direction = cross(&q, &plane_normal_vector);
  center_to_q_direction = normalize(&center_to_q_direction);
  pcl::PointXYZ q_center((p1->x + p2->x)/2.0, (p1->y + p2->y)/2.0, (p1->z + p2->z)/2.0);

  
  float circle_x_1 = q_center.x + center_to_q_distance*center_to_q_direction.x;
  float circle_y_1 = q_center.y + center_to_q_distance*center_to_q_direction.y;
  float circle_z_1 = q_center.z + center_to_q_distance*center_to_q_direction.z;
  result->push_back(pcl::PointXYZ(circle_x_1, circle_y_1, circle_z_1));
  float circle_x_2 = q_center.x - center_to_q_distance*center_to_q_direction.x;
  float circle_y_2 = q_center.y - center_to_q_distance*center_to_q_direction.y;
  float circle_z_2 = q_center.z - center_to_q_distance*center_to_q_direction.z;
  result->push_back(pcl::PointXYZ(circle_x_2, circle_y_2, circle_z_2));

  // ROS_INFO("p1: %f, %f, %f", p1->x, p1->y, p1->z);
  // ROS_INFO("p2: %f, %f, %f", p2->x, p2->y, p2->z);
  // ROS_INFO("Center 1: %f, %f, %f", circle_x_1, circle_y_1, circle_z_1);
  // ROS_INFO("Center 2: %f, %f, %f", circle_x_2, circle_y_2, circle_z_2);
  // ROS_INFO("q center: %f, %f, %f", q_center.x, q_center.y, q_center.z);

  return result; 
}
Gate gateRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, float max_deviation, int raw_max_iterations, Plane * plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_full){
  const int MIN_POINTS = 6; // Minimum points in a cloud
  const int MIN_FITS = 6; // Minimum points in a circle
  const float MIN_FITS_FRACTION = 0.50;
  bool USE_FULL_CLOUDS = false;
  int n = cloud->size();
  int n_full = 0;
  if (cloud_full != 0){ // Use unfiltered cloud for counting fitted points
    USE_FULL_CLOUDS = true;
    n_full = cloud_full->size();
  } else {
    n_full = n;
  }

  // Cap number of iteration if small cloud
  float max_iterations = (raw_max_iterations < n) ? raw_max_iterations : n;

  pcl::PointXYZ best_fit(0,0,0);
  int best_number_of_fits = 0;
  float best_error = -1;
  float best_fit_fraction = -1;

  if(n < MIN_POINTS) return Gate(); // Avoid small clouds

  for(int i = 0; i < max_iterations; i++){
    // Make random selection
    int p1_index = rand()%(n);
    int p2_index, p3_index;
    do{
        p2_index = rand()%(n);
    }while (p1_index == p2_index);

    // Find circle from selected points
    pcl::PointXYZ p1 = cloud->at(p1_index);
    pcl::PointXYZ p2 = cloud->at(p2_index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr centers = findCircleCenters(&p1, &p2, R, plane);
    if(centers->size() != 2){
      continue; // No centers found!
    } 

    float center_deviation[2] = {0, 0};

    // Count fits
    int center_counts[2] = {0, 0};
    for (size_t center_index = 0; center_index < 2; center_index++)
    {
      if(USE_FULL_CLOUDS){
        for (size_t point_index = 0; point_index < n_full; point_index++)
        {
            float distance = distanceBetweenPoint(&(centers->at(center_index)), &(cloud_full->at(point_index)));
            if(fabs(distance - R) <= max_deviation){
              center_counts[center_index]++;
              center_deviation[center_index] += fabs(distance - R)*fabs(distance - R);
            }
        }
      }else{
        for (size_t point_index = 0; point_index < n; point_index++)
        {
            float distance = distanceBetweenPoint(&(centers->at(center_index)), &(cloud->at(point_index)));
            if(fabs(distance - R) <= max_deviation){
              center_counts[center_index]++;
              center_deviation[center_index] += fabs(distance - R)*fabs(distance - R);
            } 
        }
      }
    }

    // Update best fit
    int best_center_index = (center_counts[0] > center_counts[1]) ? 0 : 1;

    float fit_fraction = (float) center_counts[best_center_index] / (float) n_full;
    float fit_error = center_deviation[best_center_index] / (float) n_full; // Std. deviation

    if(center_counts[best_center_index] >= MIN_FITS && fit_fraction >= MIN_FITS_FRACTION && best_number_of_fits <= center_counts[best_center_index]){
      // Check if same number of fits
      if(best_number_of_fits == center_counts[best_center_index]){
        // Only update if better fit
        if(fit_error < best_error){
          best_number_of_fits = center_counts[best_center_index];
          best_fit = centers->at(best_center_index);
          best_error = center_deviation[best_center_index] / (float) n_full;
          best_fit_fraction = fit_fraction;
        }
      } else{
        best_number_of_fits = center_counts[best_center_index];
        best_fit = centers->at(best_center_index);
        best_error = center_deviation[best_center_index] / (float) n_full;
        best_fit_fraction = fit_fraction;
      }
    }
  }

  if(best_number_of_fits > 0){
    return Gate(best_fit, best_fit_fraction, best_error, best_number_of_fits);
  }else{
    // No accepted fit found!
    return Gate(); // Will be 0,0,0
  }
}
