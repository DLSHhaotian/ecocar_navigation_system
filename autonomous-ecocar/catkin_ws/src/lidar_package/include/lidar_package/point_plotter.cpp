#include "point_plotter.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * clouds, std::vector<int> * color_id_vector){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    int size = 0;
    for (size_t i = 0; i < clouds->size(); i++)
    {
        size += clouds->at(i)->size();
    }
    coloredPoints->points.resize(size);
    int n = 0;
    for (size_t i = 0; i < clouds->size(); i++)
    {
        for (size_t j = 0; j < clouds->at(i)->size(); j++)
        {
            coloredPoints->points[n].x = clouds->at(i)->at(j).x;
            coloredPoints->points[n].y = clouds->at(i)->at(j).y;
            coloredPoints->points[n].z = clouds->at(i)->at(j).z;

            if(color_id_vector){ // If given a vector with IDs to determine colors (ex. to keep cloud in an obstalc the same color)
                if(color_id_vector->at(i) == -1){ // Outlier
                    coloredPoints->points[n].r = 0xFF;
                    coloredPoints->points[n].g = 0x00;
                    coloredPoints->points[n].b = 0x00;
                } else if(color_id_vector->at(i) == -2){ // Ignored segment
                    coloredPoints->points[n].r = 0xFF;
                    coloredPoints->points[n].g = 0xFF;
                    coloredPoints->points[n].b = 0xFF;
                } else{
                    int color_index = (color_id_vector->at(i) % (NUMBER_OF_DISTINCT_COLORS - 1)) + 1; // Avoid using the first color
                    coloredPoints->points[n].r = (distinct_colors[color_index] >> 16) & 0xFF;
                    coloredPoints->points[n].g = (distinct_colors[color_index] >> 8) & 0xFF;
                    coloredPoints->points[n].b = (distinct_colors[color_index] >> 0) & 0xFF;
                }
            } else {
                coloredPoints->points[n].r = (distinct_colors[i] >> 16) & 0xFF;
                coloredPoints->points[n].g = (distinct_colors[i] >> 8) & 0xFF;
                coloredPoints->points[n].b = (distinct_colors[i] >> 0) & 0xFF;
            }


            n++;
        }
    }
    coloredPoints->header.frame_id = FRAME_ID;
    // coloredPoints->header.stamp = ros::Time::now().toNSec(); // TODO: This causes std::runtime_error if rviz is running, need fix
    return coloredPoints;
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredPoints (new pcl::PointCloud<pcl::PointXYZRGB>);
    int size = cloud->size();
    coloredPoints->resize(size);
        for (size_t i = 0; i < cloud->size(); i++)
        {
            coloredPoints->points[i].x = cloud->at(i).x;
            coloredPoints->points[i].y = cloud->at(i).y;
            coloredPoints->points[i].z = cloud->at(i).z;
            coloredPoints->points[i].r = r & 0xFF;
            coloredPoints->points[i].g = g & 0xFF;
            coloredPoints->points[i].b = b & 0xFF;
        }
    coloredPoints->header.frame_id = FRAME_ID;
    return coloredPoints;
}
visualization_msgs::Marker makePoints(pcl::PointCloud<pcl::PointXYZ> * pointList, float r, float g, float b, std::string ns, int id, float alpha, float size){
    visualization_msgs::Marker points;
    points.header.frame_id = FRAME_ID;
    points.header.stamp = ros::Time::now();
    points.ns = ns; // namespace for this marker
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = id;
    points.type = visualization_msgs::Marker::POINTS;

    // Color
    points.color.r = r;
    points.color.b = g;
    points.color.g = b;
    points.color.a = alpha;

    // Size
    points.scale.x = size;
    points.scale.y = size;

    for(int i = 0; i < pointList->size(); i++){
        geometry_msgs::Point p;
        p.x = pointList->points.at(i).x;
        p.y = pointList->points.at(i).y;
        p.z = pointList->points.at(i).z;
        points.points.push_back(p);
    }
    return points;
}

visualization_msgs::Marker makeLineStrip(pcl::PointCloud<pcl::PointXYZ> * pointList, float r, float g, float b, std::string ns, int id, bool loop, float thickness){
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = FRAME_ID;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = ns; // namespace for this marker
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = thickness; // Thickness

    // Color
    line_strip.color.r = r;
    line_strip.color.b = g;
    line_strip.color.g = b;
    if(pointList == 0){
        line_strip.color.a = 0;
    }else{
        line_strip.color.a = 0.8;

        for(int i = 0; i < pointList->size(); i++){
            geometry_msgs::Point p;
            p.x = pointList->points.at(i).x;
            p.y = pointList->points.at(i).y;
            p.z = pointList->points.at(i).z;
            line_strip.points.push_back(p);
        }
        if(loop){
            geometry_msgs::Point p;
            p.x = pointList->points.at(0).x;
            p.y = pointList->points.at(0).y;
            p.z = pointList->points.at(0).z;
            line_strip.points.push_back(p);
        }
    }
    return line_strip;
}

visualization_msgs::Marker makeLineList(pcl::PointCloud<pcl::PointXYZ> * pointListA, pcl::PointCloud<pcl::PointXYZ> * pointListB, float r, float g, float b, float thickness, float alpha, std::string ns, int id){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = FRAME_ID;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns; // namespace for this marker
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = thickness; // Thickness

    // Color
    line_list.color.r = r;
    line_list.color.b = g;
    line_list.color.g = b;
    line_list.color.a = alpha;

    int n = pointListA->size();
    if(n != pointListB->size()){
        ROS_WARN("Line-lists does not match in lenght!");
        return line_list;
    }
    for(int i = 0; i < n; i++){
        geometry_msgs::Point p;
        p.x = pointListA->points.at(i).x;
        p.y = pointListA->points.at(i).y;
        p.z = pointListA->points.at(i).z;
        line_list.points.push_back(p);
        p.x = pointListB->points.at(i).x;
        p.y = pointListB->points.at(i).y;
        p.z = pointListB->points.at(i).z;
        line_list.points.push_back(p);
    }
    return line_list;
}

visualization_msgs::Marker makePlanes(pcl::PointCloud<pcl::PointXYZ> * pointList, float r, float g, float b, bool reverse, std::string ns, int id){
    //Uses sets of 4 point to make planes

    visualization_msgs::Marker plane;
    plane.header.frame_id = FRAME_ID;
    plane.header.stamp = ros::Time::now();
    plane.ns = ns; // namespace for this marker
    plane.action = visualization_msgs::Marker::ADD;
    //plane.pose.orientation.w = 1.0;
    plane.id = id;
    plane.type = visualization_msgs::Marker::TRIANGLE_LIST;

    // Color
    plane.color.r = r;
    plane.color.b = g;
    plane.color.g = b;
    plane.color.a = 1.0;

    // Size
    plane.scale.x = 1.0;
    plane.scale.y = 1.0;
    plane.scale.z = 1.0;

    if(reverse){
        for(int i = 0; i < pointList->size() -4; i = i + 4){
            geometry_msgs::Point p;

            //First triangle
            p.x = pointList->points.at(i+1).x;
            p.y = pointList->points.at(i+1).y;
            p.z = pointList->points.at(i+1).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i).x;
            p.y = pointList->points.at(i).y;
            p.z = pointList->points.at(i).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+2).x;
            p.y = pointList->points.at(i+2).y;
            p.z = pointList->points.at(i+2).z;
            plane.points.push_back(p);

            //Second triangle
            p.x = pointList->points.at(i+1).x;
            p.y = pointList->points.at(i+1).y;
            p.z = pointList->points.at(i+1).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+2).x;
            p.y = pointList->points.at(i+2).y;
            p.z = pointList->points.at(i+2).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+3).x;
            p.y = pointList->points.at(i+3).y;
            p.z = pointList->points.at(i+3).z;
            plane.points.push_back(p);
        }
    } else {
        for(int i = 0; i + 3 < pointList->size(); i = i + 4){
            geometry_msgs::Point p;

            //First triangle
            p.x = pointList->points.at(i).x;
            p.y = pointList->points.at(i).y;
            p.z = pointList->points.at(i).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+1).x;
            p.y = pointList->points.at(i+1).y;
            p.z = pointList->points.at(i+1).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+2).x;
            p.y = pointList->points.at(i+2).y;
            p.z = pointList->points.at(i+2).z;
            plane.points.push_back(p);

            //Second triangle
            p.x = pointList->points.at(i+2).x;
            p.y = pointList->points.at(i+2).y;
            p.z = pointList->points.at(i+2).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+1).x;
            p.y = pointList->points.at(i+1).y;
            p.z = pointList->points.at(i+1).z;
            plane.points.push_back(p);
            p.x = pointList->points.at(i+3).x;
            p.y = pointList->points.at(i+3).y;
            p.z = pointList->points.at(i+3).z;
            plane.points.push_back(p);
        }
    }
    return plane;
}

visualization_msgs::Marker makeGrids(pcl::PointCloud<pcl::PointXYZ> * pointList, int n, float r, float g, float b, float thickness, std::string ns, int id){
    visualization_msgs::Marker grids;
    grids.header.frame_id = FRAME_ID;
    grids.header.stamp = ros::Time::now();
    grids.ns = ns; // namespace for this marker
    grids.action = visualization_msgs::Marker::ADD;
    grids.pose.orientation.w = 1.0;
    grids.id = id;
    grids.type = visualization_msgs::Marker::LINE_LIST;

    grids.scale.x = thickness; // Thickness

    // Color
    grids.color.r = r;
    grids.color.b = g;
    grids.color.g = b;
    grids.color.a = 0.5;

    int list_index = pointList->size();
    if(list_index % 4 != 0){
        ROS_WARN("Grid needs four points per grid!");
        return grids;
    }
    if(n <= 1){
        ROS_WARN("Grid more than one section (n)!");
        return grids;
    }
    geometry_msgs::Point p;
    for(int i = 0; i + 3 < list_index; i = i + 4){
        for(int j = 0; j < n; j++){
            float fraction = ((float) j)/(((float) n)-1.0);

            // P1 -> P3
            p.x = pointList->points.at(i).x + (pointList->points.at(i+2).x - pointList->points.at(i).x)*fraction;
            p.y = pointList->points.at(i).y + (pointList->points.at(i+2).y - pointList->points.at(i).y)*fraction;
            p.z = pointList->points.at(i).z + (pointList->points.at(i+2).z - pointList->points.at(i).z)*fraction;
            grids.points.push_back(p);
            // P2 -> P4
            p.x = pointList->points.at(i+1).x + (pointList->points.at(i+3).x - pointList->points.at(i+1).x)*fraction;
            p.y = pointList->points.at(i+1).y + (pointList->points.at(i+3).y - pointList->points.at(i+1).y)*fraction;
            p.z = pointList->points.at(i+1).z + (pointList->points.at(i+3).z - pointList->points.at(i+1).z)*fraction;
            grids.points.push_back(p);

            // P1 -> P2
            p.x = pointList->points.at(i).x + (pointList->points.at(i+1).x - pointList->points.at(i).x)*fraction;
            p.y = pointList->points.at(i).y + (pointList->points.at(i+1).y - pointList->points.at(i).y)*fraction;
            p.z = pointList->points.at(i).z + (pointList->points.at(i+1).z - pointList->points.at(i).z)*fraction;
            grids.points.push_back(p);
            // P3 -> P4
            p.x = pointList->points.at(i+2).x + (pointList->points.at(i+3).x - pointList->points.at(i+2).x)*fraction;
            p.y = pointList->points.at(i+2).y + (pointList->points.at(i+3).y - pointList->points.at(i+2).y)*fraction;
            p.z = pointList->points.at(i+2).z + (pointList->points.at(i+3).z - pointList->points.at(i+2).z)*fraction;
            grids.points.push_back(p);
        }
    }
    return grids;
}
void makeBoxPoints(Extreme * extreme, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud){
    //Top
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_max, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_min, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_max, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_min, extreme->z_max));
    // Sides
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_max, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_max, extreme->z_min));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_min, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_min, extreme->z_min));
    // Sides
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_max, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_max, extreme->z_min));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_min, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_min, extreme->z_min));      
    // Sides
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_max, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_max, extreme->z_min));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_max, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_max, extreme->z_min)); 
    // Sides
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_min, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_max, extreme->y_min, extreme->z_min));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_min, extreme->z_max));
    point_cloud->push_back(pcl::PointXYZ(extreme->x_min, extreme->y_min, extreme->z_min)); 
}
