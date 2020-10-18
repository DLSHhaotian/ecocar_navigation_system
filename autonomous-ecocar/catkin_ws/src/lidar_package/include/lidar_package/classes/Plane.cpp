#include "Plane.h"

Plane::Plane(){
    is_set = false;
    is_nomalized = false;
};
Plane::Plane(pcl::PointXYZ * p_1, pcl::PointXYZ * p_2, pcl::PointXYZ * p_3){
    float u_1 = p_2->x - p_1->x;
    float u_2 = p_2->y - p_1->y;
    float u_3 = p_2->z - p_1->z;
    float v_1 = p_3->x - p_1->x;
    float v_2 = p_3->y - p_1->y;
    float v_3 = p_3->z - p_1->z;
    a = u_2*v_3 - v_2*u_3; //i
    b = u_3*v_1 - v_3*u_1; //j
    c = u_1*v_2 - v_1*u_2; //k
    d = -(a*p_1->x + b*p_1->y + c*p_1->z);
    length = sqrt(a*a + b*b + c*c);
    is_set = true;
    is_nomalized = false;
};
Plane::Plane(float input_a, float input_b, float input_c){
    // a*x + b*y + c = z
    a = input_a;
    b = input_b;
    c = -1;
    d = input_c;
    length = sqrt(a*a + b*b + c*c);
    is_set = true;
    is_nomalized = false;
};
Plane::Plane(float input_a, float input_b, float input_c, float input_d){
    // a*x + b*y + c*z + d = 0
    a = input_a;
    b = input_b;
    c = input_c;
    d = input_d;
    length = sqrt(a*a + b*b + c*c);
    is_set = true;
    is_nomalized = false;
};
void Plane::Normalize(){
    if(is_set && !is_nomalized){
    a = a/length;
    b = b/length;
    c = c/length;
    d = d/length;
    length = 1;
    if(c < 0){ // Flip if negative normal vector
        a = -a;
        b = -b;
        c = -c;
        d = -d;
    }
    is_nomalized = true;
    }
}
float Plane::getLength(){
    if(is_set){
    return length;
    }else{
        ROS_WARN("Plane not set!");
        return -1;
    };
};
pcl::PointXYZ Plane::getNormalVector(){
    Normalize();
    pcl::PointXYZ p(a, b, c);
    return p;
}
float Plane::DistToPoint(pcl::PointXYZ * p){
    if(is_set){
        return fabs(a*(p->x) + b*(p->y) + c*(p->z) + d)/length;
    }else{
        ROS_WARN("Plane not set!");
        return -1;
    };
};
pcl::PointXYZ Plane::projectOntoPlane(pcl::PointXYZ * p){
    Normalize();
    float dist = DistToPoint(p);
    pcl::PointXYZ result;
    if(dist <= 0.001){
    result.x = p->x;
    result.y = p->y;
    result.z = p->z;
    return result;
    } else{
    result.x = p->x - (dist*a);
    result.y = p->y - (dist*b);
    result.z = p->z - (dist*c);
    return result;
    }

}
bool Plane::abovePlane(pcl::PointXYZ * p){ // in respect to Z
    if(c == 0) return false;
    float temp = -a*(p->x) - b*(p->y) - d;
    if(c > 0){
    if(temp >= 0) return true;
    } else {
    if(temp < 0) return true;
    }
    return false;
}
float Plane::SqErrToPLC(pcl::PointCloud<pcl::PointXYZ> * plc){
    if(is_set){
    float sq_err = 0;
    for(int point_index = 0; point_index < plc->size(); point_index++){
        float dist = DistToPoint(&plc->at(point_index));
        sq_err += dist * dist;
    }
    return sq_err;
    }else{
        ROS_WARN("Plane not set!");
        return -1;
    };
}

void Plane::filterDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float distance, pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_output){
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers (new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);    

    if(inliers_output){ // Calculate inliers too
    for(int i = 0; i < point_cloud->size(); i++){
        if(DistToPoint(&point_cloud->at(i)) <= distance) inliers->push_back(point_cloud->at(i));
        else outliers->push_back(point_cloud->at(i));
    } 
    point_cloud->clear();
    * point_cloud += *outliers;
    inliers_output->clear();
    * inliers_output += * inliers;
    }else{
    for(int i = 0; i < point_cloud->size(); i++){
        if(DistToPoint(&point_cloud->at(i)) > distance) outliers->push_back(point_cloud->at(i));
    } 
    point_cloud->clear();
    * point_cloud += *outliers;
    }
}

float Plane::getAngleToNorm(float x, float y, float z){
    float result = acos((a*x + b*y + c*z)/(length*sqrt(x*x + y*y + z*z)));
    if(result > PI/2) result = PI - result;
    return result;
};
float Plane::getAngleBetweenPlanes(Plane * other_plane){
    float result = acos((a*other_plane->a + b*other_plane->b + c*other_plane->c)/(length*other_plane->getLength()));
    if(result > PI/2) result = PI - result;
    return result;
};
float Plane::getZ(float x, float y){
    return -(a*x + b*y + d)/c;
};
pcl::PointXYZ Plane::getXYPoint(float x, float y){
    pcl::PointXYZ p;
    p.x = x;
    p.y = y;
    p.z = getZ(x, y);
    return p;
};
void Plane::clear(){
    is_set = false;
};
bool Plane::isSet(){
    return is_set;
};

