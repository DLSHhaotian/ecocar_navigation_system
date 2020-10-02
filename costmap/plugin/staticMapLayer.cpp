//
// Created by dlsh on 2020/9/22.
//

#include <costmap/staticMapLayer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


PLUGINLIB_EXPORT_CLASS(costmap::staticMapLayer, costmap::costmapLayer);

using costmap::NO_INFORMATION;
using costmap::LETHAL_OBSTACLE;
using costmap::FREE_SPACE;
namespace costmap{
    void staticMapLayer::ownInitialize() {
        ros::NodeHandle nh("~/"+name_),g_nh;
        current_=true;

        globalFrame_=manager_->getGlobalFrame();
        std::string inputMap_topic;
        nh.param("map_topic",inputMap_topic,std::string("map"));
        nh.param("first_map_only", first_map_only_, false);
        //subscribe the input map
        ROS_INFO("Requesting the map");
        map_sub_=g_nh.subscribe(inputMap_topic,1,&staticMapLayer::inputMap,this);
        mapReceived_= false;
        dataUpdated_ = false;

        ros::Rate loopRate(10);
        while(!mapReceived_&&g_nh.ok()){
            ros::spinOnce();
            loopRate.sleep();
        }
        ROS_INFO("Received a %d X %d map at %f m/pix",get_mapSizeX_meter(),get_mapSizeY_meter(),get_Resolution());

    }
    void staticMapLayer::inputMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
        unsigned int sizeX=new_map->info.width;
        unsigned int sizeY=new_map->info.height;
        ROS_DEBUG("Received a %d X %d map at %f m/pix",sizeX,sizeY);
        costmap_base* master=manager_->getCostmapMaster();
        if(manager_->isLocalMap()&&(sizeX_!=sizeX||sizeY_!=sizeY||resolution_!=new_map->info.resolution||originX_!=new_map->info.origin.position.x||originY_!=new_map->info.origin.position.y)){
            ROS_INFO("Resizing static layer to %d X %d at %f m/pix", sizeX, sizeY, new_map->info.resolution);
            resizeMap(sizeX,sizeY,new_map->info.resolution,new_map->info.origin.position.x,new_map->info.origin.position.y);
        }
        unsigned index=0;
        for(unsigned int i=0;i<sizeY;++i){
            for(unsigned int j=0;j<sizeX;++j){
                unsigned costValue=new_map->data[index];
                mapChar_[index]=interpretValue(costValue);
                ++index;
            }
        }
        mapFrame_=new_map->header.frame_id;
        staticX_=0;
        staticY_=0;
        staticSizeX_=sizeX;
        staticSizeY_=sizeY;
        mapReceived_=true;
        dataUpdated_=true;

        if(first_map_only_){
            ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
            map_sub_.shutdown();
        }

    }
    unsigned char staticMapLayer::interpretValue(unsigned char value) {
        if(value==-1)//nav_msgs::OccupancyGrid -1=no info
            return NO_INFORMATION;//255
        else if(value==0)//0=free
            return FREE_SPACE;//0
        else if(value>=100)//100=completely occupied
            return LETHAL_OBSTACLE;//254
        double scale=(double)value/100;
        return scale*LETHAL_OBSTACLE;
    }
    void staticMapLayer::updateBound(double robotX, double robotY, double robotTheta, double *minX, double *minY,
                                     double *maxX, double *maxY) {
        if(!mapReceived_||!dataUpdated_)
            return;
        double worldX=0,worldY=0;
        mapToWorld(staticX_,staticY_,worldX,worldY);
        *minX=std::min(worldX,*minX);
        *minY=std::min(worldY,*minY);

        mapToWorld(staticX_+staticSizeX_,staticY_+staticSizeY_,worldX,worldY);
        *maxX=std::max(worldX,*maxX);
        *maxY=std::max(worldY,*maxY);

        dataUpdated_=false;

    }
    void staticMapLayer::updateCost(costmap_base &master_grid, int boundX0, int boundY0, int boundXn, int boundYn) {
        if(!mapReceived_)
            return;
        if(manager_->isLocalMap()){
            unsigned int mapX=0,mapY=0;
            double worldX=0,worldY=0;
            geometry_msgs::TransformStamped transform;
            try{
                transform=tf_->lookupTransform(mapFrame_,globalFrame_,ros::Time(0));
            }
            catch(tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
                return;
            }
            //convert type of transformation
            tf2::Transform tf2_tranform;
            tf2::convert(transform.transform,tf2_tranform);
            //write the cost into master map
            for(unsigned int index_x=boundX0;index_x<boundXn;++index_x){
                for(unsigned int index_y=boundY0;index_y<boundYn;++index_y){
                    //convert master map coordinate to world coordinate
                    master_grid.mapToWorld(index_x,index_y,worldX,worldY);
                    //transform from global_frame to map_frame(static map frame)(in world coordinate)
                    tf2::Vector3 point(worldX,worldY,0);
                    point=tf2_tranform*point;
                    //convert from world coordinate to map coordinate
                    if(worldToMap(point.x(),point.y(),mapX,mapY)){
                        master_grid.set_cost(index_x,index_y,get_cost(mapX,mapY));
                    }
                }
            }
        }
    }
}