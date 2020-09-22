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
        std::string inputMao_topic;
        nh.param("map_topic",inputMao_topic,std::string("map"));
        nh.param("first_map_only", first_map_only_, false);
        //subscribe the input map
        ROS_INFO("Requesting the map");
        map_sub_=g_nh.subscribe(inputMao_topic,1,&staticMapLayer::inputMap,this);
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
}