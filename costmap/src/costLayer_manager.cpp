//
// Created by dlsh on 2020/9/21.
//

#include "costmap/costLayer_manager.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

namespace costmap{
    costLayer_manager::costLayer_manager(std::string global_frame, bool local_map) :
    costmapMaster_(),
    globalFrame_(global_frame),
    ifLocalMap_(local_map),
    current_(false),
    minX_(0.0),
    minY_(0.0),
    maxX_(0.0),
    maxY_(0.0),
    boundX0_(0),
    boundXn_(0),
    boundY0_(0),
    boundYn_(0),
    initialized_(false),
    sizeLocked_(false){
        costmapMaster_.set_costDefaultValue(255);
    }

    costLayer_manager::~costLayer_manager() {
        while(plugins_.size()>0){
            plugins_.pop_back();
        }
    }

    void costLayer_manager::updateMap(double robotX, double robotY, double robotTheta) {
        boost::unique_lock<boost::recursive_mutex> lock(*(costmapMaster_.get_mutex()));

        if(plugins_.size()==0){return;}
        //if the local map is needed, the origin has to be computed
        if(ifLocalMap_){

            double new_originX=robotX-costmapMaster_.get_mapSizeX_meter()/2;
            double new_originY=robotY-costmapMaster_.get_mapSizeY_meter()/2;
            costmapMaster_.updateOrigin(new_originX,new_originY);
        }
        //call updatebounds by plugin of each layer
        //compute the updating area bounds
        minX_=1e10;
        minY_=1e10;
        maxX_=-1e10;
        maxY_=-1e10;
        for(std::vector<boost::shared_ptr<costLayer_base>>::iterator plugin=plugins_.begin();plugin!=plugins_.end();++plugin) {
            double minX_pre = minX_;
            double minY_pre = minY_;
            double maxX_pre = maxX_;
            double maxY_pre = maxY_;
            (*plugin)->updateBound(robotX, robotY, robotTheta, &minX_, &minY_, &maxX_, &maxY_);
            if (minX_ > minX_pre || minY_ > minY_pre || maxX_ < maxX_pre || maxY_ < maxY_pre) {
                ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                                       "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                                  minX_pre, minY_pre, maxX_pre, maxY_pre,
                                  minX_, minY_, maxX_, maxY_,
                                  (*plugin)->getName().c_str());
            }
        }
        //convert the region from world coordinate to map index
        int boundX0=0,boundXn=0,boundY0=0,boundYn=0;
        costmapMaster_.worldToMapBounds(minX_,minY_,boundX0,boundY0);
        costmapMaster_.worldToMapBounds(maxX_,maxY_,boundXn,boundYn);

        if(boundX0>boundXn||boundY0>boundYn){
            return;
        }
        else{
            ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", boundX0, boundXn, boundY0, boundYn);
        }
        //reset the updated region
        costmapMaster_.resetMap(boundX0,boundY0,boundXn,boundYn);
        for(std::vector<boost::shared_ptr<costLayer_base>>::iterator plugin=plugins_.begin();plugin!=plugins_.end();++plugin) {
            (*plugin)->updateCost(costmapMaster_,boundX0,boundY0,boundXn,boundYn);
        }
        boundX0_=boundX0;
        boundY0_=boundY0;
        boundXn_=boundXn;
        boundYn_=boundYn;

        initialized_=true;
    }
    void costLayer_manager::resizeMap(unsigned int sizeX, unsigned int sizeY, double resolution, double originX,
                                      double originY, bool sizeLocked) {
        boost::unique_lock<boost::recursive_mutex> lock(*(costmapMaster_.get_mutex()));
        sizeLocked_=sizeLocked;
        costmapMaster_.resizeMap(sizeX,sizeY,resolution,originX,originY);
        for(std::vector<boost::shared_ptr<costLayer_base>>::iterator plugin=plugins_.begin();plugin!=plugins_.end();++plugin){
            (*plugin)->matchParentSize();
        }

    }

    bool costLayer_manager::isCurrent() {
        current_=true;
        for(std::vector<boost::shared_ptr<costLayer_base>>::iterator plugin=plugins_.begin();plugin!=plugins_.end();++plugin){
            current_=current_&&(*plugin)->isCurrent();
        }
        return current_;
    }
    bool costLayer_manager::isSizeLocked() {
        return sizeLocked_;
    }
    bool costLayer_manager::isInitialized() {
        return initialized_;
    }
    bool costLayer_manager::isLocalMap() {
        return ifLocalMap_;
    }

    std::string costLayer_manager::getGlobalFrame() const {
        return globalFrame_;
    }
    std::vector<boost::shared_ptr<costLayer_base>>* costLayer_manager::getPlugins() {
        return &plugins_;
    }
    costmap_base* costLayer_manager::getCostmapMaster() {
        return &costmapMaster_;
    }
    void costLayer_manager::getBound(unsigned int &minX, unsigned int &minY, unsigned int &maxX, unsigned int &maxY) {
        minX=boundX0_;
        minY=boundY0_;
        maxX=boundXn_;
        maxY=boundYn_;
    }
    void costLayer_manager::addPlugin(boost::shared_ptr<costLayer_base> plugin) {
        plugins_.push_back(plugin);
    }
}//namespace costmap end