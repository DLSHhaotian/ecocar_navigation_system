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
        //if the local map is needed, the origin has to be computed
        if(ifLocalMap_){
            //The local map only contains the part in front of the robot
            double new_originX=robotX-costmapMaster_.get_mapSizeX_meter()/2;
            double new_originY=robotY;
            costmapMaster_.updateOrigin(new_originX,new_originY);
        }
    }

}//namespace costmap end