//
// Created by dlsh on 2020/9/22.
//

#ifndef SRC_COSTMAPLAYER_H
#define SRC_COSTMAPLAYER_H
#include <ros/ros.h>
#include <costmap/costLayer_base.h>
#include <costmap/costLayer_manager.h>

namespace costmap{
    class costmapLayer:public costLayer_base,public costmap_base{
    public:
        costmapLayer();
        virtual void matchParentSize();
    //private:
    };
}




#endif //SRC_COSTMAPLAYER_H
