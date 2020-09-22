//
// Created by dlsh on 2020/9/22.
//

#include "costmap/costmapLayer.h"

namespace costmap{
    void costmapLayer::matchParentSize() {
        costmap_base* master=manager_->getCostmapMaster();
        resizeMap(master->get_mapSizeX(),master->get_mapSizeY(),master->get_Resolution(),master->get_originX(),master->get_originY());

    }
}