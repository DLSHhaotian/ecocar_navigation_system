//
// Created by dlsh on 2020/9/22.
//

#include "costmap/costmapLayer.h"

namespace costmap{
    void costmapLayer::matchParentSize() {
        costmap_base* master=manager_->getCostmapMaster();
        resizeMap(master->get_mapSizeX(),master->get_mapSizeY(),master->get_Resolution(),master->get_originX(),master->get_originY());
    }
    void costmapLayer::updateBoundsForPoint(double x, double y, double *minX, double *minY, double *maxX,
                                            double *maxY) {
        *minX=std::min(x,*minX);
        *minY=std::min(y,*minY);
        *maxX=std::max(x,*maxX);
        *maxY=std::max(y,*maxY);
    }
    void costmapLayer::updateWithOverWrite(costmap::costmap_base &master, int minX, int minY, int maxX, int maxY) {
        unsigned char* masterMap=master.get_mapChar();
        unsigned int span=master.get_mapSizeX();

        for(int j=minY;j<maxY;++j){
            unsigned int it=span*j+minX;
            for(int i=minX;i<maxX;++i){
                masterMap[it]=mapChar_[it];
                ++it;
            }
        }
    }
}