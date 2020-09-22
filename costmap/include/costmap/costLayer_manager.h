// Created by dlsh on 2020/9/21.
//

#ifndef SRC_COSTLAYER_MANAGER_H
#define SRC_COSTLAYER_MANAGER_H
#include <costmap/costLayer_base.h>
#include <costmap/costmap_base.h>
#include <vector>
#include <cstring>

namespace costmap{
    class costLayer_base;
    class costLayer_manager{
        //constructor
    public:
        costLayer_manager(std::string global_frame, bool local_map);
        ~costLayer_manager();

        void updateMap(double robotX,double robotY,double robotTheta);
        void resizeMap(unsigned int sizeX, unsigned int sizeY, double resolution, double originX, double originY,
                       bool sizeLocked = false);
        void addPlugin(boost::shared_ptr<costLayer_base> plugin);

        void getBound(double& minX, double& minY, double& maxX, double& maxY);
        bool isCurrent();
        bool isLocalMap();
        bool isInitialized();
        bool isSizeLocked();
        costmap_base* getCostmapMaster();
        std::vector<boost::shared_ptr<costLayer_base>>* getPlugins();
        std::string getGlobalFrame() const;


    private:
        costmap_base costmapMaster_;
        std::string globalFrame_;
        bool ifLocalMap_;
        bool current_;
        bool initialized_;
        bool sizeLocked_;
        double minX_,maxX_,minY_,maxY_;
        unsigned int boundX0_,boundXn_,boundY0_,boundYn_;

        std::vector<boost::shared_ptr<costLayer_base>> plugins_;
    };
}
#endif //SRC_COSTLAYER_MANAGER_H
