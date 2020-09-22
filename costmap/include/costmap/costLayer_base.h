//
// Created by dlsh on 2020/9/21.
//

#ifndef SRC_COSTLAYER_BASE_H
#define SRC_COSTLAYER_BASE_H



#include <costmap/costmap_base.h>
#include <cstring>
#include <tf2_ros/buffer.h>

namespace costmap{
    class costLayer_manager;

    class costLayer_base{
    public:
        //default constructor
        costLayer_base();
        virtual ~costLayer_base(){}
        void initialize(costLayer_manager* parent,std::string name,tf2_ros::Buffer *tf);
        virtual void updateBound(double robotX, double robotY, double robotTheta, double* minX, double* minY,
                         double* maxX, double* maxY){}
        virtual void updateCost(costmap_base& master_grid, int boundX0, int boundY0, int boundXn, int boundYn){}

        virtual void matchParentSize(){}
        virtual void deactivate(){}
        virtual void activate(){}
        virtual void reset() {}
        bool isCurrent() const;
        std::string getName() const;
    protected:
        //called at the end of initialize()
        //Each plugin layer can rewrite its own initialization function(owninitialize())
        virtual void ownInitialize(){}
        costLayer_manager* manager_;
        bool current_;
        std::string name_;
        tf2_ros::Buffer* tf_;
    };
}//namespace cstmap end

#endif //SRC_COSTLAYER_BASE_H