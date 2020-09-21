//
// Created by dlsh on 2020/9/21.
//

#include "costmap/costLayer_base.h"
namespace costmap{
    costLayer_base::costLayer_base() :manager_(nullptr),current_(false),name_(),tf_(nullptr){}

    void costLayer_base::initialize(costLayer_manager *parent, std::string name, tf2_ros::Buffer *tf) {
        manager_=parent;
        name_=name;
        tf_=tf;
        ownInitialize();
    }

    std::string costLayer_base::getName() const {return name_;}
    bool costLayer_base::isCurrent() const {return current_;}
}//namespace costmap end