//
// Created by dlsh on 2020/10/2.
//

#ifndef SRC_COSTMAP_SYSTEM_H
#define SRC_COSTMAP_SYSTEM_H

#include <costmap/costLayer_manager.h>
#include <costmap/costLayer_base.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Transform.h>


class SuperValue : public XmlRpc::XmlRpcValue
{
public:
    void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
    {
        _type = TypeStruct;
        _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
    }
    void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
    {
        _type = TypeArray;
        _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
    }
};


namespace costmap{

    class costmap_system{
    public:
        costmap_system(const std::string &name, tf2_ros::Buffer& tf);
        ~costmap_system();

        void updateMap();
        void resetLayers();
        bool isCurrent() const;
        bool isStopped() const;
        bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;
        std::string getName() const;
        double getTransformTolerance() const;
        costmap_base* getCostmap() const;
        std::string getGlobalFrameID() const;
        std::string getBaseFrameID() const;
        costLayer_manager* getCostLayerManager() const;
    protected:
        costLayer_manager* manager_;
        std::string name_;
        tf2_ros::Buffer& tf_;
        std::string global_frame_;
        std::string robot_base_frame_;
        double transform_tolerance_;
    private:
        void copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh);
        void mapUpdateLoop(double frequency);
        void loadOldParameters(ros::NodeHandle& nh);
        bool map_update_thread_shutdown_;
        bool stop_updates_, initialized_, stopped_, robot_stopped_;
        boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
        ros::Timer timer_;
        ros::Time last_publish_;
        ros::Duration publish_cycle;
        pluginlib::ClassLoader<costLayer_base> plugin_loader_;
        geometry_msgs::PoseStamped old_pose_;

    };

}
#endif //SRC_COSTMAP_SYSTEM_H
