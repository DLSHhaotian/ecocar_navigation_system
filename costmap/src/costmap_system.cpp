//
// Created by dlsh on 2020/10/2.
//

#include "costmap/costmap_system.h"
#include <costmap/costLayer_manager.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace costmap{
    void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name, bool should_delete = true)
    {
        if (!old_h.hasParam(name))
            return;

        XmlRpc::XmlRpcValue value;
        old_h.getParam(name, value);
        new_h.setParam(name, value);
        if (should_delete) old_h.deleteParam(name);
    }




    costmap_system::costmap_system(const std::string &name, tf2_ros::Buffer &tf) :
    manager_(NULL),
    name_(name),
    tf_(tf),
    transform_tolerance_(0.3),
    map_update_thread_shutdown_(false),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
    plugin_loader_("costmap","costmap::costLayer_base"){
        tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle g_nh;

        // get global and robot base frame names
        private_nh.param("global_frame", global_frame_, std::string("map"));
        private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

        ros::Time last_error = ros::Time::now();
        std::string tf_error;
        // we need to make sure that the transform between the robot base frame and the global frame is available
        while (ros::ok()
               && !tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error))
        {
            ros::spinOnce();
            if (last_error + ros::Duration(5.0) < ros::Time::now())
            {
                ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
                         robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
                last_error = ros::Time::now();
            }
            // The error string will accumulate and errors will typically be the same, so the last
            // will do for the warning above. Reset the string here to avoid accumulation.
            tf_error.clear();
        }

        bool rolling_window, always_send_full_costmap;
        private_nh.param("rolling_window", rolling_window, false);
        private_nh.param("always_send_full_costmap", always_send_full_costmap, false);

        manager_=new costLayer_manager(global_frame_,rolling_window);
        if (!private_nh.hasParam("plugins"))
        {
            loadOldParameters(private_nh);
        }

        if (private_nh.hasParam("plugins"))
        {
            XmlRpc::XmlRpcValue my_list;
            private_nh.getParam("plugins", my_list);
            for (int32_t i = 0; i < my_list.size(); ++i)
            {
                std::string pname = static_cast<std::string>(my_list[i]["name"]);
                std::string type = static_cast<std::string>(my_list[i]["type"]);
                ROS_INFO("%s: Using plugin \"%s\"", name_.c_str(), pname.c_str());

                copyParentParameters(pname, type, private_nh);

                boost::shared_ptr<costLayer_base> plugin = plugin_loader_.createInstance(type);
                manager_->addPlugin(plugin);
                plugin->initialize(manager_, name + "/" + pname, &tf_);
            }
        }

        private_nh.param("transform_tolerance_", transform_tolerance_, 0.3);
        if (map_update_thread_ != NULL)
        {
            map_update_thread_shutdown_ = true;
            map_update_thread_->join();
            delete map_update_thread_;
        }
        map_update_thread_shutdown_ = false;
        double map_update_frequency=0,map_publish_frequency=0;
        private_nh.param("map_update_frequency", map_update_frequency, 10.0);
        private_nh.param("map_publish_frequency", map_publish_frequency, 10.0);

        if (map_publish_frequency > 0)
            publish_cycle = ros::Duration(1 / map_publish_frequency);
        else
            publish_cycle = ros::Duration(-1);
        double map_width_meters=0,map_height_meters=0,resolution=0,origin_x=0,origin_y=0;
        private_nh.param("map_width_meters", map_width_meters, 6.5);
        private_nh.param("map_height_meters", map_height_meters, 6.5);
        private_nh.param("resolution", resolution, 10.0);
        private_nh.param("origin_x", origin_x, 0.0);
        private_nh.param("origin_y", origin_y, 0.0);


        if (!manager_->isSizeLocked())
        {
            manager_->resizeMap((unsigned int)(map_width_meters / resolution),
                                        (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
        }

        map_update_thread_ = new boost::thread(boost::bind(&costmap_system::mapUpdateLoop, this, map_update_frequency));

    }

    void costmap_system::loadOldParameters(ros::NodeHandle& nh)
    {
        ROS_WARN("%s: Parameter \"plugins\" not provided, loading pre-Hydro parameters", name_.c_str());
        bool flag;
        std::string s;
        std::vector < XmlRpc::XmlRpcValue > plugins;

        XmlRpc::XmlRpcValue::ValueStruct map;
        SuperValue super_map;
        SuperValue super_array;

        if (nh.getParam("static_map", flag) && flag) {
            map["name"] = XmlRpc::XmlRpcValue("static_layer");
            map["type"] = XmlRpc::XmlRpcValue("costmap::staticMapLayer");
            super_map.setStruct(&map);
            plugins.push_back(super_map);

            ros::NodeHandle map_layer(nh, "static_layer");
            move_parameter(nh, map_layer, "map_topic");
            move_parameter(nh, map_layer, "unknown_cost_value");
            move_parameter(nh, map_layer, "lethal_cost_threshold");
        }
        ros::NodeHandle obstacles(nh, "obstacle_layer");
        map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
        map["type"] = XmlRpc::XmlRpcValue("costmap::obstacleMapLayer");
        super_map.setStruct(&map);
        plugins.push_back(super_map);


        move_parameter(nh, obstacles, "max_obstacle_height");
        move_parameter(nh, obstacles, "raytrace_range");
        move_parameter(nh, obstacles, "obstacle_range");
        nh.param("observation_sources", s, std::string(""));
        std::stringstream ss(s);
        std::string source;
        while (ss >> source)
        {
            move_parameter(nh, obstacles, source);
        }
        move_parameter(nh, obstacles, "observation_sources");
        super_array.setArray(&plugins);
        nh.setParam("plugins", super_array);
    }

    void costmap_system::copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh)
    {
        ros::NodeHandle target_layer(nh, plugin_name);

        if(plugin_type == "costmap::staticMapLayer")
        {
            move_parameter(nh, target_layer, "map_topic", false);
            move_parameter(nh, target_layer, "unknown_cost_value", false);
            move_parameter(nh, target_layer, "lethal_cost_threshold", false);
        }
/*        else if(plugin_type == "costmap_2d::VoxelLayer")
        {
            move_parameter(nh, target_layer, "origin_z", false);
            move_parameter(nh, target_layer, "z_resolution", false);
            move_parameter(nh, target_layer, "z_voxels", false);
            move_parameter(nh, target_layer, "mark_threshold", false);
            move_parameter(nh, target_layer, "unknown_threshold", false);
            move_parameter(nh, target_layer, "publish_voxel_map", false);
        }*/
        else if(plugin_type == "costmap_2d::ObstacleLayer")
        {
            move_parameter(nh, target_layer, "max_obstacle_height", false);
            move_parameter(nh, target_layer, "raytrace_range", false);
            move_parameter(nh, target_layer, "obstacle_range", false);
        }
/*        else if(plugin_type == "costmap_2d::InflationLayer")
        {
            move_parameter(nh, target_layer, "cost_scaling_factor", false);
           move_parameter(nh, target_layer, "inflation_radius", false);
         }*/
    }
    void costmap_system::mapUpdateLoop(double frequency) {
        if (frequency == 0.0)
            return;
        ros::NodeHandle nh;
        ros::Rate r(frequency);
        while (nh.ok() && !map_update_thread_shutdown_)
        {
            updateMap();
            if (publish_cycle.toSec() > 0 && manager_->isInitialized())
            {
                unsigned int x0, y0, xn, yn;
                manager_->getBound(x0, xn, y0, yn);
                //publisher_->updateBounds(x0, xn, y0, yn);

                ros::Time now = ros::Time::now();
                if (last_publish_ + publish_cycle < now)
                {
                    //publisher_->publishCostmap();
                    last_publish_ = now;
                }
            }
            r.sleep();
            // make sure to sleep for the remainder of our cycle time
            if (r.cycleTime() > ros::Duration(1 / frequency))
                ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
                         r.cycleTime().toSec());
        }
    }
    void costmap_system::updateMap()
    {
        if (!stop_updates_)
        {
            // get global pose
            geometry_msgs::PoseStamped pose;
            if (getRobotPose (pose))
            {
                double x = pose.pose.position.x,
                        y = pose.pose.position.y,
                        yaw = tf2::getYaw(pose.pose.orientation);

                manager_->updateMap(x, y, yaw);

                initialized_ = true;
            }
        }
    }
    bool costmap_system::isCurrent() const {
        return manager_->isCurrent();
    }
    bool costmap_system::isStopped() const {
        return stopped_;
    }
    std::string costmap_system::getGlobalFrameID() const {
        return global_frame_;
    }
    std::string costmap_system::getBaseFrameID() const {
        return robot_base_frame_;
    }
    std::string costmap_system::getName() const {
        return name_;
    }
    double costmap_system::getTransformTolerance() const {
        return transform_tolerance_;
    }
    costmap_base* costmap_system::getCostmap() const {
        return manager_->getCostmapMaster();
    }
    costLayer_manager* costmap_system::getCostLayerManager() const {
        return manager_;
    }
    bool costmap_system::getRobotPose(geometry_msgs::PoseStamped& global_pose) const
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time();
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

        // get the global pose of the robot
        try
        {
            tf_.transform(robot_pose, global_pose, global_frame_);
        }
        catch (tf2::LookupException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        // check global_pose timeout
        if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                              current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
            return false;
        }

        return true;
    }


}
