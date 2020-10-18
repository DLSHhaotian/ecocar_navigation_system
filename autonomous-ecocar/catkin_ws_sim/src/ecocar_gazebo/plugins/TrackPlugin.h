/*
    Gazebo plugin responsible for barrier spawning.
    Other things it could do:
        - Detect collision
        - Track lap time
        - Generation of lines or other graphics on track (parking spot)

    Author: Thomas Passer Jensen (s134234@student.dtu.dk)
*/

#include <ros/ros.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "ecocar_gazebo_msgs/Track.h"

#include <random>
#include <vector>

struct gap {
  double pos;
  double length;
};

struct barrier {
  bool red;
  unsigned int index;
  ignition::math::Vector3d pos;
  ignition::math::Quaterniond rot;
  bool spawned;
};

namespace gazebo
{
class TrackPlugin : public WorldPlugin
{
  public:
    TrackPlugin();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
    void onUpdate();
    void onReset();
    void OnContacts(ConstContactsPtr &_msg);

  private:
    physics::WorldPtr world;
    transport::NodePtr node;
    gazebo::physics::ContactManager *cm;
    physics::ModelPtr ecocarModel;
    event::ConnectionPtr updateConnection;
    event::ConnectionPtr resetConnection;
    transport::SubscriberPtr contactSub;
    std::vector<ignition::math::Vector3d> roadPoints;
    //std::vector<physics::ModelPtr> barriers;
    bool loop;
    bool sharpTurns;
    bool spawnLeft, spawnRight;
    double widthGaps;
    unsigned int numGaps;
    double stdGaps;
    std::vector<barrier> barriers;

    bool carCrashed;
    common::Time lastBarrierPubTime;
    common::Time lastCrashPubTime;

    ros::NodeHandle nh;
    ros::Publisher trackPub;
    ros::Publisher crashPub;

    std::string red_barrier_URI;
    std::string gray_barrier_URI;
    std::string ecocarBaseLinkName;
    sdf::SDFPtr redBarrierModelSDF;
    sdf::ElementPtr redBarrierModel;
    sdf::SDFPtr grayBarrierModelSDF;
    sdf::ElementPtr grayBarrierModel;
    unsigned int barrierNumber;
    float barrierLength;
    float barrierWidth;
    double trackWidth;
    //void LineBarrier(float trackWidth, float trackLength, float x0, float y0);
    //void CircleBarrier(float radius, float x0, float y0);
    void publishBarrierPositions();
    void placeBarriers();
    void spawnAllBarriers();
    void spawnBarrier(barrier b); //bool red, ignition::math::Vector3d pos, ignition::math::Quaterniond rot);
    std::vector<ignition::math::Pose3d> calculateBarrierPositions(std::vector<ignition::math::Vector3d> point, std::vector<double> angle, std::vector<ignition::math::Vector3d> grad, std::vector<gap> gaps);
    std::vector<std::array<unsigned int, 2> > findIntersections(std::vector<ignition::math::Vector3d> points);
    std::vector<gap> getGaps(double length, unsigned int numGaps, double widthGaps, double stdGaps);
    std::vector<ignition::math::Vector3d> calcGradient(std::vector<ignition::math::Vector3d> points);
    unsigned int pointClosestTo(ignition::math::Vector3d pos, std::vector<ignition::math::Vector3d> point, unsigned int start);
    std::vector<ignition::math::Vector3d> catmull_rom(ignition::math::Vector3d p0, ignition::math::Vector3d p1, ignition::math::Vector3d p2, ignition::math::Vector3d p3, unsigned int numpoints);
};



}
