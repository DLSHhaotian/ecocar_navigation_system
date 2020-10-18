#ifndef ROAD_MARKER_PLUGIN_H
#define ROAD_MARKER_PLUGIN_H

#include <boost/algorithm/string.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"

#define Z_OFFSET 0.01

struct ParkingSpot {
    ignition::math::Vector3d pos;
    double rot;
    double length;
    double width;
    double lineWidth;
};

namespace gazebo
{
namespace rendering
{
class RoadMarkerPlugin : public VisualPlugin
{
  public:
    /// \brief Constructor
    RoadMarkerPlugin();

    /// \brief Destructor
    virtual ~RoadMarkerPlugin();

    /// \param node XML config node
    void Load(VisualPtr _parent, sdf::ElementPtr _sdf);

  protected:
    /// \brief Update the visual plugin
    virtual void UpdateChild();

  private:
    VisualPtr visual_;

    std::vector<ignition::math::Vector3d> bonusSquares;
    std::vector<ParkingSpot> parkingSpots;

    void drawBox(ignition::math::Vector3d startPos, double rot, double a, double b, std::string materialName);

    void drawParkingSpot(ParkingSpot parkingSpot);
    void drawBonusSquare(ignition::math::Vector3d pos);
    bool drawn;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;
};
} // namespace rendering
} // namespace gazebo

#endif