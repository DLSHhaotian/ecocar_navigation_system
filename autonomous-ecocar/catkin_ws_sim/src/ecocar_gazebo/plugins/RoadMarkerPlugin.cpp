#include "RoadMarkerPlugin.h"

namespace gazebo {
namespace rendering {
    RoadMarkerPlugin::RoadMarkerPlugin(){
        this->drawn = false;
    }

    RoadMarkerPlugin::~RoadMarkerPlugin()
    {
        // Finalize the visualizer
    }

    // Load the plugin
    void RoadMarkerPlugin::Load(VisualPtr _parent, sdf::ElementPtr _sdf)
    {
        this->visual_ = _parent;

        // Extract parking spot info from SDF
        if (_sdf->HasElement("parkingSpots")) {
            gzdbg << "Found parking spot definition\n";
            sdf::ElementPtr parkingSpotsElem = _sdf->GetElement("parkingSpots");

            parkingSpots.clear();
            sdf::ElementPtr parkingSpotElem = parkingSpotsElem->GetElement("parkingSpot");

            while(parkingSpotElem){
                ParkingSpot thisParkingSpot;
                // Load parameters of the parking spot
                std::string paramName = "width";
                if (parkingSpotElem->HasElement(paramName))
                    thisParkingSpot.width = parkingSpotElem->Get<double>(paramName);
                else
                    thisParkingSpot.width = 1.5;

                paramName = "length";
                if (parkingSpotElem->HasElement(paramName))
                    thisParkingSpot.length = parkingSpotElem->Get<double>(paramName);
                else
                    thisParkingSpot.length = 2.5;

                paramName = "lineWidth";
                if (parkingSpotElem->HasElement(paramName))
                    thisParkingSpot.lineWidth = parkingSpotElem->Get<double>(paramName);
                else
                    thisParkingSpot.lineWidth = 0.2;

                paramName = "pose";
                if (parkingSpotElem->HasElement(paramName)) {
                    std::string poseStr = parkingSpotElem->Get<std::string>(paramName); // "x y theta"
                    std::vector<std::string> pose;
                    boost::split(pose, poseStr, boost::is_any_of(" "));
                    if (pose.size() == 3) {
                        thisParkingSpot.pos = ignition::math::Vector3d(std::stod(pose[0]), std::stod(pose[1]), 0);
                        thisParkingSpot.rot = std::stod(pose[2]);
                    } else {
                        thisParkingSpot.pos = ignition::math::Vector3d(0, 0, 0);
                        thisParkingSpot.rot = 0.0;
                    }
                } else {
                    thisParkingSpot.pos = ignition::math::Vector3d(0, 0, 0);
                    thisParkingSpot.rot = 0.0;
                }
                parkingSpots.push_back(thisParkingSpot);

                // Get next one
                parkingSpotElem = parkingSpotElem->GetNextElement("parkingSpot");
            }

            
        } else {
            gzdbg << "no parking spot found\n";
        }

        // Extract bonus square info from SDF
        if (_sdf->HasElement("bonusSquares")) {
            gzdbg << "Found bonus square definition\n";

            sdf::ElementPtr bonusSquaresElem = _sdf->GetElement("bonusSquares");

            bonusSquares.clear();

            sdf::ElementPtr bonusSquareElem = bonusSquaresElem->GetElement("square");

            while(bonusSquareElem){
                ignition::math::Vector3d pose = bonusSquareElem->Get<ignition::math::Vector3d>();
                bonusSquares.push_back(pose);
                bonusSquareElem = bonusSquareElem->GetNextElement("square");
            }
      }

        this->update_connection_ = event::Events::ConnectRender(
            std::bind(&RoadMarkerPlugin::UpdateChild, this));
    }

    // Update the visualizer
    void RoadMarkerPlugin::UpdateChild()
    {
        if(!drawn) {
            if(parkingSpots.size() > 0){
                for(unsigned int i = 0; i < parkingSpots.size(); i++){
                    drawParkingSpot(parkingSpots[i]);
                }
            }
            if(bonusSquares.size() > 0){
                for(unsigned int i = 0; i < bonusSquares.size(); i++){
                    drawBonusSquare(bonusSquares[i]);
                }
            }
            drawn = true;
        }
    }

    ignition::math::Vector3d rotate(ignition::math::Vector3d vec, double t){
        return ignition::math::Vector3d(vec.X() * std::cos(t) - vec.Y() * std::sin(t),
                                        vec.X() * std::sin(t) + vec.Y() * std::cos(t), 
                                        vec.Z());
    }

    void RoadMarkerPlugin::drawBox(ignition::math::Vector3d startPos, double rot, double a, double b, std::string materialName){
        ignition::math::Vector3d pose = startPos + rotate(ignition::math::Vector3d(a/2, b/2, Z_OFFSET), rot);

        rendering::VisualPtr plane(new rendering::Visual("plane", visual_, false));
        plane->AttachMesh("unit_plane");
        plane->SetScale(ignition::math::Vector3d(a,b,1)); // scales the whole visual object
        plane->SetPose(ignition::math::Pose3d(pose.X(), pose.Y(), pose.Z(), 0, 0, rot));
        plane->SetMaterial(materialName);
        plane->SetVisible(true);
        plane->SetVisibilityFlags(GZ_VISIBILITY_ALL);
        this->visual_->AttachVisual(plane);
    }

    void RoadMarkerPlugin::drawBonusSquare(ignition::math::Vector3d pos){
        // Uses pos.Z() as rotation
        drawBox(ignition::math::Vector3d(pos.X(), pos.Y(), Z_OFFSET), pos.Z(), 0.3, 0.3, "RoadMarkers/BonusSquare");
    }

    void RoadMarkerPlugin::drawParkingSpot(ParkingSpot parkingSpot){
        gzdbg << "Drawing parking spot\n";

        drawBox(parkingSpot.pos + rotate(ignition::math::Vector3d(0, 0, 0), parkingSpot.rot),
                parkingSpot.rot,
                parkingSpot.lineWidth,
                parkingSpot.width,
                "RoadMarkers/Tape"); // parallel with y axis, closest

        drawBox(parkingSpot.pos + rotate(ignition::math::Vector3d(parkingSpot.lineWidth, parkingSpot.lineWidth, 0), parkingSpot.rot),
                parkingSpot.rot-M_PI/2,
                parkingSpot.lineWidth,
                parkingSpot.length-2*parkingSpot.lineWidth,
                "RoadMarkers/Tape"); // parallel with x axis, right

        drawBox(parkingSpot.pos + rotate(ignition::math::Vector3d(parkingSpot.length-parkingSpot.lineWidth, 0, 0),
                parkingSpot.rot), 
                parkingSpot.rot,
                parkingSpot.lineWidth,
                parkingSpot.width,
                "RoadMarkers/Tape"); // parallel with y axis, far

        drawBox(parkingSpot.pos + rotate(ignition::math::Vector3d(parkingSpot.lineWidth, parkingSpot.width, 0), parkingSpot.rot),
                parkingSpot.rot-M_PI/2,
                parkingSpot.lineWidth,
                parkingSpot.length-2*parkingSpot.lineWidth,
                "RoadMarkers/Tape"); // parallel with x axis, left

        this->visual_->SetVisible(true);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(RoadMarkerPlugin)
} // namespace rendering
} // namespace gazebo
