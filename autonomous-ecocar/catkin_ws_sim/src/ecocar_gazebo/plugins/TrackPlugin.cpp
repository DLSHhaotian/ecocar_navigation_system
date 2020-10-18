

#include "TrackPlugin.h"

using namespace gazebo;

TrackPlugin::TrackPlugin(){

  this->ecocarModel = nullptr; // will be pointer to car object

  this->red_barrier_URI = "model://red_barrier";
  this->gray_barrier_URI = "model://gray_barrier";
  this->barrierLength = 0.81;
  this->barrierWidth = 0.38;

  this->carCrashed = false;

  this->ecocarBaseLinkName = "ecocar::base_link::base_link_collision";

  if (!ros::isInitialized())
  {
    int argc = 0;
    char* argv = nullptr;
    ros::init(argc, &argv, "GazeboTrackPlugin");
  }
  ROS_DEBUG("TrackPlugin init");

  trackPub = nh.advertise<ecocar_gazebo_msgs::Track>("sim/track",1);
  crashPub = nh.advertise<std_msgs::Bool>("sim/crashed",1);


  lastCrashPubTime = 0;
  lastBarrierPubTime = 0;
}
void TrackPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){
  ROS_DEBUG("TrackPlugin in Load");
  world = _parent;

  // Init new barrier SDF (red)
  redBarrierModelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", redBarrierModelSDF);

  // Load the barrier into it
  std::string filename = common::ModelDatabase::Instance()->GetModelFile(this->red_barrier_URI);
  sdf::readFile(filename, redBarrierModelSDF);
  redBarrierModel = redBarrierModelSDF->Root()->GetElement("model");

  // Init new barrier SDF (gray)
  grayBarrierModelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", grayBarrierModelSDF);

  // Load the barrier into it
  filename = common::ModelDatabase::Instance()->GetModelFile(this->gray_barrier_URI);
  sdf::readFile(filename, grayBarrierModelSDF);
  grayBarrierModel = grayBarrierModelSDF->Root()->GetElement("model");

  // Extract the required information from the world file
  // Width of road and coordinates of all the road points

  std::string paramName = "loop";
  if(_sdf->HasElement(paramName))
    loop = _sdf->Get<std::string>(paramName) == "true";
  else
    loop = false;

  paramName = "sharpTurns";
  if(_sdf->HasElement(paramName))
    sharpTurns = _sdf->Get<std::string>(paramName) == "true";
  else
    sharpTurns = false;

  paramName = "numGaps";
  if(_sdf->HasElement(paramName))
    numGaps = _sdf->Get<unsigned int>(paramName);
  else
    numGaps = 0;

  paramName = "widthGaps";
  if(_sdf->HasElement(paramName))
    widthGaps = _sdf->Get<double>(paramName);
  else
    widthGaps = 0;

  paramName = "stdGaps";
  if(_sdf->HasElement(paramName))
    stdGaps = _sdf->Get<double>(paramName);
  else
    stdGaps = 0;

  paramName = "spawnLeft";
    if(_sdf->HasElement(paramName))
    spawnLeft = _sdf->Get<std::string>(paramName) == "true";
  else
    spawnLeft = true;

  paramName = "spawnRight";
    if(_sdf->HasElement(paramName))
    spawnRight = _sdf->Get<std::string>(paramName) == "true";
  else
    spawnRight = true;

  if(_sdf->GetParent()->HasElement("road")){
    roadPoints.clear();

    sdf::ElementPtr road = _sdf->GetParent()->GetElement("road");

    if(road->HasElement("width")){

      std::string width_string = road->GetElement("width")->GetValue()->GetAsString();
      ROS_DEBUG("Track width: %s", width_string.c_str());
      trackWidth = std::stod(width_string);

      sdf::ElementPtr pointElem = road->GetElement("point");
      while (pointElem){
        ignition::math::Vector3d point = pointElem->Get<ignition::math::Vector3d>();

        roadPoints.push_back(point);

        pointElem = pointElem->GetNextElement("point");
      }

    ROS_DEBUG("Found %lu road points.", roadPoints.size());

    } else {
      ROS_DEBUG("No width element.");
    }
  } else {
    ROS_DEBUG("No road found.");
  }

  // Create a new transport node
  this->node = transport::NodePtr(new transport::Node());

  // Initialize the node with the world name
  this->node->Init(world->Name());

  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&TrackPlugin::onUpdate, this));
  resetConnection = event::Events::ConnectWorldReset(std::bind(&TrackPlugin::onReset, this));

  // Place barriers
  if(roadPoints.size() > 3){ // need atleast 4 points to do anything
    placeBarriers();
    spawnAllBarriers();
  }

  cm = this->world->Physics()->GetContactManager();

  std::string topic = cm->CreateFilter("ecocar_collision",
        this->ecocarBaseLinkName);
  
  this->contactSub = this->node->Subscribe(topic,
          &TrackPlugin::OnContacts, this);

  ROS_INFO("Finished loading Track Plugin.");

}

void TrackPlugin::publishBarrierPositions(){
  ecocar_gazebo_msgs::Track msg;
  
  for(unsigned int i = 0; i < barriers.size(); i++){
    geometry_msgs::Point p;
    p.x = barriers[i].pos.X();
    p.y = barriers[i].pos.Y();
    p.z = barriers[i].pos.Z();
    msg.points.push_back(p);
    trackPub.publish(msg);
  }
}

void TrackPlugin::spawnAllBarriers(){

  for(unsigned int i = 0; i < barriers.size(); i++){
    if(!barriers[i].spawned){
      // spawn barrier
      spawnBarrier(barriers[i]);
    }
  }

} 

void TrackPlugin::placeBarriers(){
  // First increase the number of points, we are not given that many in the world file
  // Use Catmull-Rom spline interpolation

  // Determine how many more points we need
  ignition::math::Vector3d d = roadPoints[1]-roadPoints[0];
  ROS_DEBUG("Distance between road points: %f", d.Length());
  unsigned int numpoints = d.Length()/barrierLength * 50; // to get ~50 points for each barrier
  ROS_DEBUG("Numpoints: %u", numpoints);

  std::vector<ignition::math::Vector3d> points2; // will contain original points and added points from spline
  // loop through given points, add more points

  if(loop){ // if the track is a loop
    for(unsigned int i=0; i<roadPoints.size()-1; i++){
      unsigned int pp, pn, pnn;
      // Handle edge cases
      if(i==0){
        // First points
        pp = roadPoints.size()-2;
        pn = i+1;
        pnn = i+2;
      } else if(i == roadPoints.size()-2){
        // Next last point
        pp = i-1;
        pn = i+1;
        pnn = 1;
      } else if(i == roadPoints.size()-1){
        std::cout << "DOES IT EVEN REACH THIS??" << std::endl; // should not
        // Last point
        pp = i-1;
        pn = 1;
        pnn = 2;
      } else {
        pp = i-1;
        pn = i+1;
        pnn = i+2;
      }
      // add points to new vector
      std::vector<ignition::math::Vector3d> c = catmull_rom(roadPoints[pp],roadPoints[i],roadPoints[pn],roadPoints[pnn],numpoints);
      points2.insert(points2.end(), c.begin(), c.end());
    }

  } else { // track is not a loop

    for(unsigned int i=0; i<roadPoints.size()-1; i++){
      std::vector<ignition::math::Vector3d> c;
      if(i == 0){
        ignition::math::Vector3d pb = roadPoints[i] - (roadPoints[i+1]-roadPoints[i]);
        c = catmull_rom(pb,roadPoints[i],roadPoints[i+1],roadPoints[i+2],numpoints);
      } else if(i == roadPoints.size()-2){
        ignition::math::Vector3d pa = roadPoints[i+1] + (roadPoints[i+1]-roadPoints[i]);
        c = catmull_rom(roadPoints[i-1],roadPoints[i],roadPoints[i+1],pa,numpoints);
      } else {
        c = catmull_rom(roadPoints[i-1],roadPoints[i],roadPoints[i+1],roadPoints[i+2],numpoints);
      }
      // add points to new vector
      points2.insert(points2.end(), c.begin(), c.end());
    }
  }

  // Points are middle of road, calculate the edges
  // Calculate vector perpendicular to the gradient and add/subtract
  std::vector<ignition::math::Vector3d> points_left(points2.size());
  std::vector<ignition::math::Vector3d> points_right(points2.size());


  for(unsigned int i=0; i<points2.size(); i++){
    ignition::math::Vector3d temp;

    if(i==0 || i == points2.size()-1){
      if(loop){
        temp = points2[1]-points2[points2.size()-2];
      } else {
        if(i==0){
          temp = points2[i+1]-points2[i];
        } else {
          temp = points2[i]-points2[i-1];
        }
      }
    } else {
      temp = points2[i+1]-points2[i-1];
    }

    ignition::math::Vector2d thisgrad(temp.X(),temp.Y());

    thisgrad.Normalize();
    ignition::math::Vector3d perp(-thisgrad.Y(), thisgrad.X(), 0);

    points_left[i]  = points2[i] + trackWidth/2 * perp;
    points_right[i] = points2[i] - trackWidth/2 * perp;
  }

  std::vector<ignition::math::Vector3d> grad_left = calcGradient(points_left);
  std::vector<ignition::math::Vector3d> grad_right = calcGradient(points_right);

  if(sharpTurns){
    if(spawnLeft){
      // The above way of finding the sides will sometimes create "loops" at sharp turns in the road.
      // Find those loops (find the intersections in the points) and remove the loops
      std::vector<std::array<unsigned int, 2> > left_intersections = findIntersections(points_left); // quite slow algorithm, 6 seconds with ~10k points..
      ROS_DEBUG("Found %lu intersections left", left_intersections.size());

      std::vector<unsigned int> removeListLeft; // list with all the indexes of points to be removed
      for(unsigned int i=0; i < left_intersections.size(); i++){
        ROS_DEBUG("Adding to list: %u to %u", left_intersections[i][0]+1, left_intersections[i][1]);

        // Add all numbers between left_intersection[i][0] and left_intersection[i][1]
        unsigned int idx = left_intersections[i][0]+1;
        while(idx != left_intersections[i][1]){
          removeListLeft.push_back(idx);
          idx++;
          if (idx > (points_left.size()-1)) idx = 0; // wrap
        }
      }
      // Sort
      //std::sort(removeListLeft.begin(), removeListLeft.end());
      std::set<unsigned int> removeSetLeft(removeListLeft.begin(), removeListLeft.end());
      // Remove points from end
      std::set<unsigned int>::reverse_iterator rit;
      for(rit=removeSetLeft.rbegin(); rit != removeSetLeft.rend(); ++rit){
        //ROS_DEBUG("Removing %u", *rit);
        points_left.erase(points_left.begin()+(*rit)); // TODO fix problem, sometimes crashes here
        grad_left.erase(grad_left.begin()+(*rit));
      }
    }

    if(spawnRight){
      // Same for right (but this side is more unlikely to have the problem)
      std::vector<std::array<unsigned int, 2> > right_intersections = findIntersections(points_right);
      ROS_DEBUG("Found %lu intersections right", right_intersections.size());

      std::vector<unsigned int> removeListRight; // list with all the indexes of points to be removed
      for(unsigned int i=0; i < right_intersections.size(); i++){
        ROS_DEBUG("Adding to list: %u to %u", right_intersections[i][0]+1, right_intersections[i][1]);

        unsigned int idx = right_intersections[i][0]+1;
        while(idx != right_intersections[i][1]){
          removeListRight.push_back(idx);
          idx++;
          if (idx > (points_right.size()-1)) idx = 0; // wrap
        }
      }
      // Sort
      //std::sort(removeListRight.begin(), removeListRight.end());
      std::set<unsigned int> removeSetRight(removeListRight.begin(), removeListRight.end());
      // Remove points
      std::set<unsigned int>::reverse_iterator rit;
      for(rit=removeSetRight.rbegin(); rit != removeSetRight.rend(); ++rit){
        //ROS_DEBUG("Removing %u", *rit);
        points_right.erase(points_right.begin()+(*rit)); // TODO fix problem, sometimes crashes here
        grad_right.erase(grad_right.begin()+(*rit));
      }
    }

    if(points_left.size() < 2 || points_right.size() < 2){
      ROS_ERROR("Something went wrong when looking for intersections! Sorry about that, try to restart simulation.");
      return;
    } else {
      ROS_DEBUG("Now %lu remaining points on left side.", points_left.size());
      ROS_DEBUG("Now %lu remaining points on right side.", points_right.size());
    }
  }

  // Calculate gradients of points for barrier rotations
  std::vector<double> angles_left(grad_left.size());
  std::vector<double> angles_right(grad_right.size());

  std::transform(grad_left.begin(), grad_left.end(), angles_left.begin(), [](ignition::math::Vector3d grad){return atan2(grad.Y(), grad.X());});
  std::transform(grad_right.begin(), grad_right.end(), angles_right.begin(), [](ignition::math::Vector3d grad){return atan2(grad.Y(), grad.X());});

  //ROS_DEBUG("test grad: (%f, %f)" , grad_left[0].X(), grad_left[0].Y());
  //ROS_DEBUG("test angle: %f", angles_left[0]);
  // Calculate gaps in between barriers
  // First calculate length of each side

  double leftLength = 0;
  for(unsigned int i = 0; i < points_left.size()-1; i++){
    leftLength += (points_left[i+1]-points_left[i]).Length();
  }
  ROS_DEBUG("Left length: %f", leftLength);
  
  double rightLength = 0;
  for(unsigned int i = 0; i < points_right.size()-1; i++){
    rightLength += (points_right[i+1]-points_right[i]).Length();
  }
  ROS_DEBUG("Right length: %f", rightLength);

  ROS_DEBUG("Gaps: %u", numGaps);
  std::vector<gap> gapsLeft = getGaps(leftLength, numGaps, widthGaps, stdGaps);
  std::vector<gap> gapsRight = getGaps(rightLength, numGaps, widthGaps, stdGaps);

  // Begin barrier placement calculations
  ROS_DEBUG("Calculating barrier positions...");
  // Left side
  std::vector<ignition::math::Pose3d> barrierPosesLeft = calculateBarrierPositions(points_left, angles_left, grad_left, gapsLeft);
  ROS_DEBUG("Done left");
  // Right side
  std::vector<ignition::math::Pose3d> barrierPosesRight = calculateBarrierPositions(points_right, angles_right, grad_right, gapsRight);

  unsigned int barrierNum = 0;
  
  ROS_DEBUG("Placing barriers...");

  if(spawnLeft){
    // Place barriers, left
    for(unsigned int i = 0; i<barrierPosesLeft.size(); i++){
      barrier thisBarrier;
      thisBarrier.red = i%2==0;
      thisBarrier.pos = barrierPosesLeft[i].Pos();
      thisBarrier.rot = barrierPosesLeft[i].Rot();
      thisBarrier.spawned = false;
      thisBarrier.index = barrierNum;

      barriers.push_back(thisBarrier);
      barrierNum++;
    }
    ROS_DEBUG("Placed %lu barriers left", barrierPosesLeft.size());
  }

  if(spawnRight){
    // Place barriers, right
    for(unsigned int i = 0; i<barrierPosesRight.size(); i++){
      barrier thisBarrier;
      thisBarrier.red = i%2==0;
      thisBarrier.pos = barrierPosesRight[i].Pos();
      thisBarrier.rot = barrierPosesRight[i].Rot(); 
      thisBarrier.spawned = false;
      thisBarrier.index = barrierNum;

      barriers.push_back(thisBarrier);
      barrierNum++;
    }
    ROS_DEBUG("Placed %lu barriers right", barrierPosesRight.size());
  }
  

}

std::vector<ignition::math::Vector3d> TrackPlugin::calcGradient(std::vector<ignition::math::Vector3d> points){
  std::vector<ignition::math::Vector3d> grad(points.size());

    for(unsigned int i=0; i<points.size(); i++){
      ignition::math::Vector3d thisgrad;
      
      if(i==0 || i == points.size()-1){
        thisgrad = 0.5*(points[1]-points[points.size()-2]);
      } else {
        thisgrad = 0.5*(points[i+1]-points[i-1]);
      }
    //ROS_DEBUG("This grad: (%f, %f)", thisgrad.X(), thisgrad.Y());

    grad[i] = thisgrad.Normalize();
    
    }
  return grad;
} 

std::vector<ignition::math::Pose3d> TrackPlugin::calculateBarrierPositions(std::vector<ignition::math::Vector3d> point, std::vector<double> angle, std::vector<ignition::math::Vector3d> grad, std::vector<gap> gaps){
  // Begin barrier placement calculations
  // Left side
  std::vector<ignition::math::Pose3d> barriers;

  // Offset points to get all potential barrier start positions
  std::vector<ignition::math::Vector3d> barrierStartPositions;
  for(unsigned int i = 0; i < point.size()-1; i++){
    barrierStartPositions.push_back(point[i]+grad[i]*(-barrierLength/2));
  }
  //std::vector<ignition::math::Vector3d> barrierPositions;
  //std::vector<double> barrierRotations;

  double d = 0; // distance on side so far

  // The first is easy
  unsigned int lasti = 0;

  ignition::math::Pose3d thisBarrier;
  thisBarrier.Pos() = point[0];
  thisBarrier.Rot() = ignition::math::Quaterniond(0, 0, angle[0]);
  barriers.push_back(thisBarrier);
  d += barrierLength/2;

  bool done = false;

  unsigned int endThreshold = 10; // index

  unsigned int nextGap = 0;

  while(!done){
    // Loops one time for each barrier placed

    // End position of last placed barrier or gap
    ignition::math::Vector3d lastEndPos = point[lasti] + grad[lasti]*barrierLength/2;

    if(gaps.size() > 0 && abs(gaps[nextGap].pos - d) <= barrierLength/2){
      // Place gap
      // calculate lasti
      double l = 0;

      unsigned int startIndex = pointClosestTo(lastEndPos, point, lasti); // index of point closest to last barrier end
      unsigned int endIndex;

      // Find end point of gap, by adding distances between points
      for(unsigned int i = startIndex; i < point.size()-1; i++){
        l += (point[i+1]-point[i]).Length();

        if(l >= gaps[nextGap].length){
          endIndex = i;
          break;
        }
      }

      lasti = endIndex; // this is where we will end the gap
      d += gaps[nextGap].length;
      nextGap++;
    } else {
      // Place barrier

      // Find index of point from barrierStartPositions closest to the last end position
      unsigned int k = pointClosestTo(lastEndPos, barrierStartPositions, lasti);
      //ROS_DEBUG("Placing barrier at %u/%lu", k, point.size());

      ignition::math::Pose3d thisBarrier;
      thisBarrier.Pos() = point[k];
      thisBarrier.Rot() = ignition::math::Quaterniond(0, 0, angle[k]);
      barriers.push_back(thisBarrier);
      d += (point[lasti]-point[k]).Length();
      lasti = k;
    }

    if(point.size() - lasti < endThreshold){
      done = true;
    }

  }

  return barriers;
}

std::vector<std::array<unsigned int, 2> > TrackPlugin::findIntersections(std::vector<ignition::math::Vector3d> points){
  std::vector<std::array<unsigned int, 2> > res;

  //ROS_DEBUG("points.size(): %lu", points.size());
  for(unsigned int i = 0;  i < points.size()-1; i++){
    // For each point i, the line between it and the next point
    ignition::math::Vector3d d1 = points[i+1]-points[i];

    for(unsigned int j = i+2; j < points.size()-1; j++){
      // For each point j, the line between it and the next point
      ignition::math::Vector3d d2 = points[j+1]-points[j];
      ignition::math::Vector3d d3 = points[j]-points[i];
      ignition::math::Vector3d d4 = points[j+1]-points[i];
      ignition::math::Vector3d d5 = points[j]-points[i+1];

      // Source: https://stackoverflow.com/a/14177062
      if( ( ( d3.X()*d1.Y()-d3.Y()*d1.X() ) * ( d4.X()*d1.Y()-d4.Y()*d1.X() ) < 0 ) && ( ( -d3.X()*d2.Y()+d3.Y()*d2.X() ) * ( -d5.X()*d2.Y()+d5.Y()*d2.X() ) < 0 ) ){
          // The lines intersect
          // Sometimes this is because of numerical errors (?) or something else..
          // Check angle between the two lines that intersect, if greater than some threshold we believe it
          double angle1 = atan2(d1.Y(), d1.X());
          double angle2 = atan2(d2.Y(), d2.X());

           // Wrap to [-pi;pi]
          double da = angle2-angle1;
          da = fmod(da + M_PI, 2*M_PI);
          if (da < 0) da += 2*M_PI;
          da -= M_PI;

          double da2 = M_PI/2 - fabs(fabs(da)-M_PI/2); // This gives the angle between 0 and pi/2 ignoring that lines could be in "opposite" directions (da2 is min(da, pi-da))
          if(da2 > 0.3){
            // ROS_DEBUG("Found intersection at i: %u, j: %u points: %f %f %f %f %f %f %f %f", i,j,points[i].X(), points[i].Y(),points[i+1].X(), points[i+1].Y(),points[j].X(), points[j].Y(),points[j+1].X(), points[j+1].Y());
            // ROS_DEBUG("Angles: %f %f, diff: %f", angle1, angle2, angle2-angle1);
            // ROS_DEBUG("da: %f, da2: %f", da, da2);

            double f1 = ((float)j-i)/points.size();
            double f2 = ((float)points.size()-j+i)/points.size();
            // ROS_DEBUG("f1: %f, f2: %f", f1, f2);

            unsigned int indStart, indStop;
            if(f1 < 0.1){
              indStart = i;
              indStop = j;
            } else if(f2<0.1){
              indStart = j;
              indStop = i;
            } else {
              ROS_WARN("Something is very wrong. %f %f", f1, f2);
              // Just guess?
              indStart = i;
              indStop = j;
            }
            std::array<unsigned int, 2> thisIntersection = {indStart, indStop};
            res.push_back(thisIntersection);
          }
        }
    }
  }

  return res;
}

// Returns the index of the point closest to
unsigned int TrackPlugin::pointClosestTo(ignition::math::Vector3d pos, std::vector<ignition::math::Vector3d> point, unsigned int start){
  double searchThreshold = 2*barrierLength; // m

  double smallestDist = std::numeric_limits<double>::max();
  unsigned int smallesti;

  for(unsigned int i=start; i < point.size()-1; i++){
    double pointDif = (point[i] - pos).Length();
    if(pointDif < smallestDist){
      //ROS_DEBUG("New smallest dist: %f", pointDif);
      smallestDist = pointDif;
      smallesti = i;
    }

    if(pointDif > searchThreshold){
      break;
    }
    if(i == point.size()-1){
      break;
    }
  }

  return smallesti;
}

void TrackPlugin::onUpdate(){

  const common::Time curTime = this->world->SimTime();

  if(ecocarModel == nullptr){
    // Try to find ecocar model
    std::vector<physics::ModelPtr> models = world->Models();
    for(unsigned int i = 0; i<models.size(); i++){
      if(models[i]->GetName() == "ecocar"){
        ROS_DEBUG("Found ecocar model.");
        ecocarModel = models[i];
        break;
      } 
    }

    // If we still haven't found it after a while, print error
    if(ecocarModel == nullptr && curTime >= 1.0){
      ROS_ERROR_ONCE("Could not find ecocar model.."); // TODO print only after a little while
    }
  }

  if(ecocarModel != nullptr){
    // If we can find ecocar model
    // Stuff can be done to the model here
  }

  if(curTime - this->lastBarrierPubTime > 1){
    publishBarrierPositions();

    this->lastBarrierPubTime = curTime;
  }

  if(curTime - this->lastCrashPubTime > 0.1){
    std_msgs::Bool msg;
    msg.data = carCrashed;
    crashPub.publish(msg);
    
    this->lastCrashPubTime = curTime;
  }
  
}

void TrackPlugin::onReset(){
  this->lastBarrierPubTime = 0;
  this->lastCrashPubTime = 0;
}

void TrackPlugin::OnContacts(ConstContactsPtr &_msg){
  if(_msg->contact_size() > 0){
    //ROS_DEBUG("Contacts detected: %u", _msg->contact_size());
  
    // Iterate over all the contacts in the message
    for (int i = 0; i < _msg->contact_size(); ++i)
    {
      std::string collision1 = _msg->contact(i).collision1();
      std::string collision2 = _msg->contact(i).collision2();

      std::string collidedWith;
      if(collision1 == ecocarBaseLinkName){
        collidedWith = collision2;
      } else if(collision2 == ecocarBaseLinkName){
        collidedWith = collision1;
      }

      collidedWith = collidedWith.substr(0, collidedWith.find(":"));

      ROS_DEBUG("The ecocar collided with: %s", collidedWith.c_str());
      carCrashed = true;
    }
      
  }
}

// Function to calculate catmull rom spline between points p1 and p2 with numpoint points
std::vector<ignition::math::Vector3d> TrackPlugin::catmull_rom(ignition::math::Vector3d p0, ignition::math::Vector3d p1, ignition::math::Vector3d p2, ignition::math::Vector3d p3, unsigned int numpoints){
  std::vector<double> t;
  for(unsigned int i=0; i<numpoints; i++){
    double thist = 1.0 + 1.0*i/(numpoints-1);
    t.push_back(thist);
  }

  std::vector<double> P0 = {p0.X(),p0.Y(), p0.Z()};
  std::vector<double> P1 = {p1.X(),p1.Y(), p1.Z()};
  std::vector<double> P2 = {p2.X(),p2.Y(), p2.Z()};
  std::vector<double> P3 = {p3.X(),p3.Y(), p3.Z()};

  std::vector<std::vector<double>> A1(numpoints,std::vector<double> (3,0));
  std::vector<std::vector<double>> A2(numpoints,std::vector<double> (3,0));
  std::vector<std::vector<double>> A3(numpoints,std::vector<double> (3,0));

  std::vector<std::vector<double>> B1(numpoints,std::vector<double> (3,0));
  std::vector<std::vector<double>> B2(numpoints,std::vector<double> (3,0));

  std::vector<std::vector<double>> C(numpoints,std::vector<double> (3,0));

  std::vector<ignition::math::Vector3d> c(numpoints);

  for(unsigned int i=0; i<numpoints; i++){
    A1[i][0] = (1-t[i])*P0[0] + t[i]*P1[0];
    A1[i][1] = (1-t[i])*P0[1] + t[i]*P1[1];
    A1[i][2] = (1-t[i])*P0[2] + t[i]*P1[2];

    A2[i][0] = (2-t[i])*P1[0] + (t[i]-1)*P2[0];
    A2[i][1] = (2-t[i])*P1[1] + (t[i]-1)*P2[1];
    A2[i][2] = (2-t[i])*P1[2] + (t[i]-1)*P2[2];

    A3[i][0] = (3-t[i])*P2[0] + (t[i]-2)*P3[0];
    A3[i][1] = (3-t[i])*P2[1] + (t[i]-2)*P3[1];
    A3[i][2] = (3-t[i])*P2[2] + (t[i]-2)*P3[2];

    B1[i][0] = (2-t[i])/2.0 * A1[i][0] + t[i]/2.0 * A2[i][0];
    B1[i][1] = (2-t[i])/2.0 * A1[i][1] + t[i]/2.0 * A2[i][1];
    B1[i][2] = (2-t[i])/2.0 * A1[i][2] + t[i]/2.0 * A2[i][2];

    B2[i][0] = (3-t[i])/2.0 * A2[i][0] + (t[i]-1)/2.0 * A3[i][0];
    B2[i][1] = (3-t[i])/2.0 * A2[i][1] + (t[i]-1)/2.0 * A3[i][1];
    B2[i][2] = (3-t[i])/2.0 * A2[i][2] + (t[i]-1)/2.0 * A3[i][2];

    C[i][0] = (2-t[i])*B1[i][0] + (t[i]-1)*B2[i][0];
    C[i][1] = (2-t[i])*B1[i][1] + (t[i]-1)*B2[i][1];
    C[i][2] = (2-t[i])*B1[i][2] + (t[i]-1)*B2[i][2];

    c[i] = ignition::math::Vector3d(C[i][0],C[i][1],C[i][2]);
  }
  return c;
}

// Not used
/*
void TrackPlugin::CircleBarrier(float radius, float x0, float y0){
  // This function was mostly a test of the SpawnBarrier function
  float circumference = 2*IGN_PI*radius;
  unsigned int imax = floor(circumference/barrierLength);

  for(unsigned int i = 0; i < imax; i++){
    this->SpawnBarrier(i%2==0, ignition::math::Vector3d(x0 + radius*cos((float)i/(float)imax * 2 * IGN_PI), y0 + radius*sin((float)i/(float)imax * 2 * IGN_PI), 0), ignition::math::Quaterniond(0, 0, (float)i/(float)imax * 2 * IGN_PI + 0.5*IGN_PI));
  }
}
*/

// Not used
/*
void TrackPlugin::LineBarrier(float trackWidth, float trackLength, float x0, float y0){

  unsigned int imax = boost::math::iround(trackLength/barrierLength);

  for(unsigned int i = 0; i < imax; i++){
    this->SpawnBarrier(i%2==0, ignition::math::Vector3d(x0+i*barrierLength, y0+0.5*trackWidth, 0), ignition::math::Quaterniond(0, 0, 0));
    this->SpawnBarrier(i%2==0, ignition::math::Vector3d(x0+i*barrierLength, y0-0.5*trackWidth, 0), ignition::math::Quaterniond(0, 0, 0));
  }
}
*/

std::vector<gap> TrackPlugin::getGaps(double length, unsigned int numGaps, double widthGaps, double stdGaps){
  std::vector<gap> res;

  // Pulls width of gaps from normal distribution
  // Could do the same for positions, but do we then want some way to avoid gaps on top of each other?
  std::random_device rd;
  std::mt19937 e2(rd());
  std::normal_distribution<> dist(widthGaps, stdGaps);

  for(unsigned i = 0; i < numGaps; i++){
    gap thisgap;
    thisgap.pos = length/(double)(numGaps+1) * (i+1) - 0.5*widthGaps;
    thisgap.length =  dist(e2);// widthGaps;

    //std::cout << thisgap.length << std::endl;
    res.push_back(thisgap);
  }

  return res;
}

void TrackPlugin::spawnBarrier(barrier b){//bool red, ignition::math::Vector3d pos, ignition::math::Quaterniond rot){
  // Spawns a barrier at given pose

  sdf::SDFPtr barrierModelSDF;
  sdf::ElementPtr barrierModel;

  // We use the loaded SDF files from Load(), because loading it each time adds substantial overhead
  if(b.red){
    barrierModelSDF = redBarrierModelSDF;
    barrierModel = redBarrierModel;
  } else {
    barrierModelSDF = grayBarrierModelSDF;
    barrierModel = grayBarrierModel;
  }

  // Change the name (not changing the name will overwrite the previously placed barrier)
  std::string name = "barrier";
  name += std::to_string(b.index);

  barrierModel->GetAttribute("name")->SetFromString(name);

  // Set pose
  barrierModel->GetElement("pose")->Set(ignition::math::Pose3d(b.pos, b.rot));

  // Spawn
  this->world->InsertModelSDF(*barrierModelSDF);
  b.spawned = true;

  //barrierNumber++;
}

GZ_REGISTER_WORLD_PLUGIN(TrackPlugin)
