/*
This tool is used for validating autopilot configuration files.
It checks for syntax errors and existence of the executables in the file.

Author: Thomas Passer Jensen (s134234@student.dtu.dk)
*/

#include "usb_utilities.cpp"

#include <ros/package.h>

#include <vector>
#include <set>
#include <string>
#include <iostream>
#include <iomanip>

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>

typedef struct {
  std::string package;
  std::string executable;
} node;

int main(){
  bool ok = true;

  std::cout << "-- Looking for USB storage device with autopilot configuration file --" << std::endl;

  std::vector<std::string> autonomous_nodes = getAutopilotNodes();


  if(!autonomous_nodes.empty()){
    std::vector<node> nodes;

    unsigned int longestpackagename = 0;
    unsigned int longestexecutablename = 0;

    std::set<std::string> usedpackages;

    std::cout << std::endl << "Configuration file found. After trimming, the lines are:" << std::endl;
    for(unsigned int i = 0; i < autonomous_nodes.size(); i++){
      std::cout << "==>" << autonomous_nodes[i] << "<==" << std::endl;

      size_t pos = 0;
      if((pos = autonomous_nodes[i].find(" ")) != std::string::npos){
        std::string thispackage = autonomous_nodes[i].substr(0,pos);
        std::string thisexecutable;

        size_t posex = 0;
        if((posex = autonomous_nodes[i].find(" ", pos+1)) != std::string::npos){
          thisexecutable = autonomous_nodes[i].substr(pos+1,posex-pos-1);
        } else {
          // There is no second space because we have no arguments
          thisexecutable = autonomous_nodes[i].substr(pos+1,std::string::npos);
          //std::cout << "Couldn't parse line, missing second space." << std::endl;
        }
        //std::cout << ">" << thisexecutable << "<" << std::endl;

        usedpackages.insert(thispackage);

        node thisnode;
        thisnode.package = thispackage;
        thisnode.executable = thisexecutable;
        nodes.push_back(thisnode);

        if(thispackage.size() > longestpackagename){
          longestpackagename = thispackage.size();
        }

        if(thisexecutable.size() > longestexecutablename){
          longestexecutablename = thisexecutable.size();
        }

      } else {
        std::cout << "Couldn't parse line, missing space!" << std::endl;
        ok = false;
      }


    }

    std::cout << std::endl << "-- Checking if all the packages exist --" << std::endl;

    std::string availablepackages = ros::package::command("list-names");// exec("rospack list-names");
    //std::cout << availablepackages << std::endl;

    for(auto f : usedpackages){
      //std::cout << "==>" << thispackage << "<==" << std::endl;
      if(availablepackages.find(f) != std::string::npos){
        std::cout << "Package: " << std::setw(longestpackagename) << std::left << f << "  found!" << std::endl;
      } else {
        std::cout << "Package: " << std::setw(longestpackagename) << std::left << f << "  missing!" << std::endl;
        ok = false;
      }
    }

    std::cout << std::endl << "-- Checking if all the nodes exist --" << std::endl;

    std::string pathraw = ros::package::getPath("dynamo_helper");
    std::string remov = "src/dynamo_helper";

    std::string catkin_path = pathraw.substr(0, pathraw.size()-remov.size());
    //std::cout << catkin_path << std::endl;

    for(unsigned int i = 0; i < nodes.size(); i++){
      // Figure out how to do this check
      std::string execpath = catkin_path + "devel/lib/" + nodes[i].package + "/" + nodes[i].executable;
      if(is_file_exists(execpath)){
        std::cout << "Executable: " << std::setw(longestexecutablename) << nodes[i].executable << " | Package: " << std::setw(longestpackagename) << std::left << nodes[i].package << "  found!" << std::endl;
      } else {
        std::string altexecpath = "/opt/ros/kinetic/lib/" + nodes[i].package + "/" + nodes[i].executable;
        if(is_file_exists(altexecpath)){
          std::cout << "Executable: " << std::setw(longestexecutablename) << nodes[i].executable << " | Package: " << std::setw(longestpackagename) << std::left << nodes[i].package << "  found! (native)" << std::endl;
        } else {
          std::cout << "Executable: " << std::setw(longestexecutablename) << nodes[i].executable << " | Package: " << std::setw(longestpackagename) << std::left << nodes[i].package << "  missing!" << std::endl;
          ok = false;
        }
      }

    }


  } else {
    std::cout << "No nodes found in configuration file." << std::endl;
    ok = false;
  }

  if(ok){
    std::cout << std::endl << "Success, all checks passed!" << std::endl;
  } else {
    std::cout << std::endl << "Error, some checks failed!" << std::endl;
  }

}
