//
// Created by dlsh on 2020/9/20.
//

#ifndef SRC_COSTMAP_BASE_H
#define SRC_COSTMAP_BASE_H



#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>
namespace costmap {
    //storing x/y point pairs
    struct mapIndex{
        unsigned int x;
        unsigned int y;
    };
    //
    class costmap_base{
    public:
        //constructor
        costmap_base(unsigned int sizeX, unsigned int sizeY, double resolution,
                  double originX, double originY, unsigned char costDefaultValue = 0);
        //default constructor
        costmap_base();
        //destructor
        virtual ~costmap_base();


        //convert from two map coordinates to index of 1-D array
        unsigned int get_MapToIndex(unsigned int mapX, unsigned int mapY) const {return mapX*sizeX_+sizeY_;}
        //convert from index of 1-D array to two map coordinates
        void indexToMap(unsigned int index, unsigned int& mapX,unsigned int& mapY) const
        {
            mapY=index/sizeX_;
            mapX=index-(mapY*sizeX_);
        }
        //convert from map coordinates(start from 0) to world coordinates
        void mapToWorld(unsigned int mapX,unsigned int mapY,double& worldX,double& worldY) const
        {
            worldX=originX_+(mapX+0.5)*resolution_;
            worldY=originY_+(mapY+0.5)*resolution_;
        }
        //convert from world coordinates to map coordinates
        bool worldToMap(double worldX,double worldY,unsigned int& mapX,unsigned int& mapY) const
        {
            if(worldX<originX_||worldY<originY_){return false;}
            mapX=(int)((worldX-originX_)/resolution_);
            mapY=(int)((worldY-originY_)/resolution_);
            if(mapX<sizeX_&&mapY<sizeY_){return true;}
            return false;
        }

        //get the pointer of map(cell)
        unsigned char* get_mapChar() const{return mapChar_;}
        unsigned char get_cost(unsigned int mapX,unsigned int mapY) const{return mapChar_[get_MapToIndex(mapX,mapY)];}
        void set_cost(unsigned int mapX,unsigned int mapY,unsigned char cost)
        {
            mapChar_[get_MapToIndex(mapX,mapY)]=cost;
        }
        //get the number of cells in the map X
        unsigned int get_mapSizeX() const{return sizeX_;}
        //get the number of cells in the map Y
        unsigned int get_mapSizeY() const{return sizeY_;}
        //get the size(m) of map X(from origin point to last point)
        double get_mapSizeX_meter() const{return (sizeX_-1+0.5)*resolution_;}
        //get the size(m) of map Y(from origin point to last point)
        double get_mapSizeY_meter() const{return (sizeY_-1+0.5)*resolution_;}
        //get the x(m) of map's origin point in world coordinate
        double get_originX() const{return originX_;}
        //get the y(m) of map's origin point in world coordinate
        double get_originY() const{return originY_;}
        //get the resolution
        double get_Resolution() const{return resolution_;}
        //get the cost default value
        unsigned char get_costDefaultValue() const{return costDefaultValue_;}

    protected:
        virtual void initMap();
        virtual void deleteMap();
        virtual void resetMap();
    protected:
        unsigned int sizeX_;//The number of cells in the map X coordinate
        unsigned int sizeY_;//The number of cells in the map Y coordinate
        double resolution_;// m/cell
        double originX_;// bottom left point
        double originY_;
        unsigned char* mapChar_;//cell's pointer
        unsigned char costDefaultValue_;//default cost value of cell
    private:
        //mutex_t* access_;

    };
}//namespace costmap end

#endif //SRC_COSTMAP_BASE_H