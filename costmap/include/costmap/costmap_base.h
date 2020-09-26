//
// Created by dlsh on 2020/9/20.
//

#ifndef SRC_COSTMAP_BASE_H
#define SRC_COSTMAP_BASE_H



#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>
#include <cmath>
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
        //copy constructor
        costmap_base(const costmap_base& map);
        //copy operator
        costmap_base& operator=(const costmap_base& map);
        //destructor
        virtual ~costmap_base();


        //convert from two map coordinates to index of 1-D array
        unsigned int get_MapToIndex(unsigned int mapX, unsigned int mapY) const;
        //convert from index of 1-D array to two map coordinates
        void indexToMap(unsigned int index, unsigned int& mapX,unsigned int& mapY) const;
        //convert from map coordinates(start from 0) to world coordinates
        void mapToWorld(unsigned int mapX,unsigned int mapY,double& worldX,double& worldY) const;
        //convert from world coordinates to map coordinates
        bool worldToMap(double worldX,double worldY,unsigned int& mapX,unsigned int& mapY) const;
        //convert from world coordinates to map coordinates in bounds
        void worldToMapBounds(double worldX,double worldY,int& mapX, int& mapY) const;
        //convert the distance from world to map(cell)
        unsigned int worldToMapDistance(double distWorld);

        //get the pointer of map(cell)
        unsigned char* get_mapChar() const;
        unsigned char get_cost(unsigned int mapX,unsigned int mapY) const;

        //get the number of cells in the map X
        unsigned int get_mapSizeX() const;
        //get the number of cells in the map Y
        unsigned int get_mapSizeY() const;
        //get the size(m) of map X(from origin point to last point)
        double get_mapSizeX_meter() const;
        //get the size(m) of map Y(from origin point to last point)
        double get_mapSizeY_meter() const;
        //get the x(m) of map's origin point in world coordinate
        double get_originX() const;
        //get the y(m) of map's origin point in world coordinate
        double get_originY() const;
        //get the resolution
        double get_Resolution() const;
        //get the cost default value
        unsigned char get_costDefaultValue() const;
        boost::recursive_mutex* get_mutex() const;


        void updateOrigin(double originX_new,double originY_new);
        void resizeMap(unsigned int sizeX,unsigned int sizeY,double resolution,double originX,double originY);
        void resetMap(unsigned int boundX0, unsigned int boundY0, unsigned int boundXn, unsigned int boundYn);
        void set_cost(unsigned int mapX,unsigned int mapY,unsigned char cost);
        void set_costDefaultValue(unsigned char costDefasultValue);
    protected:
        virtual void initMap_(unsigned int sizeX,unsigned int sizeY);
        virtual void deleteMap_();
        virtual void resetMap_();
        //Copy the memory space of the selected part of the map
        //.Pointer to source map(first cell)
        //2.The bottom left x point of the source map(cell number)
        //3.The bottom left y point of the source map(cell number)
        //4.The x size of the source map(cell number)
        //5.Pointer to destination map(first cell)
        //6.The bottom left x point of the destination map(cell number)
        //7.The bottom left y point of the destination map(cell number)
        //8.The x size of the destination map(cell number)
        //9.The x size of the region to copy(cell number)
        //10.The y size of the region to copy(cell number)
        void copyMapRegion(unsigned char* source_map,unsigned int sourceX,unsigned int sourceY,unsigned int sourceSizeX,unsigned char* destMap,unsigned int destX,unsigned int destY,unsigned int destSizeX,unsigned int regionSizeX,unsigned int regionSizeY);

        template<class cellType>
        void rayTraceLine(cellType cellMark,unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                          unsigned int maxLength = UINT_MAX);

    private:
        template<class cellType>
        void bresenham(cellType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                       int offset_b, unsigned int offset, unsigned int max_length);
        inline int sign(int x){
            return x>0? 1:-1;
        }
    protected:
        unsigned int sizeX_;//The number of cells in the map X coordinate
        unsigned int sizeY_;//The number of cells in the map Y coordinate
        double resolution_;// m/cell
        double originX_;// The map only contains the part in front of the r
        double originY_;
        unsigned char* mapChar_;//cell's pointer
        unsigned char costDefaultValue_;//default cost value of cell
    private:
        boost::recursive_mutex* mutex_;
    protected:
        class markCell {
        public:
            markCell(unsigned char *costmap, unsigned char value) :mapChar_(costmap),costValue_(value){}
            inline void operator()(unsigned int offset)
            {
                mapChar_[offset]=costValue_;
            }
        private:
            unsigned char* mapChar_;
            unsigned char costValue_;
        };
    };
}//namespace costmap end

#endif //SRC_COSTMAP_BASE_H