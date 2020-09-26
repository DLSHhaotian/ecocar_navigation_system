//
// Created by dlsh on 2020/9/20.
//
#include <costmap/costmap_base.h>
#include <cstdio>
using namespace std;
namespace costmap{
    costmap_base::costmap_base(unsigned int sizeX, unsigned int sizeY, double resolution,
                               double originX, double originY, unsigned char costDefaultValue):
                               sizeX_(sizeX),sizeY_(sizeY),resolution_(resolution),originX_(originX),originY_(originY),costDefaultValue_(costDefaultValue),mapChar_(nullptr){
        mutex_=new boost::recursive_mutex();
        //initialize the costmap
        initMap_(sizeX_,sizeY_);
        resetMap_();
    }
    //default constructor
    costmap_base::costmap_base():
        sizeX_(0),sizeY_(0),resolution_(0),originX_(0),originY_(0),costDefaultValue_(0),mapChar_(nullptr){
        mutex_=new boost::recursive_mutex();
    }
    //copy constructor
    costmap_base::costmap_base(const costmap_base& map):mapChar_(nullptr){
        mutex_=new boost::recursive_mutex();
        *this=map;
    }
    //copy operator
    costmap_base& costmap_base::operator=(const costmap_base &map) {
        if(this==&map)
            return *this;
        deleteMap_();
        sizeX_=map.sizeX_;
        sizeY_=map.sizeY_;
        resolution_=map.resolution_;
        originX_=map.originX_;
        originY_=map.originY_;
        initMap_(sizeX_,sizeY_);
        memcpy(mapChar_,map.mapChar_,sizeX_*sizeY_*sizeof(unsigned char));
    }
    costmap_base::~costmap_base() {
        deleteMap_();
        delete mutex_;
    }




    //convert from two map coordinates to index of 1-D array
    unsigned int costmap_base::get_MapToIndex(unsigned int mapX, unsigned int mapY) const {return mapX*sizeX_+sizeY_;}
    //convert from index of 1-D array to two map coordinates
    void costmap_base::indexToMap(unsigned int index, unsigned int& mapX,unsigned int& mapY) const{
        mapY=index/sizeX_;
        mapX=index-(mapY*sizeX_);
    }
    //convert from map coordinates(start from 0) to world coordinates
    void costmap_base::mapToWorld(unsigned int mapX,unsigned int mapY,double& worldX,double& worldY) const{
        worldX=originX_+(mapX+0.5)*resolution_;
        worldY=originY_+(mapY+0.5)*resolution_;
    }
    //convert from world coordinates to map coordinates
    bool costmap_base::worldToMap(double worldX,double worldY,unsigned int& mapX,unsigned int& mapY) const{
        if(worldX<originX_||worldY<originY_){return false;}
        mapX=(int)((worldX-originX_)/resolution_);
        mapY=(int)((worldY-originY_)/resolution_);
        if(mapX<sizeX_&&mapY<sizeY_){return true;}
        return false;
    }
    void costmap_base::worldToMapBounds(double worldX, double worldY, int &mapX, int &mapY) const {
        if(worldX<originX_)
            {mapX=0;}
        else if(worldX>=originX_+resolution_*sizeX_)
            {mapX=sizeX_-1;}
        else{mapX=(int)((worldX-originX_)/resolution_);}

        if(worldY<originY_)
            {mapY=0;}
        else if(worldY>=originY_+resolution_*sizeY_)
            {mapX=sizeY_-1;}
        else{mapY=(int)((worldY-originY_)/resolution_);}

    }

    unsigned int costmap_base::worldToMapDistance(double distWorld){
        double distCell=std::max(0.0,std::ceil(distWorld/resolution_));
        return (unsigned int)distCell;
    }

    template <class cellType>
    void costmap_base::rayTraceLine(cellType cellMark, unsigned int x0, unsigned int y0, unsigned int x1,
                                    unsigned int y1, unsigned int maxLength) {
        int dx=x1-x0;
        int dy=y1-y0;

        unsigned int abs_dx=abs(dx);
        unsigned int abs_dy=abs(dy);

        int offset_dx=sign(dx);
        int offset_dy=sign(dy)*sizeX_;

        unsigned int offset=y0*sizeX_+x0;
        //scale based on the given raytraceRange
        double dist=std::hypot(dx,dy);
        double scale=(dist==0)? 1.0:std::min(1.0,maxLength/dist);

        //x line is dominant
        if (abs_dx >= abs_dy){
            int error_y = abs_dx / 2;
            bresenham2D(cellMark, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        }
        else {
            // otherwise y is dominant
            int error_x = abs_dy / 2;
            bresenham2D(cellMark, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                        (unsigned int) (scale * abs_dy));
        }
    }


    template<class cellType>
   void costmap_base::bresenham(cellType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                                int offset_b, unsigned int offset, unsigned int max_length) {
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i)
        {
            at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }



    //get the pointer of map(cell)
    unsigned char* costmap_base::get_mapChar() const{return mapChar_;}
    unsigned char costmap_base::get_cost(unsigned int mapX,unsigned int mapY) const{return mapChar_[get_MapToIndex(mapX,mapY)];}

    //get the number of cells in the map X
    unsigned int costmap_base::get_mapSizeX() const{return sizeX_;}
    //get the number of cells in the map Y
    unsigned int costmap_base::get_mapSizeY() const{return sizeY_;}
    //get the size(m) of map X(from origin point to last point)
    double costmap_base::get_mapSizeX_meter() const{return (sizeX_-1+0.5)*resolution_;}
    //get the size(m) of map Y(from origin point to last point)
    double costmap_base::get_mapSizeY_meter() const{return (sizeY_-1+0.5)*resolution_;}
    //get the x(m) of map's origin point in world coordinate
    double costmap_base::get_originX() const{return originX_;}
    //get the y(m) of map's origin point in world coordinate
    double costmap_base::get_originY() const{return originY_;}
    //get the resolution
    double costmap_base::get_Resolution() const{return resolution_;}
    //get the cost default value
    unsigned char costmap_base::get_costDefaultValue() const{return costDefaultValue_;}
    //get the mutex*
    boost::recursive_mutex* costmap_base::get_mutex() const{return mutex_;}



    void costmap_base::updateOrigin(double originX_new, double originY_new) {
        //compute the original point change as the number of cell(can be positive or negative)
        int cellOriginSubX=int((originX_new-originX_)*resolution_);
        int cellOriginSubY=int((originY_new-originY_)*resolution_);
        if(cellOriginSubX==0&&cellOriginSubY==0){return;}
        //compute the new origin point suitable for the current resolution
        double mapOriginX_new=originX_+cellOriginSubX*resolution_;
        double mapOriginY_new=originY_+cellOriginSubY*resolution_;
        //convert the size of map from (unsigned int) to (int)
        int sizeX_int=sizeX_;
        int sizeY_int=sizeY_;
        //compute the coordinates(number of cell) of the overlapping area on the source map
        int bottomLeftX=min(max(cellOriginSubX,0),sizeX_int);
        int bottomLeftY=min(max(cellOriginSubY,0),sizeY_int);
        int upperRightX=min(max(cellOriginSubX+sizeX_int,0),sizeX_int);
        int upperRightY=min(max(cellOriginSubY+sizeY_int,0),sizeY_int);
        //compute the size of overlapping region
        unsigned int sizeX_region=upperRightX-bottomLeftX;
        unsigned int sizeY_region=upperRightY-bottomLeftY;
        //Define a temporary map storage overlap area
        unsigned char* temp_map=new unsigned char[sizeX_region*sizeY_region];
        //copy the region from source map to temporary map
        copyMapRegion(mapChar_,bottomLeftX,bottomLeftY,sizeX_,temp_map,0,0,sizeX_region,sizeX_region,sizeY_region);
        resetMap_();
        originX_=mapOriginX_new;
        originY_=mapOriginY_new;
        //compute the position of the starting point of the overlapping area on the updated map
        int startX_newMap=bottomLeftX-cellOriginSubX;
        int startY_newMap=bottomLeftY-cellOriginSubY;
        //copy the region from temporary map to updated map
        copyMapRegion(temp_map,0,0,sizeX_region,mapChar_,startX_newMap,startY_newMap,sizeX_,sizeX_region,sizeY_region);
        delete[] temp_map;
    }
    void costmap_base::resizeMap(unsigned int sizeX, unsigned int sizeY, double resolution, double originX,
                                 double originY){
        sizeX_=sizeX;
        sizeY_=sizeY;
        originX_=originX;
        originY_=originY;
        resolution_=resolution;
        initMap_(sizeX,sizeY);
        resetMap_();
    }
    void costmap_base::resetMap(unsigned int boundX0, unsigned int boundY0, unsigned int boundXn,
                                unsigned int boundYn) {
        boost::unique_lock<boost::recursive_mutex> lock(*mutex_);
        unsigned int lenX=boundXn-boundX0;
        for(unsigned int y=boundY0*sizeX_+boundX0;y<boundYn*sizeX_+boundX0;y+=sizeX_){
            memset(mapChar_+y,costDefaultValue_,lenX*sizeof(unsigned char));
        }
    }
    void costmap_base::set_cost(unsigned int mapX,unsigned int mapY,unsigned char cost){
        mapChar_[get_MapToIndex(mapX,mapY)]=cost;
    }
    void costmap_base::set_costDefaultValue(unsigned char costDefasultValue) {
        costDefaultValue_=costDefasultValue;
    }


    void costmap_base::initMap_(unsigned int sizeX, unsigned int sizeY){
        boost::unique_lock<boost::recursive_mutex> lock(*mutex_);
        delete[] mapChar_;
        mapChar_=new unsigned char[sizeX*sizeY];

    }
    void costmap_base::resetMap_() {
        boost::unique_lock<boost::recursive_mutex> lock(*mutex_);
        memset(mapChar_,costDefaultValue_,sizeX_*sizeY_*sizeof(unsigned char));
    }
    void costmap_base::deleteMap_() {
        boost::unique_lock<boost::recursive_mutex> lock(*mutex_);
        delete [] mapChar_;
        mapChar_= nullptr;
    }
    void costmap_base::copyMapRegion(unsigned char *sourceMap, unsigned int sourceX, unsigned int sourceY,
                                     unsigned int sourceSizeX, unsigned char *destMap, unsigned int destX,
                                     unsigned int destY, unsigned int destSizeX, unsigned int regionSizeX,
                                     unsigned int regionSizeY) {
        //compute the starting point(pointer to cell) of region in the source map and destination map
        unsigned char* source_start=sourceMap+(sourceY*sourceSizeX+sourceX);
        unsigned char* dest_start=destMap+(destY*destSizeX+destX);
        //start copy
        for(unsigned int i=0;i<regionSizeY;++i){
            memcpy(dest_start,source_start,regionSizeX*sizeof(unsigned char));
            source_start+=sourceSizeX;
            dest_start+=destSizeX;
        }
    }
}//namespace costmap end
