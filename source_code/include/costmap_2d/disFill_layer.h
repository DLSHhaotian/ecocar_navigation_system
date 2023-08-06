
#ifndef COSTMAP_2D_DISFILL_LAYER_H_
#define COSTMAP_2D_DISFILL_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/disFillPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace costmap_2d
{
    class CellIndex
    {
    public:
        CellIndex(unsigned int x, unsigned int y) :
                 x_(x), y_(y)
        {
        }
        unsigned int x_, y_;
    };

class disFillLayer : public Layer
{
public:
    disFillLayer();

  virtual ~disFillLayer()
  {
    if (dsrv_)
        delete dsrv_;
  }

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void matchSize();

  virtual void reset() { onInitialize(); }


protected:
  boost::recursive_mutex* disFill_access_;

  double resolution_;

private:

  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<costmap_2d::disFillPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::disFillPluginConfig &config, uint32_t level);
  bool haveTwoRoadSidePoints(std::multimap<unsigned int,unsigned int>& obsCell, unsigned int key, int size);
  unsigned char gridValue_max;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_DISFILL_LAYER_H_
