#include <eigen3/Eigen/Core>
#include <vector>

#ifndef _REGION_GROW_H
#define _REGION_GROW_H

#define OK    0
#define ERROR 1

namespace rg {

class RGProblem 
{
private:
  std::vector<Eigen::Vector3i> obstacle_pts;
  Eigen::Vector3i seed;

public:
  RGProblem() {}
  void addObstacle(Eigen::Vector3d new_obstacle_vertices);
  void addObstacle(Eigen::Vector3i new_obstacle_vertices);
  void setSeedPoint(Eigen::Vector3i point);
  const std::vector<Eigen::Vector3i>& getObstacles() const;
  Eigen::Vector3i getSeed() const;
  bool isObstacle(const Eigen::Vector3i &point) const;
  bool isObstacleOnPath(const Eigen::Vector3i &src, const Eigen::Vector3i &dst) const;
  bool isObstacleOnPath(const Eigen::Vector3i &src, const Eigen::Vector3i &dst, std::vector<Eigen::Vector3i> &obstacles) const;
  int getSubproblem(const Eigen::Vector3i &seed_point, const int range, RGProblem *subproblem);
  int getInnerObstacles(const Eigen::Vector3i &upper_vertex, const Eigen::Vector3i &lower_vertex, std::vector<Eigen::Vector3i> *obstacles) const;
};

class RGPoint
{
public:
  Eigen::Vector3i point;
  bool is_boundary;
  RGPoint() {}
  RGPoint(int &x, int &y, int &z, bool is_bound)
  {
    this->point = Eigen::Vector3i(x, y, z);
    this->is_boundary = is_bound;
  }
  RGPoint(Eigen::Vector3i pt)
  {
    this->point = pt;
    this->is_boundary = false;
  }
};

class RGRegion 
{
public:
  std::vector<RGPoint> region_bound_pts;
  std::vector<Eigen::Vector3i> inner_pts;
  RGRegion() {}
  RGRegion(Eigen::Vector3i seed)
  {
    RGPoint point(seed);
    region_bound_pts.push_back(point);
  }
  
};

class RGOptions
{
public:
  int iter_limit;
  RGOptions():
    iter_limit(100) {};
};

int get_important_obstacles(const RGProblem &problem, const RGRegion &region, std::vector<Eigen::Vector3i> &obstacles); 
int get_neighbor_points(const Eigen::Vector3i &point, std::vector<Eigen::Vector3i> &neighbor_points); 
inline bool is_point_in_vector(const std::vector<Eigen::Vector3i> &pv, const Eigen::Vector3i &point);
RGRegion inflate_region(const RGProblem &problem, const RGOptions &options);
bool is_required_point(const RGProblem &problem, const RGRegion &region, const Eigen::Vector3i &point);
//bool is_required_point(const RGProblem &problem, const RGRegion &region, const Eigen::Vector3i &point, std::vector<Eigen::Vector3i> obstacles);
int get_new_candidiates(const RGProblem &problem, const RGRegion &region, std::vector<Eigen::Vector3i> &candidates);

} 

#endif