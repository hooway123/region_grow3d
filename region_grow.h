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
  std::vector<Eigen::Vector3d> obstacle_pts;
  Eigen::Vector3d seed;

public:
  RGProblem() {}
  void addObstacle(Eigen::Vector3d new_obstacle_vertices);
  void setSeedPoint(Eigen::Vector3d point);
  const std::vector<Eigen::Vector3d>& getObstacles() const;
  Eigen::Vector3d getSeed() const;
  bool isObstacle(const Eigen::Vector3d &point) const;
  bool isObstacleOnPath(const Eigen::Vector3d &src, const Eigen::Vector3d &dst) const;
  int getSubproblem(const Eigen::Vector3d &seed_point, const int range, RGProblem *subproblem);
};

class RGPoint
{
public:
  Eigen::Vector3d point;
  bool is_boundary;
  RGPoint() {}
  RGPoint(int &x, int &y, int &z, bool is_bound)
  {
    this->point = Eigen::Vector3d(x, y, z);
    this->is_boundary = is_bound;
  }
  RGPoint(Eigen::Vector3d pt)
  {
    this->point = pt;
    this->is_boundary = false;
  }
};

class RGRegion 
{
public:
  std::vector<RGPoint> region_bound_pts;
  std::vector<Eigen::Vector3d> inner_pts;
  RGRegion() {}
  RGRegion(Eigen::Vector3d seed)
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

int get_neighbor_points(const Eigen::Vector3d &point, std::vector<Eigen::Vector3d> &neighbor_points); 
inline bool is_point_in_vector(const std::vector<Eigen::Vector3d> &pv, const Eigen::Vector3d &point);
RGRegion inflate_region(const RGProblem &problem, const RGOptions &options);
bool is_required_point(const RGProblem &problem, const RGRegion &region, const Eigen::Vector3d &point);
int get_new_candidiates(const RGProblem &problem, const RGRegion &region, std::vector<Eigen::Vector3d> &candidates);

} 

#endif