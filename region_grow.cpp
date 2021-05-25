#include "region_grow.h"
#include "math.h"
#include <iostream>

namespace rg {

void RGProblem::addObstacle(Eigen::Vector3d new_obstacle_vertices) 
{
  Eigen::Vector3d obstacle(round(new_obstacle_vertices.x()), round(new_obstacle_vertices.y()), round(new_obstacle_vertices.z())); 
  //if (!isObstacle(obstacle))
  this->obstacle_pts.push_back(obstacle);
}

void RGProblem::setSeedPoint(Eigen::Vector3d point) 
{
  this->seed = Eigen::Vector3d(round(point.x()), round(point.y()), round(point.z()));
}

Eigen::Vector3d RGProblem::getSeed() const 
{
  return this->seed;
}

const std::vector<Eigen::Vector3d>& RGProblem::getObstacles() const 
{
  return this->obstacle_pts;
}

bool RGProblem::isObstacle(const Eigen::Vector3d &point) const
{
  return is_point_in_vector(this->obstacle_pts, point);
}

bool RGProblem::isObstacleOnPath(const Eigen::Vector3d &src, const Eigen::Vector3d &dst) const
{
  for (auto it = this->obstacle_pts.cbegin(); it != this->obstacle_pts.cend(); ++it)
  {
    if ( ( (it->x() >= src.x()) ^ (it->x() > dst.x()) )
    && ( (it->y() >= src.y()) ^ (it->y() > dst.y()) )
    && ( (it->z() >= src.z()) ^ (it->z() > dst.z()) ) )
    {
      return true;
    }
  }

  return false;
}

int RGProblem::getSubproblem(const Eigen::Vector3d &seed_point, const int range, RGProblem *subproblem)
{
  subproblem->setSeedPoint(seed_point);

  for (auto it = this->obstacle_pts.cbegin(); it != this->obstacle_pts.cend(); ++it)
  {
    Eigen::Vector3d diff = (*it) - seed_point;
    
    if (diff.norm() < range)
    {
      subproblem->addObstacle(*it);
    }
  }

  return OK;
}

/*-------------------------------*/

bool is_required_point(const RGProblem &problem, const RGRegion &region, const Eigen::Vector3d &point)
{
  for (auto it = region.region_bound_pts.cbegin(); it != region.region_bound_pts.cend(); ++it)
  {
    if (problem.isObstacleOnPath((*it).point, point))
      return false;
  }

  return true;
}

inline bool is_point_in_vector(const std::vector<Eigen::Vector3d> &pv, const Eigen::Vector3d &point)
{
  return std::find(pv.begin(), pv.end(), point) != pv.end();
}

int get_neighbor_points(const Eigen::Vector3d &point, std::vector<Eigen::Vector3d> &neighbor_points)
{
  neighbor_points.clear();
  neighbor_points.push_back(Eigen::Vector3d(point.x() - 1, point.y(), point.z()));
  neighbor_points.push_back(Eigen::Vector3d(point.x() + 1, point.y(), point.z()));
  neighbor_points.push_back(Eigen::Vector3d(point.x(), point.y() - 1, point.z()));
  neighbor_points.push_back(Eigen::Vector3d(point.x(), point.y() + 1, point.z()));
  neighbor_points.push_back(Eigen::Vector3d(point.x(), point.y(), point.z() - 1));
  neighbor_points.push_back(Eigen::Vector3d(point.x(), point.y(), point.z() + 1));

  return OK;
}

int get_new_candidiates(const RGProblem &problem, const RGRegion &region, std::vector<Eigen::Vector3d> &candidates)
{
  candidates.clear();
  for (auto it = region.region_bound_pts.cbegin(); it != region.region_bound_pts.cend(); ++it)
  {
    if (it->is_boundary)
      continue;
    
    std::vector<Eigen::Vector3d> neighbor_points;
    get_neighbor_points(it->point, neighbor_points);
   
    for (auto it2 = neighbor_points.cbegin(); it2 != neighbor_points.cend(); ++it2)
    {
      if (!(is_point_in_vector(candidates, *it2) || problem.isObstacle(*it2)))
        candidates.push_back(*it2);
    }
  }

  return OK;
}

RGRegion inflate_region(const RGProblem &problem, const RGOptions &options)
{
  RGRegion region(problem.getSeed());

  int iter_times = 0;
  int new_candidates_num = 0;
  
  do
  {
    std::vector<Eigen::Vector3d> executed_candidates;
    std::vector<RGPoint> new_bound_pts;

    new_candidates_num = 0;

    for (auto it = region.region_bound_pts.begin(); it != region.region_bound_pts.end(); ++it)
    {
      if (it->is_boundary)
      {
        new_bound_pts.push_back(*it);
        continue;
      }
      
      int flag = 0; /* the flag that the point has been put into the vector */
      std::vector<Eigen::Vector3d> neighbor_points;
      get_neighbor_points(it->point, neighbor_points);

      for (auto it2 = neighbor_points.cbegin(); it2 != neighbor_points.cend(); ++it2)
      {
        if (is_point_in_vector(region.inner_pts, *it2))
          continue;

        if (is_point_in_vector(executed_candidates, *it2))
          continue;

        region.inner_pts.push_back(it->point);
        executed_candidates.push_back(*it2);

        if (problem.isObstacle(*it2))
        {
          if (!flag)
          {
            if (!it->is_boundary)
            {
              it->is_boundary = true;
              new_bound_pts.push_back(*it);
              flag++;
            }
          }

          continue;
        }

        if (is_required_point(problem, region, *it2))
        {
          new_bound_pts.push_back(RGPoint(*it2));
          new_candidates_num++;
        }
        else
        {
          if (!flag)
          {
            if (!it->is_boundary)
            {
              it->is_boundary = true;
              new_bound_pts.push_back(*it);
              flag++;
            }
          }
        }
      }
       
    }
    
    region.region_bound_pts.swap(new_bound_pts);

    iter_times++;

    if (iter_times == options.iter_limit)
      break;
    
    std::cout << "iter time: " << iter_times << " " << "new candidate: " << new_candidates_num << std::endl;

  } while (new_candidates_num);

  return region;
}

}
