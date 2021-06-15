#include "region_grow.h"
#include "math.h"
#include <iostream>

namespace rg {

void RGProblem::addObstacle(Eigen::Vector3d new_obstacle_vertices) 
{
  Eigen::Vector3i obstacle(round(new_obstacle_vertices.x()), round(new_obstacle_vertices.y()), round(new_obstacle_vertices.z())); 
  //if (!isObstacle(obstacle))
  this->obstacle_pts.push_back(obstacle);
}

void RGProblem::addObstacle(Eigen::Vector3i new_obstacle_vertices) 
{
  Eigen::Vector3i obstacle(round(new_obstacle_vertices.x()), round(new_obstacle_vertices.y()), round(new_obstacle_vertices.z())); 
  //if (!isObstacle(obstacle))
  this->obstacle_pts.push_back(obstacle);
}

void RGProblem::setSeedPoint(Eigen::Vector3i point) 
{
  this->seed = Eigen::Vector3i(round(point.x()), round(point.y()), round(point.z()));
}

Eigen::Vector3i RGProblem::getSeed() const 
{
  return this->seed;
}

const std::vector<Eigen::Vector3i>& RGProblem::getObstacles() const 
{
  return this->obstacle_pts;
}

bool RGProblem::isObstacle(const Eigen::Vector3i &point) const
{
  return is_point_in_vector(this->obstacle_pts, point);
}

bool RGProblem::isObstacleOnPath(const Eigen::Vector3i &src, const Eigen::Vector3i &dst) const
{
  /* 1.square rule */
  /*
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
  */

  /* 2.triangle rule */
  
  for (auto it = this->obstacle_pts.cbegin(); it != this->obstacle_pts.cend(); ++it)
  {
    if ( ( (it->x() >= src.x()) ^ (it->x() > dst.x()) )
      && ( (it->y() >= src.y()) ^ (it->y() > dst.y()) )
      && ( (it->z() >= src.z()) ^ (it->z() > dst.z()) ) )
    {
      Eigen::Vector3i v0 = src - this->seed;
      Eigen::Vector3i v1 = dst - this->seed;
      Eigen::Vector3i v2 = (*it) - this->seed;

      int dot00 = v0.dot(v0);
      int dot01 = v0.dot(v1);
      int dot02 = v0.dot(v2);
      int dot11 = v1.dot(v1);
      int dot12 = v1.dot(v2);

      int inverDeno = dot00 * dot11 - dot01 * dot01; 

      if (!inverDeno)
        continue;

      float u = (dot11 * dot02 - dot01 * dot12) / (inverDeno * 1.0);

      if (u < 0 || u > 1)
        continue;

      float v = (dot00 * dot12 - dot01 * dot02) / (inverDeno * 1.0);

      if (v < 0 || v > 1)
        continue;

      if (u + v <= 1)
      {
        //std::cout << "u: " << u << " v: " << v  << " inverDeno: " << inverDeno << std::endl;
        //std::cout << "obs: " << " x: " << it->x() << " y: " << it->y() << " z: " << it->z() << std::endl;
        //std::cout << "src: " << " x: " << src.x() << " y: " << src.y() << " z: " << src.z() << std::endl;
        //std::cout << "dst: " << " x: " << dst.x() << " y: " << dst.y() << " z: " << dst.z() << std::endl;
        return true;
      }
    }
  }

  return false;
  
}

bool RGProblem::isObstacleOnPath(const Eigen::Vector3i &src, const Eigen::Vector3i &dst, std::vector<Eigen::Vector3i> &obstacles) const
{
  /* 1.square rule */
  /*
  for (auto it = obstacles.cbegin(); it != obstacles.cend(); ++it)
  {
    if ( ( (it->x() >= src.x()) ^ (it->x() > dst.x()) )
    && ( (it->y() >= src.y()) ^ (it->y() > dst.y()) )
    && ( (it->z() >= src.z()) ^ (it->z() > dst.z()) ) )
    {
      return true;
    }
  }

  return false;
   */

  /* 2.triangle rule */
  
  for (auto it = obstacles.cbegin(); it != obstacles.cend(); ++it)
  {
    if ( ( (it->x() >= src.x()) ^ (it->x() > dst.x()) )
      && ( (it->y() >= src.y()) ^ (it->y() > dst.y()) )
      && ( (it->z() >= src.z()) ^ (it->z() > dst.z()) ) )
    {
      Eigen::Vector3i v0 = src - this->seed;
      Eigen::Vector3i v1 = dst - this->seed;
      Eigen::Vector3i v2 = (*it) - this->seed;

      int dot00 = v0.dot(v0);
      int dot01 = v0.dot(v1);
      int dot02 = v0.dot(v2);
      int dot11 = v1.dot(v1);
      int dot12 = v1.dot(v2);

      int inverDeno = dot00 * dot11 - dot01 * dot01; 

      if (!inverDeno)
        continue;

      float u = (dot11 * dot02 - dot01 * dot12) / (inverDeno * 1.0);

      if (u <= 0 || u >= 1)
        continue;

      float v = (dot00 * dot12 - dot01 * dot02) / (inverDeno * 1.0);

      if (v <= 0 || v >= 1)
        continue;

      if (u + v < 1)
      {
        //std::cout << "u: " << u << " v: " << v  << " inverDeno: " << inverDeno << std::endl;
        //std::cout << "obs: " << " x: " << it->x() << " y: " << it->y() << " z: " << it->z() << std::endl;
        //std::cout << "src: " << " x: " << src.x() << " y: " << src.y() << " z: " << src.z() << std::endl;
        //std::cout << "dst: " << " x: " << dst.x() << " y: " << dst.y() << " z: " << dst.z() << std::endl;
        return true;
      }
    }
  }

  return false;
  
}

int RGProblem::getSubproblem(const Eigen::Vector3i &seed_point, const int range, RGProblem *subproblem)
{
  subproblem->setSeedPoint(seed_point);

  for (auto it = this->obstacle_pts.cbegin(); it != this->obstacle_pts.cend(); ++it)
  {
    Eigen::Vector3i diff = (*it) - seed_point;
    
    if (diff.norm() < range)
    {
      subproblem->addObstacle(*it);
    }
  }

  return OK;
}

int RGProblem::getInnerObstacles(const Eigen::Vector3i &upper_vertex, const Eigen::Vector3i &lower_vertex, std::vector<Eigen::Vector3i> *obstacles) const
{
  obstacles->clear();

  for (auto it = this->obstacle_pts.cbegin(); it != this->obstacle_pts.cend(); ++it)
  {
    if ( (it->x() <= upper_vertex.x() + 1) && (it->x() >= lower_vertex.x() - 1)
    && (it->y() <= upper_vertex.y() + 1) && (it->y() >= lower_vertex.y() - 1)
    && (it->z() <= upper_vertex.z() + 1) && (it->z() >= lower_vertex.z() - 1) )
      obstacles->push_back(*it);
  }
}

/*-------------------------------*/

int get_important_obstacles(const RGProblem &problem, const RGRegion &region, std::vector<Eigen::Vector3i> *obstacles)
{
  int max_x = -10000;
  int min_x = 10000; 
  int max_y = -10000;
  int min_y = 10000; 
  int max_z = -10000;
  int min_z = 10000;
  for (auto it = region.region_bound_pts.cbegin(); it != region.region_bound_pts.cend(); ++it)
  {
    if (it->point.x() > max_x)
      max_x = it->point.x();
    if (it->point.x() < min_x)
      min_x = it->point.x();
    if (it->point.y() > max_y)
      max_y = it->point.y();
    if (it->point.y() < min_y)
      min_y = it->point.y();
    if (it->point.z() > max_z)
      max_z = it->point.z();
    if (it->point.z() < min_z)
      min_z = it->point.z();
  }

  problem.getInnerObstacles(Eigen::Vector3i(max_x, max_y, max_z), Eigen::Vector3i(min_x, min_y, min_z), obstacles);

  return OK;
}
/*
bool is_required_point(const RGProblem &problem, const RGRegion &region, const Eigen::Vector3i &point)
{
  for (auto it = region.region_bound_pts.cbegin(); it != region.region_bound_pts.cend(); ++it)
  {
    if (problem.isObstacleOnPath((*it).point, point))
      return false;
  }

  return true;
}
*/
bool is_required_point(const RGProblem &problem, const RGRegion &region, const Eigen::Vector3i &point, std::vector<Eigen::Vector3i> &obstacles)
{
  for (auto it = region.region_bound_pts.cbegin(); it != region.region_bound_pts.cend(); ++it)
  {
    if (problem.isObstacleOnPath((*it).point, point, obstacles))
      return false;
  }

  return true;
}

inline bool is_point_in_vector(const std::vector<Eigen::Vector3i> &pv, const Eigen::Vector3i &point)
{
  return std::find(pv.begin(), pv.end(), point) != pv.end();
}

int get_neighbor_points(const Eigen::Vector3i &point, std::vector<Eigen::Vector3i> &neighbor_points)
{
  neighbor_points.clear();
  neighbor_points.push_back(Eigen::Vector3i(point.x() - 1, point.y(), point.z()));
  neighbor_points.push_back(Eigen::Vector3i(point.x() + 1, point.y(), point.z()));
  neighbor_points.push_back(Eigen::Vector3i(point.x(), point.y() - 1, point.z()));
  neighbor_points.push_back(Eigen::Vector3i(point.x(), point.y() + 1, point.z()));
  neighbor_points.push_back(Eigen::Vector3i(point.x(), point.y(), point.z() - 1));
  neighbor_points.push_back(Eigen::Vector3i(point.x(), point.y(), point.z() + 1));

  return OK;
}

int get_new_candidiates(const RGProblem &problem, const RGRegion &region, std::vector<Eigen::Vector3i> &candidates)
{
  candidates.clear();
  for (auto it = region.region_bound_pts.cbegin(); it != region.region_bound_pts.cend(); ++it)
  {
    if (it->is_boundary)
      continue;
    
    std::vector<Eigen::Vector3i> neighbor_points;
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
    std::vector<Eigen::Vector3i> executed_candidates;
    std::vector<RGPoint> new_bound_pts;
    
    std::vector<Eigen::Vector3i> obstacles;
    get_important_obstacles(problem, region, &obstacles);
    std::cout << "obstaclecount" << obstacles.size() << std::endl;

    new_candidates_num = 0;

    for (auto it = region.region_bound_pts.begin(); it != region.region_bound_pts.end(); ++it)
    {
      if (it->is_boundary)
      {
        new_bound_pts.push_back(*it);
        continue;
      }
      
      int flag = 0; /* the flag that the point has been put into the vector */
      std::vector<Eigen::Vector3i> neighbor_points;
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

        if (is_required_point(problem, region, *it2, obstacles))
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
