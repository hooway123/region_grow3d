#include <iostream>
#include <eigen3/Eigen/Core>
#include "region_grow.h"

int test_is_required_point()
{
  rg::RGRegion region;
  rg::RGProblem problem;
  problem.setSeedPoint(Eigen::Vector3i(3, 3, 3));

  Eigen::Vector3d obs;
  // Inflate a region inside a 1x1 box 
  
  obs << 0, 0.1, 0.3;
  problem.addObstacle(obs);
  obs << 0, 0, 5; 
  problem.addObstacle(obs);
  obs << 0, 5, 0;
  problem.addObstacle(obs);
  obs << 0, 5, 5;
  problem.addObstacle(obs);
  obs << 5, 0, 0;
  problem.addObstacle(obs);
  obs << 5, 0, 5;
  problem.addObstacle(obs);
  obs << 5, 5, 0;
  problem.addObstacle(obs);
  obs << 5, 5, 5;
  problem.addObstacle(obs);
  obs << 5, 5, 5;
  problem.addObstacle(obs);

  if (problem.isObstacleOnPath(Eigen::Vector3i(3, 0, 0), Eigen::Vector3i(8, 1, 0)))
  {
    std::cout << "yes" << std::endl;
  }
  else
  {
    std::cout << "no" << std::endl;
  }

  return 0;
}

int main(int argc, char** argv) {
  //test_is_required_point();
  
  
  rg::RGProblem problem;

  problem.setSeedPoint(Eigen::Vector3i(3, 3, 3));

  Eigen::Vector3i obs;

  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      obs << 0, i, j;
      problem.addObstacle(obs);
    }
  }

  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      obs << 5, i, j;
      problem.addObstacle(obs);
    }
  }

  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      obs << i, 0, j;
      problem.addObstacle(obs);
    }
  }

  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      obs << i, 5, j;
      problem.addObstacle(obs);
    }
  }

  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      obs << i, j, 5;
      problem.addObstacle(obs);
    }
  }

  for (int i = 1; i < 4; i++)
  {
    for (int j = 1; j < 4; j++)
    {
      obs << i, j, 5;
      problem.addObstacle(obs);
    }
  }

  rg::RGProblem subproblem;
  problem.getSubproblem(Eigen::Vector3i(3, 3, 3), 10, &subproblem);
  //Eigen::Vector3i point = subproblem.getSeed();
  //std::cout << "sssss: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
  //std::vector<Eigen::Vector3i> obstacles = problem.getObstacles();

  //for (auto it = obstacles.begin(); it != obstacles.end(); ++it)
  //{
  //  std::cout << it->x() << "," << it->y() << "," << it->z() << std::endl;
  //}

  rg::RGOptions options;

  options.iter_limit = 100;

  rg::RGRegion region = inflate_region(problem, options);

  for (auto it = region.region_bound_pts.begin(); it != region.region_bound_pts.end(); ++it)
  {
    std::cout << it->point.x() << "," << it->point.y() << "," << it->point.z() << std::endl;
  }



  return 0;
}
