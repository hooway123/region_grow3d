#include <iostream>
#include <eigen3/Eigen/Core>
#include "region_grow.h"

int test_is_required_point()
{
  rg::RGRegion region;
  rg::RGProblem problem;
  problem.setSeedPoint(Eigen::Vector3d(3, 3, 3));

  Eigen::Vector3d obs;
  // Inflate a region inside a 1x1 box 
  
  obs << 0, 0, 0;
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

  if (problem.isObstacleOnPath(Eigen::Vector3d(3, 4, 13), Eigen::Vector3d(3, 3, 3)))
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

  problem.setSeedPoint(Eigen::Vector3d(3, 3, 3));

  Eigen::Vector3d obs;
  // Inflate a region inside a 1x1 box 
  
  obs << 0, 0, 0;
  problem.addObstacle(obs);
  obs << 0, 0, 10; 
  problem.addObstacle(obs);
  obs << 0, 10, 0;
  problem.addObstacle(obs);
  obs << 0, 10, 10;
  problem.addObstacle(obs);
  obs << 10, 0, 0;
  problem.addObstacle(obs);
  obs << 10, 0, 10;
  problem.addObstacle(obs);
  obs << 10, 10, 0;
  problem.addObstacle(obs);
  obs << 10, 10, 10;
  problem.addObstacle(obs);

  rg::RGOptions options;

  options.iter_limit = 100;

  rg::RGRegion region = inflate_region(problem, options);

  for (auto it = region.region_bound_pts.begin(); it != region.region_bound_pts.end(); ++it)
  {
    std::cout << it->point.x() << "," << it->point.y() << "," << it->point.z() << std::endl;
  }



  return 0;
}