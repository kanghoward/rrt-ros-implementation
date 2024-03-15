#include "rrt_planner/rrt_planner.h"

int main(int argv, char ** argc)
{
  ros::init(argv, argc, "rrt_planner");
  ros::NodeHandle node;

  double resolution, occupied_thresh, free_thresh;
  int iterations, neighbourhood_radius, step_size, goal_radius;
  bool rrt_star;
  std::vector<double> origin;

  node.getParam("/iterations", iterations);
  node.getParam("/step_size", step_size);
  node.getParam("/goal_radius", goal_radius);
  node.getParam("/resolution", resolution);
  node.getParam("/neighbourhood_radius", neighbourhood_radius);
  node.getParam("/rrt_star", rrt_star);

  new rrt_planner::RRTPlanner(&node, iterations, step_size, goal_radius, resolution, neighbourhood_radius, rrt_star);
}
