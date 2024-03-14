#include "rrt_planner/rrt_planner.h"

int main(int argv, char ** argc)
{
  ros::init(argv, argc, "rrt_planner");
  ros::NodeHandle node;

  double resolution, occupied_thresh, free_thresh;
  int iterations, neighbourhood_radius, step_size, goal_radius;
  bool rrt_star;
  std::vector<double> origin;
  // Get parameters from the parameter server
  // node.getParam("/occupied_thresh", occupied_thresh);
  // node.getParam("/free_thresh", free_thresh);
  // node.getParam("/origin", origin);
  node.getParam("/iterations", iterations);
  node.getParam("/step_size", step_size);
  node.getParam("/goal_radius", goal_radius);
  node.getParam("/resolution", resolution);
  node.getParam("/neighbourhood_radius", neighbourhood_radius);
  node.getParam("/rrt_star", rrt_star);


  // // Print the retrieved parameters
  
  // ROS_INFO("Occupied Threshold: %f", occupied_thresh);
  // ROS_INFO("Free Threshold: %f", free_thresh);

  // // Print the origin vector
  // ROS_INFO("Origin: [%f, %f, %f]", origin[0], origin[1], origin[2]);

  
  




  new rrt_planner::RRTPlanner(&node, iterations, step_size, goal_radius, resolution, neighbourhood_radius, rrt_star);
}
