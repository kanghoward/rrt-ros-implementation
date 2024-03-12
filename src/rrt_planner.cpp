#include "rrt_planner/rrt_planner.h"
#include <cmath>
#include <stdexcept>

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node, int iterations, int step_size, int goal_radius, double resolution)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false),
  iterations_(iterations),
  step_size_(step_size),
  goal_radius_(goal_radius),
  resolution_(resolution)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");
  ROS_INFO_STREAM(iterations);
  ROS_INFO_STREAM(step_size);
  ROS_INFO_STREAM(goal_radius);

  // Subscribe to map topic
  map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);

  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  // This loops until the node is running, will exit when the node is killed
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      buildMapImage();
      drawGoalInitPose();
      plan();
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert mas to Point2D
  poseToPoint(init_pose_, msg->pose.pose);

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Initial pose obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("Goal obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, goal_radius_, cv::Scalar(0, 100, 0)); // added a line to draw area surrounding goal
    drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

void RRTPlanner::plan()
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;

  // // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  // //       path through the map starting from the initial pose and ending at the goal pose

  // Main RRT (Rapidly-Exploring Random Tree) algorithm
  // RRT Pseudocode
  // Qgoal //region that identifies success
  // Counter = 0 //keeps track of iterations
  // lim = n //number of iterations algorithm should run for
  // G(V,E) //Graph containing edges and vertices, initialized as empty
  // While counter < lim:
  //     Xnew  = RandomPosition()
  //     if IsInObstacle(Xnew) == True:
  //         continue
  //     Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
  //     Link = Chain(Xnew,Xnearest)
  //     G.append(Link)
  //     if Xnew in Qgoal:
  //         Return G
  // Return G

  // max iteration limit (defined above as a C++ macro constant) & iteration counter
  int counter = 0;

  ROS_INFO_STREAM(iterations_);
  ROS_INFO_STREAM(step_size_);
  ROS_INFO_STREAM(goal_radius_);
  ROS_INFO_STREAM(resolution_);

  // init graph with initial position vertex
  rrt_graph.reset();
  ROS_INFO("Init RRT Graph.");
  rrt_graph.addVertex(init_pose_);

  // core RRT logic loop
  while (counter < iterations_) {
    ROS_INFO_STREAM("Point Number: " << counter);
    // generate random point within map
    Point2D xRandom = RandomPosition();

    //find nearest vertex + rescale random point
    int xNearestIndex = rrt_graph.findNearestVertex(xRandom);
    Point2D xNearest = rrt_graph.getVertex(xNearestIndex);
    Point2D xNew = rescalePoint(xNearest, xRandom, step_size_);
    
    // logic to check if potential edge/rescaled point will intersect an obstacle
    if (!isPointUnoccupied(xNew)) {continue;}
    if (isEdgeIntersectingObstacle(xNearest,xNew)) {continue;}

    //if all tests pass, create new edge and add to RRT graph
    xNew.setParent(xNearestIndex);
    Chain(rrt_graph, xNew, xNearestIndex);
    drawCircle(xNew, 2, cv::Scalar(0, 0, 255));
    drawLine(xNew, xNearest, cv::Scalar (0, 0, 255), 1);
    // ROS_INFO_STREAM("New X: " << xNew.x());
    displayMapImage();

    // Check if xNew is in the Qgoal region + end iteration if found
    if (fabs(xNew.x() - goal_.x()) < goal_radius_ && fabs(xNew.y() - goal_.y()) < goal_radius_) {
      // add one final edge to the goal node
      int xNew_index = static_cast<int>(rrt_graph.getVertexCount()) - 1;
      goal_.setParent(xNew_index);
      Chain(rrt_graph, goal_, xNew_index);
      drawCircle(xNew, 2, cv::Scalar(0, 0, 255));
      drawLine(goal_, xNew, cv::Scalar (0, 0, 255), 1);
      displayMapImage();
      ROS_INFO("A path has been found.");

      publishPath();
      return;
    } 
    counter++;
  }
  //failed to find a path within iteration limit
  ROS_INFO_STREAM("No path from start to goal found within iteration limit. Total iterations: " << iterations_);
}


void RRTPlanner::publishPath()
{
  // Create new Path msg
  nav_msgs::Path path;
  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT

  // backtrack from goal node
  std::vector<Point2D> backtracked_path = rrt_graph.backtrackPath(goal_.getParent());
  // double scale_factor = 0.1; //same value as resolution in map.yaml file
  // convert point to pose
  for (const auto& point : backtracked_path) {
    geometry_msgs::PoseStamped pose;
    // SWAP X Y DUE TO OPENCV'S IN-BUILT COORDINATE FLIP
    pose.pose.position.x = point.y() * resolution_;
    pose.pose.position.y = point.x() * resolution_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    pose.header = path.header;
    pose.header.stamp = ros::Time::now();  // Timestamp each pose

    path.poses.push_back(pose);
  }
  // Publish the calculated path

  // ROS_INFO_STREAM("Path is : " << path);
  path_pub_.publish(path);

  displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map
  return (!map_grid_->data[toIndex(p.x(), p.y())]);
  // future improvments: can add a exception handler to check if point selected is within map bounds
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

// ADDITIONAL UTLITY FUNCTIONS (BEGIN SECTION)

Point2D RRTPlanner::RandomPosition() {
  return {std::rand() % (int) map_grid_-> info.width, std::rand() % (int) map_grid_->info.height};
}


Point2D RRTPlanner::rescalePoint(const Point2D& curr, const Point2D& randPoint, int stepDistance) {
        int dx = randPoint.x() - curr.x();
        int dy = randPoint.y() - curr.y();

        // Calculate the distance between the current point and the random point
        double currentDistance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        // ROS_INFO_STREAM(currentDistance);
        // Check if the current distance is zero to avoid division by zero
        if (currentDistance == 0.0) {
            // If the current distance is zero, return the current point
            return curr;
        }

        // Calculate the scaling factor to achieve the desired distance
        double scalingFactor = stepDistance / currentDistance;

        // Calculate the rescaled point
        Point2D scaledPoint;
        int newX = static_cast<int>(round(curr.x() + scalingFactor * dx));
        int newY = static_cast<int>(round(curr.y() + scalingFactor * dy));
        
        // Check if new point is out-of-bounds (e.g. scaled up instead)
        if (newX > map_grid_->info.width || newY > map_grid_->info.height || newX < 0 || newY < 0) {
          return curr;
        }

        return {newX, newY};
    }

void RRTPlanner::Chain(Graph& graph, const Point2D& newX, int nearestIndex) {
  graph.addVertex(newX);
  int newIndex = static_cast<int>(graph.getVertexCount()) - 1;
  graph.addEdge(nearestIndex, newIndex);
}


bool RRTPlanner::isEdgeIntersectingObstacle(const Point2D & curr, const Point2D & target)
{
  int x0 = curr.x();
  int y0 = curr.y();
  int x1 = target.x();
  int y1 = target.y();
 
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int x = x0;
  int y = y0;
  int n = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n)
  {
    if (!isPointUnoccupied({x,y})) {return true;}

    if (error > 0)
    {
        x += x_inc;
        error -= dy;
    }
    else
    {
        y += y_inc;
        error += dx;
    }
  }

  return false;
}

// ADDITIONAL UTLITY FUNCTIONS (END SECTION)

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}


inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}

}
// namespace rrt_planner
