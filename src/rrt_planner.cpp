#include "rrt_planner/rrt_planner.h"
#include <cmath>
#include <stdexcept>
#include <float.h>
namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node, int iterations, int step_size, int goal_radius, double resolution, int neighbourhood_radius, bool rrt_star)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false),
  iterations_(iterations),
  step_size_(step_size),
  goal_radius_(goal_radius),
  resolution_(resolution),
  neighbourhood_radius_(neighbourhood_radius),
  rrt_star_(rrt_star)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");

  // CHECK IF PARAMS ARE RECEIVED PROPERLY
  ROS_INFO_STREAM("Iterations: " << iterations);
  ROS_INFO_STREAM("Step Size: " << step_size);
  ROS_INFO_STREAM("Goal Radius: " << goal_radius);
  ROS_INFO_STREAM("Resolution: " << resolution);
  ROS_INFO_STREAM("DBL_MAX: " << DBL_MAX);

  if (rrt_star) {
    ROS_INFO_STREAM("RRT* enabled");
    ROS_INFO_STREAM("neighbourhood_radius: " << neighbourhood_radius);
  } else {
    ROS_INFO_STREAM("Normal RRT enabled");
  }


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
      ros::Duration(0.1).sleep(); //default value is 0.1, change back later
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

  // max iteration limit & iteration counter
  int counter = 0;

  ROS_INFO_STREAM(iterations_);
  ROS_INFO_STREAM(step_size_);
  ROS_INFO_STREAM(goal_radius_);
  ROS_INFO_STREAM(resolution_);

  // init graph with initial position vertex
  rrt_graph.reset();
  ROS_INFO("Init RRT Graph.");
  rrt_graph.addVertex(init_pose_);

  ROS_INFO_STREAM(init_pose_.getDistanceToParent());

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

    // displays random point that was sampled
    // drawCircle(xNew, 2, cv::Scalar(0, 255, 0));
    if (rrt_star_) {
      // the edge intersecting obstacle needs to be done for every point in the neighbourhood
      int goalParentIndex = RRT_STAR(rrt_graph, xNew, neighbourhood_radius_);
      if (goalParentIndex >= 0) {
        attachGoal(goalParentIndex);
        ROS_INFO("A path has been found.");
        publishPath();
        return;
      }
    } else {
      // the edge intersecting obstacle needs to be done only for the nearest point
      if (isEdgeIntersectingObstacle(xNearest,xNew)) {continue;}
      // if all tests pass, create new edge and add to RRT graph
      xNew.setParent(xNearestIndex);
      xNew.setDistanceToParent(rrt_graph.calculateEuclideanDistance(xNew, xNearest));
      rrt_graph.addVertex(xNew);
      drawCircle(xNew, 2, cv::Scalar(0, 0, 255));
      drawLine(xNew, xNearest, cv::Scalar (0, 0, 255), 1);
      // ROS_INFO_STREAM("New X: " << xNew.x());
      if (rrt_graph.calculateEuclideanDistance(goal_, xNew) < goal_radius_) {
        attachGoal(rrt_graph.getVertexCount() - 1);
        ROS_INFO("A path has been found.");
        publishPath();
        return;
      } 
    }
    
    displayMapImage();
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
  int count = 0;
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

    ROS_INFO_STREAM("Total Distance is : " << rrt_graph.getTotalPathDistance(point));
    ROS_INFO_STREAM("Rel Distance to Parent is : " << point.getDistanceToParent());
    ROS_INFO_STREAM("Parent Node is : " << point.getParent());
    ROS_INFO_STREAM("index from goal is : " << count);
    ROS_INFO_STREAM("-------------------------------------------");
    drawCircle(point, 2, cv::Scalar(255, 0, 0));
    count++;
  }
  // Publish the calculated path

  
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

void RRTPlanner::drawCircle(const Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(const Point2D & p1, const Point2D & p2, const cv::Scalar & color, int thickness)
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

void RRTPlanner::attachGoal(const int index) {
  // add one final edge to the goal node
    Point2D xNew = rrt_graph.getVertex(index);
    goal_.setParent(index);
    rrt_graph.addVertex(xNew);
    drawCircle(xNew, 2, cv::Scalar(0, 0, 255));
    drawLine(goal_, xNew, cv::Scalar (0, 0, 255), 1);
}
  



int RRTPlanner::RRT_STAR(Graph& graph, Point2D& newNode, int neighbourhoodRadius)  {
  std::vector<int> neighboursIndex = graph.getNeighbours(newNode, neighbourhoodRadius);
  std::vector<double> totalDistances;
  std::vector<double> relativeDistances; // implicitly stores intersecting edges with distance=-1
  // ROS_INFO_STREAM("neighbours index: " << std::to_string(neighboursIndex));

  // ADDING VERTEX TO MIN PATH LENGTH, NOT NEAREST NODE
  int minIndex = -1;
  double minTotalDistanceToNodeFromNeighbour = DBL_MAX;
  for (size_t i = 0; i < neighboursIndex.size(); ++i) {
    Point2D neighbour = graph.getVertex(neighboursIndex[i]);

    if (!isEdgeIntersectingObstacle(newNode, neighbour)) {
      double relativeDistance = graph.calculateEuclideanDistance(newNode, neighbour);
      relativeDistances.push_back(relativeDistance);
      double totalDistanceToNodeFromNeighbour = graph.getTotalPathDistance(neighbour) + relativeDistance;
      // ROS_INFO_STREAM("current neighbour: " << neighboursIndex[i]);
      // ROS_INFO_STREAM("distanceToNodeFromNeighbour: " << distanceToNodeFromNeighbour);
      if (totalDistanceToNodeFromNeighbour < minTotalDistanceToNodeFromNeighbour) {
          minTotalDistanceToNodeFromNeighbour = totalDistanceToNodeFromNeighbour;
          minIndex = neighboursIndex[i];
      }
    } else {
      relativeDistances.push_back(-1);
    }
  }

  ROS_INFO_STREAM("Minindex: " << minIndex);

  // rrt* needs distances to continuously update lol
  // SAVE DISTANCES AS DELTAS, MORE EFFICIENT TO UPDATE
  // RETRIEVAL CAN BE DONE BY BACKTRACKING

  if (minIndex == -1) {
    return -1; // stops if no possible node to connect to
  }  
  
  Point2D shortestNode = graph.getVertex(minIndex);
  newNode.setParent(minIndex);
  newNode.setDistanceToParent(graph.calculateEuclideanDistance(newNode, shortestNode));
  graph.addVertex(newNode);
  drawCircle(newNode, 2, cv::Scalar(0, 0, 255));
  drawLine(newNode, shortestNode, cv::Scalar (0, 0, 255), 1);
  ROS_INFO_STREAM("last vertex: " << graph.getVertexCount());
  
  if (graph.calculateEuclideanDistance(goal_, newNode) < goal_radius_) {
    return graph.getVertexCount() - 1; // stops if the new node is already inside the goal radius
  }


  for (size_t i = 0; i < relativeDistances.size(); ++i) {
    ROS_INFO_STREAM("relativeDistance: " << relativeDistances[i]);
  }


double distanceToNewNode = graph.getTotalPathDistance(newNode);

  // REWIRE SURROUNDING VERTICES only if new point added
  for (size_t i = 0; i < neighboursIndex.size(); ++i) { 
    // run only for non-intersecting potential edges
    if (relativeDistances[i] > 0) {
      Point2D neighbour = graph.getVertex(neighboursIndex[i]);
      double totalDistanceToNeighbourFromNode = distanceToNewNode + relativeDistances[i];
      // ROS_INFO_STREAM("current neighbour: " << neighboursIndex[i]);
      // ROS_INFO_STREAM("distanceToNodeFromNeighbour: " << distanceToNodeFromNeighbour);

      if ( totalDistanceToNeighbourFromNode < graph.getTotalPathDistance(neighbour)) {

          ROS_INFO_STREAM("-----------------------------------------");
          ROS_INFO_STREAM("old parent: " << graph.getVertex(neighboursIndex[i]).getParent());
          ROS_INFO_STREAM("old total distance: " << graph.getTotalPathDistance(neighbour));
          ROS_INFO_STREAM("dl: " << totalDistanceToNeighbourFromNode - graph.getTotalPathDistance(neighbour));


          graph.getVertex(neighboursIndex[i]).setDistanceToParent(relativeDistances[i]); 
          // update all distances of downstream nodes by dl (TO BE IMPLEMENTED IF TIME PERMITS)
          // graph.updateDistances(dl);
          
          // update opencv map styling 
          // (delete old line connection)
          // Point2D neighbourOldParent = graph.getVertex(neighbour.getParent());

          // impractical to "delete" old edges (repaint old edges white), hence might need to redraw entire map every time a new node is added. It's doable, but it is a matter of styling
          // drawLine(neighbourOldParent, neighbour, cv::Scalar (255, 255, 255), 2);

          // delete existing edge in graph (TO BE IMPLEMENTED IF TIME PERMITS)
          // graph.deleteEdge(neighbourOldParent, neighbour);

          // draw new line connection (green is used to highlight the differences btw old lines and new lines)
          drawLine(newNode, neighbour, cv::Scalar (0, 255, 0), 1);
          
          // update parent (last node added is the newNode) # NEED TO UPDATE IN GRAPH DATA STRUCTURE
          graph.setVertexParent(neighboursIndex[i], graph.getVertexCount() - 1);
          // neighbour.setParent(graph.getVertexCount() - 1); 
          ROS_INFO_STREAM("new parent: " << graph.getVertex(neighboursIndex[i]).getParent());          
          ROS_INFO_STREAM("new total distance: " << graph.getTotalPathDistance(neighbour));
          ROS_INFO_STREAM("new dist to parent: " << graph.getVertex(neighboursIndex[i]).getDistanceToParent());          
      }
    } 
  }

  return -1;
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
