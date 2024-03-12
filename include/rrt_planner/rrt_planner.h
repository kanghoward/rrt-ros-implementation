#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

namespace rrt_planner
{

/**
 * A utility class to represent a 2D point
 */
class Point2D
{
public:
  Point2D(): x_(0), y_(0) , parent_index(-1) {}
  Point2D(int x, int y): x_(x), y_(y), parent_index(-1) {}

  int x() const
  {
    return x_;
  }

  int y() const
  {
    return y_;
  }

  int getParent() const
  {
    return parent_index;
  }

  void x(int x)
  {
    x_ = x;
  }

  void y(int y)
  {
    y_ = y;
  }

  // set optional parent index
  void setParent(int index = -1)
  {
    parent_index = index;
  }


private:
  int x_;
  int y_;
  int parent_index;
};


/**
 * Additional utility class to represent the a graph
 */
struct Graph {
private:
    std::vector<Point2D> vertices;
    std::vector<std::pair<int, int>> edges;

    double calculateDistance(const Point2D& point1, const Point2D& point2) const {
        return std::sqrt(std::pow(point1.x() - point2.x(), 2) + std::pow(point1.y() - point2.y(), 2));
    }

public:
    // Public constructor
    Graph() = default;

    // Public method to add a vertex to the graph
    void addVertex(const Point2D& vertex) {
        vertices.push_back(vertex);
    }

    // Public method to read a vertex from the graph
    Point2D getVertex(int index) {
        return vertices[index];
    }

    // Public method to add an edge to the graph
    void addEdge(int vertex1, int vertex2) {
        edges.emplace_back(vertex1, vertex2);
    }

    // Public method to get the number of vertices in the graph
    size_t getVertexCount() const {
        return vertices.size();
    }

    // Public method to get the number of edges in the graph
    size_t getEdgeCount() const {
        return edges.size();
    }

    // Public method to display information about the graph
    void displayGraphInfo() const {
        std::cout << "Number of Vertices: " << getVertexCount() << std::endl;
        std::cout << "Number of Edges: " << getEdgeCount() << std::endl;

        std::cout << "Vertices: ";
        for (const auto& vertex : vertices) {
            std::cout << "(" << vertex.x() << ", " << vertex.y() << ") ";
        }
        std::cout << std::endl;

        std::cout << "Edges: ";
        for (const auto& edge : edges) {
            std::cout << edge.first << " -> " << edge.second << " ";
        }
        std::cout << std::endl;
    }

    int findNearestVertex(const Point2D& target) const {
        int nearestIndex = 0;
        double minDistance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < vertices.size(); ++i) {
            double distance = calculateDistance(target, vertices[i]);

            if (distance < minDistance) {
                minDistance = distance;
                nearestIndex = static_cast<int>(i);
            }
        }
        return nearestIndex;
    }

    std::vector<Point2D> backtrackPath(int goalIndex) const {
        std::vector<Point2D> path;
        int currentIndex = goalIndex;

        // Traverse the tree from the goal to the start
        while (currentIndex != -1) {
            path.push_back(vertices[currentIndex]);
            currentIndex = vertices[currentIndex].getParent();
            // ROS_INFO_STREAM(currentIndex);
        }

        // Reverse the path to get it from start to goal
        std::reverse(path.begin(), path.end());

        return path;
    }

    // Public method to reset the graph
    void reset() {
        vertices.clear();
        edges.clear();
    }
};

/**
 * Main class which implements the RRT algorithm
 */
class RRTPlanner
{
public:
  explicit RRTPlanner(ros::NodeHandle *, int iterations, int step_size, int goal_radius, double resolution);

  ~RRTPlanner() = default;

  




  /**
   * Given a map, the initial pose, and the goal, this function will plan
   * a collision-free path through the map from the initial pose to the goal
   * using the RRT algorithm
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void plan();

  /**
   * Callback for map subscriber
   */
  void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);

  /**
   * Callback for initial pose subscriber
   */
  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);

  /**
   * Callback for goal subscriber
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

private:

  /**
   * Publishes the path calculated by RRT as a nav_msgs::Path msg
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void publishPath();

  /**
   * Utility function to check if a given point is free/occupied in the map
   * @param p: point in the map
   * @return boolean true if point is unoccupied, false if occupied
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  bool isPointUnoccupied(const Point2D & p);

  /**
   * Utility function to build a CV::Mat from a nav_msgs::OccupancyGrid for display
   */
  void buildMapImage();

  /**
   * Utility function to display the CV::Mat map image
   * @param delay
   */
  void displayMapImage(int delay = 1);

  /**
   * Utility function to draw initial pose and goal pose on the map image
   */
  void drawGoalInitPose();

  /**
   * Utility function to draw a circle on the map
   * @param p: center point of the circle
   * @param radius: radius of the circle
   * @param color: color of the circle
   */
  void drawCircle(Point2D & p, int radius, const cv::Scalar & color);

  /**
   * Utility function to draw a line on the map
   * @param p1: starting point of the line
   * @param p2: end point of the line
   * @param color: color of the line
   * @param thickness: thickness of the line
   */
  void drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness = 1);

  /**
   * Utility function to convert a Point2D object to a geometry_msgs::PoseStamped object
   * @return corresponding geometry_msgs::PoseStamped object
   */
  inline geometry_msgs::PoseStamped pointToPose(const Point2D &);

  /**
   * Utility function to convert a geometry_msgs::PoseStamped object to a Point2D object
   */
  inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);

  /**
   * Utility function to convert (x, y) matrix coordinate to corresponding vector coordinate
   */
  inline int toIndex(int, int);


// ADDITIONAL UTLITY FUNCTIONS (BEGIN SECTION)
  /**
   * Utility function to generate a random point within the map
   */
  Point2D RandomPosition();

  /**
   * Utility function to link a new node to the existing RRT Tree
   */
  void Chain(Graph& graph, const Point2D& newX, int nearestIndex);


  /**
   * Utility function to check if an edge intersects any obstacles
   */
  bool isEdgeIntersectingObstacle(const Point2D & p1, const Point2D & p2);

  
  /**
   * Utility function to rescale a point to a step distance
   */
  Point2D rescalePoint(const Point2D& curr, const Point2D& randPoint, int distance);

// ADDITIONAL UTLITY FUNCTIONS (END SECTION)

  ros::NodeHandle * nh_;
  ros::NodeHandle private_nh_;

  bool map_received_;
  std::unique_ptr<cv::Mat> map_;
  nav_msgs::OccupancyGrid::Ptr map_grid_;

  bool init_pose_received_;
  Point2D init_pose_;

  bool goal_received_;
  Point2D goal_;

  Graph rrt_graph;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;

  int iterations_;
  int step_size_;
  int goal_radius_;
  double resolution_;
};

}

#endif  // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
