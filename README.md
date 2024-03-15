# RRT for Path Planning in ROS

## The Task (Successfully Implemented)
Implement the Rapidly Exploring Random Trees (RRT) algorithm to plan collision-free paths in a 2D environment.

## How to run it?
Place it in your catkin workspace `src` folder and build it using:
```bash
catkin build
```

Update the Linux shell about the existence of the built rrt_planner package using:
```bash
source devel/setup.bash
```

You can run the launch file given in the `launch` folder using:
```bash
roslaunch rrt_planner rrt_planner.launch
```

This launch file automatically launches three nodes:
- **RViz** For visualization
- **Map Server** to load a map from a .png file and publish it as a `nav_msgs::OccupancyGrid` on the `/map` topic
- **RRT Planner** to receive a map, initial pose, and goal pose, and calculate and publish a collision-free path as a `nav_msgs::Path` msg

#### Map Server
We have provided 5 example map images in the [resources](resources) directory that can be used to test your RRT implementation.
The map server node is responsible for loading a map file and publishing it as a `nav_msgs::OccupancyGrid` on the `/map` topic.
To select the map file that should be loaded and published, configure the parameters in [cfg/map.yaml](cfg/map.yaml) file.

#### RViz
When a map has been loaded successfully it should be visible in RViz. The user can then set the initial pose and the goal pose through RViz.
Press the `2D Pose Estimate` button in RViz to set the initial pose. Press the `2D Nav Goal` button in RViz to set the goal pose.
Or you can provide the same through the topics `/initialpose` and `/move_base_simple/goal` respectively.

## Tuning
Parameters can be provided to the RRT Planner node using the [cfg/config.yaml](cfg/config.yaml) file.
Certain RRT parameters can be made configurable by adding them to this file.


## Additional stuff (Part of Assignment)

## RRT Algorithm Configuration

This repository contains a configuration file (`config.yaml`) to tune the Rapidly-exploring Random Tree (RRT) algorithm. This file allows you to customize various parameters to tailor the behavior of the RRT algorithm according to your specific needs.

### Parameters

```yaml
iterations: 5000
step_size: 20
goal_radius: 10     tweak this to set the end-search condition

// for RRT*
rrt_star: true     // toggle to enable RRT*, else it is vanilla RRT
neighbourhood_radius: 30    // tweak this to set the RRT* radius


## Additional Utility Functions

This section provides an overview of the utility functions included in this repository. These functions can be used to enhance the functionality of your RRT algorithm implementation.

### RandomPosition

```cpp
/**
 * Utility function to generate a random point within the map
 */
Point2D RandomPosition();

/**
 * Utility function to rewire nodes in the surrounding radius (used in RRT*)
 */
int RRT_STAR(Graph& graph, Point2D& newNode, int neighbourhoodRadius);

/**
 * Utility function to check if an edge intersects any obstacles
 */
bool isEdgeIntersectingObstacle(const Point2D & p1, const Point2D & p2);

/**
 * Utility function to rescale a point to a step distance
 */
Point2D rescalePoint(const Point2D& curr, const Point2D& randPoint, int distance);

/**
 * Utility function to check if a point is within the goal region
 */
void attachGoal(const int index);
