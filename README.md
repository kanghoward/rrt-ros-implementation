# RRT for Path Planning in ROS

## The Task (Successfully Implemented)
Implement the Rapidly Exploring Random Trees (RRT) algorithm to plan collision-free paths in a 2D environment.

## Instructions to Run

Create an empty folder and initialize the catkin workspace using:
```bash
mkdir catkin_ws/src -p
cd catkin_ws/src
catkin_init_workspace
```

Clone the repo and place it in your catkin workspace `src` folder and build it using:
```bash
git clone https://github.com/kanghoweee/rrt-ros-implementation.git
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




## Additional stuff (Part of Assignment)

## Tuning
Parameters can be provided to the RRT Planner node using the [cfg/config.yaml](cfg/config.yaml) file.
Certain RRT parameters can be made configurable by adding them to this file.

## RRT Algorithm Configuration

This repository contains a configuration file (`config.yaml`) to tune the Rapidly-exploring Random Tree (RRT) algorithm. This file allows you to customize various parameters to tailor the behavior of the RRT algorithm according to your specific needs. The map configuration file (`config.yaml`) is used to specify the resolution and the map image used for the RRT algorithm. 

#### config.yaml


- `iterations: 5000`
    -> Sets the maximum number of iterations for the RRT algorithm.

- `step_size: 20`
    -> Specifies the distance the algorithm will attempt to extend the tree in each iteration.

- `goal_radius: 10`
    -> Tweak this parameter to set the end-search condition.

- `rrt_star: true`
    -> Toggle to enable RRT* algorithm. If disabled, the algorithm operates as vanilla RRT.

- `neighbourhood_radius: 30`
    -> Tweak this parameter to set the radius for RRT* algorithm.

#### map.yaml
- `image: ../resources/extra4.png`
    -> Specifies the image file path for the map.

- `resolution: 0.1`
    -> Sets the resolution of the map.

- `origin: [0.0, 0.0, 0.0]`
    -> Sets the origin of the map.

- `occupied_thresh: 0.65`
    -> Specifies the threshold value for considering a cell occupied.

- `free_thresh: 0.196`
    -> Specifies the threshold value for considering a cell free.

- `negate: 1`
    -> Set negate to zero if black walls on white background, set to one if white walls on black background.




## Graph and Point2D Utility Classes

This section introduces the Graph and Point2D utility classes, which are essential components of the RRT algorithm implementation. These classes provide functionalities for managing vertices, edges, and calculations related to points in the graph.

#### Point2D Class

The Point2D class represents a two-dimensional point with optional parent index and distance to parent attributes. It offers the following methods:

- `Point2D()`: 
    -> Default constructor.

- `Point2D(int x, int y)`: 
    -> Constructor with coordinates.

- `Point2D(int x, int y, int index, double relativeDistance)`: 
    -> Constructor with coordinates, parent index, and distance to parent.

- `x()`, `y()`: 
    -> Accessor methods for coordinates.

- `getParent()`: 
    -> Accessor method for the parent index.

- `getDistanceToParent()`: 
    -> Accessor method for the distance to the parent.

- `setParent(int index = -1)`: 
    -> Setter method for the parent index.

- `setDistanceToParent(double distance = 0)`: 
    -> Setter method for the distance to the parent.


#### Graph Class

The Graph class represents a graph structure composed of vertices and edges. It provides methods for managing vertices, edges, calculating distances, and finding nearest vertices. The key methods include:

- `addVertex(const Point2D& vertex)`: 
-> Adds a vertex to the graph.
    
- `getVertex(int index)`: 
    -> Retrieves a vertex from the graph.
    
- `setVertexParent(int curr_index, int parent_index)`: 
    -> Sets the parent index for a vertex.
    
- `addEdge(int vertex1, int vertex2)`: 
    -> Adds an edge between two vertices.
    
- `getVertexCount()`, `getEdgeCount()`: 
    -> Returns the number of vertices and edges.
    
- `calculateEuclideanDistance(const Point2D& point1, const Point2D& point2)`: 
    -> Calculates the Euclidean distance between two points.
    
- `findNearestVertex(const Point2D& target)`: 
    -> Finds the index of the nearest vertex to a given point.
    
- `backtrackPath(int goalIndex)`: 
    -> Backtracks the path from a goal vertex to the start.
    
- `getTotalPathDistance(const Point2D& point)`: 
    -> Calculates the total path distance from a given point to the start.
    
- `getNeighbours(const Point2D& newNode, int neighbourhoodRadius)`: 
    -> Retrieves the indices of neighbors within a given radius.



## Additional Utility Functions

This section provides an overview of the utility functions included in this repository. These functions can be used to enhance the functionality of your RRT algorithm implementation.

- `Point2D RandomPosition()`: 
    -> Utility function to generate a random point within the map.

- `int RRT_STAR(Graph& graph, Point2D& newNode, int neighbourhoodRadius)`: 
    -> Utility function to rewire nodes in the surrounding radius (used in RRT*).

- `bool isEdgeIntersectingObstacle(const Point2D & p1, const Point2D & p2)`: 
    -> Utility function to check if an edge intersects any obstacles.

- `Point2D rescalePoint(const Point2D& curr, const Point2D& randPoint, int distance)`: 
    -> Utility function to rescale a point to a step distance.

- `void attachGoal(const int index)`: 
    -> Utility function to check if a point is within the goal region.



