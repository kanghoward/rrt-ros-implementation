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




# Assignment Deliverables

## Visualization

### Images

![Vanilla RRT](visualizations/rrt.jpg) ![RRT*](visualizations/rrtstar.jpg)

### Videos

[![Vanilla RRT](visualizations/rrt.jpg)](visualizations/rrt.mp4) [![RRT*](visualizations/rrtstar.jpg)](visualizations/rrtstar.mp4)



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

## Main Functions (Assignment Requirements)

- `void publishPath()`: 
    -> Publishes the path calculated by RRT as a nav_msgs::Path message. The candidate is required to implement the logic in this function.

- `bool isPointUnoccupied(const Point2D & p)`: 
    -> Utility function to check if a given point is free or occupied in the map. 
    - `p`: Point in the map.
    - Returns a boolean value indicating true if the point is unoccupied and false if occupied. The candidate is required to implement the logic in this function.

- `void plan()`: 
    -> Given a map, the initial pose, and the goal, this function will plan a collision-free path through the map from the initial pose to the goal using the RRT or RRT* algorithm based on configuration in [cfg/config.yaml]



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


## RRT & RRT* Theory

#### RRT Pseudocode

This section presents the pseudocode for the RRT (Rapidly-Exploring Random Tree) algorithm, outlining the steps to plan a collision-free path through the map from the initial pose to the goal pose. 

```cpp
Main RRT (Rapidly-Exploring Random Tree) algorithm
RRT Pseudocode
Qgoal //region that identifies success
Counter = 0 //keeps track of iterations
lim = n //number of iterations algorithm should run for
G(V,E) //Graph containing edges and vertices, initialized as empty
While counter < lim:
    Xnew  = RandomPosition()
    if IsInObstacle(Xnew) == True:
        continue
    Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
    Link = Chain(Xnew,Xnearest)
    G.append(Link)
    if Xnew in Qgoal:
        Return G
```

#### RRT* Pseudocode

This pseudocode outlines the steps for implementing the RRT* (Rapidly-Exploring Random Tree Star) algorithm, which is an enhanced version of RRT that optimizes the tree structure to produce more efficient paths.

```cpp
Main RRT* (Rapidly-Exploring Random Tree Star) algorithm
RRT* Pseudocode
Qgoal //region that identifies success
Counter = 0 //keeps track of iterations
lim = n //number of iterations algorithm should run for
G(V,E) //Graph containing edges and vertices, initialized as empty
While counter < lim:
    Xnew  = RandomPosition()
    if IsInObstacle(Xnew) == True:
        continue
    Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
    Xnew = Steer(Xnearest, Xnew) // steer towards Xnew within a certain radius
    if IsInObstacle(Xnew) == True:
        continue
    Xnearests = Near(G(V,E),Xnew, neighbourhood_radius) // find nearby vertices within a radius
    Xnearest = ChooseParent(G(V,E), Xnew, Xnearests) // choose the best parent among the nearby vertices
    Link = Chain(Xnew,Xnearest)
    G.append(Link)
    Rewire(G(V,E), Xnew, Xnearests) // rewire the tree considering new node Xnew
    if Xnew in Qgoal:
        Return G
```

## RRT* Algorithm Optimization

In the context of the RRT* algorithm, optimization strategies are crucial for enhancing efficiency and performance. Here's how various optimization techniques benefit the algorithm:

- **Continuous Distance Updates**:
  - Saving distances as deltas allows for efficient updates in the RRT* algorithm. By storing incremental changes in distances, the algorithm can swiftly adjust and optimize paths without the need to recalculate distances from scratch.

- **Redrawing Efficiency**:
  - Redrawing the entire map for each new node addition may initially seem inefficient. However, this approach offers optimization benefits in terms of map rendering. By redrawing only the necessary elements, such as newly added nodes and edges, the algorithm minimizes computational overhead while maintaining an updated and visually informative map representation.

- **Deleting Old Edges**:
  - Instead of directly deleting old edges, the algorithm repaints them white for optimization purposes. This optimization choice avoids the computational burden of explicitly removing old edges and allows the algorithm to focus on updating and rendering new map elements efficiently.

Overall, these optimization strategies prioritize efficiency and performance in the RRT* algorithm, enabling smoother and faster path planning in dynamic environments.




## OpenCV Map Styling

In an OpenCV application, managing the styling of the map is crucial for clarity and visual appeal. Here's how different styling choices are implemented:

- `Red for New Points and Edges`: 
    -> New points and edges added to the map are highlighted in red. This color choice helps draw attention to newly added elements, making them easily distinguishable from existing ones.

- `Green for Redrawn Edges (RRT*)`: 
    -> When redrawing edges, particularly in the context of the RRT* algorithm, green is used. This color signifies redrawn edges, indicating modifications or updates made to the map structure. 

- `Blue for Final Path Taken`: 
    -> The final path taken by the algorithm is represented in blue. This color choice emphasizes the path taken, providing clarity on the trajectory followed from the initial pose to the goal pose.

- `Concentric Circle Displaying the Goal Radius`: 
    -> To visualize the goal radius, concentric circles are displayed on the map. These circles help depict the area around the goal pose that defines the success region for the algorithm.

By employing distinct colors for different elements of the map, such as new points and edges, redrawn edges, final paths, and goal radius, the map remains informative and visually appealing. Users can easily interpret the map and understand the results of the algorithm's computations.








