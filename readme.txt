====================================================================================================
Discuss the implementation for section 2, 3, and 4 and any improvement that you have implemented.
=====================================================================================================
    Section 2 Implementation
  ------------------------------
    Our environment is 100cm X 100cm in size which is represented by a 10.0 X 10.0 cartesian plane which is specified down to 
    0.1 accuracy (representing 1cm). The program accepts 1 command line parameter which represents the number of obstacles for the
    environment. If obstacle number is less than 30 then the size per obstacle will be randomly chosen between 0.1 - 5.0. Otherwise
    the maximum size per obstacle will be between 0.1 - 2.0. The inital and goal positions will be chosen randomly and made sure to 
    reside within an open node.

    Section 3 Implementation
  ------------------------------
    QuadTree & FBSP
   -----------------
    Since A* is concerned with the free, traversable nodes I decided to add a list object to both the QuadTree and BinarySpace classes.
    This free_node list is initialized as an empty list on instantiation, and is filled with all the free nodes in the domain when the
    Decompose function is called for each class respectively.

    A*
    ----
    The first issue I faced was locating which node the inital/goal rectangle resided within. To do this I searched through the list
    of free nodes that is passed through the A* class parameter. I was confused as to how to implement A* algorithm when dealing 
    with nodes of different sizes/states (obstacle,mixed,free). I decided to only consider the free nodes, and disregard the obstacle 
    and mixed nodes. Once I had this figured out the next issue was finding out which nodes were reachable from the current node. This 
    is simple to figure out when the environment is made up of same sized nodes arranged in a 2d fashion. However, with quadtree & FBSP 
    the free nodes will be many different sizes. I found a simple algorithm called Axis-aligned minimum bounding box which to detects if 
    one rectangle is adjacent to another. I called this algorithm on every free node (stored in a free_node list) in order to determine if 
    it was adjacent with the current node. I used Manhattan Distance to calculate the cost of traversing from the center of node node to another. 
    For an improvement in performance I decided to use a priority queue (min heap) object for the open node list. This allows for quick access 
    to least cost node. Once these issues were dealt with implementing A* was no different than when done with a simple 2d matrix domain.

