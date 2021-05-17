# Path Planning Project

# Introduction
Path Planning is a vital attribute in many applications of engineering. It can allow us to determine the shortest path (also known as optimal path) between two points, or multiple points if need be. Path planning is used in many industries such as automation industry for autonomous robotics, healthcare industry for surgical equipment, gaming industry for video games, and much more. The purpose of this project is to understand the two most commonly known industry path planning algorithms and create a simple user interface game that will display both algorithms in action. The two algorithms that will be created are the Dijkstras Path Planning Algorithm and the A* Path Planning Algorithm. The pygame library was imported which was used to create the game board for the user, as well as update the colors of each square to display the start node, end node, barrier nodes, highlighted optimal path, and the nodes checked by the selected algorithm.  

# Dijkstras Path Planning Algorithm
The Dijkstras Algorithm (also known as the Uniform-Cost Algorithm) is a popular path planning algorithm when actions (or in our case nodes/squares) have different costs, but also since it is complete and produces a cost-optimal solution. To put it simply, the cost of traveling from one node to the next contains a specific cost or weight, and these costs get added when determining the optimal path. The backbone of the algorithm consists of performing a best-first-search call to determine the lowest cost node and updating/analyzing the path cost for the current neighbouring nodes. The program was initialized by creating a 'distance' array which stored the cost of traveling to each node, as well as a 'visited' array to keep track of which nodes have been visited. Each node was initialized to have a cost of infinity in the 'distance' array except for the starting node which had a cost of 0, as well as all nodes were initialized to be False in the 'visited' array excpet for the starting node which was set to True. From there, the algorithm was restrained to only check the top, bottom, right, and left nodes of the current node being analyzed and determine the cost of the neighbouring nodes and store them in the 'distance' array. If the neighbour node cost was greater than the current node cost plus 1, then the new neighbour cost was updated and the node was marked as visited. The program would update all neighbour node costs, then search for the lowest node code that has not been visited. From there the program would repeat the steps described until the end node is reached, where the program would back track and display the shortest path.  

# A* Algorithm
The A* Algorithm is one of the most popular informed search algorithms (ie uses domain-specific hints about the location of the goals). It utilized a twp part evaluation function as shown below:
- f(n) = g(n) + h(n)

Where g(n) is the path cost from the current node to the next, and h(n), also known as the heurestic function, is the estimated cost of the shortest path from the current node to the final end node. G(n) is calculated the same way as the Dijkstras algorithm, however the heuristic function is determined base on the specific problem. In this situation since the program is only eligable to travel up/down/right/left it is most appropriate to use the Manhattan Distance as the heuristic function. The Manhattan distance is calculated as the sum of the absolute differences between the two vectors (ie in our case the current node and the end node position on the grid). The program now consists of creating and initializing a g_dist array to hold the g(n) costs, but also a f_dist array to hold the total cost of each node which is g(n) + heuristic(current node, end node). The program also utilizes a priority queue named 'open_set' which will store all total costs of each node visited. This eases the workload in determining which new node to visit next which is automatically determined by the priority queue functionality by setting the lowest cost node to be highest priority. In addition a data structure 'came_from' was created which stores the parent node of the current node being analyzed. This is used for back tracking the end node to the start node for the most optimal path. Same as the Dijkstras algorithm, the A* star Algorithm is complete and cost-optimal if the correct heurestic function is selected. 

# Instructions
1. Run the program
2. Left click on a square to set as the start node. The color of the square will turn blue
3. Left click on a square to set as the end node. The color of the square will turn red. 
4. Left click on a square(s) to create barriers for the algorithm to avoid. The color of the square(s) will trun black. IMPORTANT: ensure that there is an eligable path between the start node and the end node. If the barriers selected do not ensure this then the both algorithms will not work. 
5. Press the 'a' key to run the A* Algorithm or press the 'd' to run the Dijkstras Algorithm.
6. Watch as the selected algorithm executes. The squares that turn yellow indicate that the algorithm has analyzed the cost of that node.
7. Once the algorithm is finished, the optimal path will be displays in turqoise color.
8. Exit the program.

# Example Results
Dijkstras Algorithm: 

A* Algorithm:
