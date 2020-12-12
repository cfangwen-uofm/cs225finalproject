# AirportRouteFinder
#### A data structure of Airports and Routes providing path finding and traversal algorithms
Intro Video: https://mediaspace.illinois.edu/media/1_ij9oib2c

## Location of and Using the Code
The code is located in folder **`AirportRouteFinder`**. Everything can be ran inside this folder.   
Test cases are provided under `tests` folder and a demo is provided in `main.cpp`.  
You can run the code in `main.cpp` using `Class AirportRouteFinder`. 
Once the constructor is set, you can use public methods to serve different purposes.  
A default route database is located in `testdata/routes.txt`. Any other route database must follow the format of the data in `testdata/routes.txt`.

## Theory
This project is designed to serve the following purposes: 
1. Build a graph structure having airports as Vertices and routes as Edges
2. Use BFS traversal to find destinations after a number of transfers
3. Find preferred (shortest) route between two recorded airports with possible routes given under certain condition.

### Graph Data Structure
Graph data structure consists of vertices and edges. Here, the airports are vertices and routes are edges.   
This structure provides basis for this project to develop algorithms to create further usage. 

### BFS Traversal
BFS traversal's characteristic is "leafs searched all at the same time." With this characteristic, we can design traversals to find destinations after a number of transfer indicated by user. 

### Preferred (Shortest) Route Algorithms
Here we used Dijkstra and A* Algorithm. Dijkstra can find us the shortest path directly while A* algorithm allows us to find preferred path with restrictions indicated by us. 

## Code Implementation
### AirportRouteFinder(string filename)
Constructor taking in route data. This allows us to input different routes data as routes are constantly changing.

### destAfterMutipleTransfer()
Returns total airports in a vector we can reach after a indicated number of transfer using BFS.

### dijkstra()
Find the shortest path, using dijkstra, between two airports, and get the distance between them. 

### aStar()
Find the preferred path, using A*, between two airports, and get the distance between them with forbidden airports listed. 

### printBFS1()
Prints the result of destAfterMutipleTransfer. Results are printed in `destAfterMutipleTransfer_result.txt` and terminal. 

### printPath1()
Prints the result of path found by dijkstra. Results are printed in `dijkstra_result.txt` and terminal. 

### printPath2()
Prints the result of path found by aStar. Results are printed in `A*_result.txt` and terminal. 
