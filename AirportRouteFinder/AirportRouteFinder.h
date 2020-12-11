/**
 * @file AirportRouteFinder.h
 */

#pragma once

#include <vector>
#include <algorithm>
#include <string>
#include <map>

#include "graph.h"
#include "edge.h"

/**
 * A class for storing airports and routes, while providing destination traversal functions and pathfinding algorithms. 
 */
class AirportRouteFinder {
public:
  // Variables
  /* Graph for airport and routes */
  Graph g_;

  // Constructor
  /* Constructor. Takes route data as input */
  AirportRouteFinder(string filename);

  // Class main methods
  /* Returns a constant reference to the state space graph. */
  const Graph & getGraph() const;

  /* Returns total airports in a vector we can reach after a indicated number of transfer using BFS. */
  pair<vector<string>, int> destAfterMutipleTransfer (string initial_ap, int num_of_times);

  /* Returns the final airports (excluding the midpoint airports) in a vector we can reach after a indicated number of transfer using BFS. */
  pair<vector<string>, int> finalDestAfterMutipleTransfer (string initial_ap, int num_of_times);

  /* Find the shortest path between two airports, and get the distance between them */
  pair<vector<string>, int> dijkstra(string start, string dest);
  
  /* Find the shortest path between two airports, and get the distance between them with forbidden airports listed */
  pair<vector<string>, int> aStar(string src, string dest, vector<string> & forbidden);

  /* Prints the result of destAfterMutipleTransfer */
  void printBFS1(pair<vector<string>, int> & aps);

  /* Prints the result of finalDestAfterMutipleTransfer */
  void printBFS2(pair<vector<string>, int> & aps);

  /* Prints the result of path found by dijkstra */
  void printPath1(pair<vector<string>, int> & pathdata);

  /* Prints the result of path found by aStar */
  void printPath2(pair<vector<string>, int> & pathdata, vector<string> & forbidden);

private:
  // Helper Variables
  /* A dictionary storing airport lat long information */
  std::map<string, std::pair<double, double>> airports;

  /* A list of all airports codes */
  std::vector<string> airportLocation;

  // Helper functions
  /* Helper function for finding distance between two airports */
  double calculateGreatCircle(double lat1, double long1, double lat2, double long2);

  /* Helper function for loading airport data into the graph, a map, and a list */
  void airportDataLoader(string filename);

  /* Creates a map mapping airport to int. */
  std::map<int, string> makeMap(vector<string> src);

  /* Creates an inverse map for runtime purposes. */
  std::map<string, int> inverse_map(std::map<int,string> &oriMap);
};
