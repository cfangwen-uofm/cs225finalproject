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
 * Represents a algorithm to learn the game of Nim.
 */
class AirportRouteFinder {
public:
  AirportRouteFinder(string filename);
   /**
   * Returns a constant reference to the state space graph.
   *
   * @returns A constant reference to the state space graph.
   */
  const Graph & getGraph() const;

  void airportDataLoader(string filename);

  vector<string> destAfterMutipleTransfer (string initial_ap, int num_of_times);

  vector<string> finalDestAfterMutipleTransfer (string initial_ap, int num_of_times);

  Graph g_;

  //dijkstra algorithm scratch
  // vector<int> shortestpath(string src);
  //print shortestpath
  pair<vector<string>, int> dijkstra(string start, string dest);
  //把source的每个点map到一个int
  std::map<int, string> makeMap(vector<string> src);
  //为了runtime建的inverse map
  std::map<string, int> inverse_map(std::map<int,string> &oriMap);
  pair<vector<string>, int> aStar(string src, string dest, vector<string> forbidden);

private:
  // A dictionary storing airport lat long information
  std::map<string, std::pair<double, double>> airports;

  // Helper function for finding distance between two airports
  double calculateGreatCircle(double lat1, double long1, double lat2, double long2);

  //all the location of the airport
  std::vector<string> forbidden;
  std::vector<string> airportLocation;

};
