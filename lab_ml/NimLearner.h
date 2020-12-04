/**
 * @file NimLearner.cpp
 * CS 225: Data Structures
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
class NimLearner {
public:
  NimLearner(string filename);
   /**
   * Returns a constant reference to the state space graph.
   *
   * @returns A constant reference to the state space graph.
   */
  const Graph & getGraph() const;

  double calculateGreatCircle(double lat1, double long1, double lat2, double long2);

  void airportDataLoader(string filename);

  std::map<string, std::pair<double, double>> airports;
  //dijkstra algorithm
  vector<int> shortestpath(string src);
  //把source的每个点map到一个int
  std::map<int, string> makeMap(vector<string> src);
  //为了runtime建的inverse map
  std::map<string, int> inverse_map(std::map<int,string> &oriMap);
  //挪了位置
  std::vector<string> source;


private:
  Graph g_;
  std::vector<string> airline;
  std::vector<string> airlineID;
  std::vector<string> sourceID;
  std::vector<string> dest;
  std::vector<string> destID;
  std::vector<string> codeshare;
  std::vector<string> stops;
  std::vector<string> equip;
  std::vector<string> diff_airline;
  
};
