/**
 * @file NimLearner.cpp
 * CS 225: Data Structures
 */

#include "NimLearner.h"
#include <ctime>
#include <fstream>
//#include <sstring>


/**
 * Constructor to create a game of Nim with `startingTokens` starting tokens.
 *
 * This function creates a graph, `g_` representing all of the states of a
 * game of Nim with vertex labels "p#-X", where:
 * - # is the current player's turn; p1 for Player 1, p2 for Player2
 * - X is the tokens remaining at the start of a player's turn
 *
 * For example:
 *   "p1-4" is Player 1's turn with four (4) tokens remaining
 *   "p2-8" is Player 2's turn with eight (8) tokens remaining
 *
 * All legal moves between states are created as edges with initial weights
 * of 0.
 *
 * @param startingTokens The number of starting tokens in the game of Nim.
 */
NimLearner::NimLearner(unsigned startingTokens) : g_(true, true) {
    /* Your code goes here! */
    //define starting vertex 
    startingVertex_ = "p1-"+to_string(startingTokens);
    //create all the states using edges and vertex
    //vertex
    for(unsigned i = 0; i <= startingTokens; i++){
      g_.insertVertex("p1-"+to_string(i));
      g_.insertVertex("p2-"+to_string(i));
    }
    //edges
    //p1-4 points to p2-3 and p2-2
    //p1-1 points to p2-0
    //p1-0 points to nothing
    //
    for(unsigned j = 0; j <= startingTokens; j++){
      if(j == 0){
        //do nothing
      }
      else if(j == 1){
        g_.insertEdge("p1-"+to_string(j),"p2-"+to_string(j-1));
        g_.setEdgeWeight("p1-"+to_string(j),"p2-"+to_string(j-1), 0);
        g_.insertEdge("p2-"+to_string(j),"p1-"+to_string(j-1));
        g_.setEdgeWeight("p2-"+to_string(j),"p1-"+to_string(j-1), 0);
      }
      else{
        g_.insertEdge("p1-"+to_string(j),"p2-"+to_string(j-1));
        g_.setEdgeWeight("p1-"+to_string(j),"p2-"+to_string(j-1), 0);
        g_.insertEdge("p1-"+to_string(j),"p2-"+to_string(j-2));
        g_.setEdgeWeight("p1-"+to_string(j),"p2-"+to_string(j-2), 0);
        g_.insertEdge("p2-"+to_string(j),"p1-"+to_string(j-1));
        g_.setEdgeWeight("p2-"+to_string(j),"p1-"+to_string(j-1), 0);
        g_.insertEdge("p2-"+to_string(j),"p1-"+to_string(j-2));
        g_.setEdgeWeight("p2-"+to_string(j),"p1-"+to_string(j-2), 0);
      }
    }
}


//used for read routes only
NimLearner::NimLearner(string filename) : g_(true,true){
  std::ifstream infile(filename);
  //std::cout<<filename<<std::endl;
  string line;
  string temp;
  int count;
  while (std::getline(infile, line))
  {
    //std::cout<<line<<std::endl;
    count = 0;
    for(unsigned i = 0; i < line.length(); i++){
      //before first , airline
      //before second , airline id
      //before third , source airport
      //forth source id
      //fifth dest airport
      //sixth dest id
      //seventh codeshare
      //eighth stops
      //to the end of line equipments
      //std::cout<<temp<<std::endl;
      if(line.at(i) == ',' || i == line.length()-1){
        if(i == line.length() - 1){
          temp = temp + line.at(i);
          equip.push_back(temp);
          temp.clear();
        }
        else if(count == 0){
          airline.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 1){
          airlineID.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 2){
          source.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 3){
          sourceID.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 4){
          dest.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 5){
          destID.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 6){
          if(temp.length() == 0){
            temp = "N";
          }
          codeshare.push_back(temp);
          count++;
          temp.clear();
        }
        else if(count == 7){
          stops.push_back(temp);
          count++;
          temp.clear();
        }
      }
      if(line.at(i) != ',' && i != line.length() - 1)
        temp = temp + line.at(i);
    }
  }
  //finish reading file 
  for(unsigned j = 0; j < airline.size(); j++){
    if(!g_.vertexExists(source[j]))
      g_.insertVertex(source[j]);
    if(!g_.vertexExists(dest[j]))
      g_.insertVertex(dest[j]);
    g_.insertEdge(source[j], dest[j]);
    g_.setEdgeWeight(source[j], dest[j], 0);
  }
  //for test uses
  /*bool ex;
  for(unsigned k = 0; k < equip.size(); k++){
    ex = false;
    //std::cout<<equip[k]<<std::endl;
    for(unsigned n = 0; n < diff_airline.size(); n++){
      if(equip[k] == diff_airline[n]){
        ex = true;
      }
    }
    if(ex == false){
      diff_airline.push_back(equip[k]);
      std::cout<<equip[k]<<std::endl;
    }
  }
  std::cout<<diff_airline.size()<<std::endl;*/
}

/**
 * Plays a random game of Nim, returning the path through the state graph
 * as a vector of `Edge` classes.  The `origin` of the first `Edge` must be
 * the vertex with the label "p1-#", where # is the number of starting
 * tokens.  (For example, in a 10 token game, result[0].origin must be the
 * vertex "p1-10".)
 *
 * @returns A random path through the state space graph.
 */
std::vector<Edge> NimLearner::playRandomGame() const {
  vector<Edge> path;
 /* Your code goes here! */
  //plays a random game
  int r;
  Vertex currVertex = startingVertex_;

  while(1){
    if((g_.getAdjacent(currVertex)).size() == 0){
      return path;
    }
    r = rand()%2;
    if((g_.getAdjacent(currVertex)).size() == 1){
      r = 0;
    }
    path.push_back(g_.getEdge(currVertex, (g_.getAdjacent(currVertex))[r]));
    currVertex = (g_.getAdjacent(currVertex))[r];
  }
  return path;
}

/*
 * Updates the edge weights on the graph based on a path through the state
 * tree.
 *
 * If the `path` has Player 1 winning (eg: the last vertex in the path goes
 * to Player 2 with no tokens remaining, or "p2-0", meaning that Player 1
 * took the last token), then all choices made by Player 1 (edges where
 * Player 1 is the source vertex) are rewarded by increasing the edge weight
 * by 1 and all choices made by Player 2 are punished by changing the edge
 * weight by -1.
 *
 * Likewise, if the `path` has Player 2 winning, Player 2 choices are
 * rewarded and Player 1 choices are punished.
 *
 * @param path A path through the a game of Nim to learn.
 */
void NimLearner::updateEdgeWeights(const std::vector<Edge> & path) {
 /* Your code goes here! */
 int weight;
 if((path[path.size()-1].dest) == "p2-0"){//p1 wins
   for(unsigned i = 0; i < path.size(); i++){
     if(i%2 == 0){
        //choice made by p1
        weight = g_.getEdgeWeight(path[i].source, path[i].dest);
        g_.setEdgeWeight(path[i].source, path[i].dest, weight+1);
     }
     if(i%2 == 1){
        //choice made by p2
        weight = g_.getEdgeWeight(path[i].source, path[i].dest);
        g_.setEdgeWeight(path[i].source, path[i].dest, weight-1);
     }
   }
 }
 if((path[path.size()-1].dest) == "p1-0"){//p2 wins
   for(unsigned i = 0; i < path.size(); i++){
     if(i%2 == 0){
        //choice made by p1
        weight = g_.getEdgeWeight(path[i].source, path[i].dest);
        g_.setEdgeWeight(path[i].source, path[i].dest, weight-1);
     }
     if(i%2 == 1){
        //choice made by p2
        weight = g_.getEdgeWeight(path[i].source, path[i].dest);
        g_.setEdgeWeight(path[i].source, path[i].dest, weight+1);
     }
   }
 }
}

/**
 * Label the edges as "WIN" or "LOSE" based on a threshold.
 */
void NimLearner::labelEdgesFromThreshold(int threshold) {
  for (const Vertex & v : g_.getVertices()) {
    for (const Vertex & w : g_.getAdjacent(v)) {
      int weight = g_.getEdgeWeight(v, w);

      // Label all edges with positve weights as "WINPATH"
      if (weight > threshold)           { g_.setEdgeLabel(v, w, "WIN"); }
      else if (weight < -1 * threshold) { g_.setEdgeLabel(v, w, "LOSE"); }
    }
  }
}

/**
 * Returns a constant reference to the state space graph.
 *
 * @returns A constant reference to the state space graph.
 */
const Graph & NimLearner::getGraph() const {
  return g_;
}
