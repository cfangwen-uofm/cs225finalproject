/**
 * @file NimLearner.cpp
 * CS 225: Data Structures
 */

#include "NimLearner.h"
#include <ctime>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <queue>
//#include <sstring>
typedef pair<int, string> iPair; 
# define INF 0xFFFFFF
using namespace std;
//used for read routes only
NimLearner::NimLearner(string filename) : g_(true,true){
  airportDataLoader("testdata/airports.txt");
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

  // Reading Airports and Routes into Graph
  for(unsigned j = 0; j < airline.size(); j++){
    if(!g_.vertexExists(source[j]))
      g_.insertVertex(source[j]);
    if(!g_.vertexExists(dest[j]))
      g_.insertVertex(dest[j]);
    g_.insertEdge(source[j], dest[j]);
    int distance = (int) calculateGreatCircle(airports[source[j]].first, airports[source[j]].second, airports[dest[j]].first, airports[dest[j]].second);
    g_.setEdgeWeight(source[j], dest[j], distance);
    g_.setEdgeLabel(source[j], dest[j], airline[j]);
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
 * Returns a constant reference to the state space graph.
 *
 * @returns A constant reference to the state space graph.
 */
const Graph & NimLearner::getGraph() const {
  return g_;
}

/* This is a great circle distance calculator using haversine formula */
double NimLearner::calculateGreatCircle(double lat1, double long1, double lat2, double long2) {
  double radius = 6371;
  double pi = atan(1) * 4;
  double phi1 = lat1 * M_PI/180;
  double phi2 = lat2 * M_PI/180;
  double del_phi = (lat2-lat1) * M_PI/180;
  double del_lambda = (long2-long1) * M_PI/180;

  double a = sin(del_phi/2) * sin(del_phi/2) + cos(phi1) * cos(phi2) * sin(del_lambda/2) * sin(del_lambda/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = radius * c;

  return d;
}

/** A function that loads airport information given and provide lat-long data into a dict */
void NimLearner::airportDataLoader(string filename) {
  std::ifstream infile(filename);
  //std::cout<<filename<<std::endl;
  string line;
  string temp;
  string temp_ap;
  double temp_lat;
  double temp_long;
  int count;

  while (std::getline(infile, line))
  {
    //std::cout<<line<<std::endl;
    count = 0;
    temp_ap.clear();
    temp_lat = 0;
    temp_long = 0;
    
    for(unsigned i = 0; i < line.length(); i++){
      //before first , IATA
      //before second, lat
      //end of line, long
      if(line.at(i) == ',' || i == line.length()-1){
        if(i == line.length() - 1){
          temp = temp + line.at(i);
          temp_long = stod(temp);
          temp.clear();
        }
        else if(count == 0){
          temp_ap = temp;
          count++;
          temp.clear();
        }
        else if(count == 1){
          temp_lat = stod(temp);
          count++;
          temp.clear();
        }
      }
      if(line.at(i) != ',' && i != line.length() - 1)
        temp = temp + line.at(i);
    }
    if (!temp_ap.empty()) {
      std::pair<double, double> latlong(temp_lat, temp_long);
      airports.insert(std::pair<string, std::pair<double, double>>(temp_ap, latlong));
    }
  }
}
void NimLearner::shortestpath(string src) {
  priority_queue< iPair, vector <iPair> , std::greater<iPair> > pq; 
  int V = source.size();
  vector<int> dist(V, INF); 
  // pair <int, string> tp = make_pair(0, src);
  pq.push(make_pair(0, src)); 
  for(unsigned long i = 0; i < source.size(); i++) {
    if (source[i] == src) {
      dist[i] = 0;
    }
  }
  // dist[src] = 0;

  while (!pq.empty()) 
    { 
        // The first vertex in pair is the minimum distance 
        // vertex, extract it from priority queue. 
        // vertex label is stored in second of pair (it 
        // has to be done this way to keep the vertices 
        // sorted distance (distance must be first item 
        // in pair) 
        string u = pq.top().second; 
        pq.pop(); 
  
        // 'i' is used to get all adjacent vertices of a vertex 
        // list< pair<int, string> >::iterator i; 
        for (Vertex ver : g_.getAdjacent(u)) 
        { 
            // Get vertex label and weight of current adjacent 
            // of u. 
            string v = ver;
            int weight = g_.getEdgeWeight(u,v);
            //  If there is shorted path to v through u. 
            unsigned long j = 0;
            for (j = 0; j <source.size(); j++) {
              if (source[j] == v) {
                break;
              }
            }
            unsigned long k = 0;
            for (k = 0; k<source.size(); k++) {
              if (source[k] == u){
                break;
              }
            }
            if (dist[j] > dist[k] + weight) 
            { 
                // Uupdating distance of v 
                dist[j] = dist[k] + weight; 
                pq.push(make_pair(dist[j], v)); 
            } 
        } 
    }
        printf("Vertex   Distance from Source\n"); 
    for (int i = 0; i < V; ++i) 
        printf("%d \t\t %d\n", i, dist[i]);   
}