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
#include <stack>
typedef pair<int, string> iPair; 
# define INF 0xFFFFFF
using namespace std;

/**
 * Initializes a graph with airports in database and connect them with routes given.
 */
NimLearner::NimLearner(string filename) : g_(true,true){
  // Load Airport data into graph, a dictionary, and a list
  airportDataLoader("testdata/airports.txt");

  // Load routes into edges between vertexs
  std::ifstream infile(filename);
  // Initialize temporary varibles for recording data
  string line;
  string temp;
  int count;
  while (std::getline(infile, line))
  {
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
    if(g_.vertexExists(source[j]) && g_.vertexExists(dest[j])) {
      int distance = (int) calculateGreatCircle(airports.at(source[j]).first, airports.at(source[j]).second, airports.at(dest[j]).first, airports.at(dest[j]).second);
      g_.insertEdge(source[j], dest[j]);
      g_.setEdgeWeight(source[j], dest[j], distance);
      g_.setEdgeLabel(source[j], dest[j], airline[j]);
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


/**
 * Returns total airports in a vector we can reach after a indicated number of transfer.
 * 
 * If we have already reached the maximum destinations we can reach before the indicated number,
 * we return all possible destinations
 * 
 * This functions uses BFS traversal
 *
 * @returns total airports in a vector we can reach after a indicated number of transfer.
 */
vector<string> NimLearner::destAfterMutipleTransfer (string initial_ap, int num_of_times) {
  queue<string> q;
  map<string, int> visited;
  vector<string> toReturn;
  q.push(initial_ap);
  visited.insert(std::pair<string, int>(initial_ap, 1));

  int count = 0;

  while (!q.empty() && (count - 1) != num_of_times) {
    count++;
    string current_ap = q.front();
    q.pop();
    vector<string> neighbors = g_.getAdjacent(current_ap);
    for (size_t i = 0; i < neighbors.size(); i++) {
      if (visited[neighbors[i]] != 1) {
        visited[neighbors[i]] = 1;
        q.push(neighbors[i]);
        toReturn.push_back(neighbors[i]);
      }
    }
  }

  return toReturn;
}


/**
 * Returns the final airports (excluding the midpoint airports) in a vector we can reach after a indicated number of transfer.
 * 
 * If we have already reached the maximum destinations we can reach before the indicated number,
 * we return all possible destinations
 * 
 * This functions uses BFS traversal
 *
 * @returns total final airports (excluding the midpoint airports) in a vector we can reach after a indicated number of transfer.
 */
vector<string> NimLearner::finalDestAfterMutipleTransfer (string initial_ap, int num_of_times) {
  queue<string> q;
  map<string, int> visited;
  vector<string> toReturn;
  q.push(initial_ap);
  visited.insert(std::pair<string, int>(initial_ap, 1));

  int count = 0;

  while (!q.empty()) {
    count++;
    string current_ap = q.front();
    q.pop();
    vector<string> neighbors = g_.getAdjacent(current_ap);
    for (size_t i = 0; i < neighbors.size(); i++) {
      if (visited[neighbors[i]] != 1) {
        visited[neighbors[i]] = 1;
        q.push(neighbors[i]);
        if (count - 1 == num_of_times) {
          toReturn.push_back(neighbors[i]);
        }
      }
    }
    if (count - 1 == num_of_times) {
      break;
    }
  }

  return toReturn;
}

/**
 * Returns a distance between two locations (in Lat Long) on the earth using Haversine formula.
 *
 * @returns A distance between two locations on the earth.
 */
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

/**
 * Creates a dictionary, vertexs in graph, and a list of airport locations (in Lat and Long) from data.
 * 
 * Helper function for class constructor
 */
void NimLearner::airportDataLoader(string filename) {
  std::ifstream infile(filename);
  string line;
  string temp;
  string temp_ap;
  double temp_lat;
  double temp_long;
  int count;

  while (std::getline(infile, line))
  {
    // Initialize temporary varibles for recording data
    count = 0;
    temp_ap.clear();
    temp_lat = 0;
    temp_long = 0;
    
    for (unsigned i = 0; i < line.length(); i++){
      //before first , IATA Airport Code
      //before second, lat
      //end of line, long
      if (line.at(i) == ',' || i == line.length()-1) {
        if (i == line.length() - 1) {
          temp = temp + line.at(i);
          temp_long = stod(temp);
          temp.clear();
        }
        else if (count == 0) {
          temp_ap = temp;
          count++;
          temp.clear();
        }
        else if (count == 1) {
          temp_lat = stod(temp);
          count++;
          temp.clear();
        }
      }
      if (line.at(i) != ',' && i != line.length() - 1) {
        temp = temp + line.at(i);
      }
    }
    if (!temp_ap.empty()) {
      // Setup latlong variable describing latitude and longitude
      std::pair<double, double> latlong(temp_lat, temp_long);
      // Insert into Dictionary
      airports.insert(std::pair<string, std::pair<double, double>>(temp_ap, latlong));
      // Insert into graph
      g_.insertVertex(temp_ap);
      // Insert into a vector
      airportLocation.push_back(temp_ap);
    }
  }
}

//helper map function
std::map<int, string> NimLearner::makeMap(vector<string> src) {
  std::map<int, string> sourceMap;
  for (size_t i = 0; i < src.size(); i++) {
    sourceMap[i] = src[i];
  }
  return sourceMap;
}

//inverse map
std::map<string, int> NimLearner::inverse_map(std::map<int,string> &oriMap){
  std::map<string, int> invMap;
  std::for_each(oriMap.begin(), oriMap.end(), [&invMap] (const std::pair<int, string> &p)
                {
                    invMap.insert(std::make_pair(p.second, p.first));
                });
    return invMap;
}

/*
Find the shortest path between two airports, and get the distance between them
parameter: src -> starting point
parameter: dest-> destination
return: a string that records the completet path from the start to the destination and the distance of this path
*/
string NimLearner::dijkstra(string src, string dest){
  //initialize a minimal heap
  priority_queue< iPair, vector <iPair> , std::greater<iPair> > pq; 

  //size of vertices are the number of airports
  int V = airportLocation.size();

  //dist stores the distance from the src to the current airport
  vector<int> dist(V, INF); 

  //prevStop stores the previous airport in the shortest path
  vector<Vertex> prevStop(V, "");  

  // T stores the shortest path, it is weighted
  Graph T(true, true);

  //create a map to get faster query speed
  std::map<int, string> airportmap = makeMap(airportLocation);  
  //use inverse map to get the index given the airport
  std::map<string, int> revAirportMap = inverse_map(airportmap);

  //find the map pair of the source
  auto startIdx = revAirportMap.find(src);
  //find the map pair of the destination
  auto destIdx = revAirportMap.find(dest);

  //make sure the input is valid
  if (startIdx == revAirportMap.end() || destIdx == revAirportMap.end()) {
    return "Invalid Input";
  }

  //push the source to the heap
  pq.push(make_pair(0, src)); 
  dist[startIdx->second] = 0;

  //loop until the heap is empty
  while (!pq.empty()) 
    { 
      // The first vertex in pair is the minimum distance 
      // vertex, extract it from priority queue. 
      // vertex label is stored in second of pair (it 
      // has to be done this way to keep the vertices 
      // sorted distance (distance must be first item 
      // in pair) 
      string u = pq.top().second; 
      // T.insertVertex(u);
      //pop the vertex with the minimal distance
      pq.pop(); 
      
      //find the map pair of the current vertex
      auto uIdx = revAirportMap.find(u);

      //search every adjacent vertices for a shortest path
      for (Vertex ver : g_.getAdjacent(u)) 
      { 
        //make sure there's no duplicate vertices in the graph
          // Get vertex label and weight of current adjacent of u 
          string v = ver;
          //insert the vertex
          // T.insertVertex(v);
          //get weight from current vertex to the adjacent vertex 
          int weight = g_.getEdgeWeight(u,v);


          // find if there is shorter path to v through u. 
          auto vIdx = revAirportMap.find(v);
          //if the distance is shorter
          if (dist[vIdx->second] > dist[uIdx->second] + weight) 
          { 
              // Updating distance of v 
              dist[vIdx->second] = dist[uIdx->second] + weight; 
              pq.push(make_pair(dist[vIdx->second], v));
              //store the previous vertex of the shortest path to the prevStop vector 
              prevStop[vIdx->second] = u;
          } 
      }
    }
    //initialize the return string
    string ret = "";

    //if there's no path between src and dest
    if (dist[destIdx->second] == INF) {
      return "no path exists";
    }

    /*
    Find the path from dest to src(since only prev exists), and add the vertex along the path to a stack;
    Specifically we start from adding the previous vertex of dest, then add previous vertex of that and so on until 
    we reached the beginning of the path (the 2nd vertext which would add the src vertetx to the stack);
    Previous of dest will be at the bottom of the stack, src will be at the top; dest itself will be left out for now.
    */
    string previousVer = dest;
    stack<string> routes;
    while (previousVer != src) {
      auto previousVerIdx = revAirportMap.find(previousVer);
      previousVer = prevStop[previousVerIdx->second];
      routes.push(previousVer);
    }

    //add vertices from the stack to ret so now the ret will have proper order (src->dest)
    while(!routes.empty()) {
      ret.append(routes.top());
      ret.append(" -> ");
      routes.pop();
    }
    //since we did not add in dest before now we add dest at the end 
    ret.append(dest);

    //add the distance between src and dest to return value
    ret.append(" distance is ");
    ret.append(to_string(dist[destIdx->second]));
    ret.append(" km");
    return ret;
}

//Astar algorithm
string NimLearner::aStar(string src, string dest, vector<string> forbidden){
  //initialize a minimal heap
  priority_queue< iPair, vector <iPair> , std::greater<iPair> > pq; 
  //size of vertices are the number of airports
  int V = airportLocation.size();

  //dist stores the distance from the src to the current airport
  vector<int> dist(V, INF); 

  //prevStop stores the previous airport in the shortest path
  vector<Vertex> prevStop(V, "");  
  //keep track of visited location
  vector<string> visited;
  
  //create a map to get faster query speed
  std::map<int, string> airportmap = makeMap(airportLocation);  
  //use inverse map to get the index given the airport
  std::map<string, int> revAirportMap = inverse_map(airportmap);

  //find the map pair of the source
  auto startIdx = revAirportMap.find(src);
  //find the map pair of the destination
  auto destIdx = revAirportMap.find(dest);

  //make sure the input is valid
  if (startIdx == revAirportMap.end() || destIdx == revAirportMap.end()) {
    return "Invalid Input";
  }

  //initialize the heuristic value for all the vertices
  vector<double> hvalue;
  //create a map with airport name and h value 
  std::map<string, double> H;
  //get the latitude and longitude of the destination by -> and .
  auto destlatlong = airports.find(dest);
  double destlat = destlatlong->second.first;
  double destlong = destlatlong->second.second;

  //find all the heuristic value
  for (auto airport: airportLocation ) {
    auto curlatlong = airports.find(airport);
    double curlat = curlatlong->second.first;
    double curlong = curlatlong->second.second;
    double hairport = calculateGreatCircle(curlat, curlong, destlat, destlong);
    hvalue.push_back(hairport);
  }

  //make the forbidden location's h value to INF
  for (auto cantGo: forbidden) {
    auto cantGoIdx = revAirportMap.find(cantGo);
    hvalue[cantGoIdx->second] = INF;
  }

  //create a map (airport, h value)
  for (size_t i = 0; i < airports.size(); i++) {
    H.insert(make_pair(airportLocation[i], hvalue[i]));
  }
  //push the source to the heap 
  double srcH = H.find(src)->second;
  pq.push(make_pair(srcH, src)); 
  dist[startIdx->second] = 0;

    //loop until the heap is empty
  while (!pq.empty()) 
    { 
      // The first vertex in pair is the minimum distance 
      // vertex, extract it from priority queue. 
      // vertex label is stored in second of pair (it 
      // has to be done this way to keep the vertices 
      // sorted distance (distance must be first item 
      // in pair) 
      string u = pq.top().second; 
      if(u == dest) {
        break;
      }
      // T.insertVertex(u);
      //pop the vertex with the minimal distance
      pq.pop(); 
      visited.push_back(u);

      //find the map pair of the current vertex
      auto uIdx = revAirportMap.find(u)->second;

      //search every adjacent vertices for a shortest path
      for (Vertex v : g_.getAdjacent(u)) 
      { 
        //make sure there's no duplicate vertices in the graph
        auto it = find(visited.begin(), visited.end(), v);
        if (it == visited.end()) {
          // Get vertex label and weight of current adjacent of u 
          //get weight from current vertex to the adjacent vertex 
          int weight = g_.getEdgeWeight(u,v);

          // find if there is shorter path to v through u. 
          auto vIdx = revAirportMap.find(v)->second;
          //if the distance is shorter
          double vAscore = dist[uIdx] + weight + hvalue[vIdx];
          if (dist[vIdx] + hvalue[vIdx] > vAscore) 
          { 
              // Updating distance of v 
              dist[vIdx] = dist[uIdx] + weight; 
              if(dist[vIdx] >= INF) {
                return "No path exists";
              }
              pq.push(make_pair(vAscore, v));
              //store the previous vertex of the shortest path to the prevStop vector 
              prevStop[vIdx] = u;
          } 
        }
      }
    }
      //initialize the return string
    string ret = "";

    //if there's no path between src and dest
    if (dist[destIdx->second] >= INF) {
      return "no path exists";
    }

    /*
    Find the path from dest to src(since only prev exists), and add the vertex along the path to a stack;
    Specifically we start from adding the previous vertex of dest, then add previous vertex of that and so on until 
    we reached the beginning of the path (the 2nd vertext which would add the src vertetx to the stack);
    Previous of dest will be at the bottom of the stack, src will be at the top; dest itself will be left out for now.
    */
    string previousVer = dest;
    stack<string> routes;
    while (previousVer != src) {
      auto previousVerIdx = revAirportMap.find(previousVer)->second;
      previousVer = prevStop[previousVerIdx];
      if(hvalue[previousVerIdx] == INF){
        return "No path exists";
      }
      routes.push(previousVer);
    }

    //add vertices from the stack to ret so now the ret will have proper order (src->dest)
    while(!routes.empty()) {
      ret.append(routes.top());
      ret.append(" -> ");
      routes.pop();
    }
    //since we did not add in dest before now we add dest at the end 
    ret.append(dest);

    //add the distance between src and dest to return value
    ret.append(" distance is ");
    ret.append(to_string(dist[destIdx->second]));
    ret.append(" km");
    return ret;
}
