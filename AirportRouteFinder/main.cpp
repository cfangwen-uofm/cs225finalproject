#include "AirportRouteFinder.h"
#include <vector>
#include "edge.h"
#include <map>
#include <iostream>
#include <fstream>

int main() {
  // Initialize our airport and routes data base
  AirportRouteFinder ap("testdata/routes.txt");

  // Check all destinations with 0 transfer from CMI
  pair<vector<string>, int> v1 = ap.destAfterMutipleTransfer("CMI", 0);
  ap.printBFS1(v1);

  /* COMPARE THE FOLLOWING */
  // Check all destinations with 1 transfer from CMI
  pair<vector<string>, int> v2 = ap.destAfterMutipleTransfer("CMI", 1);
  ap.printBFS1(v2);

  // Check final destinations with 1 transfer from CMI
  pair<vector<string>, int> v3 = ap.finalDestAfterMutipleTransfer("CMI", 1);
  ap.printBFS2(v3);

  /* COMPARE THE FOLLOWING */
  // Finds shortest path from CMI to JFK
  Vertex source = "CMI";
  Vertex dest = "JFK";
  pair<vector<string>, int> r1 = ap.dijkstra(source, dest);
  ap.printPath1(r1);

  // Finds shortest path from CMI to JFK, with ORD forbidden
  vector<string> forbidden;
  forbidden.push_back("ORD");
  pair<vector<string>, int> r2 = ap.aStar(source, dest, forbidden);
  ap.printPath2(r2, forbidden);

  return 0;
}
