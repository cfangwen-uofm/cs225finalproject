#include "../cs225/catch/catch.hpp"

#include "../graph.h"
#include "../edge.h"
#include "../AirportRouteFinder.h"

# define INF 0xFFFFFF

#include <iostream>


TEST_CASE("AirportRouteFinder() constructor creates a total vertexs of 6072") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<Vertex> vertices = ap.g_.getVertices();
  REQUIRE(vertices.size() == 6072);
}

TEST_CASE("AirportRouteFinder() has setup correct edge weights") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "ORD";
  int edge_weight = ap.g_.getEdgeWeight(source, dest);
  REQUIRE(edge_weight < 225);
  REQUIRE(edge_weight > 210);
}

TEST_CASE("destAfterMutipleTransfer gives correct number of results #1") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<string> v = ap.destAfterMutipleTransfer("CMI", 0);
  REQUIRE(v.size() == 2);
}

TEST_CASE("destAfterMutipleTransfer gives correct number of results #2") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<string> v = ap.destAfterMutipleTransfer("CMI", 1);
  REQUIRE(v.size() > 2);
}

TEST_CASE("finalDestAfterMutipleTransfer gives correct number of results") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<string> v = ap.finalDestAfterMutipleTransfer("CMI", 0);
  REQUIRE(v.size() == 2);
}

TEST_CASE("finalDestAfterMutipleTransfer does not include midpoint airport #1") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<string> v = ap.finalDestAfterMutipleTransfer("CMI", 1);
  REQUIRE(v.size() > 2);
  for (size_t i = 0; i < v.size(); i++) {
    REQUIRE(v[i] != "ORD");
    REQUIRE(v[i] != "DFW");
  }
}

TEST_CASE("finalDestAfterMutipleTransfer does not include midpoint airport #2") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<string> v = ap.finalDestAfterMutipleTransfer("CMI", 2);
  REQUIRE(v.size() > 2);
  for (size_t i = 0; i < v.size(); i++) {
    REQUIRE(v[i] != "ORD");
    REQUIRE(v[i] != "DFW");
    REQUIRE(v[i] != "JFK");
  }
}

TEST_CASE("destAfterMutipleTransfer gives correct number of results #3") {
  AirportRouteFinder ap("testdata/routes.txt");
  vector<string> v1 = ap.destAfterMutipleTransfer("CMI", 3);
  vector<string> v2 = ap.finalDestAfterMutipleTransfer("CMI", 3);
  vector<string> v3 = ap.destAfterMutipleTransfer("CMI", 2);
  REQUIRE(v1.size() - v2.size() == v3.size());
}

TEST_CASE("dijkstra finds a sole path between CMI and ORD") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "ORD";
  int edge_weight = ap.g_.getEdgeWeight(source, dest);
  std::pair<vector<string>, int> r = ap.dijkstra(source, dest);
  REQUIRE(edge_weight == r.second);
  vector<string> v = r.first;
  REQUIRE(v[0] == "CMI");
  REQUIRE(v[1] == "ORD");
}

TEST_CASE("dijkstra finds a path using 1 transfer between CMI and JFK") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "JFK";
  int edge_weight_1 = ap.g_.getEdgeWeight(source, "ORD");
  int edge_weight_2 = ap.g_.getEdgeWeight("ORD", dest);
  std::pair<vector<string>, int> r = ap.dijkstra(source, dest);
  REQUIRE(edge_weight_1 + edge_weight_2 == r.second);
  vector<string> v = r.first;
  REQUIRE(v[0] == "CMI");
  REQUIRE(v[1] == "ORD");
  REQUIRE(v[2] == "JFK");
}

TEST_CASE("dijkstra finds a path using mutiple transfers between CMI and AXT") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "AXT";
  int direct_dist = 9908;
  std::pair<vector<string>, int> r = ap.dijkstra(source, dest);
  REQUIRE(r.second > 9908);
  vector<string> v = r.first;
  REQUIRE(v.size() > 3);
}

TEST_CASE("dijkstra finds a path with infinite distance (unreachable) between CMI and TKF") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "TKF";
  std::pair<vector<string>, int> r = ap.dijkstra(source, dest);
  REQUIRE(r.second == INF);
}

TEST_CASE("dijkstra finds a path of 0 distance between CMI and CMI") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "CMI";
  std::pair<vector<string>, int> r = ap.dijkstra(source, dest);
  REQUIRE(r.second == 0);
}

TEST_CASE("A* finds a path with infinite distance (unreachable) between CMI and TKF with no forbid") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "TKF";
  std::vector<string> forbid;
  std::pair<vector<string>, int> r = ap.aStar(source, dest, forbid);
  REQUIRE(r.second == INF);
}

TEST_CASE("dijkstra finds a path of 0 distance between CMI and CMI with no forbid") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "CMI";
  std::vector<string> forbid;
  std::pair<vector<string>, int> r = ap.aStar(source, dest, forbid);
  REQUIRE(r.second == 0);
}

TEST_CASE("A* finds a path using 1 transfer between CMI and JFK with no forbid") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "JFK";
  int edge_weight_1 = ap.g_.getEdgeWeight(source, "ORD");
  int edge_weight_2 = ap.g_.getEdgeWeight("ORD", dest);
  std::pair<vector<string>, int> r = ap.aStar(source, dest, vector<string>());
  REQUIRE(edge_weight_1 + edge_weight_2 == r.second);
  vector<string> v = r.first;
  REQUIRE(v[0] == "CMI");
  REQUIRE(v[1] == "ORD");
  REQUIRE(v[2] == "JFK");
}

TEST_CASE("A* finds a path with infinite distance (unreachable) between CMI and JFK with ORD DFW forbid") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "JFK";
  std::vector<string> forbid;
  forbid.push_back("DFW");
  forbid.push_back("ORD");
  std::pair<vector<string>, int> r = ap.aStar(source, dest, forbid);
  REQUIRE(r.second == INF);
}

TEST_CASE("A* finds a path with 0 distance (unreachable) between CMI and CMI with ORD DFW forbid") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "CMI";
  std::vector<string> forbid;
  forbid.push_back("DFW");
  forbid.push_back("ORD");
  std::pair<vector<string>, int> r = ap.aStar(source, dest, forbid);
  REQUIRE(r.second == 0);
}

TEST_CASE("A* takes a different a path between CMI and JFK with ORD forbidden") {
  AirportRouteFinder ap("testdata/routes.txt");
  Vertex source = "CMI";
  Vertex dest = "JFK";
  std::vector<string> forbid;
  forbid.push_back("ORD");
  std::pair<vector<string>, int> r1 = ap.dijkstra(source, dest);
  std::pair<vector<string>, int> r2 = ap.aStar(source, dest, forbid);
  REQUIRE(r2.second != r1.second);
  vector<string> v = r2.first;
  for (size_t i = 0; i < v.size(); i++) {
    REQUIRE(v[i] != "ORD");
  }
}