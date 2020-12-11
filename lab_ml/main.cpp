#include "NimLearner.h"
#include <vector>
#include "edge.h"
#include <map>
#include <iostream>
#include <fstream>

int main() {

  /*NimLearner nim("地面.txt");
  NimLearner nim1("墙面上.txt");
  NimLearner nim2("墙面右上.txt");
  NimLearner nim3("墙面左上.txt");
  NimLearner nim4("墙面下.txt");
  NimLearner nim5("墙面右下.txt");
  NimLearner nim6("墙面左下.txt");
  NimLearner nim7("墙面左.txt");
  NimLearner nim8("墙面右.txt");*/
  // NimLearner nim("testdata/test-1.txt");
  // NimLearner nim("testdata/test-2-dup.txt");
  NimLearner nim("testdata/routes.txt");

  vector<string> list = nim.destAfterMutipleTransfer("CMI", 2);
  vector<string> list2 = nim.finalDestAfterMutipleTransfer("CMI", 2);
  for (size_t i = 0; i < list.size(); i++) {
    std::cout << list[i] << std::endl;
  }
  std::cout << "Finished list 1" << std::endl;
  for (size_t i = 0; i < list2.size(); i++) {
    std::cout << list2[i] << std::endl;
  }
  // NimLearner nim("testdata/routes-copy.txt");
  

  // Play 10,000 games of Nim:
  //const int gamesPlayed = 10000;
  //for (int i = 0; i < gamesPlayed; i++) {
  //  vector<Edge> path = nim.playRandomGame();
  //  nim.updateEdgeWeights(path);
  //}
  //nim.labelEdgesFromThreshold(gamesPlayed / 50);

  // Print textual output of the graph:
  // nim.getGraph().print();

  // Save an graph PNG:
  //std::cout<<"# of airports " << nim.g_.getVertices().size() <<std::endl;
  //std::cout<<"# of airports " <<nim.airports.size() <<std::endl;
  //std::cout<<"# of airports " <<nim.airportLocation.size() <<std::endl;
  //nim.getGraph().savePNG("Out");
  
  // vector<int> output = nim.shortestpath("AER");
  std::ofstream myfile;
  std::ofstream myfile1;

  // std::cout<<"# of lines " <<output.size() <<std::endl;
  // myfile.open ("dist-test.txt");
  // for(size_t i = 0; i < output.size(); i++) {
  //   myfile  <<i<<"the distance to " <<nim.airportLocation[i].c_str()<<" is "<< output[i]<<endl;
  // }
  // myfile.close();
  vector<string>forbiden;
  forbiden.push_back("UUU");
  // forbiden.push_back("DFW");
  std::cout<<nim.aStar("PEK","AER",forbiden)<<std::endl;

  // std::cout<<nim.dijkstra("CMI","ABD")<<std::endl;


  return 0;
}
