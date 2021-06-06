
//
//  shortestpath.hpp
//
//
//  Created by Sonal Sannigrahi on 01/05/2021.
/*
Useful links for the data:
http://snap.stanford.edu/data/index.html
http://www.crawdad.org
https://gis.stackexchange.com/questions/104452/building-a-graph-from-osm

delta-stepping
https://old.insight-centre.org/sites/default/files/publications/engineering_a_parallel_-stepping_algorithm.pdf
*/
#ifndef shortestpath_hpp
#define shortestpath_hpp
#include <map>
#include <stdio.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <stack>
#include <iostream>
#include <sys/time.h>
#include <set>
#include <limits.h>
#include <stdio.h>

#define INF -1
typedef unsigned int Edge;
typedef std::vector<int> Path;


class Node{
    public:
        int index;//index of the node
        //int dist;//minimal distance between the source node and the node, can change
        //int status;
        std::map<Node,int> AdjNodes;
        
        Node(int x){
            this->index=x;
            //this->status=status;
        }
    
        void add_edge(Node vertex, int weight){
            if(*find(this->AdjNodes.begin(), this->AdjNodes.end(), vertex) != vertex){
                this->AdjNodes.insert(std::make_pair(vertex, weight));
            }
            else if(this->AdjNodes[vertex] > weight){
                this->AdjNodes[vertex] = weight;
            }
        }
        
        //Node parentNode; one of the IN nodes...?
        //int status;//0=unvisited, 1=tempo labeled 2=perma labeled
        //std::vector<std::pair<Node,Edge> > AdjNodes; //OUT nodes, not IN, vectors of pair (destination node, weight)
};
class Graph{
    public:
        int num_nodes;
        std::vector<Node> Nodes;
        std::map<std::pair<int, int>,int> Edges;
        
        Graph(int node_number) {
            this->num_nodes = node_number;
            for(int i=0; i <= this->num_nodes; i++){
                this->Nodes.push_back(Node(i));
            }
        };
    
        void add_edge(int u, int v, int w);
        int get_edge_data(int u, int v);
        std::vector<int> get_neighbours(int vertex);
};

void Graph::add_edge(int u, int v, int w){
    this->Nodes[u].add_edge(this->Nodes[v], w);
    std::map<std::pair<int, int>,int>::iterator result;
    result = this->Edges.find(std::make_pair(u,v));
    if(result != this->Edges.end()){
        int weight = this->Edges[std::make_pair(u,v)];
        if(weight < w){
            weight = w;
        }
    }
    else{
        this->Edges.insert({std::make_pair(u,v), w});
    }
}

int Graph::get_edge_data(int u, int v){
    return this->Edges[std::make_pair(u,v)];
}

std::vector<int> Graph::get_neighbours(int vertex){
    std::vector<int> result;
    std::map<Node,int>::iterator iter = this->Nodes[vertex].AdjNodes.begin();
    std::map<Node,int>::iterator endIter = this->Nodes[vertex].AdjNodes.end();
    for(; iter != endIter; ++iter){
        result.push_back(iter->first.index);
    }
    return result;
}


    
#endif /* shortestpath_hpp */
