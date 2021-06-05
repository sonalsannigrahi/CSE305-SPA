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


class Node {
    public:
        Node(int index, std::vector<std::pair<Node,Edge> > AdjNodes,int dist=INF,int status=0){
            this->index=index;
            this->AdjNodes=AdjNodes;
            this->dist=dist;
            this->status=status;
        }
        Node(){
            this->dist=INF;
            this->status=0;
        }
        int index;//index of the node
        int dist;//minimal distance between the source node and the node, can change
        //Node parentNode; one of the IN nodes...?
        int status;//0=unvisited, 1=tempo labeled 2=perma labeled
        std::vector<std::pair<Node,Edge> > AdjNodes; //OUT nodes, not IN, vectors of pair (destination node, weight)
};
class Graph{
    public:
        explicit Graph(std::vector<Node> Nodes, std::vector<std::vector<std::pair<Node,Edge> > > Edges, Node s) {
            for (int i=0;i<Nodes.size();i++){
                Nodes[i].AdjNodes=Edges[i];
            }
            this->Edges=Edges;
            this->Nodes=Nodes;
            this->s=s;
        };
        std::vector<Node> Nodes;
        std::vector<std::vector<std::pair<Node,Edge> > > Edges;
        Node s;//source node
        std::vector<std::vector<std::pair<Node,Edge> > > get_edges(){
            return this->Edges;
        }
        std::vector<Node> get_Nodes(){
            return this->Nodes;
        }
        std::vector<Node> Neighbours(Node node){
            std::vector<Node> list;
            for(int i=0; i<node.AdjNodes.size();i++){
                list.push_back(node.AdjNodes[i].first);
            }
            return list;
        }
    };
        
#endif /* shortestpath_hpp */
