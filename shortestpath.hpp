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
#include <set>
#include <limits.h>
#include <algorithm>
#include <iostream>

#define INF -1
typedef unsigned int Edge;
typedef std::vector<int> Path;


class Node {
    public:
        Node(int index){
            this->index=index;
            this->dist=INF;
            this->status=0;
        }
        Node(){
            this->dist=INF;
            this->status=0;
        }
        int index;//index of the node
        int dist;//minimal distance between the source node and the node, can change
        std::vector<std::pair<Node*,Edge> > ParNodes; //one of the IN nodes...?
        int status;//0=unvisited, 1=tempo labeled 2=perma labeled
        int GraphIndex;
        std::vector<std::pair<Node*,Edge> > AdjNodes; //OUT nodes, not IN, vectors of pair (destination node, weight)
        void add_AdjNodes(std::pair<Node*,Edge> p1){
            this->AdjNodes.push_back(p1);
        };
        void add_ParNodes(std::pair<Node*,Edge> p1){
            this->ParNodes.push_back(p1);
        };
};
class Graph{
    public:
        Graph(){}
        void add_edges(int i, int j, Edge w){
            for (int k=0;k<this->Nodes.size();k++){
                if(i==this->Nodes[k]->index){
                    for (int s=0; s<this->Nodes.size();s++){
                        if(j==this->Nodes[s]->index){
                            std::pair<Node*,Edge> p1(this->Nodes[s],w);
                            std::pair<Node*,Edge> p2(this->Nodes[k],w);
                            this->Nodes[k]->add_AdjNodes(p1);
                            this->Nodes[s]->add_ParNodes(p2);
                            this->Nodes[s]->add_AdjNodes(p2);//for undirected graphs
                            this->Nodes[k]->add_ParNodes(p1);//for undirected graphs
                            this->nbe+=1;
                        }
                    }
                }
            }
        };
        void add_nodes(int index,bool s){
            Node* node= new Node(index);
            if(s){
                this->s=node;
                this->s->dist=0;
                this->s->status=2;
                this->Nodes.push_back(this->s);
            }
            else{
                this->Nodes.push_back(node);
            }
        }
        void add_nodes (Node* node, bool s){
            if (s){
                this->s=node;
                this->Nodes.push_back(this->s);
            }
            else { 
                this->Nodes.push_back(node);
            }
        }
        std::vector<Node*> Nodes;
        std::set<int> AdjGraphs;
        int index;
        int nbe;
        Node* s;//source node
    };
        
#endif /* shortestpath_hpp */
