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
#include <algorithm>
#include <iostream>

#define INF 10000
typedef unsigned int Edge;
typedef std::vector<int> Path;


class Node {
    public:
        //Node(int index, std::vector<std::pair<Node,Edge> > AdjNodes,int dist=INF,int status=0){
        //    this->index=index;
        //    this->AdjNodes=AdjNodes;
        //    this->dist=dist;
        //    this->status=status;
        //}
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
        //Node parentNode; one of the IN nodes...?
        int status;//0=unvisited, 1=tempo labeled 2=perma labeled
        std::vector<std::pair<Node*,Edge> > AdjNodes; //OUT nodes, not IN, vectors of pair (destination node, weight)
        void add_AdjNodes(std::pair<Node*,Edge> p1){
            this->AdjNodes.push_back(p1);
        };
};
class Graph{
    public:
        //Graph(std::vector<Node> Nodes, std::vector<std::vector<std::pair<Node,Edge> > > Edges, Node s) {
        //    this->Edges=Edges;
        //    this->Nodes=Nodes;
        //    this->s=s;
        //    for (int i=0;i<Nodes.size();i++){
        //        this->Nodes[i].AdjNodes=Edges[i];
        //    }
        //}
        Graph(){}
        void add_edges(int i, int j, Edge w){
            for (int k=0;k<this->Nodes.size();k++){
                if(i==this->Nodes[k]->index){
                    for (int s=0; s<this->Nodes.size();s++){
                        if(j==this->Nodes[s]->index){
                            std::pair<Node*,Edge> p1(this->Nodes[s],w);
                            this->Nodes[k]->add_AdjNodes(p1);
                        }
                    }
                }
            }
        };
        void add_nodes(std::vector<Node*> Nodes,int s){
            for (int i=0;i<Nodes.size();i++){
                if(i==s){
                    this->s=Nodes[i];
                    this->s->dist=0;
                    this->s->status=2;
                    this->Nodes.push_back(this->s);
                }
                else {
                    this->Nodes.push_back(Nodes[i]);
                }
            }
        }
        std::vector<Node*> Nodes;
        std::vector<std::vector<std::pair<Node*,Edge> > > Edges;
        Node* s;//source node
        //std::vector<std::vector<std::pair<Node*,Edge> > > get_edges(){
        //    return this->Edges;
        //}
        //std::vector<Node*> get_Nodes(){
        //    return this->Nodes;
        //}
        //std::vector<Node> Neighbours(Node node){
        //    std::vector<Node> list;
        //    for(int i=0; i<node.AdjNodes.size();i++){
        //        list.push_back(node.AdjNodes[i].first);
        //    }
        //    return list;
        //}
    };
        
#endif /* shortestpath_hpp */
