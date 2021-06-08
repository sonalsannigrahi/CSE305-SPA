//
//  shortestpath.cpp
//  
//
//  Created by Sonal Sannigrahi on 01/05/2021.
//

#include "shortestpath.hpp"

//First implementing Djikstra in parallel

//sequential djikstra for comparison

#include <limits.h>
#include <stdio.h>
#include <queue>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <functional>
#include <utility>
// Number of vertices in the graph
#define V 9
  
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
//int minDistance(int dist[], bool sptSet[])
//{
//    // Initialize min value
//    int min = INT_MAX, min_index;
//  
//    for (int v = 0; v < V; v++)
//        if (sptSet[v] == false && dist[v] <= min)
//            min = dist[v], min_index = v;
//  
//    return min_index;
//}
//  
//// A utility function to print the constructed distance array
//void printSolution(int dist[])
//{
//    printf("Vertex \t\t Distance from Source\n");
//    for (int i = 0; i < V; i++)
//        printf("%d \t\t %d\n", i, dist[i]);
//}
//  
//// Function that implements Dijkstra's single source shortest path algorithm
//// for a graph represented using adjacency matrix representation
//void dijkstra(int graph[V][V], int src)
//{
//    int dist[V]; // The output array.  dist[i] will hold the shortest
//    // distance from src to i
//  
//    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest
//    // path tree or shortest distance from src to i is finalized
//  
//    // Initialize all distances as INFINITE and stpSet[] as false
//    for (int i = 0; i < V; i++)
//        dist[i] = INT_MAX, sptSet[i] = false;
//  
//    // Distance of source vertex from itself is always 0
//    dist[src] = 0;
//  
//    // Find shortest path for all vertices
//    for (int count = 0; count < V - 1; count++) {
//        // Pick the minimum distance vertex from the set of vertices not
//        // yet processed. u is always equal to src in the first iteration.
//        int u = minDistance(dist, sptSet);
//  
//        // Mark the picked vertex as processed
//        sptSet[u] = true;
//  
//        // Update dist value of the adjacent vertices of the picked vertex.
//        for (int v = 0; v < V; v++)
//  
//            // Update dist[v] only if is not in sptSet, there is an edge from
//            // u to v, and total weight of path from src to  v through u is
//            // smaller than current value of dist[v]
//            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
//                && dist[u] + graph[u][v] < dist[v])
//                dist[v] = dist[u] + graph[u][v];
//    }
//  
//    // print the constructed distance array
//    printSolution(dist);
//}
//  
//// driver program to test above function
//int main()
//{
//    /* Let us create the example graph discussed above */
//    int graph[V][V] = { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
//                        { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
//                        { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
//                        { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
//                        { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
//                        { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
//                        { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
//                        { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
//                        { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };
//  
//    dijkstra(graph, 0);
//  
//    return 0;
//}



class Queue{
    public:
        Queue(){}
        std::deque<Node*> Q1;
        std::deque<Node*> Q2;
        std::deque<Node*> tot_queue(){
            std::deque<Node*> Q3;
            for (int i=0;i<this->Q1.size();i++){
                Q3.push_back(Q1[i]);
            }
            for (int j=0;j<this->Q2.size();j++){
                Q3.push_back(Q2[j]);
            }
            return Q3;
        };
        void TwoQueueInit(Graph G);
        void TwoQueueInsert(Node* j);
        void TwoQueueRemove(Node* j);
};

void printSolution(std::vector<int> Q)
{
    printf("Vertex \t\t Path\n");
    for (int i = 0; i < Q.size(); i++){
            printf("%d \t\t %d\n", i, Q[i]);
    }
}

void Queue::TwoQueueInit(Graph G){
    for (int i=0; i<G.Nodes.size();i++){
        this->Q1.push_back(G.Nodes[i]);
    }
}
void Queue::TwoQueueInsert(Node* j){
    bool b=1;
    std::deque<Node*> Q1=this->tot_queue();
    for (int i=0; i<Q1.size();i++){
        if (j->index==Q1[i]->index){
            b=0;
        }
    }
    if (b){
        if(j->status==0){
            this->Q1.push_back(j);
        }
        else if (j->status==1){
            this->Q2.push_back(j);
        }
    }
}
void Queue::TwoQueueRemove(Node* j){
    if(Q1.size()){
        if (Q1[0]->index==j->index){
            this->Q1.pop_front();
        }
    }
    else{
        if(Q2.size()){
            if (Q2[0]->index==j->index){
            this->Q2.pop_front();
            }
        }
    }
}
std::vector<int> TwoQueue(Graph G){
    Queue Q;
    std::vector<int> path;
    Q.TwoQueueInit(G);
    while (Q.tot_queue().size()){
        for (int i=0;i<G.Nodes.size();i++){
            Q.TwoQueueRemove(G.Nodes[i]);
            for (int j=0; j<G.Nodes[i]->AdjNodes.size();j++){
                if (G.Nodes[i]->AdjNodes[j].first->dist>G.Nodes[i]->dist+G.Nodes[i]->AdjNodes[j].second){
                    G.Nodes[i]->AdjNodes[j].first->dist=G.Nodes[i]->dist+G.Nodes[i]->AdjNodes[j].second;
                    Q.TwoQueueInsert(G.Nodes[i]->AdjNodes[j].first);
                    G.Nodes[i]->AdjNodes[j].first->status=1;
                }
            }
        }
    }
    for (int i=0; i<G.Nodes.size();i++){
        G.Nodes[i]->status=2;
        path.push_back(G.Nodes[i]->dist);
    }
    return path;
}

std::vector<Graph> GraphPartitioning(Graph G,int proc){//N number of nodes for each graphs
    std::vector<Graph> Graphs;
    for (int i=0; i<G.Nodes.size();i++){
        
    }
    return Graphs;
}

//Path ParallelSSSP(Graph G, Node s){
//
//}
int main(){
    std::vector<Node*> Nodes;
    Node n1(1);
    Node n2(2);
    Node n3(3);
    Node n4(4);
    Node n5(5);
    Node n6(6);
    Node s(0);
    Nodes.push_back(&s);
    Nodes.push_back(&n1);
    Nodes.push_back(&n2);
    Nodes.push_back(&n3);
    Nodes.push_back(&n4);
    Nodes.push_back(&n5);
    Nodes.push_back(&n6);
    Graph G;
    G.add_nodes(Nodes,0);
    G.add_edges(0,1,2);
    G.add_edges(1,0,2);
    G.add_edges(0,2,3);
    G.add_edges(2,0,3);
    G.add_edges(2,3,5);
    G.add_edges(3,2,5);
    G.add_edges(2,4,3);
    G.add_edges(4,2,3);
    G.add_edges(3,4,1);
    G.add_edges(4,3,1);
    G.add_edges(3,5,6);
    G.add_edges(5,3,6);
    G.add_edges(4,1,3);
    G.add_edges(1,4,3);
    G.add_edges(5,6,2);
    G.add_edges(6,5,2);
    G.add_edges(4,6,2);
    G.add_edges(6,4,2);
    G.add_edges(6,1,2);
    G.add_edges(1,6,2);
    std::vector<int> Q=TwoQueue(G);
    printSolution(Q);
}