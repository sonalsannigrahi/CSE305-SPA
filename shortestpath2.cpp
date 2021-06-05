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

/*
two queues, Qi(tentative node distances) and Q*i (IO steps)
Step 1: finds global minimum of Q*
*/

//std::vector<int> deletenode(std::vector<int> Qi,int n){
//    Qi.erase(std::remove(Qi.begin(),Qi.end(),n),Qi.end());
//    return Qi;
//}
//std::vector<std::vector<int> > GraphPartitioning(int graph[V][V],int n){
//    std::vector<std::vector<int> > Qs;//random drawer of vertices, gives several partitions of nodes
//    std::vector<int> Qi;
//    for (int i=0;i<V;i++){
//        Qi.push_back(i);
//    }
//    size_t len=V/n;
//    for (int i=0;i<len-1;i++){
//        std::vector<int> Q;
//        for (int j=0;j<n;j++){
//            int u=rand()/RAND_MAX*V;
//            Q.push_back(u);
//            deletenode(Qi,u);
//        }
//        Qs.push_back(Q);
//    }
//    Qs.push_back(Qi);
//    return Qs;
//}
//
//int globalmin(std::vector<int> Q){//<O(log n) time..?
//    int min=ULONG_MAX;
//    if (Q.size()==1){
//        return Q.front();
//    }
//    size_t len=Q.size()/2;
//    return std::min(globalmin(std::vector<int>(Q.begin(),Q.end()-len)),globalmin(std::vector<int>(Q.begin()+len,Q.end())));
//}
//
//std::pair<std::vector<int>,std::vector<int> > deletemin(std::vector<int> Q,std::vector<int> Qd, int L){
//    std::vector<int> R;
//    
//    std::pair<std::vector<int>,std::vector<int> > p1(Q,Qd);
//    return p1;
//}
//std::vector<std::vector<int> > dijkpar(std::vector<int> Q){
//
//}
//void dijkstra(){
//
//}
typedef std::vector<int> Queue;

Queue TwoQueueInit(Queue Q){}
Queue TwoQueueInsert(Queue Q, Node j){}
Queue TwoQueueRemove(Queue Q, Node j){}
Path TwoQueue(Graph G){
    Queue Q;
    TwoQueueInit(Q);
    for (int i=0;i<G.Nodes.size();i++){
        G.s;
        while (Q.size()){
            TwoQueueRemove(Q,G.Nodes[i]);

        }
    }

}