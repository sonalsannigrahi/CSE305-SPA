//
//  shortestpath.cpp
//
//
//  Created by Sonal Sannigrahi on 01/05/2021.
//

#include "shortestpath.hpp"
#define MAX 214
#define INF -1
//First implementing Djikstra in parallel

//Sequential djikstra for comparison
// Number of vertices in the graph
#define V 9
  
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
    // Initialize min value
    int min = INT_MAX, min_index;
  
    for (int v = 0; v < V; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;
  
    return min_index;
}
  
// A utility function to print the constructed distance array
void printSolution(int dist[])
{
    printf("Vertex \t\t Distance from Source\n");
    for (int i = 0; i < V; i++)
        printf("%d \t\t %d\n", i, dist[i]);
}
  
// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra(int graph[V][V], int src)
{
    int dist[V]; // The output array.  dist[i] will hold the shortest
    // distance from src to i
  
    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest
    // path tree or shortest distance from src to i is finalized
  
    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
  
    // Distance of source vertex from itself is always 0
    dist[src] = 0;
  
    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always equal to src in the first iteration.
        int u = minDistance(dist, sptSet);
  
        // Mark the picked vertex as processed
        sptSet[u] = true;
  
        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v = 0; v < V; v++)
  
            // Update dist[v] only if is not in sptSet, there is an edge from
            // u to v, and total weight of path from src to  v through u is
            // smaller than current value of dist[v]
            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
                && dist[u] + graph[u][v] < dist[v])
                dist[v] = dist[u] + graph[u][v];
    }
  
    // print the constructed distance array
    printSolution(dist);
}
  
// driver program to test above function


/*
namespace std
{
    template<class _container,
        class _Ty> inline
        bool contains(_container _C, const _Ty& _Val)
        {return std::find(_C.begin(), _C.end(), _Val) != _C.end(); }
};

void vertices(int graph[V][V], std::vector<int>)


void djikstra_parallel(size_t num_processors,int graph[V][V], int src, int tgt){
    std::vector<int> cluster;
    cluster.push_back(src);
    
    while(!std::contains(cluster, tgt)){
        
    }
    
    
}*/

/*

 int main() <---Potential Graph Structure
 {
     // Let us create the example graph discussed above
     int graph[V][V] = { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
                         { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
                         { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
                         { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
                         { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
                         { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
                         { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
                         { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
                         { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };
   
     dijkstra(graph, 0);
   
     return 0;
 } */

// delta-stepping method
static int thread_num;
int vertex_num, edge_num, min, c;
int **adjacency_matrix;
std::vector<int> parent;

//bool *visit;//true means visiting done, otherwise false

std::vector<std::pair <int,int>> N_l;
std::vector<std::pair <int,int>> N_h;
std::set<int> buckets[MAX+1];

std::vector<int> relax(int u, int v, int w, int delta, std::vector<int> distance){
    if(distance[v] + w < distance[u]){
        int i = distance[u]/delta;
        int j = (distance[v] + w)/delta;
        int remove= *buckets[i].find(u);
        buckets[i].erase(remove);
        buckets[j].insert(u);
        distance[u] = distance[v] + w;
        parent[u] = v;
        return distance;
    }
    return distance;
}

std::set<std::vector<int>> GenRequests(std::set<std::vector<int>> R, std::set<int> N, int v){
    for (auto u : N){
        std::vector<int> elem;
        elem.push_back(u);
        elem.push_back(v);
        elem.push_back(adjacency_matrix[u][v]);
        R.insert(elem);
        std::cout<<"Inserted something in R "<<std::endl;
    }
    std::cout<<"returning R "<<std::endl;
    return R;
}

int main(int argc, char *argv[]){
    //check type of argument
    if (argc != 5) {
        fprintf(stderr, "Insufficient arguments\n");
        fprintf(stderr, "Usage: ./%s thread input_file output_file source_id \n", argv[0]);
        exit(EXIT_FAILURE);
    }
    //load argument
    thread_num = atoi(argv[1]);
    const char *INPUT_NAME = argv[2];
    const char *OUTPUT_NAME = argv[3];
    const int source = atoi(argv[4]) - 1;
    const int delta = 1; //to be made into an argv input (was giving errors before!)
    //load vertex&edge information
    FILE *fh_in, *fh_out;
    fh_in = fopen(INPUT_NAME,"r");
    if(fh_in == NULL){
        printf("Input file open failed.\n");
    }
    fscanf(fh_in,"%d %d",&vertex_num,&edge_num);
    if(vertex_num - 1 > edge_num){
        fprintf(stderr, "Input graph is not a connected graph\n");
        fprintf(stderr, "Error on type of input graph\n");
        exit(EXIT_FAILURE);
    }

    //dynamic allocate memory to adjacency_matrix
    adjacency_matrix = new int*[vertex_num];
    for(int i = 0; i < vertex_num; i++) adjacency_matrix[i] = new int[vertex_num];

    //initialize matrix
    for(int i = 0; i < vertex_num; i++)
        for(int j = 0; j < vertex_num; j++)
            adjacency_matrix[i][j] = INF;
            
    //load weight
    int a, b, weight;//a and b is vertex_id1 and vertex_id2 respectively
    for(int i = 0; i < edge_num; i++){
        fscanf(fh_in, "%d %d %d", &a, &b, &weight);
        adjacency_matrix[a - 1][b - 1] = weight;
        adjacency_matrix[b - 1][a - 1] = weight;
    }
    
    //auxiliary to deal with neighbours
    for(int i=0; i<vertex_num; i++){
        for(int j=0; j<vertex_num; j++){
            int edge =adjacency_matrix[i][j];
            if(edge != INF){
                if(edge<= delta){
                    N_l.push_back(std::make_pair(i,j));

                }
                else{
                    N_h.push_back(std::make_pair(i,j));
                }
            }
        }
    }
    
    std::vector<std::set<int>> L(vertex_num);
    std::vector<std::set<int>> H(vertex_num);
    
    //structure of L and H-> [{neighbours of v1}, {neighbours of v2},....]
    for(auto iter: N_l){
        //defining light weight vertex neighbours
        L[iter.first].insert(iter.second);
    }
    for(auto iter: N_h){
        //defining heavy weight vertex neighbours
        H[iter.first].insert(iter.second);
    }
    fclose(fh_in);
    std::cout<<"Adjacency matrix and neighbours created"<<std::endl;
    std::vector<int> distance(vertex_num);
    for(int v=0; v< vertex_num; v++){
        distance[v] = MAX;
        parent.push_back(INF);
        buckets[MAX].insert(v);
    }
    distance[source] = 0;
    parent[source] = source;
    buckets[MAX].erase(source);
    buckets[0].insert(source);

    int k = 0;
    while(k < MAX){
        //define heavy and light requests
        std::set<std::vector<int>> R_h;
        std::set<std::vector<int>> R_l;
        while(!buckets[k].empty()){
            while(!buckets[k].empty()){
                int v= *buckets[k].begin(); //do in parallel
                buckets[k].erase(buckets[k].begin());
                /*auto *res_L = GenRequests(&R_l, L[v],v);
                auto *res_H = GenRequests(&R_h, H[v],v);
                R_l.insert(res_L->begin(), res_L->end());
                R_h.insert(res_H->begin(), res_H->end());*/
                R_l = GenRequests(R_l, L[v],v);
                R_h = GenRequests(R_h, H[v],v);
            }
            while(!R_l.empty()){ //do in parallel
                //Relax light weight requests
                std::vector<int> elem = *R_l.begin();
                R_l.erase(R_l.begin());
                distance = relax(elem[0],elem[1],elem[2], delta, distance);
                std::cout<<"Lightweights relaxed"<<std::endl;

            }
        }
        while(!R_h.empty()){ //do in parallel
            //Relax heavy weight requests
            std::vector<int> elem = *R_h.begin();
            R_h.erase(R_h.begin());
            distance = relax(elem[0],elem[1],elem[2], delta, distance);
            std::cout<<"Heavyweights relax"<<std::endl;
        };
        
        //find the smallest non-empty bucket
        int i =0;
        while(buckets[i].empty()){
            i++;
        }
        k = i;
    }
    //write file
    for(int i=0; i< vertex_num; i++){
        std::cout<<i+1<< " and parent "<< parent[i]<< " and shortest distance "<< distance[i]<<std::endl;
    }
    
    //parents vector is wrong, distances are correct!
    fh_out = fopen(OUTPUT_NAME,"w");
    if(fh_out == NULL){
        printf("Output file open failed.\n");
    }
    std::stack<int> S;
    for(int i = 0; i < vertex_num; i++){
        int in_stack = i;
        S.push(in_stack);
        while(S.top() != source){
            in_stack = parent[in_stack];
            S.push(in_stack);
        }
        fprintf(fh_out,"%d", source + 1);
    
        while(!S.empty()){
            if(S.top() == source && i != source) {
                S.pop();
                continue;
            }
            fprintf(fh_out," %d", S.top() + 1);
            S.pop();
        }
        fprintf(fh_out,"\n");
    }
    fclose(fh_out);
}
