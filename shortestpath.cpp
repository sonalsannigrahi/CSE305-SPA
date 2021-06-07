
#include "shortestpath.hpp"
#define MAX 240886
#define INF -1

// delta-stepping method

int **adjacency_matrix;
std::vector<std::pair <int,int>> N_l;
std::vector<std::pair <int,int>> N_h;
class DeltaStep{
public:
     //static int thread_num;
     int vertex_num, edge_num;
     std::vector<int> parent;
     std::set<int> buckets[MAX+1];
     std::vector<int> distance;
     int delta;
     std::set<int> neighbours;
     std::vector<std::set<int>> L;
     std::vector<std::set<int>> H;
     
     //class methods
     
     void relax(int u, int v, int w);
     
     std::set<std::vector<int>> GenRequests(std::set<std::vector<int>> R, std::set<int> N, int v);
     
     void deltastepping(int source);
    
     bool validate(int source);
    int minDistance(int dist[], bool sptSet[]);
     //constructor
     
     DeltaStep(int delta, int vertex_num, int edge_num, std::vector<std::set<int>> L, std::vector<std::set<int>> H){
         this->delta = delta;
         this->vertex_num = vertex_num;
         this->edge_num = edge_num;
         this->L = L;
         this->H = H;
         for(int v=0; v < this->vertex_num; v++){
             this->distance.push_back(MAX);
             this->parent.push_back(INF);
             this->buckets[MAX].insert(v);
         }
     }
 };

 void DeltaStep::relax(int u, int v, int w){
     if(distance[v] + w < distance[u]){
         int i = distance[u]/this->delta;
         int j = (distance[v] + w)/this->delta;
         int remove= *this->buckets[i].find(u);
         this->buckets[i].erase(remove);
         this->buckets[j].insert(u);
         this->distance[u] = this->distance[v] + w;
         this->parent[u] = v;
         return;
     }
     return;
 }

 std::set<std::vector<int>> DeltaStep::GenRequests(std::set<std::vector<int>> R, std::set<int> N, int v){
     for (auto u : N){
         std::vector<int> elem;
         elem.push_back(u);
         elem.push_back(v);
         elem.push_back(adjacency_matrix[u][v]);
         R.insert(elem);
     }
     return R;
 }

 void DeltaStep::deltastepping(int source){
     this->distance[source] = 0;
     this->parent[source] = source;
     this->buckets[MAX].erase(source);
     this->buckets[0].insert(source);

     int k = 0;
     while(k < MAX){
         //define heavy and light requests
         std::set<std::vector<int>> R_h;
         std::set<std::vector<int>> R_l;
         while(!this->buckets[k].empty()){
             while(!this->buckets[k].empty()){
                 int v= *this->buckets[k].begin(); //do in parallel
                 this->buckets[k].erase(this->buckets[k].begin());
                 R_l = DeltaStep::GenRequests(R_l, this->L[v],v);
                 R_h = DeltaStep::GenRequests(R_h, this->H[v],v);
             }
             while(!R_l.empty()){ //do in parallel
                 //Relax light weight requests
                 std::vector<int> elem = *R_l.begin();
                 R_l.erase(R_l.begin());
                 DeltaStep::relax(elem[0],elem[1],elem[2]);
             }
         }
         while(!R_h.empty()){ //do in parallel
             //Relax heavy weight requests
             std::vector<int> elem = *R_h.begin();
             R_h.erase(R_h.begin());
             DeltaStep::relax(elem[0],elem[1],elem[2]);
         };
         
         //find the smallest non-empty bucket
         int i =0;
         while(this->buckets[i].empty()){
             i++;
         }
         k = i;
     }
 };

int DeltaStep::minDistance(int dist[], bool path[])
{
    int min = INT_MAX, min_index;
    for (int v = 0; v < this->vertex_num; v++)
        if (path[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;
  
    return min_index;
}
bool DeltaStep::validate(int source){
    //verify result with sequential dijkstra
    int dist[this->vertex_num];
  
    bool path[this->vertex_num];
  
    for (int i = 0; i < this->vertex_num; i++)
        dist[i] = INT_MAX, path[i] = false;
  
    dist[source] = 0;
  
    for (int count = 0; count < this->vertex_num - 1; count++) {
        int u = DeltaStep::minDistance(dist, path);
        path[u] = true;
        for (int v = 0; v < this->vertex_num; v++)
            if (!path[v] && dist[u] != INT_MAX
                && adjacency_matrix[u][v] != INF
                && dist[u] + adjacency_matrix[u][v] < dist[v])
                dist[v] = dist[u] + adjacency_matrix[u][v];
    }
    
    //verify distance vector is the same
    for(int v=0; v < this->vertex_num; v++){
        if(this->distance[v] != dist[v]){
            return false;
        }
    }
    return true;
    
}
 int main(int argc, char *argv[]){
     //check type of argument
     if (argc != 5) {
         fprintf(stderr, "Insufficient arguments\n");
         fprintf(stderr, "Usage: ./%s thread input_file output_file source_id \n", argv[0]);
         exit(EXIT_FAILURE);
     }
     //load argument
     int thread_num = atoi(argv[1]);
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
     int vertex_num;
     int edge_num;
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
     
     //create instance of DeltaStep
     
     DeltaStep main_algorithm(delta, vertex_num, edge_num, L, H);
     
     main_algorithm.deltastepping(source);


     //write file
     for(int i=0; i< main_algorithm.vertex_num; i++){
         std::cout<<i+1<< " and parent "<< main_algorithm.parent[i]<< " and shortest distance "<< main_algorithm.distance[i]<<std::endl;
     }
     
     //validating result
     
     bool res = main_algorithm.validate(source);
     
     if(res){
         std::cout<<"Result is correct!"<<std::endl;
     }
     else{
         std::cout<<"Delta Stepping gave an incorrect result"<<std::endl;
     }
     
     //parents vector is wrong, distances are correct!
     fh_out = fopen(OUTPUT_NAME,"w");
     if(fh_out == NULL){
         printf("Output file open failed.\n");
     }
     std::stack<int> S;
     for(int i = 0; i < main_algorithm.vertex_num; i++){
         int in_stack = i;
         S.push(in_stack);
         while(S.top() != source){
             in_stack = main_algorithm.parent[in_stack];
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
