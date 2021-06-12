#include "shortestpath.hpp"
#include "metis.h"
//First implementing Djikstra in parallel

//sequential djikstra for comparison
#include <limits.h> 
#include <stdio.h>
#include <queue>
#include <random>
#include <algorithm>
#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <functional>
#include <mutex>
#include <utility>
#include <string>
#include <chrono>
// Number of vertices in the graph
#define V 9
std::mutex lk;

//input: array of n*n weights, the index of the node to start and to end (was planning to use this for big graphs, but not useful anymore), source index s
Graph build_graph(int graph[],int nbNodes_start,int nbNodes_end,int s){
    Graph G;
    std::vector<Node*> Nodes;
    for (int i=nbNodes_start;i<nbNodes_end;i++){
        if(i==s){
            G.add_nodes(i,1);
        }
        else{
            G.add_nodes(i,0);
        }
    }
    int j=nbNodes_start;
    int width=nbNodes_start;
    int len=nbNodes_end-nbNodes_start;
    for (int i=nbNodes_start;i<len*len;i++){
        if(graph[i]){
            G.add_edges(j,width,graph[i]);
            }
        width++;
        if(width==len){
            j++;
            width=0;
        }
    }
    return G;
}

//input: filename of the txt created by the parser, index of the source node s
Graph build_graph(const char* filename,int s){
    Graph G;
    FILE* file;
    file=fopen(filename,"r");
    int i=0;
    int nbv,nbe,v1,v2,w;
    std::string ch;
    if(!file){
        std::cout<<"no file"<<std::endl;
    }
    else{
        fscanf(file,"%d %d",&nbv,&nbe);
        for (int i=0;i<nbv;i++){
            if(i==s) {
                G.add_nodes(i,1);
            }
            else{
                G.add_nodes(i,0);
            }
        }
        for(int i = 0; i < nbe; i++){
            fscanf(file, "%d %d %d", &v1, &v2, &w);
            G.add_edges(v1-1,v2-1,w);
        }

    }
    return G;
}

std::vector<std::pair<Node*,int> > seqDijk(Graph G, int s){
    std::queue<Node*> Q;
    std::vector<std::pair<Node*,int> > path;
    for (int i=0;i<G.Nodes.size();i++){
        Q.push(G.Nodes[i]);
    }
    while (Q.size()){
        for(int i=0;i<G.Nodes.size();i++){
            Node* node=Q.front();
            Q.pop();
            for (int j=0;j<node->AdjNodes.size();j++){
                if(node->AdjNodes[j].first->dist>node->dist+node->AdjNodes[j].second){
                    node->AdjNodes[j].first->dist=node->dist+node->AdjNodes[j].second;
                }
            }
        }
    }
    for (int i=0;i<G.Nodes.size();i++){
        std::pair<Node*,int> p1(G.Nodes[i],G.Nodes[i]->dist);
        path.push_back(p1);
    }
    std::cout<<path.size()<<std::endl;
    return path;
}
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
    printf("Vertex \t\t Distance from the source\n");
    for (int i = 0; i < Q.size(); i++){
            printf("%d \t\t %d\n", i, Q[i]);
    }
}

void printSolution(std::vector<std::pair<Node*,Edge> > path)
{
    printf("Vertex \t\t Distance from the source\n");
    for (int i = 0; i < path.size(); i++){
            printf("%d \t\t %d\n", path[i].first->index, path[i].second);
    }
}
void verifySeqandPar(std::vector<int> Q,std::vector<std::pair<Node*,int> > path){
    bool b=1;
    std::cout<<Q.size()<<" ,"<<path.size()<<std::endl;
    if(Q.size()!=path.size()){
        std::cout<<"size not correct"<<std::endl;
        return;
    }
    for (int i=0;i<path.size();i++){
        if(Q[path[i].first->index]!=path[i].second){
            b=0;
        }
    }
    if(b){
        std::cout<<"results match"<<std::endl;
    }
    else{
        std::cout<<"big F"<<std::endl;
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

//Sequential Dijkstra using two queues
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
            //G.Nodes[i]->status=1;
        }
    }
    for (int i=0; i<G.Nodes.size();i++){
        G.Nodes[i]->status=2;
        path.push_back(G.Nodes[i]->dist);
    }
    return path;
}

/*graph partiioning:
Input: graph, nb of processors using, index of the source node s
Output: an array of partitioned graphs
for each node, set the graph nb when partitioning
also, for each graph, set the adjacent graph vector

how we do: 
first, construct the graph that represents the whole dataset
second, get the index of the partitioned graph from graph_partitioning.ipynb
third, read the file written by python, and partition the graph
last, run pardijk for each partitioned graph in a parallel manner, by running ParallelSSSP
end tadaa
*/
std::vector<Graph> GraphPartitioning1(Graph G, int proc,int s){
    //idx_t xadj[G.Nodes.size()];
    //idx_t adjncy[G.nbe];
    //idx_t weights[G.nbe];
    std::vector<idx_t> weights;
    std::vector<idx_t> xadj;
    std::vector<idx_t> adjncy;
    idx_t* edgecut;
    idx_t* part;
    std::vector<Graph> graphs;
    int nbn=G.Nodes.size();
    idx_t cons=1;
    int count=0;
    for (int i=0;i<G.Nodes.size();i++){
        xadj.push_back(count);
        count+=G.Nodes[i]->AdjNodes.size();
        for (int k=0; k<G.Nodes[i]->AdjNodes.size();k++){
            adjncy.push_back(G.Nodes[i]->AdjNodes[k].first->index);
            weights.push_back(G.Nodes[i]->AdjNodes[k].second);
        }
    }
    int a = METIS_PartGraphKway(&nbn,&cons,xadj.data(),adjncy.data(),NULL,NULL,weights.data(),&proc,NULL,NULL,NULL,edgecut,part);
    std::cout<<part[0]<<std::endl;
    for (int j=0;j<G.Nodes.size();j++){
        G.Nodes[j]->GraphIndex=part[j];
    }
    for (int p=0;p<proc;p++){
        Graph* G1 = new Graph;
        for (int r=0;r<G.Nodes.size();r++){
            if(G.Nodes[r]->GraphIndex==p){
                if (G.Nodes[r]->index==s){
                    G1->add_nodes(G.Nodes[r],1);
                }
                else{
                    G1->add_nodes(G.Nodes[r],0);
                }
            }
        }
        G1->index=p;
        for (int k=0;k<G1->Nodes.size();k++){
            for (int w=0;w<G1->Nodes[k]->AdjNodes.size();w++){
                if(G1->Nodes[k]->AdjNodes[w].first->GraphIndex!=G1->index){
                    G1->AdjGraphs.insert(G1->Nodes[k]->AdjNodes[w].first->GraphIndex);
                }
            }
        }
        graphs.push_back(*G1);
    }
    return graphs;
}
std::vector<Graph> GraphPartitioning(Graph G,int proc,int s){
    std::vector<Graph> graphs;
    std::fstream file;
    int index=0;
    file.open("data.txt",std::ios::in);
    if(!file){
        std::cout<<"no file"<<std::endl;
    }
    else{
        std::string ch;
        for (int i=0;i<G.Nodes.size();i++){
            if(file.eof() && i!=G.Nodes.size()-1){
                std::cout<<"bad file"<<std::endl;
                break;
            }
            file >>ch;
            int n=stoi(ch);
            G.Nodes[index]->GraphIndex=n;
            index++;
        }
    }
    for (int p=0;p<proc;p++){
        Graph* G1 = new Graph;
        for (int r=0;r<G.Nodes.size();r++){
            if(G.Nodes[r]->GraphIndex==p){
                if (G.Nodes[r]->index==s){
                    G1->add_nodes(G.Nodes[r],1);
                }
                else{
                    G1->add_nodes(G.Nodes[r],0);
                }
            }
        }
        G1->index=p;
        for (int k=0;k<G1->Nodes.size();k++){
            for (int w=0;w<G1->Nodes[k]->AdjNodes.size();w++){
                if(G1->Nodes[k]->AdjNodes[w].first->GraphIndex!=G1->index){
                    G1->AdjGraphs.insert(G1->Nodes[k]->AdjNodes[w].first->GraphIndex);
                }
            }
        }
        graphs.push_back(*G1);
    }
    return graphs;
}

void addMessage(std::vector<std::vector<Node*> > Messagearray,Node* node,int GraphIndex){
    std::lock_guard<std::mutex> lock(lk);
    Messagearray[GraphIndex].push_back(node);
    return;
}
void ParDijk(Graph G,std::vector<std::vector<Node*> > Messagearray,std::vector<int> sendTag,std::vector<std::pair<Node*,int> >& result){
    Queue Q;
    Q.TwoQueueInit(G);
    std::vector<std::pair<Node*,int> > path;
    while(true){
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
                //G.Nodes[i]->status=1;
                sendTag[G.index]=0;//maybe should be atomic or smth..? idk or lock and unlock while adding
                std::vector<Node*> sucNodes;
                for (int k=0; k<G.Nodes[i]->AdjNodes.size();k++){
                    if(G.Nodes[i]->AdjNodes[k].first->GraphIndex!=G.Nodes[i]->GraphIndex){
                        sucNodes.push_back(G.Nodes[i]->AdjNodes[k].first);
                    }
                }
                std::vector<int> AdjGraphs1(G.AdjGraphs.begin(),G.AdjGraphs.end()); 
                for (int r=0;r<sucNodes.size();r++){
                    for(int g=0;g<AdjGraphs1.size();g++){
                        if (sucNodes[r]->dist > G.Nodes[i]->dist+sucNodes[r]->dist && sucNodes[r]->GraphIndex==AdjGraphs1[g]){
                            addMessage(Messagearray,sucNodes[r],g);
                            sendTag[G.index]=1;
                        }
                    }
                }
            }
        }
        int ExitFlag=std::accumulate(sendTag.begin(),sendTag.end(),0);
        if(ExitFlag){
            std::vector<std::vector<Node*> > Messagearray1=Messagearray;
            std::cout<<Messagearray1[0].size()<<std::endl;
            for (int j=0;j<Messagearray1[G.index].size();j++){
                for (int k=0;k<Messagearray1[G.index][j]->ParNodes.size();k++){
                    if(Messagearray1[G.index][j]->ParNodes[k].first->GraphIndex==G.index){
                        if(Messagearray1[G.index][j]->dist>Messagearray1[G.index][j]->ParNodes[k].first->dist+Messagearray1[G.index][j]->ParNodes[k].second){
                            Messagearray1[G.index][j]->dist=Messagearray1[G.index][j]->ParNodes[k].first->dist+Messagearray1[G.index][j]->ParNodes[k].second;
                            Q.TwoQueueInsert(Messagearray1[G.index][j]->ParNodes[k].first);
                        }
                    }
                }
            }
        }
        else{
            break;
        }
    }
    for (int i=0; i<G.Nodes.size();i++){
        G.Nodes[i]->status=2;
        std::pair<Node*,Edge> p1(G.Nodes[i],G.Nodes[i]->dist);
        result.push_back(p1);
    }
    return;
}

std::vector<std::pair<Node*,int> > ParallelSSSP(Graph G, int proc){
    Queue Q;
    std::vector<Graph> graphs=GraphPartitioning(G,proc,0);
    std::vector<std::thread> thread(proc);
    std::vector<std::vector<std::pair<Node*, int> > > results(proc);
    std::vector<std::pair<Node*, int> > paths;
    std::vector<std::vector<Node*> > MessageArray(proc);
    std::vector<int> sendTag(proc);
    for (int i=0;i<proc;i++){
        thread[i]=std::thread(&ParDijk,graphs[i],MessageArray,sendTag,std::ref(results[i]));
    }
    std::for_each(thread.begin(),thread.end(),std::mem_fn(&std::thread::join));
    for (int j=0; j<proc;j++){
        for (int k=0;k<results[j].size();k++){
            paths.push_back(results[j][k]);
        }
    }
    return paths;
}
void printSolution(std::vector<std::pair<Node*,int> > paths)
{
    printf("Vertex \t\t Distance from the source\n");
    for (int i = 0; i < paths.size(); i++){
            printf("%d \t\t %d\n", paths[i].first->index, paths[i].second);
    }
}
int main() {
    std::string filename="128v1024e.txt";
    const char* file=filename.c_str();
    Graph G1=build_graph(file,0);
    const std::chrono::time_point<std::chrono::steady_clock> start2 = std::chrono::steady_clock::now();
    std::vector<std::pair<Node*,int> > Pathver=seqDijk(G1,0);
    const std::chrono::time_point<std::chrono::steady_clock> end2 = std::chrono::steady_clock::now();
    double time3=std::chrono::duration_cast<std::chrono::microseconds>(end2-start2).count();
    //printSolution(Pathver);
    const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::vector<int> Q1=TwoQueue(G1);
    const std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
    double time2=std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << time2 <<" micros."<< std::endl;
    //printSolution(Q1);
    std::vector<Graph> graphs=GraphPartitioning1(G1,3,0);
    const std::chrono::time_point<std::chrono::steady_clock> start1 = std::chrono::steady_clock::now();
    std::vector<std::pair<Node*,int> > path=ParallelSSSP(G1,3);
    const std::chrono::time_point<std::chrono::steady_clock> end1 = std::chrono::steady_clock::now();
    double time4=std::chrono::duration_cast<std::chrono::microseconds>(end1-start1).count();

    std::cout << "Sequential Dijkstra : " << time3 <<" micros."<< std::endl;
    std::cout << "Two Queue Dijkstra : " << time2 <<" micros."<< std::endl;
    std::cout << "Parallel Dijkstra : " << time4 <<" micros."<< std::endl;

    //printSolution(path);
    verifySeqandPar(Q1,path);
    verifySeqandPar(Q1,Pathver);
    return 0;
}