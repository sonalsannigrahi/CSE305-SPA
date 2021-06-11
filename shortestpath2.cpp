//
//  shortestpath.cpp
//  
//
//  Created by Sonal Sannigrahi on 01/05/2021.
//

#include "shortestpath.hpp"
//#include "./metis-5.1.0/libmetis/kmetis.c"
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
#include <functional>
#include <mutex>
#include <utility>
#include <string>
// Number of vertices in the graph
#define V 9
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

/*graph partiioning:
Input: graph, nb of processors using
Output: an array of partitioned graphs
for each node, set the graph nb when partitioning
also, for each graph, set the adjacent graph vector

*/

std::vector<Graph> GraphPartitioning(Graph G,int proc,int s){
    std::vector<Graph> graphs;
    std::fstream file;
    int index=0;
    file.open("data",std::ios::in);
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
            //size_t end=ch.find_last_not_of("\n");
            //std::string sch=ch.substr(0,end+1);
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
                    G1->AdjGraphs.push_back(G1->Nodes[k]->AdjNodes[w].first->GraphIndex);
                }
            }
        }
        graphs.push_back(*G1);
    }
    return graphs;
}

void addMessage(std::vector<int> Messagearray,Node* node){
    std::mutex lk;
    lk.lock();
    lk.unlock();
}
std::vector<std::pair<Node*,int> > ParDijk(Graph G){
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
                int sendTag=0;//maybe should be atomic or smth..? idk or lock and unlock while adding
                std::vector<Node*> sucNodes;
                for (int k=0; k<G.Nodes[i]->AdjNodes.size();k++){
                    if(G.Nodes[i]->AdjNodes[k].first->GraphIndex!=G.Nodes[i]->GraphIndex){
                        sucNodes.push_back(G.Nodes[i]->AdjNodes[k].first);
                    }
                }
                for(int g=0;g<G.AdjGraphs.size();g++){
                    for (int r=0;r<sucNodes.size();r++){
                        if (sucNodes[r]->dist>G.Nodes[i]->dist+sucNodes[r]->dist){
                            //add info to messagearray
                            sendTag=1;
                        }
                    }
                }
            }
            int ExitFlag;
            if(!ExitFlag){
                int m;//nb of received nodes from current proc n
                for (int i=0;i<G.AdjGraphs.size();i++){
                    //send and receive message array
                }
                for (int j=0;j<m;j++){

                }
            }
            else{
                break;
            }
        }
    }
        for (int i=0; i<G.Nodes.size();i++){
            G.Nodes[i]->status=2;
            std::pair<Node*,Edge> p1(G.Nodes[i],G.Nodes[i]->dist);
            path.push_back(p1);

    }
    return path;
}

std::vector<std::pair<Node*,int> > ParallelSSSP(Graph G, int proc){
    Queue Q;
    std::vector<Graph> graphs=GraphPartitioning(G,proc,0);
    std::vector<std::thread> thread(proc);
    std::vector<std::vector<std::pair<Node*, int> > > results(proc);
    std::vector<std::pair<Node*, int> > paths;
    for (int i=0;i<proc;i++){
        //thread[i]=std::thread(ParDijk,graphs[i],std::ref(results[i]));
    }
    std::for_each(thread.begin(),thread.end(),std::mem_fn(&std::thread::join));
    for (int j=0; j<proc;j++){
        for (int k=0;j<results[j].size();k++){
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
int main()
{
    /* Let us create the example graph discussed above */
    int graph[V*V] = {  0, 4, 0, 0, 0, 0, 0, 8, 0 ,
                         4, 0, 8, 0, 0, 0, 0, 11, 0 ,
                         0, 8, 0, 7, 0, 4, 0, 0, 2 ,
                         0, 0, 7, 0, 9, 14, 0, 0, 0 ,
                         0, 0, 0, 9, 0, 10, 0, 0, 0 ,
                         0, 0, 4, 14, 10, 0, 2, 0, 0 ,
                         0, 0, 0, 0, 0, 2, 0, 1, 6 ,
                         8, 11, 0, 0, 0, 0, 1, 0, 7 ,
                         0, 0, 2, 0, 0, 0, 6, 7, 0 } ;
    Graph G=build_graph(graph,0,9,0);
    std::vector<int> Q=TwoQueue(G);
    printSolution(Q);
    return 0;
}