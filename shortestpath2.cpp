#include "shortestpath.hpp"
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


std::mutex lk;


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
    std::cout<<G.Nodes.size()<<std::endl;
    std::cout<<G.nbe<<std::endl;
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
            for (int i=0;i<this->Q2.size();i++){
                Q3.push_back(Q2[i]);
            }
            for (int j=0;j<this->Q1.size();j++){
                Q3.push_back(Q1[j]);
            }
            return Q3;
        };
        Node* frontNode(){
            Node* node;
            if(Q2.size()){
                node=Q2.front();
            }
            else{
                if(Q1.size()){
                    node=Q1.front();
                }
            }
            std::cout<<node->index<<std::endl;;
            return node;
        }
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
    int count=0;
    if(Q.size()!=path.size()){
        std::cout<<"size not correct"<<std::endl;
        return;
    }
    for (int i=0;i<path.size();i++){
        if(Q[path[i].first->index]!=path[i].second){
            b=0;
            std::cout<<Q[path[i].first->index]<<" "<<path[i].second<<std::endl;
            count++;
        }
    }
    if(b){
        std::cout<<"results match"<<std::endl;
    }
    else{
        std::cout<<"results don't match"<<std::endl;
        std::cout<<count<<std::endl;
    }
}

void Queue::TwoQueueInit(Graph G){
    this->Q1.push_back(G.Nodes[0]);
}
void Queue::TwoQueueInsert(Node* j){
    bool b=1;
    std::deque<Node*> Q3=this->tot_queue();
    for (int i=0; i<Q3.size();i++){
        if (j->index==Q3[i]->index){
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
    if(j->index!=this->frontNode()->index){
        std::cout<<"bad remove"<<std::endl;
        return;
    }
    if(Q2.size()){
        this->Q2.pop_front();
    }
    else{
        if(Q1.size()){
            this->Q1.pop_front();
        }
    }
}

//Sequential Dijkstra using two queues
std::vector<int> TwoQueue(Graph G){
    Queue Q;
    std::vector<int> path;
    Q.TwoQueueInit(G);
    while (Q.tot_queue().size()){
            Node* node=Q.frontNode();
            Q.TwoQueueRemove(node);
            for (int j=0; j<node->AdjNodes.size();j++){
                if (node->AdjNodes[j].first->dist>node->dist+node->AdjNodes[j].second){
                    node->AdjNodes[j].first->dist=node->dist+node->AdjNodes[j].second;
                    Q.TwoQueueInsert(node->AdjNodes[j].first);
                    node->AdjNodes[j].first->status=1;
                }
            }
    }
    for (int i=0; i<G.Nodes.size();i++){
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

the file 256.*.txt are made apart using the metis library, because I could not run it on mac.
*/

std::vector<Graph> GraphPartitioning(Graph G,int proc,int s){
    std::vector<Graph> graphs;
    std::fstream file;
    int index=0;
    file.open("512.1.txt",std::ios::in);
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
void setsendtag(std::vector<int> sendTag,int graphindex){
    std::lock_guard<std::mutex> lock(lk);
    sendTag[graphindex]=1;
    return;
}
void exitaccumulate(int* exitFlag,std::vector<int> sendTag){
    std::lock_guard<std::mutex> lock(lk);
    int n=std::accumulate(sendTag.begin(),sendTag.end(),0);
    exitFlag=&n;
    return;
}
void ParDijk(Graph G,std::vector<std::vector<Node*> > Messagearray,std::vector<int> sendTag,std::vector<std::pair<Node*,int> >& result){
    Queue Q;
    Q.TwoQueueInit(G);
    std::vector<std::pair<Node*,int> > path;
    while(true){
        while (Q.tot_queue().size()){
            //std::cout<<Q.tot_queue().size()<<std::endl;
            Node* node=Q.frontNode();
            Q.TwoQueueRemove(node);
            for (int j=0; j<node->AdjNodes.size();j++){
                if (node->AdjNodes[j].first->dist>node->dist+node->AdjNodes[j].second){
                    node->AdjNodes[j].first->dist=node->dist+node->AdjNodes[j].second;
                    Q.TwoQueueInsert(node->AdjNodes[j].first);
                    node->AdjNodes[j].first->status=1;

                }
            }   
                //G.Nodes[i]->status=1;
                sendTag[G.index]=0;
                std::vector<std::pair<Node*,Edge> > sucNodes;
                for (int k=0; k<node->AdjNodes.size();k++){
                    if(node->AdjNodes[k].first->GraphIndex!=node->GraphIndex){
                        sucNodes.push_back(node->AdjNodes[k]);
                    }
                }
                std::vector<int> AdjGraphs1(G.AdjGraphs.begin(),G.AdjGraphs.end()); 
                for (int r=0;r<sucNodes.size();r++){
                    for(int g=0;g<AdjGraphs1.size();g++){
                        if (sucNodes[r].first->dist > node->dist+sucNodes[r].second && sucNodes[r].first->GraphIndex==AdjGraphs1[g]){
                            addMessage(Messagearray,sucNodes[r].first,g);
                            setsendtag(sendTag,G.index);
                        }
                    }
                }
            }
        int* ExitFlag;
        exitaccumulate(ExitFlag,sendTag);
        if(*ExitFlag){
            std::vector<std::vector<Node*> > Messagearray1=Messagearray;
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
    std::string filename="512v4096e.txt";
    
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
   //std::vector<Graph> graphs=GraphPartitioning(G1,4,0);
    const std::chrono::time_point<std::chrono::steady_clock> start1 = std::chrono::steady_clock::now();
    std::vector<std::pair<Node*,int> > path=ParallelSSSP(G1,1);
    const std::chrono::time_point<std::chrono::steady_clock> end1 = std::chrono::steady_clock::now();
    double time4=std::chrono::duration_cast<std::chrono::microseconds>(end1-start1).count();
//
    std::cout << "Sequential Dijkstra : " << time3 <<" micros."<< std::endl;
    std::cout << "Two Queue Dijkstra : " << time2 <<" micros."<< std::endl;
    std::cout << "Parallel Dijkstra : " << time4 <<" micros."<< std::endl;

    //printSolution(path);
    verifySeqandPar(Q1,path);
    verifySeqandPar(Q1,Pathver);
    return 0;
}