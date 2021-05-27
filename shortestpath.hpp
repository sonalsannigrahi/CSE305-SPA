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

typedef signed int Vertex;
typedef signed int Edge;
typedef std::vector<int> Path;



class Graph{
    public:
        std::vector<Vertex> Vertices;
        std::vector<Edge> Edges;
        Vertex s;
    };
        
#endif /* shortestpath_hpp */
