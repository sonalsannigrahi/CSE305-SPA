//
//  shortestpath.hpp
//  
//
//  Created by Sonal Sannigrahi on 01/05/2021.
//

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
    }
        
#endif /* shortestpath_hpp */
