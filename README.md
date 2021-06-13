# CSE305-SPA
Parallelisation of shortest path algorithms


Compile with the following command for ∆-Stepping:

Before proceeding, please comment out the main function of shortestpath2.cpp.

make

./shortestpath $number_threads $input.txt $result.txt $source_node $delta

for example:

make && ./shortestpath 5 test.txt result_mine.txt 3 2


Compile with the following command for Two Queue Dijkstra algorithm:

Before proceeding, please comment out the main function of shortestpath.cpp.

Then, choose the dataset file of the name "(nbOfVertices)v(nbOfEdges)e.txt" or "roadNet-TX-final.txt" and write on the string filename on line 383 of the shortestpath2.cpp file.

Also, there are several graph partitions for different datasets : they are of the name "(nbOfVertices).(nbOfSubgraphs).txt" and road.txt(4 partitions) and road1.txt(8 partitions) of the dataset roadNet-TX-final.txt. Please replace the string of fopen on line 234 for the subgraphs by the name of the chosen file.

Finally, change the input of the number of threads (which is equal to the number of subgraphs of the previous step) on line 399 at the second argument of ParallelSSSP function.

When compiling with make and running with ./shortestpath, the execution time for the sequential Two Queue, the standard Dijkstra algorithm and the parallel version of the Two Queue will appear on the terminal, as well as the correctness of all the algorithms. 
