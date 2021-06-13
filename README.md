# CSE305-SPA
Parallelisation of shortest path algorithms

Compile with the following command for ∆-Stepping:

make

./shortestpath $number_threads $input.txt $result.txt $source_node $delta

for example:

make && ./shortestpath 5 test.txt result_mine.txt 3 2
