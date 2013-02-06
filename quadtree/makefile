COMPILER = g++
COMPILER_FLAGS = -c -g -O0 -Wall -Werror 

LINKER = g++

quadtreeTester :  quadtree_testbench.o
	$(LINKER) -o quadtreeTester   quadtree_testbench.o

 quadtree_testbench.o : quadtree.hpp quadtree.h quadtree_testbench.cpp
	$(COMPILER) $(COMPILER_FLAGS) quadtree_testbench.cpp



