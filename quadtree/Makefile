COMPILER = g++
COMPILER_FLAGS = -c -g -O0 -Wall -Werror
LINKER = g++

QuadTreeTest : quadtree_testbench.o quadtree.o
	$(LINKER) quadtree_testbench.o quadtree.o -o QuadTreeTest

quadtree_testbench.o : quadtree_testbench.cpp quadtree.h
	$(COMPILER) $(COMPILER_FLAGS) quadtree_testbench.cpp


