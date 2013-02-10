

#include "quadtree.hpp"
#include <iostream>
#include <assert.h>

int main()
{
  QuadTree<int> testTree = QuadTree<int>();
  
  int * x = new int(6);
  int * y = new int(3);
  int * z = new int (2);
  
  testTree.add_data(x);
  assert(testTree.add_data_next_to(y,x,QuadTree<int>::South));
  assert(testTree.is_in_tree(y));
  assert(!testTree.is_in_tree(z));

	return 0;
}
