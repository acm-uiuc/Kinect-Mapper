

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
  testTree.add_data(y);
  assert(testTree.is_in_Tree(y));
  assert(!testTree.is_in_Tree(z));

	return 0;
}
