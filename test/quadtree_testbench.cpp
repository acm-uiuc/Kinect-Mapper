

#include "../quadtree/quadtree.hpp"
#include <iostream>
#include <assert.h>
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>


int main()
{
  QuadTree<int> testTree = QuadTree<int>();
  
  int * c = new int(); 
  /* the empty stats in the tree as the root 
  node*/
  int * s = new int(3);
  int * e = new int (2);
  int * w = new int(7);
  int * n = new int(8);
  int * nn = new int (9);
  
  //testTree.add_data(c,int(),QuadTree<int>::North);
  BOOST_REQUIRE (testTree.is_in_tree(c));
  BOOST_CHECK (testTree.add_data_next_to(nn,c,QuadTree<int>::South));
  BOOST_CHECK(testTree.add_data_next_to(n,c,QuadTree<int>::South));
  BOOST_CHECK(testTree.is_in_tree(nn));
  BOOST_CHECK(!testTree.is_in_tree(e));

	return 0;
}
