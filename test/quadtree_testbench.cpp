
#define BOOST_TEST_DYN_LINK
#include "../quadtree/quadtree.hpp"
#include <iostream>
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_SUITE( Quadtree )
BOOST_AUTO_TEST_CASE( test_adding_thing )
{
  BOOST_MESSAGE( "Testing is in progress" );
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
      
}
BOOST_AUTO_TEST_CASE( test_coppying)
{
  BOOST_MESSAGE( "Testing is in progress" );
  QuadTree<int> testTree = QuadTree<int>();
  
  int * c = new int(); 
  /* the empty stats in the tree as the root 
     node*/
  int * s = new int(3);
  int * e = new int (2);
  int * w = new int(7);
  int * n = new int(8);
  int * nn = new int (9);
  

  BOOST_REQUIRE (testTree.is_in_tree(c));
  BOOST_CHECK (testTree.add_data_next_to(nn,c,QuadTree<int>::South));
  BOOST_CHECK(testTree.add_data_next_to(n,c,QuadTree<int>::South));
  QuadTree<int> copy= new QuadTree<int>(&testTree);
  BOOST_CHECK(copy.is_in_tree(nn));
  BOOST_CHECK(!copy.is_in_tree(e));
      
}

BOOST_AUTO_TEST_SUITE_END()

 // int main( int argc, char* argv[] )
 // {
 //   return ::boost::unit_test::unit_test_main( &init_function, argc, argv );
 // }

