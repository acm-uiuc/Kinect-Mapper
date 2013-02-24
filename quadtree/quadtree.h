#include <assert.h>
#include <stdio.h>


#ifndef QUADTREE_H
#define QUADTREE_H



 
template <class DATA>
class QuadTree	{
 public:

enum Direction { North, South, East, West };

struct Tree_Node      {
    DATA * data;
    Tree_Node * North;
    Tree_Node * South;
    Tree_Node * East;
    Tree_Node * West;
    Tree_Node(DATA * data);
    Tree_Node(Tree_Node * oldNode);
  }; 
/**
Makes a new empty Quad Tree
 */
  QuadTree();

  ~QuadTree();
  /** \brief
     makes a deep copy of a Quad Tree
     \param[in] the old tree to copy
   */
  QuadTree(QuadTree * oldTree);

  /**\brief
     removes the branches under the node holding given data
     \param[in] the data of the branch you wish to cut at
   */
  bool delete_branch(DATA * data);

  /**\brief
     Tests to see if data is in the tree
     \param[in] the data you are checking for
     \return whether the data is in the tree
   */
  bool is_in_tree(DATA * data);

  /**\brief
     adds data to the north, south, east, or west of the node
     if data is already there it will move the old data to be to the 
      north, south, east, or west of the new data.
           \param[in] the data you wish to add
      \param[in] the data you wish to put it next to
       \param[in] the direction relative to the data 
      
   */
  bool add_data_next_to(DATA * to_add,DATA * next_to,Direction witch_way);

  /**
     adds data to an arbitrary spot available spot
     note: add_data_next_to is the preferred way to add things
     if we need to do this alot we should change this to
     closes spot to head.
      \param[in] the data you wish to add
   */
  void add_data(DATA * data);

 private:  

  void delete_branch(Tree_Node * deadNode); 
  void add_data_helper(Tree_Node * subroot, DATA * data);
  Tree_Node * new_node(DATA * data);
  Tree_Node * node_deep_copy(Tree_Node * oldNode);
  Tree_Node * copy_node(Tree_Node * oldNode);

  bool is_in_tree_helper(Tree_Node * sub_root, DATA * data);
  Tree_Node * find_in_tree_helper(Tree_Node * sub_root, DATA * data);

  int depth; //how deep is the deepetst part
  int size;
  Tree_Node * head;

  

};


 



#endif
