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

  QuadTree();
  ~QuadTree();
  QuadTree(QuadTree * oldTree);

  void delete_branch(Tree_Node * deadNode);
  bool is_in_tree(DATA * data);
  bool add_data_next_to(DATA * to_add,DATA * next_to,Direction witch_way);
  void add_data(DATA * data);

 private:
 
  void add_data_helper(Tree_Node * subroot, DATA * data);
  Tree_Node * new_node(DATA * data);
  Tree_Node * node_deep_copy(Tree_Node * oldNode);
  Tree_Node * coppy_node(Tree_Node * oldNode);

  bool is_in_tree_helper(Tree_Node * sub_root, DATA * data);
  Tree_Node * find_in_tree_helper(Tree_Node * sub_root, DATA * data);

  int depth; //how deep is the deepetst part
  int size;
  Tree_Node * head;

  

};


 



#endif
