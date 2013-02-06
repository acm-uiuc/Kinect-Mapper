#include <assert.h>
#include <stdio.h>


#ifndef QUADTREE_H
#define QUADTREE_H



 
template <class DATA>
class QuadTree	{
 public:

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
  bool is_in_Tree(DATA * data);
  void add_data(DATA * data);

 private:
  void add_data_helper(Tree_Node * subroot, DATA * data);
  Tree_Node * newNode(DATA * data);
  void deleteBranch(Tree_Node * deadNode);
  Tree_Node * node_Deep_Copy(Tree_Node * oldNode);
  Tree_Node * coppyNode(Tree_Node * oldNode);

  bool is_in_Tree_helper(Tree_Node * sub_root, DATA * data);
  int depth; //how deep is the deepetst part
  int size;
  Tree_Node * head;

  

};


 



#endif
