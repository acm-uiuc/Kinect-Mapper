 
#ifndef _QuadTree_C_
#define _QuadTree_C_


#include "quadtree.h"
#include <stdlib.h>

template <class DATA>
QuadTree<DATA>:: QuadTree()
{
  head=NULL;
  depth=0;
  size=0;
}

template <class DATA>
QuadTree<DATA>::QuadTree(QuadTree * oldTree)
{
  head=nodeDeepCopy(oldTree->head);
  depth=oldTree->depth;
  size=oldTree->size;
  
}

template <class DATA>
typename QuadTree<DATA>::Tree_Node * QuadTree<DATA>::node_Deep_Copy(Tree_Node * oldNode)
{
  if (oldNode ==NULL)
    {
      return NULL;
    }

  Tree_Node * subroot=coppyNode(oldNode);

  ((*subroot).North)=node_Deep_Copy((*subroot).North);
  ((*subroot).South)=node_Deep_Copy((*subroot).South);
  ((*subroot).East)=node_Deep_Copy((*subroot).East);
  ((*subroot).West) =node_Deep_Copy((*subroot).West);

}
template <class DATA>
typename QuadTree<DATA>::Tree_Node * QuadTree<DATA>::coppyNode(Tree_Node * oldNode)
{
  if (oldNode==NULL)
    {
      return NULL;
    }
  Tree_Node * newNode=new Tree_Node(oldNode);

  return newNode;
}
template <class DATA>
QuadTree<DATA>:: ~QuadTree()
{
  delete_branch(head);
}
template <class DATA>
void QuadTree<DATA>::delete_branch(Tree_Node * deadNode)
{
  if (deadNode!=NULL)
    {
      delete_branch(deadNode->North);
      delete_branch(deadNode->South);
      delete_branch(deadNode->East);
      delete_branch(deadNode->West);
      delete(deadNode->data);
      delete(deadNode);
    }
}

template <class DATA>
QuadTree<DATA>::Tree_Node::Tree_Node(Tree_Node * oldNode)
{
  newNode-> North=(*oldNode).North;
  newNode-> South=(*oldNode).South; 
  newNode-> East=(*oldNode).East; 
  newNode-> West=(*oldNode).West;
  newNode-> data=new DATA((*oldNode).data);
}

template <class DATA>
QuadTree<DATA>::Tree_Node:: Tree_Node(DATA * data)
{
  North=NULL;
  South=NULL; 
  East=NULL; 
  West=NULL;
  this->data=data;
}

template <class DATA>
void QuadTree<DATA>::add_data(DATA * data)
{
  if (head==NULL)
    {
      head = new Tree_Node(data);
    }
  else
    {
      add_data_helper(head,data);
    }
}

template <class DATA>
void QuadTree<DATA>::add_data_helper(Tree_Node * subroot,DATA * data)
{
  if (subroot->North==NULL)
    {
      subroot->North = new Tree_Node(data);  
    }
  else
    {
      add_data_helper(subroot->North,data);
    }
}

template <class DATA>
bool QuadTree<DATA>:: is_in_Tree( DATA * compare_data)
{
  return is_in_Tree_helper(head,compare_data);
}

template <class DATA>
bool QuadTree<DATA>::is_in_Tree_helper(Tree_Node * sub_root, DATA * compare_data)
{
  if (sub_root == NULL)
    {
      return 0;
    }
  if ((*compare_data)==(*(sub_root->data)))
    {
      return 1;
    }
  if (is_in_Tree_helper(sub_root-> North,compare_data))
    {
      return 1;
    }
  if (is_in_Tree_helper(sub_root-> South,compare_data))
    {
      return 1;
    }
  if (is_in_Tree_helper(sub_root-> East,compare_data))
    {
      return 1;
    }
  if (is_in_Tree_helper(sub_root-> West,compare_data))
    {
      return 1;
    }
  return 0;
}
#endif
