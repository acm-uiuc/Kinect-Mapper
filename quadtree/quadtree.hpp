 
#ifndef _QuadTree_HPP_
#define _QuadTree_HPP_


#include "quadtree.h"
#include <stdlib.h>

template <class DATA>
QuadTree<DATA>:: QuadTree()
{
  head=new Tree_Node(new DATA()); 
  //the first node hold only empty data
  //it cant be removed 
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
typename QuadTree<DATA>::Tree_Node * QuadTree<DATA>::node_deep_copy(Tree_Node * oldNode)
{
  if (oldNode ==NULL)
    {
      return NULL;
    }

  Tree_Node * subroot=copy_node(oldNode);

  ((*subroot).North)=node_deep_copy((*subroot).North);
  ((*subroot).South)=node_deep_copy((*subroot).South);
  ((*subroot).East)=node_deep_copy((*subroot).East);
  ((*subroot).West) =node_deep_copy((*subroot).West);

}
template <class DATA>
typename QuadTree<DATA>::Tree_Node * QuadTree<DATA>::copy_node(Tree_Node * oldNode)
{
  if (oldNode==NULL)
    {
      return NULL;
    }
  Tree_Node * new_node=new Tree_Node(oldNode);

  return new_node;
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
bool QuadTree<DATA>::delete_branch(DATA * data)
{

   Tree_Node * target_node=find_in_tree_helper(head,data);
 if (target_node==NULL)
    {
      return false;
    }
 if (head!=NULL)
    {
      delete_branch(target_node->North);
      delete_branch(target_node->South);
      delete_branch(target_node->East);
      delete_branch(target_node->West);
      target_node.North=NULL;
      target_node.South=NULL;
      target_node.East=NULL;
      target_node.West=NULL;

    }
}

template <class DATA>
QuadTree<DATA>::Tree_Node::Tree_Node(Tree_Node * oldNode)
{
  new_node-> North=(*oldNode).North;
  new_node-> South=(*oldNode).South; 
  new_node-> East=(*oldNode).East; 
  new_node-> West=(*oldNode).West;
  new_node-> data=new DATA((*oldNode).data);
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
bool QuadTree<DATA>::add_data_next_to(DATA * data,DATA * next_to,QuadTree::Direction witch_way)
{
  if (head==NULL)
    {
      return false;
    }
  else
    {
      Tree_Node * target_node=find_in_tree_helper(head,next_to);
      if (target_node==NULL)
	{
	  return false;
	}
	Tree_Node * temp=NULL;
      switch (witch_way){
      case North :

	if (target_node->North != NULL)
	  {
	    temp= target_node->North;
	  }
	target_node->North=new Tree_Node(data);
	target_node->North->North=temp;
	break;
      case South :
         temp=NULL;
	if (target_node->South != NULL)
	  {
	   temp= target_node->South;
	  }
	target_node->South=new Tree_Node(data);
	target_node->South->South=temp;
	break;

      case East :

	if (target_node->East != NULL)
	  {
	    temp= target_node->East;
	  }
	target_node->East=new Tree_Node(data);
	target_node->East->East=temp;
	break;

      case West :

	if (target_node->West != NULL)
	  {
	    temp= target_node->West;
	  }
	target_node->West=new Tree_Node(data);
	target_node->West->West=temp;
	break;
      }
      
    }
  return true;
}


template <class DATA>
void QuadTree<DATA>::add_data_helper(Tree_Node * subroot,DATA * data)
{
  if (subroot->North==NULL)
    {
      subroot->North = new Tree_Node(data);  
    }
  else if (subroot->South==NULL)
    {
      subroot->South = new Tree_Node(data);  
    }
  else if (subroot->East==NULL)
    {
      subroot->East = new Tree_Node(data);  
    }
  else if (subroot->North==NULL)
    {
      subroot->West = new Tree_Node(data);  
    }
  else
    {
      
      add_data_helper(subroot->North,data);
    }
}



template <class DATA>
bool QuadTree<DATA>:: is_in_tree( DATA * compare_data)
{
  return is_in_tree_helper(head,compare_data);
}

template <class DATA>
bool QuadTree<DATA>::is_in_tree_helper(Tree_Node * sub_root, DATA * compare_data)
{
  if (sub_root == NULL)
    {
      return 0;
    }
  if ((*compare_data)==(*(sub_root->data)))
    {
      return 1;
    }
  if (is_in_tree_helper(sub_root-> North,compare_data))
    {
      return 1;
    }
  if (is_in_tree_helper(sub_root-> South,compare_data))
    {
      return 1;
    }
  if (is_in_tree_helper(sub_root-> East,compare_data))
    {
      return 1;
    }
  if (is_in_tree_helper(sub_root-> West,compare_data))
    {
      return 1;
    }
  return 0;
}

template <class DATA>
typename QuadTree<DATA>::Tree_Node * QuadTree<DATA>::find_in_tree_helper(Tree_Node * sub_root, DATA * compare_data)
{
  if (sub_root == NULL)
    {
      return NULL;
    }

  if ((*compare_data)==(*(sub_root->data)))
    {
      return sub_root;
    }
  Tree_Node * check_dir;
  
  check_dir = find_in_tree_helper(sub_root-> North,compare_data);
  if (check_dir)
    {
      return check_dir;
    }
  check_dir = find_in_tree_helper(sub_root-> South,compare_data);
  if (check_dir)
    {
      return check_dir;
    }
  check_dir = find_in_tree_helper(sub_root-> East,compare_data);
    if (check_dir)
    {
      return check_dir;
    }
  check_dir =find_in_tree_helper(sub_root-> West,compare_data);
      if (check_dir)
    {
      return check_dir;
    }
 
  return NULL;
}

#endif
