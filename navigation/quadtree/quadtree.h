#include "stdafx.h"
#include "assert.h"
#include <iostream>
#ifndef QUADTREE_H
#define QUADTREE_H

template <typename T>
class QuadTree {
private:
	//Data Structures
	typedef struct Quadrant {
		int x1;
		int x2;
		int y1;
		int y2;
	};

	typedef struct TreeNode	{
		T data;
		TreeNode * UpperLeft;
		TreeNode * BottomLeft;
		TreeNode * BottomRight;
		TreeNode * UpperRight;
	};
	int length;				//length is the square root number of coordinates or nodes
	int size;				//size = length * length
	TreeNode * head;		//head or root of the tree

	//initializeNode sets all the pointers to null
	void initializeNode(TreeNode * node) {
		node->data = T();
		node->UpperLeft = NULL;
		node->BottomLeft = NULL;
		node->BottomRight = NULL;
		node->UpperRight = NULL;
		return;
	}

	void initializeNode(TreeNode * node, T data) {
		node->data = data;
		node->UpperLeft = NULL;
		node->BottomLeft = NULL;
		node->BottomRight = NULL;
		node->UpperRight = NULL;
		return;
	}

	void initializeQuadrant(Quadrant * quad) {
		quad->x1 = 0;
		quad->x2 = length - 1; // subtract 1 becuase grid is from 0 to length - 1
		quad->y1 = 0;
		quad->y2 = length - 1; // subtract 1 becuase grid is from 0 to length - 1
	}

	bool doesNodeHaveAllChildren(TreeNode * node) {
		if (node->UpperLeft == NULL) {
			return false;
		} else if (node->UpperRight == NULL) {
			return false;
		} else if (node->BottomLeft == NULL) {
			return false;
		} else if (node->BottomRight == NULL) {
			return false;
		} else {
			return true;
		}
	}

	TreeNode * getNode(Quadrant * current_quadrant_lengths, int x, int y, TreeNode * node) {
		assert(current_quadrant_lengths->x1 <= x && current_quadrant_lengths->x2 >= x && current_quadrant_lengths->y1 <= y && current_quadrant_lengths->y2 >= y);
		int quadrant_half_x = (current_quadrant_lengths->x2 + 1) / 2;
		int quadrant_half_y = (current_quadrant_lengths->y2 + 1) / 2;
		if (!doesNodeHaveAllChildren(node)) {
			return node;
		}
		if (x >= quadrant_half_x) {
			current_quadrant_lengths->x1 = quadrant_half_x;
			if (y >= quadrant_half_y) {
				current_quadrant_lengths->y1 = quadrant_half_y;
				return getNode(current_quadrant_lengths, x, y, node->BottomRight);
			} else {
				current_quadrant_lengths->y2 = quadrant_half_y;
				return getNode(current_quadrant_lengths, x, y, node->UpperRight);
			}
		} else {
			current_quadrant_lengths->x2 = quadrant_half_x;
			if (y > quadrant_half_y) {
				current_quadrant_lengths->y1 = quadrant_half_y;
				return getNode(current_quadrant_lengths, x, y, node->BottomLeft);
			} else {
				current_quadrant_lengths->y2 = quadrant_half_y;
				return getNode(current_quadrant_lengths, x, y, node->UpperLeft);
			}
		}
	}

	void insertRow(TreeNode * & tree_node) {
		if (tree_node == NULL) {
			TreeNode * new_node = new TreeNode;
			initializeNode(new_node);
			tree_node = new_node;
		} else {
			insertRow(tree_node->UpperLeft);
			insertRow(tree_node->BottomLeft);
			insertRow(tree_node->BottomRight);
			insertRow(tree_node->UpperRight);
		}
		return;
	}

	void printChildren(TreeNode * node) {
		using namespace std;
		if (node == NULL) {
			return;
		} else {
			cout << "Node@: " << node << ", Data: " << node->data << ", UL: " << node->UpperLeft << ", BL: " << node->BottomLeft << ", BR: " << node->BottomRight << ", UR: " << node->UpperRight << endl;
			printChildren(node->UpperLeft);
			printChildren(node->BottomLeft);
			printChildren(node->BottomRight);
			printChildren(node->UpperRight);
			return;
		}
	}

	void deleteChildren(TreeNode * node) {
		if (node == NULL) {
			return;
		} else {
			deleteChildren(node->UpperLeft);
			deleteChildren(node->BottomLeft);
			deleteChildren(node->BottomRight);
			deleteChildren(node->UpperRight);
			delete node;
		}
	}

public:
	QuadTree() {
		length = 0;
		size = 0;
		head = NULL;
	}
	QuadTree(int length) {
		assert((length & (length - 1)) == 0); // length must be 2^k for some k
		this->length = length;
		this->size = length * length;
		int temp_size = this->size;
		while (temp_size > 0)
		{
			temp_size = temp_size / 4;
			insertRow(head);
		}
		
	}
	QuadTree(const QuadTree & tree) {
	}
	~QuadTree(){
		deleteChildren(head);
	}
	void insert(T data, int x, int y) {
		TreeNode * new_node = new TreeNode;
		//initialize it
		initializeNode(new_node);
		
		if (isEmpty()) {
			head = new_node;
		} else {
			Quadrant current_quadrant_lengths;
			initializeQuadrant(&current_quadrant_lengths);

			TreeNode * node = getNode(&current_quadrant_lengths, x, y, head);

			node->data = data;

		}
	}

	TreeNode * findNode(T data) {
		return head;
	}

	TreeNode * findNode(int x, int y) {
		Quadrant current_quadrant_lengths;
		initializeQuadrant(&current_quadrant_lengths);

		return getNode(&current_quadrant_lengths, x, y, head);
	}

	void resetCoordinate(int x, int y) {
		Quadrant current_quadrant_lengths;
		initializeQuadrant(&current_quadrant_lengths);

		getNode(&current_quadrant_lengths, x, y, head)->data = T();
	}
	bool isEmpty() {
		return head == NULL;
	}

	void printTree() {
		printChildren(head);
	}

	void growTree() {
	}
};

#endif
