// QuadAVL.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "QuadTree.h"
#include <iostream>


int _tmain(int argc, _TCHAR* argv[])
{
	using namespace std;
	cout << "begin" << endl;

	QuadTree<int> tree = QuadTree<int>(8);

	tree.insert(4231, 0, 0);

	cout << (tree.findNode(0, 0))->data << endl;

	tree.resetCoordinate(0, 0);

	cout << (tree.findNode(0, 0))->data << endl;

	tree.insert(37, 5, 4);

	cout << tree.findNode(5, 4)->data << endl;

	cout << "finish" << endl;

	cin.get();
	return 0;
}
