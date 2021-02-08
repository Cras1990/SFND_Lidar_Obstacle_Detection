/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void _insert(Node*& node, unsigned int depth, std::vector<float> point, int id)
	{
		
		unsigned int current_dimension = depth % 2;

		if (node == nullptr)
		{
			node = new Node(point, id);
		}
		else if (point[current_dimension] < node->point[current_dimension]) /* x or y depending on depth. If less than ... go to left */
		{
			_insert(node->left, depth+1, point, id);
		}
		else /* x or y depending on depth  */
		{
			_insert(node->right, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		_insert(root, 0, point, id);

	}


	void _search(Node*& node, unsigned int depth, std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{

		unsigned int current_dimension = depth % 2;
		
		if (node != nullptr)
		{
			if (((node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol))) && ((node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol))))
			{

				float dist = sqrt((target[0] - node->point[0]) * (target[0] - node->point[0]) + (target[1] - node->point[1]) * (target[1] - node->point[1]));
				if (dist <= distanceTol)
				{
					ids.push_back(node->id);
				}

			}

			if((node->point[current_dimension] > (target[current_dimension] - distanceTol))) // move to right since point[current_dimension] might lie inside of distance tol to the right of target 
				_search(node->left, depth + 1, target, distanceTol, ids);

			if ((node->point[current_dimension] < (target[current_dimension] + distanceTol))) // move to left since point[current_dimension] might lie inside of distance tol to the left of target
				_search(node->right, depth + 1, target, distanceTol, ids);


		}	
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		_search(root, 0, target, distanceTol, ids);

		return ids;
	}
	

};




