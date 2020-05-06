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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		// Case tree is empty
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		// Otherwise, traverse down the tree
		else {
			// Calculate split type based on current depth
			int treeheight = depth % 3; //treeheight

			if (point[treeheight] < ((*node)->point[treeheight])) { // if less, left branch
				insertHelper(&((*node)->left), depth + 1, point, id);
			}
			else {
				insertHelper(&((*node)->right), depth + 1, point, id); //else right branch
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		//Recursive function to find and insert at a position
		insertHelper(&root, 0, point, id); //starting at depth 0 to start checking from root 

	}

		void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			float node_x = node->point[0];
			float node_y = node->point[1];
			float node_z = node->point[2];
			float target_x = target[0];
			float target_y = target[1];
			float target_z = target[2];
			if ((node_x >= target_x - distanceTol) && (node_x <= target_x + distanceTol)
				&& (node_y >= target_y - distanceTol) && (node_y <= target_y + distanceTol)
				&& (node_z >= target_z - distanceTol) && (node_z <= target_z + distanceTol))
			{
				float x_diff = node_x - target_x;
				float y_diff = node_y - target_y;
				float z_diff = node_z - target_z;
				float distance = sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff* z_diff));
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// recursive iterate
			int treeheight = depth % 3;
			if (target[treeheight] - distanceTol < node->point[treeheight])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if (target[treeheight] + distanceTol > node->point[treeheight])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};




