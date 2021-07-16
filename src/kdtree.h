/* \author Aaron Brown */
// Quiz on implementing kd tree


// Structure to represent node of kd tree
struct Node
{

	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
		: root(NULL)
	{}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else {

			int cd = depth % 3;
			if (point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	template<typename PointT>
	void insert(PointT* p, int id)
	{
		//converting point to vector
		std::vector<float> point;
		point.push_back(p->x);
		point.push_back(p->y);
		point.push_back(p->z);
		
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if ((node->point[0] < (target[0] + distanceTol) && node->point[0] > target[0] - distanceTol) && (node->point[1] < (target[1] + distanceTol) && node->point[1] > target[1] - distanceTol) && (node->point[2] < (target[2] + distanceTol) && node->point[2] > target[2] - distanceTol))
			{
				float distance = sqrt(((node->point[0] - target[0]) * (node->point[0] - target[0]))
					+ ((node->point[1] - target[1]) * (node->point[1] - target[1]))
					+ ((node->point[2] - target[2]) * (node->point[2] - target[2])));
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			if (target[depth % 3] - distanceTol < node->point[depth % 3])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if (target[depth % 3] + distanceTol > node->point[depth % 3])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	template<typename PointT>
	std::vector<int> search(PointT* target, float distanceTol)
	{
		std::vector<int> ids;
		
		//converting point to vector
		std::vector<float> targetVector;
		targetVector.push_back(target->x);
		targetVector.push_back(target->y);

		targetVector.push_back(target->z);

		searchHelper(targetVector, root, 0, distanceTol, ids);

		return ids;
	}
	
	
};




