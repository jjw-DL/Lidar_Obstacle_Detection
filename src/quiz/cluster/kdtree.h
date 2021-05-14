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

    void insertHelper(Node** node, uint depth,std::vector<float> point,int id)
    {
        if(*node == NULL)
        {
            *node = new Node(point,id);
        }
        else
        {
            uint cd = depth%2; // cd = 0 或 1（分割轴）

            if(point[cd] < (*node)->point[cd])
            {   
                insertHelper(&(*node)->left,depth+1,point,id);
            }
            else
            {
                insertHelper(&(*node)->right,depth+1,point,id);
            }
        }
    }
    
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(&root,0,point,id);
	}

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if(node!=NULL)
		{   
            // 判断该点是否在box（圆的外界正方形）范围内，如果在则继续判断该点是否在圆内
			if( (node->point[0]>=(target[0]-distanceTol)) && (node->point[0]<=(target[0]+distanceTol)) \
				&& (node->point[1]>=(target[1]-distanceTol)) && (node->point[1]<=(target[1]+distanceTol)))
            {
                float dist = sqrt(pow((node->point[0]-target[0]),2) + pow((node->point[1]-target[1]),2));
                if(dist<=distanceTol)
                    ids.push_back(node->id);
            }
            // 如何该点在box范围内，则左右子树都要搜索
			if((target[depth%2]-distanceTol) < node->point[depth%2])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((target[depth%2]+distanceTol) > node->point[depth%2])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	
};




