#ifndef HIERARCHYTREE_H
#define HIERARCHYTREE_H

#include <vector>

template <typename dataType>
struct node
{
    dataType data;
    node* parent;
    unsigned int childrenSize;
};

//template <typename dataType>
//struct tree
//{
//    std::vector<node*> node_list;
//};

class HierarchyTree
{
public:
    HierarchyTree();
};

#endif // HIERARCHYTREE_H
