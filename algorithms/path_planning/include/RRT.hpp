#ifndef RRT_H
#define RRT_H

#include <vector>
#include <cppTypes.h>
class Node;
class NodeCheck;

namespace ExtResult{
    constexpr int Trapped = -1;
    constexpr int Reached = 0;
    constexpr int Advanced = 1;
};


class RRT{
    public: 
        RRT(const Node* ini, const Node* fin);
        ~RRT();

        bool FindPath();
    protected:
        int _ExtendTree(Node* node);
        Node* _FindNearestNeighbor(const Node* node);

        // Select Node (can be randomly generated or occationally choose goal)
        virtual void _CreateTestNode(Node* & node) = 0;
        virtual bool _NewState(const Node* test_node, Node* node_near, Node* & new_node) = 0;
        void _print_summary(const size_t & iter);
        void _save_path();
        void _save_result();

        std::vector<Node*> _tree;
        Node* _fin;
        NodeCheck* _node_checker;

};

#endif
