#ifndef NODE_H
#define NODE_H

#include <cppTypes.h>

namespace NodeType{
    constexpr int root = 0;
    constexpr int middle = 1;
    constexpr int end = 2;
    constexpr int goal = -1;
};

class Input;

class Node{
    public:
        Node(){ _type = NodeType::end; _parent = NULL; _u = NULL; }
        virtual ~Node(){}

        bool IsGoal(){ 
            if(_type == NodeType::goal) return true;
            else return false;
        }
        virtual Node* _MakeAndCopyNode() const = 0;
        virtual double Dist(const Node*) const = 0;
        virtual bool CloseEnough(const Node* ) const = 0;
        virtual void print(const std::string & node_name) const = 0;
        virtual void saveNode(const std::string & file_name) const = 0;

        int _type;
        Node* _parent;
        Input* _u; // From parent to here
};

class Input{
    public:
        Input(){}
        virtual ~Input(){}
        virtual void Integration(
                const Node* curr, const size_t & idx, Input* u, Node* next) = 0;

        size_t _size_input_set;
};

class NodeCheck{
    public:
        NodeCheck(){}
        virtual ~NodeCheck(){}

        virtual bool CheckNode(Node* node) = 0;
};

#endif
