#ifndef COLLISION_H
#define COLLISION_H

#include <Node.hpp>

class StairTopCheck: public NodeCheck{
    public:
        StairTopCheck();
        virtual ~StairTopCheck(){}
        virtual bool CheckNode(Node* node);
};
#endif
