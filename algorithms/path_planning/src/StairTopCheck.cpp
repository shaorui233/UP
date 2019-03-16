#include <StairTopCheck.hpp>
#include <RRT_LocYaw.hpp>

StairTopCheck::StairTopCheck():NodeCheck(){}

bool StairTopCheck::CheckNode(Node* node){
    LocYaw* ly_node = ((LocYaw*)node);
    double x = ly_node->_x;
    double y = ly_node->_y;
    if(y> 0.6) return false;
    if(y< -1.9) return false;
    if(x> 2.7) return false;

    if((-0.9 < y) && (y < -0.5) && (x<2.2) ) return false;
    if((-0.9 > y) && (x < 1.4) ) return false;

    return true;
}
