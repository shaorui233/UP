#include <RRT_LocYaw.hpp>
#include <Node.hpp>
#include <Utilities/save_file.h>
#include <StairTopCheck.hpp>

RRT_LocYaw::RRT_LocYaw(const Node* ini, const Node* fin):RRT(ini, fin){
    _node_checker = new StairTopCheck();
    printf("[RRT LocYaw] Constructed\n");
}
RRT_LocYaw::~RRT_LocYaw(){ delete _node_checker; }

// Select Node (can be randomly generated or occationally choose goal)
void RRT_LocYaw::_CreateTestNode(Node* & node){
    if(_count%_goal_bias_frequency == 1){
        node = _fin->_MakeAndCopyNode();
    }else{
        // Random Generation
        node = new LocYaw();
    }
    ++_count;
}

// Create new node and test whether it is valide (obstacle free)
bool RRT_LocYaw::_NewState(const Node* test_node, Node* node_near, Node* & new_node){

    static size_t idx(0);
    Node* node_tmp = new LocYaw();

    double min_dist(1.e10);
    Input* input = new ForwardYaw();
    Input* u = new ForwardYaw();

    bool choose_optimal(false);
    if(choose_optimal){
        while(idx < input->_size_input_set){
            input->Integration(node_near, idx, u, node_tmp);
            if(test_node->Dist(node_tmp) < min_dist){  //Find the best input & node
                new_node = node_tmp;
                u = input;
            }
            ++idx;
        }
    }else {
        input->Integration(node_near, idx, u, node_tmp);
        new_node = node_tmp;
        u = input;
        ++idx;
        if(idx > input->_size_input_set){
            idx = 0;
        }
    }

    // Check the new node is valid
    if(_node_checker->CheckNode(new_node)){
        new_node->_u = u;
        new_node->_type = NodeType::end;
        if(node_near->_type != NodeType::root){
            node_near->_type = NodeType::middle;
        }
        new_node->_parent = node_near;
        //node_near->print("near");
        //new_node->_parent->print("parent");
        return true;
    }else {
        return false;
    }
}






// *****************  Node  **************************** //
double LocYaw::Dist(const Node* node) const {
    const LocYaw* n = (const LocYaw*)node;
    double dist(0.);
    dist += (_x - n->_x) * (_x - n->_x);
    dist += (_y - n->_y) * (_y - n->_y);
    dist += (_theta - n->_theta) * (_theta - n->_theta);

    return sqrt(dist);
}
bool LocYaw::CloseEnough(const Node* node) const {
    if(this->Dist(node) < 0.05){ return true; }
    else { return false; }
}

Node* LocYaw::_MakeAndCopyNode() const{
    Node* node = new LocYaw();
    ((LocYaw*)node)->_x = this->_x;
    ((LocYaw*)node)->_y = this->_y;
    ((LocYaw*)node)->_theta = this->_theta;
    ((LocYaw*)node)->_type = this->_type;

    return node;
}
void LocYaw::print(const std::string & node_name) const{
    printf("%s] x, y, theta: (%f, %f, %f), type: %d\n", 
            node_name.c_str(), _x, _y, _theta, _type);
}

void LocYaw::saveNode(const std::string & folder_name, const std::string & file_name) const {
    DVec<double> info(4 + 3); info.setZero();
    info[0] = _x;
    info[1] = _y;
    info[2] = _theta;
    info[3] = _type;

    if(_type != NodeType::root){
        info[4] = ((ForwardYaw*)_u)->_xdot;
        info[5] = ((ForwardYaw*)_u)->_ydot;
        info[6] = ((ForwardYaw*)_u)->_delta_theta;
    }
    saveVector(info, folder_name, file_name);
}

// *****************  Input **************************** //
ForwardYaw::ForwardYaw():Input(){
    // 0.035 rad -0.35 ~ 0.35 (~20 degree) 
    _dtheta = 0.035;
    _theta_min = -0.35;
    _size_input_set = 20;
}

void ForwardYaw::Integration(const Node* curr, const size_t & idx, Input * u, Node* next){
    if(idx > _size_input_set){
        printf("[Input] Warning: Index size is over. %lu/%lu\n", idx, _size_input_set);
        exit(0);
    }
    const LocYaw* curr_locyaw = (const LocYaw*)curr;
    LocYaw* next_locyaw = (LocYaw*)next;

    double delta_theta = _theta_min + _dtheta*((double)idx);
    next_locyaw->_x = curr_locyaw->_x + _fv*cos(curr_locyaw->_theta + delta_theta) * _dt;
    next_locyaw->_y = curr_locyaw->_y + _fv*sin(curr_locyaw->_theta + delta_theta) * _dt;
    next_locyaw->_theta = curr_locyaw->_theta + delta_theta;

    ((ForwardYaw*)u)->_xdot =_fv*cos(curr_locyaw->_theta + delta_theta) ;
    ((ForwardYaw*)u)->_ydot =_fv*sin(curr_locyaw->_theta + delta_theta) ;
    ((ForwardYaw*)u)->_delta_theta = delta_theta ;
}
