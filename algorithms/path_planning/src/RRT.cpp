#include <RRT.hpp>
#include <Node.hpp>
#include <save_file.hpp>


RRT::RRT(const Node* ini, const Node* fin){
    Node* ini_tmp = ini->_MakeAndCopyNode();
    ini_tmp->_type = NodeType::root;
    _tree.push_back(ini_tmp);

    _fin = fin->_MakeAndCopyNode();
    _fin->_type = NodeType::goal;
}

RRT::~RRT(){
    std::vector<Node*>::iterator iter = _tree.begin();
    while(iter != _tree.end()){
        delete (*iter);
    }
    _tree.clear();
}

bool RRT::FindPath(){
    size_t iter(0);
    while(true){
        //printf("1\n");
        Node* node = NULL;
        // Create Node
        _CreateTestNode(node);
        //printf("2\n");
        // Extend Tree
        int ret = _ExtendTree(node);
        //printf("3\n");

        if(ret == ExtResult::Reached && node->IsGoal()){
            printf("[RRT] Found Path\n");
        //printf("4\n");
            _save_path();
        //printf("5\n");
            _save_result();
        //printf("6\n");
            return true;
        }
        delete node;

        if(iter > 70000){
            printf("[RRT] iteration reaches maximum: %lu\n", iter);
            _save_path();
            _save_result();
            return false;
        }
        if(iter%10000 == 1){
            _print_summary(iter);
        }
        ++iter;
    }
    return false;
}
void RRT::_save_path(){
    printf("[RRT] Save Path\n");
    Node* x_node = _FindNearestNeighbor(_fin);

    x_node->saveNode("final_path");
    while(x_node->_type != NodeType::root){
        x_node = x_node->_parent;
        x_node->saveNode("final_path");
    }
}
void RRT::_save_result(){

    printf("[RRT] Save Result\n");
    std::vector<Node*> end_node_list;

    std::vector<Node*>::iterator iter = _tree.begin();
    while(iter != _tree.end()){
        if ( (*iter)->_type == NodeType::end){
            end_node_list.push_back(*iter);
        }
        ++ iter;
    }
 
    Node* node;
    for(size_t i(0); i<end_node_list.size(); ++i){
        node = end_node_list[i];
        node->saveNode("planning_result");
        while(node->_type != NodeType::root){
            node = node->_parent;
            node->saveNode("planning_result");
        }
    }

}
void RRT::_print_summary(const size_t & iter){

    Node* x_near = _FindNearestNeighbor(_fin);
    double dist = x_near->Dist(_fin);
    _tree[0]->print("Start Node");
    _fin->print("Goal Node");
    x_near->print("Nearest Node");
    printf("%lu th Summary) size of tree: %lu, shortest dist from goal: %f \n", 
            iter, _tree.size(), dist);
    printf("\n");
}

int RRT::_ExtendTree(Node* test_node){
    Node* x_near = _FindNearestNeighbor(test_node);
    //printf("size of tree:%lu\n", _tree.size());

    Node* x_new;
    if(_NewState(test_node, x_near, x_new) ){
        //test_node->print("test node");
        //x_near->print("nearest node");
        //x_new->print("new node");

        _tree.push_back(x_new);
        if(test_node->CloseEnough(x_new) ){
            return ExtResult::Reached;
        }else{
            return ExtResult::Advanced;
        }
    }else {
        return ExtResult::Trapped;
    }
}

Node* RRT::_FindNearestNeighbor(const Node* node){
    std::vector<Node*>::iterator iter = _tree.begin();
    double dist;
    double min_dist(1.e15);

    Node* nearest_node = NULL;
    while(iter != _tree.end()){
        //(*iter)->print("search");
        dist = (*iter)->Dist(node);
        if( dist < min_dist){
            min_dist = dist;
            nearest_node = (*iter);
            //nearest_node->print("near");
        }

        ++ iter;
    }
    return nearest_node;
}
