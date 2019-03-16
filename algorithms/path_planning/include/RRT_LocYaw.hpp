#ifndef RRT_LOCATION_YAW
#define RRT_LOCATION_YAW

#include <RRT.hpp>
#include <cppTypes.h>
#include <Node.hpp>

class RRT_LocYaw: public RRT{
    public:
        RRT_LocYaw(const Node* ini, const Node* fin);
        virtual ~RRT_LocYaw();

        // Select Node (can be randomly generated or occationally choose goal)
        virtual void _CreateTestNode(Node* & node);
        // Create new node and test whether it is valide (obstacle free)
        virtual bool _NewState(const Node* test_node, Node* node_near, Node* & new_node);

    protected:
        size_t _count = 0;
        size_t _goal_bias_frequency = 20;
};


// 2 Dim Location & yaw
class LocYaw: public Node{
    public:
        LocYaw():Node(){ // Random Node
            _x = (_x_max - _x_min)*(double)rand()/RAND_MAX + _x_min;
            _y = (_y_max - _y_min)*(double)rand()/RAND_MAX + _y_min;
            _theta = (_theta_max - _theta_min)*(double)rand()/RAND_MAX + _theta_min;
        }
        LocYaw(const double & x, const double & y, const double & theta):
            Node(),_x(x), _y(y), _theta(theta){
        }
        virtual ~LocYaw(){}

        // Virtual Functions
        virtual Node* _MakeAndCopyNode() const;
        virtual double Dist(const Node*) const ;
        virtual bool CloseEnough(const Node* node) const;
        virtual void print(const std::string & node_name) const;
        virtual void saveNode(const std::string & file_name) const;

        double _x, _y, _theta;

        double _x_min = -1;
        double _x_max = 3.;
        double _y_min = -2.5;
        double _y_max = 1.;
        double _theta_min = -M_PI;
        double _theta_max = M_PI;
};

class ForwardYaw: public Input{
    public:
        ForwardYaw();
        virtual ~ForwardYaw(){}

        // Output Node and Input must be allocated before this function call
        virtual void Integration(
                const Node* curr, const size_t & idx, Input* u, Node* next);

        double _xdot;
        double _ydot;
        double _delta_theta;
    protected:
        double _theta_min;
        double _dtheta; 
        double _dt = 0.2;
        double _fv = 0.5;
 
};


#endif
