#include <iostream>

#include <RRT_LocYaw.hpp>
#include <Node.hpp>

class a{
    public:
        a(){ aa = 3; bb = 4; }
        ~a(){}

        double aa, bb;

        a* getnewA(){
            a* tmp = new a();
            tmp->aa = 7.;
            printf("tmp: %f, %f\n", tmp->aa, tmp->bb);
            return tmp;
        }

        void createA(a* & tmp){
            tmp = new a();
        }
};

int main(int argc, char ** argv){
    (void)argc;
    (void)argv;


    // TEST
    //a* test = new a();
    //a* test2 = NULL;
    //a* test3 = NULL;
    //test2 = test->getnewA();
    //test->createA(test3);
    // *****************************************// 
    
    LocYaw* ini = new LocYaw(0., 0., 0.);
    LocYaw* fin = new LocYaw(1.6, -1.4, -M_PI*0.95);
    
    RRT* rrt = new RRT_LocYaw(ini, fin);
    rrt->FindPath();

    return 0;
}
