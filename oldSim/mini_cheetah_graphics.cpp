#include "mini_cheetah_graphics.h"

using std::cout;

// class to generate mini cheetah graphics
// uses the parent window to get robot state
mini_cheetah_graphics::mini_cheetah_graphics(int use_cheetah_3)
{
    load_models(use_cheetah_3);
}

mini_cheetah_graphics::~mini_cheetah_graphics()
{
    free(u_link_colors);
    free(l_link_colors);
    free(ground_colors);
    free(body_colors);
    free(abad_colors);
}

// loads model from file
void mini_cheetah_graphics::load_models(int use_cheetah_3)
{


    if(!use_cheetah_3)
    {
        cout<<"[Mini Cheetah Graphics] Loading mini cheetah models...\n";
        // use objloaders to open files
        u_link_loader.load("../models/ulink2.obj");
        //u_link_loader.load("../models/u_link_mini.obj");
        u_link_vert_count = u_link_loader.getVertCount();
        l_link_loader.load("../models/l_link_mini.obj");
        l_link_vert_count = l_link_loader.getVertCount();
        ground_loader.load("../models/ground.obj");
        ground_vert_count = ground_loader.getVertCount();
        body_loader.load("../models/body.obj");
        body_vert_count = body_loader.getVertCount();
        abad_loader.load("../models/abad.obj");
        abad_vert_count = abad_loader.getVertCount();

        cout<<"[Mini Cheetah Graphics] Loaded "<<u_link_vert_count<<" (upper leg), "<<l_link_vert_count<<" (lower leg), \n";
        cout<<ground_vert_count<<" (floor), "<<body_vert_count<<" (body), "<<abad_vert_count<<" (abad) vertices.\n";

        // get vertices/normals
        u_link_vertices = u_link_loader.getPositions();
        u_link_normals = u_link_loader.getNormals();

        l_link_vertices = l_link_loader.getPositions();
        l_link_normals = l_link_loader.getNormals();

        ground_vertices = ground_loader.getPositions();
        ground_normals = ground_loader.getNormals();

        body_vertices = body_loader.getPositions();
        body_normals = body_loader.getNormals();

        abad_vertices = abad_loader.getPositions();
        abad_normals = abad_loader.getNormals();

        // objloaders don't do anything related to color, so we need to allocate these here
        u_link_colors = (float*)malloc(u_link_vert_count * sizeof(float)*3);
        l_link_colors = (float*)malloc(l_link_vert_count * sizeof(float)*3);
        ground_colors = (float*)malloc(ground_vert_count * sizeof(float) * 3);
        body_colors = (float*)malloc(body_vert_count * sizeof(float) * 3);
        abad_colors = (float*)malloc(abad_vert_count*sizeof(float) * 3);

        // solid colors for robot
        fill_with_boring_color(u_link_colors,u_link_vert_count*3,.3,.1,.1);
        fill_with_boring_color(l_link_colors,l_link_vert_count*3,.1,.3,.1);
        fill_with_boring_color(body_colors,body_vert_count*3,.0,.0,.5);
        fill_with_boring_color(abad_colors,abad_vert_count*3,.7,.2,.2);

        // checkerboard white/black
        fill_checkerboard_colors(ground_colors,ground_vert_count*3);

        /* JUNK*/
        for(int i = 0; i < 10; i++)
            model_matrices[i].setToIdentity();

        // initialize all the transformations
        // this is specific to mini-cheetah geometry
        // it also puts the robot is a reasonable default state
        // so the graphics don't look stupid before the simulator/robot starts running
        float abad_length= .19*2;
        float abad_width = .049*2;
        body_xform.setToIdentity();
        body_to_abad[0].translate(abad_length/2,-abad_width/2,0);
        body_to_abad[1].translate(abad_length/2,abad_width/2,0);
        body_to_abad[2].translate(-abad_length/2,-abad_width/2,0);
        body_to_abad[3].translate(-abad_length/2,abad_width/2,0);
        link_1_offset.setToIdentity();
        link_1_offset.rotate(-90,0,1,0);
        link_2_offset_p.rotate(180,0,1,0);
        link_2_offset_n.rotate(180,0,1,0);
        l1_xform.translate(0,0,-.209);
        l0_xform_p.translate(0,.062,0);
        l0_xform_n.translate(0,-.062,0);
        body_xform.translate(0,0,.35);

        link_0_offset_n.rotate(-90,0,0,1);
        link_0_offset_p.rotate(-90,0,0,1);
        link_0_offset_n.translate(0,-.0565,0);
        link_0_offset_p.translate(0,-.0565,0);
        link_0_offset_n.rotate(180,0,1,0);
        link_0_offset_p.rotate(0,0,1,0);

        link_0_offset_bn.rotate(90,0,0,1);
        link_0_offset_bp.rotate(90,0,0,1);
        link_0_offset_bn.translate(0,-.0565,0);
        link_0_offset_bp.translate(0,-.0565,0);
        link_0_offset_bn.rotate(0,0,1,0);
        link_0_offset_bp.rotate(180,0,1,0);

        body_offset.setToIdentity();

    }
    else
    {
        cout<<"[Mini Cheetah Graphics] Loading cheetah 3 models...\n";
        // use objloaders to open files
        u_link_loader.load("../models/u_link_c3.obj");
        u_link_vert_count = u_link_loader.getVertCount();
        l_link_loader.load("../models/l_link_c3.obj");
        l_link_vert_count = l_link_loader.getVertCount();
        ground_loader.load("../models/ground.obj");
        ground_vert_count = ground_loader.getVertCount();
        body_loader.load("../models/body_c3.obj");
        body_vert_count = body_loader.getVertCount();
        abad_loader.load("../models/abad.obj");
        abad_vert_count = abad_loader.getVertCount();

        cout<<"[Mini Cheetah Graphics] Loaded "<<u_link_vert_count<<" (upper leg), "<<l_link_vert_count<<" (lower leg), \n";
        cout<<ground_vert_count<<" (floor), "<<body_vert_count<<" (body), "<<abad_vert_count<<" (abad) vertices.\n";

        // get vertices/normals
        u_link_vertices = u_link_loader.getPositions();
        u_link_normals = u_link_loader.getNormals();

        l_link_vertices = l_link_loader.getPositions();
        l_link_normals = l_link_loader.getNormals();

        ground_vertices = ground_loader.getPositions();
        ground_normals = ground_loader.getNormals();

        body_vertices = body_loader.getPositions();
        body_normals = body_loader.getNormals();

        abad_vertices = abad_loader.getPositions();
        abad_normals = abad_loader.getNormals();

        // objloaders don't do anything related to color, so we need to allocate these here
        u_link_colors = (float*)malloc(u_link_vert_count * sizeof(float)*3);
        l_link_colors = (float*)malloc(l_link_vert_count * sizeof(float)*3);
        ground_colors = (float*)malloc(ground_vert_count * sizeof(float) * 3);
        body_colors = (float*)malloc(body_vert_count * sizeof(float) * 3);
        abad_colors = (float*)malloc(abad_vert_count*sizeof(float) * 3);

        // solid colors for robot
        fill_with_boring_color(u_link_colors,u_link_vert_count*3,.05,.05,.05);
        fill_with_boring_color(l_link_colors,l_link_vert_count*3,.05,.05,.05);
        fill_with_boring_color(body_colors,body_vert_count*3,.1,.1,.1);
        fill_with_boring_color(abad_colors,abad_vert_count*3,.3,.3,.2);

        // checkerboard white/black
        fill_checkerboard_colors(ground_colors,ground_vert_count*3);

        /* JUNK*/
        for(int i = 0; i < 10; i++)
            model_matrices[i].setToIdentity();

        body_offset.rotate(90,1,0,0);
        body_offset.rotate(90,0,0,1);
        // initialize all the transformations
        // this is specific to mini-cheetah geometry
        // it also puts the robot is a reasonable default state
        // so the graphics don't look stupid before the simulator/robot starts running
        float abad_length= .6;
        float abad_width = .256;
        body_xform.setToIdentity();
        body_to_abad[0].translate(abad_length/2,-abad_width/2,0);
        body_to_abad[1].translate(abad_length/2,abad_width/2,0);
        body_to_abad[2].translate(-abad_length/2,-abad_width/2,0);
        body_to_abad[3].translate(-abad_length/2,abad_width/2,0);
        link_1_offset.setToIdentity();
        link_1_offset.rotate(-180,0,1,0);
        link_2_offset_p.rotate(180,0,1,0);
        link_2_offset_n.rotate(180,0,1,0);
        l1_xform.translate(0,0,-.342);
        l0_xform_p.rotate(180,0,0,1);
        link_2_offset_p.translate(0,-.045,0);
        link_2_offset_n.translate(0,-.045,0);
        body_xform.translate(0,0,.35);

        link_0_offset_n.rotate(-90,0,0,1);
        link_0_offset_p.rotate(-90,0,0,1);
        link_0_offset_n.translate(0,-.0565,0);
        link_0_offset_p.translate(0,-.0565,0);
        link_0_offset_n.rotate(180,0,1,0);
        link_0_offset_p.rotate(0,0,1,0);

        link_0_offset_bn.rotate(90,0,0,1);
        link_0_offset_bp.rotate(90,0,0,1);
        link_0_offset_bn.translate(0,-.0565,0);
        link_0_offset_bp.translate(0,-.0565,0);
        link_0_offset_bn.rotate(0,0,1,0);
        link_0_offset_bp.rotate(180,0,1,0);

        using_cheetah_3 = true;

    }

    memset(&cs,0,sizeof(cs));

}

// fill with solid color
void mini_cheetah_graphics::fill_with_boring_color(float *data, int size, float r, float g, float b)
{
    if( (size%3) != 0)
    {
        cout<<"[Mini Cheetah Graphics Error] fill_with_boring_color should have a size divisble by 3.\n";
        return;
    }
    for(int i = 0; i < size/3; i++)
    {
        data[i*3 + 0] = r;
        data[i*3 + 1] = g;
        data[i*3 + 2] = b;
    }
}

// fill checkerboard
void mini_cheetah_graphics::fill_checkerboard_colors(float *data, int size)
{
       //data[i] = 0.f;

    for(int i = 0; i < size/36; i++)
    {
        for(int j = 0; j < 18; j++)
            data[i*36 + j] = 0.4f;
        for(int j = 18; j < 36; j++)
            data[i*36 + j] = 0.f;
    }
}

// number of bodies to render
int mini_cheetah_graphics::number_of_models()
{
    // 8 leg links
    //
    return 14;
}

void mini_cheetah_graphics::set_robot_state(CheetahState *state)
{
    memcpy(&cs, state,sizeof(cs));
    //printf("robot z height update: %.3f\n",cs.xfb[6]);
}

// updates all the transform matrices
void mini_cheetah_graphics::update()
{


    fc++;
    float rad2deg = 180.f/M_PI;
    for(int i = 0; i < 4; i++)
    {

        float side_sign = (i%2)?(using_cheetah_3?-1.f:1.f):1.f;
         r_abad[i].setToIdentity();
         r_abad[i].rotate(rad2deg * cs.q[i*3],1,0,0);
        // r_abad[i].rotate(20.f * sin(fc * .02f),1,0,0); //legs to the right

         r_hip[i].setToIdentity();
         //r_hip[i].rotate(30.f * sin(fc * .03f),0,1,0); //legs to the right
         r_hip[i].rotate(rad2deg * cs.q[i*3 + 1]*side_sign,0,-1,0);


         r_knee[i].setToIdentity();
         //r_knee[i].rotate(150.f * sin(fc * .04f),0,1,0); //legs to the right
         r_knee[i].rotate(rad2deg * cs.q[i*3 + 2]*side_sign,0,-1,0);
    }


    body_xform.setToIdentity();



    // if the simulator isn't running, main window returns null
//    if(cs == nullptr)
//        return;

    ground_x_scroll = cs.xfb[4];
    ground_y_scroll = cs.xfb[5];
    while(ground_x_scroll > 2.f) ground_x_scroll -=2.f;
    while(ground_x_scroll < -2.f) ground_x_scroll +=2.f;

    while(ground_y_scroll > 2.f) ground_y_scroll -=2.f;
    while(ground_y_scroll < -2.f) ground_y_scroll +=2.f;
    body_xform.translate(0.f*cs.xfb[4],0.f*cs.xfb[5],cs.xfb[6]);
    QQuaternion quat(cs.xfb[0],cs.xfb[1],cs.xfb[2],cs.xfb[3]);
    body_xform.rotate(quat);

    ground_scroll.setToIdentity();
    ground_scroll.translate(-ground_x_scroll,-ground_y_scroll,0.f);

    //body_xform = body_xform.inverted();

//    body_xform.setToIdentity();
//    body_xform.translate(0,0,.35);
//    body_xform.rotate(15.f * sin(fc*.02f),1,0,0);
//    body_xform.rotate(15.f * sin(fc*.03f),0,1,0);
//    body_xform.rotate(10.f * sin(fc*.04f),0,0,1);

    for(int i = 0; i < 10; i++)
        model_matrices[i].setToIdentity();

}

// gets model to world matrix
// other parts of the code will do the rotate-world-for-orbit and camera-projection matrices
QMatrix4x4 mini_cheetah_graphics::get_model_xform(int model)
{
    QMatrix4x4 l0x = ((model%2)?l0_xform_p:l0_xform_n);
    QMatrix4x4 l2x = ((model%2)?link_2_offset_p:link_2_offset_n);
    if(model >= number_of_models())
    {
        cout<<"[Mini Cheetah Graphics Error] get_model_xform model number too big: "<<model;
        cout<<" should be less than "<<number_of_models()<<"\n";
        return body_xform ;
    }

    // Lower legs
    if(model >= 0 && model <= 3)
    {
        return  body_xform * body_to_abad[model]  * r_abad[model] * l0x * r_hip[model]* l1_xform  * r_knee[model] * l2x;
    }
    //upper legs
    else if(model >= 4 && model <= 7)
    {
        return  body_xform * body_to_abad[model - 4] *  r_abad[model - 4] * l0x   * r_hip[model - 4] * link_1_offset;
    }
    // ground
    else if(model == 8)
    {
        return ground_scroll;
    }
    // body
    else if(model == 9)
    {
        return body_xform* body_offset;
    }
    // ab ad (front)
    else if(model >= 10 && model <= 11)
    {
        return body_xform * body_to_abad[model - 10] * r_abad[model - 10] * ((model%2)?link_0_offset_p:link_0_offset_n);
    }
    // ab ad (back)
    else if(model >= 12 && model <= 13)
    {
        return body_xform * body_to_abad[model - 10] * r_abad[model - 10] * ((model%2)?link_0_offset_bp:link_0_offset_bn);
    }
}

int mini_cheetah_graphics::get_number_of_faces(int model)
{
    //lower legs
    if(model >= 0 && model <= 3)
        return l_link_vert_count/3;
    if(model >=4 && model <= 7)
        return u_link_vert_count/3;
    if(model == 8)
        return ground_vert_count/3;
    if(model == 9)
        return body_vert_count/3;
    if(model >= 10 && model <= 13)
        return abad_vert_count/3;

    cout<<"[Mini Cheetah Graphics Error] get_number_of_faces for unknown model: "<<model<<"\n";
}

float* mini_cheetah_graphics::get_vertices(int model)
{
    //lower legs
    if(model >= 0 && model <= 3)
        return l_link_vertices;
    if(model >=4 && model <= 7)
        return u_link_vertices;
    if(model == 8)
        return ground_vertices;
    if(model == 9)
        return body_vertices;
    if(model >= 10 && model <= 13)
        return abad_vertices;
    cout<<"[Mini Cheetah Graphics Error] get_number_of_vertices for unknown model: "<<model<<"\n";
}

float* mini_cheetah_graphics::get_normals(int model)
{
    //lower legs
    if(model >= 0 && model <= 3)
        return l_link_normals;
    if(model >=4 && model <= 7)
        return u_link_normals;
    if(model == 8)
        return ground_normals;
    if(model == 9)
        return body_normals;
    if(model >= 10 && model <= 13)
        return abad_normals;

    cout<<"[Mini Cheetah Graphics Error] get_number_of_normals for unknown model: "<<model<<"\n";
}

float* mini_cheetah_graphics::get_colors(int model)
{
    //lower legs
    if(model >= 0 && model <= 3)
        return l_link_colors;
    if(model >=4 && model <= 7)
        return u_link_colors;
    if(model == 8)
        return ground_colors;
    if(model == 9)
        return body_colors;
    if(model >= 10 && model <= 13)
        return abad_colors;

    cout<<"[Mini Cheetah Graphics Error] get_number_of_colors for unknown model: "<<model<<"\n";
}
