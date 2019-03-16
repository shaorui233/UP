/*! @file obj_loader.h
 *  @brief Utility to load .obj files, containing 3D models of robots.
 *
 *  This utility is based off of a library found on Github.  Unfortunately, the library had some bugs and was
 *  modified.  The original library can no longer be found online. This code is not very good.
 */

#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <vector>


// object file loader
// taken from the internet and modified because it didn't work

namespace objLoader {
    struct Vec3 {
        float x;
        float y;
        float z;

        Vec3() : x(0), y(0), z(0) {}

        Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

        float &operator[](int idx);
    };

    struct TexCoord {
        float u;
        float v;

        TexCoord() : u(0), v(0) {}

        TexCoord(float _u, float _v) : u(_u), v(_v) {}

        float &operator[](int idx);
    };

    struct Face {
        int v_a;
        int v_b;
        int v_c;
        int vn_a;
        int vn_b;
        int vn_c;
    };

    class SimObjectLoader {
        public:
            SimObjectLoader();

            // load obj file (this method does the heavy-lifting)
            void load(const char *filename, std::vector<float>& positions, std::vector<float>& normals);
            int getIndexCount();
            int getVertCount();
            const unsigned int *getFaces();
            float *getPositions();
            float *getNormals();
            int getTexCoordLayers();
            const float *getTexCoords(int multiTexCoordLayer);

        private:

            std::vector<Face> Faces;
            std::vector<Vec3> Positions;
            std::vector<Vec3> Normals;

            // these were added because the original positions/normals were wrong
            // this method does double up on the storage of model data, but the biggest models
            // are only a few Mb, so it doesn't matter
            float *positions_jdc;
            float *normals_jdc;

            // obj's only have 1 layer ever
            std::vector<TexCoord> TexCoords;
            unsigned int TexCoordLayers;
    };
}

#endif // OBJLOADER_H
