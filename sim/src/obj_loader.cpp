/*! @file obj_loader.cpp
 *  @brief Utility to load .obj files, containing 3D models of robots.
 *
 *  This utility is based off of a library found on Github.  Unfortunately, the library had some bugs and was
 *  modified.  The original library can no longer be found online. This code is not very good.
 */

#include "obj_loader.h"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cstring>
#include <assert.h>
#include <map>

using namespace std;
using namespace objLoader;

float &Vec3::operator[](int idx) {
  assert(idx > 0);
  assert(idx < 3);
  return *(&x + idx);
}

float &TexCoord::operator[](int idx) {
  assert(idx > 0);
  assert(idx < 3);
  return *(&u + idx);
}

struct FaceVert {
  FaceVert() : vert(-1), norm(-1), coord(-1) {}

  int vert;
  int norm;
  int coord;
};

struct vert_less {
  bool operator()(const FaceVert &lhs, const FaceVert &rhs) const {
    // handle any size mesh
    if (lhs.vert != rhs.vert) return (lhs.vert < rhs.vert);
    if (lhs.norm != rhs.norm) return (lhs.norm < rhs.norm);
    if (lhs.coord != rhs.coord) return (lhs.coord < rhs.coord);
    return false;
  }
};

ObjLoader::ObjLoader()
        : TexCoordLayers(1) {

}

void ObjLoader::load(const char *filename, std::vector<float> &positions, std::vector<float> &normals) {
  ifstream inf;
  inf.open(filename, ios_base::in);
  if (!inf.is_open()) {
    cerr << "[!] Failed to load file: " << filename << endl;
  }

  Positions.clear();
  Normals.clear();
  TexCoords.clear();
  Faces.clear();

  const char *delims = " \n\r";
  const unsigned int CHARACTER_COUNT = 500;
  char line[CHARACTER_COUNT] = {0};

  std::vector<Vec3> verts;
  std::vector<Vec3> norms;
  std::vector<TexCoord> texcoords;

  std::map<FaceVert, int, vert_less> uniqueverts;
  unsigned int vert_count = 0;

  while (inf.good()) {
    memset((void *) line, 0, CHARACTER_COUNT);
    inf.getline(line, CHARACTER_COUNT);
    if (inf.eof()) break;

    char *token = strtok(line, delims);
    if (token == NULL || token[0] == '#' || token[0] == '$')
      continue;

    // verts look like:
    //	v float float float
    if (strcmp(token, "v") == 0) {
      float x = 0, y = 0, z = 0, w = 1;
      sscanf(line + 2, "%f %f %f %f", &x, &y, &z, &w);
      verts.push_back(Vec3(x / w, y / w, z / w));
    }
      // normals:
      // 	nv float float float
    else if (strcmp(token, "vn") == 0) {
      float x = 0, y = 0, z = 0;
      sscanf(line + 3, "%f %f %f", &x, &y, &z);
      norms.push_back(Vec3(x, y, z));
    }
      // texcoords:
      //	vt	float float
    else if (strcmp(token, "vt") == 0) {
      float x = 0, y = 0, z = 0;
      sscanf(line + 3, "%f %f %f", &x, &y, &z);
      texcoords.push_back(TexCoord(x, y));
    }

      // keep track of smoothing groups
      // s [number|off]
    else if (strcmp(token, "s") == 0) {

    }

      // faces start with:
      //	f
      // Note: I rewrote this part because it didn't work
    else if (strcmp(token, "f") == 0) {

      std::vector<int> vindices;
      std::vector<int> nindices;
      std::vector<int> tindices;

      char *lineptr = line + 2;
      Face face;
      //while (lineptr[0] != 0) {
      while (lineptr[0] == ' ') ++lineptr;


      int vi = 0, ni = 0, ti = 0;
      if (sscanf(lineptr, "%d/%d/%d", &vi, &ni, &ti) == 3) {
        //cout<<"face line read: "<<vi<<"\n";
        face.v_a = vi - 1;
        face.vn_a = ni - 1;
      } else if (sscanf(lineptr, "%d//%d", &vi, &ni) == 2) {
        //cout<<"face line read: "<<vi<<"\n";
        face.v_a = vi - 1;
        face.vn_a = ni - 1;
      }

      while (lineptr[0] != ' ' && lineptr[0] != 0) ++lineptr;
      while (lineptr[0] == ' ') ++lineptr;
      if (sscanf(lineptr, "%d/%d/%d", &vi, &ni, &ti) == 3) {
        //cout<<"face line read2: "<<vi<<"\n";
        face.v_b = vi - 1;
        face.vn_b = ni - 1;
      } else if (sscanf(lineptr, "%d//%d", &vi, &ni) == 2) {

        face.v_b = vi - 1;
        face.vn_b = ni - 1;
      }
      while (lineptr[0] != ' ' && lineptr[0] != 0) ++lineptr;
      while (lineptr[0] == ' ') ++lineptr;
      if (sscanf(lineptr, "%d/%d/%d", &vi, &ni, &ti) == 3) {
        face.v_c = vi - 1;
        face.vn_c = ni - 1;
      } else if (sscanf(lineptr, "%d//%d", &vi, &ni) == 2) {

        face.v_c = vi - 1;
        face.vn_c = ni - 1;
      }

      while (lineptr[0] != ' ' && lineptr[0] != 0) ++lineptr;
      //}
      Faces.push_back(face);


    }
  }
  inf.close();

  // use resize instead of reserve because we'll be indexing in random locations.
  Positions.resize(Faces.size() * 3);
  if (norms.size() > 0)
    Normals.resize(Faces.size() * 3);
  if (texcoords.size() > 0)
    TexCoords.resize(vert_count);


  for (unsigned int i = 0; i < Faces.size(); i++) {
    Positions[i * 3 + 0] = verts[Faces[i].v_a];
    Positions[i * 3 + 1] = verts[Faces[i].v_b];
    Positions[i * 3 + 2] = verts[Faces[i].v_c];
    Normals[i * 3 + 0] = norms[Faces[i].vn_a];
    Normals[i * 3 + 1] = norms[Faces[i].vn_b];
    Normals[i * 3 + 2] = norms[Faces[i].vn_c];

  }

  positions.resize(Positions.size() * 3);
  normals.resize(Normals.size() * 3);

  for (unsigned int i = 0; i < Positions.size(); i++) {
    positions[i * 3 + 0] = Positions[i].x;
    positions[i * 3 + 1] = Positions[i].y;
    positions[i * 3 + 2] = Positions[i].z;
    normals[i * 3 + 0] = Normals[i].x;
    normals[i * 3 + 1] = Normals[i].y;
    normals[i * 3 + 2] = Normals[i].z;
  }

  printf("[obj_loader] loaded %s (%lu, %lu)\n", filename, positions.size(), normals.size());
}

int ObjLoader::getIndexCount() {
  return (int) Faces.size() * 3;
}

int ObjLoader::getVertCount() {
  return (int) Positions.size();
}

const unsigned int *ObjLoader::getFaces() {
  return (const unsigned int *) &Faces[0];
}

float *ObjLoader::getPositions() {
  return positions_jdc;
}

float *ObjLoader::getNormals() {
  return normals_jdc;
}

int ObjLoader::getTexCoordLayers() {
  return TexCoordLayers;
}

const float *ObjLoader::getTexCoords(int multiTexCoordLayer) {
  (void) (multiTexCoordLayer);
  return (const float *) &TexCoords[0];
}
