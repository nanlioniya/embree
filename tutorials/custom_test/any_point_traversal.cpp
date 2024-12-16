/*
    this is for converting ann search into embree bvh search
    GOAL: 
        1. form scene from dataset
        2. traverse from any given point
    
    current task:
        1. play with args from RTCIntersectArguments
    
    thought:
        1. maintain a table with specific nodeID and node pointer for traversal
            1.1 achieve store and retrieve data inside bvh
    today goal:
        1. understand bvh_access and retrieve data
*/

// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/tutorial_device.h"
#include "../../include/embree4/rtcore.h"
RTC_NAMESPACE_USE
#include "../../kernels/bvh/bvh.h"
#include "../../kernels/geometry/trianglev.h"

namespace embree
{
  /* error reporting function */
  void error_handler(void* userPtr, const RTCError code, const char* str)
  {
    if (code == RTC_ERROR_NONE) 
      return;
    
    printf("Embree: ");
    switch (code) {
    case RTC_ERROR_UNKNOWN          : printf("RTC_ERROR_UNKNOWN"); break;
    case RTC_ERROR_INVALID_ARGUMENT : printf("RTC_ERROR_INVALID_ARGUMENT"); break;
    case RTC_ERROR_INVALID_OPERATION: printf("RTC_ERROR_INVALID_OPERATION"); break;
    case RTC_ERROR_OUT_OF_MEMORY    : printf("RTC_ERROR_OUT_OF_MEMORY"); break;
    case RTC_ERROR_UNSUPPORTED_CPU  : printf("RTC_ERROR_UNSUPPORTED_CPU"); break;
    case RTC_ERROR_CANCELLED        : printf("RTC_ERROR_CANCELLED"); break;
    default                         : printf("invalid error code"); break;
    }
    if (str) { 
      printf(" ("); 
      while (*str) putchar(*str++); 
      printf(")\n"); 
    }
    exit(1);
  }

  /* adds a cube to the scene */
  unsigned int addCube (RTCDevice device_i, RTCScene scene_i, const Vec3fa& pos)
  {
    /* create a triangulated cube with 12 triangles and 8 vertices */
    RTCGeometry mesh = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_TRIANGLE);
    
    /* set vertices */
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), 8); 
    vertices[0].x = pos.x + -1; vertices[0].y = pos.y + -1; vertices[0].z = pos.z + -1; 
    vertices[1].x = pos.x + -1; vertices[1].y = pos.y + -1; vertices[1].z = pos.z + +1; 
    vertices[2].x = pos.x + -1; vertices[2].y = pos.y + +1; vertices[2].z = pos.z + -1; 
    vertices[3].x = pos.x + -1; vertices[3].y = pos.y + +1; vertices[3].z = pos.z + +1; 
    vertices[4].x = pos.x + +1; vertices[4].y = pos.y + -1; vertices[4].z = pos.z + -1; 
    vertices[5].x = pos.x + +1; vertices[5].y = pos.y + -1; vertices[5].z = pos.z + +1; 
    vertices[6].x = pos.x + +1; vertices[6].y = pos.y + +1; vertices[6].z = pos.z + -1; 
    vertices[7].x = pos.x + +1; vertices[7].y = pos.y + +1; vertices[7].z = pos.z + +1; 
    
    /* set triangles */
    int tri = 0;
    Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 12);
    
    // left side
    triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 1; tri++;
    triangles[tri].v0 = 1; triangles[tri].v1 = 2; triangles[tri].v2 = 3; tri++;
    
    // right side
    triangles[tri].v0 = 4; triangles[tri].v1 = 5; triangles[tri].v2 = 6; tri++;
    triangles[tri].v0 = 5; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;
    
    // bottom side
    triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 4; tri++;
    triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 4; tri++;
    
    // top side
    triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 3; tri++;
    triangles[tri].v0 = 3; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;
    
    // front side
    triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 2; tri++;
    triangles[tri].v0 = 2; triangles[tri].v1 = 4; triangles[tri].v2 = 6; tri++;
    
    // back side
    triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 5; tri++;
    triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 5; tri++;

    rtcCommitGeometry(mesh);
    unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
    rtcReleaseGeometry(mesh);
    return geomID;
  }

  /* prints the bvh4.triangle4v data structure */
  void print_bvh4_triangle4v(BVH4::NodeRef node, size_t depth)
  {
    if (node.isAABBNode())
    {
      BVH4::AABBNode* n = node.getAABBNode();
      std::cout << n->num << std::endl;
      std::cout << "AABBNode {" << std::endl;
      for (size_t i=0; i<4; i++)
      {
        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  bounds" << i << " = " << n->bounds(i) << std::endl;
      }

      for (size_t i=0; i<4; i++)
      {
        if (n->child(i) == BVH4::emptyNode)
          continue;

        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  child" << i << " = ";
        print_bvh4_triangle4v(n->child(i),depth+1); 
      }
      for (size_t k=0; k<depth; k++) std::cout << "  ";
      std::cout << "}" << std::endl;
    }
    else
    {
      size_t num; 
      const Triangle4v* tri = (const Triangle4v*) node.leaf(num);

      std::cout << "Leaf {" << std::endl;
      for (size_t i=0; i<num; i++) {
        for (size_t j=0; j<tri[i].size(); j++) {
          for (size_t k=0; k<depth; k++) std::cout << "  ";
          std::cout << "  Triangle { v0 = (" << tri[i].v0.x[j] << ", " << tri[i].v0.y[j] << ", " << tri[i].v0.z[j] << "),  "
            "v1 = (" << tri[i].v1.x[j] << ", " << tri[i].v1.y[j] << ", " << tri[i].v1.z[j] << "), "
            "v2 = (" << tri[i].v2.x[j] << ", " << tri[i].v2.y[j] << ", " << tri[i].v2.z[j] << "), "
            "geomID = " << tri[i].geomID(j) << ", primID = " << tri[i].primID(j) << " }" << std::endl;
        }
      }
      for (size_t k=0; k<depth; k++) std::cout << "  ";
      std::cout << "}" << std::endl;
    }
  }

  /* prints the triangle BVH of a scene */
  void print_bvh(RTCScene scene)
  {
    BVH4* bvh4 = nullptr; 

    /* if the scene contains only triangles, the BVH4 acceleration structure can be obtained this way */
    AccelData* accel = ((Accel*)scene)->intersectors.ptr;
    if (accel->type == AccelData::TY_BVH4) 
      bvh4 = (BVH4*)accel;

    /* if there are also other geometry types, one has to iterate over the toplevel AccelN structure */
    else if (accel->type == AccelData::TY_ACCELN)
    {
      AccelN* accelN = (AccelN*)(accel);
      for (size_t i=0; i<accelN->accels.size(); i++) {
        if (accelN->accels[i]->intersectors.ptr->type == AccelData::TY_BVH4) {
          bvh4 = (BVH4*)accelN->accels[i]->intersectors.ptr;
          if (std::string(bvh4->primTy->name()) == "triangle4v") break;
          bvh4 = nullptr;
        }
      }
    }
    if (bvh4 == nullptr)
      throw std::runtime_error("cannot access BVH4 acceleration structure"); // will not happen if you use this Embree version
      
    /* now lets print the entire hierarchy */
    print_bvh4_triangle4v(bvh4->root,0);
  }

  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    /* create new Embree device and force bvh4.triangle4v hierarchy for triangles */
    RTCDevice device = rtcNewDevice("tri_accel=bvh4.triangle4v");
    error_handler(nullptr,rtcGetDeviceError(device));
    
    /* set error handler */
    rtcSetDeviceErrorFunction(device,error_handler,nullptr);
    
    /* create scene */
    RTCScene scene = rtcNewScene(device);
    addCube(device,scene,Vec3fa(-1,0,0));
    addCube(device,scene,Vec3fa(1,0,0));
    addCube(device,scene,Vec3fa(0,0,-1));
    addCube(device,scene,Vec3fa(0,0,1));
    rtcCommitScene (scene);
    /* print triangle BVH */
    print_bvh(scene);

    /* cleanup */
    rtcReleaseScene (scene);
    rtcReleaseDevice(device);
    
    return 0;
  }
}

int main(int argc, char** argv)
{
  try {
    return embree::main(argc, argv);
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return 1;
  }
  catch (...) {
    std::cout << "Error: unknown exception caught." << std::endl;
    return 1;
  }
}
