/*
    this is for converting ann search into embree bvh search
*/

// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
#include "../../kernels/bvh/bvh.h"
#include "../common/tutorial/tutorial_device.h"
#include "../../include/embree4/rtcore.h"
#include "../../kernels/geometry/trianglev.h"
#include <limits>
#include <iostream>

void errorFunction(void* userPtr, RTCError error, const char* str)
{
    std::cout << "Embree Error [" << error << "]: " << str << std::endl;
}

/* adds a cube to the scene */
unsigned int addCube (RTCDevice device_i, RTCScene scene_i, const embree::Vec3fa& pos)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_TRIANGLE);
  
  /* set vertices */
  embree::Vec3fa* vertices = (embree::Vec3fa*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(embree::Vec3fa), 8); 
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

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCDevice device_i, RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_TRIANGLE);
  
  /* set vertices */
  embree::Vec3fa* vertices = (embree::Vec3fa*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(embree::Vec3fa), 4); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  
  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a hair to the scene */
unsigned int addHair(RTCDevice device_i, RTCScene scene_i)
{
  RTCGeometry geom = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);

  embree::vfloat4* pos = (embree::vfloat4*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(embree::vfloat4), 4);
  pos[0] = embree::vfloat4(0.0f,0.0f,0.0f,0.1f);
  pos[1] = embree::vfloat4(0.0f,1.0f,0.0f,0.1f);
  pos[2] = embree::vfloat4(0.0f,2.0f,0.0f,0.1f);
  pos[3] = embree::vfloat4(0.0f,3.0f,0.0f,0.1f);

  int* index = (int*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(int), 1);
  index[0] = 0;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

void print_bvh4_triangle4v(embree::BVH4::NodeRef node, size_t depth)
{
    for (size_t k=0; k<depth; k++) std::cout << "  ";
    std::cout << "Node @" << node.ptr << " (";
    std::cout << (node.isAABBNode() ? "AABBNode" : "Leaf") << ") {" << std::endl;

    if (node.isAABBNode())
    {
        embree::BVH4::AABBNode* n = node.getAABBNode();
        
        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  NodeType: Internal" << std::endl;
        
        for (size_t i=0; i<4; i++)
        {
            for (size_t k=0; k<depth; k++) std::cout << "  ";
            std::cout << "  bounds" << i << " = " << n->bounds(i);

            embree::BBox3fa bounds = n->bounds(i);
            embree::Vec3fa size = bounds.size();
            std::cout << " (size = " << size << ")" << std::endl;
        }

        for (size_t i=0; i<4; i++)
        {
            if (n->child(i) == embree::BVH4::emptyNode) {
                for (size_t k=0; k<depth; k++) std::cout << "  ";
                std::cout << "  child" << i << " = Empty" << std::endl;
                continue;
            }

            for (size_t k=0; k<depth; k++) std::cout << "  ";
            std::cout << "  child" << i << " = ";
            print_bvh4_triangle4v(n->child(i), depth+1);
        }
    }
    else
    {
        size_t num;
        const embree::Triangle4v* tri = (const embree::Triangle4v*) node.leaf(num);
        
        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  NodeType: Leaf" << std::endl;
        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  TriangleCount: " << num << std::endl;

        for (size_t i=0; i<num; i++) {
            for (size_t j=0; j<tri[i].size(); j++) {
                for (size_t k=0; k<depth; k++) std::cout << "  ";
                std::cout << "  Triangle[" << i << "," << j << "] { ";
                std::cout << "v0 = (" << tri[i].v0.x[j] << ", " << tri[i].v0.y[j] << ", " << tri[i].v0.z[j] << "), ";
                std::cout << "v1 = (" << tri[i].v1.x[j] << ", " << tri[i].v1.y[j] << ", " << tri[i].v1.z[j] << "), ";
                std::cout << "v2 = (" << tri[i].v2.x[j] << ", " << tri[i].v2.y[j] << ", " << tri[i].v2.z[j] << "), ";
                std::cout << "geomID = " << tri[i].geomID(j) << ", primID = " << tri[i].primID(j) << " }" << std::endl;
            }
        }
    }
    
    for (size_t k=0; k<depth; k++) std::cout << "  ";
    std::cout << "}" << std::endl;
}

/* prints the triangle BVH of a scene */
void print_bvh(RTCScene scene)
{
  embree::BVH4* bvh4 = nullptr; 

  /* if the scene contains only triangles, the BVH4 acceleration structure can be obtained this way */
  embree::AccelData* accel = ((embree::Accel*)scene)->intersectors.ptr;
  if (accel->type == embree::AccelData::TY_BVH4) 
    bvh4 = (embree::BVH4*)accel;

  /* if there are also other geometry types, one has to iterate over the toplevel AccelN structure */
  else if (accel->type == embree::AccelData::TY_ACCELN)
  {
    embree::AccelN* accelN = (embree::AccelN*)(accel);
    for (size_t i=0; i<accelN->accels.size(); i++) {
      if (accelN->accels[i]->intersectors.ptr->type == embree::AccelData::TY_BVH4) {
        bvh4 = (embree::BVH4*)accelN->accels[i]->intersectors.ptr;
        if (std::string(bvh4->primTy->name()) == "triangle4v") break;
        bvh4 = nullptr;
      }
    }
  }
  if (bvh4 == nullptr)
    throw std::runtime_error("cannot access BVH4 acceleration structure"); // will not happen if you use this Embree version
    
  /* now lets print the entire hierarchy */
  // print_bvh4_triangle4v(bvh4->root.getAABBNode()->child(0).ptr,0);
  print_bvh4_triangle4v(bvh4->root,0);
}


int main()
{
    // set verbose=2 for explicit output
    // RTCDevice device = rtcNewDevice("verbose=2,threads=1");
    RTCDevice device = rtcNewDevice("tri_accel=bvh4.triangle4v");

    rtcSetDeviceErrorFunction(device, errorFunction, nullptr);
    
    /* create scene */
    RTCScene scene = rtcNewScene(device);
    addCube(device,scene,embree::Vec3fa(-1,0,0));
    addCube(device,scene,embree::Vec3fa(1,0,0));
    addCube(device,scene,embree::Vec3fa(0,0,-1));
    addCube(device,scene,embree::Vec3fa(0,0,1));
    
    // addHair(device,scene);
    // addGroundPlane(device,scene);
    
    std::cout << "Starting scene commit..." << std::endl;
    rtcCommitScene(scene);
    std::cout << "Scene commit completed." << std::endl;
    
    /* print triangle BVH */
    print_bvh(scene);

    RTCRayHit rayhit;
    rayhit.ray.org_x = -1.f; rayhit.ray.org_y = 0.f; rayhit.ray.org_z = 10.f;
    rayhit.ray.dir_x = 0.f; rayhit.ray.dir_y = 0.f; rayhit.ray.dir_z = -1.f;
    rayhit.ray.tnear = 0.f;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;
    rayhit.ray.time = 0.f;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;

    embree::BVH4* bvh4 = nullptr;
    embree::AccelData* accel = ((embree::Accel*)scene)->intersectors.ptr;
    if (accel->type == embree::AccelData::TY_BVH4) 
    {
      std::cout << "one geo\n";
      bvh4 = (embree::BVH4*)accel;
    }
    /* if there are also other geometry types, one has to iterate over the toplevel AccelN structure */
    else if (accel->type == embree::AccelData::TY_ACCELN)
    {
      std::cout << "more than one geo\n";
      embree::AccelN* accelN = (embree::AccelN*)(accel);
      for (size_t i=0; i<accelN->accels.size(); i++) {
        if (accelN->accels[i]->intersectors.ptr->type == embree::AccelData::TY_BVH4) {
          bvh4 = (embree::BVH4*)accelN->accels[i]->intersectors.ptr;
          if (std::string(bvh4->primTy->name()) == "triangle4v") break;
          bvh4 = nullptr;
        }
      }
    }
    if (bvh4 == nullptr)
      throw std::runtime_error("cannot access BVH4 acceleration structure"); // will not happen if you use this Embree version


    int childIndex;
    std::cout << "Enter child index: ";
    std::cin >> childIndex;

    RTCIntersectArguments args;
    // rtcInitIntersectArguments(&args);
    rtcInitIntersectArgumentsWithID(&args, 38, bvh4->root.getAABBNode()->child(childIndex).ptr);
    std::cout << "args.num set to: " << args.num << "\n";
    std::cout << "original ptr: " << bvh4->root.ptr << "\n";
    std::cout << "args.ptr set to: " << args.startNodePtr << "\n";

    // start intersection
    std::cout << "Starting ray intersection test..." << std::endl;
    std::cout << "[main program] rayhit.ray.tfar: " << rayhit.ray.tfar << "\n"; 
    rtcIntersect1(scene, &rayhit, &args);

    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        std::cout << "Intersection at t = " << rayhit.ray.tfar << std::endl;
        std::cout << "Hit primitive ID = " << rayhit.hit.primID << std::endl;
        std::cout << "Hit geometry ID = " << rayhit.hit.geomID << std::endl;
    } else {
        std::cout << "No Intersection" << std::endl;
    }

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
    return 0;
}