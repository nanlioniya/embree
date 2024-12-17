/*
    this is for converting ann search into embree bvh search
*/

// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
#include "../../kernels/bvh/bvh.h"
#include "../../include/embree4/rtcore.h"
#include <limits>
#include <iostream>

// 添加錯誤回調函數以捕獲Embree訊息
void errorFunction(void* userPtr, RTCError error, const char* str)
{
    std::cout << "Embree Error [" << error << "]: " << str << std::endl;
}

int main()
{
    // 創建device時設置verbose=2來開啟詳細輸出
    // RTCDevice device = rtcNewDevice("verbose=2,threads=1");
    RTCDevice device = rtcNewDevice("tri_accel=bvh4.triangle4v");
    // 設置錯誤回調函數
    rtcSetDeviceErrorFunction(device, errorFunction, nullptr);
    
    // 其餘代碼保持不變
    RTCScene scene = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    float* vb = (float*) rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), 3);
    vb[0] = 0.f; vb[1] = 0.f; vb[2] = 0.f;
    vb[3] = 1.f; vb[4] = 0.f; vb[5] = 0.f;
    vb[6] = 0.f; vb[7] = 1.f; vb[8] = 0.f;

    unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
        RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned), 1);
    ib[0] = 0; ib[1] = 1; ib[2] = 2;

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    
    std::cout << "Starting scene commit..." << std::endl;
    rtcCommitScene(scene);
    std::cout << "Scene commit completed." << std::endl;

    RTCRayHit rayhit;
    rayhit.ray.org_x = 0.f; rayhit.ray.org_y = 0.f; rayhit.ray.org_z = -1.f;
    rayhit.ray.dir_x = 0.f; rayhit.ray.dir_y = 0.f; rayhit.ray.dir_z = 1.f;
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



    RTCIntersectArguments args;
    // rtcInitIntersectArguments(&args);
    rtcInitIntersectArgumentsWithID(&args, 38, bvh4->root);
    std::cout << "args.num set to: " << args.num << "\n";
    std::cout << "original ptr: " << bvh4->root.ptr << "\n";
    std::cout << "args.ptr set to: " << args.startNodePtr << "\n";

    // start intersection
    std::cout << "Starting ray intersection test..." << std::endl;
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