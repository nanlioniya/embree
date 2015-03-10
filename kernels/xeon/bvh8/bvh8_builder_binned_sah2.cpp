// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "bvh8.h"
#include "bvh8_statistics.h"
#include "common/profile.h"

#include "builders_new/primrefgen.h"
#include "builders_new/presplit.h"
#include "builders_new/bvh_builder2.h"

#include "geometry/triangle4.h"
#include "geometry/triangle8.h"

#define PROFILE 0

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    struct CreateAlloc
    {
      __forceinline CreateAlloc (BVH8* bvh) : bvh(bvh) {}
      __forceinline Allocator* operator() () const { return bvh->alloc2.threadLocal2();  }

      BVH8* bvh;
    };

    struct CreateBVH8Node
    {
      __forceinline CreateBVH8Node (BVH8* bvh) : bvh(bvh) {}
      
      __forceinline int operator() (const isa::BuildRecord2<BVH8::NodeRef>& current, BuildRecord2<BVH8::NodeRef>** children, const size_t N, Allocator* alloc) 
      {
        BVH8::Node* node = NULL;
        //if (current.pinfo.size() > 4096) node = (BVH8::Node*)   bvh->alloc2.malloc(sizeof(BVH8::Node),sizeof(BVH8::Node));
        //else
        node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node)); 
        node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->pinfo.geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return 0;
      }

      BVH8* bvh;
    };

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH8* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline int operator() (const BuildRecord2<BVH8::NodeRef>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH8::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene,false);
        }
        *current.parent = node;
	return 1;
      }

      BVH8* bvh;
      PrimRef* prims;
    };
    
    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<typename Mesh, typename Primitive>
    struct BVH8BuilderBinnedSAH2 : public Builder
    {
      BVH8* bvh;
      Scene* scene;
      Mesh* mesh;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH8BuilderBinnedSAH2 (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH8BuilderBinnedSAH2 (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH8::emptyNode,empty,0);
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
      
        /* verbose mode */
        if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderBinnedSAH2 " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
	    
          if ((g_benchmark || g_verbose >= 1) && mesh == NULL) t0 = getSeconds();
	    
	    bvh->alloc2.init(numSplitPrimitives*sizeof(PrimRef),numSplitPrimitives*sizeof(BVH8::Node));  // FIXME: better estimate
	    prims.resize(numSplitPrimitives);
	    PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
            if (presplitFactor > 1.0f)
              pinfo = presplit<Mesh>(scene, pinfo, prims);
	    BVH8::NodeRef root = bvh_builder_binned_sah2_internal<BVH8::NodeRef>
	      (CreateAlloc(bvh),CreateBVH8Node(bvh),CreateLeaf<Primitive>(bvh,prims.data()),
	       prims.data(),pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);

            bvh->set(root,pinfo.geomBounds,pinfo.size());
            bvh->layoutLargeNodes(numSplitPrimitives*0.005f);

	    if ((g_benchmark || g_verbose >= 1) && mesh == NULL) dt = getSeconds()-t0;

#if PROFILE
           dt = timer.avg();
        }); 
#endif	

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) prims.resize(0,true);
	bvh->alloc2.cleanup();

	/* verbose mode */
	if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	if (g_verbose >= 2 && mesh == NULL)
	  bvh->printStatistics();

        /* benchmark mode */
        if (g_benchmark) {
          BVH8Statistics stat(bvh);
          std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
        }
      }

      void clear() {
        prims.clear();
      }
    };

    /* entry functions for the scene builder */
    Builder* BVH8Triangle4SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderBinnedSAH2<TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle8SceneBuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderBinnedSAH2<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    struct CreateListBVH8Node // FIXME: merge with above class
    {
      __forceinline CreateListBVH8Node (BVH8* bvh) : bvh(bvh) {}
      
      __forceinline BVH8::Node* operator() (const isa::BuildRecord2<BVH8::NodeRef,PrimRefList>& current, BuildRecord2<BVH8::NodeRef,PrimRefList>** children, const size_t N, Allocator* alloc) 
      {
        BVH8::Node* node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i]->pinfo.geomBounds);
          children[i]->parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH8* bvh;
    };

    template<typename Primitive>
    struct CreateListLeaf
    {
      __forceinline CreateListLeaf (BVH8* bvh) : bvh(bvh) {}
      
      __forceinline size_t operator() (BuildRecord2<BVH8::NodeRef, PrimRefList>& current, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t n = current.pinfo.size();
        size_t N = Primitive::blocks(n);
        Primitive* leaf = (Primitive*) alloc->alloc1.malloc(N*sizeof(Primitive));
        BVH8::NodeRef node = bvh->encodeLeaf((char*)leaf,N);

        /*PrimRefList::block_iterator_unsafe iter1(current.prims);
        while (iter1) {
          const int i = iter1->lower.a;
          iter1->lower.a = geomIDprimID[i].first;
          iter1->upper.a = geomIDprimID[i].second;
          iter1++;
          }*/
        PrimRefList::block_iterator_unsafe iter1(current.prims);
        while (iter1) {
          iter1->lower.a &= 0x00FFFFFF; // FIXME: hack
          iter1++;
        }

        /* insert all triangles */
        PrimRefList::block_iterator_unsafe iter(current.prims);
        for (size_t i=0; i<N; i++) leaf[i].fill(iter,bvh->scene,false);
        assert(!iter);
        
        /* free all primitive blocks */
        while (PrimRefList::item* block = current.prims.take())
          delete block;

        *current.parent = node;
	return n;
      }

      BVH8* bvh;
      //const vector_t<std::pair<int,int>>& geomIDprimID;
    };

    template<typename Mesh, typename Primitive>
    struct BVH8BuilderSpatialBinnedSAH2 : public Builder
    {
      BVH8* bvh;
      Scene* scene;
      Mesh* mesh;
      //vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const float presplitFactor;

      BVH8BuilderSpatialBinnedSAH2 (BVH8* bvh, Scene* scene, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(NULL), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      BVH8BuilderSpatialBinnedSAH2 (BVH8* bvh, Mesh* mesh, const size_t leafBlockSize, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(NULL), mesh(mesh), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,leafBlockSize*BVH8::maxLeafBlocks)),
          presplitFactor((mode & MODE_HIGH_QUALITY) ? 1.5f : 1.0f) {}

      void build(size_t, size_t) 
      {
	/* skip build for empty scene */
	const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,1>();
        if (numPrimitives == 0) {
          //prims.resize(numPrimitives);
          bvh->set(BVH8::emptyNode,empty,0);
          return;
        }
        const size_t numSplitPrimitives = max(numPrimitives,size_t(presplitFactor*numPrimitives));
      
        /* reduction function */
	auto rotate = [&] (BVH8::Node* node, const size_t* counts, const size_t N) 
	{
          size_t n = 0;
#if ROTATE_TREE
	  assert(N <= BVH8::N);
          for (size_t i=0; i<N; i++) 
            n += counts[i];
          if (n >= 4096) {
            for (size_t i=0; i<N; i++) {
              if (counts[i] < 4096) {
                for (int j=0; j<ROTATE_TREE; j++) 
                  BVH8Rotate::rotate(bvh,node->child(i)); 
                node->child(i).setBarrier();
              }
            }
          }
#endif
	  return n;
	};

        /* verbose mode */
        if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "building BVH8<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH8BuilderBinnedSAH2 " << (presplitFactor != 1.0f ? "presplit" : "") << " ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
#if PROFILE
	profile(2,20,numPrimitives,[&] (ProfileTimer& timer)
        {
#endif
	    
          if ((g_benchmark || g_verbose >= 1) && mesh == NULL) t0 = getSeconds();
	    
	    bvh->alloc2.init(numSplitPrimitives*sizeof(PrimRef),numSplitPrimitives*sizeof(BVH8::Node));  // FIXME: better estimate
	    //prims.resize(numSplitPrimitives);
	    //PrimInfo pinfo = mesh ? createPrimRefArray<Mesh>(mesh,prims) : createPrimRefArray<Mesh,1>(scene,prims);
            PrimRefList prims;
            PrimInfo pinfo = createPrimRefList<Mesh,1>(scene,prims);
            //PRINT(pinfo.size());

            //SpatialSplitHeuristic heuristic(scene);

            /* calculate total surface area */
            PrimRefList::iterator iter = prims;
            const size_t threadCount = TaskSchedulerNew::threadCount();
            const double A = parallel_reduce(size_t(0),threadCount,0.0, [&] (const range<size_t>& r) // FIXME: this sum is not deterministic
            {
              double A = 0.0f;
              while (PrimRefList::item* block = iter.next()) {
                for (size_t i=0; i<block->size(); i++) 
                  A += area(block->at(i));
                //A += heuristic(block->at(i));
              }
              return A;
            },std::plus<double>());

            /* calculate number of maximal spatial splits per primitive */
            float f = 10.0f;
            iter = prims;
            const size_t N = parallel_reduce(size_t(0),threadCount,size_t(0), [&] (const range<size_t>& r)
            {
              size_t N = 0;
              while (PrimRefList::item* block = iter.next()) {
                for (size_t i=0; i<block->size(); i++) {
                  PrimRef& prim = block->at(i);
                  assert((prim.lower.a & 0xFF000000) == 0);
                  const float nf = ceil(f*pinfo.size()*area(prim)/A);
                  //const size_t n = 16;
                  const size_t n = 4+min(ssize_t(127-4), max(ssize_t(1), ssize_t(nf)));
                  N += n;
                  prim.lower.a |= n << 24;
                }
              }
              return N;
            },std::plus<size_t>());

	    BVH8::NodeRef root = bvh_builder_reduce_spatial_sah2_internal<BVH8::NodeRef>
	      (scene,CreateAlloc(bvh),size_t(0),CreateListBVH8Node(bvh),rotate,CreateListLeaf<Primitive>(bvh),
               [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o)
               {
                TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); 
                TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
                const Vec3fa v0 = mesh->vertex(tri.v[0]);
                const Vec3fa v1 = mesh->vertex(tri.v[1]);
                const Vec3fa v2 = mesh->vertex(tri.v[2]);
                splitTriangle(prim,dim,pos,v0,v1,v2,left_o,right_o);
              },
	       prims,pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize,BVH8::travCost,intCost);
	    bvh->set(root,pinfo.geomBounds,pinfo.size());
            
#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH8Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif
            
            bvh->layoutLargeNodes(pinfo.size()*0.005f);

            if ((g_benchmark || g_verbose >= 1) && mesh == NULL) dt = getSeconds()-t0;
            
#if PROFILE
            dt = timer.avg();
        }); 
#endif	

	/* clear temporary data for static geometry */
	//bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	//if (staticGeom) prims.resize(0,true);
	bvh->alloc2.cleanup();

        /* verbose mode */
	if (g_verbose >= 1 && mesh == NULL)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	if (g_verbose >= 2 && mesh == NULL)
	  bvh->printStatistics();

        /* benchmark mode */
        if (g_benchmark) {
          BVH8Statistics stat(bvh);
          std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
        }
      }

      void clear() {
        //prims.clear();
      }
    };

    /* entry functions for the scene builder */
    Builder* BVH8Triangle4SceneBuilderSpatialBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSpatialBinnedSAH2<TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle8SceneBuilderSpatialBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH8BuilderSpatialBinnedSAH2<TriangleMesh,Triangle8>((BVH8*)bvh,scene,8,4,1.0f,8,inf,mode); }
  }
}
