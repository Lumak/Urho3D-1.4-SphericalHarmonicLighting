/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include "Math.hpp"
//#   include "3d/Mesh.hpp"
#include "String.hpp"
#include "Platform.hpp"

#include <vector>

namespace FW
{
class BVH;
class RayBuffer;
struct RayStats;

//=============================================================================
//=============================================================================
struct NvMeshData
{
    std::vector<Vec3i> listTris;
    std::vector<Vec3f> listVerts;
	std::vector<Vec3f> listNormals;
};

//=============================================================================
//=============================================================================
class NScene
{
public:
                    //Scene               (const MeshBase& mesh);
                    NScene               ();
                    ~NScene              ();

    int             getNumTriangles             (void) const    { return m_numTriangles; }
    int             getNumVertices              (void) const    { return m_numVertices; }

	const std::vector<Vec3f>& getVertexList()   { return m_vlistVerts; }
	const std::vector<Vec3f>& getNormalList()   { return m_vlistNormals; }
	const std::vector<Vec3i>& getTriIndexList() { return m_vlistTriangles; }

    //Buffer&         getTriVtxIndexBuffer()      { return m_triVtxIndex; }
    //Buffer&         getTriNormalBuffer()        { return m_triNormal; }
    //Buffer&         getVtxPosBuffer()           { return m_vtxPos; }
    //Buffer&         getTriMaterialColorBuffer   (void)          { return m_triMaterialColor; }
    //Buffer&         getTriShadedColorBuffer     (void)          { return m_triShadedColor; }

    U32             hash                        (void);

    //lumak
    int GetTriangleCount() const 
    {
        return m_numTriangles;
    }

    int GetVerticesCount() const 
    {
        return m_numVertices;
    }

    void AddTriangleMesh(NvMeshData& meshData);
    void CreateBVH(const void* platform, const void* params);
    const BVH* GetBVH() { return m_pBVH; }
	const BVHNode* GetBVHRoot();
    void RayCast(RayBuffer& rays, RayStats* stats = NULL);


protected:
    void FillBuffers();

private:
                    NScene           (const NScene&); // forbidden
					NScene&          operator=           (const NScene&); // forbidden

protected:
    S32             m_numTriangles;
    S32             m_numVertices;
    //Buffer          m_triVtxIndex;      // Vec3i[numTriangles]
    //Buffer          m_triNormal;        // Vec3f[numTriangles]
    //Buffer          m_triMaterialColor; // U32[numTriangles], ABGR
    //Buffer          m_triShadedColor;   // U32[numTriangles], ABGR
    //Buffer          m_vtxPos;           // Vec3f[numVertices]

    // bvh
    BVH     *m_pBVH;

    // lists for bvh
    std::vector<Vec3f> m_vlistVerts;
    std::vector<Vec3f> m_vlistNormals;
    std::vector<Vec3i> m_vlistTriangles;
};

//------------------------------------------------------------------------
}
