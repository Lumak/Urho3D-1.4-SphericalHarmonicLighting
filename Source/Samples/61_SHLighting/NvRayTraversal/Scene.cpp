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

#include "Scene.hpp"
#include "BVH.hpp"
#include "BVHNode.hpp"
#include "Platform.hpp"

using namespace FW;

//=============================================================================
//=============================================================================
NScene::NScene()
{
    m_pBVH = NULL;

    m_numTriangles = 0;
    m_numVertices = 0;
}

//=============================================================================
//=============================================================================
NScene::~NScene()
{
    if ( m_pBVH )
    {
        delete m_pBVH;
        m_pBVH = NULL;
    }
}

//=============================================================================
//=============================================================================
U32 NScene::hash(void)
{
    //return hashBits(
    //    hashBuffer(m_triVtxIndex.getPtr(), (int)m_triVtxIndex.getSize()),
    //    hashBuffer(m_triNormal.getPtr(), (int)m_triNormal.getSize()),
    //    hashBuffer(m_triMaterialColor.getPtr(), (int)m_triMaterialColor.getSize()),
    //    hashBuffer(m_triShadedColor.getPtr(), (int)m_triShadedColor.getSize()),
    //    hashBuffer(m_vtxPos.getPtr(), (int)m_vtxPos.getSize()));
    return 0;
}

//=============================================================================
//=============================================================================
void NScene::AddTriangleMesh(NvMeshData& meshData)
{
    // add verts and normals
    for ( unsigned i = 0; i < meshData.listVerts.size(); ++i )
    {
        m_vlistVerts.push_back( meshData.listVerts[ i ] );
        m_vlistNormals.push_back( meshData.listNormals[ i ] );
    }

    // add triangles
    for ( unsigned i = 0; i < meshData.listTris.size(); ++i )
    {
        Vec3i t = meshData.listTris[ i ];

        // add offset
        t.x += m_numVertices;
        t.y += m_numVertices;
        t.z += m_numVertices;

        m_vlistTriangles.push_back( t );
    }

    // update verts and tri counters
    m_numVertices += meshData.listVerts.size();
    m_numTriangles += meshData.listTris.size();
}

//=============================================================================
//=============================================================================
void NScene::FillBuffers()
{
#if 0
    m_triVtxIndex.resizeDiscard(m_numTriangles * sizeof(Vec3i));
    m_triNormal.resizeDiscard(m_numTriangles * sizeof(Vec3f));
    m_vtxPos.resizeDiscard(m_numVertices * sizeof(Vec3f));

    Vec3i* triVtxIndex = (Vec3i*)m_triVtxIndex.getMutablePtr();
    Vec3f* triNormal   = (Vec3f*)m_triNormal.getMutablePtr();
    Vec3f* vtxPos      = (Vec3f*)m_vtxPos.getMutablePtr();

    // Copy vertices.
    //const VertexP* v = meshP.getVertexPtr();
    //m_vlistVerts.push_back( meshData.listVerts[ i ] );
    //m_vlistNormals.push_back( meshData.listNormals[ i ] );

    for ( unsigned i = 0; i < m_vlistVerts.size(); ++i )
    {
        vtxPos[i] = m_vlistVerts[i];
        triNormal[i] = m_vlistNormals[i];
    }

    for ( unsigned i = 0; i < m_vlistTriangles.size(); ++i )
    {
        triVtxIndex[i] = m_vlistTriangles[i];
    }

    // Collapse submeshes to a single triangle list.

    //for (int submesh = 0; submesh < meshP.numSubmeshes(); submesh++)
    //{
    //    const Array<Vec3i>& indices = meshP.indices(submesh);
    //    const MeshBase::Material& material = meshP.material(submesh);
    //    U32 colorU32 = material.diffuse.toABGR();
    //    Vec3f colorVec3f = material.diffuse.getXYZ();
    //
    //    for (int i = 0; i < indices.getSize(); i++)
    //    {
    //        const Vec3i& vi     = indices[i];
    //        Vec3f normal = normalize(cross(vtxPos[vi.y] - vtxPos[vi.x], vtxPos[vi.z] - vtxPos[vi.x]));
    //
    //        *triVtxIndex++      = vi;
    //        *triNormal++        = normal;
    //        //*triMaterialColor++ = colorU32;
    //        //*triShadedColor++   = Vec4f(colorVec3f * (dot(normal, light) * 0.5f + 0.5f), 1.0f).toABGR();
    //    }
    //}
#endif
}

//=============================================================================
//=============================================================================
void NScene::CreateBVH(const void* platform, const void* params)
{
    // create bvh (runs automatically)
	m_pBVH = new BVH(this, *(Platform*)platform, *(BVH::BuildParams*)params);
}

//=============================================================================
//=============================================================================
const BVHNode* NScene::GetBVHRoot() 
{
    return m_pBVH->getRoot();
}

//=============================================================================
//=============================================================================
void NScene::RayCast(RayBuffer& rays, RayStats* stats)
{
    if ( m_pBVH )
    {
        m_pBVH->trace(rays, stats);
    }
    //void BVH::trace(RayBuffer& rays, RayStats* stats) const
    //struct Ray
    //{
    //    FW_CUDA_FUNC            Ray         (void)      : origin(0.0f), tmin(0.0f), direction(0.0f), tmax(0.0f) {}
    //    FW_CUDA_FUNC    void    degenerate  (void)      { tmax = tmin - 1.0f; }
    //
    //    Vec3f           origin;
    //    float           tmin;
    //    Vec3f           direction;
    //    float           tmax;
    //};
}














