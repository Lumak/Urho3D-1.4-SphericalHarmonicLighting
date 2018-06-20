//=============================================================================
// Copyright (c) 2016 Lumak Software
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
//=============================================================================

#include <Urho3D/Urho3D.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/IO/Filesystem.h>
#include <Urho3D/Container/Str.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/OctreeQuery.h>

#include "StaticModelPoolMgr.h"
#include "VertexUtil.h"
#include "SHConfigure.h"

#include "NvRayTraversal/Scene.hpp"
#include "NvRayTraversal/Math.hpp"
#include "NvRayTraversal/BVH.hpp"
#include "NvRayTraversal/Platform.hpp"
#include "NvRayTraversal/RayBuffer.hpp"

#include <Urho3D/DebugNew.h>
#include <SDL/SDL_log.h>

//=============================================================================
//=============================================================================
FW::Ray ToNVRay(Urho3D::Ray &ray, float fdistance)
{
    // Nvidia's box-to-ray intersection algorithm divides the box length by the ray direction 
    // component's x, y, and z, and if the component is zero then the result obviously becomes #INF
    // prevent this by changing ray's zero components to a small epsilon
    if ( ray.direction_.x_ < M_EPSILON && ray.direction_.x_ > -M_EPSILON )
    {
        ray.direction_.x_ = M_EPSILON;
    }
    if ( ray.direction_.y_ < M_EPSILON && ray.direction_.y_ > -M_EPSILON )
    {
        ray.direction_.y_ = M_EPSILON;
    }
    if ( ray.direction_.z_ < M_EPSILON && ray.direction_.z_ > -M_EPSILON )
    {
        ray.direction_.z_ = M_EPSILON;
    }

    // convert 
    FW::Ray vray;
    vray.direction  = Vec3f( ray.direction_.x_, ray.direction_.y_, ray.direction_.z_ );
    vray.origin     = Vec3f( ray.origin_.x_, ray.origin_.y_, ray.origin_.z_ );
    vray.tmin       = 0.0f;
    vray.tmax       = fdistance;

    return vray;
}

//=============================================================================
//=============================================================================
void StaticModelPoolMgr::RegisterObject(Context *context)
{
    context->RegisterFactory<StaticModelPoolMgr>();
}

//=============================================================================
//=============================================================================
StaticModelPoolMgr::StaticModelPoolMgr(Context *context_)
    : Component( context_ )
{
    m_pNvScene = new NScene();
}

//=============================================================================
//=============================================================================
StaticModelPoolMgr::~StaticModelPoolMgr()
{
    DEL_AND_NULL( m_pNvScene );
}

//=============================================================================
//=============================================================================
void StaticModelPoolMgr::AddStaticModelData(Node *pNode, StaticModel *pStaticModel)
{
    NvMeshData nvMeshData;
    StaticModelData staticModelData;
    Model *pModel = pStaticModel->GetModel();

    // only one geom currently supprted
    assert( pModel && pModel->GetNumGeometries() == 1 && "multiple gemoetries currently NOT supported" );

    Matrix3x4 objMatrix = pNode->GetTransform();
    Quaternion objRotation = pNode->GetRotation();
    Geometry *pGeometry = pModel->GetGeometry(0, 0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);

    unsigned uElementMask = pVbuffer->GetElementMask();
    unsigned vertexSize = pVbuffer->GetVertexSize();
    const unsigned char *pVertexData = (const unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());

    // get verts, normals, uv, etc.
    if ( pVertexData )
    {
        unsigned numVertices = pVbuffer->GetVertexCount();

        for ( unsigned i = 0; i < numVertices; ++i )
        {
            unsigned char *pDataAlign = (unsigned char *)(pVertexData + i * vertexSize);

            if ( uElementMask & MASK_POSITION )
            {
                const Vector3 vPos = *reinterpret_cast<Vector3*>( pDataAlign );
                pDataAlign += sizeof( Vector3 );

                Vector3 vxformPos = objMatrix * vPos; // xform

                // verts list
                staticModelData.listVerts.Push( vxformPos );
                nvMeshData.listVerts.push_back( Vec3f( vxformPos.x_, vxformPos.y_, vxformPos.z_ ) );
            }
            if ( uElementMask & MASK_NORMAL )
            {
                const Vector3 vNorm = *reinterpret_cast<Vector3*>( pDataAlign );
                pDataAlign += sizeof( Vector3 );

                // normal list
                Vector3 vxformNorm = objRotation * vNorm; // xform

                staticModelData.listNormals.Push( vxformNorm );
                nvMeshData.listNormals.push_back( Vec3f( vxformNorm.x_, vxformNorm.y_, vxformNorm.z_ ) );
            }
            if ( uElementMask & MASK_COLOR )
            {
                const unsigned uColor = *reinterpret_cast<unsigned*>( pDataAlign );
                pDataAlign += sizeof( unsigned );
            }
            if ( uElementMask & MASK_TEXCOORD1 )
            {
                const Vector2 vUV = *reinterpret_cast<Vector2*>( pDataAlign );
                pDataAlign += sizeof( Vector2 );

                // uv list
                staticModelData.listUVs.Push( vUV );
            }

            // skip other mask elements - we got what we wanted
        }

        //unlock
        pVbuffer->Unlock();
    }
    else
    {
        // error
        assert( false && "failed to unlock vertex buffer" );
    }

    // get indeces
    IndexBuffer *pIbuffer = pGeometry->GetIndexBuffer();
    const unsigned *pIndexData = (const unsigned *)pIbuffer->Lock( 0, pIbuffer->GetIndexCount() );
    const unsigned short *pUShortData = (const unsigned short *)pIndexData;

    if ( pUShortData )
    {
        unsigned numIndeces = pIbuffer->GetIndexCount();
        unsigned indexSize = pIbuffer->GetIndexSize();
        assert( indexSize == sizeof(unsigned short) );

        for( unsigned i = 0; i < numIndeces; i += 3 )
        {
            int idx0 = (int)pUShortData[i  ];
            int idx1 = (int)pUShortData[i+1];
            int idx2 = (int)pUShortData[i+2];

            staticModelData.listTris.Push( IntVector3( idx0, idx1, idx2 ) );
            nvMeshData.listTris.push_back( Vec3i( idx0, idx1, idx2 ) );
        }

        //unlock
        pIbuffer->Unlock();
    }
    else
    {
        // error
        assert( false && "failed to unlock index buffer" );
    }

    // rest of the static model data
    staticModelData.node       = pNode;
    staticModelData.drawable   = pStaticModel;
    staticModelData.begVertex  = m_pNvScene->getNumVertices();
    staticModelData.begTriList = m_pNvScene->getNumTriangles();

    // resize coeff lists
    staticModelData.listVertexUnshadoweCoeff.Resize( staticModelData.listVerts.Size() );
    staticModelData.listVertexShadowCoeff.Resize( staticModelData.listVerts.Size() );
    staticModelData.listVertexDiffuseColor.Resize( staticModelData.listVerts.Size() );
    staticModelData.listVertexSampleOcclude.resize( staticModelData.listVerts.Size() );

    // find material matcolor
    VertexUtil *pVertexUtil = GetScene()->GetComponent<VertexUtil>();
    staticModelData.materialColor = pVertexUtil->GetModelMaterialColor( pStaticModel );

    // save model
    m_vStaticModelPool.Push( staticModelData );

    // push it to nv Scene
    m_pNvScene->AddTriangleMesh( nvMeshData );
}

//=============================================================================
//=============================================================================
void StaticModelPoolMgr::CreateBVH()
{
    // default settings 
    Platform platform;
    BVH::BuildParams params;
    BVH::Stats bvhStats;
    params.stats = &bvhStats;

    // disable print in NVraytrav
    params.enablePrints = false;

    // create bvh
    HiresTimer htimer;
    m_pNvScene->CreateBVH( &platform, &params );
    long long elapsed = htimer.GetUSec( true );

    // stats
    params.enablePrints = true;
    if ( params.enablePrints )
    {
        const BVHNode *root = m_pNvScene->GetBVHRoot();

        SDL_Log("BVH build time=%I64d msec\n", elapsed/1000);
        SDL_Log("BVH builder: %d tris, %d vertices\n", m_pNvScene->getNumTriangles(), m_pNvScene->getNumVertices());
        SDL_Log("BVH: Scene bounds: (%.1f,%.1f,%.1f) - (%.1f,%.1f,%.1f)\n", 
                root->m_bounds.min().x, root->m_bounds.min().y, root->m_bounds.min().z,
                root->m_bounds.max().x, root->m_bounds.max().y, root->m_bounds.max().z);
        SDL_Log("BVH: SAHCost         = %.2f\n", params.stats->SAHCost );
        SDL_Log("BVH: branchingFactor = %d\n",   params.stats->branchingFactor );
        SDL_Log("BVH: numLeafNodes    = %d\n",   params.stats->numLeafNodes );
        SDL_Log("BVH: numInnerNodes   = %d\n",   params.stats->numInnerNodes );
        SDL_Log("BVH: numTris         = %d\n",   params.stats->numTris );
        SDL_Log("BVH: numChildNodes   = %d\n",   params.stats->numChildNodes );
    }
}

//=============================================================================
//=============================================================================
bool StaticModelPoolMgr::Raycast(Urho3D::Ray &ray, float fdistance, NvRayResult &result)
{
    // error checking
    if ( m_pNvScene->GetBVH() == NULL )
    {
        return false;
    }

    // ray cast
    RayBuffer raysbuf(1);
    FW::Ray vray = ToNVRay( ray, fdistance );
    raysbuf.setRay(0, vray);
    m_pNvScene->RayCast( raysbuf );
    const RayResult &rayResult = raysbuf.getResultForSlot( 0 );
    bool bhit = false;

    // copy result
    if ( rayResult.hit() )
    {
        unsigned uid = rayResult.id;

        for ( unsigned i = 0; i < m_vStaticModelPool.Size(); ++i )
        {
            const StaticModelData &staticModelData = m_vStaticModelPool[ i ];

            if ( uid >= staticModelData.begTriList && uid < staticModelData.begTriList + staticModelData.listTris.Size() )
            {
                IntVector3 tri   = staticModelData.listTris[ uid - staticModelData.begTriList ];
                result.position_ = ray.origin_ + ray.direction_ * rayResult.t;
                result.normal_   = staticModelData.listNormals[ tri.x_ ];// this could be barycentric normal
                result.distance_ = rayResult.t;
                result.drawable_ = staticModelData.drawable;
                result.node_     = staticModelData.node;

                // pool and tri id
                result.triIndex_ = uid - staticModelData.begTriList;
                result.poolIdx_  = i;
                bhit = true;
                break;
            }
        }
    }
    return bhit;
}

















