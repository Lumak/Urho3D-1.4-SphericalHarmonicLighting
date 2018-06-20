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
#include <Urho3D/Core/Context.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/IO/FileSystem.h>

#include "VertexUtil.h"
#include "SHConfigure.h"

//=============================================================================
//=============================================================================
void VertexUtil::RegisterObject(Context *context)
{
    context->RegisterFactory<VertexUtil>();
}

//=============================================================================
//=============================================================================
VertexUtil::VertexUtil(Context *context)
    : Component(context)
{
}

//=============================================================================
//=============================================================================
VertexUtil::~VertexUtil()
{
}

//=============================================================================
//=============================================================================
bool VertexUtil::CreateModelVertexColorElement(StaticModelData &staticModelData)
{
    StaticModel *pStaticModel = (StaticModel*)staticModelData.drawable;
    Model *pModel = pStaticModel->GetModel();
    Geometry *pGeometry = pModel->GetGeometry(0, 0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    unsigned uElementMask = pVbuffer->GetElementMask();

    // exit if it already has vertex color
    if ( uElementMask & MASK_COLOR )
    {
        return false;
    }

    unsigned vertexSize = pVbuffer->GetVertexSize();
    unsigned numVertices = pVbuffer->GetVertexCount();

    // create new vertex buffer
    SharedPtr<VertexBuffer> buffer( new VertexBuffer( context_ ) );
    buffer->SetShadowed( pVbuffer->IsShadowed() );
    buffer->SetSize( numVertices, uElementMask | MASK_COLOR );
    unsigned newvertexSize = buffer->GetVertexSize();
    unsigned uNewElementMask = buffer->GetElementMask();

    const unsigned char *pVertexData = (const unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());
    const unsigned char *pNewVertexData = (const unsigned char*)buffer->Lock(0, buffer->GetVertexCount());
    bool bValid = false;

    if ( pVertexData && pNewVertexData )
    {
        bValid = true;

        for ( unsigned i = 0; i < numVertices; ++i )
        {
            unsigned char *pDataAlign = (unsigned char *)(pVertexData + i * vertexSize);
            unsigned char *pNewDataAlign = (unsigned char *)(pNewVertexData + i * newvertexSize);

            if ( uElementMask & MASK_POSITION )
            {
                const Vector3 vPos = *reinterpret_cast<Vector3*>( pDataAlign );
                pDataAlign += sizeof( Vector3 );

                Vector3 &vNnewPos = *reinterpret_cast<Vector3*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector3 );

                vNnewPos = vPos;
            }

            if ( uElementMask & MASK_NORMAL )
            {
                const Vector3 vNorm = *reinterpret_cast<Vector3*>( pDataAlign );
                pDataAlign += sizeof( Vector3 );

                Vector3 &vNewNorm = *reinterpret_cast<Vector3*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector3 );

                vNewNorm = vNorm;
            }

            // obviously the original doesn't have this element
            if ( uNewElementMask & MASK_COLOR )
            {
                unsigned &uNewColor = *reinterpret_cast<unsigned*>( pNewDataAlign );
                pNewDataAlign += sizeof( unsigned );

                uNewColor = ( staticModelData.materialColor ).ToUInt();
            }

            if ( uElementMask & MASK_TEXCOORD1 )
            {
                const Vector2 vUV = *reinterpret_cast<Vector2*>( pDataAlign );
                pDataAlign += sizeof( Vector2 );

                Vector2 &vNewUV = *reinterpret_cast<Vector2*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector2 );

                vNewUV = vUV;
            }

            if ( uElementMask & MASK_TEXCOORD2 )
            {
                const Vector2 vUV = *reinterpret_cast<Vector2*>( pDataAlign );
                pDataAlign += sizeof( Vector2 );

                Vector2 vNewUV = *reinterpret_cast<Vector2*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector2 );

                vNewUV = vUV;
            }

            if ( uElementMask & MASK_CUBETEXCOORD1 )
            {
                const Vector2 vUV = *reinterpret_cast<Vector2*>( pDataAlign );
                pDataAlign += sizeof( Vector2 );

                Vector2 &vNewUV = *reinterpret_cast<Vector2*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector2 );

                vNewUV = vUV;
            }

            if ( uElementMask & MASK_CUBETEXCOORD2 )
            {
                const Vector2 vUV = *reinterpret_cast<Vector2*>( pDataAlign );
                pDataAlign += sizeof( Vector2 );

                Vector2 &vNewUV = *reinterpret_cast<Vector2*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector2 );

                vNewUV = vUV;
            }

            if ( uElementMask & MASK_TANGENT )
            {
                const Vector4 vTan = *reinterpret_cast<Vector4*>( pDataAlign );
                pDataAlign += sizeof( Vector4 );

                Vector4 &vNewTan = *reinterpret_cast<Vector4*>( pNewDataAlign );
                pNewDataAlign += sizeof( Vector4 );

                vNewTan = vTan;
            }

            // todo: copy other mask elements
            assert( (uElementMask & (MASK_BLENDWEIGHTS | MASK_BLENDINDICES | 
                                     MASK_INSTANCEMATRIX1 | MASK_INSTANCEMATRIX2 | MASK_INSTANCEMATRIX3)) == 0 
                    && "implement other elements that you need" );
        }

        //unlock
        pVbuffer->Unlock();
        buffer->Unlock();

        // assign new buffer
        pGeometry->SetVertexBuffer( 0, buffer, uNewElementMask );
        Vector<SharedPtr<VertexBuffer> > buffers;
        PODVector<unsigned> morphRangeStarts;
        morphRangeStarts.Push( 0 );
        PODVector<unsigned> morphRangeCounts;
        morphRangeCounts.Push( 0 );
        buffers.Push( buffer );
        pModel->SetVertexBuffers( buffers, morphRangeStarts, morphRangeCounts );
   }

    return bValid;
}
//=============================================================================
//=============================================================================
bool VertexUtil::SetVertexColorDiffuseTransfer(StaticModelData &staticModelData, int itype)
{
    StaticModel *pStaticModel = (StaticModel*)staticModelData.drawable;
    Model *pModel = pStaticModel->GetModel();
    Geometry *pGeometry = pModel->GetGeometry(0, 0);
    VertexBuffer *pVbuffer = pGeometry->GetVertexBuffer(0);
    unsigned uElementMask = pVbuffer->GetElementMask();

    // skip if no color mask
    if ( ( uElementMask & MASK_COLOR ) == 0 )
    {
        return false;
    }

    bool bValid = false;
    {
        unsigned vertexSize = pVbuffer->GetVertexSize();
        unsigned numVertices = pVbuffer->GetVertexCount();
        const unsigned char *pVertexData = (const unsigned char*)pVbuffer->Lock(0, pVbuffer->GetVertexCount());

        if ( pVertexData )
        {
            bValid = true;
            PODVector<TCoeffs> &listVertexUnshadoweCoeff = staticModelData.listVertexUnshadoweCoeff;
            PODVector<TCoeffs> &listVertexShadowCoeff    = staticModelData.listVertexShadowCoeff;
            PODVector<Color>  &listVertexDiffuseColor    = staticModelData.listVertexDiffuseColor;

            for ( unsigned j = 0; j < numVertices; ++j )
            {
                unsigned char *pDataAlign = (unsigned char *)(pVertexData + j * vertexSize);

                // get the alignment to the vertex color
                if ( uElementMask & MASK_POSITION ) pDataAlign += sizeof( Vector3 );
                if ( uElementMask & MASK_NORMAL )   pDataAlign += sizeof( Vector3 );

                unsigned &uColor = *reinterpret_cast<unsigned*>( pDataAlign );

                // accumulate sh coeff vcolor
                Color resultColor(Color::BLACK);
                Color matColor = staticModelData.materialColor;
                Color col = matColor;

                for ( unsigned k = 0; k < kSH_Coeffs; ++k )
                {
                    if ( itype == kSHDifTrans_Off )
                    {
                        break;
                    }

                    if ( itype == kSHDifTrans_Interreflected )
                    {
                        resultColor = listVertexDiffuseColor[ j ];
                        break;
                    }

                    switch ( itype )
                    {
                    case kSHDifTrans_Samples:
                        resultColor += matColor;// * staticModelData.shProjectedPolarCoeff[ k ];
                        break;
                    case kSHDifTrans_Lighting:
                        resultColor += matColor * listVertexUnshadoweCoeff[ j ].shCoeff[ k ] * staticModelData.shProjectedPolarCoeff.shCoeff[ k ];
                        break;
                    case kSHDifTrans_Shadowed:
                        resultColor += matColor * listVertexShadowCoeff[ j ].shCoeff[ k ] * staticModelData.shProjectedPolarCoeff.shCoeff[ k ];
                        break;
                    }
                }

                // set vertex color
                if ( itype == kSHDifTrans_Off )
                {
                    ;//do nothing, col is already defaulted to matColor
                }
                else
                {
                    col.r_ = Clamp( resultColor.r_, Min( matColor.r_, SH_MIN_VCOLOR_VAL), 1.0f );
                    col.g_ = Clamp( resultColor.g_, Min( matColor.g_, SH_MIN_VCOLOR_VAL), 1.0f );
                    col.b_ = Clamp( resultColor.b_, Min( matColor.b_, SH_MIN_VCOLOR_VAL), 1.0f );
                    col.a_ = 1.0f;
                }

                uColor = (col).ToUInt();
            }

            pVbuffer->Unlock();
        }
    }

    return bValid;
}

//=============================================================================
//=============================================================================
void VertexUtil::ChangeModelMaterial(StaticModelData &staticModelData, int itype, const String &strVtxOffName, const String &strVtxOnName)
{
    StaticModel *pStaticModel = (StaticModel*)staticModelData.drawable;
    Material *pMaterial = pStaticModel->GetMaterial();
    String strMatName = pMaterial->GetName();

    String strDir;
    String strFilenameNoExt;
    ExtractFileDirAndName( strMatName, strDir, strFilenameNoExt );

    String strAltMatFileName = strDir + strFilenameNoExt;

    // diff type
    switch ( itype )
    {
    case kSHDifTrans_Off:
        strAltMatFileName += strVtxOffName;//".xml";
        break;

    case kSHDifTrans_Lighting:
    case kSHDifTrans_Shadowed:
    case kSHDifTrans_Interreflected:
        strAltMatFileName += strVtxOnName;//"-diffvcol.xml";
        break;

    default:
        return;
    }

    // switch material
    pMaterial = GetSubsystem<ResourceCache>()->GetResource<Material>( strAltMatFileName );

    if ( pMaterial )
    {
        pStaticModel->SetMaterial( 0, pMaterial );
    }

}

//=============================================================================
// find material matcolor
//=============================================================================
Color VertexUtil::GetModelMaterialColor(StaticModel *pStaticModel) const
{
    Color matColor(Color::WHITE);
    Material* pMaterial = pStaticModel->GetMaterial();

    if ( pMaterial )
    {
        Variant var = pMaterial->GetShaderParameter( "MatDiffColor" );
        
        if ( VAR_COLOR == var.GetType() )
        {
            matColor = var.GetColor();
        }
        else if ( VAR_VECTOR4 == var.GetType() )
        {
            matColor = Color( var.GetVector4().Data() ) ;
        }
    }
    return matColor;
}


