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

#include "SHFileUtil.h"

//=============================================================================
//=============================================================================
void SHFileUtil::RegisterObject(Context *context)
{
    context->RegisterFactory<SHFileUtil>();
}

//=============================================================================
//=============================================================================
SHFileUtil::SHFileUtil(Context *context)
    : Component(context)
{
}

//=============================================================================
//=============================================================================
SHFileUtil::~SHFileUtil()
{
}

//=============================================================================
//=============================================================================
void SHFileUtil::SetParameters(SHConfigParameter &param)
{
    memcpy( &m_SHConfigParameter, &param, sizeof( SHConfigParameter ));
}

//=============================================================================
//=============================================================================
const SHConfigParameter& SHFileUtil::GetParameters() const
{
    return m_SHConfigParameter;
}

//=============================================================================
//=============================================================================
bool SHFileUtil::ReadCoeffFile(StaticModelData *pStaticModelData)
{
    // coeff lists
    PODVector<TCoeffs> &listVertexUnshadoweCoeff    = pStaticModelData->listVertexUnshadoweCoeff;
    PODVector<TCoeffs> &listVertexShadowCoeff = pStaticModelData->listVertexShadowCoeff;
    PODVector<Color> &listVertexDiffuseColor = pStaticModelData->listVertexDiffuseColor;
    std::vector<std::vector<int> > &listVertexSampleOcclude = pStaticModelData->listVertexSampleOcclude;
    Color materialColor = pStaticModelData->materialColor;

    StaticModel *pStaticModel = (StaticModel*)pStaticModelData->drawable;
    Model *pModel = pStaticModel->GetModel();
    String strFullModelfile = pModel->GetName();
    String strDir;
    String strFilenameNoExt;
    ExtractFileDirAndName( strFullModelfile, strDir, strFilenameNoExt );

    String strCoeffFilename = GetSubsystem<FileSystem>()->GetProgramDir() + "Data/" + strDir + strFilenameNoExt + "-coeff.dat";
    bool bFileRead = false;
    
    if ( GetSubsystem<FileSystem>()->FileExists( strCoeffFilename ) )
    {
        File dest( GetContext() );
    
        if ( dest.Open( strCoeffFilename, FILE_READ ) )
        {
            unsigned uNumVerts = 0;
            unsigned uNumBytes = dest.Read( &uNumVerts, sizeof(uNumVerts) );
    
            for ( unsigned i = 0; i < uNumVerts; ++i )
            {
                std::vector<int> &sampleOcclude = listVertexSampleOcclude[ i ];

                uNumBytes = dest.Read( listVertexUnshadoweCoeff[ i ].shCoeff, sizeof(listVertexUnshadoweCoeff[i].shCoeff) );
                uNumBytes = dest.Read( listVertexShadowCoeff[ i ].shCoeff, sizeof(listVertexShadowCoeff[ i ].shCoeff));

                int isize;
                uNumBytes = dest.Read( &isize, sizeof(int) );
                for ( int j = 0; j < isize; ++j )
                {
                    int val;
                    uNumBytes = dest.Read( &val, sizeof(int) );
                    sampleOcclude.push_back( val );
                }
            }
            dest.Close();
            bFileRead = true;
        }
    }
    return bFileRead;
}

//=============================================================================
//=============================================================================
bool SHFileUtil::WriteCoeffFile(StaticModelData *pStaticModelData)
{
    StaticModel *pStaticModel = (StaticModel*)pStaticModelData->drawable;
    Model *pModel = pStaticModel->GetModel();

    // coeff list
    PODVector<Vector3> &listVerts = pStaticModelData->listVerts;
    PODVector<TCoeffs> &listVertexUnshadoweCoeff    = pStaticModelData->listVertexUnshadoweCoeff;
    PODVector<TCoeffs> &listVertexShadowCoeff = pStaticModelData->listVertexShadowCoeff;
    std::vector<std::vector<int> > &listVertexSampleOcclude = pStaticModelData->listVertexSampleOcclude;

    String strFullModelfile = pModel->GetName();
    String strDir;
    String strFilenameNoExt;
    ExtractFileDirAndName( strFullModelfile, strDir, strFilenameNoExt );

    String strCoeffFilename = GetSubsystem<FileSystem>()->GetProgramDir() + "Data/" + strDir + strFilenameNoExt + "-coeff.dat";
    bool bFileWritten = false;

    File dest( GetContext() );

    if ( dest.Open( strCoeffFilename, FILE_WRITE ) )
    {
        unsigned uNumVerts = listVerts.Size();
        unsigned uNumBytes = dest.Write( &uNumVerts, sizeof(uNumVerts) );

        for ( unsigned i = 0; i < uNumVerts; ++i )
        {
            std::vector<std::vector<int> > &listVertexSampleOcclude = pStaticModelData->listVertexSampleOcclude;
            std::vector<int> &sampleOcclude = listVertexSampleOcclude[ i ];

            uNumBytes = dest.Write( listVertexUnshadoweCoeff[ i ].shCoeff, sizeof(listVertexUnshadoweCoeff[i].shCoeff) );       
            uNumBytes = dest.Write( listVertexShadowCoeff[ i ].shCoeff, sizeof(listVertexShadowCoeff[ i ].shCoeff) );

            int isize = sampleOcclude.size();
            uNumBytes = dest.Write( &isize, sizeof(int) );

            for ( int j = 0; j < isize; ++j )
            {
                int val = sampleOcclude[j];
                uNumBytes = dest.Write( &val, sizeof(int) );
            }
        }
        dest.Close();
        bFileWritten = true;
    }

    return bFileWritten;
}

//=============================================================================
//=============================================================================
void SHFileUtil::WriteSHToModel(StaticModelData *pStaticModelData)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    StaticModel *pStaticModel = (StaticModel*)pStaticModelData->drawable;

    Material *pMaterial = pStaticModel->GetMaterial();
    Texture *pTextureDiff = pMaterial->GetTexture( TU_DIFFUSE );
    String strTextNameDiff = pTextureDiff->GetName();
    SharedPtr<Image> diffImage;
    diffImage = cache->GetResource<Image>( strTextNameDiff );

    int texSizeX = diffImage->GetWidth();
    int texSizeY = diffImage->GetHeight();
    float texSizeXINV = 1.0f/(float)texSizeX;
    float texSizeYINV = 1.0f/(float)texSizeY;

    // cycle triangle list
    const PODVector<IntVector3> &listTris = pStaticModelData->listTris;
    const PODVector<Vector2> &listUVs     = pStaticModelData->listUVs;

    const PODVector<TCoeffs> &listVertexUnshadoweCoeff    = pStaticModelData->listVertexUnshadoweCoeff;
    const PODVector<TCoeffs> &listVertexShadowCoeff = pStaticModelData->listVertexShadowCoeff;

    for ( unsigned i = 0; i < listTris.Size(); ++i )
    {
        int i0 = listTris[ i ].x_;
        int i1 = listTris[ i ].y_;
        int i2 = listTris[ i ].z_;

        float xMin = 1.0f;	
        float xMax = 0.0f;	
        float yMin = 1.0f;
        float yMax = 0.0f;

        // uvs
        Vector2 uv0 = listUVs[ i0 ];
        Vector2 uv1 = listUVs[ i1 ];
        Vector2 uv2 = listUVs[ i2 ];

        // min/max of the uvs
        if ( uv0.x_ < xMin ) xMin = uv0.x_; 
        if ( uv1.x_ < xMin ) xMin = uv1.x_; 
        if ( uv2.x_ < xMin ) xMin = uv2.x_; 
                                         
        if ( uv0.x_ > xMax ) xMax = uv0.x_; 
        if ( uv1.x_ > xMax ) xMax = uv1.x_; 
        if ( uv2.x_ > xMax ) xMax = uv2.x_; 
                                         
        if ( uv0.y_ < yMin ) yMin = uv0.y_; 
        if ( uv1.y_ < yMin ) yMin = uv1.y_; 
        if ( uv2.y_ < yMin ) yMin = uv2.y_; 
                                         
        if ( uv0.y_ > yMax ) yMax = uv0.y_;
        if ( uv1.y_ > yMax ) yMax = uv1.y_;
        if ( uv2.y_ > yMax ) yMax = uv2.y_;

        // get pixels to iterate
        int pixMinX = (int)Max( (float)xMin*texSizeX, 0.0f); 
        int pixMaxX = (int)Min( (float)xMax*texSizeX + 1.5f, (float)texSizeX); 
        int pixMinY = (int)Max( (float)yMin*texSizeY, 0.0f); 
        int pixMaxY = (int)Min( (float)yMax*texSizeY + 1.5f, (float)texSizeY);

        for ( int x = pixMinX; x < pixMaxX; ++x ) 
        {
            for ( int y = pixMinY; y < pixMaxY; ++y ) 
            {
                Vector2 pixel = Vector2( (float)x * texSizeXINV, (float)y * texSizeYINV );
                Vector3 bary = Barycentric( uv0, uv1, uv2, pixel );

                // make sure the pixel is in the triangle
                if ( bary.x_ == M_INFINITY || !BaryInsideTriangle( bary ) )
                {
                    continue; 
                }
                
                {
                    float result = 0.0f;

                    // get sh coeff based on barycentric
                    for ( int j = 0; j < kSH_Coeffs; ++j )
                    {
                        #ifdef BAKE_DIFFUSE_LIGHT_ONLY
                        float shCoeff = listVertexUnshadoweCoeff[ i0 ].shCoeff[ j ] * bary.x_ + 
                                        listVertexUnshadoweCoeff[ i1 ].shCoeff[ j ] * bary.y_ + 
                                        listVertexUnshadoweCoeff[ i2 ].shCoeff[ j ] * bary.z_;
                        #else
                        float shCoeff = listVertexShadowCoeff[ i0 ].shCoeff[ j ] * bary.x_ + 
                                        listVertexShadowCoeff[ i1 ].shCoeff[ j ] * bary.y_ + 
                                        listVertexShadowCoeff[ i2 ].shCoeff[ j ] * bary.z_;
                        #endif

                        result += shCoeff;// * pStaticModelData->shProjectedPolarCoeff[ j ];
                    }

                    // update pix color
                    Color color = diffImage->GetPixel(x,y);
                    result = Clamp(result, SH_MIN_VCOLOR_VAL, 1.0f);
                    color.r_ = color.r_ * result;
                    color.g_ = color.g_ * result;
                    color.b_ = color.b_ * result;

                    diffImage->SetPixel(x, y, color);
                }
            }
        }
    }

    // save to a file
    String filename = GetSubsystem<FileSystem>()->GetProgramDir() + "Data/SHLighting/sphereSHout.png";
    diffImage->SavePNG( filename );
}


//=============================================================================
//=============================================================================
void SHFileUtil::SaveVertexColorModel(StaticModel *pStaticModel)
{
    Model *pModel = pStaticModel->GetModel();
    String strFullModelfile = pModel->GetName();

    String strDir;
    String strMainFilename;
    ExtractFileDirAndName( strFullModelfile, strDir, strMainFilename );

    String path = GetSubsystem<FileSystem>()->GetProgramDir() + "Data/";
    String strSavefilename = path + strDir + strMainFilename + "vc.mdl";
    SharedPtr<File> file;
    file = new File( context_ );

    if ( file->Open( strSavefilename, FILE_WRITE ) )
    {
        pModel->Save( *file );
        file->Close();
    }
}

//=============================================================================
//=============================================================================
void SHFileUtil::DrawStratifiedSHSamples(const Vector<SHSample> &shSamples)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    FileSystem *pFileSystem = GetSubsystem<FileSystem>();
    Image* image = cache->GetResource<Image>("SHLighting/whitepanel.png");

    for ( unsigned i = 0; i < shSamples.Size(); ++i ) 
    {   
        Vector3 vec = shSamples[ i ].vec;
        float result = 0.0f;

        for ( int j = 0; j < kSH_Coeffs; ++j ) 
        {     
            result += (float)shSamples[ i ].coeff[ j ];
        }
        result = Clamp( result, 0.0f, 1.0f);

        int x = 127 + (int)(vec.x_ * 127.0f);
        int y = 127 + (int)(vec.y_ * 127.0f);

        image->SetPixel( x, y, Color( result, 0.0f, 0.0f) );
    }

    String filename = GetSubsystem<FileSystem>()->GetProgramDir() + "Data/SHLighting/SHstratifiedSamples.png";
    image->SavePNG( filename );
}


