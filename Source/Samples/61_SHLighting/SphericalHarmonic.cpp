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
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/OctreeQuery.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Math/BoundingBox.h>

#include "SphericalHarmonic.h"
#include "StaticModelPoolMgr.h"
#include "SHFileUtil.h"

#include <Urho3D/DebugNew.h>
#include <SDL/SDL_log.h>

//=============================================================================
// Spherical Harmonic Lighting based on: 
// http://www.research.scea.com/gdc2003/spherical-harmonic-lighting.pdf
// 
// three types of diffused transfer
// 1 - diffused unshadowed transfer
// 2 - shadowed diffuse transfer
// 3 - diffuse interreflected transfer
//=============================================================================
#ifndef max
double max(double a, double b) { return (a) > (b) ? (a) : (b); }
#endif

#define MIN_NV_RAY_OFFSET       0.03f
#define LARGEST_AXIS_METHOD
//#define USE_VARYING_RAYS

//=============================================================================
// static var for project polar fn
//=============================================================================
double SphericalHarmonic::s_X[kNLights_Max];
double SphericalHarmonic::s_Y[kNLights_Max];
double SphericalHarmonic::s_Z[kNLights_Max];
double SphericalHarmonic::s_Brightness[kNLights_Max];
int    SphericalHarmonic::s_NLights;

//=============================================================================
//=============================================================================
void SphericalHarmonic::RegisterObject(Context *context)
{
    context->RegisterFactory<SphericalHarmonic>();
}

//=============================================================================
//=============================================================================
SphericalHarmonic::SphericalHarmonic(Context *context)
    : Component(context)
    , m_nSamples(0)
    , m_nSqrtSamples(0)
{
    GenerateRandRayVectors();
}

//=============================================================================
//=============================================================================
SphericalHarmonic::~SphericalHarmonic()
{
}

//=============================================================================
//=============================================================================
void SphericalHarmonic::CreateSHSamples(int nsamples)
{
    assert( nsamples > 0 && "bad sample value" );

    int nsqrt = (int)sqrt( nsamples );

    // error checking
    if ( nsamples < SH_MIN_SAMPLES || nsqrt*nsqrt != nsamples ) 
    {
        nsamples = SH_MIN_SAMPLES;
        nsqrt = (int)sqrt( SH_MIN_SAMPLES );
    }

    m_nSamples = nsamples;
    m_nSqrtSamples = nsqrt;

    SetupSphericalSamples();
}

//=============================================================================
// sh lighting samples
//=============================================================================
void SphericalHarmonic::SetupSphericalSamples()
{
    // fill an N*N*2 array with uniformly distributed
    // samples across the sphere using jittered stratification
    double oneoverN = 1.0/m_nSqrtSamples;
    const double halfPi = M_PI*0.5;

    m_vSHSamples.Clear();

	for (int a = 0; a<m_nSqrtSamples; a++)
    {
		for (int b = 0; b < m_nSqrtSamples; ++b)
        {
            SHSample shsample;

            // generate unbiased distribution of spherical coords
            double x = (double)(a + Random()) * oneoverN; // do not reuse results    
            double y = (double)(b + Random()) * oneoverN; // each sample must be random    
            double theta = 2.0 * acos(sqrt(1.0 - x));
            double phi = 2.0 * M_PI * y;

            shsample.sph = Vector3((float)theta,(float)phi,1.0f);    // convert spherical coords to unit vector
            shsample.vec = Vector3((float)(sin(theta)*cos(phi)), (float)(sin(theta)*sin(phi)), (float)cos(theta));
             
            // precompute all SH coefficients for this sample    
            for(int l=0; l<kSH_Bands; ++l)
            {
                for(int m=-l; m<=l; ++m)
                {
                    int index = GetIndex(l, m);

                    shsample.coeff[index] = SH(l,m,theta,phi);
                }
            }

            m_vSHSamples.Push( shsample );
        }
    }

    // write stratified image
    if ( GetScene()->GetComponent<SHFileUtil>()->GetParameters().writeStratifiedSampleToImage )
    {
        GetScene()->GetComponent<SHFileUtil>()->DrawStratifiedSHSamples( m_vSHSamples );
    }
}

//=============================================================================
//=============================================================================
void SphericalHarmonic::CreateModelCoeffs()
{
    StaticModelPoolMgr *pStaticModelPoolMgr = GetScene()->GetComponent<StaticModelPoolMgr>();
    Vector<StaticModelData>& modelPool = pStaticModelPoolMgr->GetStaticModelList();

    // create polar coeff first
    for ( unsigned i = 0; i < modelPool.Size(); ++i )
    {
        CreateModelPolarCoeff( &modelPool[ i ] );
    }

    //dbg out
    HiresTimer htimer;

    for ( unsigned i = 0; i < modelPool.Size(); ++i )
    {
        CreateDiffuseVertexCoeff( i );
    }

    long long elapsed1 = htimer.GetUSec( true );
    SDL_Log( "SH CreateModelCoeffs build time = %I64d msec\n", elapsed1/1000 );
}

//=============================================================================
// SelfTransferSH
//=============================================================================
void SphericalHarmonic::SelfTransferSH()
{
    StaticModelPoolMgr *pStaticModelPoolMgr = GetScene()->GetComponent<StaticModelPoolMgr>();
    Vector<StaticModelData>& statModelList = pStaticModelPoolMgr->GetStaticModelList();

    //dbg out
    HiresTimer htimer;

    for ( unsigned i= 0; i < statModelList.Size(); ++i )
    {
        SelfTransferDiffuseVertex( i );
    }

    long long elapsed1 = htimer.GetUSec( true );
    SDL_Log( "SH SelfTransferSH build time = %I64d msec\n", elapsed1/1000 );
}

//=============================================================================
//=============================================================================
void SphericalHarmonic::CreateModelPolarCoeff(StaticModelData *pStaticModelData)
{
    #ifdef MULTIPLE_LIGHTS_TEST
    PODVector<Node*> listNode;
    GetScene()->GetChildrenWithComponent( listNode, "Light", true);

    s_NLights = listNode.Size() <= kNLights_Max ? listNode.Size() : kNLights_Max;

    for ( unsigned i = 0; i < listNode.Size() && i < kNLights_Max; ++i )
    {
        Node *pLightNode = listNode[ i ];
        Light *pLight = pLightNode->GetComponent<Light>();
        Vector3 dirToLight(Vector3::UP);

        // get dir to light
        if ( pLight->GetLightType() == LIGHT_POINT )
        {
            dirToLight = ( pLightNode->GetPosition() - pStaticModelData->node->GetPosition() ).Normalized();
        }
        else if ( pLight->GetLightType() == LIGHT_DIRECTIONAL )
        {
            dirToLight = pLightNode->GetDirection() * -1.0f;
        }

        s_X[ i ] = (double)dirToLight.x_;
        s_Y[ i ] = (double)dirToLight.y_;
        s_Z[ i ] = (double)dirToLight.z_;

        s_Brightness[ i ] = (double)pLight->GetBrightness();

    }

    struct ProjectFunc
    {
        static double PolarFnNLights(double theta, double phi)
        {
            double result = 0.0;

            for ( int i = 0; i < s_NLights; ++i )
            {
                result += max( 0, s_Brightness[ i ] * ( s_X[i]*cos( phi )*sin( theta ) + s_Y[i]*sin( phi )*sin( theta ) + s_Z[i]*cos( theta ) ) );
            }
            return result;
        }
    };

    // init
    double dataCoeffResult[ kSH_Coeffs ];
    memset( dataCoeffResult, 0, sizeof(dataCoeffResult) );

    // project
    ProjectPolarFunction( ProjectFunc::PolarFnNLights, dataCoeffResult );

    #else 
    //-----------------------------------------------
    //-----------------------------------------------
    // get light
    PODVector<Node*> listNode;
    GetScene()->GetChildrenWithComponent( listNode, "Light", true);

    assert( listNode.Size() == 1 && "currently only works with a SINGLE light in the scene" );

    Node *pLightNode = listNode[ 0 ];
    Light *pLight = pLightNode->GetComponent<Light>();
    Vector3 dirToLight(Vector3::UP);

    // get dir to light
    if ( pLight->GetLightType() == LIGHT_POINT )
    {
        dirToLight = ( pLightNode->GetPosition() - pStaticModelData->node->GetPosition() ).Normalized();
    }
    else if ( pLight->GetLightType() == LIGHT_DIRECTIONAL )
    {
        dirToLight = pLightNode->GetDirection() * -1.0f;
    }

    s_X[0] = (double)dirToLight.x_;
    s_Y[0] = (double)dirToLight.y_;
    s_Z[0] = (double)dirToLight.z_;

    // sh project polar func
    // *** spherical-to-cartesian ***
    // x = cos(phi)*sin(theta)      phi = arctan( y/x )
    // y = sin(phi)*sin(theta)      theta = arctan( sqrt( x^2 + y^2 ) /z )
    // z = cos(theta)     
    struct ProjectFunc
    {
        static double PolarFn(double theta, double phi)
        {
            // **from the SH lighting doc
            //return max(0, 5 * cos(theta) - 4) + max(0, -4 * sin(theta - M_PI) * cos(phi - 2.5) - 3);

            // spherical cartesian eqn: 0 <= sum <= 1 for unit sphere
            return max( 0, s_X[0]*cos( phi )*sin( theta ) + s_Y[0]*sin( phi )*sin( theta ) + s_Z[0]*cos( theta ) );
        }
    };

    // init
    double dataCoeffResult[ kSH_Coeffs ];
    memset( dataCoeffResult, 0, sizeof(dataCoeffResult) );

    // project
    ProjectPolarFunction( ProjectFunc::PolarFn, dataCoeffResult );

    #endif

    // **double to float
    for ( int i = 0; i < kSH_Coeffs; ++i )
    {
        pStaticModelData->shProjectedPolarCoeff.shCoeff[ i ] = (float)dataCoeffResult[ i ];
    }
}

//=============================================================================
// model obj projection coeffs
//=============================================================================
void SphericalHarmonic::ProjectPolarFunction(SH_polar_fn fn, double result[])
{
    const double weight = 4.0*M_PI;
    const double factor = weight / m_nSamples;

    // for each sample
    for ( int i = 0; i < m_nSamples; ++i )
    {
        double theta = (double)m_vSHSamples[i].sph.x_;
        double phi   = (double)m_vSHSamples[i].sph.y_;

        for ( int n = 0; n < kSH_Coeffs; ++n )
        {
            result[n] += fn( theta, phi ) * m_vSHSamples[i].coeff[n];
        }
    }  

    // divide the result by weight and number of samples  
    for ( int i = 0; i < kSH_Coeffs; ++i )
    {
        result[i] *= factor;
    }
}

//=============================================================================
//=============================================================================
void SphericalHarmonic::CreateDiffuseVertexCoeff(int modelIdx)
{
    StaticModelPoolMgr *pStaticModelPoolMgr = GetScene()->GetComponent<StaticModelPoolMgr>();

    Vector<StaticModelData>& statModelData = pStaticModelPoolMgr->GetStaticModelList();
    StaticModelData *pStaticModelData = &statModelData[ modelIdx ];
    StaticModel *pStaticModel = (StaticModel*)pStaticModelData->drawable;
    Model *pModel = pStaticModel->GetModel();

    // skip if the coeff file exist and read
    if ( ReadCoeffFile( pStaticModelData) )
    {
       return;
    }

    //******************************
    // calculate vert coeffs
    const float area = (float)( 4.0 * M_PI );
    const float factor = area/(float)m_nSamples;

    // const lists
    const PODVector<Vector3> &listVerts = pStaticModelData->listVerts;
    const PODVector<Vector3> &listNormals = pStaticModelData->listNormals;

    // mutable coeff lists
    PODVector<TCoeffs> &listVertexUnshadoweCoeff = pStaticModelData->listVertexUnshadoweCoeff;
    PODVector<TCoeffs> &listVertexShadowCoeff = pStaticModelData->listVertexShadowCoeff;
    std::vector<std::vector<int> > &listVertexSampleOcclude = pStaticModelData->listVertexSampleOcclude;
    PODVector<Color>  &listVertexDiffuseColor = pStaticModelData->listVertexDiffuseColor;
    TCoeffs &shProjectedPolarCoeff = pStaticModelData->shProjectedPolarCoeff;
    Color materialColor = pStaticModelData->materialColor;

    //dbg
    HiresTimer htimer;

    for ( unsigned i = 0; i < listVerts.Size(); ++i )
    {
        // clear data
        memset( listVertexUnshadoweCoeff[i].shCoeff, 0, sizeof(listVertexUnshadoweCoeff[i].shCoeff) );
        memset( listVertexShadowCoeff[i].shCoeff, 0, sizeof(listVertexShadowCoeff[i].shCoeff) );

        std::vector<int> &sampleOcclude = listVertexSampleOcclude[ i ];

        for ( int j = 0; j < m_nSamples; ++j )
        {
            float dot = m_vSHSamples[ j ].vec.DotProduct( listNormals[ i ] );

            if ( dot > 0.0f )
            {
                NvRayResult result;
                Ray ray( listVerts[ i ] + listNormals[ i ] * MIN_NV_RAY_OFFSET, m_vSHSamples[ j ].vec );
                bool occlude = pStaticModelPoolMgr->Raycast( ray, SHORT_RAY_LENGTH, result );

                // save occlude sample idx
                if ( occlude )
                {
                    sampleOcclude.push_back( j );
                }

                // accumulate coeffs results
                for ( int k = 0; k < kSH_Coeffs; ++k )
                {
                    float fResult = dot * (float)m_vSHSamples[ j ].coeff[ k ];

                    listVertexUnshadoweCoeff[ i ].shCoeff[ k ] += fResult;

                    if ( !occlude )
                    {
                        listVertexShadowCoeff[ i ].shCoeff[ k ] += fResult;
                    }
                }
            }
        }

        Color colorResult(Color::BLACK);

        // rescale the coefficients
        for ( int j = 0; j < kSH_Coeffs; ++j )
        {
            listVertexUnshadoweCoeff[ i ].shCoeff[ j ] *= factor;
            listVertexShadowCoeff[ i ].shCoeff[ j ] *= factor;

            const float result = listVertexShadowCoeff[ i ].shCoeff[ j ] * shProjectedPolarCoeff.shCoeff[ j ];

            colorResult.r_ += materialColor.r_ * result;
            colorResult.g_ += materialColor.g_ * result;
            colorResult.b_ += materialColor.b_ * result;
        }
        listVertexDiffuseColor[ i ] = colorResult;
    }

    //dbg
    long long elapsed1 = htimer.GetUSec( true );
    SDL_Log( "SH model-%d coeff build time = %I64d msec\n", modelIdx, elapsed1/1000 );

    // write file
    WriteCoeffFile( pStaticModelData );
}

//=============================================================================
// SelfTransferDiffuseVertex
// -intentionally left the original code from the SHL doc commented out
//=============================================================================
void SphericalHarmonic::SelfTransferDiffuseVertex(int modelIdx)
{
    StaticModelPoolMgr *pStaticModelPoolMgr = GetScene()->GetComponent<StaticModelPoolMgr>();
    Vector<StaticModelData>& statModelList = pStaticModelPoolMgr->GetStaticModelList();

    // static model data
    StaticModelData& staticModelData = statModelList[ modelIdx ];
    const PODVector<Vector3> &listVerts = staticModelData.listVerts;
    const PODVector<Vector3> &listNormals = staticModelData.listNormals;
    PODVector<TCoeffs> &listVertexShadowCoeff = staticModelData.listVertexShadowCoeff;
    PODVector<Color> &listVertexDiffuseColor = staticModelData.listVertexDiffuseColor;
    std::vector<std::vector<int> > &listVertexSampleOcclude = staticModelData.listVertexSampleOcclude;
    Color materialColor = staticModelData.materialColor;

    //dbg
    HiresTimer htimer;

    // iterate through all verts and find missing occlude flags
    for ( unsigned vtx = 0; vtx < listVerts.Size(); ++vtx )
    {
        TCoeffs &listShadowCoeff = listVertexShadowCoeff[ vtx ];
        Color &vertexDiffuseColor = listVertexDiffuseColor[vtx];
        std::vector<int> &sampleOcclude = listVertexSampleOcclude[ vtx ];
        bool bOccluded = false;

        if ( sampleOcclude.size() == 0 )
        {
            continue;
        }

        const int n_bounces = 1;
        const int n_lighting = 1;
        const float area = 4.0f * (float)M_PI;  
        //float *sh_buffer[n_bounces+1]; 
        Color otherColorCoeff[kSH_Coeffs];
        // list of light bounce buffers.

        // allocate and clear buffers for self transferred light  
        //sh_buffer[0] = sh_coeff; // already calculated from direct lighting  

        //for(int i=1; i<=n_bounces; ++i) 
        //{   
        //    sh_buffer[i] = new float[n_lighting * 3 * kSH_Coeffs];   
        //    memset(sh_buffer[i], 0, n_lighting*3*kSH_Coeffs*sizeof(float));  
        //}
        for ( int i = 0; i < kSH_Coeffs; ++i )
        {
            otherColorCoeff[i] = Color::BLACK;
        }

        // for each bounce of light  
        for(int bounce=1; bounce<=n_bounces; ++bounce) 
        {
            // loop through all lighting points redistributing self light
            for(int i=0; i<n_lighting; ++i) 
            {    
                // find rays that hit self    
                //bitvector::iterator j;    
                int n = 0;    
                float u = 0.0f, v = 0.0f, w = 0.0f;    
                //Face *fptr = 0;    
                float sh[kSH_Coeffs];    
                // get the surface albedo of the lighting point.    
                //float albedo_red   = mlist[plist[i].material].kd.x / PI;    
                //float albedo_green = mlist[plist[i].material].kd.y / PI;    
                //float albedo_blue  = mlist[plist[i].material].kd.z / PI;    

                // loop through boolean vector looking for a ray that hits self…    
                //for(j=hit_self[i].begin(); j!=hit_self[i].end(); ++n,++j) 
                for ( unsigned sn = 0; sn < sampleOcclude.size(); ++sn )
                {
                    n = sampleOcclude[ sn ];
                    bOccluded = true;
                
                    //if(*j) **if it's in the sampleOcclude[] container then it's a hit
                    {
                        // calc H cosine term about surface normal      
                        //float Hs = DotProduct(sample[n].vec, plist[i].norm);
                        float Hs = m_vSHSamples[ n ].vec.DotProduct( listNormals[ vtx ] );
                        // if ray inside hemisphere, continue processing.      
                        if ( Hs > 0.0f ) 
                        {       
                            // trace ray to find tri and (u,v,w) barycentric coords of hit       
                            //u = v = w = 0.0;       
                            //fptr = 0;
                            //bool ret = raytrace_closest_triangle(plist[i].pos,                                            
                            //                                     sample[n].vec,                                            
                            //                                     face_ptr, u, v);   
                            NvRayResult result;
                            Vector3 pos = listVerts[ vtx ] + listNormals[ vtx ] * MIN_NV_RAY_OFFSET;
                            bool ret = RaytraceClosestTriangle( pos, m_vSHSamples[ n ].vec, result);
                
                            // if (surprise, surprise) the ray hits something...       
                            if ( ret ) 
                            {
                                // the hit surface data
                                StaticModelData &BstaticModelData = statModelList[ result.poolIdx_ ];
                                const PODVector<Vector3> &BlistVerts = BstaticModelData.listVerts;
                                const PODVector<IntVector3> &BlistTris = BstaticModelData.listTris;
                                PODVector<TCoeffs> &BlistVertexShadowCoeff = BstaticModelData.listVertexShadowCoeff;
                                TCoeffs &BshProjectedPolarCoeff = BstaticModelData.shProjectedPolarCoeff;
                                Color BmaterialColor = BstaticModelData.materialColor * (1.0f/(float)M_PI);
                
                                const IntVector3 &tri = BlistTris[ result.triIndex_ ];
                                const Vector3 &v0 = BlistVerts[ tri.x_ ];
                                const Vector3 &v1 = BlistVerts[ tri.y_ ];
                                const Vector3 &v2 = BlistVerts[ tri.z_ ];
                
                                Vector3 bary = Barycentric( v0, v1, v2, result.position_ );
                
                                // lerp vertex SH vector to get SH at hit point        
                                //w = 1.0 - (u+v);
                                //float *ptr0 = ;//sh_buffer[bounce-1] + face_ptr->vert[0]*3*kSH_Coeffs;        
                                //float *ptr1 = ;//sh_buffer[bounce-1] + face_ptr->vert[1]*3*kSH_Coeffs;        
                                //float *ptr2 = ;//sh_buffer[bounce-1] + face_ptr->vert[2]*3*kSH_Coeffs;   
                                u = bary.x_;
                                v = bary.y_;
                                w = bary.z_;
                                const TCoeffs &sc0 = BlistVertexShadowCoeff[ tri.x_ ];
                                const TCoeffs &sc1 = BlistVertexShadowCoeff[ tri.y_ ];
                                const TCoeffs &sc2 = BlistVertexShadowCoeff[ tri.z_ ];
                
                                //for(int k=0; k<3*kSH_Coeffs; ++k) 
                                for(int k=0; k<kSH_Coeffs; ++k) 
                                {            
                                    //sh[k] = u*(*ptr0++) + v*(*ptr1++) + w*(*ptr2++);        
                                    sh[k] = u*sc0.shCoeff[k] + v*sc1.shCoeff[k] + w*sc2.shCoeff[k];        
                                }        
                                // sum reflected SH light for this vertex
                                for(int k=0; k<kSH_Coeffs; ++k) 
                                {
                                    otherColorCoeff[ k ].r_ += BmaterialColor.r_* Hs * sh[ k ] * BshProjectedPolarCoeff.shCoeff[ k ];
                                    otherColorCoeff[ k ].g_ += BmaterialColor.g_* Hs * sh[ k ] * BshProjectedPolarCoeff.shCoeff[ k ];
                                    otherColorCoeff[ k ].b_ += BmaterialColor.b_* Hs * sh[ k ] * BshProjectedPolarCoeff.shCoeff[ k ];
                                    //sh_buffer[bounce][i*3*kSH_Coeffs + k+0*kSH_Coeffs] += albedo_red * Hs * sh[k+0*kSH_Coeffs];         
                                    //sh_buffer[bounce][i*3*kSH_Coeffs + k+1*kSH_Coeffs] += albedo_green * Hs * sh[k+1*kSH_Coeffs];         
                                    //sh_buffer[bounce][i*3*kSH_Coeffs + k+2*kSH_Coeffs] += albedo_blue  * Hs * sh[k+2*kSH_Coeffs];        
                                }       
                            } // ray test      
                        } // hemisphere test     
                    } // hit self bit is true    
                } // loop for bool vector   
            } // each lighting point   
        } // loop over all bounces

        // divide through by n_samples * bounces
        if ( bOccluded )
        {
            const float factor = area / (float)( m_nSamples * n_bounces );
            //float *ptr = sh_buffer[bounce];   
            //for(int j=0; j<n_lighting * 3 * kSH_Coeffs; ++j)  
            for(int j=0; j< kSH_Coeffs; ++j)  
            {
                //*ptr++ *= factor;
                otherColorCoeff[ j ] = otherColorCoeff[ j ] * factor;
            }
        }

        // sum all bounces of self transferred light back into sh_coeff  
        //for(int i=1; i<=n_bounces; ++i) 
        //{   
        //     float *ptra = sh_buffer[0];   
        //     float *ptrb = sh_buffer[i];   
        //     //for(int j=0; j<n_lighting * 3 * kSH_Coeffs; ++j)    
        //     for(int j=0; j<n_lighting * kSH_Coeffs; ++j)    
        //         *ptra++ += *ptrb++;
        //}
        Color colorIn = listVertexDiffuseColor[ vtx ];
        Color colorResult(Color::BLACK);

        for ( int i = 0; i < kSH_Coeffs; ++i )
        {
            colorResult.r_ += otherColorCoeff[ i ].r_;
            colorResult.g_ += otherColorCoeff[ i ].g_;
            colorResult.b_ += otherColorCoeff[ i ].b_;
        }

        colorIn.r_ += colorResult.r_;
        colorIn.g_ += colorResult.g_;
        colorIn.b_ += colorResult.b_;

        listVertexDiffuseColor[ vtx ] = colorIn;

        // deallocate SH buffers  
        //for(int i=1; i<=n_bounces; ++i) 
        //{   
        //    delete[] sh_buffer[i];  
        //}  
    }

    //dbg
    long long elapsed1 = htimer.GetUSec( true );
    SDL_Log( "SH model-%d transfer diffuse vertex build time = %I64d msec\n", modelIdx, elapsed1/1000 );
}

//=============================================================================
//=============================================================================
bool SphericalHarmonic::RaytraceClosestTriangle(const Vector3 &pos, const Vector3 &vec, NvRayResult &result) const
{
#ifndef USE_VARYING_RAYS
    Ray ray(pos, vec);
#else
    #ifdef LARGEST_AXIS_METHOD
    float x = Abs( vec.x_ );
    float y = Abs( vec.y_ );
    float z = Abs( vec.z_ );
    float n = 5.0f;
    float a = Random(  n, n * 2.0f );
    float b = Random( -n, n );
    float c = Random( -n, n );

    Vector3 dir;

    if ( x > y && x > z)
    {
        dir = Vector3( vec.x_ * a, vec.y_ * b, vec.z_ * c ).Normalized();
    }
    else if ( y > x && y > z )
    {
        dir = Vector3( vec.x_ * b, vec.y_ * a, vec.z_ * c ).Normalized();
    }
    else
    {
        dir = Vector3( vec.x_ * b, vec.y_ * c, vec.z_ * a ).Normalized();
    }
    #else
    Vector3 dir = GetRandRayVector();

    while ( dir.DotProduct( vec ) < 0.8f )
    {
        dir = GetRandRayVector();
    }
    #endif
    Ray ray(pos, dir);
#endif

    return GetScene()->GetComponent<StaticModelPoolMgr>()->Raycast( ray, SHORT_RAY_LENGTH, result );
}

//=============================================================================
//=============================================================================
double SphericalHarmonic::Factorial(int index) const
{
    const double dFactorial[ 16 ] = 
    {	
        1.0,             // 0
        1.0,             // 1
        2.0,             // 2 
        6.0,             // 3
        24.0,            // 4
        120.0,           // 5
        720.0,           // 6
        5040.0,          // 7
        40320.0,         // 8
        362880.0,        // 9
        3628800.0,       // 10
        39916800.0,      // 11
        479001600.0,     // 12
        6227020800.0,    // 13
        87178291200.0,   // 14
        1307674368000.0, // 15
    };

    return dFactorial[ index ];
}

//-----------------------------------------------
//-----------------------------------------------
double SphericalHarmonic::P(int l,int m, double x) const
{
    double pmm = 1.0;
    if (m>0){
        double somx2 = sqrt((1.0-x)*(1.0+x));
        double fact = 1.0;
        for (int i = 1; i<=m; i++){
            pmm *= (-fact)*somx2;
            fact += 2.0;
        }
    }
    if (l==m) return pmm;
    double pmmp1 = x * (2.0*m+1.0)*pmm;
    if (l==m+1) return pmmp1;
    double pll = 0.0;
    for (int ll=m+2; ll<=l; ll++){
        pll = (((double)(2.0*ll) -1.0)*x*pmmp1 - ((double)(ll+m) -1.0)*pmm) / (ll-m);
        pmm = pmmp1;
        pmmp1 = pll;
    }
    return pll;
}

//-----------------------------------------------
//-----------------------------------------------
double SphericalHarmonic::K(int l, int m) const
{
    return sqrt( (2.0*l+1.0)*Factorial(l-m) / (4.0*M_PI*Factorial(l+m)) );
}

//-----------------------------------------------
//-----------------------------------------------
double SphericalHarmonic::SH(int l,int m, double theta, double phi) const
{
    // return a point sample of a Spherical Harmonic basis function"
    // l is the band, range [0..N]"
    // m in the range [-l..l]"
    // theta in the range [0..Pi]"
    // phi in the range [0..2*Pi]"
    const double sqrt2 = sqrt(2.0);
    double cosTheta = cos(theta);
    if (m==0) return K(l,0)*P(l,m,cosTheta);
    else if(m>0) return sqrt2*K(l,m)*cos(m*phi)*P(l,m,cosTheta);
    else return sqrt2*K(l,-m)*sin(-m*phi)*P(l,-m,cosTheta);
}

//-----------------------------------------------
//-----------------------------------------------
int SphericalHarmonic::GetIndex(int l, int m) const
{
    return l*(l+1)+m;
}

//=============================================================================
//=============================================================================
bool SphericalHarmonic::ReadCoeffFile(StaticModelData *pStaticModelData)
{
    if ( GetScene()->GetComponent<SHFileUtil>()->GetParameters().loadProjCoeffFromFile )
    {
        return GetScene()->GetComponent<SHFileUtil>()->ReadCoeffFile( pStaticModelData );
    }

    return false;
}

//=============================================================================
//=============================================================================
bool SphericalHarmonic::WriteCoeffFile(StaticModelData *pStaticModelData)
{
    if ( GetScene()->GetComponent<SHFileUtil>()->GetParameters().writeModelProjCoeffFile )
    {
        return GetScene()->GetComponent<SHFileUtil>()->WriteCoeffFile( pStaticModelData );
    }

    return false;
}

//=============================================================================
//=============================================================================
void SphericalHarmonic::GenerateRandRayVectors()
{
    m_iMaxRandRays = 256;

    for ( int i = 0; i < m_iMaxRandRays; ++i )
    {
        float x = Random( -20.0f, 20.0f );
        float y = Random( -20.0f, 20.0f );
        float z = Random( -20.0f, 20.0f );

        m_vRandVector.Push( Vector3(x, y, z).Normalized() );
    }
}

//=============================================================================
//=============================================================================
const Vector3& SphericalHarmonic::GetRandRayVector() const
{
    return m_vRandVector[ Random( 0, m_iMaxRandRays ) ];
}

















