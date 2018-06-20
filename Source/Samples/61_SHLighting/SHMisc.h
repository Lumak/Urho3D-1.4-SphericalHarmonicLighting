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

#pragma once

#include <Urho3D/Urho3D.h>
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Resource/Image.h>
#include <Urho3D/Math/Color.h>
#include <vector>

namespace Urho3D
{
class Vector3;
class Vector2;
class Drawable;
class Node;
}

using namespace Urho3D;

//=============================================================================
//=============================================================================
//-------------------------------------
// macros
//-------------------------------------
#define DEL_AND_NULL(x) if(x) { delete x; x = NULL; }

//-------------------------------------
//-------------------------------------
enum SHSizeType
{
    kSH_Bands  = 4,
    kSH_Coeffs = kSH_Bands * kSH_Bands,
};

//-------------------------------------
//-------------------------------------
struct SHSample 
{  
    Vector3 sph;  
    Vector3 vec;  
    double coeff[ kSH_Coeffs ];
};  

//-------------------------------------
//-------------------------------------
enum SHDiffuseTransferType
{
    kSHDifTrans_Off,
    kSHDifTrans_Samples,
    kSHDifTrans_Lighting,
    kSHDifTrans_Shadowed,
    kSHDifTrans_Interreflected,
};

//-------------------------------------
//-------------------------------------
struct IntVector3
{
    IntVector3() : x_(0),y_(0),z_(0){}

    IntVector3(int x, int y, int z) : x_(x),y_(y),z_(z){}

    int x_, y_, z_;
};

//-------------------------------------
//-------------------------------------
struct NvRayResult
{
    NvRayResult() : drawable_(NULL), node_(NULL){}

    Vector3     position_;
    Vector3     normal_;
    float       distance_;
    Drawable*   drawable_;
    Node*       node_;

    // extra vars
    int         poolIdx_;
    int         triIndex_;
};

//-------------------------------------
//-------------------------------------
struct TCoeffs 
{ 
    float shCoeff[ kSH_Coeffs ]; 
};

struct TColor  
{ 
    Color color[ kSH_Coeffs ]; 
};

//-------------------------------------
// static mesh data
//-------------------------------------
struct StaticModelData
{
    // indeces used for raycast
    unsigned begVertex;
    unsigned begTriList;

    // object id's
    Drawable           *drawable;
    Node               *node;

    // geom component list
    PODVector<IntVector3> listTris;
    PODVector<Vector3>    listVerts;
    PODVector<Vector3>    listNormals;
    PODVector<Vector2>    listUVs;

    // coeffs
    TCoeffs shProjectedPolarCoeff;  // model coeff

    // vertex color
    PODVector<TCoeffs>  listVertexUnshadoweCoeff;
    PODVector<TCoeffs>  listVertexShadowCoeff;
    std::vector<std::vector<int> > listVertexSampleOcclude;
    PODVector<Color>   listVertexDiffuseColor;
    
    // material color
    Color               materialColor;
};

//=============================================================================
// misc funcs
//=============================================================================
float AreaOfTriangle(const Vector3 &v0, const Vector3 &v1, const Vector3 &v2);
void ExtractFileDirAndName(const String &strFullPathFilename, String &strDir, String &strFilenameNoExt, char delimiter='-');

//=============================================================================
//=============================================================================
inline float InvLerp(float a, float b, float t)
{ 
    return ( t - a )/( b - a ); 
}

//=============================================================================
//=============================================================================
inline float AngleBetweenVectors(const Vector2& a, const Vector2& b) 
{ 
    return Acos( a.DotProduct(b) / (a.Length() * b.Length() ) ); 
}

// barycentric for 3D and 2D
//=============================================================================
// http://answers.unity3d.com/questions/383804/calculate-uv-coordinates-of-3d-point-on-plane-of-m.html
// describes a barycentric calculation for 3D using proportional area calculation
//=============================================================================
inline Vector3 Barycentric(const Vector3 &v0, const Vector3 &v1, const Vector3 &v2, const Vector3 &vp)
{
    Vector3 bary( M_INFINITY, M_INFINITY, M_INFINITY );

    // edge seg
    Vector3 e0 = v0 - vp;
    Vector3 e1 = v1 - vp;
    Vector3 e2 = v2 - vp;

    float area = (v1 - v0).CrossProduct(v2 - v0).Length();

    if ( area > M_EPSILON )
    {
        // segment area: the subscripts a0, a1, and a2 are 
        // derived from the opposite subscripts of e0, e1, and e2
        float a0 = e1.CrossProduct( e2 ).Length() / area;
        float a1 = e2.CrossProduct( e0 ).Length() / area;
        float a2 = e0.CrossProduct( e1 ).Length() / area;

        bary = Vector3(a0, a1, a2);
    }

    return bary;
}

//=============================================================================
// vecto2 crossproduct
//=============================================================================
inline float CrossProduct(const Vector2 &a, const Vector2 &b)
{
    return Abs( a.x_ * b.y_ - a.y_ * b.x_ );
}

//=============================================================================
// http://answers.unity3d.com/questions/383804/calculate-uv-coordinates-of-3d-point-on-plane-of-m.html
// describes a barycentric calculation for 3D using proportional area calculation
// also works for 2D
//=============================================================================
inline Vector3 Barycentric(const Vector2 &v0, const Vector2 &v1, const Vector2 &v2, const Vector2 &vp)
{
    Vector3 bary( M_INFINITY, M_INFINITY, M_INFINITY );

    // edge seg
    Vector2 e0 = v0 - vp;
    Vector2 e1 = v1 - vp;
    Vector2 e2 = v2 - vp;

    float area = CrossProduct( v1 - v0, v2 - v0 );

    if ( area > M_EPSILON )
    {
        // segment area: the subscripts a0, a1, and a2 are 
        // derived from the opposite subscripts of e0, e1, and e2
        float a0 = CrossProduct( e1, e2 ) / area;
        float a1 = CrossProduct( e2, e0 ) / area;
        float a2 = CrossProduct( e0, e1 ) / area;

        bary = Vector3( a0, a1, a2 );
    }

    return bary;
}

//=============================================================================
// **note: the sum of the three segment areas is equal to the area of triangle and 
// the sum of the barycentric is equal to one, if it's inside the triangle
//=============================================================================
inline bool BaryInsideTriangle(const Vector3 &bary)
{
    return (bary.x_ + bary.y_ + bary.z_) < (1.0f + M_EPSILON);
}

