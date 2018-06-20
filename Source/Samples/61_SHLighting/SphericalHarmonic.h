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

#include "SHMisc.h"

using namespace Urho3D;

//=============================================================================
//=============================================================================
class SphericalHarmonic : public Component
{
    OBJECT(SphericalHarmonic);

    typedef double (*SH_polar_fn)(double theta, double phi);
    enum NLightType { kNLights_Max = 8 };

public:

    static void RegisterObject(Context *context);

    SphericalHarmonic(Context *_context);
    ~SphericalHarmonic();

    void CreateSHSamples(int nsamples=100);
    void CreateModelCoeffs();
    void CreateModelPolarCoeff(StaticModelData *pStaticModelData);

    bool ReadCoeffFile(StaticModelData *pStaticModelData);
    bool WriteCoeffFile(StaticModelData *pStaticModelData);

    void SelfTransferSH();

protected:
    // sh lighting samples
    void SetupSphericalSamples();

    // model obj coeffs
    void ProjectPolarFunction(SH_polar_fn fn, double result[]);
    void CreateDiffuseVertexCoeff(int modelIdx);
    void SelfTransferDiffuseVertex(int modelIdx);
    bool RaytraceClosestTriangle(const Vector3 &pos, const Vector3 &vec, NvRayResult &result) const;

    // sh computational methods
    double Factorial(int index) const;
    double P(int l,int m, double x) const;
    double K(int l, int m) const;
    double SH(int l,int m, double theta, double phi) const;
    int GetIndex(int l, int m) const;

    // rand ray vector
    void GenerateRandRayVectors();
    const Vector3& GetRandRayVector() const;

protected:
    int                  m_nSamples;
    int                  m_nSqrtSamples;

    Vector<SHSample>     m_vSHSamples;
    Vector<double>       m_vSHCoeff;

    // static vars used for project functors
    static double       s_X[kNLights_Max];
    static double       s_Y[kNLights_Max];
    static double       s_Z[kNLights_Max];
    static double       s_Brightness[kNLights_Max];
    static int          s_NLights;

    // random ray
    int                 m_iMaxRandRays;
    PODVector<Vector3>  m_vRandVector;

};







