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

#include "SHConfigure.h"
#include "SHMisc.h"

using namespace Urho3D;

//=============================================================================
//=============================================================================
class SHFileUtil : public Component
{
    OBJECT(SHFileUtil);
public:

    static void RegisterObject(Context *context);

    SHFileUtil(Context *_context);
    ~SHFileUtil();

    void SetParameters(SHConfigParameter &param);
    const SHConfigParameter& GetParameters() const;
    bool ReadCoeffFile(StaticModelData *pStaticModelData);
    bool WriteCoeffFile(StaticModelData *pStaticModelData);
    void WriteSHToModel(StaticModelData *pStaticModelData);
    void SaveVertexColorModel(StaticModel *pStaticModel);

    void DrawStratifiedSHSamples(const Vector<SHSample> &shSamples);

protected:

    SHConfigParameter m_SHConfigParameter;

};


















