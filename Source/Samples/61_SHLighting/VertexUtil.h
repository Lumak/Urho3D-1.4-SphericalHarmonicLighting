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

namespace Urho3D
{
class StaticModel;
}

using namespace Urho3D;

//=============================================================================
//=============================================================================
class VertexUtil : public Component
{
    OBJECT(VertexUtil);
public:

    static void RegisterObject(Context *context);

    VertexUtil(Context *_context);
    ~VertexUtil();

    bool CreateModelVertexColorElement(StaticModelData &staticModelData);
    bool SetVertexColorDiffuseTransfer(StaticModelData &staticModelData, int itype);
    void ChangeModelMaterial(StaticModelData &staticModelData, int itype, const String &strVtxOffName, const String &strVtxOnName);
    Color GetModelMaterialColor(StaticModel *pStaticModel) const;

protected:

};


















