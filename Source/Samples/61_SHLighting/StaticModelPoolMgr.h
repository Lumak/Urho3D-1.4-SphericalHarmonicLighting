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
#include <Urho3D/Core/Object.h>
#include <Urho3D/Math/BoundingBox.h>

#include "SHMisc.h"

namespace Urho3D
{
class Object;
class StaticModel;
class Material;
class Ray;
class Drawable;
class Node;
class Color;
}

namespace FW
{
class NScene;
}

using namespace Urho3D;
using namespace FW;

//=============================================================================
//=============================================================================
class StaticModelPoolMgr : public Component
{
    OBJECT(StaticModelPoolMgr);
public:
    // Register object factory and attributes.
    static void RegisterObject(Context* context);

    StaticModelPoolMgr(Context *context_);
    ~StaticModelPoolMgr();

    void AddStaticModelData(Node *pNode, StaticModel *pStaticModel);
    bool CreateModelVertexColorElement(StaticModel *pStaticModel);
    void SetVertexColorDiffuseTransfer(int itype);

    void CreateBVH();
    bool Raycast(Ray &ray, float fdistance, NvRayResult &results);

    // pool access method
    Vector<StaticModelData>& GetStaticModelList() { return m_vStaticModelPool; }

protected:
    Color GetModelMaterialColor() const;

protected:
    NScene                  *m_pNvScene;
    Vector<StaticModelData>  m_vStaticModelPool;
};

