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

#include "Sample.h"

#include <Urho3D/Scene/LogicComponent.h>

namespace Urho3D
{
class Node;
class Scene;
}

class SphericalHarmonic;
class StaticModelPoolMgr;
class VertexUtil;
class SHFileUtil;

//=============================================================================
//=============================================================================
/// Custom logic component for rotating a scene node.
class Rotator : public LogicComponent
{
    OBJECT(Rotator);
    
public:
    /// Construct.
    Rotator(Context* context);
    
    /// Set rotation speed about the Euler axes. Will be scaled with scene update time step.
    void SetRotationSpeed(const Vector3& speed);
    /// Handle scene update. Called by LogicComponent base class.
    virtual void Update(float timeStep);
    
    /// Return rotation speed.
    const Vector3& GetRotationSpeed() const { return rotationSpeed_; }
    
private:
    /// Rotation speed.
    Vector3 rotationSpeed_;
};


//=============================================================================
//=============================================================================
/// Render to texture example
/// This sample demonstrates:
///     - Creating two 3D scenes and rendering the other into a texture
///     - Creating rendertarget texture and material programmatically
class RenderToTexture : public Sample
{
    OBJECT(RenderToTexture);

public:
    /// Construct.
    RenderToTexture(Context* context);

    /// Setup after engine initialization and before running the main loop.
    virtual void Setup();
    virtual void Start();

private:
    /// Construct the scene content.
    void CreateScene();
    /// Construct an instruction text to the UI.
    void CreateInstructions();
    /// Set up a viewport for displaying the scene.
    void SetupViewport();
    /// Subscribe to application-wide logic update events.
    void SubscribeToEvents();
    /// Read input and moves the camera.
    void MoveCamera(float timeStep);
    /// Handle the logic update event.
    void HandleUpdate(StringHash eventType, VariantMap& eventData);

    void CreateSHComponents();
    void SetupSH();
    void PoolStaticModels();
    void CreateModelVertexColorElement();
    void SetVertexColorDiffuseTransfer(int itype);

protected:
    /// Scene that is rendered to a texture.
    SharedPtr<Scene> rttScene_;
    /// Camera scene node in the render-to-texture scene.
    SharedPtr<Node> rttCameraNode_;

    // sh
    SharedPtr<SphericalHarmonic>   m_pSphericalHarmonic;
    SharedPtr<StaticModelPoolMgr>   m_pStaticModelPoolMgr;
    SharedPtr<SHFileUtil>           m_pSHFileUtil;

    //  text
    SharedPtr<Text>                 m_pTextInstruction;
    SharedPtr<Text>                 m_pTextMode;
    unsigned                        m_uTicksWait;
    bool                            m_SHSceneProcessed;

    // util
    SharedPtr<VertexUtil>           m_pVertexUtil;

    // dbg
    SharedPtr<DebugRenderer>    m_pDbgRenderer;

};


