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

#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/RenderSurface.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Technique.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/Graphics/Texture2D.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include "SHLighting.h"
#include "SphericalHarmonic.h"
#include "StaticModelPoolMgr.h"
#include "VertexUtil.h"
#include "SHFileUtil.h"
#include "SHConfigure.h"

#include <Urho3D/DebugNew.h>
#include <SDL/SDL_log.h>

//=============================================================================
// Spherical Harmonic Lighting based on: 
// http://www.research.scea.com/gdc2003/spherical-harmonic-lighting.pdf
// 
// three types of diffused transfer
// 1 - diffused unshadowed transfer
// 2 - shadowed diffuse transfer (AO)
// 3 - diffuse interreflected transfer
//=============================================================================
DEFINE_APPLICATION_MAIN(RenderToTexture)

//=============================================================================
//=============================================================================
RenderToTexture::RenderToTexture(Context* context) :
    Sample(context)
{
    // component classes
    SphericalHarmonic::RegisterObject( context );
    StaticModelPoolMgr::RegisterObject( context );
    VertexUtil::RegisterObject( context );
    SHFileUtil::RegisterObject( context );

    // Register an object factory for our custom Rotator component so that we can create them to scene nodes
    context->RegisterFactory<Rotator>();
    m_pStaticModelPoolMgr = NULL;
    m_uTicksWait = 0;
    m_SHSceneProcessed = false;
}

//=============================================================================
//=============================================================================
void RenderToTexture::Setup()
{
    // Modify engine startup parameters
    engineParameters_["WindowTitle"] = GetTypeName();
    engineParameters_["LogName"]     = GetSubsystem<FileSystem>()->GetAppPreferencesDir("urho3d", "logs") + GetTypeName() + ".log";
    engineParameters_["FullScreen"]  = false;
    engineParameters_["Headless"]    = false;
    engineParameters_["WindowWidth"] = 1280; 
    engineParameters_["WindowHeight"]= 720;
}

//=============================================================================
//=============================================================================
void RenderToTexture::Start()
{
    // Execute base class startup
    Sample::Start();

    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateInstructions();
    
    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update events
    SubscribeToEvents();
}

//=============================================================================
//=============================================================================
void RenderToTexture::CreateScene()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    // Create the scene in which we move around
    scene_ = new Scene(context_);

    // create sh components
    CreateSHComponents();
    
    m_pDbgRenderer = scene_->CreateComponent<DebugRenderer>();
 
    // Create octree, use also default volume (-1000, -1000, -1000) to (1000, 1000, 1000)
    scene_->CreateComponent<Octree>();
    
    // Create a Zone component for ambient lighting & fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    Zone* zone = zoneNode->CreateComponent<Zone>();
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone->SetAmbientColor(Color(0.6f, 0.6f, 0.6f));
    zone->SetFogColor(Color(0.3f, 0.3f, 0.9f));
    zone->SetFogStart(100.0f);
    zone->SetFogEnd(300.0f);
    
    // camera    
    cameraNode_ = scene_->CreateChild("Camera");
    cameraNode_->SetPosition( Vector3(0.0f, 1.0f, -20.0f) );

    Camera* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);
    
    // light
    Node* plightNode = scene_->CreateChild("lightbulb");
    Light* plight = plightNode->CreateComponent<Light>();

    // point or directional light
    if ( m_pSHFileUtil->GetParameters().usePointLightTest )
    {
        plightNode->SetPosition( Vector3(-5.5f, 4.5f, 0.0f) );
        plightNode->SetScale( 0.4f );
        plight->SetLightType( LIGHT_POINT );
        plight->SetRange( 18.0f );
    }
    else
    {
        plight->SetLightType( LIGHT_DIRECTIONAL );
        Vector3 lightDir = Vector3( 0.1f, -0.9f, 0.4f ).Normalized();
        plightNode->SetDirection( lightDir );
    }

    plight->SetColor(Color(1.0f, 1.0f, 1.0f));
    plight->SetCastShadows( true );
    plight->SetBrightness( 1.0f );

    #ifdef MULTIPLE_LIGHTS_TEST
    // light2
    if ( m_pSHFileUtil->GetParameters().usePointLightTest )
    {
        Node* plightNode2 = scene_->CreateChild("lightbulb");
        Light* plight2 = plightNode2->CreateComponent<Light>();

        plightNode2->SetPosition( Vector3(-1.0f, -5.0f, -1.0f) );
        plight2->SetLightType( LIGHT_POINT );
        plight2->SetRange( 5.0f );
        plight2->SetColor(Color(1.0f, 1.0f, 1.0f));
        plight2->SetCastShadows( true );
        plight2->SetBrightness( 0.3f );
    }
    #endif

    //---------------------------------
    // scene static models
    //---------------------------------
    Model *pModel;

    // wall top
    Node* wallNode1 = scene_->CreateChild("Wall1-top");
    wallNode1->SetPosition( Vector3(0.0f, 5.2f, 0.0f ));
    StaticModel* wallObj1 = wallNode1->CreateComponent<StaticModel>();
    pModel = cache->GetResource<Model>("SHLighting/wall-ls1.mdl");
    wallObj1->SetModel( pModel );
    wallObj1->SetMaterial(cache->GetResource<Material>("SHLighting/Materials/wall.xml") );

    // wall btm
    Node* wallNode2 = scene_->CreateChild("Wall2-btm");
    StaticModel* wallObj2 = wallNode2->CreateComponent<StaticModel>();
    wallNode2->SetPosition( Vector3(0.0f, -5.6f, 0.0f ));
    pModel = cache->GetResource<Model>("SHLighting/wall2.mdl");
    wallObj2->SetModel( pModel );
    wallObj2->SetMaterial(cache->GetResource<Material>("SHLighting/Materials/wall.xml") );

    // wall back
    Node* wallNode3 = scene_->CreateChild("Wall3-bak");
    StaticModel* wallObj3 = wallNode3->CreateComponent<StaticModel>();
    wallNode3->SetPosition( Vector3(0.0f, -0.2f, 4.8f ));
    wallNode3->SetRotation( Quaternion(90.0f, 0.0f, 0.0f ));
    pModel = cache->GetResource<Model>("SHLighting/wall-ls3.mdl");
    wallObj3->SetModel( pModel );
    wallObj3->SetMaterial(cache->GetResource<Material>("SHLighting/Materials/wall.xml") );

    // sphere1
    Node* sphNode1 = scene_->CreateChild("sphere1");
    sphNode1->SetPosition( Vector3(-1.0f, -3.0f, 0.0f ));
    sphNode1->SetRotation( Quaternion(0, 0, 90.0f ) );
    StaticModel* sphObject = sphNode1->CreateComponent<StaticModel>();
    sphObject->SetCastShadows(true);
    pModel = cache->GetResource<Model>("SHLighting/sphere1.mdl");
    sphObject->SetModel( pModel );
    sphObject->SetMaterial(cache->GetResource<Material>("SHLighting/Materials/sphere1.xml") );
    
    // sphere2
    Node* sphNode2 = scene_->CreateChild("sphere2");
    sphNode2->SetPosition( Vector3(-3.6f, -3.4f, -2.6f ));
    sphNode2->SetScale( Vector3(0.8f, 0.8f, 0.8f ));
    StaticModel* sphObject2 = sphNode2->CreateComponent<StaticModel>();
    sphObject2->SetCastShadows(true);
    pModel = cache->GetResource<Model>("SHLighting/sphere2.mdl");
    sphObject2->SetModel( pModel );
    sphObject2->SetMaterial(cache->GetResource<Material>("SHLighting/Materials/sphere2.xml") );

    // torus
    Node* torNode = scene_->CreateChild("torus");
    torNode->SetPosition( Vector3(2.3f, -3.1f, 0.0f ));
    torNode->SetRotation( Quaternion( 0.0f, 0.0f, -50.0f) );
    StaticModel* torObject = torNode->CreateComponent<StaticModel>();
    torObject->SetCastShadows(true);
    torObject->SetModel(cache->GetResource<Model>("SHLighting/torus.mdl"));

    torObject->SetMaterial(cache->GetResource<Material>("SHLighting/Materials/torus.xml") );
}

//=============================================================================
//=============================================================================
void RenderToTexture::CreateSHComponents()
{
    // components
    m_pStaticModelPoolMgr = scene_->CreateComponent<StaticModelPoolMgr>();
    m_pSphericalHarmonic = scene_->CreateComponent<SphericalHarmonic>();
    m_pVertexUtil = scene_->CreateComponent<VertexUtil>();
    m_pSHFileUtil = scene_->CreateComponent<SHFileUtil>();

    // config
    SHConfigParameter shConfig;
    shConfig.usePointLightTest                = true;
    shConfig.writeVertexColorModelFile        = false;
    shConfig.loadProjCoeffFromFile            = false;
    shConfig.writeModelProjCoeffFile          = false;
    shConfig.writeStratifiedSampleToImage    = false;
    shConfig.showRaycastTest                  = false;

    m_pSHFileUtil->SetParameters( shConfig );
}

//=============================================================================
//=============================================================================
void RenderToTexture::SetupSH()
{
    //dbg
    HiresTimer htimer;

    //************************************
    // SH lighting setup process - BEGIN
    //************************************
    // step 1 - pool all models and create BVH
    PoolStaticModels();

    // step 2 - create SH samples
    m_pSphericalHarmonic->CreateSHSamples( 49 );

    // step 3 - create projectpolar coeff for each model
    // ** all models in the scene must be pooled before calling CreateModelCoeffs()
    m_pSphericalHarmonic->CreateModelCoeffs();

    // step 4 - create vertex color element for models missing vc element
    CreateModelVertexColorElement();

    // step 5 - self transfer lights
    m_pSphericalHarmonic->SelfTransferSH();

    //dbg
    long long elapsed1 = htimer.GetUSec( true );
    SDL_Log( "SH total build time = %I64d msec\n", elapsed1/1000 );

    //************************************
    // SH lighting process - END
    //************************************

    m_pTextInstruction->SetText("F4-normal light, F5-diff unshadowed, F6-shadow diff, F7-interreflected");

    // show sh unshadowed transfer by default
    //SetVertexColorDiffuseTransfer( kSHDifTrans_Lighting );

    // ************************
    //dbg - ray test
    if ( m_pSHFileUtil->GetParameters().showRaycastTest )
    {
        const Vector3 start(-6.0f, 4.5f, 0);
        const Vector3 end(-1.0f, -3.4f, 0.0f );
        Vector3 dir = ( end - start ).Normalized();
        Ray ray( start, dir );
        const float castLen = 100.0f;
        HiresTimer htimer;

        // nvray cast
        NvRayResult nvResult;
        m_pStaticModelPoolMgr->Raycast( ray, castLen, nvResult );
        long long elapsed1 = htimer.GetUSec( true );

        // urho raycast
        PODVector<RayQueryResult> results;
        RayOctreeQuery query( results, ray, RAY_TRIANGLE, castLen, DRAWABLE_GEOMETRY );
        htimer.Reset();
        scene_->GetComponent<Octree>()->Raycast( query );
        long long elapsed2 = htimer.GetUSec( true );

        SDL_Log("NVRay time=%I64d, UrhoRay time=%I64d\n", elapsed1, elapsed2);
    }
}

//=============================================================================
//=============================================================================
void RenderToTexture::PoolStaticModels()
{
    PODVector<Node*> listNode;
    scene_->GetChildrenWithComponent( listNode, "StaticModel", true);

    // load static models
    for ( unsigned i = 0; i < listNode.Size(); ++i )
    {
        m_pStaticModelPoolMgr->AddStaticModelData( listNode[ i ], listNode[ i ]->GetComponent<StaticModel>() );
    }

    // create BVH
    m_pStaticModelPoolMgr->CreateBVH();
}

//=============================================================================
//=============================================================================
void RenderToTexture::CreateModelVertexColorElement()
{
    // create missing vertex color element
    Vector<StaticModelData>& statModelList = m_pStaticModelPoolMgr->GetStaticModelList();

    for ( unsigned i = 0; i < statModelList.Size(); ++i )
    {
        StaticModelData &staticModelData = statModelList[ i ];

        if ( m_pVertexUtil->CreateModelVertexColorElement( staticModelData ) )
        {
            // save the model to .mdl file - appends "vc" to the model name
            if ( m_pSHFileUtil->GetParameters().writeVertexColorModelFile )
            {
                m_pSHFileUtil->SaveVertexColorModel( (StaticModel*)staticModelData.drawable );
            }
        }
    }
}

//=============================================================================
//=============================================================================
void RenderToTexture::SetVertexColorDiffuseTransfer(int itype)
{
    Vector<StaticModelData>& statModelList = m_pStaticModelPoolMgr->GetStaticModelList();

    for ( unsigned i = 0; i < statModelList.Size(); ++i )
    {
        StaticModelData &staticModelData = statModelList[ i ];

        if ( m_pVertexUtil->SetVertexColorDiffuseTransfer( staticModelData, itype ) )
        {
            String strMatVtxOff( ".xml" );
            String strMatVtxOn( "-diffvcol.xml" );
            m_pVertexUtil->ChangeModelMaterial( staticModelData, itype, strMatVtxOff, strMatVtxOn );
        }
    }
}

//=============================================================================
//=============================================================================
void RenderToTexture::CreateInstructions()
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    UI* ui = GetSubsystem<UI>();
    Graphics* graphics = GetSubsystem<Graphics>();

    // Construct new Text object, set string to display and font to use
    m_pTextInstruction = ui->GetRoot()->CreateChild<Text>();
    m_pTextInstruction->SetText("Press F9 to build SH lighting");
    m_pTextInstruction->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    m_pTextInstruction->SetColor( Color( 100.0f, 100.0f, 100.0f ));
    
    // Position the text relative to the screen center
    //instructionText->SetHorizontalAlignment(HA_CENTER);
    //instructionText->SetVerticalAlignment(VA_CENTER);
    m_pTextInstruction->SetPosition( 300, graphics->GetHeight()-50);

    m_pTextMode = ui->GetRoot()->CreateChild<Text>();
    m_pTextMode->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    m_pTextMode->SetColor( Color::YELLOW );
    m_pTextMode->SetPosition( 300, graphics->GetHeight()-20);
    m_pTextMode->SetText("");
}

//=============================================================================
//=============================================================================
void RenderToTexture::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    // Move the camera, scale movement with time step
    MoveCamera(timeStep);

    Input* input = GetSubsystem<Input>();

    if ( input->GetKeyPress( KEY_F4 ) )
    {
        SetVertexColorDiffuseTransfer( kSHDifTrans_Off );
    }
    if ( input->GetKeyPress( KEY_F5 ) )
    {
        SetVertexColorDiffuseTransfer( kSHDifTrans_Lighting );
    }
    if ( input->GetKeyPress( KEY_F6 ) )
    {
        SetVertexColorDiffuseTransfer( kSHDifTrans_Shadowed );
    }
    if ( input->GetKeyPress( KEY_F7 ) )
    {
        SetVertexColorDiffuseTransfer( kSHDifTrans_Interreflected );
    }
    if ( input->GetKeyPress( KEY_F9 ) && !m_SHSceneProcessed )
    {
        m_SHSceneProcessed = true;
    }

    if ( m_SHSceneProcessed )
    {
        m_uTicksWait++;

        if ( m_uTicksWait < 10 )
        {
            m_pTextInstruction->SetText("Processing please wait...");
        }

        if ( m_uTicksWait == 10)
        {
            SetupSH();
        }
    }

    // dbg render lights
    PODVector<Node*> listNode;
    scene_->GetChildrenWithComponent( listNode, "Light", true);

    for ( unsigned i = 0; i < listNode.Size(); ++i )
    {
        Node *pLightNode = listNode[ i ];
        Sphere sph( pLightNode->GetPosition(), 0.07f );
        m_pDbgRenderer->AddSphere( sph, Color::WHITE );
    }
}

//=============================================================================
//=============================================================================
void RenderToTexture::SetupViewport()
{
    Renderer* renderer = GetSubsystem<Renderer>();
    
    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

//=============================================================================
//=============================================================================
void RenderToTexture::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;
    
    Input* input = GetSubsystem<Input>();
    
    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;
    
    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);
    
    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
    
    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown('W'))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown('S'))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown('A'))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown('D'))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
}

//=============================================================================
//=============================================================================
void RenderToTexture::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, HANDLER(RenderToTexture, HandleUpdate));
}

//=============================================================================
//=============================================================================
//=============================================================================
Rotator::Rotator(Context* context) :
    LogicComponent(context),
    rotationSpeed_(Vector3::ZERO)
{
    // Only the scene update event is needed: unsubscribe from the rest for optimization
    SetUpdateEventMask(USE_UPDATE);
}

void Rotator::SetRotationSpeed(const Vector3& speed)
{
    rotationSpeed_ = speed;
}

void Rotator::Update(float timeStep)
{
    // Components have their scene node as a member variable for convenient access. Rotate the scene node now: construct a
    // rotation quaternion from Euler angles, scale rotation speed with the scene update time step
    node_->Rotate(Quaternion(rotationSpeed_.x_ * timeStep, rotationSpeed_.y_ * timeStep, rotationSpeed_.z_ * timeStep));
}

