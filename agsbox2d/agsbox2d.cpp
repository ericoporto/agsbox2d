// agsbox2d.cpp : Defines the entry point for the DLL application.
/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma region Defines_and_Includes

#include "Box2D.h"
#include "core/platform.h"

#define MIN_EDITOR_VERSION 1
#define MIN_ENGINE_VERSION 3

#if AGS_PLATFORM_OS_WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

#if !defined(BUILTIN_PLUGINS)
#define THIS_IS_THE_PLUGIN
#endif

#include "plugin/agsplugin.h"
#include "agsbox2d.h"

#include "AgsNumberInterface.h"
#include "Scale.h"

#include "PointF.h"
#include "AgsWorld.h"
#include "AgsBody.h"
#include "AgsShape.h"
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"
#include "AgsFixture.h"
#include "AgsFixtureArray.h"
#include "AgsRaycastResult.h"
#include "AgsJoint.h"
#include "AgsJointDistance.h"
#include "AgsJointMotor.h"
#include "AgsJointMouse.h"
#include "AgsJointPulley.h"
#include "Book.h"

#include "DebugDraw.h"

#pragma endregion // Defines_and_Includes

#if AGS_PLATFORM_OS_WINDOWS

//==============================================================================

// ***** Design time *****

IAGSEditor *editor; // Editor interface

const char *ourScriptHeader =
"  \r\n"
"enum BodyType { \r\n"
"  eBodyStatic=0, \r\n"
"  eBodyKinematic=1, \r\n"
"  eBodyDynamic=2, \r\n"
"}; \r\n"
" \r\n"
"enum RaycastType { \r\n"
"  eRaycastPassthrough=0, \r\n"
"  eRaycastUntilHit=1, \r\n"
"}; \r\n"
" \r\n"
"enum JointType { \r\n"
"  eJointUnknown=0, \r\n"
"  eJointRevolute=1, \r\n"
"  eJointPrismatic=2, \r\n"
"  eJointDistance=3, \r\n"
"  eJointPulley=4, \r\n"
"  eJointMouse=5, \r\n"
"  eJointGear=6, \r\n"
"  eJointWheel=7, \r\n"
"  eJointWeld=8, \r\n"
"  eJointFriction=9, \r\n"
"  eJointRope=10, \r\n"
"  eJointMotor=11, \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct PointF { \r\n"
"  \r\n"
"  /// Creates a point with float X and Y coordinates. \r\n"
"  import static PointF* Create(float x, float y); // $AUTOCOMPLETESTATICONLY$ \r\n"
"  \r\n"
"  /// Float X coordinate of the point. \r\n"
"  import attribute float X; \r\n"
"  \r\n"
"  /// Float Y coordinate of the point. \r\n"
"  import attribute float Y; \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct Body { \r\n"
"  \r\n"
"  /// Returns true after the body is targeted with AgsBox2D.DestroyBody(). \r\n"
"  readonly import attribute bool IsDestroyed; \r\n"
"  \r\n"
"  /// Set to true to prevent the body from rotating. \r\n"
"  import attribute bool FixedRotation; \r\n"
"  \r\n"
"  /// Set to true if the body is small and moves really fast. \r\n"
"  import attribute bool Bullet; \r\n"
"  \r\n"
"  /// X position of the center of the body as int. \r\n"
"  import attribute int X; \r\n"
"  \r\n"
"  /// Y position of the center of the body as int. \r\n"
"  import attribute int Y; \r\n"
"  \r\n"
"  /// Real X position of the center of the body. \r\n"
"  import attribute float fX; \r\n"
"  \r\n"
"  /// Real Y position of the center of the body. \r\n"
"  import attribute float fY; \r\n"
"  \r\n"
"  /// The angle of the body. Requires FixedRotation be false. \r\n"
"  import attribute float Angle; \r\n"
"  \r\n"
"  /// A value between 0.0 and 1.0 that reduces movement independent of contact. \r\n"
"  import attribute float LinearDamping; \r\n"
"  \r\n"
"  /// Angular drag of the movement that happens independent of contact. \r\n"
"  import attribute float AngularDamping; \r\n"
"  \r\n"
"  /// The angular velocity of the body. \r\n"
"  import attribute float AngularVelocity; \r\n"
"  \r\n"
"  /// The rotational inertia of the body. \r\n"
"  import attribute float Inertia; \r\n"
"  \r\n"
"  /// X vector of the linear velocity of the body. \r\n"
"  readonly import attribute float LinearVelocityX; \r\n"
"  \r\n"
"  /// Y vector of the linear velocity of the body. \r\n"
"  readonly import attribute float LinearVelocityY; \r\n"
"  \r\n"
"  /// Sets the body linear velocity vector. \r\n"
"  import void SetLinearVelocity(float fx, float fy); \r\n"
"  \r\n"
"  /// Applies a force on the body from it's center, with the specified vector. \r\n"
"  import void ApplyForce(float fx, float fy); \r\n"
"  \r\n"
"  /// Applies an angular impulse on the body. \r\n"
"  import void ApplyAngularImpulse(float impulse); \r\n"
"  \r\n"
"  /// Applies an impulse from the body center with the specified vector. \r\n"
"  import void ApplyLinearImpulse(float intensity_x, float intensity_y); \r\n"
"  \r\n"
"  /// Applies a rotational force on the body. \r\n"
"  import void ApplyTorque(float torque); \r\n"
"  \r\n"
"  /// Returns true if the body is in contact with other body when evaluated. \r\n"
"  import bool IsTouching(Body* other); \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct FixtureArray; \r\n"
"builtin managed struct RaycastResult; \r\n"
" \r\n"
"builtin managed struct World { \r\n"
"  \r\n"
"  /// Advances one step of time dt in seconds of the simulation. \r\n"
"  import void Step(float dt, int velocityIteractions = 8, int positionIteractions = 3); \r\n"
"  \r\n"
"  /// Returns a sprite with debug data. Set as GUI Background over screen for debugging your physics. \r\n"
"  import int GetDebugSprite(int camera_x = 0, int camera_y = 0); \r\n"
"  \r\n"
"  /// Returns array of fixtures which their bounding boxes are overlapped by the supplied box. \r\n"
"  import FixtureArray* BoundingBoxQuery(float lower_x, float lower_y, float upper_x, float upper_y); \r\n"
"  \r\n"
"  /// Returns RaycastResult with fixtures hit by a line, along with the hit normals. Raycast can also stop at a fixture or specific fixtures. \r\n"
"  import RaycastResult* Raycast(float x0, float y0, float x1, float y1, RaycastType rc_type = 0, FixtureArray* stopping_fixtures = null); \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct ShapeCircle; \r\n"
"builtin managed struct ShapeRectangle; \r\n"
" \r\n"
"builtin managed struct Shape { \r\n"
"  \r\n"
"  /// If this shape is a circle, returns the ShapeCircle interface; otherwise null. \r\n"
"  readonly import attribute ShapeCircle* AsCircle;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"  /// If this shape is a rectangle, returns the ShapeRectangle interface; otherwise null. \r\n"
"  readonly import attribute ShapeRectangle* AsRectangle;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct ShapeCircle extends Shape { \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct ShapeRectangle extends Shape { \r\n"
"  \r\n"
"  import attribute float fWidth; \r\n"
"  import attribute float fHeight; \r\n"
"  import attribute int Height; \r\n"
"  import attribute int Width; \r\n"
"  \r\n"
"  /// Array of the local x position of the corners of the rectangle. \r\n"
"  readonly import attribute float PointsfX[]; \r\n"
"  \r\n"
"  /// Array of the local y position of the corners of the rectangle. \r\n"
"  readonly import attribute float PointsfY[]; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct Fixture { \r\n"
"  \r\n"
"  /// Usually a value between 0.0 and 1.0 to make objects slide realistically. \r\n"
"  import attribute float Friction; \r\n"
"  \r\n"
"  /// Used to compute the mass, prefer similar densities to all your fixtures. \r\n"
"  import attribute float Density; \r\n"
"  \r\n"
"  /// Usually a value between 0.0 and 1.0 to make objects bounce. \r\n"
"  import attribute float Restitution; \r\n"
"  \r\n"
"  /// Returns Bodyif it's defined for this fixture, otherwise null. \r\n"
"  readonly import attribute Body* Body; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"managed struct FixtureArray { \r\n"
"  \r\n"
"  /// Gets the size of the array \r\n"
"  import attribute int Size; \r\n"
"  \r\n"
"  /// Access a Fixture directly \r\n"
"  import attribute Fixture* Items[]; \r\n"
"  \r\n"
"  /// Creates a new array of Fixtures.\r\n"
"  import static FixtureArray* Create (); // $AUTOCOMPLETESTATICONLY$\r\n"
"  \r\n"
"  /// Copies an existing array.\r\n"
"  import FixtureArray* Copy (); \r\n"
"  \r\n"
"  /// Clears all values in the array.\r\n"
"  import void Clear (); \r\n"
"  \r\n"
"  /// Returns true when the array is empty.\r\n"
"  import bool IsEmpty (); \r\n"
"  \r\n"
"  /// Removes specified value(s) from the array.\r\n"
"  import void Erase (int pos, int number = 1); \r\n"
"  \r\n"
"  /// Inserts a value at a specified place in the array.\r\n"
"  import void Insert (int pos, Fixture* fixture); \r\n"
"  \r\n"
"  /// Removes the last item of the array and returns it.\r\n"
"  import Fixture* Pop (); \r\n"
"  \r\n"
"  /// Adds the specified value to the end of the array.\r\n"
"  import void Push (Fixture* fixture); \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct RaycastResult{ \r\n"
"  \r\n"
"  /// Gets the length of the result so you don't access a invalid position on any array. \r\n"
"  readonly import attribute int Length; \r\n"
"  \r\n"
"  /// Position X where the raycast hit a fixture. \r\n"
"  readonly import attribute float PointX[]; \r\n"
"  \r\n"
"  /// Position Y where the raycast hit a fixture. \r\n"
"  readonly import attribute float PointY[]; \r\n"
"  \r\n"
"  /// X coordinate of a vector representing a normal to the hit. \r\n"
"  readonly import attribute float NormalX[]; \r\n"
"  \r\n"
"  /// Y coordinate of a vector representing a normal to the hit. \r\n"
"  readonly import attribute float NormalY[]; \r\n"
"  \r\n"
"  /// A proportion value between 0 and 1 of the position of raycast line where the hit occurred. \r\n"
"  readonly import attribute float Fraction[]; \r\n"
"  \r\n"
"  /// Read a Fixture that was hit. \r\n"
"  readonly import attribute Fixture* Fixture[]; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct JointDistance; \r\n"
"builtin managed struct JointMotor; \r\n"
"builtin managed struct JointMouse; \r\n"
"builtin managed struct JointPulley; \r\n"
" \r\n"
"builtin managed struct Joint { \r\n"
"  \r\n"
"  /// If this joint is a distance joint, returns the JointDistance interface; otherwise null. \r\n"
"  readonly import attribute JointDistance* AsDistance;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"  /// If this joint is a motor joint, returns the JointMotor interface; otherwise null. \r\n"
"  readonly import attribute JointMotor* AsMotor;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"  /// If this joint is a mouse joint, returns the JointMouse interface; otherwise null. \r\n"
"  readonly import attribute JointMouse* AsMouse;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"  /// If this joint is a pulley joint, returns the JointPulley interface; otherwise null. \r\n"
"  readonly import attribute JointPulley* AsPulley;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"  /// If this joint is valid, returns true. \r\n"
"  readonly import attribute bool IsValid; \r\n"
"  \r\n"
"  /// If this joint is active, returns true. \r\n"
"  readonly import attribute bool IsActive; \r\n"
"  \r\n"
"  /// Returns Body A if it's defined, otherwise null, for this joint. \r\n"
"  readonly import attribute Body* BodyA; \r\n"
"  \r\n"
"  /// Returns Body B if it's defined, otherwise null, for this joint. \r\n"
"  readonly import attribute Body* BodyB; \r\n"
"  \r\n"
"  /// Returns this joint type. \r\n"
"  readonly import attribute JointType Type;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct JointDistance extends Joint { \r\n"
"  \r\n"
"  /// The equilibrium distance between the two Bodies. \r\n"
"  import attribute float Length; \r\n"
"  \r\n"
"  /// The damping ratio, typically between 0 and 1. At 1, the damping is critical. \r\n"
"  import attribute float DampingRatio; \r\n"
"  \r\n"
"  /// The frequency of a harmonic oscillator. Should be smaller than half the frame rate. \r\n"
"  import attribute float Frequency; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct JointMotor extends Joint { \r\n"
"  \r\n"
"  /// Sets the target linear offset between the two bodies the joint is attached to. \r\n"
"  import void SetLinearOffset(float fx, float fy); \r\n"
"  \r\n"
"  /// The target linear offset X axis between the two bodies the joint is attached to. \r\n"
"  import attribute float LinearOffsetX; \r\n"
"  \r\n"
"  /// The target linear offset Y axis between the two bodies the joint is attached to. \r\n"
"  import attribute float LinearOffsetY; \r\n"
"  \r\n"
"  /// The target angular offset between the two bodies the joint is attached to. \r\n"
"  import attribute float AngularOffset; \r\n"
"  \r\n"
"  /// The Maximum Force applied to reach target position. \r\n"
"  import attribute float MaxForce; \r\n"
"  \r\n"
"  /// The Maximum Torque applied to reach target rotation. \r\n"
"  import attribute float MaxTorque; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct JointMouse extends Joint { \r\n"
"  \r\n"
"  /// Sets the target point. \r\n"
"  import void SetTarget(float fx, float fy); \r\n"
"  \r\n"
"  /// The target point X axis. \r\n"
"  readonly import attribute float TargetX; \r\n"
"  \r\n"
"  /// The target point Y axis. \r\n"
"  readonly import attribute float TargetY; \r\n"
"  \r\n"
"  /// The damping ratio, typically between 0 and 1. At 1, the damping is critical. \r\n"
"  import attribute float DampingRatio; \r\n"
"  \r\n"
"  /// The frequency of a harmonic oscillator. Should be smaller than half the frame rate. \r\n"
"  import attribute float Frequency; \r\n"
"  \r\n"
"  /// The Maximum Force applied to reach target position. \r\n"
"  import attribute float MaxForce; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin managed struct JointPulley extends Joint { \r\n"
"  \r\n"
"  /// The current length of the segment attached to the first body. \r\n"
"  readonly import attribute float LengthA; \r\n"
"  \r\n"
"  /// The current length of the segment attached to the second body. \r\n"
"  readonly import attribute float LengthB; \r\n"
"  \r\n"
"  /// The pulley ratio. \r\n"
"  readonly import attribute float Ratio; \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"builtin struct AgsBox2D { \r\n"
"  \r\n"
"  /// Sets how many pixels equals to a meter. Doesn't apply retroactively. \r\n"
"  import static void SetMeter(float meter); \r\n"
"  \r\n"
"  /// Get how many pixels equals to a meter. Prefer moving objects between 0.1 and 10 meters. \r\n"
"  import static float GetMeter(); \r\n"
"  \r\n"
"  /// Create a World, and set it's gravity vector. Positive Y acceleration goes downwards. \r\n"
"  import static World* CreateWorld(float gravityX, float gravityY); \r\n"
"  \r\n"
"  /// Creates a body at world x,y position, with type eBodyStatic, eBodyDynamic or eBodyKinematic. \r\n"
"  import static Body* CreateBody(World* world,  float x, float y, BodyType bodytype); \r\n"
"  \r\n"
"  /// Removes a body from the world, and marks it with the property IsDestroyed true. \r\n"
"  import static void DestroyBody(World* world, Body* body);  \r\n"
"  \r\n"
"  /// Create Rectangle Shape with specified size. You can change local shape pivot from center. \r\n"
"  import static Shape* CreateRectangleShape(float w,  float h,  float x=0, float y=0); \r\n"
"  \r\n"
"  /// Create Circle Shape with specified radius. You can change local shape pivot from center. \r\n"
"  import static Shape* CreateCircleShape(float radius,  float x=0, float y=0); \r\n"
"  \r\n"
"  /// Adds a shape to a body, and a specified density. Shape is copyied to body. \r\n"
"  import static Fixture* CreateFixture(Body* body, Shape* shape, float density=0); \r\n"
"  \r\n"
"  /// Create Distance Joint, pass anchors on bodies A and B using world coordinates. \r\n"
"  import static Joint* CreateDistanceJoint(Body* bodyA, Body* bodyB, float a_x, float a_y, float b_x, float b_y, bool collideConnected = 0); \r\n"
"  \r\n"
"  /// Create Motor Joint. \r\n"
"  import static Joint* CreateMotorJoint(Body* bodyA, Body* bodyB, float correction_factor,  bool collideConnected = 0); \r\n"
"  \r\n"
"  /// Create Mouse Joint between body and a target point. \r\n"
"  import static Joint* CreateMouseJoint(Body* bodyA, float x, float y); \r\n"
"  \r\n"
"  /// Create Pulley Joint. \r\n"
"  import static Joint* CreatePulleyJoint(Body* bodyA, Body* bodyB, PointF* groundAnchorA, PointF* groundAnchorB, PointF* localAnchorA, PointF* localAnchorB, float ratio, bool collideConnected = 0); \r\n"
"  \r\n"
"  /// Removes a joint from the world, and marks it as invalid. \r\n"
"  import static void DestroyJoint(World* world, Joint* joint);  \r\n"
"  \r\n"
"}; \r\n";


//------------------------------------------------------------------------------

LPCSTR AGS_GetPluginName()
{
        return ("agsbox2d");
}

//------------------------------------------------------------------------------

int AGS_EditorStartup(IAGSEditor *lpEditor)
{
        // User has checked the plugin to use it in their game

        // If it's an earlier version than what we need, abort.
        if (lpEditor->version < MIN_EDITOR_VERSION)
                return (-1);

        editor = lpEditor;
        editor->RegisterScriptHeader(ourScriptHeader);

        // Return 0 to indicate success
        return (0);
}

//------------------------------------------------------------------------------

void AGS_EditorShutdown()
{
        // User has un-checked the plugin from their game
        editor->UnregisterScriptHeader(ourScriptHeader);
}

//------------------------------------------------------------------------------

void AGS_EditorProperties(HWND parent)                        //*** optional ***
{
        // User has chosen to view the Properties of the plugin
        // We could load up an options dialog or something here instead
	MessageBox(parent,
             L"agsbox2d v0.2.0 By eri0o",
             L"About",
         MB_OK | MB_ICONINFORMATION);

}

//------------------------------------------------------------------------------

int AGS_EditorSaveGame(char *buffer, int bufsize)             //*** optional ***
{
        // Called by the editor when the current game is saved to disk.
        // Plugin configuration can be stored in [buffer] (max [bufsize] bytes)
        // Return the amount of bytes written in the buffer
        return (0);
}

//------------------------------------------------------------------------------

void AGS_EditorLoadGame(char *buffer, int bufsize)            //*** optional ***
{
        // Called by the editor when a game is loaded from disk
        // Previous written data can be read from [buffer] (size [bufsize]).
        // Make a copy of the data, the buffer is freed after this function call.
}

//==============================================================================

#endif  //AGS_PLATFORM_OS_WINDOWS

// ***** Run time *****

// Engine interface

//------------------------------------------------------------------------------

//define engine
IAGSEngine* engine;

AgsDebugDraw debugDraw;  // Declare an instance of our DebugDraw class so we can actually use it

AgsBody* FindAgsBodyFromB2Body(b2World* world, int32 world_id, b2Body* b2body) {
    // body was null in the first place
    if(b2body == nullptr)
        return nullptr;

    int32 b2body_id = Book::b2BodyToID(world_id,b2body);

    // body is not from this world
    if(b2body_id < 0)
        return nullptr;

    // we found an already registered AgsBody
    AgsBody* agsbody = Book::b2bodyIDtoAgsBody(b2body_id, world_id);
    if(agsbody != nullptr)
        return agsbody;

    // we didn't find an already registered AgsBody, so let's create one
    AgsBody* body = new AgsBody();
    body->ID = engine->RegisterManagedObject(body, &AgsBody_Interface);
    body->B2BodyID = b2body_id;
    body->World = Book::IDtoAgsWorld(world_id);

    Book::RegisterAgsBody(body->ID, body);
    return body;
}

AgsFixture* FindAgsFixtureFromB2Fixture(b2World* world, int32 world_id, b2Fixture* b2fixture) {
    // fixture was null in the first place
    if(b2fixture == nullptr)
        return nullptr;

    int32 b2fixture_id = Book::b2FixtureToID(world_id,b2fixture);

    // fixture is not from this world
    if(b2fixture_id < 0)
        return nullptr;

    // we found an already registered AgsFixture
    AgsFixture* agsfixture = Book::b2FixtureIDtoAgsFixture(b2fixture_id, world_id);
    if(agsfixture != nullptr)
        return agsfixture;

    // we didn't find an already registered AgsFixture, so let's create one
    AgsFixture* fixture = new AgsFixture(world_id,  Book::b2BodyToID(world_id, b2fixture->GetBody()), b2fixture_id);
    fixture->ID = engine->RegisterManagedObject(fixture, &AgsFixture_Interface);

    Book::RegisterAgsFixture(fixture->ID, fixture);
    return fixture;
}

//-----------------------------------------------------------------------------
#pragma region PointF_ScriptAPI

PointF* PointF_Create(uint32_t x, uint32_t y){
    float32 fx = ToNormalFloat(x);
    float32 fy = ToNormalFloat(y);
    PointF* point_f = new PointF(fx, fy);
    engine->RegisterManagedObject(point_f, &PointF_Interface);
    return  point_f;
}

int32 PointF_GetY(PointF* self) {
    return ToAgsFloat(self->GetY());
}

void PointF_SetX(PointF* self, uint32_t x) {
    float32 fx = ToNormalFloat(x);

    self->SetX(fx);
}

uint32_t PointF_GetX(PointF* self) {
    return ToAgsFloat(self->GetX());
}

void PointF_SetY(PointF* self, uint32_t y) {
    float32 fy = ToNormalFloat(y);

    self->SetX(fy);
}

#pragma endregion // PointF_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region agsbox2d_ScriptAPI

void agsbox2d_SetMeter(uint32_t meter) {
	Scale::SetMeter(ToNormalFloat(meter));
}

uint32_t agsbox2d_GetMeter() {
	return ToAgsFloat(Scale::GetMeter());
}

AgsWorld* agsbox2d_newWorld(uint32_t gravityX, uint32_t gravityY) {
	float32 gx = ToNormalFloat(gravityX);
	float32 gy = ToNormalFloat(gravityY);

	AgsWorld* world = new AgsWorld(gx, gy);

	world->ID = engine->RegisterManagedObject(world, &AgsWorld_Interface);
	Book::RegisterAgsWorld(world->ID, world);

	return world;
}

AgsBody* agsbox2d_newBody(AgsWorld* world, uint32_t x, uint32_t y, uint32_t bodytype) {
	float32 bx = ToNormalFloat(x);
	float32 by = ToNormalFloat(y);
	b2BodyType bt;
	if (bodytype == 0)
		bt = b2_staticBody;
	else if (bodytype == 1)
		bt = b2_kinematicBody;
	else
		bt = b2_dynamicBody;

	AgsBody* body = world->NewBody(bx, by, bt);

	body->ID = engine->RegisterManagedObject(body, &AgsBody_Interface);
	Book::RegisterAgsBody(body->ID, body);

	int b2body_id = Book::GetNewBodyID(world->ID);
	Book::RegisterBodyFromWorld(body->GetB2AgsBody(), b2body_id, world->ID);

	return body;
}

void agsbox2d_DestroyBody(AgsWorld* world, AgsBody* body) {
	if (world == nullptr)
		return;

	world->DestroyBody(body);
}

AgsShape* agsbox2d_newRectangleShape(uint32_t w, uint32_t h, uint32_t x, uint32_t y) {
	float32 fx, fy, fw, fh;

	if (x == 0)
		fx = 0.0f;
	else
		fx = ToNormalFloat(x);

	if (y == 0)
		fy = 0.0f;
	else
		fy = ToNormalFloat(y);

	fw = ToNormalFloat(w);
	fh = ToNormalFloat(h);

	AgsShape* shape = new AgsShape(new AgsShapeRect(fw, fh, fx, fy));

	shape->ID = engine->RegisterManagedObject(shape, &AgsShape_Interface);
	Book::RegisterAgsShape(shape->ID, shape);

	return shape;
}


AgsShape* agsbox2d_newCircleShape(uint32_t radius, uint32_t x, uint32_t y) {
	float32 fx = ToNormalFloat(x);
	float32 fy = ToNormalFloat(y);
	float32 fradius = ToNormalFloat(radius);

	AgsShape* shape = new AgsShape(new AgsShapeCircle(fradius));

	shape->ID = engine->RegisterManagedObject(shape, &AgsShape_Interface);
	Book::RegisterAgsShape(shape->ID, shape);

	return shape;
}

AgsFixture* agsbox2d_newFixture(AgsBody* body, AgsShape* shape, uint32_t density) {
	float32 fdensity;
	if (density == 0) {
		fdensity = 0.0f;
	}
	else {
		fdensity = ToNormalFloat(density);
	}

	AgsFixture* fixture = new AgsFixture(body, shape, fdensity);

	fixture->ID = engine->RegisterManagedObject(fixture, &AgsFixture_Interface);
	Book::RegisterAgsFixture(fixture->ID, fixture);

    int b2fixture_id = Book::GetNewFixtureID(body->World->ID);
    Book::RegisterFixtureFromWorld(fixture->GetB2AgsFixture(), b2fixture_id, body->World->ID);

	return fixture;
}

AgsJoint* agsbox2d_newDistanceJoint(AgsBody* agsbody_a, AgsBody* agsbody_b,
                                    uint32_t a_x, uint32_t a_y, uint32_t b_x, uint32_t b_y,
                                    int32 collide_connected) {
    float32 f_x_a = ToNormalFloat(a_x);
    float32 f_y_a = ToNormalFloat(a_y);
    float32 f_x_b = ToNormalFloat(b_x);
    float32 f_y_b = ToNormalFloat(b_y);
    bool bcollide_connected = collide_connected != 0;
    AgsWorld* world = Book::IDtoAgsWorld(agsbody_a->World->ID);

    AgsJointDistance* agsJointDistance = new AgsJointDistance(
            world, agsbody_a, agsbody_b, f_x_a, f_y_a, f_x_b, f_y_b, bcollide_connected);

    int b2joint_id = Book::GetNewJointID(world->ID);
    agsJointDistance->b2Joint_ID = b2joint_id;

    AgsJoint* joint = new AgsJoint(agsJointDistance);

    joint->ID = engine->RegisterManagedObject(joint, &AgsJoint_Interface);
    Book::RegisterAgsJoint(joint->ID, joint);

    joint->b2Joint_ID = b2joint_id;
    Book::RegisterJointFromWorld(joint->GetB2AgsJoint(), b2joint_id, world->ID);

    return joint;
}

AgsJoint* agsbox2d_newMotorJoint(AgsBody* agsbody_a, AgsBody* agsbody_b,
                                 uint32_t correction_factor,
                                 int32 collide_connected) {

    float32 fcorrection_factor = ToNormalFloat(correction_factor);
    bool bcollide_connected = collide_connected != 0;
    AgsWorld* world = Book::IDtoAgsWorld(agsbody_a->World->ID);

    AgsJointMotor* agsJointMotor = new AgsJointMotor(
            world, agsbody_a, agsbody_b, fcorrection_factor, bcollide_connected);

    int b2joint_id = Book::GetNewJointID(world->ID);
    agsJointMotor->b2Joint_ID = b2joint_id;

    AgsJoint* joint = new AgsJoint(agsJointMotor);

    joint->ID = engine->RegisterManagedObject(joint, &AgsJoint_Interface);
    Book::RegisterAgsJoint(joint->ID, joint);

    joint->b2Joint_ID = b2joint_id;
    Book::RegisterJointFromWorld(joint->GetB2AgsJoint(), b2joint_id, world->ID);

    return joint;
}

AgsJoint* agsbox2d_newMouseJoint(AgsBody* agsbody_a,
                                uint32_t x, uint32_t y) {
    float32 fx = ToNormalFloat(x);
    float32 fy = ToNormalFloat(y);
    AgsWorld* world = Book::IDtoAgsWorld(agsbody_a->World->ID);

    AgsJointMouse* agsJointMouse = new AgsJointMouse(
            world, agsbody_a, fx, fy);

    int b2joint_id = Book::GetNewJointID(world->ID);
    agsJointMouse->b2Joint_ID = b2joint_id;

    AgsJoint* joint = new AgsJoint(agsJointMouse);

    joint->ID = engine->RegisterManagedObject(joint, &AgsJoint_Interface);
    Book::RegisterAgsJoint(joint->ID, joint);

    joint->b2Joint_ID = b2joint_id;
    Book::RegisterJointFromWorld(joint->GetB2AgsJoint(), b2joint_id, world->ID);

    return joint;
}

AgsJoint* agsbox2d_newPulleyJoint(AgsBody* agsbody_a, AgsBody* agsbody_b,
                                  PointF* ground_anchor_a, PointF* ground_anchor_b,
                                  PointF* anchor_a, PointF* anchor_b,
                                  uint32_t ratio, int32 collide_connected) {
    float32 fground_anchor_a_x = ground_anchor_a->GetX(); // ToNormalFloat(ground_anchor_a_x);
    float32 fground_anchor_a_y = ground_anchor_a->GetY(); // ToNormalFloat(ground_anchor_a_y);
    float32 fground_anchor_b_x = ground_anchor_b->GetX(); // ToNormalFloat(ground_anchor_b_x);
    float32 fground_anchor_b_y = ground_anchor_b->GetY(); // ToNormalFloat(ground_anchor_b_y);
    float32 fanchor_a_x = anchor_a->GetX(); // ToNormalFloat(anchor_a_x);
    float32 fanchor_a_y = anchor_a->GetY(); // ToNormalFloat(anchor_a_y);
    float32 fanchor_b_x = anchor_b->GetX(); // ToNormalFloat(anchor_b_x);
    float32 fanchor_b_y = anchor_b->GetY(); // ToNormalFloat(anchor_b_y);
    float32 f_ratio = ToNormalFloat(ratio);
    bool b_collide_connected = collide_connected != 0;
    AgsWorld* world = Book::IDtoAgsWorld(agsbody_a->World->ID);

    AgsJointPulley* agsJointPulley = new AgsJointPulley(
            world, agsbody_a, agsbody_b,
            fground_anchor_a_x, fground_anchor_a_y, fground_anchor_b_x, fground_anchor_b_y,
            fanchor_a_x, fanchor_a_y, fanchor_b_x, fanchor_b_y,
            f_ratio, b_collide_connected);

    AgsJoint* joint = new AgsJoint(agsJointPulley);

    int b2joint_id = Book::GetNewJointID(world->ID);
    agsJointPulley->b2Joint_ID = b2joint_id;

    joint->ID = engine->RegisterManagedObject(joint, &AgsJoint_Interface);
    Book::RegisterAgsJoint(joint->ID, joint);

    joint->b2Joint_ID = b2joint_id;
    Book::RegisterJointFromWorld(joint->GetB2AgsJoint(), b2joint_id, world->ID);

    return joint;
}

void agsbox2d_DestroyJoint(AgsWorld* world, AgsJoint* joint) {
    if (world == nullptr || joint == nullptr )
        return;

    world->DestroyJoint(joint);
}

#pragma endregion // agsbox2d_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsWorld_ScriptAPI

void AgsWorld_Step(AgsWorld* self, uint32_t dt, int32 velocityIterations, int32 positionIterations) {
	float32 fdt = ToNormalFloat(dt);

	if (velocityIterations <= 0) velocityIterations = 8;
	if (positionIterations <= 0) velocityIterations = 3;

	self->Step(fdt, velocityIterations, positionIterations);
}

int32 AgsWorld_GetDebugSprite(AgsWorld* self, int32 camera_x, int32 camera_y) {
	debugDraw.ClearSprite();
	debugDraw.GetSurfaceForDebugDraw(camera_x, camera_y);
	debugDraw.SetFlags(b2Draw::e_centerOfMassBit | b2Draw::e_shapeBit | b2Draw::e_jointBit | b2Draw::e_pairBit);
	self->B2AgsWorld->SetDebugDraw(&debugDraw);
	self->B2AgsWorld->DrawDebugData();
	debugDraw.ReleaseSurfaceForDebugDraw();
	return debugDraw.GetDebugSprite();
}

AgsFixtureArray* AgsWorld_BoundingBoxQuery(AgsWorld* self, uint32_t lx, uint32_t ly, uint32_t ux, uint32_t uy) {
    float32 f_lx = ToNormalFloat(lx);
    float32 f_ly = ToNormalFloat(ly);
    float32 f_ux = ToNormalFloat(ux);
    float32 f_uy = ToNormalFloat(uy);

    AgsFixtureArray* agsFixtureArray = self->BoundingBoxQuery(f_lx,f_ly,f_ux,f_uy);
    if(agsFixtureArray == nullptr) {
        return nullptr;
    }

    agsFixtureArray->ID = engine->RegisterManagedObject(agsFixtureArray, &AgsFixtureArray_Interface);
    Book::RegisterAgsFixtureArray(agsFixtureArray->ID, agsFixtureArray);
    return agsFixtureArray;
}

AgsRaycastResult* AgsWorld_Raycast(AgsWorld* self, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, int32 raycast_type, AgsFixtureArray* agsFixtureArray) {
    float32 f_x0 = ToNormalFloat(x0);
    float32 f_y0 = ToNormalFloat(y0);
    float32 f_x1 = ToNormalFloat(x1);
    float32 f_y1 = ToNormalFloat(y1);

    if(agsFixtureArray == 0) agsFixtureArray = nullptr;

    AgsRaycastResult* agsRaycastResult = self->RaycastQuery(f_x0, f_y0, f_x1, f_y1, (RaycastType) raycast_type, agsFixtureArray);
    if(agsRaycastResult == nullptr) {
        return nullptr;
    }

    engine->RegisterManagedObject(agsRaycastResult, &AgsRaycastResult_Interface);
    return agsRaycastResult;
}

#pragma endregion // AgsWorld_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsRaycastResult_ScriptAPI

int32 AgsRaycastResult_GetLength(AgsRaycastResult* self){
    return self->GetLength();
}

uint32_t AgsRaycastResult_GetPointX(AgsRaycastResult* self, int32 i) {
    return ToAgsFloat(self->GetPointX(i));
}

uint32_t AgsRaycastResult_GetPointY(AgsRaycastResult* self, int32 i) {
    return ToAgsFloat(self->GetPointY(i));
}

uint32_t AgsRaycastResult_GetNormalX(AgsRaycastResult* self, int32 i) {
    return ToAgsFloat(self->GetNormalX(i));
}

uint32_t AgsRaycastResult_GetNormalY(AgsRaycastResult* self, int32 i) {
    return ToAgsFloat(self->GetNormalY(i));
}

uint32_t AgsRaycastResult_GetFraction(AgsRaycastResult* self, int32 i) {
    return ToAgsFloat(self->GetFraction(i));
}

AgsFixture* AgsRaycastResult_GetFixture(AgsRaycastResult* self, int32 i) {
    if ((i < 0) || (i >= self->GetLength()))
        return nullptr;

    AgsFixtureData fad = self->GetFixtureData(i);
    AgsFixture* agsFixture = FindAgsFixtureFromB2Fixture(
            Book::IDtoAgsWorld(fad.WorldID)->B2AgsWorld,
            fad.WorldID, Book::IDtoB2Fixture(fad.WorldID, fad.B2FixtureID));
    return agsFixture;
}

#pragma endregion // AgsRaycastResult_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsBody_ScriptAPI

int32 AgsBody_IsDestroyed(AgsBody* self) {
	if (self->GetIsDestroyed()) {
		return 1;
	}
	return 0;
}

void AgsBody_SetFixedRotation(AgsBody* self, int32 fixed) {
	if(fixed == 1)
		self->SetFixedRotation(true);
	self->SetFixedRotation(false);
}

int32 AgsBody_GetFixedRotation(AgsBody* self) {
	if (self->GetFixedRotation())
		return 1;
	return 0;
}

void AgsBody_SetBullet(AgsBody* self, int32 bullet) {
	if (bullet == 1)
		self->SetIsBullet(true);
	self->SetIsBullet(false);
}

int32 AgsBody_GetBullet(AgsBody* self) {
	if (self->GetIsBullet())
		return 1;
	return 0;
}

void AgsBody_SetIntPositionX(AgsBody* self, int32 x) {
	float32 fx = (float32) x;

	self->SetPosX(fx);
}

int32 AgsBody_GetIntPositionX(AgsBody* self) {
	return (int32) self->GetPosX();
}

void AgsBody_SetIntPositionY(AgsBody* self, int32 y) {
	float32 fy = (float32)y;

	self->SetPosY(fy);
}

int32 AgsBody_GetIntPositionY(AgsBody* self) {
	return (int32)self->GetPosY();
}

void AgsBody_SetPositionX(AgsBody* self, uint32_t x) {
	float32 fx = ToNormalFloat(x);

	self->SetPosX(fx);
}

uint32_t AgsBody_GetPositionX(AgsBody* self) {
	return ToAgsFloat(self->GetPosX());
}

void AgsBody_SetPositionY(AgsBody* self, uint32_t y) {
	float32 fy = ToNormalFloat(y);

	self->SetPosX(fy);
}

uint32_t AgsBody_GetPositionY(AgsBody* self) {
	return ToAgsFloat(self->GetPosY());
}

void AgsBody_SetAngle(AgsBody* self, uint32_t angle) {
	float32 fangle = ToNormalFloat(angle);

	self->SetAngle(fangle);
}

uint32_t AgsBody_GetAngle(AgsBody* self) {
	return ToAgsFloat(self->GetAngle());
}

void AgsBody_SetLinearDamping(AgsBody* self, uint32_t ldamping) {
	float32 fldamping = ToNormalFloat(ldamping);
	self->SetLinearDamping(fldamping);
}

uint32_t AgsBody_GetLinearDamping(AgsBody* self) {
	return ToAgsFloat(self->GetLinearDamping());
}

void AgsBody_SetAngularDamping(AgsBody* self, uint32_t adamping) {
	float32 fadamping = ToNormalFloat(adamping);
	self->SetAngularDamping(fadamping);
}

uint32_t AgsBody_GetAngularDamping(AgsBody* self) {
	return ToAgsFloat(self->GetAngularDamping());
}

void AgsBody_SetAngularVelocity(AgsBody* self, uint32_t avel) {
	float32 favel = ToNormalFloat(avel);
	self->SetAngularVelocity(favel);
}

uint32_t AgsBody_GetAngularVelocity(AgsBody* self) {
	return ToAgsFloat(self->GetAngularVelocity());
}

void AgsBody_SetInertia(AgsBody* self, uint32_t inertia) {
	float32 finertia = ToNormalFloat(inertia);

	self->SetInertia(finertia);
}

uint32_t AgsBody_GetInertia(AgsBody* self) {
	return ToAgsFloat(self->GetInertia());
}

uint32_t AgsBody_GetLinearVelocityX(AgsBody* self) {
	return ToAgsFloat(self->GetLinearVelocityX());
}

uint32_t AgsBody_GetLinearVelocityY(AgsBody* self) {
	return ToAgsFloat(self->GetLinearVelocityY());
}

void AgsBody_SetLinearVelocity(AgsBody* self, uint32_t vel_x, uint32_t vel_y) {
	float32 f_vel_x = ToNormalFloat(vel_x);
	float32 f_vel_y = ToNormalFloat(vel_y);

	self->SetLinearVelocity(f_vel_x, f_vel_y);
}

void AgsBody_SetPosition(AgsBody* self, uint32_t x, uint32_t y) {
	float32 fx = ToNormalFloat(x);
	float32 fy = ToNormalFloat(y);

	self->SetPos(fx, fy);
}

void AgsBody_ApplyForce(AgsBody* self, uint32_t force_x, uint32_t force_y) {
	float32 f_forcex = ToNormalFloat(force_x);
	float32 f_forcey = ToNormalFloat(force_y);

	self->ApplyForce(f_forcex, f_forcey);
}

void AgsBody_ApplyAngularImpulse(AgsBody* self, uint32_t impulse) {
	self->ApplyAngularImpulse(ToNormalFloat(impulse));
}

void AgsBody_ApplyLinearImpulse(AgsBody* self, uint32_t intensity_x, uint32_t intensity_y) {
	float32 f_intensity_x = ToNormalFloat(intensity_x);
	float32 f_intensity_y = ToNormalFloat(intensity_y);

	self->ApplyLinearImpulse(f_intensity_x, f_intensity_y);
}

void AgsBody_ApplyTorque(AgsBody* self, uint32_t torque) {
	float32 f_torque = ToNormalFloat(torque);

	self->ApplyTorque(f_torque);
}

int32 AgsBody_IsTouching(AgsBody* self, AgsBody* other) {
	if (self->IsTouching(other))
		return 1;
	return 0;
}

#pragma endregion // AgsBody_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsShapeRect_ScriptAPI

uint32_t AgsShapeRect_GetWidthF(AgsShapeRect* self) {
	return ToAgsFloat(self->GetWidthF());
}

void AgsShapeRect_SetWidthF(AgsShapeRect* self, uint32_t w) {
	float32 fw = ToNormalFloat(w);
	self->SetWidthF(fw);
}

uint32_t AgsShapeRect_GetHeightF(AgsShapeRect* self) {
	return ToAgsFloat(self->GetHeightF());
}

void AgsShapeRect_SetHeightF(AgsShapeRect* self, uint32_t h) {
	float32 fh = ToNormalFloat(h);
	self->SetHeightF(fh);
}

int32 AgsShapeRect_GetWidth(AgsShapeRect* self) {
	return self->GetWidth();
}

void AgsShapeRect_SetWidth(AgsShapeRect* self, int32 w) {
	self->SetWidth(w);
}

int32 AgsShapeRect_GetHeight(AgsShapeRect* self) {
	return self->GetHeight();
}

void AgsShapeRect_SetHeight(AgsShapeRect* self, int32 h) {
	self->SetHeight(h);
}

uint32_t AgsShapeRect_GetPointsfX(AgsShapeRect *self, int32 i)
{
	if ((i < 0) || (i > 3))
		return (0);

	return ToAgsFloat(self->GetPointX(i));
}

uint32_t AgsShapeRect_GetPointsfY(AgsShapeRect *self, int32 i)
{
	if ((i < 0) || (i > 3))
		return (0);

	return ToAgsFloat(self->GetPointY(i));
}
#pragma endregion // AgsShapeRect_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsShapeCircle_ScriptAPI

#pragma endregion // AgsShapeCircle_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsShape_ScriptAPI

AgsShapeCircle* AgsShape_AsCircle(AgsShape* self) {
	AgsShapeCircle* shapeAsCircle = self->ShapeCircle;

	if (shapeAsCircle != NULL) {
		engine->RegisterManagedObject(shapeAsCircle, &AgsShapeCircle_Interface);
	}

	return shapeAsCircle;
}

AgsShapeRect* AgsShape_AsRectangle(AgsShape* self) {
	AgsShapeRect* shapeAsRect = self->ShapeRect;

	if (shapeAsRect != NULL) {
		engine->RegisterManagedObject(shapeAsRect, &AgsShapeRect_Interface);
	}

	return shapeAsRect;
}

#pragma endregion // AgsShape_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsFixture_ScriptAPI

uint32_t AgsFixture_GetDensity(AgsFixture* self) {
	return ToAgsFloat(self->GetDensity());
}

void AgsFixture_SetDensity(AgsFixture* self, uint32_t density) {
	float32 f_density = ToNormalFloat(density);

	self->SetDensity(f_density);
}

uint32_t AgsFixture_GetFriction(AgsFixture* self) {
	return ToAgsFloat(self->GetFriction());
}

void AgsFixture_SetFriction(AgsFixture* self, uint32_t friction) {
	float32 f_friction = ToNormalFloat(friction);

	self->SetFriction(f_friction);
}

uint32_t AgsFixture_GetRestitution(AgsFixture* self) {
	return ToAgsFloat(self->GetRestitution());
}

void AgsFixture_SetRestitution(AgsFixture* self, uint32_t restitution) {
	float32 f_restitution = ToNormalFloat(restitution);

	self->SetRestitution(f_restitution);
}

AgsBody* AgsFixture_GetBody(AgsFixture* self) {
    AgsWorld* world = Book::IDtoAgsWorld(self->WorldID);

    return FindAgsBodyFromB2Body(world->B2AgsWorld, world->ID, self->GetB2Body());
}

#pragma endregion // AgsFixture_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsFixtureArray_ScriptAPI

AgsFixtureArray* AgsFixtureArray_Create()
{
    AgsFixtureArray* arr;

    arr = new AgsFixtureArray();

    arr->ID = engine->RegisterManagedObject(arr, &AgsFixtureArray_Interface);
    Book::RegisterAgsFixtureArray(arr->ID, arr);
    return (arr);
}

AgsFixtureArray* AgsFixtureArray_Copy(AgsFixtureArray* self)
{
    AgsFixtureArray* arr;

    if (self == nullptr)
        arr = new AgsFixtureArray();
    else
        arr = new AgsFixtureArray(self);

    arr->ID = engine->RegisterManagedObject(arr, &AgsFixtureArray_Interface);
    Book::RegisterAgsFixtureArray(arr->ID, arr);
    return (arr);
}

void AgsFixtureArray_Clear(AgsFixtureArray* self) {
    self->clear();
}

int32 AgsFixtureArray_IsEmpty(AgsFixtureArray* self) {
    return self->empty();
}

void AgsFixtureArray_Erase(AgsFixtureArray* self, int32 pos, int32 number) {
    if (pos < 0)
        pos += self->size();

    if (pos >= self->size())
        pos = self->size() - 1;

    if (number < 2)
        self->erase(pos);
    else
    {
        number += pos;
        if (number < 0)
            number = 0;
        else if (number > self->size())
            number = self->size();

        self->erase(pos, number);
    }
}

void AgsFixtureArray_Insert(AgsFixtureArray* self, int32 pos, AgsFixture* agsFixture) {
    if (agsFixture == nullptr)
        return;

    if (pos < 0)
        pos += self->size();

    if (pos > self->size())
        pos = self->size();

    self->insert(pos, agsFixture);
}

AgsFixture* AgsFixtureArray_GetItems(AgsFixtureArray* self, int32 i) {
    if ((i < 0) || (i >= self->size()))
        return nullptr;

    AgsFixtureArrayData fad = (*self).data[i];
    AgsFixture* agsFixture = FindAgsFixtureFromB2Fixture(fad.B2World, fad.WorldID, fad.B2Fixture);
    return agsFixture;
}

void AgsFixtureArray_SetItems(AgsFixtureArray* self, int32 i, AgsFixture* agsFixture) {
    if (agsFixture == nullptr)
        return;

    if ((i < 0) || (i >= self->size()))
        return;

    self->set_item(i, agsFixture);
}

AgsFixture* AgsFixtureArray_Pop(AgsFixtureArray* self) {
    if (!self->empty()) {
        AgsFixtureArrayData fad = self->pop();
        AgsFixture* agsFixture = FindAgsFixtureFromB2Fixture(fad.B2World, fad.WorldID, fad.B2Fixture);
        return agsFixture;
    }
    else {
        return (0);
    }
}

void AgsFixtureArray_Push(AgsFixtureArray* self, AgsFixture* agsFixture) {
    if (agsFixture == nullptr)
        return;

    self->push(agsFixture);
}

int32 AgsFixtureArray_GetSize(AgsFixtureArray* self) {
    return self->size();
}

void AgsFixtureArray_SetSize(AgsFixtureArray* self, int32 size) {
    self->resize(size);
}

void AgsFixtureArray_Reserve(AgsFixtureArray* self, int32 number) {
    self->reserve(number);
}

#pragma endregion // AgsFixtureArray_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsJointDistance_ScriptAPI


uint32_t AgsJointDistance_GetLength (AgsJointDistance* self) {
    return ToAgsFloat(self->GetLength());
}

void AgsJointDistance_SetLength (AgsJointDistance* self, uint32_t length) {
    float32 f_length = ToNormalFloat(length);
    self->SetLength(f_length);
}

uint32_t AgsJointDistance_GetDampingRatio (AgsJointDistance* self) {
    return ToAgsFloat(self->GetDampingRatio());
}

void AgsJointDistance_SetDampingRatio (AgsJointDistance* self, uint32_t dratio) {
    float32 f_dratio = ToNormalFloat(dratio);
    self->SetDampingRatio(f_dratio);
}

uint32_t AgsJointDistance_GetFrequency (AgsJointDistance* self) {
    return ToAgsFloat(self->GetFrequency());
}

void AgsJointDistance_SetFrequency (AgsJointDistance* self, uint32_t hz) {
    float32 fhz = ToNormalFloat(hz);
    self->SetFrequency(fhz);
}

#pragma endregion // AgsJointDistance_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsJointMotor_ScriptAPI

uint32_t AgsJointMotor_GetLinearOffsetX (AgsJointMotor* self) {
    return ToAgsFloat(self->GetLinearOffsetX());
}

void AgsJointMotor_SetLinearOffsetX (AgsJointMotor* self, uint32_t x) {
    float32 fx = ToNormalFloat(x);
    self->SetLinearOffsetX(fx);
}

uint32_t AgsJointMotor_GetLinearOffsetY (AgsJointMotor* self) {
    return ToAgsFloat(self->GetLinearOffsetY());
}

void AgsJointMotor_SetLinearOffsetY (AgsJointMotor* self, uint32_t y) {
    float32 fy = ToNormalFloat(y);
    self->SetLinearOffsetY(fy);
}

void AgsJointMotor_SetLinearOffset (AgsJointMotor* self, uint32_t x, uint32_t y) {
    float32 fx = ToNormalFloat(x);
    float32 fy = ToNormalFloat(y);
    self->SetLinearOffset(fx,fy);
}

uint32_t AgsJointMotor_GetAngularOffset (AgsJointMotor* self) {
    return ToAgsFloat(self->GetAngularOffset());
}

void AgsJointMotor_SetAngularOffset (AgsJointMotor* self, uint32_t angularOffset) {
    float32 fangularOffset = ToNormalFloat(angularOffset);
    self->SetAngularOffset(fangularOffset);
}

uint32_t AgsJointMotor_GetMaxForce (AgsJointMotor* self) {
    return ToAgsFloat(self->GetMaxForce());
}

void AgsJointMotor_SetMaxForce (AgsJointMotor* self, uint32_t force) {
    float32 f_force = ToNormalFloat(force);
    self->SetMaxForce(f_force);
}

uint32_t AgsJointMotor_GetMaxTorque (AgsJointMotor* self) {
    return ToAgsFloat(self->GetMaxTorque());
}

void AgsJointMotor_SetMaxTorque (AgsJointMotor* self, uint32_t torque) {
    float32 ftorque = ToNormalFloat(torque);
    self->SetMaxTorque(ftorque);
}

#pragma endregion // AgsJointMotor_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsJointMouse_ScriptAPI

uint32_t AgsJointMouse_GetDampingRatio (AgsJointMouse* self) {
    return ToAgsFloat(self->GetDampingRatio());
}

void AgsJointMouse_SetDampingRatio (AgsJointMouse* self, uint32_t dratio) {
    float32 f_dratio = ToNormalFloat(dratio);
    self->SetDampingRatio(f_dratio);
}

uint32_t AgsJointMouse_GetFrequency (AgsJointMouse* self) {
    return ToAgsFloat(self->GetFrequency());
}

void AgsJointMouse_SetFrequency (AgsJointMouse* self, uint32_t hz) {
    float32 fhz = ToNormalFloat(hz);
    self->SetFrequency(fhz);
}

uint32_t AgsJointMouse_GetTargetX (AgsJointMouse* self) {
    return ToAgsFloat(self->GetTargetX());
}

uint32_t AgsJointMouse_GetTargetY (AgsJointMouse* self) {
    return ToAgsFloat(self->GetTargetY());
}

void AgsJointMouse_SetTarget (AgsJointMouse* self, uint32_t x, uint32_t y) {
    float32 fx = ToNormalFloat(x);
    float32 fy = ToNormalFloat(y);
    self->SetTarget(fx,fy);
}

uint32_t AgsJointMouse_GetMaxForce (AgsJointMouse* self) {
    return ToAgsFloat(self->GetMaxForce());
}

void AgsJointMouse_SetMaxForce (AgsJointMouse* self, uint32_t force) {
    float32 f_force = ToNormalFloat(force);
    self->SetMaxForce(f_force);
}

#pragma endregion // AgsJointMouse_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsJointPulley_ScriptAPI

uint32_t AgsJointPulley_GetLengthA (AgsJointPulley* self) {
    return ToAgsFloat(self->GetLengthA());
}

uint32_t AgsJointPulley_GetLengthB (AgsJointPulley* self) {
    return ToAgsFloat(self->GetLengthB());
}

uint32_t AgsJointPulley_GetRatio (AgsJointPulley* self) {
    return ToAgsFloat(self->GetRatio());
}

#pragma endregion // AgsJointPulley_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsJoint_ScriptAPI

AgsJointDistance* AgsJoint_AsDistance (AgsJoint* self) {
    if(self->GetB2AgsJoint()->GetType() != b2JointType ::e_distanceJoint)
        return nullptr;

    AgsJointDistance* agsJointDistance = new AgsJointDistance(self->GetAgsWorld()->ID, dynamic_cast<b2DistanceJoint*>(self->GetB2AgsJoint()));
    agsJointDistance->ID = engine->RegisterManagedObject(agsJointDistance, &AgsJointDistance_Interface);
    Book::RegisterAgsJointDistance(agsJointDistance->ID, agsJointDistance);
    return agsJointDistance;
}

AgsJointMotor* AgsJoint_AsMotor (AgsJoint* self) {
    if(self->GetB2AgsJoint()->GetType() != b2JointType ::e_motorJoint)
        return nullptr;

    AgsJointMotor* agsJointMotor = new AgsJointMotor(self->GetAgsWorld()->ID, dynamic_cast<b2MotorJoint*>(self->GetB2AgsJoint()));
    agsJointMotor->ID = engine->RegisterManagedObject(agsJointMotor, &AgsJointMotor_Interface);
    Book::RegisterAgsJointMotor(agsJointMotor->ID, agsJointMotor);
    return agsJointMotor;
}

AgsJointMouse* AgsJoint_AsMouse (AgsJoint* self) {
    if(self->GetB2AgsJoint()->GetType() != b2JointType ::e_mouseJoint)
        return nullptr;

    AgsJointMouse* agsJointMouse = new AgsJointMouse(self->GetAgsWorld()->ID, dynamic_cast<b2MouseJoint*>(self->GetB2AgsJoint()));
    agsJointMouse->ID = engine->RegisterManagedObject(agsJointMouse, &AgsJointMouse_Interface);
    Book::RegisterAgsJointMouse(agsJointMouse->ID, agsJointMouse);
    return agsJointMouse;
}

AgsJointPulley* AgsJoint_AsPulley (AgsJoint* self) {
    if(self->GetB2AgsJoint()->GetType() != b2JointType ::e_pulleyJoint)
        return nullptr;

    AgsJointPulley* agsJointPulley = new AgsJointPulley(self->GetAgsWorld()->ID, dynamic_cast<b2PulleyJoint*>(self->GetB2AgsJoint()));
    agsJointPulley->ID = engine->RegisterManagedObject(agsJointPulley, &AgsJointPulley_Interface);
    Book::RegisterAgsJointPulley(agsJointPulley->ID, agsJointPulley);
    return agsJointPulley;
}

int32 AgsJoint_GetIsValid (AgsJoint* self) {
    return self->isValid();
}

int32 AgsJoint_GetIsActive (AgsJoint* self) {
    return  self->isActive();
}

AgsBody* AgsJoint_GetBodyA (AgsJoint* self) {
    b2World* b2world = self->GetAgsWorld()->B2AgsWorld;
    int32 world_id = self->GetAgsWorld()->ID;
    if(self->GetB2AgsJoint()->GetType() == e_mouseJoint)
        return FindAgsBodyFromB2Body(b2world, world_id, self->GetBodyB());

    return FindAgsBodyFromB2Body(b2world, world_id, self->GetBodyA());
}

AgsBody* AgsJoint_GetBodyB (AgsJoint* self) {
    b2World* b2world = self->GetAgsWorld()->B2AgsWorld;
    int32 world_id = self->GetAgsWorld()->ID;
    if(self->GetB2AgsJoint()->GetType() == e_mouseJoint)
        return nullptr;

    return FindAgsBodyFromB2Body(b2world, world_id, self->GetBodyB());
}

int32 AgsJoint_GetType (AgsJoint* self) {
    return  self->GetType();
}

#pragma endregion // AgsJoint_ScriptAPI
//-----------------------------------------------------------------------------

#define REGISTER(x) engine->RegisterScriptFunction(#x, (void *) (x));
#define STRINGIFY(s) STRINGIFY_X(s)
#define STRINGIFY_X(s) #s

void AGS_EngineStartup(IAGSEngine *lpEngine)
{
	engine = lpEngine;

	// Make sure it's got the version with the features we need
	if (engine->version < MIN_ENGINE_VERSION)
			engine->AbortGame("Plugin needs engine version " STRINGIFY(MIN_ENGINE_VERSION) " or newer.");

	//register functions
	engine->AddManagedObjectReader(PointFInterface::name, &PointF_Reader);

	engine->AddManagedObjectReader(AgsWorldInterface::name, &AgsWorld_Reader);
	engine->AddManagedObjectReader(AgsBodyInterface::name, &AgsBody_Reader);
	engine->AddManagedObjectReader(AgsShapeInterface::name, &AgsShape_Reader);
	engine->AddManagedObjectReader(AgsShapeRectInterface::name, &AgsShapeRect_Reader);
	engine->AddManagedObjectReader(AgsShapeCircleInterface::name, &AgsShapeCircle_Reader);
	engine->AddManagedObjectReader(AgsFixtureInterface::name, &AgsFixture_Reader);
    engine->AddManagedObjectReader(AgsFixtureArrayInterface::name, &AgsFixtureArray_Reader);
    engine->AddManagedObjectReader(AgsRaycastResultInterface::name, &AgsRaycastResult_Reader);

    engine->AddManagedObjectReader(AgsJointInterface::name, &AgsJoint_Reader);
    engine->AddManagedObjectReader(AgsJointDistanceInterface::name, &AgsJointDistance_Reader);
    engine->AddManagedObjectReader(AgsJointMotorInterface::name, &AgsJointMotor_Reader);
    engine->AddManagedObjectReader(AgsJointMouseInterface::name, &AgsJointMouse_Reader);
    engine->AddManagedObjectReader(AgsJointPulleyInterface::name, &AgsJointPulley_Reader);

    engine->RegisterScriptFunction("PointF::Create^2", (void*)PointF_Create);
    engine->RegisterScriptFunction("PointF::set_X", (void*)PointF_SetX);
    engine->RegisterScriptFunction("PointF::get_X", (void*)PointF_GetX);
    engine->RegisterScriptFunction("PointF::set_Y", (void*)PointF_SetY);
    engine->RegisterScriptFunction("PointF::get_Y", (void*)PointF_GetY);

	engine->RegisterScriptFunction("AgsBox2D::SetMeter^1", (void*)agsbox2d_SetMeter);
	engine->RegisterScriptFunction("AgsBox2D::GetMeter^0", (void*)agsbox2d_GetMeter);
	engine->RegisterScriptFunction("AgsBox2D::CreateWorld^2", (void*)agsbox2d_newWorld);
	engine->RegisterScriptFunction("AgsBox2D::CreateBody^4", (void*)agsbox2d_newBody);
	engine->RegisterScriptFunction("AgsBox2D::DestroyBody^2", (void*)agsbox2d_DestroyBody);
	engine->RegisterScriptFunction("AgsBox2D::CreateRectangleShape^4", (void*)agsbox2d_newRectangleShape);
	engine->RegisterScriptFunction("AgsBox2D::CreateCircleShape^3", (void*)agsbox2d_newCircleShape);
	engine->RegisterScriptFunction("AgsBox2D::CreateFixture^3", (void*)agsbox2d_newFixture);
    engine->RegisterScriptFunction("AgsBox2D::CreateDistanceJoint^7", (void*)agsbox2d_newDistanceJoint);
    engine->RegisterScriptFunction("AgsBox2D::CreateMotorJoint^4", (void*)agsbox2d_newMotorJoint);
    engine->RegisterScriptFunction("AgsBox2D::CreateMouseJoint^3", (void*)agsbox2d_newMouseJoint);
    engine->RegisterScriptFunction("AgsBox2D::CreatePulleyJoint^8", (void*)agsbox2d_newPulleyJoint);
    engine->RegisterScriptFunction("AgsBox2D::DestroyJoint^2", (void*)agsbox2d_DestroyJoint);

	engine->RegisterScriptFunction("World::Step^3", (void*)AgsWorld_Step);
	engine->RegisterScriptFunction("World::GetDebugSprite^2", (void*)AgsWorld_GetDebugSprite);
	engine->RegisterScriptFunction("World::BoundingBoxQuery^4", (void*)AgsWorld_BoundingBoxQuery);
    engine->RegisterScriptFunction("World::Raycast^6", (void*)AgsWorld_Raycast);

    engine->RegisterScriptFunction("RaycastResult::get_Length", (void*)AgsRaycastResult_GetLength);
    engine->RegisterScriptFunction("RaycastResult::geti_PointX", (void*)AgsRaycastResult_GetPointX);
    engine->RegisterScriptFunction("RaycastResult::geti_PointY", (void*)AgsRaycastResult_GetPointY);
    engine->RegisterScriptFunction("RaycastResult::geti_NormalX", (void*)AgsRaycastResult_GetNormalX);
    engine->RegisterScriptFunction("RaycastResult::geti_NormalY", (void*)AgsRaycastResult_GetNormalY);
    engine->RegisterScriptFunction("RaycastResult::geti_Fraction", (void*)AgsRaycastResult_GetFraction);
    engine->RegisterScriptFunction("RaycastResult::geti_Fixture", (void*)AgsRaycastResult_GetFixture);

	engine->RegisterScriptFunction("Body::get_IsDestroyed", (void*)AgsBody_IsDestroyed);
	engine->RegisterScriptFunction("Body::set_FixedRotation", (void*)AgsBody_SetFixedRotation);
	engine->RegisterScriptFunction("Body::get_FixedRotation", (void*)AgsBody_GetFixedRotation);
	engine->RegisterScriptFunction("Body::set_Bullet", (void*)AgsBody_SetBullet);
	engine->RegisterScriptFunction("Body::get_Bullet", (void*)AgsBody_GetBullet);
	engine->RegisterScriptFunction("Body::set_X", (void*)AgsBody_SetIntPositionX);
	engine->RegisterScriptFunction("Body::get_X", (void*)AgsBody_GetIntPositionX);
	engine->RegisterScriptFunction("Body::set_Y", (void*)AgsBody_SetIntPositionY);
	engine->RegisterScriptFunction("Body::get_Y", (void*)AgsBody_GetIntPositionY);
	engine->RegisterScriptFunction("Body::set_fX", (void*)AgsBody_SetPositionX);
	engine->RegisterScriptFunction("Body::get_fX", (void*)AgsBody_GetPositionX);
	engine->RegisterScriptFunction("Body::set_fY", (void*)AgsBody_SetPositionY);
	engine->RegisterScriptFunction("Body::get_fY", (void*)AgsBody_GetPositionY);
	engine->RegisterScriptFunction("Body::set_Angle", (void*)AgsBody_SetAngle);
	engine->RegisterScriptFunction("Body::get_Angle", (void*)AgsBody_GetAngle);
	engine->RegisterScriptFunction("Body::set_LinearDamping", (void*)AgsBody_SetLinearDamping);
	engine->RegisterScriptFunction("Body::get_LinearDamping", (void*)AgsBody_GetLinearDamping);
	engine->RegisterScriptFunction("Body::set_AngularDamping", (void*)AgsBody_SetAngularDamping);
	engine->RegisterScriptFunction("Body::get_AngularDamping", (void*)AgsBody_GetAngularDamping);
	engine->RegisterScriptFunction("Body::set_AngularVelocity", (void*)AgsBody_SetAngularVelocity);
	engine->RegisterScriptFunction("Body::get_AngularVelocity", (void*)AgsBody_GetAngularVelocity);
	engine->RegisterScriptFunction("Body::set_Inertia", (void*)AgsBody_SetInertia);
	engine->RegisterScriptFunction("Body::get_Inertia", (void*)AgsBody_GetInertia);
	engine->RegisterScriptFunction("Body::get_LinearVelocityX", (void*)AgsBody_GetLinearVelocityX);
	engine->RegisterScriptFunction("Body::get_LinearVelocityY", (void*)AgsBody_GetLinearVelocityY);
	engine->RegisterScriptFunction("Body::SetLinearVelocity^2", (void*)AgsBody_SetLinearVelocity);
	engine->RegisterScriptFunction("Body::ApplyForce^2", (void*)AgsBody_ApplyForce);
	engine->RegisterScriptFunction("Body::ApplyAngularImpulse^1", (void*)AgsBody_ApplyAngularImpulse);
	engine->RegisterScriptFunction("Body::ApplyLinearImpulse^2", (void*)AgsBody_ApplyLinearImpulse);
	engine->RegisterScriptFunction("Body::ApplyTorque^1", (void*)AgsBody_ApplyTorque);
	engine->RegisterScriptFunction("Body::IsTouching^1", (void*)AgsBody_IsTouching);

	engine->RegisterScriptFunction("ShapeRectangle::get_Width", (void*)AgsShapeRect_GetWidth);
	engine->RegisterScriptFunction("ShapeRectangle::set_Width", (void*)AgsShapeRect_SetWidth);
	engine->RegisterScriptFunction("ShapeRectangle::get_Height", (void*)AgsShapeRect_GetHeight);
	engine->RegisterScriptFunction("ShapeRectangle::set_Height", (void*)AgsShapeRect_SetHeight);
	engine->RegisterScriptFunction("ShapeRectangle::get_fWidth", (void*)AgsShapeRect_GetWidthF);
	engine->RegisterScriptFunction("ShapeRectangle::set_fWidth", (void*)AgsShapeRect_SetWidthF);
	engine->RegisterScriptFunction("ShapeRectangle::get_fHeight", (void*)AgsShapeRect_GetHeightF);
	engine->RegisterScriptFunction("ShapeRectangle::set_fHeight", (void*)AgsShapeRect_SetHeightF);
	engine->RegisterScriptFunction("ShapeRectangle::geti_PointsfX", (void*)AgsShapeRect_GetPointsfX);
	engine->RegisterScriptFunction("ShapeRectangle::geti_PointsfY", (void*)AgsShapeRect_GetPointsfY);

	engine->RegisterScriptFunction("Shape::get_AsCircle", (void*)AgsShape_AsCircle);
	engine->RegisterScriptFunction("Shape::get_AsRectangle", (void*)AgsShape_AsRectangle);

	engine->RegisterScriptFunction("Fixture::get_Density", (void*)AgsFixture_GetDensity);
	engine->RegisterScriptFunction("Fixture::set_Density", (void*)AgsFixture_SetDensity);
	engine->RegisterScriptFunction("Fixture::get_Friction", (void*)AgsFixture_GetFriction);
	engine->RegisterScriptFunction("Fixture::set_Friction", (void*)AgsFixture_SetFriction);
	engine->RegisterScriptFunction("Fixture::get_Restitution", (void*)AgsFixture_GetRestitution);
	engine->RegisterScriptFunction("Fixture::set_Restitution", (void*)AgsFixture_SetRestitution);
    engine->RegisterScriptFunction("Fixture::get_Body", (void*)AgsFixture_GetBody);

    engine->RegisterScriptFunction("JointDistance::get_Length", (void*)AgsJointDistance_GetLength);
    engine->RegisterScriptFunction("JointDistance::set_Length", (void*)AgsJointDistance_SetLength);
    engine->RegisterScriptFunction("JointDistance::get_DampingRatio", (void*)AgsJointDistance_GetDampingRatio);
    engine->RegisterScriptFunction("JointDistance::set_DampingRatio", (void*)AgsJointDistance_SetDampingRatio);
    engine->RegisterScriptFunction("JointDistance::get_Frequency", (void*)AgsJointDistance_GetFrequency);
    engine->RegisterScriptFunction("JointDistance::set_Frequency", (void*)AgsJointDistance_SetFrequency);

    engine->RegisterScriptFunction("FixtureArray::Create^0", (void*)AgsFixtureArray_Create);
    engine->RegisterScriptFunction("FixtureArray::Copy^0", (void*)AgsFixtureArray_Copy);
    engine->RegisterScriptFunction("FixtureArray::Clear^0", (void*)AgsFixtureArray_Clear);
    engine->RegisterScriptFunction("FixtureArray::IsEmpty^0", (void*)AgsFixtureArray_IsEmpty);
    engine->RegisterScriptFunction("FixtureArray::Erase^2", (void*)AgsFixtureArray_Erase);
    engine->RegisterScriptFunction("FixtureArray::Insert^2", (void*)AgsFixtureArray_Insert);
    engine->RegisterScriptFunction("FixtureArray::geti_Items", (void*)AgsFixtureArray_GetItems);
    engine->RegisterScriptFunction("FixtureArray::seti_Items", (void*)AgsFixtureArray_SetItems);
    engine->RegisterScriptFunction("FixtureArray::Pop^0", (void*)AgsFixtureArray_Pop);
    engine->RegisterScriptFunction("FixtureArray::Push^1", (void*)AgsFixtureArray_Push);
    engine->RegisterScriptFunction("FixtureArray::get_Size", (void*)AgsFixtureArray_GetSize);
    engine->RegisterScriptFunction("FixtureArray::set_Size", (void*)AgsFixtureArray_SetSize);

    engine->RegisterScriptFunction("JointMotor::get_LinearOffsetX", (void*)AgsJointMotor_GetLinearOffsetX);
    engine->RegisterScriptFunction("JointMotor::set_LinearOffsetX", (void*)AgsJointMotor_SetLinearOffsetX);
    engine->RegisterScriptFunction("JointMotor::get_LinearOffsetY", (void*)AgsJointMotor_GetLinearOffsetY);
    engine->RegisterScriptFunction("JointMotor::set_LinearOffsetY", (void*)AgsJointMotor_SetLinearOffsetY);
    engine->RegisterScriptFunction("JointMotor::SetLinearOffset^2", (void*)AgsJointMotor_SetLinearOffset);
    engine->RegisterScriptFunction("JointMotor::get_AngularOffset", (void*)AgsJointMotor_GetAngularOffset);
    engine->RegisterScriptFunction("JointMotor::set_AngularOffset", (void*)AgsJointMotor_SetAngularOffset);
    engine->RegisterScriptFunction("JointMotor::get_MaxForce", (void*)AgsJointMotor_GetMaxForce);
    engine->RegisterScriptFunction("JointMotor::set_MaxForce", (void*)AgsJointMotor_SetMaxForce);
    engine->RegisterScriptFunction("JointMotor::get_MaxTorque", (void*)AgsJointMotor_GetMaxTorque);
    engine->RegisterScriptFunction("JointMotor::set_MaxTorque", (void*)AgsJointMotor_SetMaxTorque);

    engine->RegisterScriptFunction("JointMouse::get_DampingRatio", (void*)AgsJointMouse_GetDampingRatio);
    engine->RegisterScriptFunction("JointMouse::set_DampingRatio", (void*)AgsJointMouse_SetDampingRatio);
    engine->RegisterScriptFunction("JointMouse::get_Frequency", (void*)AgsJointMouse_GetFrequency);
    engine->RegisterScriptFunction("JointMouse::set_Frequency", (void*)AgsJointMouse_SetFrequency);
    engine->RegisterScriptFunction("JointMouse::get_TargetX", (void*)AgsJointMouse_GetTargetX);
    engine->RegisterScriptFunction("JointMouse::get_TargetY", (void*)AgsJointMouse_GetTargetY);
    engine->RegisterScriptFunction("JointMouse::SetTarget^2", (void*)AgsJointMouse_SetTarget);
    engine->RegisterScriptFunction("JointMouse::get_MaxForce", (void*)AgsJointMouse_GetMaxForce);
    engine->RegisterScriptFunction("JointMouse::set_MaxForce", (void*)AgsJointMouse_SetMaxForce);

    engine->RegisterScriptFunction("JointPulley::get_LengthA", (void*)AgsJointPulley_GetLengthA);
    engine->RegisterScriptFunction("JointPulley::get_LengthB", (void*)AgsJointPulley_GetLengthB);
    engine->RegisterScriptFunction("JointPulley::get_Ratio", (void*)AgsJointPulley_GetRatio);

    engine->RegisterScriptFunction("Joint::get_AsDistance", (void*)AgsJoint_AsDistance);
    engine->RegisterScriptFunction("Joint::get_AsMotor", (void*)AgsJoint_AsMotor);
    engine->RegisterScriptFunction("Joint::get_AsMouse", (void*)AgsJoint_AsMouse);
    engine->RegisterScriptFunction("Joint::get_AsPulley", (void*)AgsJoint_AsPulley);
    engine->RegisterScriptFunction("Joint::get_IsValid", (void*)AgsJoint_GetIsValid);
    engine->RegisterScriptFunction("Joint::get_IsActive", (void*)AgsJoint_GetIsActive);
    engine->RegisterScriptFunction("Joint::get_BodyA", (void*)AgsJoint_GetBodyA);
    engine->RegisterScriptFunction("Joint::get_BodyB", (void*)AgsJoint_GetBodyB);
    engine->RegisterScriptFunction("Joint::get_Type", (void*)AgsJoint_GetType);

  engine->RequestEventHook(AGSE_PRESCREENDRAW);
}

//------------------------------------------------------------------------------

void AGS_EngineShutdown()
{
	// Called by the game engine just before it exits.
	// This gives you a chance to free any memory and do any cleanup
	// that you need to do before the engine shuts down.
}

//------------------------------------------------------------------------------

bool do_only_once = false;
int AGS_EngineOnEvent(int event, int data)                    //*** optional ***
{
  if(event==AGSE_PRESCREENDRAW){
  	//initialize debug
  	if(!do_only_once) {
        int screenWidth, screenHeight, colDepth;
        engine->GetScreenDimensions(&screenWidth, &screenHeight, &colDepth);
        debugDraw.InitializeAgsDebugDraw(engine, screenWidth, screenHeight, colDepth);
        printf("\nagsbox2d 0.2.0\n");
        do_only_once = true;
    }
  }

/*
	switch (event)
	{
    case AGSE_KEYPRESS:
    case AGSE_MOUSECLICK:
    case AGSE_POSTSCREENDRAW:
    case AGSE_PRESCREENDRAW:
    case AGSE_SAVEGAME:
    case AGSE_RESTOREGAME:
    case AGSE_PREGUIDRAW:
    case AGSE_LEAVEROOM:
    case AGSE_ENTERROOM:
    case AGSE_TRANSITIONIN:
    case AGSE_TRANSITIONOUT:
    case AGSE_FINALSCREENDRAW:
    case AGSE_TRANSLATETEXT:
    case AGSE_SCRIPTDEBUG:
    case AGSE_SPRITELOAD:
    case AGSE_PRERENDER:
    case AGSE_PRESAVEGAME:
    case AGSE_POSTRESTOREGAME:
	default:
			break;
	}
*/

	// Return 1 to stop event from processing further (when needed)
	return (0);
}

//------------------------------------------------------------------------------

int AGS_EngineDebugHook(const char *scriptName,
                        int lineNum, int reserved)            //*** optional ***
{
	// Can be used to debug scripts, see documentation
	return 0;
}

//------------------------------------------------------------------------------

void AGS_EngineInitGfx(const char *driverID, void *data)      //*** optional ***
{
	// This allows you to make changes to how the graphics driver starts up.
	// See documentation
}

//..............................................................................


#if defined(BUILTIN_PLUGINS)
}
#endif
