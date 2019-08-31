// dllmain.cpp : Defines the entry point for the DLL application.

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

#include "AgsWorld.h"
#include "AgsBody.h"
#include "AgsShape.h"
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"
#include "AgsFixture.h"

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
"managed struct Body { \r\n"
"  import attribute float fX; \r\n"
"  import attribute float fY; \r\n"
"  import attribute int X; \r\n"
"  import attribute int Y; \r\n"
"}; \r\n"
" \r\n"
"managed struct World { \r\n"
"  \r\n"
"  /// Advances one step of the simulation \r\n"
"  import void Step(float dt, int velocityIteractions = 8, int positionIteractions = 3); \r\n"
"}; \r\n"
" \r\n"
"managed struct ShapeCircle; \r\n"
"managed struct ShapeRectangle; \r\n"
" \r\n"
"managed struct Shape { \r\n"
"  \r\n"
"  /// If this shape is a circle, returns the ShapeCircle interface; otherwise null. \r\n"
"  readonly import attribute ShapeCircle* AsCircle;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"  \r\n"
"  /// If this shape is a rectangle, returns the ShapeRectangle interface; otherwise null. \r\n"
"  readonly import attribute ShapeRectangle* AsRectangle;  // $AUTOCOMPLETENOINHERIT$  \r\n"
"}; \r\n"
" \r\n"
"managed struct ShapeCircle extends Shape { \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"managed struct ShapeRectangle extends Shape { \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"managed struct Fixture { \r\n"
"  \r\n"
"}; \r\n"
" \r\n"
"struct AgsBox2D { \r\n"
"  \r\n"
"  /// Set Meter \r\n"
"  import static void SetMeter(float meter); \r\n"
"  \r\n"
"  /// Get Meter \r\n"
"  import static float GetMeter(); \r\n"
"  \r\n"
"  /// Create World \r\n"
"  import static World* CreateWorld(float gravityX, float gravityY); \r\n"
"  \r\n"
"  /// Create Body \r\n"
"  import static Body* CreateBody(World* world,  float x, float y, BodyType bodytype); \r\n"
"  \r\n"
"  /// Create Rectangle Shape \r\n"
"  import static Shape* CreateRectangleShape(float w,  float h,  float x=0, float y=0); \r\n"
"  \r\n"
"  /// Create Circle Shape \r\n"
"  import static Shape* CreateCircleShape(float radius,  float x=0, float y=0); \r\n"
"  \r\n"
"  /// Create Fixture \r\n"
"  import static Fixture* CreateFixture(Body* body, Shape* shape, float density=0); \r\n"
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
/*	MessageBox(parent,
             L"agsfastwfc v1.0 By Calin Leafshade",
             L"About",
         MB_OK | MB_ICONINFORMATION);
 */
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

#endif

// ***** Run time *****

// Engine interface

//------------------------------------------------------------------------------

//define engine
IAGSEngine* engine;


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

	engine->RegisterManagedObject(world, &AgsWorld_Interface);

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

	engine->RegisterManagedObject(body, &AgsBody_Interface);

	return body;
}

AgsShape* agsbox2d_newRectangleShape(uint32_t w, uint32_t h, uint32_t x, uint32_t y) {
	float32 fx = ToNormalFloat(x);
	float32 fy = ToNormalFloat(y);
	float32 fw = ToNormalFloat(w);
	float32 fh = ToNormalFloat(h);

	AgsShape* shape = new AgsShape(new AgsShapeRect(fw, fh, fx, fy));

	engine->RegisterManagedObject(shape, &AgsShape_Interface);

	return shape;
}


AgsShape* agsbox2d_newCircleShape(uint32_t radius, uint32_t x, uint32_t y) {
	float32 fx = ToNormalFloat(x);
	float32 fy = ToNormalFloat(y);
	float32 fradius = ToNormalFloat(radius);

	AgsShape* shape = new AgsShape(new AgsShapeCircle(fradius));

	engine->RegisterManagedObject(shape, &AgsShape_Interface);

	return shape;
}

AgsFixture* agsbox2d_newFixture(AgsBody* body, AgsShape* shape, uint32_t density) {
	float32 fdensity;
	if (density == 0) {
		fdensity = 1.0f;
	}
	else {
		fdensity = ToNormalFloat(density);
	}

	AgsFixture* fixture = new AgsFixture(body, shape, fdensity);

	engine->RegisterManagedObject(fixture, &AgsFixture_Interface);

	return fixture;
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

#pragma endregion // AgsWorld_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsBody_ScriptAPI


int32 AgsBody_GetIntPositionX(AgsBody* self) {
	return (int32) self->GetPosX();
}

int32 AgsBody_GetIntPositionY(AgsBody* self) {
	return (int32)self->GetPosY();
}

void AgsBody_SetIntPositionX(AgsBody* self, int32 x) {
	float32 fx = (float32) x;

	self->SetPosX(fx);
}

void AgsBody_SetIntPositionY(AgsBody* self, int32 y) {
	float32 fy = (float32)y;

	self->SetPosX(fy);
}
//end of int positions

//float positions
uint32_t AgsBody_GetPositionX(AgsBody* self) {
	return ToAgsFloat(self->GetPosX());
}

uint32_t AgsBody_GetPositionY(AgsBody* self) {
	return ToAgsFloat(self->GetPosY());
}

void AgsBody_SetPositionX(AgsBody* self, uint32_t x) {
	float32 fx = ToNormalFloat(x);

	self->SetPosX(fx);
}

void AgsBody_SetPositionY(AgsBody* self, uint32_t y) {
	float32 fy = ToNormalFloat(y);

	self->SetPosX(fy);
}

void AgsBody_SetPosition(AgsBody* self, uint32_t x, uint32_t y) {
	float32 fx = ToNormalFloat(x);
	float32 fy = ToNormalFloat(y);

	self->SetPos(fx, fy);
}

//end of float positions

void AgsBody_ApplyForce(AgsBody* self, uint32_t force_x, uint32_t force_y) {
	float32 f_forcex = ToNormalFloat(force_x);
	float32 f_forcey = ToNormalFloat(force_y);

	self->ApplyForce(f_forcex, f_forcey);
}

#pragma endregion // AgsBody_ScriptAPI
//-----------------------------------------------------------------------------
#pragma region AgsShapeRect_ScriptAPI

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

#pragma endregion // AgsFixture_ScriptAPI
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

      //  engine->RegisterScriptFunction("AgsBox2D::newWorld^7", (void*)AgsFastWFC_Overlapping);

}

//------------------------------------------------------------------------------

void AGS_EngineShutdown()
{
        // Called by the game engine just before it exits.
        // This gives you a chance to free any memory and do any cleanup
        // that you need to do before the engine shuts down.
}

//------------------------------------------------------------------------------

int AGS_EngineOnEvent(int event, int data)                    //*** optional ***
{
        switch (event)
        {
/*
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
 */
        default:
                break;
        }

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
