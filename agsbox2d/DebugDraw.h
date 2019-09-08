#pragma once

#ifndef AGSBOX2D_DEBUGDRAW_H
#define AGSBOX2D_DEBUGDRAW_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class AgsDebugDraw : public b2Draw
{
	int32 SpriteId;
	int32 ScreenWidth;
	int32 ScreenHeight;
	IAGSEngine * Engine;
	uint32 **Longbuffer;
	float32 CameraX;
	float32 CameraY;

public:
	void InitializeAgsDebugDraw(IAGSEngine* engine, int screenWidth, int screenHeight, int colDepth);

	int GetDebugSprite();
	void ClearSprite();
	void GetSurfaceForDebugDraw(int camera_x, int camera_y);
	void ReleaseSurfaceForDebugDraw();

	void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);

	/// Draw a solid closed polygon provided in CCW order.
	void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);

	/// Draw a circle.
	void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color);

	/// Draw a solid circle.
	void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color);

	/// Draw a line segment.
	void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color);

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	void DrawTransform(const b2Transform& xf);

	/// Draw a point.
	void DrawPoint(const b2Vec2& p, float32 size, const b2Color& color);
};

#endif /* AGSBOX2D_DEBUGDRAW_H */
