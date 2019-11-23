/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "DebugDraw.h"
#include "Scale.h"

#if !AGS_PLATFORM_OS_WINDOWS
#define min(x,y) (((x) < (y)) ? (x) : (y))
#define max(x,y) (((x) > (y)) ? (x) : (y))
#endif

#pragma region Color_Functions

#define DEFAULT_RGB_R_SHIFT_32  16
#define DEFAULT_RGB_G_SHIFT_32  8
#define DEFAULT_RGB_B_SHIFT_32  0
#define DEFAULT_RGB_A_SHIFT_32  24

int getr32(uint32 c)
{
	return ((c >> DEFAULT_RGB_R_SHIFT_32) & 0xFF);
}


int getg32(uint32 c)
{
	return ((c >> DEFAULT_RGB_G_SHIFT_32) & 0xFF);
}


int getb32(uint32 c)
{
	return ((c >> DEFAULT_RGB_B_SHIFT_32) & 0xFF);
}


int geta32(uint32 c)
{
	return ((c >> DEFAULT_RGB_A_SHIFT_32) & 0xFF);
}


uint32 makeacol32(uint32 r, uint32 g, uint32 b, uint32 a)
{
	return ((r << DEFAULT_RGB_R_SHIFT_32) |
		(g << DEFAULT_RGB_G_SHIFT_32) |
		(b << DEFAULT_RGB_B_SHIFT_32) |
		(a << DEFAULT_RGB_A_SHIFT_32));
}

#pragma endregion

#pragma region Pixel32_Definition

struct Pixel32 {

public:
	Pixel32(uint32 r = 0, uint32 g = 0, uint32 b = 0, uint32 alpha = 0);
	Pixel32(b2Color color);
	~Pixel32() = default;
	uint32 GetColorAsInt();
	uint32 Red;
	uint32 Green;
	uint32 Blue;
	uint32 Alpha;

};

Pixel32::Pixel32(uint32 r, uint32 g, uint32 b, uint32 alpha) {
	Red = r;
	Blue = b;
	Green = g;
	Alpha = alpha;
}

Pixel32::Pixel32(b2Color color) {
	Red = (uint32) 255.0 - color.r*255.0f;
	Blue = (uint32)  255.0 -  color.b*255.0f;
	Green = (uint32)  255.0 -  color.g*255.0f;
	Alpha = (uint32)  color.a*255.0f;
}

uint32 Pixel32::GetColorAsInt() {
	return makeacol32(Red, Green, Blue, Alpha);
}

#pragma endregion

void _DrawPixel(uint32 **longbufferBitmap, int x, int y, int agsColor, int width, int height) {
	if (x >= 0 && x < width && y >= 0 && y < height) {
		longbufferBitmap[y][x] = agsColor;
	}
}

void _DrawCircle(
	uint32 **lbb,
	unsigned int x0,
	unsigned int y0,
	unsigned int radius,
	uint32 agsColor, int width, int height)
{
	int f = 1 - radius;
	int ddF_x = 0;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;

	_DrawPixel(lbb, x0, y0 + radius, agsColor, width, height);
	_DrawPixel(lbb, x0, y0 - radius, agsColor, width, height);
	_DrawPixel(lbb, x0 + radius, y0, agsColor, width, height);
	_DrawPixel(lbb, x0 - radius, y0, agsColor, width, height);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x + 1;
		_DrawPixel(lbb, x0 + x, y0 + y, agsColor, width, height);
		_DrawPixel(lbb, x0 - x, y0 + y, agsColor, width, height);
		_DrawPixel(lbb, x0 + x, y0 - y, agsColor, width, height);
		_DrawPixel(lbb, x0 - x, y0 - y, agsColor, width, height);
		_DrawPixel(lbb, x0 + y, y0 + x, agsColor, width, height);
		_DrawPixel(lbb, x0 - y, y0 + x, agsColor, width, height);
		_DrawPixel(lbb, x0 + y, y0 - x, agsColor, width, height);
		_DrawPixel(lbb, x0 - y, y0 - x, agsColor, width, height);
	}
}

void _DrawLine(uint32 **longbufferBitmap, int x0, int y0, int x1, int y1, int agsColor, int width, int height) {

	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;

	for (;;) {
		_DrawPixel(longbufferBitmap, x0, y0, agsColor, width, height);
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 > -dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
	}
}

void AgsDebugDraw::InitializeAgsDebugDraw(IAGSEngine* engine, int screenWidth, int screenHeight, int colDepth) {
	Engine = engine;
	ScreenWidth = screenWidth;
	ScreenHeight = screenHeight;
	CameraX = 0.0f;
	CameraY = 0.0f;

}

int AgsDebugDraw::GetDebugSprite() {
	if(Engine == nullptr || SpriteId <= 0) return 0;

	Engine->NotifySpriteUpdated(SpriteId);
	return SpriteId;
}

void AgsDebugDraw::GetSurfaceForDebugDraw(int camera_x, int camera_y) {
	if (Engine == nullptr) return;
	if (SpriteId <= 0)  SpriteId = Engine->CreateDynamicSprite(32, ScreenWidth, ScreenHeight);

	CameraX = Scale::ScaleDown((float32) camera_x);
	CameraY = Scale::ScaleDown((float32) camera_y);

	BITMAP *engineSprite = Engine->GetSpriteGraphic(SpriteId);
	unsigned char **charbuffer = Engine->GetRawBitmapSurface(engineSprite);
	Longbuffer = (uint32**)charbuffer;
}

void AgsDebugDraw::ReleaseSurfaceForDebugDraw() {
	if (Engine == nullptr || SpriteId <= 0) return;

	BITMAP *engineSprite = Engine->GetSpriteGraphic(SpriteId);
	Engine->ReleaseBitmapSurface(engineSprite);
	Engine->NotifySpriteUpdated(SpriteId);
}


void AgsDebugDraw::ClearSprite() {
	if (Engine == nullptr || SpriteId <= 0) return;

	BITMAP *engineSprite = Engine->GetSpriteGraphic(SpriteId);
	unsigned char **charbuffer = Engine->GetRawBitmapSurface(engineSprite);
	uint32 **longbuffer = (uint32**)charbuffer;

	uint32 clearColor = makeacol32(0, 0, 0, 255);

	int srcWidth, srcHeight;
	Engine->GetBitmapDimensions(engineSprite, &srcWidth, &srcHeight, nullptr);

	for (int y = 0; y < srcHeight; y++) {
		for (int x = 0; x < srcWidth; x++) {
			_DrawPixel(longbuffer, x, y, clearColor, srcWidth, srcHeight);
		}
	}

	Engine->ReleaseBitmapSurface(engineSprite);
}


void AgsDebugDraw::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color) {
	if(Engine == nullptr || SpriteId <= 0) return;

	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	_DrawPixel(Longbuffer, Scale::ScaleUp(p.x-CameraX), Scale::ScaleUp(p.y-CameraY), 0, ScreenWidth, ScreenHeight);
}

void AgsDebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	if(Engine == nullptr || SpriteId <= 0) return;

	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	for (int i = 0; i < vertexCount; i++) {
		if (i == 0) {
			_DrawPixel(Longbuffer, Scale::ScaleUp(vertices[i].x-CameraX), Scale::ScaleUp(vertices[i].y-CameraY), agscolor, ScreenWidth, ScreenHeight);
		}
		else {
			_DrawLine(Longbuffer,
				Scale::ScaleUp(vertices[i-1].x-CameraX), Scale::ScaleUp(vertices[i-1].y-CameraY),
				Scale::ScaleUp(vertices[i].x-CameraX), Scale::ScaleUp(vertices[i].y-CameraY),
				agscolor, ScreenWidth, ScreenHeight);
		}
		if(i==vertexCount-1){
			_DrawLine(Longbuffer,
				Scale::ScaleUp(vertices[0].x-CameraX), Scale::ScaleUp(vertices[0].y-CameraY),
				Scale::ScaleUp(vertices[i].x-CameraX), Scale::ScaleUp(vertices[i].y-CameraY),
				agscolor, ScreenWidth, ScreenHeight);
		}
	}

}

void AgsDebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	if(Engine == nullptr || SpriteId <= 0) return;

	DrawPolygon(vertices, vertexCount, color);
}

void AgsDebugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {
	if(Engine == nullptr || SpriteId <= 0) return;

	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	_DrawCircle(Longbuffer,
		Scale::ScaleUp(center.x-CameraX), Scale::ScaleUp(center.y-CameraY),
		Scale::ScaleUp(radius), agscolor, ScreenWidth, ScreenHeight);
}

void AgsDebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
    if(Engine == nullptr || SpriteId <= 0) return;

    Pixel32 pix = Pixel32(color);
    int agscolor = pix.GetColorAsInt();

    _DrawLine(Longbuffer,
              Scale::ScaleUp(p1.x-CameraX), Scale::ScaleUp(p1.y-CameraY),
              Scale::ScaleUp(p2.x-CameraX), Scale::ScaleUp(p2.y-CameraY),
              agscolor, ScreenWidth, ScreenHeight);
}

void AgsDebugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
	if(Engine == nullptr || SpriteId <= 0) return;

	DrawCircle(center, radius, color);

    b2Vec2 p = center + radius * axis;
    AgsDebugDraw::DrawSegment(center, p, color);
}



void AgsDebugDraw::DrawTransform(const b2Transform& xf) {
	if(Engine == nullptr || SpriteId <= 0) return;

    const float32 k_axisScale = 0.3f;
    b2Color red(1.0f, 0.0f, 0.0f);
    b2Color green(0.0f, 1.0f, 0.0f);
    b2Vec2 p1 = xf.p, p2;

    p2 = p1 + k_axisScale * xf.q.GetXAxis();
    AgsDebugDraw::DrawSegment(p1, p2, red);

    p2 = p1 + k_axisScale * xf.q.GetYAxis();
    AgsDebugDraw::DrawSegment(p1, p2, green);
}
