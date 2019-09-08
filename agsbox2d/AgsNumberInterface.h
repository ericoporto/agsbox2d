/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_NUMBER_INTERFACE_H
#define _AGS_NUMBER_INTERFACE_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

union
{
	float32 f;
	uint32_t ui32;
} AgsNumber;

uint32_t ToAgsFloat(float32 f) {
	AgsNumber.f = f;
	return AgsNumber.ui32;
}

float32 ToNormalFloat(uint32_t ui32) {
	AgsNumber.ui32 = ui32;
	return AgsNumber.f;
}

#endif /* _AGS_NUMBER_INTERFACE_H */

//..............................................................................