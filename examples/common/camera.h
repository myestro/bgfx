/*
 * Copyright 2013 Dario Manesku. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx#license-bsd-2-clause
*/

#ifndef CAMERA_H_HEADER_GUARD
#define CAMERA_H_HEADER_GUARD

#include "entry/entry.h"

#define CAMERA_KEY_FORWARD   UINT8_C(0x01)
#define CAMERA_KEY_BACKWARD  UINT8_C(0x02)
#define CAMERA_KEY_LEFT      UINT8_C(0x04)
#define CAMERA_KEY_RIGHT     UINT8_C(0x08)
#define CAMERA_KEY_UP        UINT8_C(0x10)
#define CAMERA_KEY_DOWN      UINT8_C(0x20)

///
void cameraCreate();

///
void cameraDestroy();

void cameraReset();

///
void cameraSetPosition(const float* _pos);

///
void cameraSetHorizontalAngle(float _horizontalAngle);
float cameraGetHorizontalAngle();


///
void cameraSetVerticalAngle(float _verticalAngle);
float cameraGetVerticalAngle();

///
void cameraSetKeyState(uint8_t _key, bool _down);

///
void cameraGetViewMtx(float* _viewMtx);

///
void cameraGetPosition(float* _pos);

///
void cameraGetAt(float* _at);

void cameraSetAt(float* _at);

void cameraGetUp(float* _up);

void cameraSetUp(float* _up);

///
void cameraUpdate(float _deltaTime, const entry::MouseState& _mouseState);

void cameraSetSpeed(float _move, float _mouse);

#endif // CAMERA_H_HEADER_GUARD
