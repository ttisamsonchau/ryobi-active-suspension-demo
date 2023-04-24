#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define MATH_PI 3.14159274f
#define MATH_RAD_TO_DEG 57.2957795f
#define MATH_DEG_TO_MS 0.01745329251f
#define MATH_MS_TO_RPM 9.54929658551f

#define U8_MAX ((uint8_t)255)
#define S8_MAX ((int8_t)127)
#define S8_MIN ((int8_t)-128)
#define U16_MAX ((uint16_t)65535u)
#define S16_MAX ((int16_t)32767)
#define S16_MIN ((int16_t)-32768)
#define U32_MAX ((uint32_t)4294967295uL)
#define S32_MAX ((int32_t)2147483647)
#define S32_MIN ((int32_t)-2147483648)

#define LIMIT(A, B, C) ((A) <= (B) ? (B) : ((A) >= (C) ? (C) : (A)))
#define ADD(p, q) (p) + (q)
#define MAX(a, b) ((a < b) ? (b) : (a))
#define AVERAGE(x, y) (((x) + (y)) / 2.0)
#define ABS(x) (((x) < 0) ? -(x) : (x))
#define DEADBAND(x, y) ((ABS(x) > y) ? x : 0)
#define SWAP(a, b) ({a ^= b; b ^= a; a ^= b; })
#define SQUARE(x) (x * x)

// float mapfloat(int x, float in_min, float in_max, float out_min, float out_max)
// {
//     float inrange = in_max - in_min;
//     float outrange = out_max - out_min;
//     float output = out_min + (outrange / inrange) * (x - in_min);
//     return output;
// }

// uint8_t exp_axis_curve(int8_t value, float soften){
//   return (int8_t) value * pow(abs(value/127), soften);
// }
