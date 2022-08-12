
#pragma once

#if _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

namespace Utils
{
double DegToRad(double deg);
double RadToDeg(double rad);
} // namespace Utils
