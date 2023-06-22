#include "control_utils.h"

void Clamp(float *val, const float &min, const float &max)
{
    // Ensure that the value is within the constraints.
    if (*val < min)
    {
        *val = min;
    }
    else if (*val > max)
    {
        *val = max;
    }
}
