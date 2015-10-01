#include "common.h"

float normal_pdf(float x, float m, float s)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}

int compareIndexedAngle (const void * a, const void * b)
{ 
  float _t = ( ((IndexedAngle*)a)->angle - ((IndexedAngle*)b)->angle );
  if (_t < 0 ) return 1;
  else if (_t > 0 ) return -1;
  else return 0;
}
