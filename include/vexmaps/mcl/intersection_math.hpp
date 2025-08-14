#pragma once

namespace vexmaps {
extern "C" void circleIntersection(float* res,
                                   float* x,
                                   float* y,

                                   int len,

                                   float vx,
                                   float vy,

                                   float ox,
                                   float oy,
                                   float r);
}
