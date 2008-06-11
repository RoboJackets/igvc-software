#include "VideoView.h"
#include "vision/vision.h"



// The view that shows exactly what the camera sees
VisColorView rawView = VisColorView("Raw", &visRaw);

// Debug view
VisColorView debugView = VisColorView("Debug", &visRaw);
