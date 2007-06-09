#include "VideoView.h"
#include "vision/vision.h"
#include "vision/vision_barrels.h"
#include "vision/vision_navigation.h"
#include "vision/vision_color.h"
#include "vision/vision_line_blobber.h"

// Same as "rawView", but with a graphics representation of the
// current navigation parameters superimposed
VisColorView navigationParamsView = VisColorView("Navigation Params", &visNavigationParams);

// The view that shows exactly what the camera sees
VisColorView rawView = VisColorView("Raw", &visRaw);

// Debugging view
VisColorView testView = VisColorView("Debug", &visTestViewContent);

VisColorView whiteDetectionCalibrationView = VisColorView("Calibrate: Color - White: General", &visWhiteDetectionCalibration);
VisColorView whiteConditionView = VisColorView("Calibrate: Color - White: Conditions", &visWhiteCondition);

VisGrayView redMinusGreenView = VisGrayView("Red-Green", &visRedMinusGreen);

// Same as "rawView", but with identified barrels boxed
VisColorView barrelView = VisColorView("Barrels", &visBarrels);

VisBlackAndWhiteView orangeView = VisBlackAndWhiteView("Color - Orange", &pixelIsOrange);
VisBlackAndWhiteView whiteView = VisBlackAndWhiteView("Color - White", &pixelIsWhite);
VisBlackAndWhiteView paulView = VisBlackAndWhiteView("PAUL BLOB!!", &paulBlob);
//VisBlackAndWhiteView yellowView = VisBlackAndWhiteView("Color - Yellow", &pixelIsYellow);

VisGrayView HSBHue = VisGrayView("HSB - Hue", &visHSBHue);
VisGrayView HSBSaturation = VisGrayView("HSB - Saturation", &visHSBSaturation);
VisGrayView HSBBrightness = VisGrayView("HSB - Brightness", &visHSBBrightness);

VisGrayView HSLHue = VisGrayView("HSL - Hue", &visHSLHue);
VisGrayView HSLSaturation = VisGrayView("HSL - Saturation", &visHSLSaturation);
VisGrayView HSLLightness = VisGrayView("HSL - Lightness", &visHSLLightness);
