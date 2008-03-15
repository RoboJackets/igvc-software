/*
  DC1394.h
  
  DC1394-wrapper-class
  
  1.9.04 G.Glock
*/

#ifndef DC1394_H
#define DC1394_H

#include "constants.h"
#include "messagelogger.h"

#include <qobject.h>
#include <qstring.h>

#include <libdc1394/dc1394_control.h>
#include <libavt1394/avt1394.h>



class DC1394 : public QObject {
    Q_OBJECT
    
public:
    DC1394(MessageLogger *ml);
    ~DC1394();
    
    static char *featureString(unsigned int featureIndex);
    
    int initRaw1394();
    
    int dc1394Init(QString devName, bool subDirs = false, unsigned int devNo = 0);
    bool dc1394Cleanup();
    
    bool createHandle(unsigned int id);
    bool destroyHandle(unsigned int id);
    raw1394handle_t getHandle(unsigned int id);
    
    bool getCameraID(unsigned int id, unsigned int *camID);
    
    unsigned int getPortNo(unsigned int id);
    unsigned int getNodeNo(unsigned int id);
    
    quadlet_t getSupportedFormats(unsigned int id);
    quadlet_t getSupportedModes(unsigned int id, unsigned int format);
    quadlet_t getSupportedFramerates(unsigned int id, unsigned int format, unsigned int mode);
    
    bool setParameters(unsigned int id, unsigned int format, unsigned int mode,
                       unsigned int framerate, unsigned int colorID);
    
    unsigned int getFormat(unsigned int id);
    unsigned int getMode(unsigned int id);
    unsigned int getFramerate(unsigned int id);
    unsigned int getWidth(unsigned int id);
    unsigned int getHeight(unsigned int id);
    unsigned int getX(unsigned int id);
    unsigned int getY(unsigned int id);
    unsigned int getF7FPS(unsigned int id);
    
    QString getFramerateString(unsigned int id);
    QString getColorCodingIDString(unsigned int id);
    QString getModeString(unsigned int id);
    
    unsigned int getIsoChannel(unsigned int id);
    unsigned int getIsoSpeed(unsigned int id);
    bool setIsoSpeed(unsigned int id, unsigned int speed);
    
    bool dmaSetup(unsigned int id);
    bool dmaF7Setup(unsigned int id);
    bool dmaRelease(unsigned int id);
    
    bool startIsoTransmission(unsigned int id);
    bool stopIsoTransmission(unsigned int id);
    bool getIsoStatus(unsigned int id, bool *status);
    
    int getNumberOfCameras();
    char *getCameraModel(unsigned int id);
    char *getCameraVendor(unsigned int id);
    octlet_t getCCROffset(unsigned int id);
    
    u_int64_t getEUID(unsigned int id);
    bool getSWVersion(unsigned int id, int *swVersion);
    bool getRevision(unsigned int id, quadlet_t *revision);
    bool getuCVersion(unsigned int id, unsigned int *major, unsigned int *minor);
    bool getFirmwareVersion(unsigned int id, unsigned int *camID, unsigned int *major,
                            unsigned int *minor, bool *marlin);
    
    bool dmaSingleCapture(unsigned int id);
    bool dmaReleaseBuffer(unsigned int id);
    int *getCaptureBuffer(unsigned int id);
    
    bool getCurrentCaptureMode(unsigned int id, unsigned int *mode, unsigned int *frameCount);
    
    dc1394_cameracapture getCameracapture(unsigned int id);
    dc1394_feature_info featureInfo(unsigned int id, unsigned int feature);
    dc1394_feature_set featureSet(unsigned int id);
    
    bool featureReadOutCapable(unsigned int id, int featureIndex);
    
    bool dc1394InitCamera(unsigned int id);
    
    // direct access
    bool readData(unsigned int id, unsigned long address, unsigned int *data, bool sio = false,
                  bool f7 = false, unsigned int f7Mode = 0);
    bool writeData(unsigned int id, unsigned long address, unsigned int data, bool sio = false,
                   bool f7 = false, unsigned int f7Mode = 0);
    
    // Std. Features
    bool setFeatureValue(unsigned int id, int feature, int newValue);
    bool setWhiteBalanceValue(unsigned int id, int newUValue, int newVValue);
    bool setFeatureAutoMode(unsigned int id, int feature, bool newValue);
    bool setFeatureOn(unsigned int id, int feature, bool newValue);
    bool featureOnePush(unsigned int id, int feature);
    // Std. Trigger
    bool setTriggerMode(unsigned int id, int mode);
    bool setTriggerPolarity(unsigned int id, dc1394bool_t polarity);
    int getTriggerMode(unsigned int id);
    bool getTriggerModeCapableMask(unsigned int id, unsigned int *mask);
    bool getTriggerState(unsigned int id, bool *state);
    bool getTriggerSources(unsigned int id, unsigned int *sources);
    bool getCurrentTriggerSource(unsigned int id, unsigned int *source);
    bool setTriggerSource(unsigned int id, unsigned int source);
    bool swTriggerAvailable(unsigned int in);
    bool getSWTriggerState(unsigned int id, bool *state);
    bool startSWTriggerExecute(unsigned int id);
    bool stopSWTriggerExecute(unsigned int id);
    // Std. Trigger-Delay
    bool triggerDelayAvailable(unsigned int id);
    bool getTriggerDelayInfo(unsigned int id, unsigned int *info);
    bool triggerDelayEnabled(unsigned int id, bool *on);
    bool triggerDelayEnable(unsigned int id, bool on);
    bool getTriggerDelay(unsigned int id, unsigned int *delay);
    bool setTriggerDelay(unsigned int id, unsigned int delay);
    
    bool enableAbsoluteControl(unsigned int id, unsigned int feature, bool enable);
    bool enableTriggerDelayAbsControl(unsigned int id, bool enable);
    
    bool printISOState(unsigned int id);
    
    // Format 7
    bool f7Available(unsigned int id);
    unsigned int getF7ColorCodingID(unsigned int id, unsigned int md);
    bool getF7ColorCoding(unsigned int id, unsigned int mode, unsigned int *colorCoding);
    bool getF7MaxSize(unsigned int id, unsigned int mode,
                      unsigned int *maxX, unsigned int *maxY);
    bool getF7ImageSize(unsigned int id, unsigned int *width, unsigned int *height);
    bool setF7ImageSize(unsigned int id, unsigned int width, unsigned int height);
    bool getF7ImagePos(unsigned int id, unsigned int *x, unsigned int *y);
    bool setF7ImagePos(unsigned int id, unsigned int x, unsigned int y);
    bool getF7Units(unsigned int id, unsigned int *hUnitSize, unsigned int *vUnitSize,
                    unsigned int *hPosUnit, unsigned int *vPosUnit);
    bool getF7PixelNumber(unsigned int id, unsigned int *pixelNumber);
    bool getF7TotalBytes(unsigned int id, unsigned long long int *totalBytes);
    bool getF7PacketPerFrame(unsigned int id, unsigned int *ppf);
    bool getF7BytePerPacket(unsigned int id, unsigned int *bpp);
    bool setF7BytePerPacket(unsigned int id, unsigned int bpp);
    bool getF7RecBytePerPacket(unsigned int id, unsigned int *bpp);
    bool getF7PacketParameters(unsigned int id, unsigned int *minBPP, unsigned int *maxBPP);
    
    // adv. Features (AVT-specific)
    bool extShutterAvailable(unsigned int id);
    bool getExtShutter(unsigned int id, unsigned int *time);
    bool setExtShutter(unsigned int id, unsigned int time);
    bool getExposureTime(unsigned int id, unsigned int *time, bool *valid);
    bool getTimebase(unsigned int id, unsigned int *timebase);
    bool setTimebase(unsigned int id, unsigned int timebase);
    void getShutterOffset(unsigned int id, unsigned int *offset);
    
    bool shadingCorrectionAvailable(unsigned int id);
    bool shadingCorrectionBusy(unsigned int id, bool *busy);
    bool getShadingCorrectionInfos(unsigned int id, bool *showImage,
                                   unsigned int *grabCount,
                                   unsigned int *maxSize,
                                   bool *on);
    bool shadingCorrectionValid(unsigned int id);
    bool shadingCorrectionBuildImage(unsigned int id, unsigned int grabCount);
    bool shadingCorrectionLoadImage(unsigned int id, unsigned char *img, unsigned int size);
    bool shadingCorrectionReadImage(unsigned int id, unsigned char *img, unsigned int size);
    bool shadingCorrectionShowImage(unsigned int id, bool show);
    bool shadingCorrectionEnable(unsigned int id, bool enable);
    bool shadingCorrectionSetGrabCount(unsigned int id, unsigned int grabCount);
    
    bool lutAvailable(unsigned int id);
    bool getLUTInfos(unsigned int id, unsigned int *maxLutNo, unsigned int *maxSize, bool *on);
    bool getLUTSelection(unsigned int id, unsigned int *lutNo);
    bool loadLUT(unsigned int id, unsigned char *lut, unsigned int size, unsigned int lutNo);
    bool selectLUT(unsigned int id, unsigned int lutNo);
    bool lutEnable(unsigned int id, bool enable);
    
    bool hdrAvailable(unsigned int id);
    bool getHDRInfos(unsigned int id, unsigned int *maxKnees, unsigned int *knee1,
                     unsigned int *knee2, unsigned int *knee3, unsigned int *activeKnees,
                     bool *on);
    bool hdrSetValues(unsigned int id, unsigned int numKnees, unsigned int knee1,
                      unsigned int knee2, unsigned int knee3);
    bool hdrEnable(unsigned int id, bool enable);
    
    bool mirrorImageAvailable(unsigned int id);
    bool getMirrorImageState(unsigned int id, bool *state);
    bool mirrorImageEnable(unsigned int id, bool enable);
    
    bool testImageAvailable(unsigned int id);
    bool getAvailableTestImages(unsigned int id, unsigned int *testImages);
    bool getTestImage(unsigned int id, unsigned int *imageNo);
    bool testImageSelect(unsigned int id, unsigned int imageNo);
    bool testImageEnable(unsigned int id, bool enable, unsigned int testImageNo);
    
    bool colorCorrAvailable(unsigned int id);
    bool getColorCorrState(unsigned int id, bool *state);
    bool colorCorrEnable(unsigned int id, bool enable);
    
    bool ioAvailable(unsigned int id, unsigned int ioNo);
    bool getIOInCtrl(unsigned int id, unsigned int ioNo, bool *polarity, unsigned int *mode,
                     bool *state);
    bool getIOOutCtrl(unsigned int id, unsigned int ioNo, bool *polarity, unsigned int *mode,
                      bool *state);
    bool setIOInCtrl(unsigned int id, unsigned int ioNo, bool polarity, unsigned int mode);
    bool setIOOutCtrl(unsigned int id, unsigned int ioNo, bool polarity, unsigned int mode,
                      bool state);
    
    bool serialAvailable(unsigned int id);
    bool getSerialParameters(unsigned int id, unsigned int *baud, unsigned int *len,
                             unsigned int *par, unsigned int *stop,
                             unsigned int *bufSize);
    bool setSerialParameters(unsigned int id, unsigned int baud, unsigned int len,
                             unsigned int par, unsigned int stop);
    bool getSerialControl(unsigned int id, bool *rxEnable, bool *txEnable,
                          bool *rxBufferReady, bool *txBufferReady,
                          bool *overrunError, bool *framingError, bool *parityError);
    bool setSerialControl(unsigned int id, bool rxEnable, bool txEnable, bool clearErrorFlags);
    bool getSerialBufferSize(unsigned int id, unsigned int *size);
    bool getNoOfRxBytes(unsigned int id, unsigned int *number);
    int readSerialData(unsigned int id, char *byte, unsigned int number);
    bool sendSerialData(unsigned int id, char *buffer, unsigned int reqSize);
    
    bool advTriggerDelayAvailable(unsigned int id);
    bool advTriggerDelayEnabled(unsigned int id, bool *on);
    bool advTriggerDelayEnable(unsigned int id, bool on);
    bool getAdvTriggerDelay(unsigned int id, unsigned int *delay);
    bool setAdvTriggerDelay(unsigned int id, unsigned int delay);
    
    bool dsnuAvailable(unsigned int id);
    bool blemishAvailable(unsigned int id);
    bool dsnuEnabled(unsigned int id, bool *on);
    bool dsnuEnable(unsigned int id, bool on);
    bool blemishEnabled(unsigned int id, bool *on);
    bool blemishEnable(unsigned int id, bool on);
    bool dsnuBusy(unsigned int id, bool *busy);
    bool blemishBusy(unsigned int id, bool *busy);
    bool getDSNUImgCnt(unsigned int id, unsigned int *cnt);
    bool getBlemishImgCnt(unsigned int id, unsigned int *cnt);
    bool setDSNUImgCnt(unsigned int id, unsigned int cnt);
    bool setBlemishImgCnt(unsigned int id, unsigned int cnt);
    bool dsnuShowImg(unsigned int id, bool show);
    bool blemishShowImg(unsigned int id, bool show);
    bool dsnuImgOn(unsigned int, bool *on);
    bool blemishImgOn(unsigned int, bool *on);
    bool dsnuComputeImg(unsigned int id);
    bool blemishComputeImg(unsigned int id);
    bool dsnuZeroImg(unsigned int id);
    bool blemishZeroImg(unsigned int id);
    bool dsnuLoadFactoryData(unsigned int id);
    bool blemishLoadFactoryData(unsigned int id);
    
    bool oneShotEnabled(unsigned int id, bool *on);
    bool multiShotEnabled(unsigned int id, bool *on, unsigned int *frames);
    bool freerunEnabled(unsigned int id, bool *on);
    bool enableOneShot(unsigned int id, bool on);
    bool enableMultiShot(unsigned int id, bool on, unsigned int frameNumber);
    bool getMultiShotFrameCount(unsigned int id, unsigned int *frames);
    bool enableFreerun(unsigned int id, bool on);
    
    bool autoShutterCtlAvailable(unsigned int id);
    bool getAutoShutterLimits(unsigned int id, unsigned int *minValue,
                              unsigned int *maxValue);
    bool setAutoShutterLimits(unsigned int id, unsigned int minValue,
                              unsigned int maxValue);
    
    bool autoGainCtlAvailable(unsigned int id);
    bool getAutoGainLimits(unsigned int id, unsigned int *minValue,
                              unsigned int *maxValue);
    bool setAutoGainLimits(unsigned int id, unsigned int minValue,
                              unsigned int maxValue);
    
    bool autoAOIAvailable(unsigned int id);
    bool getAutoAOISize(unsigned int id, unsigned int *width, unsigned int *height);
    bool setAutoAOISize(unsigned int id, unsigned int width, unsigned int height);
    bool getAutoAOIPos(unsigned int id, unsigned int *x, unsigned int *y);
    bool setAutoAOIPos(unsigned int id, unsigned int x, unsigned int y);
    bool getAutoAOIStatus(unsigned int id, bool *on, bool *areaShown);
    bool enableAutoAOI(unsigned int id, bool on);
    bool showAutoAOIArea(unsigned int id, bool show);
    
    bool deferredImgAvailable(unsigned int id);
    bool getDeferredImgStatus(unsigned int id, bool *status);
    bool enableDeferredImg(unsigned int id, bool enable);
    bool deferredImgSend(unsigned int id, unsigned int frameCount);
    bool deferredImgSendActive(unsigned int id, bool *sendActive);
    bool deferredImgFastCaptureEnable(unsigned int id, bool enable);
    bool deferredImgFastCaptureStatus(unsigned int id, bool *on);
    bool getDeferredImgFifoSize(unsigned int id, unsigned int *size);
    bool setDeferredImgFrameCount(unsigned int id, unsigned int number);
    bool getDeferredImgFrameCount(unsigned int id, unsigned int *number);
    
    bool frameInfoAvailable(unsigned int id);
    bool frameInfoReset(unsigned int id);
    bool getFrameInfoCount(unsigned int id, unsigned int *count);
    
    bool delIntAvailable(unsigned int id);
    bool delIntStatus(unsigned int id, bool *status);
    bool delIntEnable(unsigned int id, bool enable);
    bool getDelIntValue(unsigned int id, unsigned int *value);
    bool setDelIntValue(unsigned int id, unsigned int value);
    
    bool incDecAvailable(unsigned int id);
    bool incDecStatus(unsigned int id, bool *status);
    bool incDecEnable(unsigned int id, bool enable);
    bool getIncDecCompare(unsigned int id, unsigned int *value);
    bool setIncDecCompare(unsigned int id, unsigned int value);
    bool getIncDecCounter(unsigned int id, unsigned int *value);
    bool incDecReset(unsigned int id);
    
    bool softResetAvailable(unsigned int id);
    bool getSoftResetDelay(unsigned int id, unsigned int *delay);
    bool setSoftResetDelay(unsigned int id, unsigned int delay);
    bool softReset(unsigned int id);
    bool delayedSoftReset(unsigned int id, unsigned int delay);
    
    bool highSNRAvailable(unsigned int id);
    bool highSNRStatus(unsigned int id, unsigned int *grabCount, bool *enabled);
    bool highSNREnable(unsigned int id, bool enable);
    bool setHighSNRGrabCount(unsigned int id, unsigned int grabCount);
    
    bool seqCtlAvailable(unsigned int id);
    bool seqCtlStatus(unsigned int id, bool *status, bool *autoRewind, bool *autoInc);
    bool seqCtlEnable(unsigned int id, bool enable);
    bool seqCtlEnableAutoRewind(unsigned int id, bool enable);
    bool seqCtlEnableAutoInc(unsigned int id, bool enable);
    bool getSeqCtlInfo(unsigned int id, unsigned int *seqLimit, unsigned int *seqLength,
                       unsigned int *imgNo);
    bool setSeqCtlImageNo(unsigned int id, unsigned int imgNo);
    bool setSeqCtlLength(unsigned int id, unsigned int length);
    bool seqCtlApply(unsigned int id, bool autoInc);
    bool seqCtlBusy(unsigned int id, bool *busy);
    
    
protected:
    MessageLogger *logger;
    
    unsigned int numPorts;
    unsigned int camCount;
    
    bool isoStarted[MAX_CAMERAS];
    bool dmaSetUp[MAX_CAMERAS];
    
    unsigned int format[MAX_CAMERAS];
    unsigned int mode[MAX_CAMERAS];
    unsigned int framerate[MAX_CAMERAS];
    unsigned int colorCodingID[MAX_CAMERAS];
    
    unsigned int f7FPS[MAX_CAMERAS];
    
    unsigned int width[MAX_CAMERAS];
    unsigned int height[MAX_CAMERAS];
    unsigned int x[MAX_CAMERAS];
    unsigned int y[MAX_CAMERAS];
    
    QString deviceName[MAX_CAMERAS];
    
    unsigned int isoChannel[MAX_CAMERAS];
    unsigned int isoSpeed[MAX_CAMERAS];
    
    raw1394handle_t handles[MAX_CAMERAS];
    dc1394_cameracapture cameras[MAX_CAMERAS];
    dc1394_camerainfo camInfos[MAX_CAMERAS];
    dc1394_feature_set featureSets[MAX_CAMERAS];
    unsigned int colorCodings[NUM_MODE_FORMAT7][MAX_CAMERAS];
    unsigned int camPort[MAX_CAMERAS];
    unsigned int camNode[MAX_CAMERAS];
    
    bool shadingValid[MAX_CAMERAS];
    
    void setWidthAndHeight(unsigned int id);
    
};


#endif // DC1394_H
