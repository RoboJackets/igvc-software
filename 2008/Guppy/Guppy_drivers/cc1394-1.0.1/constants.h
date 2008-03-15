/*
  constants.h
  
  definition of some global constants
  
  27.8.04 G.Glock
*/

#define DROP_FRAMES 1
#define MAX_PORTS   4
#define MAX_CAMERAS 8
#define NUM_BUFFERS 8

#define CCIR601_8BIT_BLACK 16
#define CCIR601_8BIT_WHITE  235
#define CCIR601_8BIT_CHROMAOFFSET 128

#define MIN_RGB_8BIT 0
#define MAX_RGB_8BIT 255

// format-constants
#define F0 0
#define F1 1
#define F2 2
#define F6 6
#define F7 7

// format-texts
#define FORMAT0 "Format 0 - up to 640x480"
#define FORMAT1 "Format 1 - up to 1024x768"
#define FORMAT2 "Format 2 - up to 1600x1200"
#define FORMAT6 "Format 6"
#define FORMAT7 "Format 7 - 'Partial Image'"

// framerate-texts
#define FR_1_875 "1.875 fps"
#define FR_3_75 "3.75 fps"
#define FR_7_5 "7.5 fps"
#define FR_15 "15 fps"
#define FR_30 "30 fps"
#define FR_60 "60 fps"

// resolution-texts
// format 0
#define F0_M0 "160x120 YUV 4:4:4"
#define F0_M1 "320x240 YUV 4:2:2"
#define F0_M2 "640x480 YUV 4:1:1"
#define F0_M3 "640x480 YUV 4:2:2"
#define F0_M4 "640x480 RGB"
#define F0_M5 "640x480 Mono 8bit"
#define F0_M6 "640x480 Mono 16bit"

// format 1
#define F1_M0 "800x600 YUV 4:2:2"
#define F1_M1 "800x600 RGB"
#define F1_M2 "800x600 Mono 8bit"
#define F1_M3 "1024x768 YUV 4:2:2"
#define F1_M4 "1024x768 RGB"
#define F1_M5 "1024x768 Mono 8bit"
#define F1_M6 "800x600 Mono 16bit"
#define F1_M7 "1024x768 Mono 16bit"

// format2
#define F2_M0 "1280x960 YUV 4:2:2"
#define F2_M1 "1280x960 RGB"
#define F2_M2 "1280x960 Mono 8bit"
#define F2_M3 "1600x1200 YUV 4:2:2"
#define F2_M4 "1600x1200 RGB"
#define F2_M5 "1600x1200 Mono 8bit"
#define F2_M6 "1280x960 Mono 16bit"
#define F2_M7 "1600x1200 Mono 16bit"

// format7
#define F7_M0 "Mode 0"
#define F7_M1 "Mode 1"
#define F7_M2 "Mode 2"
#define F7_M3 "Mode 3"
#define F7_M4 "Mode 4"

#define F7_CC0 "Mono 8bit"
#define F7_CC1 "YUV 4:1:1"
#define F7_CC2 "YUV 4:2:2"
#define F7_CC3 "YUV 4:4:4"
#define F7_CC4 "RGB 8bit"
#define F7_CC5 "Mono 16bit"
#define F7_CC6 "RGB 16bit"




// dc1394-masks

// formats
#define F0_MASK 0x80000000
#define F1_MASK 0x40000000
#define F2_MASK 0x20000000
#define F6_MASK 0x02000000
#define F7_MASK 0x01000000

// modes
#define M0_MASK 0x80000000
#define M1_MASK 0x40000000
#define M2_MASK 0x20000000
#define M3_MASK 0x10000000
#define M4_MASK 0x08000000
#define M5_MASK 0x04000000
#define M6_MASK 0x02000000
#define M7_MASK 0x01000000


// AVT-specific Camera-IDs
// Dolphin-Cameras
#define F145b 1
#define F145c 2
#define F201b 3
#define F201c 4
#define F145b_1 5
#define F145c_1 6
#define F201b_1 7
#define F201c_1 8
// Marlin-Cameras
#define MF033B 9
#define MF033C 10
#define MF046B 11
#define MF046C 12
#define MF080B 13
#define MF080C 14
#define MF145B2 15
#define MF145C2 16
#define MF131B 17
#define MF131C 18
#define MF145B2_15fps 19
#define MF145C2_15fps 20


// AVT-specific FrameBuffer Sizes
//  Marlin-Cameras
#define MF131_FB_SIZE 4
#define MF145_FB_SIZE 3
#define MF033_FB_SIZE 13
#define MF046_FB_SIZE 13
#define MF080_FB_SIZE 7

//  Dolphin-Cameras (tbd)
#define F145_FB_SIZE 16
#define F201_FB_SIZE 16
#define F145_1_FB_SIZE 16
#define F201_1_FB_SIZE 16


// I/O-Mode constants

// input
#define IO_IN_OFF 0x00
#define IO_IN_OFF_STRING "Off"
#define IO_IN_TRIGGER 0x02
#define IO_IN_TRIGGER_STRING "Trigger"
#define IO_IN_INC_DEC 0x03
#define IO_IN_INC_DEC_STRING "Inc. Dec. In"
#define IO_IN_SPI 0x05
#define IO_IN_SPI_STRING "SPI ext. DCLK"

// output
#define IO_OUT_OFF 0x00
#define IO_OUT_OFF_STRING "Off"
#define IO_OUT_PIN_STATE 0x01
#define IO_OUT_PIN_STATE_STRING "Output State"
#define IO_OUT_INT_ENA 0x02
#define IO_OUT_INT_ENA_STRING "Integr. Enable"
#define IO_OUT_INC_DEC_CMP 0x03
#define IO_OUT_INC_DEC_CMP_STRING "Inc. Dec. Cmp"
#define IO_OUT_SPI_INT 0x04
#define IO_OUT_SPI_INT_STRING "SPI int. DCLK"
#define IO_OUT_SPI_EXT 0x05
#define IO_OUT_SPI_EXT_STRING "SPI ext. DCLK"
#define IO_OUT_FRAME_VALID 0x06
#define IO_OUT_FRAME_VALID_STRING "Frame Valid"
#define IO_OUT_BUSY 0x07
#define IO_OUT_BUSY_STRING "Busy"
#define IO_OUT_CORR_IN 0x08
#define IO_OUT_CORR_IN_STRING "Corresp. Input"


// Serial-I/O constants

#define SERIAL_NO_PARITY 0
#define SERIAL_ODD_PARITY 1
#define SERIAL_EVEN_PARITY 2

#define SERIAL_1_STOPBIT 0
#define SERIAL_3_HALF_STOPBIT 1
#define SERIAL_2_STOPBIT 2

// GP-I/O3 - Serial-Interface-WidgetStack-IDs

#define IO3_CONTROL 0
#define SIO_CONTROL 1


// Trigger-Mode constants

// Features Inquiry Register
#define FEATURE_HI_INQ_OFFSET 0x00000404UL

// Trigger Register Offsets
#define TRIGGER_INQ_OFFSET 0x00000530UL
#define TRIGGER_MODE_OFFSET 0x00000830UL

// Features Inquiry Register Mask for TriggerDelay
#define TRIGGER_DLY_AVAIL_MASK 0x00040000UL
// TriggerDelay Inquiry Register Constants (triggerDelayInfo)
#define TRIGGER_DLY_INQ_OFFSET 0x00000534UL
#define TRIGGER_DLY_ABSCTL_MASK 0x40000000UL
#define TRIGGER_DLY_ONEPUSH_MASK 0x10000000UL
#define TRIGGER_DLY_READOUT_MASK 0x08000000UL
#define TRIGGER_DLY_ONOFF_MASK 0x04000000UL
#define TRIGGER_DLY_AUTO_MODE_MASK 0x02000000UL
#define TRIGGER_DLY_MANUAL_MODE_MASK 0x01000000UL
#define TRIGGER_DLY_MINMAX_MASK 0x00000fffUL
// TriggerDelay Value Register Constants
#define TRIGGER_DLY_OFFSET 0x00000834UL
#define TRIGGER_DLY_PRESENT_MASK 0x80000000UL
#define TRIGGER_DLY_ON_MASK 0x02000000UL
#define TRIGGER_DLY_VALUE_MASK 0x00000fffUL
#define TRIGGER_DLY_ABS_MASK 0x04000000UL

#define RAW_TRIGGER_MODE_MASK 0xfff0ffffUL
#define RAW_TRIGGER_MODE_15 0x000f0000

#define SW_TRIGGER_OFFSET 0x0000062cUL
#define SW_TRIGGER_SET 0x80000000UL
#define SW_TRIGGER_RESET 0x00000000UL

#define SW_TRIGGER_READY true
#define SW_TRIGGER_BUSY false


// Capture-Mode constants

#define CAPTURE_MODE_FREERUN_STRING "Free-Run"
#define CAPTURE_MODE_MULTISHOT_STRING "Multi-Shot"
#define CAPTURE_MODE_ONESHOT_STRING "One-Shot"

#define CAPTURE_MODE_UNSUPPORTED 0
#define CAPTURE_MODE_FREERUN 1
#define CAPTURE_MODE_ONESHOT 2
#define CAPTURE_MODE_MULTISHOT 3


// DRM-Button-Group-IDs

#define STDADDR 0
#define ADVADDR 1
#define F7ADDR 2
#define SIOADDR 3

