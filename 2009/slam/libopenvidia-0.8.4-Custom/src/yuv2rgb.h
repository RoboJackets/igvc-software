#ifndef YUV2RGB_DOT_H
#define YUV2RGB_DOT_H

/* ripped from xawtv's libng/color_yuv2rgb.c  */
/* needed by yuv2rgb() in V4L1.cc */

#define CLIP        320
#define RED_NULL    128
#define BLUE_NULL   128
#define LUN_MUL     256
#define RED_MUL     512
#define BLUE_MUL    512

#define GREEN1_MUL  (-RED_MUL/2)
#define GREEN2_MUL  (-BLUE_MUL/6)
#define RED_ADD     (-RED_NULL  * RED_MUL)
#define BLUE_ADD    (-BLUE_NULL * BLUE_MUL)
#define GREEN1_ADD  (-RED_ADD/2)
#define GREEN2_ADD  (-BLUE_ADD/6)


/* lookup tables */
static unsigned int  ng_yuv_gray[256];
static unsigned int  ng_yuv_red[256];
static unsigned int  ng_yuv_blue[256];
static unsigned int  ng_yuv_g1[256];
static unsigned int  ng_yuv_g2[256];
static unsigned int  ng_clip[256 + 2 * CLIP];

/* conversion macros */
#define GRAY(val)               ng_yuv_gray[val]
#define RED(gray,red)           ng_clip[ CLIP + gray + ng_yuv_red[red] ]
#define GREEN(gray,red,blue)    ng_clip[ CLIP + gray + ng_yuv_g1[red] + \
                                                       ng_yuv_g2[blue] ]
#define BLUE(gray,blue)         ng_clip[ CLIP + gray + ng_yuv_blue[blue] ]

/* prototypes */
void yuv2rgb_init(void);
static void yuv2rgb (char *out_addr, char *in_addr, int rowstride, int width, int height);

#endif /* YUV2RGB_DOT_H */
