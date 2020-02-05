#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "dcache-control.h"
#include "AsciiFont.h"
#include "r_dk2_if.h"
#include "r_drp_bayer2grayscale.h"
#include "r_drp_image_rotate.h"
#include "r_drp_median_blur.h"
#include "r_drp_canny_calculate.h"
#include "r_drp_canny_hysterisis.h"
#include "r_drp_binarization_fixed.h"
#include "r_drp_erode.h"
#include "r_drp_dilate.h"
#include "r_drp_gaussian_blur.h"
#include "r_drp_sobel.h"
#include "r_drp_prewitt.h"
#include "r_drp_laplacian.h"
#include "r_drp_unsharp_masking.h"
#include "r_drp_cropping.h"
#include "r_drp_resize_bilinear_fixed.h"
#include "r_drp_histogram_normalization.h"

#define RAM_TABLE_DYNAMIC_LOADING   1
// 0: Use the configuration data stored in ROM directly.
// 1: Deploy configuration data to RAM to speed up loading to DRP.

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         (640)
#define VIDEO_PIXEL_VW         (480)

#define DATA_SIZE_PER_PIC      (1u)
#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)

#define DRP_FLG_TILE_ALL       (R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5)
#define DRP_FLG_CAMER_IN       (0x00000100)

#define DRP_LIB_MAX            (10)

typedef struct {
    uint32_t  drp_lib_no;
    uint8_t * p_drp_lib_bin;
    uint8_t * src;
    uint8_t * dst;
    uint32_t  load_time;
    uint32_t  run_time;
} drp_lib_ctl_t;

typedef void (*drp_func_t)(drp_lib_ctl_t * p_drp_lib_ctl);

typedef struct {
    drp_func_t      p_func;
    const char *    lib_name;
    const uint8_t * lib_bin;
    uint32_t        lib_bin_size;
} drp_lib_func;

static drp_lib_ctl_t drp_lib[DRP_LIB_MAX];

static DisplayBase Display;
static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_work0[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_work1[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_clat8[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(32)));
static uint8_t fbuf_overlay[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(32)));
static uint8_t drp_work_buf[FRAME_BUFFER_STRIDE * (FRAME_BUFFER_HEIGHT + (2 * 3)) * 2]__attribute((section("NC_BSS")));
static uint8_t nc_memory[512] __attribute((section("NC_BSS")));
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static Thread drpTask(osPriorityHigh, 1024 * 8);
static uint32_t mode_req = 0;
static Timer t;
static Timer event_time;
static AsciiFont ascii_font(fbuf_overlay, VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, FRAME_BUFFER_STRIDE, DATA_SIZE_PER_PIC);
static InterruptIn button(USER_BUTTON0);

#if RAM_TABLE_DYNAMIC_LOADING
static uint8_t * drp_work_memory_top;
static uint8_t drp_lib_work_memory[800 * 1024]__attribute((aligned(32)));
#endif

static const uint32_t clut_data_resut[] = {0x00000000, 0xff00ff00};  // ARGB8888

#define DRP_MODE_MAX              11

#define DRP_LIB_BAYER2GRAYSCALE    0
#define DRP_LIB_IMAGEROTATE        1
#define DRP_LIB_MEDIANBLUR         2
#define DRP_LIB_CANNYCALCULATE     3
#define DRP_LIB_CANNYHYSTERISIS    4
#define DRP_LIB_BINARIZATION       5
#define DRP_LIB_ERODE              6
#define DRP_LIB_DILATE             7
#define DRP_LIB_GAUSSIANBLUR       8
#define DRP_LIB_SOBEL              9
#define DRP_LIB_PREWITT           10
#define DRP_LIB_LAPLACIAN         11
#define DRP_LIB_UNSHARPMASKING    12
#define DRP_LIB_CROPPING          13
#define DRP_LIB_RESIZEBILINEARF   14
#define DRP_LIB_HISTOGRAM         15

static void drp_sample_Bayer2Grayscale(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_ImageRotate(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_MedianBlur(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_CannyCalculate(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_CannyHysterisis(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Binarization(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Erode(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Dilate(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_GaussianBlur(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Sobel(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Prewitt(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Laplacian(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_UnsharpMasking(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Cropping(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_ResizeBilinearF(drp_lib_ctl_t * drp_lib_ctl);
static void drp_sample_Histogram(drp_lib_ctl_t * drp_lib_ctl);

static const drp_lib_func drp_lib_func_tbl[] = {
//   p_func                       lib_name            lib_bin                           lib_bin_size
    {&drp_sample_Bayer2Grayscale, "Bayer2Grayscale",  g_drp_lib_bayer2grayscale,        sizeof(g_drp_lib_bayer2grayscale)        }, // DRP_LIB_BAYER2GRAYSCALE
    {&drp_sample_ImageRotate,     "ImageRotate    ",  g_drp_lib_image_rotate,           sizeof(g_drp_lib_image_rotate)           }, // DRP_LIB_IMAGEROTATE
    {&drp_sample_MedianBlur,      "MedianBlur     ",  g_drp_lib_median_blur,            sizeof(g_drp_lib_median_blur)            }, // DRP_LIB_MEDIANBLUR
    {&drp_sample_CannyCalculate,  "CannyCalculate ",  g_drp_lib_canny_calculate,        sizeof(g_drp_lib_canny_calculate)        }, // DRP_LIB_CANNYCALCULATE
    {&drp_sample_CannyHysterisis, "CannyHysterisis",  g_drp_lib_canny_hysterisis,       sizeof(g_drp_lib_canny_hysterisis)       }, // DRP_LIB_CANNYHYSTERISIS
    {&drp_sample_Binarization,    "Binarization   ",  g_drp_lib_binarization_fixed,     sizeof(g_drp_lib_binarization_fixed)     }, // DRP_LIB_BINARIZATION
    {&drp_sample_Erode,           "Erode          ",  g_drp_lib_erode,                  sizeof(g_drp_lib_erode)                  }, // DRP_LIB_ERODE
    {&drp_sample_Dilate,          "Dilate         ",  g_drp_lib_dilate,                 sizeof(g_drp_lib_dilate)                 }, // DRP_LIB_DILATE
    {&drp_sample_GaussianBlur,    "GaussianBlur   ",  g_drp_lib_gaussian_blur,          sizeof(g_drp_lib_gaussian_blur)          }, // DRP_LIB_GAUSSIANBLUR
    {&drp_sample_Sobel,           "Sobel          ",  g_drp_lib_sobel,                  sizeof(g_drp_lib_sobel)                  }, // DRP_LIB_SOBEL
    {&drp_sample_Prewitt,         "Prewitt        ",  g_drp_lib_prewitt,                sizeof(g_drp_lib_prewitt)                }, // DRP_LIB_PREWITT
    {&drp_sample_Laplacian,       "Laplacian      ",  g_drp_lib_laplacian,              sizeof(g_drp_lib_laplacian)              }, // DRP_LIB_LAPLACIAN
    {&drp_sample_UnsharpMasking,  "UnsharpMasking ",  g_drp_lib_unsharp_masking,        sizeof(g_drp_lib_unsharp_masking)        }, // DRP_LIB_UNSHARPMASKING
    {&drp_sample_Cropping,        "Cropping       ",  g_drp_lib_cropping,               sizeof(g_drp_lib_cropping)               }, // DRP_LIB_CROPPING
    {&drp_sample_ResizeBilinearF, "ResizeBilinearF",  g_drp_lib_resize_bilinear_fixed,  sizeof(g_drp_lib_resize_bilinear_fixed)  }, // DRP_LIB_RESIZEBILINEARF
    {&drp_sample_Histogram,       "Histogram      ",  g_drp_lib_histogram_normalization,sizeof(g_drp_lib_histogram_normalization)}, // DRP_LIB_HISTOGRAM
};

//
// Start camera
//
static void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

//
// Start LCD
//
static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;
    DisplayBase::clut_t clut_param;

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_clat8,
        FRAME_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_CLUT8,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    memset(fbuf_overlay, 0, sizeof(fbuf_overlay));
    clut_param.color_num = sizeof(clut_data_resut) / sizeof(uint32_t);
    clut_param.clut = clut_data_resut;

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_2,
        (void *)fbuf_overlay,
        FRAME_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_CLUT8,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect,
        &clut_param
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_2);

    ThisThread::sleep_for(50);
    EasyAttach_LcdBacklight(true);
}

//
// Callback functions
//
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}

static void cb_drp_finish(uint8_t id) {
    uint32_t tile_no;
    uint32_t set_flgs = 0;

    // Change the operation state of the DRP library notified by the argument to finish
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++) {
        if (drp_lib_id[tile_no] == id) {
            set_flgs |= (1 << tile_no);
        }
    }
    drpTask.flags_set(set_flgs);
}

//
// DRP sample functions 
// See "mbed-gr-libs\drp-for-mbed\TARGET_RZ_A2XX\r_drp\doc" for details
//
static void drp_sample_Bayer2Grayscale(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Bayer2Grayscale  | */
    /*        +------------------+ */
    /* tile 1 | Bayer2Grayscale  | */
    /*        +------------------+ */
    /* tile 2 | Bayer2Grayscale  | */
    /*        +------------------+ */
    /* tile 3 | Bayer2Grayscale  | */
    /*        +------------------+ */
    /* tile 4 | Bayer2Grayscale  | */
    /*        +------------------+ */
    /* tile 5 | Bayer2Grayscale  | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_bayer2grayscale_t * param_b2g = (r_drp_bayer2grayscale_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_b2g[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_b2g[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_b2g[idx].width  = VIDEO_PIXEL_HW;
        param_b2g[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_b2g[idx].top    = (idx == 0) ? 1 : 0;
        param_b2g[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_b2g[idx], sizeof(r_drp_bayer2grayscale_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_ImageRotate(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | ImageRotate      | */
    /*        +------------------+ */
    /* tile 1 | ImageRotate      | */
    /*        +------------------+ */
    /* tile 2 | ImageRotate      | */
    /*        +------------------+ */
    /* tile 3 | ImageRotate      | */
    /*        +------------------+ */
    /* tile 4 | ImageRotate      | */
    /*        +------------------+ */
    /* tile 5 | ImageRotate      | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_image_rotate_t * param_rotate = (r_drp_image_rotate_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_rotate[idx].src        = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_rotate[idx].dst        = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * (R_DK2_TILE_NUM - 1 - idx));
        param_rotate[idx].src_width  = VIDEO_PIXEL_HW;
        param_rotate[idx].src_height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_rotate[idx].dst_stride = FRAME_BUFFER_STRIDE;
        param_rotate[idx].mode       = 2; // Rotate 180‹ clockwise
        R_DK2_Start(drp_lib_id[idx], (void *)&param_rotate[idx], sizeof(r_drp_image_rotate_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_MedianBlur(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | MedianBlur       | */
    /*        +------------------+ */
    /* tile 1 | MedianBlur       | */
    /*        +------------------+ */
    /* tile 2 | MedianBlur       | */
    /*        +------------------+ */
    /* tile 3 | MedianBlur       | */
    /*        +------------------+ */
    /* tile 4 | MedianBlur       | */
    /*        +------------------+ */
    /* tile 5 | MedianBlur       | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_median_blur_t * param_median = (r_drp_median_blur_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_median[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_median[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_median[idx].width  = VIDEO_PIXEL_HW;
        param_median[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_median[idx].top    = (idx == 0) ? 1 : 0;
        param_median[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_median[idx], sizeof(r_drp_median_blur_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_CannyCalculate(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 |                  | */
    /*        + CannyCalculate   + */
    /* tile 1 |                  | */
    /*        +------------------+ */
    /* tile 2 |                  | */
    /*        + CannyCalculate   + */
    /* tile 3 |                  | */
    /*        +------------------+ */
    /* tile 4 |                  | */
    /*        + CannyCalculate   + */
    /* tile 5 |                  | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_2 | R_DK2_TILE_4,
        R_DK2_TILE_PATTERN_2_2_2, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_canny_calculate_t * param_canny_cal = (r_drp_canny_calculate_t *)nc_memory;
    for (uint32_t idx = 0; idx < 3; idx++) {
        param_canny_cal[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / 3) * idx);
        param_canny_cal[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / 3) * idx);
        param_canny_cal[idx].width  = VIDEO_PIXEL_HW;
        param_canny_cal[idx].height = (VIDEO_PIXEL_VW / 3);
        param_canny_cal[idx].top    = ((idx * 2) == 0) ? 1 : 0;
        param_canny_cal[idx].bottom = ((idx * 2) == 4) ? 1 : 0;
        param_canny_cal[idx].work   = (uint32_t)&drp_work_buf[((VIDEO_PIXEL_HW * ((VIDEO_PIXEL_VW / 3) + 2)) * 2) * idx];
        param_canny_cal[idx].threshold_high = 0x28;
        param_canny_cal[idx].threshold_low  = 0x18;
        R_DK2_Start(drp_lib_id[(idx * 2)], (void *)&param_canny_cal[idx], sizeof(r_drp_canny_calculate_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_CannyHysterisis(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 |                  | */
    /*        +                  + */
    /* tile 1 |                  | */
    /*        +                  + */
    /* tile 2 |                  | */
    /*        + CannyHysterisis  + */
    /* tile 3 |                  | */
    /*        +                  + */
    /* tile 4 |                  | */
    /*        +                  + */
    /* tile 5 |                  | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0,
        R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_canny_hysterisis_t * param_canny_hyst = (r_drp_canny_hysterisis_t *)nc_memory;
    param_canny_hyst[0].src    = (uint32_t)drp_lib_ctl->src;
    param_canny_hyst[0].dst    = (uint32_t)drp_lib_ctl->dst;
    param_canny_hyst[0].width  = VIDEO_PIXEL_HW;
    param_canny_hyst[0].height = VIDEO_PIXEL_VW;
    param_canny_hyst[0].work   = (uint32_t)drp_work_buf;
    param_canny_hyst[0].iterations = 2;
    R_DK2_Start(drp_lib_id[0], (void *)&param_canny_hyst[0], sizeof(r_drp_canny_hysterisis_t));
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Binarization(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Binarization     | */
    /*        +------------------+ */
    /* tile 1 | Binarization     | */
    /*        +------------------+ */
    /* tile 2 | Binarization     | */
    /*        +------------------+ */
    /* tile 3 | Binarization     | */
    /*        +------------------+ */
    /* tile 4 | Binarization     | */
    /*        +------------------+ */
    /* tile 5 | Binarization     | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_binarization_fixed_t * param_binfix = (r_drp_binarization_fixed_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_binfix[idx].src       = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_binfix[idx].dst       = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_binfix[idx].width     = VIDEO_PIXEL_HW;
        param_binfix[idx].height    = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_binfix[idx].threshold = 100;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_binfix[idx], sizeof(r_drp_binarization_fixed_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Erode(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Erode            | */
    /*        +------------------+ */
    /* tile 1 | Erode            | */
    /*        +------------------+ */
    /* tile 2 | Erode            | */
    /*        +------------------+ */
    /* tile 3 | Erode            | */
    /*        +------------------+ */
    /* tile 4 | Erode            | */
    /*        +------------------+ */
    /* tile 5 | Erode            | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_erode_t * param_erode = (r_drp_erode_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_erode[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_erode[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_erode[idx].width  = VIDEO_PIXEL_HW;
        param_erode[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_erode[idx].top    = (idx == 0) ? 1 : 0;
        param_erode[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_erode[idx], sizeof(r_drp_erode_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Dilate(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Dilate           | */
    /*        +------------------+ */
    /* tile 1 | Dilate           | */
    /*        +------------------+ */
    /* tile 2 | Dilate           | */
    /*        +------------------+ */
    /* tile 3 | Dilate           | */
    /*        +------------------+ */
    /* tile 4 | Dilate           | */
    /*        +------------------+ */
    /* tile 5 | Dilate           | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_dilate_t * param_dilate = (r_drp_dilate_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_dilate[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_dilate[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_dilate[idx].width  = VIDEO_PIXEL_HW;
        param_dilate[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_dilate[idx].top    = (idx == 0) ? 1 : 0;
        param_dilate[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_dilate[idx], sizeof(r_drp_dilate_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_GaussianBlur(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Gaussian Blur    | */
    /*        +------------------+ */
    /* tile 1 | Gaussian Blur    | */
    /*        +------------------+ */
    /* tile 2 | Gaussian Blur    | */
    /*        +------------------+ */
    /* tile 3 | Gaussian Blur    | */
    /*        +------------------+ */
    /* tile 4 | Gaussian Blur    | */
    /*        +------------------+ */
    /* tile 5 | Gaussian Blur    | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_gaussian_blur_t * param_gauss = (r_drp_gaussian_blur_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_gauss[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_gauss[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_gauss[idx].width  = VIDEO_PIXEL_HW;
        param_gauss[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_gauss[idx].top    = (idx == 0) ? 1 : 0;
        param_gauss[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_gauss[idx], sizeof(r_drp_gaussian_blur_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Sobel(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Sobel            | */
    /*        +------------------+ */
    /* tile 1 | Sobel            | */
    /*        +------------------+ */
    /* tile 2 | Sobel            | */
    /*        +------------------+ */
    /* tile 3 | Sobel            | */
    /*        +------------------+ */
    /* tile 4 | Sobel            | */
    /*        +------------------+ */
    /* tile 5 | Sobel            | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_sobel_t * param_sobel = (r_drp_sobel_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_sobel[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_sobel[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_sobel[idx].width  = VIDEO_PIXEL_HW;
        param_sobel[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_sobel[idx].top    = (idx == 0) ? 1 : 0;
        param_sobel[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_sobel[idx], sizeof(r_drp_sobel_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Prewitt(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Prewitt          | */
    /*        +------------------+ */
    /* tile 1 | Prewitt          | */
    /*        +------------------+ */
    /* tile 2 | Prewitt          | */
    /*        +------------------+ */
    /* tile 3 | Prewitt          | */
    /*        +------------------+ */
    /* tile 4 | Prewitt          | */
    /*        +------------------+ */
    /* tile 5 | Prewitt          | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_prewitt_t * param_prewitt = (r_drp_prewitt_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_prewitt[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_prewitt[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_prewitt[idx].width  = VIDEO_PIXEL_HW;
        param_prewitt[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_prewitt[idx].top    = (idx == 0) ? 1 : 0;
        param_prewitt[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_prewitt[idx], sizeof(r_drp_prewitt_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Laplacian(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Laplacian        | */
    /*        +------------------+ */
    /* tile 1 | Laplacian        | */
    /*        +------------------+ */
    /* tile 2 | Laplacian        | */
    /*        +------------------+ */
    /* tile 3 | Laplacian        | */
    /*        +------------------+ */
    /* tile 4 | Laplacian        | */
    /*        +------------------+ */
    /* tile 5 | Laplacian        | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_laplacian_t * param_laplacian = (r_drp_laplacian_t *)nc_memory;
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_laplacian[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_laplacian[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_laplacian[idx].width  = VIDEO_PIXEL_HW;
        param_laplacian[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_laplacian[idx].top    = (idx == 0) ? 1 : 0;
        param_laplacian[idx].bottom = (idx == 5) ? 1 : 0;
        R_DK2_Start(drp_lib_id[idx], (void *)&param_laplacian[idx], sizeof(r_drp_laplacian_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_UnsharpMasking(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 |                  | */
    /*        + UnsharpMasking   + */
    /* tile 1 |                  | */
    /*        +------------------+ */
    /* tile 2 |                  | */
    /*        + UnsharpMasking   + */
    /* tile 3 |                  | */
    /*        +------------------+ */
    /* tile 4 |                  | */
    /*        + UnsharpMasking   + */
    /* tile 5 |                  | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_2 | R_DK2_TILE_4,
        R_DK2_TILE_PATTERN_2_2_2, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_unsharp_masking_t * param_unsharp = (r_drp_unsharp_masking_t *)nc_memory;
    for (uint32_t idx = 0; idx < 3; idx++) {
        param_unsharp[idx].src      = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / 3) * idx);
        param_unsharp[idx].dst      = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / 3) * idx);
        param_unsharp[idx].width    = VIDEO_PIXEL_HW;
        param_unsharp[idx].height   = (VIDEO_PIXEL_VW / 3);
        param_unsharp[idx].strength = 255;
        param_unsharp[idx].top      = ((idx * 2) == 0) ? 1 : 0;
        param_unsharp[idx].bottom   = ((idx * 2) == 4) ? 1 : 0;
        R_DK2_Start(drp_lib_id[(idx * 2)], (void *)&param_unsharp[idx], sizeof(r_drp_unsharp_masking_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Cropping(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Cropping         | */
    /*        +------------------+ */
    /* tile 1 |                  | */
    /*        +                  + */
    /* tile 2 |                  | */
    /*        +                  + */
    /* tile 3 | Not used         | */
    /*        +                  + */
    /* tile 4 |                  | */
    /*        +                  + */
    /* tile 5 |                  | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_cropping_t * param_cropping = (r_drp_cropping_t *)nc_memory;
    param_cropping[0].src        = (uint32_t)drp_lib_ctl->src;
    param_cropping[0].dst        = (uint32_t)drp_lib_ctl->dst;
    param_cropping[0].src_width  = VIDEO_PIXEL_HW;
    param_cropping[0].src_height = VIDEO_PIXEL_VW;
    param_cropping[0].offset_x   = VIDEO_PIXEL_HW / 4;
    param_cropping[0].offset_y   = VIDEO_PIXEL_VW / 4;
    param_cropping[0].dst_width  = VIDEO_PIXEL_HW / 2;
    param_cropping[0].dst_height = VIDEO_PIXEL_VW / 2;
    R_DK2_Start(drp_lib_id[0], (void *)&param_cropping[0], sizeof(r_drp_cropping_t));
    ThisThread::flags_wait_all(R_DK2_TILE_0);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_ResizeBilinearF(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 |                  | */
    /*        +                  + */
    /* tile 1 |                  | */
    /*        + ResizeBilinearF  + */
    /* tile 2 |                  | */
    /*        +                  + */
    /* tile 3 |                  | */
    /*        +------------------+ */
    /* tile 4 |                  | */
    /*        + Not used         + */
    /* tile 5 |                  | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0,
        R_DK2_TILE_PATTERN_4_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_resize_bilinear_fixed_t * param_resize = (r_drp_resize_bilinear_fixed_t *)nc_memory;
    param_resize[0].src        = (uint32_t)drp_lib_ctl->src;
    param_resize[0].dst        = (uint32_t)drp_lib_ctl->dst;
    param_resize[0].src_width  = VIDEO_PIXEL_HW / 2;
    param_resize[0].src_height = VIDEO_PIXEL_VW / 2;
    param_resize[0].fx         = 0x08;  // 2x
    param_resize[0].fy         = 0x08;  // 2x
    R_DK2_Start(drp_lib_id[0], (void *)&param_resize[0], sizeof(r_drp_resize_bilinear_fixed_t));
    ThisThread::flags_wait_all(R_DK2_TILE_0);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

static void drp_sample_Histogram(drp_lib_ctl_t * drp_lib_ctl) {
    /* Load DRP Library            */
    /*        +------------------+ */
    /* tile 0 | Histogram        | */
    /*        +------------------+ */
    /* tile 1 | Histogram        | */
    /*        +------------------+ */
    /* tile 2 | Histogram        | */
    /*        +------------------+ */
    /* tile 3 | Histogram        | */
    /*        +------------------+ */
    /* tile 4 | Histogram        | */
    /*        +------------------+ */
    /* tile 5 | Histogram        | */
    /*        +------------------+ */
    t.reset();
    R_DK2_Load(
        drp_lib_ctl->p_drp_lib_bin,
        R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5,
        R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
    R_DK2_Activate(0, 0);
    drp_lib_ctl->load_time = t.read_us();

    t.reset();
    r_drp_histogram_normalization_t * param_histo = (r_drp_histogram_normalization_t *)nc_memory;
    r_drp_histogram_normalization_output_mode1_t * param_histogram_normalization1;
    param_histogram_normalization1 = (r_drp_histogram_normalization_output_mode1_t *)((uint32_t)nc_memory + sizeof(r_drp_histogram_normalization_t));

    // MODE1: Survey the overall brightness of the image
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_histo[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_histo[idx].dst    = (uint32_t)&param_histogram_normalization1[idx];
        param_histo[idx].width  = VIDEO_PIXEL_HW;
        param_histo[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_histo[idx].src_pixel_mean = 0;
        param_histo[idx].src_pixel_rstd = 0;
        param_histo[idx].dst_pixel_mean = 0;
        param_histo[idx].dst_pixel_std  = 0;
        param_histo[idx].mode           = 1;  // MODE1
        R_DK2_Start(drp_lib_id[idx], (void *)&param_histo[idx], sizeof(r_drp_histogram_normalization_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);

    volatile double sum = 0;
    volatile double square_sum = 0;

    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        sum += param_histogram_normalization1[idx].sum;
        square_sum += param_histogram_normalization1[idx].square_sum;
    }

    volatile double mean = sum / (VIDEO_PIXEL_HW * VIDEO_PIXEL_VW);
    volatile double std = sqrt(square_sum / (VIDEO_PIXEL_HW * VIDEO_PIXEL_VW) - (mean * mean));
    volatile uint32_t src_pixel_mean = mean * 4096;
    volatile uint32_t src_pixel_rstd = 4096 / std;

    // MODE2: Normalize the image
    for (uint32_t idx = 0; idx < R_DK2_TILE_NUM; idx++) {
        param_histo[idx].src    = (uint32_t)drp_lib_ctl->src + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_histo[idx].dst    = (uint32_t)drp_lib_ctl->dst + (VIDEO_PIXEL_HW * (VIDEO_PIXEL_VW / R_DK2_TILE_NUM) * idx);
        param_histo[idx].width  = VIDEO_PIXEL_HW;
        param_histo[idx].height = VIDEO_PIXEL_VW / R_DK2_TILE_NUM;
        param_histo[idx].src_pixel_mean = src_pixel_mean;
        param_histo[idx].src_pixel_rstd = src_pixel_rstd;
        param_histo[idx].dst_pixel_mean = 112;
        param_histo[idx].dst_pixel_std  = 48;
        param_histo[idx].mode           = 2;  // MODE2
        R_DK2_Start(drp_lib_id[idx], (void *)&param_histo[idx], sizeof(r_drp_histogram_normalization_t));
    }
    ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
    R_DK2_Unload(0, drp_lib_id);
    drp_lib_ctl->run_time = t.read_us();
}

//
// Register DRP function
//
static void init_drp_work_memory(void) {
#if RAM_TABLE_DYNAMIC_LOADING
    drp_work_memory_top = drp_lib_work_memory;
#endif
}

static uint8_t * get_configuration_data(const uint8_t * lib_bin, uint32_t lib_bin_size) {
#if RAM_TABLE_DYNAMIC_LOADING
    uint8_t * ret_addr = drp_work_memory_top;

    drp_work_memory_top = (uint8_t *)(((uint32_t)drp_work_memory_top + lib_bin_size + 32ul) & ~31ul);
    if ((uint32_t)drp_work_memory_top > ((uint32_t)drp_lib_work_memory + sizeof(drp_lib_work_memory))) {
        printf("drp_lib_work_memory size error\r\n");
        while (1);
    }
    memcpy(ret_addr, lib_bin,  lib_bin_size);
    dcache_clean(ret_addr, lib_bin_size);

    return ret_addr;
#else
    (void)lib_bin_size;

    return (uint8_t *)lib_bin;
#endif
}

static void set_drp_func(drp_lib_ctl_t * p_drp_lib, uint32_t drp_lib_no, uint8_t * src, uint8_t * dst) {
    const drp_lib_func * p_drp_lib_func = &drp_lib_func_tbl[drp_lib_no];

    p_drp_lib->drp_lib_no = drp_lib_no;
    p_drp_lib->p_drp_lib_bin = get_configuration_data(p_drp_lib_func->lib_bin, p_drp_lib_func->lib_bin_size);
    p_drp_lib->src = src;
    p_drp_lib->dst = dst;
}

static uint32_t init_drp_lib(uint32_t mode) {
    uint32_t idx = 0;

    init_drp_work_memory();
    memset(fbuf_overlay, 0, sizeof(fbuf_overlay));

    switch (mode) {
        case 0:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_clat8);  // ImageRotate
            break;
        case 1:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_BINARIZATION,    fbuf_work1, fbuf_clat8);  // Binarization
            break;
        case 2:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_MEDIANBLUR,      fbuf_work1, fbuf_work0);  // MedianBlur
            set_drp_func(&drp_lib[idx++], DRP_LIB_CANNYCALCULATE,  fbuf_work0, fbuf_work1);  // CannyCalculate
            set_drp_func(&drp_lib[idx++], DRP_LIB_CANNYHYSTERISIS, fbuf_work1, fbuf_clat8);  // CannyHysterisis
            break;
        case 3:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_ERODE,           fbuf_work1, fbuf_clat8);  // Erode
            break;
        case 4:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_DILATE,          fbuf_work1, fbuf_clat8);  // Dilate
            break;
        case 5:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_GAUSSIANBLUR,    fbuf_work1, fbuf_clat8);  // GaussianBlur
            break;
        case 6:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_SOBEL,           fbuf_work1, fbuf_clat8);  // Sobel
            break;
        case 7:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_PREWITT,         fbuf_work1, fbuf_clat8);  // Prewitt
            break;
        case 8:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_LAPLACIAN,       fbuf_work1, fbuf_clat8);  // Laplacian
            break;
        case 9:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_UNSHARPMASKING,  fbuf_work1, fbuf_clat8);  // UnsharpMasking
            break;
        case 10:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_CROPPING,        fbuf_work1, fbuf_work0);  // Cropping
            set_drp_func(&drp_lib[idx++], DRP_LIB_RESIZEBILINEARF, fbuf_work0, fbuf_clat8);  // ResizeBilinearF
            break;
        case 11:
            set_drp_func(&drp_lib[idx++], DRP_LIB_BAYER2GRAYSCALE, fbuf_bayer, fbuf_work0);  // Bayer2Grayscale
            set_drp_func(&drp_lib[idx++], DRP_LIB_IMAGEROTATE,     fbuf_work0, fbuf_work1);  // ImageRotate
            set_drp_func(&drp_lib[idx++], DRP_LIB_HISTOGRAM,       fbuf_work1, fbuf_clat8);  // Histogram
            break;
        default:
            // do nothing
            break;
    }

    return idx;
}

//
// Drawing of DRP processing time
//
static void draw_str(const char * str, uint32_t line) {
    ascii_font.DrawStr(str, 5, 5 + (AsciiFont::CHAR_PIX_HEIGHT + 1) * 2 * line, 1, 2);
}

static void draw_processing_time(uint32_t drp_lib_num) {
    char str[64];
    uint32_t i;
    uint32_t time_sum = 0;
    drp_lib_ctl_t * p_drp_lib = &drp_lib[0];

    for (i = 0; i < drp_lib_num; i++) {
        sprintf(str, "%s : Load %2.1fms + Run %2.1fms", drp_lib_func_tbl[p_drp_lib->drp_lib_no].lib_name,
                (float_t)p_drp_lib->load_time / 1000, (float_t)p_drp_lib->run_time / 1000);
        draw_str(str, i);
        time_sum += p_drp_lib->load_time;
        time_sum += p_drp_lib->run_time;
        p_drp_lib++;
    }
    sprintf(str, "Total           : %2.1fms", (float_t)time_sum / 1000);
    draw_str(str, i);
}

//
// Button operation
//
static void button_fall(void) {
    if (mode_req < DRP_MODE_MAX) {
        mode_req++;
    } else {
        mode_req = 0;
    }
    event_time.reset();
}

//
// DRP task processing
//
static void drp_task(void) {
    uint32_t mode = 0xffffffff;
    uint32_t drp_lib_num = 0;

    button.fall(&button_fall);

    EasyAttach_Init(Display);
    Start_LCD_Display();
    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, IntCallbackFunc_Vfield);
    Start_Video_Camera();

    R_DK2_Initialize();

    t.start();
    event_time.start();

    while (true) {
        // Check event timer
        if (event_time.read_ms() >= 10000) {
            button_fall();
        }

        // Check mode change
        if (mode_req != mode) {
            mode = mode_req;
            drp_lib_num = init_drp_lib(mode);
        }

        // Waiting for camera image
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);

        // DRP execution
        drp_lib_ctl_t * p_drp_lib = &drp_lib[0];
        for (uint32_t i = 0; i < drp_lib_num; i++) {
            drp_lib_func_tbl[p_drp_lib->drp_lib_no].p_func(p_drp_lib);
            p_drp_lib++;
        }

        // Draw processing time
        draw_processing_time(drp_lib_num);
    }
}

int main(void) {
    // Start DRP task
    drpTask.start(callback(drp_task));

    wait(osWaitForever);
}
