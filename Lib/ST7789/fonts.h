#ifndef __FONT_H
#define __FONT_H

#include <stdint.h>

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

//Font lib.
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

//16-bit(RGB565) Image lib.
/*******************************************
 *             CAUTION:
 *   If the MCU onchip flash cannot
 *  store such huge image data,please
 *           do not use it.
 * These pics are for test purpose only.
 *******************************************/

/* 128x128 pixel RGB565 image */
//extern const uint16_t saber[][128];
extern const uint16_t IMG_128x128_instagram2[];
extern const unsigned short IMG_37x37_BT_CN[];
extern const unsigned short IMG_37x37_BT_NC[];
extern const unsigned short IMG_37x37_SD_MT[];
extern const unsigned short IMG_37x37_SD_UM[];
extern const unsigned short IMG_24x37_BAT_CG[];
extern const unsigned short IMG_24x37_BAT_CAN[];
extern const unsigned short IMG_24x30_POSISION[];
/* 240x240 pixel RGB565 image 
extern const uint16_t knky[][240];
extern const uint16_t tek[][240];
extern const uint16_t adi1[][240];
*/
#endif
