/**
  ******************************************************************************
  * @file    BitmapFile.h
  * @author  SOLiD Software 1 Team
  * @brief   <USER UPDATE>
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy;Copyright (c) 2017, SOLiD, Inc.  All rights reserved.</center></h2>
  *
  * This Software is the property of SOLiD. The Software and all
  * accompanying documentation are copyrighted.  The Software made available
  * here constitutes the proprietary information of SOLiD.  You
  * agree to take reasonable steps to prevent the disclosure, unauthorized use
  * or unauthorized distribution of the Software.
  *
  * Except as expressly permitted in a separate Software License Agreement
  * between You and SOLiD, you shall not modify, decompile,
  * disassemble, extract, or otherwise reverse engineer this Software.  You
  * shall not make any copy of the Software or its accompanying documentation,
  * except for copying incident to the ordinary and intended use of the
  * Software and the Underlying Program and except for the making of a single
  * archival copy.
  *
  * This Software, including technical data, may be subject to U.S. export
  * control laws, including the U.S.  Export Administration Act and its
  * associated regulations, and may be subject to export or import regulations
  * in other countries.  You warrant that You will comply strictly in all
  * respects with all such regulations and acknowledge that you have the
  * responsibility to obtain licenses to export, re-export or import the
  * Software.
  *
  * TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
  * AND WITH ALL FAULTS AND SOLiD MAKES NO PROMISES, REPRESENTATIONS OR
  * WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH RESPECT
  * TO THE SOFTWARE, INCLUDING ITS CONDITION, ITS CONFORMITY TO ANY
  * REPRESENTATION OR DESCRIPTION, OR THE EXISTENCE OF ANY LATENT OR PATENT
  * DEFECTS, AND SOLID SPECIFICALLY DISCLAIMS ALL IMPLIED (IF ANY) WARRANTIES
  * OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR A PARTICULAR
  * PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET
  * POSSESSION OR CORRESPONDENCE TO DESCRIPTION.  THE ENTIRE RISK ARISING OUT
  * OF USE OR PERFORMANCE OF THE SOFTWARE LIES WITH YOU.
  *
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef   _BITMAPFILE_H_
#define   _BITMAPFILE_H_

/* Includes */
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>


#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BITMAPFILE BitmapFile
  * @{
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup BITMAPFILE_Private_Macros BitmapFile Private Macros
  * @{
  */

#define BF_TYPE 0x4D42             /* "MB" */
#define BI_RGB       0             /* No compression - straight BGR data */
#define BI_RLE8      1             /* 8-bit run-length compression */
#define BI_RLE4      2             /* 4-bit run-length compression */
#define BI_BITFIELDS 3             /* RGB bitmap with RGB masks */

/* End of BITMAPFILE_Private_Macros */
/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup BITMAPFILE_Exported_Macros BitmapFile Exported Macros
  * @{
  */
/* End of BITMAPFILE_Exported_Macros */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @defgroup BITMAPFILE_Private_Types BitmapFile Private Types
  * @{
  */
/* End of BITMAPFILE_Private_Types */
/**
  * @}
  */

/* Exported types -------------------------------------------------------------*/
/** @defgroup BITMAPFILE_Exported_Types BitmapFile Exported Types
  * @{
  */

/*
 * Bitmap file data structures (these are defined in <wingdi.h> under
 * Windows...)
 *
 * Note that most Windows compilers will pack the following structures, so
 * when reading them under MacOS or UNIX we need to read individual fields
 * to avoid differences in alignment...
 */

#pragma pack(push, 1)

struct BitmapfileHeader           /**** BMP file header structure ****/
{
    uint16_t    bfType;           /* Magic number for file */
    uint32_t    bfSize;           /* Size of file */
    uint16_t    bfReserved1;      /* Reserved */
    uint16_t    bfReserved2;      /* ... */
    uint32_t    bfOffBytes;       /* Offset to bitmap data */
};

struct BitmapInfoHeader            /**** BMP file info structure ****/
{
    uint32_t    biSize;           /* Size of info header */
    int32_t     biWidth;          /* Width of image */
    int32_t     biHeight;         /* Height of image */
    uint16_t    biPlanes;         /* Number of color planes */
    uint16_t    biBitCount;       /* Number of bits per pixel */
    uint32_t    biCompression;    /* Type of compression to use */
    uint32_t    biSizeImage;      /* Size of image data */
    int32_t     biXPelsPerMeter;  /* X pixels per meter */
    int32_t     biYPelsPerMeter;  /* Y pixels per meter */
    uint32_t    biClrUsed;        /* Number of colors used */
    uint32_t    biClrImportant;   /* Number of important colors */
};

struct PixelColor {
    uint8 b;
    uint8 g;
    uint8 r;
};

struct LTEChannel2Color {
    uint8 lte_ch;
    struct PixelColor color;
};

#pragma pack(pop)


/* End of BITMAPFILE_Exported_Types */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup BITMAPFILE_Private_Variables BitmapFile Private Variables
  * @{
  */
/* End of BITMAPFILE_Private_Variables */
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
/** @defgroup BITMAPFILE_Exported_Variables BitmapFile Exported Variables
  * @{
  */


/* End of BITMAPFILE_Exported_Variables */
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup BITMAPFILE_Private_Functions BitmapFile Private Functions
  * @{
  */
/* End of BITMAPFILE_Private_Functions */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup BITMAPFILE_Exported_Functions BitmapFile Exported Functions
  * @{
  */


/* End of BITMAPFILE_Exported_Functions */
/**
  * @}
  */

/* End of BITMAPFILE */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _BITMAPFILE_H_ */

