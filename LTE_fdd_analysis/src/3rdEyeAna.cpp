// 3rdEyeAna.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

typedef unsigned long  uint32;
typedef unsigned char  uint08;
typedef unsigned short uint16;
typedef long           int32;

#define INPUT_LENGTH          512
#define OUTPUT_LENGTH          64
#define MAX_DST_BIT_POS       128
#define DIVIDE_LINE             5
#define MD5_IMG_SHOW_LINE     128
#define MD5_HASH_BIT_NUMBER   128
#define MD5_HASH_BYTE_NUMBER   16
#define START_BLOCK             5
#define _3rdEye_HEADER_BYTES    9

#define MD5_HASH_BLOCK_SIZE     (0x20000)

#define IMG_BLOCK_SCALE     10
#define IMG_BLOCK_BLANK     1
#define IMG_BLOCK_BLANK2    2
#define HASH_BIT_0_0        254
#define HASH_BIT_0_1        222

#define HASH_BIT_0_BASE     144
#define HASH_BIT_0_INC      16
#define HASH_BIT_0_TH       3
#define HASH_BIT_1_TH       1

#define HASH_BIT_0_VALUE    0x0E
#define HASH_BIT_1_VALUE_1  0x0F
#define HASH_BIT_1_VALUE_2  0x0B
#define HASH_BIT_1_VALUE_3  0x07


#define HASH_BIT_UNDEF      0
#define HASH_BIT_0          1
#define HASH_BIT_1          2
#define HASH_BIT_NOT_0      3
#define HASH_BIT_NOT_1      4

#define IMG_BLACK_OFFSET    0x00
#define IMG_BLUE_OFFSET     0x10
#define IMG_GREEN_OFFSET    0x20
#define IMG_RED_OFFSET      0x30
#define IMG_YELLOW_OFFSET   0x40

#define HASH_BIT_0_INDEX    (IMG_YELLOW_OFFSET)
#define HASH_BIT_1_INDEX    (IMG_GREEN_OFFSET)

#define INPUT_DATA_LENGTH   128

#define OUTPUT_IMG_WIDTH    128
#define OUTPUT_IMG_SKIP     5

#define OUTPUT_IMG_WIDTH2   128

#define ENCRYPT_IMG_WIDTH   4096
#define HASH_BLOCK_W        128

#define _3rdEye_MODIFY_BIT  0x10



/* F, G, H and I are basic MD5 functions */
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))

/* ROTATE_LEFT rotates x left n bits */
#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32-(n))))

/* FF, GG, HH, and II transformations for rounds 1, 2, 3, and 4 */

/* Rotation is separate from addition to prevent recomputation */
#define FF(a, b, c, d, x, s, ac) \
  {(a) += F ((b), (c), (d)) + (x) + (uint32)(ac); \
   (a) = ROTATE_LEFT ((a), (s)); \
   (a) += (b); \
  }

#define GG(a, b, c, d, x, s, ac) \
  {(a) += G ((b), (c), (d)) + (x) + (uint32)(ac); \
   (a) = ROTATE_LEFT ((a), (s)); \
   (a) += (b); \
  }

#define HH(a, b, c, d, x, s, ac) \
  {(a) += H ((b), (c), (d)) + (x) + (uint32)(ac); \
   (a) = ROTATE_LEFT ((a), (s)); \
   (a) += (b); \
  }

#define II(a, b, c, d, x, s, ac) \
  {(a) += I ((b), (c), (d)) + (x) + (uint32)(ac); \
   (a) = ROTATE_LEFT ((a), (s)); \
   (a) += (b); \
  }

#pragma pack(push, 1)

typedef struct tagBITMAPFILEHEADER { // bmfh 
    uint16   bfType; 
    uint32   bfSize; 
    uint16   bfReserved1; 
    uint16   bfReserved2; 
    uint32   bfOffBits; 
} BITMAPFILEHEADER; 

typedef struct tagBITMAPINFOHEADER {
    uint32  biSize;
    int32   biWidth;
    int32   biHeight;
    uint16  biPlanes;
    uint16  biBitCount;
    uint32  biCompression;
    uint32  biSizeImage;
    int32   biXPelsPerMeter;
    int32   biYPelsPerMeter;
    uint32  biClrUsed;
    uint32  biClrImportant;
} BITMAPINFOHEADER;

#pragma pack(pop)

typedef struct {
    uint08  Data;
    uint08  Count;
    uint08  Mask;
} BitDataType;

static uint08 TargetData[128] = { 
                              /* 0x54 */                            0, 1, 0, 1,  0, 1, 0, 0, 
                              /* 0x54 */                            0, 1, 0, 1,  0, 1, 0, 0, 
                              /* 0x45 */                            0, 1, 0, 0,  0, 1, 0, 1, 
                              /* FileNameLength < 128 */            0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                              /* FileLength < 512Kbyts */           0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                              /* FileName (Ascii Only MSB == 0) */  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                                                                    0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0
                            };
    
static uint08 TargetMask[128] = { 
                              /* 0x54 */                            1, 1, 1, 1,  1, 1, 1, 0, 
                              /* 0x54 */                            1, 1, 1, 1,  1, 1, 1, 1, 
                              /* 0x45 */                            1, 1, 1, 1,  1, 1, 1, 1, 
                              /* FileNameLength < 128 */            1, 0, 0, 0,  0, 0, 0, 0,  1, 1, 1, 1,  1, 1, 1, 1,
                              /* FileLength < 512Kbyts */           0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  1, 1, 1, 1,  0, 0, 0, 0,  1, 1, 1, 1,  1, 1, 1, 1,
                              /* FileName (Ascii Only MSB == 0) */  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,
                                                                    1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0
                            };




void MD5Transform (uint32 *buf, uint32 *in)
{
  uint32 a = buf[0], b = buf[1], c = buf[2], d = buf[3];

  /* Round 1 */

#define S11 7
#define S12 12
#define S13 17
#define S14 22
  FF ( a, b, c, d, in[ 0], S11, 3614090360); /* 1 */
  FF ( d, a, b, c, in[ 1], S12, 3905402710); /* 2 */
  FF ( c, d, a, b, in[ 2], S13,  606105819); /* 3 */
  FF ( b, c, d, a, in[ 3], S14, 3250441966); /* 4 */
  FF ( a, b, c, d, in[ 4], S11, 4118548399); /* 5 */
  FF ( d, a, b, c, in[ 5], S12, 1200080426); /* 6 */
  FF ( c, d, a, b, in[ 6], S13, 2821735955); /* 7 */
  FF ( b, c, d, a, in[ 7], S14, 4249261313); /* 8 */
  FF ( a, b, c, d, in[ 8], S11, 1770035416); /* 9 */
  FF ( d, a, b, c, in[ 9], S12, 2336552879); /* 10 */
  FF ( c, d, a, b, in[10], S13, 4294925233); /* 11 */
  FF ( b, c, d, a, in[11], S14, 2304563134); /* 12 */
  FF ( a, b, c, d, in[12], S11, 1804603682); /* 13 */
  FF ( d, a, b, c, in[13], S12, 4254626195); /* 14 */
  FF ( c, d, a, b, in[14], S13, 2792965006); /* 15 */
  FF ( b, c, d, a, in[15], S14, 1236535329); /* 16 */

  /* Round 2 */
#define S21 5
#define S22 9
#define S23 14
#define S24 20
  GG ( a, b, c, d, in[ 1], S21, 4129170786); /* 17 */
  GG ( d, a, b, c, in[ 6], S22, 3225465664); /* 18 */
  GG ( c, d, a, b, in[11], S23,  643717713); /* 19 */
  GG ( b, c, d, a, in[ 0], S24, 3921069994); /* 20 */
  GG ( a, b, c, d, in[ 5], S21, 3593408605); /* 21 */
  GG ( d, a, b, c, in[10], S22,   38016083); /* 22 */
  GG ( c, d, a, b, in[15], S23, 3634488961); /* 23 */
  GG ( b, c, d, a, in[ 4], S24, 3889429448); /* 24 */
  GG ( a, b, c, d, in[ 9], S21,  568446438); /* 25 */
  GG ( d, a, b, c, in[14], S22, 3275163606); /* 26 */
  GG ( c, d, a, b, in[ 3], S23, 4107603335); /* 27 */
  GG ( b, c, d, a, in[ 8], S24, 1163531501); /* 28 */
  GG ( a, b, c, d, in[13], S21, 2850285829); /* 29 */
  GG ( d, a, b, c, in[ 2], S22, 4243563512); /* 30 */
  GG ( c, d, a, b, in[ 7], S23, 1735328473); /* 31 */
  GG ( b, c, d, a, in[12], S24, 2368359562); /* 32 */

  /* Round 3 */
#define S31 4
#define S32 11
#define S33 16
#define S34 23
  HH ( a, b, c, d, in[ 5], S31, 4294588738); /* 33 */
  HH ( d, a, b, c, in[ 8], S32, 2272392833); /* 34 */
  HH ( c, d, a, b, in[11], S33, 1839030562); /* 35 */
  HH ( b, c, d, a, in[14], S34, 4259657740); /* 36 */
  HH ( a, b, c, d, in[ 1], S31, 2763975236); /* 37 */
  HH ( d, a, b, c, in[ 4], S32, 1272893353); /* 38 */
  HH ( c, d, a, b, in[ 7], S33, 4139469664); /* 39 */
  HH ( b, c, d, a, in[10], S34, 3200236656); /* 40 */
  HH ( a, b, c, d, in[13], S31,  681279174); /* 41 */
  HH ( d, a, b, c, in[ 0], S32, 3936430074); /* 42 */
  HH ( c, d, a, b, in[ 3], S33, 3572445317); /* 43 */
  HH ( b, c, d, a, in[ 6], S34,   76029189); /* 44 */
  HH ( a, b, c, d, in[ 9], S31, 3654602809); /* 45 */
  HH ( d, a, b, c, in[12], S32, 3873151461); /* 46 */
  HH ( c, d, a, b, in[15], S33,  530742520); /* 47 */
  HH ( b, c, d, a, in[ 2], S34, 3299628645); /* 48 */

  /* Round 4 */
#define S41 6
#define S42 10
#define S43 15
#define S44 21
  II ( a, b, c, d, in[ 0], S41, 4096336452); /* 49 */
  II ( d, a, b, c, in[ 7], S42, 1126891415); /* 50 */
  II ( c, d, a, b, in[14], S43, 2878612391); /* 51 */
  II ( b, c, d, a, in[ 5], S44, 4237533241); /* 52 */
  II ( a, b, c, d, in[12], S41, 1700485571); /* 53 */
  II ( d, a, b, c, in[ 3], S42, 2399980690); /* 54 */
  II ( c, d, a, b, in[10], S43, 4293915773); /* 55 */
  II ( b, c, d, a, in[ 1], S44, 2240044497); /* 56 */
  II ( a, b, c, d, in[ 8], S41, 1873313359); /* 57 */
  II ( d, a, b, c, in[15], S42, 4264355552); /* 58 */
  II ( c, d, a, b, in[ 6], S43, 2734768916); /* 59 */
  II ( b, c, d, a, in[13], S44, 1309151649); /* 60 */
  II ( a, b, c, d, in[ 4], S41, 4149444226); /* 61 */
  II ( d, a, b, c, in[11], S42, 3174756917); /* 62 */
  II ( c, d, a, b, in[ 2], S43,  718787259); /* 63 */
  II ( b, c, d, a, in[ 9], S44, 3951481745); /* 64 */

  buf[0] += a;
  buf[1] += b;
  buf[2] += c;
  buf[3] += d;

}


int GetData(uint08 *TargetData, uint08 *TargetMask, uint08 *Data, uint08 *Mask) 
{
    int i;

    *Data = 0;
    for(i = 0; i < 8; i++) {
        *Data = (uint08)(((*Data) << 1) + TargetData[i]);
        *Mask = (uint08)(((*Mask) << 1) + TargetMask[i]);
    }
    return 1;
}

int MakeTable(BitDataType *BitTable)
{
    int i, j, k;

    uint08   Mask;
    uint08   Count;

    uint08   MD5Bit;
    uint08   OutputData;
    uint08   InputData;

    for(i = 0; i < 256; i++) {
        for(j = 0; j < 256; j++) {
            InputData   = (uint08)i;
            MD5Bit      = (uint08)j;

            Count       = 0;
            OutputData  = 0;
            for(k = 0, Mask = 0x80; k < 8; k++, Mask = Mask >> 1) {
                if(MD5Bit & Mask) {
                    OutputData = OutputData << 1;
                    if(InputData & Mask) {
                        OutputData = OutputData + 1;
                    }
                    Count++;
                }
            }
            BitTable[i * 256 + j].Data  = (uint08)(OutputData << (8 - Count));
            BitTable[i * 256 + j].Count = Count;
            BitTable[i * 256 + j].Mask  = (uint08)(0xFF       << (8 - Count));
        }
    }

    return 0;
}


int MakeColorTable(uint08 *ColorTable)
{
    uint16               ColorValue;
    uint16               i;

    memset(ColorTable, 0, 0x400);

    for(i = 0; i < 0x100; i++) {
        ColorTable[i * 4 + 0] = (uint08)i; // B
        ColorTable[i * 4 + 1] = (uint08)i; // G
        ColorTable[i * 4 + 2] = (uint08)i; // R
    }

    for(i = 0; i < 0x10; i++) {
        ColorValue = i *0x10;
        if(ColorValue >= 0x100) ColorValue= 0xFF;
        ColorTable[(i + IMG_BLACK_OFFSET) * 4 + 0] = (uint08)(ColorValue);  // B
        ColorTable[(i + IMG_BLACK_OFFSET) * 4 + 1] = (uint08)(ColorValue);  // G
        ColorTable[(i + IMG_BLACK_OFFSET) * 4 + 2] = (uint08)(ColorValue);  // R
    }

    for(i = 0; i < 0x10; i++) {
        ColorValue = i *0x10;
        if(ColorValue >= 0x100) ColorValue= 0xFF;
        ColorTable[(i + IMG_BLUE_OFFSET) * 4 + 0] = (uint08)(ColorValue);  // B
        ColorTable[(i + IMG_BLUE_OFFSET) * 4 + 1] = (uint08)0x00;  // G
        ColorTable[(i + IMG_BLUE_OFFSET) * 4 + 2] = (uint08)0x00;  // R
    }

    for(i = 0; i < 0x10; i++) {
        ColorValue = i *0x10;
        if(ColorValue >= 0x100) ColorValue= 0xFF;
        ColorTable[(i + IMG_GREEN_OFFSET) * 4 + 0] = (uint08)0x00;  // B
        ColorTable[(i + IMG_GREEN_OFFSET) * 4 + 1] = (uint08)(ColorValue);  // G
        ColorTable[(i + IMG_GREEN_OFFSET) * 4 + 2] = (uint08)0x00;  // R
    }

    for(i = 0; i < 0x10; i++) {
        ColorValue = i *0x10;
        if(ColorValue >= 0x100) ColorValue= 0xFF;
        ColorTable[(i + IMG_RED_OFFSET) * 4 + 0] = (uint08)0x00;  // B
        ColorTable[(i + IMG_RED_OFFSET) * 4 + 1] = (uint08)0x00;  // G
        ColorTable[(i + IMG_RED_OFFSET) * 4 + 2] = (uint08)(ColorValue);  // R
    }

    for(i = 0; i < 0x10; i++) {
        ColorValue = i *0x10;
        if(ColorValue >= 0x100) ColorValue= 0xFF;
        ColorTable[(i + IMG_YELLOW_OFFSET) * 4 + 0] = (uint08)0x00;  // B
        ColorTable[(i + IMG_YELLOW_OFFSET) * 4 + 1] = (uint08)(ColorValue);  // G
        ColorTable[(i + IMG_YELLOW_OFFSET) * 4 + 2] = (uint08)(ColorValue);  // R
    }

    return 1;
}


int ReadBmpFile(char *strFileName, uint32 *pBitmapDataLength, uint08 **ppBitmapData)
{
    BITMAPFILEHEADER    BitMapHeader;
    BITMAPINFOHEADER    BitMapInfoHeader;
	uint08	            *pBitmapData;
    FILE                *fBmp;
    int32               j, k;

    fBmp = fopen(strFileName, "rb");
    if(fBmp == NULL) return 1;

    fread(&BitMapHeader, 1, sizeof(BITMAPFILEHEADER), fBmp);
    fread(&BitMapInfoHeader, 1, sizeof(BITMAPINFOHEADER), fBmp);
    BitMapInfoHeader.biSizeImage    = BitMapHeader.bfSize - BitMapHeader.bfOffBits;
    pBitmapData                     = (uint08 *)malloc(BitMapInfoHeader.biSizeImage);
    *ppBitmapData                   = pBitmapData;
    for(k = 0, j = BitMapInfoHeader.biHeight - 1; k < BitMapInfoHeader.biHeight ; k++, j--) {
        fread(&pBitmapData[j*BitMapInfoHeader.biWidth*3], 1, BitMapInfoHeader.biWidth*3, fBmp);
    }
    fclose(fBmp);

    *pBitmapDataLength = BitMapInfoHeader.biSizeImage;
    return 1;
}


int GetEncryptData(uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pEncryptBitData, uint08 *pEncryptByteData)
{
    uint32  i, j;
    uint32  BitPosition;
    uint32  BmpPosition;
    uint32  DataIndex;
    uint08  DataMask;
        
    for(BitPosition = 1, DataIndex = 0; BitPosition <= BitMapDataLength; BitPosition++, DataIndex++) {
        BmpPosition     = (BitPosition * 0x11) % (BitMapDataLength - 1);
        if(pBitmapData[BmpPosition] & 0x01) {
            pEncryptBitData[DataIndex] = 0x01;
        }
        else {
            pEncryptBitData[DataIndex] = 0x00;
        }
    }

    for(i = 0; (i * 8) <= BitMapDataLength; i++) {
        pEncryptByteData[i] = 0;
        for(j = 0, DataMask = 0x80; j < 8; j++) {
            if(pEncryptBitData[(i * 8) + j]) {
                pEncryptByteData[i] |= DataMask;
            }
            DataMask = DataMask >> 1;
        }
    }
    
    return 1;
}

int PrintEncryptData(char *strFileName, uint08 *pEncryptByteData, uint32 EncryptDataByteLength)
{
    char                TempFileName[128];
    char                OutFileName[128];
    char                *pFileName;
    FILE                *fTxt;
    uint32              i;

    sprintf(TempFileName, "%s", strFileName);
    pFileName = TempFileName;
    while((*pFileName != 0) && (*pFileName != '.')) pFileName++;
    *pFileName = 0;
    sprintf(OutFileName, "%s_r1.txt", TempFileName);
    fTxt = fopen(OutFileName, "wt");
    if(fTxt == NULL) return 1;

    for(i = 0; i < EncryptDataByteLength; i++) {
        if((i % 16) == 0) {
            fprintf(fTxt, "%02X", pEncryptByteData[i]);
        }
        else if((i % 16) == 15) {
            fprintf(fTxt, " %02X\n", pEncryptByteData[i]);
        }
        else {
            fprintf(fTxt, " %02X", pEncryptByteData[i]);
        }
    }

    if((i % 16) != 15) {
        fprintf(fTxt, "\n");
    }

    fclose(fTxt);

    return 1;
}

int EncryptData2BmpFile(char *strFileName, uint08 *pBitmapData, uint32 BitMapDataLength)
{

    char                TempFileName[128];
    char                OutFileName[128];
    char                *pFileName;
    
    FILE                *fBmp;

    uint32              i, j;
    uint32              BitPosition;
    uint32              BmpPosition;
    uint08              *pOutputBmpData;
    uint32              ImgWidth;
    uint32              ImgHeight;
    uint32              OneHeight;
    uint32              ImgPositionX;
    uint32              ImgPositionY;
    uint32              Segment;           
    

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
	uint08               ColorTable[0x400];  

    sprintf(TempFileName, "%s", strFileName);
    pFileName = TempFileName;
    while((*pFileName != 0) && (*pFileName != '.')) pFileName++;
    *pFileName = 0;
    sprintf(OutFileName, "%s_r1.bmp", TempFileName);
    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    MakeColorTable(ColorTable);

	ImgWidth                            = OUTPUT_IMG_WIDTH;
    OneHeight                           = (((BitMapDataLength + 16) / 17) + ImgWidth - 1) / ImgWidth;
    ImgHeight                           = (OneHeight + OUTPUT_IMG_SKIP) * 17 + IMG_BLOCK_BLANK;
    
    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;
    
    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;

    pOutputBmpData = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 0, OutBitMapInfoHeader.biSizeImage);
    
    ImgPositionX = 0;
    ImgPositionY = 1;
    Segment      = 0;
    for(BitPosition = 1; BitPosition <= BitMapDataLength; BitPosition++) {
        BmpPosition     = (BitPosition * 0x11) % (BitMapDataLength - 1);
        if(pBitmapData[BmpPosition] & 0x01) {
            pOutputBmpData[ImgPositionY * OUTPUT_IMG_WIDTH + ImgPositionX] = 0x0F;
        }
        
        ImgPositionX++;
        if(ImgPositionX >= OUTPUT_IMG_WIDTH) {
            if(Segment != (((BitPosition + 1) * 0x11) / (BitMapDataLength - 1))) {
                Segment         = (((BitPosition + 1) * 0x11) / (BitMapDataLength - 1));
                ImgPositionX    = 0;
                ImgPositionY   += OUTPUT_IMG_SKIP;
            }
            else {
                ImgPositionX = 0;
                ImgPositionY++;
            }
        }

    }

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
       
    for(i = 0, j = OutBitMapInfoHeader.biHeight - 1; i < (uint32)OutBitMapInfoHeader.biHeight ; i++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }

    free(pOutputBmpData);
    fclose(fBmp);

    return 1;
}

int EncryptDataCheck(char *strFileName, uint08 *pBitmapData, uint08 **ppBitmapData, uint32 argc, uint32 BitMapDataLength)
{

    char                TempFileName[128];
    char                OutFileName[128];
    char                *pFileName;
    
    FILE                *fBmp;

    uint32              i, j;
    uint32              BitPosition;
    uint32              BmpPosition;
    uint08              *pOutputBmpData;
    uint32              ImgWidth;
    uint32              ImgHeight;
    uint32              OneHeight;
    uint32              ImgPositionX;
    uint32              ImgPositionY;
    uint32              Count;
    uint32              MaxCount;
    uint32              Segment;           

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
	uint08               ColorTable[0x400];  

    sprintf(TempFileName, "%s", strFileName);
    pFileName = TempFileName;
    while((*pFileName != 0) && (*pFileName != '.')) pFileName++;
    *pFileName = 0;
    sprintf(OutFileName, "%s_r2.bmp", TempFileName);
    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    MakeColorTable(ColorTable);

	ImgWidth                            = OUTPUT_IMG_WIDTH;
    OneHeight                           = (((BitMapDataLength + 16) / 17) + ImgWidth - 1) / ImgWidth;
    ImgHeight                           = (OneHeight + OUTPUT_IMG_SKIP) * 17 + IMG_BLOCK_BLANK;
    
    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;
    
    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;

    pOutputBmpData = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 2, OutBitMapInfoHeader.biSizeImage);


    ImgPositionX = 0;
    ImgPositionY = 1;
    MaxCount     = 0;
    for(BitPosition = 1; BitPosition <= BitMapDataLength; BitPosition++) {
        BmpPosition     = (BitPosition * 0x11) % (BitMapDataLength - 1);

        Count = 0;
        for(i = 1; i < argc; i++) {
            if(pBitmapData != ppBitmapData[i]) {
                if(pBitmapData[BmpPosition] == (ppBitmapData[i])[BmpPosition]) {
                    Count++;
                }
            }
        }

        if(MaxCount < Count) MaxCount = Count;
    }
       
    ImgPositionX = 0;
    ImgPositionY = 1;
    Segment      = 0;
    for(BitPosition = 1; BitPosition <= BitMapDataLength; BitPosition++) {
        BmpPosition     = (BitPosition * 0x11) % (BitMapDataLength - 1);
        
        Count = 0;
        for(i = 1; i < argc; i++) {
            if(pBitmapData != ppBitmapData[i]) {
                if(pBitmapData[BmpPosition] == (ppBitmapData[i])[BmpPosition]) {
                    Count++;
                }
            }
        }
        pOutputBmpData[ImgPositionY * OUTPUT_IMG_WIDTH + ImgPositionX] = (uint08)(0x0E - (MaxCount - Count) * 3);

        ImgPositionX++;
        if(ImgPositionX >= OUTPUT_IMG_WIDTH) {
            ImgPositionX = 0;
            ImgPositionY++;
            if(Segment != (((BitPosition + 1) * 0x11) / (BitMapDataLength - 1))) {
                Segment         = (((BitPosition + 1) * 0x11) / (BitMapDataLength - 1));
                ImgPositionX    = 0;
                ImgPositionY   += OUTPUT_IMG_SKIP;
            }
        }
    }

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
       
    for(i = 0, j = OutBitMapInfoHeader.biHeight - 1; i < (uint32)OutBitMapInfoHeader.biHeight ; i++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);
    free(pOutputBmpData);

    return 1;
}

int EncryptDataCheck2(char *strFileName, uint08 *pEncryptBitData, uint32 BitMapDataLength)
{
    char                TempFileName[128];
    char                OutFileName[128];
    char                *pFileName;
    
    FILE                *fBmp;

    uint32              i, j, k;
    uint32              ImgWidth;
    uint32              ImgHeight;
    uint32              ImgPositionX;
    uint32              ImgPositionY;
    uint32              Segment;     
    
    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
	uint08               ColorTable[0x400];  

    sprintf(TempFileName, "%s", strFileName);
    pFileName = TempFileName;
    while((*pFileName != 0) && (*pFileName != '.')) pFileName++;
    *pFileName = 0;

    MakeColorTable(ColorTable);

    Segment                             = (uint32)(BitMapDataLength + ENCRYPT_IMG_WIDTH * MD5_HASH_BIT_NUMBER - 1) / (uint32)(ENCRYPT_IMG_WIDTH * MD5_HASH_BIT_NUMBER);
	ImgWidth                            = ((ENCRYPT_IMG_WIDTH / HASH_BLOCK_W) * (HASH_BLOCK_W + DIVIDE_LINE) - DIVIDE_LINE + 3) & (~0x03);
    ImgHeight                           = Segment * (MD5_HASH_BIT_NUMBER + DIVIDE_LINE) + 2 * IMG_BLOCK_BLANK - DIVIDE_LINE;
    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;
    
    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;

    pOutputBmpData = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);

    memset(pOutputBmpData, 1, OutBitMapInfoHeader.biSizeImage);

    sprintf(OutFileName, "%s_r4.bmp", TempFileName);
    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;
    
    ImgPositionY = IMG_BLOCK_BLANK;
    for(i = 0; i < BitMapDataLength; i += (ENCRYPT_IMG_WIDTH * MD5_HASH_BIT_NUMBER)) {
        for(k = 0; k < ENCRYPT_IMG_WIDTH; k++) {
            ImgPositionX = (k / HASH_BLOCK_W) * (HASH_BLOCK_W + DIVIDE_LINE) + (k % HASH_BLOCK_W);
            for(j = 0; j < MD5_HASH_BIT_NUMBER; j++) {
            	if((i + k * MD5_HASH_BIT_NUMBER + j) < BitMapDataLength) {
                	if(pEncryptBitData[i + k * MD5_HASH_BIT_NUMBER + j] & 1) {
                    	pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX] = 0x0F;
                    }
                	else {
                    	pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX] = 0x00;
                    }
                }
            }
        }
        ImgPositionY += (DIVIDE_LINE + MD5_HASH_BIT_NUMBER);
    }

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
    
    for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);

    free(pOutputBmpData);

    return 1;
}

int EncryptDataCheck3(char *strFileName, uint08 **ppEncryptBitData, uint32 BitMapDataLength, uint32 indexi, uint32 indexj)
{
    char                TempFileName[128];
    char                OutFileName[128];
    char                *pFileName;
    
    FILE                *fBmp;

    uint32              i, j, k;
    uint32              ImgWidth;
    uint32              ImgHeight;
    uint32              ImgPositionX;
    uint32              ImgPositionY;
    uint32              Segment;     
    
    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
	uint08               ColorTable[0x400];  

    uint08              *pEncryptBitData_i;
    uint08              *pEncryptBitData_j;
    

    sprintf(TempFileName, "%s", strFileName);
    pFileName = TempFileName;
    while((*pFileName != 0) && (*pFileName != '.')) pFileName++;
    *pFileName = 0;

    MakeColorTable(ColorTable);

    pEncryptBitData_i                   = ppEncryptBitData[indexi];
    pEncryptBitData_j                   = ppEncryptBitData[indexj];

    Segment                             = (uint32)(BitMapDataLength + ENCRYPT_IMG_WIDTH * MD5_HASH_BIT_NUMBER - 1) / (uint32)(ENCRYPT_IMG_WIDTH * MD5_HASH_BIT_NUMBER);
	ImgWidth                            = ((ENCRYPT_IMG_WIDTH / HASH_BLOCK_W) * (HASH_BLOCK_W + DIVIDE_LINE) - DIVIDE_LINE + 3) & (~0x03);
    ImgHeight                           = Segment * (MD5_HASH_BIT_NUMBER + DIVIDE_LINE) + 2 * IMG_BLOCK_BLANK - DIVIDE_LINE;
    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;
    
    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;

	sprintf(OutFileName, "%s_r4_%d_%d.bmp", TempFileName, indexi, indexj);
	fBmp = fopen(OutFileName, "wb");
	if(fBmp == NULL) return 1;

    pOutputBmpData = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
	memset(pOutputBmpData, 1, OutBitMapInfoHeader.biSizeImage);

	ImgPositionY = IMG_BLOCK_BLANK;
    for(i = 0; i < BitMapDataLength; i += (ENCRYPT_IMG_WIDTH * MD5_HASH_BIT_NUMBER)) {
        for(k = 0; k < ENCRYPT_IMG_WIDTH; k++) {
            ImgPositionX = (k / HASH_BLOCK_W) * (HASH_BLOCK_W + DIVIDE_LINE) + (k % HASH_BLOCK_W);
            for(j = 0; j < MD5_HASH_BIT_NUMBER; j++) {
            	if((i + k * MD5_HASH_BIT_NUMBER + j) < BitMapDataLength) {
                	if((pEncryptBitData_i[i + k * MD5_HASH_BIT_NUMBER + j] & 1) == (pEncryptBitData_j[i + k * MD5_HASH_BIT_NUMBER + j] & 1)) {
                    	pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX] = 0x0F;
                    }
                	else {
                    	pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX] = 0x00;
                    }
                }
            }
        }
        ImgPositionY += (DIVIDE_LINE + MD5_HASH_BIT_NUMBER);
    }

	fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
	fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
	fwrite(ColorTable, 1, 0x400, fBmp);
       
	for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
    	fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
	fclose(fBmp);

    free(pOutputBmpData);

    return 1;
}


int FindMD5HashValue(uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pEncryptByteData, uint08 *pMD5HashBit)
{
    uint16      MD5HashByte[MD5_HASH_BYTE_NUMBER*2];
    uint08      ResultData[MD5_HASH_BYTE_NUMBER*2];
    uint16      ResultBitPos[MD5_HASH_BYTE_NUMBER*2];
    uint16      ResultBytePos[MD5_HASH_BYTE_NUMBER*2];
    
    uint08      DstBitPos[MD5_HASH_BYTE_NUMBER*2];
    uint08      DstData;
    uint08      DstMask;
    uint08      EncryptData;
    
    uint08      SrcData;
	uint08      SrcMask;
    uint08      SrcBitCount;

    uint16      FileNameLength;
    uint32      FileLength;

	__int64     TotalCount;

    int32       Step;

    uint32      BitPositionX;
    uint32      DstbitPos;
    uint32      DataMask;
    uint32      DataIndex;
    BitDataType Intput2MD5DataTable[256*256]; 
    uint08      InMD5HashByteMask[MD5_HASH_BYTE_NUMBER];
	uint08      InMD5HashByte[MD5_HASH_BYTE_NUMBER];

    uint08      _3rdEyeHeaderInfo[_3rdEye_HEADER_BYTES+1];
	uint08	    *pFileName;
    uint32      InputPosition;
    uint32      i, j;
	//uint08      *pFileDataBuff;
	//FILE	    *pOutputFile;

    static uint08 TargetData[128] = { 
                                  /* 0x54 */                            0, 1, 0, 1,  0, 1, 0, 0, 
                                  /* 0x54 */                            0, 1, 0, 1,  0, 1, 0, 0, 
                                  /* 0x45 */                            0, 1, 0, 0,  0, 1, 0, 1, 
                                  /* FileNameLength < 64 */             0, 0, 0, 0,  0, 0, 0, 0,  
                                                                        0, 0, 0, 0,  0, 0, 0, 0,
                                  /* FileLength < 512Kbyts */           0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                                  /* FileName (Ascii Only MSB == 0) */  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
                                                                        0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0
                                };
        
    static uint08 TargetMask[128] = { 
                                  /* 0x54 */                            1, 1, 1, 1,  1, 1, 1, 1, 
                                  /* 0x54 */                            1, 1, 1, 1,  1, 1, 1, 1, 
                                  /* 0x45 */                            1, 1, 1, 1,  1, 1, 1, 1, 
                                  /* FileNameLength < 64 */             1, 1, 0, 0,  0, 0, 0, 0, 
                                                                        1, 1, 1, 1,  1, 1, 1, 1,
                                  /* FileLength < 512Kbyts */           0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  1, 1, 1, 1,  0, 0, 0, 0,  1, 1, 1, 1,  1, 1, 1, 1,
                                  /* FileName (Ascii Only MSB == 0) */  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,
                                                                        1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0
                                };

    MakeTable(Intput2MD5DataTable);    

    BitPositionX    = 0;
    DstData         = 0;
    DataMask        = 0x80;
    DataIndex       = 0;

    // MD5 
    for(j = 0; j < MD5_HASH_BYTE_NUMBER; j++) {
        InMD5HashByte[j]     = 0;
        InMD5HashByteMask[j] = 0xFF;
    }

    DstData         = 0;
    DataMask        = 0x80;
    DataIndex       = 0;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        if(pMD5HashBit[i] == HASH_BIT_0) {

        }
        else if(pMD5HashBit[i] == HASH_BIT_1) {
            DstData = (uint08)(DstData | DataMask);
        }
        else {
            InMD5HashByteMask[DataIndex] = (uint08)(InMD5HashByteMask[DataIndex] ^ DataMask);
        }

        DataMask = DataMask >> 1;
        if(DataMask == 0) {
            InMD5HashByte[DataIndex]	= DstData;
            DataIndex++;
            DstData						= 0x00;
            DataMask					= 0x80;
        }
    }

	TotalCount   = 0;
    for(i = 0; i < 16; i++) {
        MD5HashByte[i]   = 256;
        ResultData[i]    = 0;
        ResultBitPos[i]  = 0;
        ResultBytePos[i] = 0;
    }

    Step               = 0;
    DstBitPos[0]      = 0;   
    GetData( &TargetData[DstBitPos[0]], &TargetMask[DstBitPos[0]], &DstData, &DstMask);
    EncryptData       = pEncryptByteData[Step];
    MD5HashByte[Step] = 255;

    while(1) {

        if((MD5HashByte[Step] & InMD5HashByteMask[Step]) == InMD5HashByte[Step]) {
            SrcData      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[Step]].Data;
            SrcMask      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[Step]].Mask;
            SrcBitCount  = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[Step]].Count;

            if((DstData & SrcMask) == (SrcData & SrcMask & DstMask)) {
                if((SrcBitCount + ResultBitPos[Step]) >= 8) {
                    ResultData[ResultBytePos[Step]]     = (ResultData[ResultBytePos[Step]]     & ~(SrcMask >> ResultBitPos[Step])) | (SrcData >> ResultBitPos[Step]);
                    // ResultData[ResultBytePos[Step] + 1] = (ResultData[ResultBytePos[Step] + 1] & ~(SrcMask << (8 - ResultBitPos[Step]))) | (SrcData << (8 - ResultBitPos[Step]));
                    ResultData[ResultBytePos[Step] + 1] = SrcData << (8 - ResultBitPos[Step]);
                    ResultBytePos[Step+1]               = ResultBytePos[Step] + 1;
                    ResultBitPos[Step+1]                = ResultBitPos[Step] + SrcBitCount - 8;

                    if(ResultBytePos[Step] >= _3rdEye_HEADER_BYTES) {
                        break;
                    }
                }
                else {
                    ResultData[ResultBytePos[Step]]     = (ResultData[ResultBytePos[Step]] & ~(SrcMask >> ResultBitPos[Step])) | (SrcData >> ResultBitPos[Step]);
                	ResultBytePos[Step+1]               = ResultBytePos[Step];
                    ResultBitPos[Step+1]                = ResultBitPos[Step] + SrcBitCount;
                }

                DstBitPos[Step+1] = DstBitPos[Step] + SrcBitCount;
                if((Step + 1) >= MD5_HASH_BYTE_NUMBER) {   
                    fprintf(stderr, "MD5HashBit2     : "); 
                    for(j = 0; j < MD5_HASH_BYTE_NUMBER ; j++) {
                        fprintf(stderr, "0x%02X,", MD5HashByte[j]); 
                    }
                    fprintf(stderr, "\n"); 

                    fprintf(stderr, "ResultData      : "); 
                    for(j = 0; j < MD5_HASH_BYTE_NUMBER ; j++) {
                        fprintf(stderr, " %02X", ResultData[j]); 
                    }
                    fprintf(stderr, "\n"); 
                    
                	fprintf(stderr, "3rdEye Header :");

                    for(InputPosition = 0, DataIndex = 0, DstData = 0, DstbitPos = 0; DataIndex < _3rdEye_HEADER_BYTES; InputPosition++) {
						EncryptData  = pEncryptByteData[InputPosition];
                        SrcData      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Data;
                        SrcMask      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Mask;
                        SrcBitCount  = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Count;

                        if((SrcBitCount + DstbitPos) >= 8) {
                            _3rdEyeHeaderInfo[DataIndex]    = (DstData     & ~(SrcMask >> DstbitPos)) | (SrcData >> DstbitPos);
                            fprintf(stderr, " %02X", _3rdEyeHeaderInfo[DataIndex]);
                            DstData                         = SrcData << (8 - DstbitPos);
                            DataIndex++;
                            DstbitPos                       = DstbitPos + SrcBitCount - 8;
                        }
                        else {
                            DstData     = (DstData & ~(SrcMask >> DstbitPos)) | (SrcData >> DstbitPos);
                            DstbitPos   = DstbitPos + SrcBitCount;
                        }
                        
                    }
                    fprintf(stderr, "\n");

                	memcpy(&FileNameLength, &_3rdEyeHeaderInfo[3], 2);
                	memcpy(&FileLength, &_3rdEyeHeaderInfo[5], 4);

                	pFileName = (uint08 *)malloc(FileNameLength + 1);
                	memset(pFileName, 0, FileNameLength + 1);

                    for(DataIndex = 0; DataIndex < FileNameLength; InputPosition++) {
						EncryptData  = pEncryptByteData[InputPosition];
                        SrcData      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Data;
                        SrcMask      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Mask;
                        SrcBitCount  = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Count;

                        if((SrcBitCount + DstbitPos) >= 8) {
                            pFileName[DataIndex] = (DstData     & ~(SrcMask >> DstbitPos)) | (SrcData >> DstbitPos);
                            DstData              = SrcData << (8 - DstbitPos);
                            DataIndex++;
                            DstbitPos            = DstbitPos + SrcBitCount - 8;
                        }
                        else {
                            DstData     = (DstData & ~(SrcMask >> DstbitPos)) | (SrcData >> DstbitPos);
                            DstbitPos   = DstbitPos + SrcBitCount;
                        }
                        
                    }
                	fprintf(stderr, "FileName   : %s\n", pFileName);
                	fprintf(stderr, "FileLength : %d\n", FileLength);

#if 0
                    for(i = 0; i < FileNameLength; i++) {
                        if(((pFileName[i] >= 'a') && (pFileName[i] <= 'z')) || ((pFileName[i] >= 'A') && (pFileName[i] <= 'Z')) || ((pFileName[i] >= '0') && (pFileName[i] <= '9')) ||
                            (pFileName[i] == '.') || (pFileName[i] == '_') || (pFileName[i] == '-')) {
                        }
                        else {
                            break;
                        }
                    }

                    if(i == FileNameLength) {

                        pFileDataBuff = (uint08 *)malloc(FileLength + 1);

                    	memset(pFileDataBuff, 0, FileLength + 1);

                        for(DataIndex = 0; DataIndex < FileNameLength; InputPosition++) {
    						EncryptData  = pEncryptByteData[InputPosition];
                            SrcData      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Data;
                            SrcMask      = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Mask;
                            SrcBitCount  = Intput2MD5DataTable[EncryptData * 256 + MD5HashByte[InputPosition % MD5_HASH_BYTE_NUMBER]].Count;

                            if((SrcBitCount + DstbitPos) >= 8) {
                                pFileDataBuff[DataIndex] = (DstData     & ~(SrcMask >> DstbitPos)) | (SrcData >> DstbitPos);
                                DstData                  = SrcData << (8 - DstbitPos);
                                DataIndex++;
                                DstbitPos                = DstbitPos + SrcBitCount - 8;
                            }
                            else {
                                DstData     = (DstData & ~(SrcMask >> DstbitPos)) | (SrcData >> DstbitPos);
                                DstbitPos   = DstbitPos + SrcBitCount;
                            }
                        }
 
                    	pOutputFile = fopen((char *)pFileName, "wb");
                    	fwrite(pFileDataBuff, 1, FileLength, pOutputFile);
                    	fclose(pOutputFile);

                    	free(pFileDataBuff);


                    }
#endif                    
					free(pFileName);
                }
                else {
                    Step++;
                    DstBitPos[Step]      = DstBitPos[Step - 1] + SrcBitCount;
                    MD5HashByte[Step]    = 256;
                    EncryptData          = pEncryptByteData[Step];
                    GetData( &TargetData[DstBitPos[Step]], &TargetMask[DstBitPos[Step]], &DstData, &DstMask);
                }
            }

        }

        if(MD5HashByte[Step] == 0) {
            if(Step == 0) {
				break;
			}

            Step--;
        	if(MD5HashByte[Step] == 0) {
            	while(MD5HashByte[Step] == 0) {
                	Step--;
                }
            }
        	MD5HashByte[Step]--;
    		GetData( &TargetData[DstBitPos[Step]], &TargetMask[DstBitPos[Step]], &DstData, &DstMask);
        }
        else {
            MD5HashByte[Step]--;
        }
    }

	return 1;
}


int Reverse3rdEye(uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pMD5HashBit, uint08 *pMD5HashByte)
{
    uint32      BmpPosition;
    uint32      BitPosition;
    uint08      DstMask;
    uint08      DstData;
    uint32      DataIndex;
    uint08      _3rdEyeHeaderInfo[_3rdEye_HEADER_BYTES];
    uint32      MD5Position;
	uint16      FileNameLength;
	uint32      FileLength;
	uint08	    *pFileName;
	uint08      *pFileDataBuff;
	FILE	    *pOutputFile;
    uint08      MD5HashBit[MD5_HASH_BIT_NUMBER];
    uint32      i, j;

    DstData    = 0x00;
    DstMask    = 0x80;
    
    memcpy(MD5HashBit, pMD5HashBit, MD5_HASH_BIT_NUMBER);

    for(i = 0; i < MD5_HASH_BYTE_NUMBER; i++) {
        pMD5HashByte[i] = 0;
        for(j = 0, DstMask = 0x80; j < 8; j++) {
            if(MD5HashBit[i * 8  + j] == HASH_BIT_1) {
                pMD5HashByte[i] |= DstMask;
            }
            DstMask = DstMask >> 1;
        }
        fprintf(stdout, "%02X", pMD5HashByte[i]);
    }
    fprintf(stdout, "\n");

	fprintf(stdout, "3rdEye Header :");

    BitPosition = 1;
    DataIndex   = 0;
	MD5Position = 0;
	DstMask    = 0x80;

    while(BitPosition < BitMapDataLength) {
    	if(MD5HashBit[MD5Position] == HASH_BIT_1) {
            BmpPosition = BitPosition * 0x11 % (BitMapDataLength - 1);
            if(pBitmapData[BmpPosition] & 1) {
                DstData |= DstMask;
            }
            
            DstMask = DstMask >> 1;
            if(DstMask == 0) {
                _3rdEyeHeaderInfo[DataIndex] = DstData;
                DataIndex++;
                fprintf(stderr, " %02X", DstData);
                DstData     = 0;
                DstMask    = 0x80;
                if(DataIndex >= _3rdEye_HEADER_BYTES) {
                	BitPosition++;
                	MD5Position++;
                	if(MD5Position == MD5_HASH_BIT_NUMBER) {
                    	MD5Position = 0;
                    }
                	break;
                }
            }
        }
        BitPosition++;
    	MD5Position++;
    	if(MD5Position == MD5_HASH_BIT_NUMBER) {
        	MD5Position = 0;
        }
    }
	fprintf(stdout, "\n");

	memcpy(&FileNameLength, &_3rdEyeHeaderInfo[3], 2);
	memcpy(&FileLength, &_3rdEyeHeaderInfo[5], 4);

	pFileName = (uint08 *)malloc(FileNameLength + 1);
	memset(pFileName, 0, FileNameLength + 1);

	DataIndex   = 0;
    while(BitPosition < BitMapDataLength) {
    	if(MD5HashBit[MD5Position] == HASH_BIT_1) {
            BmpPosition = BitPosition * 0x11 % (BitMapDataLength - 1);
            if(pBitmapData[BmpPosition] & 1) {
                DstData |= DstMask;
            }
            DstMask = DstMask >> 1;

            if(DstMask == 0) {
                pFileName[DataIndex] = DstData;
                DataIndex++;
                DstData     = 0;
                DstMask    = 0x80;
                if(DataIndex >= FileNameLength) {
                	BitPosition++;
                	MD5Position++;
                	if(MD5Position == MD5_HASH_BIT_NUMBER) {
                    	MD5Position = 0;
                    }
                	break;
                }
            }
        }
        BitPosition++;
    	MD5Position++;
    	if(MD5Position == MD5_HASH_BIT_NUMBER) {
        	MD5Position = 0;
        }
    }

	fprintf(stdout, "FileName   : %s\n", pFileName);
	fprintf(stdout, "FileLength : %d\n", FileLength);
	pFileDataBuff =  (uint08 *)malloc(FileLength + 1);

	DataIndex   = 0;
    while(DataIndex < FileLength) {
    	if(MD5HashBit[MD5Position] == HASH_BIT_1) {
            BmpPosition = BitPosition * 0x11 % (BitMapDataLength - 1);
            if(pBitmapData[BmpPosition] & 1) {
                DstData |= DstMask;
            }
            DstMask = DstMask >> 1;
            if(DstMask == 0) {
                pFileDataBuff[DataIndex] = DstData;
                DataIndex++;
                DstData     = 0;
                DstMask    = 0x80;
            	if(DataIndex >= FileLength) {
                	BitPosition++;
                	MD5Position++;
                	if(MD5Position == MD5_HASH_BIT_NUMBER) {
                    	MD5Position = 0;
                    }
                	break;
                }
            }
        }

        BitPosition++;
        if(BitPosition == BitMapDataLength) {
            for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
                if(MD5HashBit[i] == HASH_BIT_1) {
                    MD5HashBit[i] = HASH_BIT_0;
                }
                else {
                    MD5HashBit[i] = HASH_BIT_1;
                }
            }
            // DstMask     = 0x80;
            MD5Position = 0;
        }
        else {
        	MD5Position++;
        	if(MD5Position == MD5_HASH_BIT_NUMBER) {
            	MD5Position = 0;
            }
        }
    }

	pOutputFile = fopen((char *)pFileName, "wb");
	fwrite(pFileDataBuff, 1, FileLength, pOutputFile);
	fclose(pOutputFile);

	free(pFileDataBuff);
	free(pFileName);

    return 1;
}


int Find_File_1_MD5HashValue(uint08 *pEncryptBitData1, uint08 *pEncryptBitData4, uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pMD5HashBit, uint08 *pMD5HashByte)
{
    char                OutFileName[128];

    uint32              ImgWidth;
    uint32              ImgHeight;

    uint32              i, j, k;
    uint32              CheckStartBitPos;
    uint32              CheckEndBitPos;
    uint32              ShowEndPos;
    uint32              PositionX1;
    uint32              PositionX2;   

    uint32              ImgPositionX;
    uint32              ImgPositionY;

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
    FILE                *fBmp;
	uint08               ColorTable[0x400];  

    CheckStartBitPos = 42 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    CheckEndBitPos   = 249 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndPos       = 46 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;

    memset(pMD5HashBit, HASH_BIT_UNDEF, MD5_HASH_BIT_NUMBER);      

    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if((pEncryptBitData1[CheckStartBitPos + PositionX1] & 1) == (pEncryptBitData4[CheckStartBitPos + PositionX1] & 1)) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if((pEncryptBitData1[PositionX2] & 1) != (pEncryptBitData4[PositionX2] & 1)) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit[PositionX1] = HASH_BIT_0;
            }
            else {
                pMD5HashBit[PositionX1] = HASH_BIT_1;
            }
        }
        else {
            pMD5HashBit[PositionX1] = HASH_BIT_1;
        }
    }

    

    MakeColorTable(ColorTable);

	ImgWidth                            = ((ShowEndPos - CheckStartBitPos) / (MD5_HASH_BIT_NUMBER) * 3 + 3 + 3) & (~0x03);
    ImgHeight                           = 3 * MD5_HASH_BIT_NUMBER + MD5_HASH_BYTE_NUMBER;

    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;

    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;
    OutBitMapHeader.bfReserved1         = 0;
    OutBitMapHeader.bfReserved2         = 0;
    
    pOutputBmpData      = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 0x02, OutBitMapInfoHeader.biSizeImage);

    ImgPositionX = 1;
    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        ImgPositionX = 4;
        for(PositionX1 = CheckStartBitPos + i; PositionX1 < ShowEndPos; PositionX1 += MD5_HASH_BIT_NUMBER) {
            for(j = 0; j < 2; j++) {
                for(k = 0; k < 2; k++) {
                    if((pEncryptBitData1[PositionX1] & 1) != (pEncryptBitData4[PositionX1] & 1)) {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x0F;
                    }
                    else {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x00;
                    }
                }
            }
            ImgPositionX += 3;
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    sprintf(OutFileName, "file_1_r6.bmp");

    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
    
    for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);
    free(pOutputBmpData);

	Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit, pMD5HashByte);
    
    return 1;
}

int Find_File_2_MD5HashValue(uint08 *pEncryptBitData2, uint08 *pEncryptBitData4, uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pMD5HashBit, uint08 *pMD5HashByte)
{
    char                OutFileName[128];

    uint32              ImgWidth;
    uint32              ImgHeight;

    uint32              i, j, k;
    uint32              CheckStartBitPos;
    uint32              CheckEndBitPos;
    uint32              ShowEndPos;
    uint32              PositionX1;
    uint32              PositionX2;
   
    uint32              ImgPositionX;
    uint32              ImgPositionY;

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
    FILE                *fBmp;
	uint08               ColorTable[0x400];  

    CheckStartBitPos = 64 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    CheckEndBitPos   = 249 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndPos       = 68 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;

    memset(pMD5HashBit, HASH_BIT_UNDEF, MD5_HASH_BIT_NUMBER);      

    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if((pEncryptBitData2[CheckStartBitPos + PositionX1] & 1) == (pEncryptBitData4[CheckStartBitPos + PositionX1] & 1)) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if((pEncryptBitData2[PositionX2] & 1) != (pEncryptBitData4[PositionX2] & 1)) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit[PositionX1] = HASH_BIT_0;
            }
            else {
                pMD5HashBit[PositionX1] = HASH_BIT_1;
            }
        }
        else {
            pMD5HashBit[PositionX1] = HASH_BIT_1;
        }
    }

    MakeColorTable(ColorTable);

	ImgWidth                            = ((ShowEndPos - CheckStartBitPos) / (MD5_HASH_BIT_NUMBER) * 3 + 3 + 3) & (~0x03);
    ImgHeight                           = 3 * MD5_HASH_BIT_NUMBER + MD5_HASH_BYTE_NUMBER;

    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;

    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;
    OutBitMapHeader.bfReserved1         = 0;
    OutBitMapHeader.bfReserved2         = 0;
    
    pOutputBmpData      = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 0x02, OutBitMapInfoHeader.biSizeImage);

    ImgPositionX = 1;
    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        ImgPositionX = 4;
        for(PositionX1 = CheckStartBitPos + i; PositionX1 < ShowEndPos; PositionX1 += MD5_HASH_BIT_NUMBER) {
            for(j = 0; j < 2; j++) {
                for(k = 0; k < 2; k++) {
                    if((pEncryptBitData2[PositionX1] & 1) != (pEncryptBitData4[PositionX1] & 1)) {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x0F;
                    }
                    else {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x00;
                    }
                }
            }
            ImgPositionX += 3;
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    sprintf(OutFileName, "file_2_r6.bmp");

    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
    
    for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);
    free(pOutputBmpData);


	Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit, pMD5HashByte);
    
    return 1;
}

int Find_File_3_MD5HashValue(uint08 *pEncryptBitData3, uint08 *pEncryptBitData4, uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pMD5HashBit, uint08 *pMD5HashByte)
{
    char                OutFileName[128];

    uint32              ImgWidth;
    uint32              ImgHeight;

    uint32              i, j, k;
    uint32              CheckStartBitPos;
    uint32              CheckEndBitPos;
    uint32              ShowEndPos;
    uint32              PositionX1;
    uint32              PositionX2;

    uint32              ImgPositionX;
    uint32              ImgPositionY;

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
    FILE                *fBmp;
	uint08               ColorTable[0x400];  

    CheckStartBitPos = 64 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    CheckEndBitPos   = 249 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndPos       = 68 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;

    memset(pMD5HashBit, HASH_BIT_UNDEF, MD5_HASH_BIT_NUMBER);      

    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if((pEncryptBitData3[CheckStartBitPos + PositionX1] & 1) == (pEncryptBitData4[CheckStartBitPos + PositionX1] & 1)) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if((pEncryptBitData3[PositionX2] & 1) != (pEncryptBitData4[PositionX2] & 1)) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit[PositionX1] = HASH_BIT_0;
            }
            else {
                pMD5HashBit[PositionX1] = HASH_BIT_1;
            }
        }
        else {
            pMD5HashBit[PositionX1] = HASH_BIT_1;
        }
    }

    

    MakeColorTable(ColorTable);

	ImgWidth                            = ((ShowEndPos - CheckStartBitPos) / (MD5_HASH_BIT_NUMBER) * 3 + 3 + 3) & (~0x03);
    ImgHeight                           = 3 * MD5_HASH_BIT_NUMBER + MD5_HASH_BYTE_NUMBER;

    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;

    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;
    OutBitMapHeader.bfReserved1         = 0;
    OutBitMapHeader.bfReserved2         = 0;
    
    pOutputBmpData      = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 0x02, OutBitMapInfoHeader.biSizeImage);

    ImgPositionX = 1;
    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        ImgPositionX = 4;
        for(PositionX1 = CheckStartBitPos + i; PositionX1 < ShowEndPos; PositionX1 += MD5_HASH_BIT_NUMBER) {
            for(j = 0; j < 2; j++) {
                for(k = 0; k < 2; k++) {
                    if((pEncryptBitData3[PositionX1] & 1) != (pEncryptBitData4[PositionX1] & 1)) {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x0F;
                    }
                    else {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x00;
                    }
                }
            }
            ImgPositionX += 3;
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    sprintf(OutFileName, "file_3_r6.bmp");

    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
    
    for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);
    free(pOutputBmpData);


    // everse3rdEye3(pEncryptBitData3, BitMapDataLength, pMD5HashBit);
	Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit, pMD5HashByte);
    
    return 1;
}

int Find_File_5_MD5HashValue(uint08 *pEncryptBitData5, uint08 *pEncryptBitData4, uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pMD5HashBit, uint08 *pMD5HashByte)
{
    char                OutFileName[128];

    uint32              ImgWidth;
    uint32              ImgHeight;

    uint32              i, j, k;
    uint32              CheckStartBitPos;
    uint32              CheckEndBitPos;
    uint32              ShowEndPos;
    uint32              PositionX1;
    uint32              PositionX2;

    uint32              ImgPositionX;
    uint32              ImgPositionY;

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
    FILE                *fBmp;
	uint08               ColorTable[0x400];  

    CheckStartBitPos = 35 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    CheckEndBitPos   = 41 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndPos       = 39 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;

    memset(pMD5HashBit, HASH_BIT_UNDEF, MD5_HASH_BIT_NUMBER);      

    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if(pEncryptBitData5[CheckStartBitPos + PositionX1] == pEncryptBitData4[CheckStartBitPos + PositionX1]) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if(pEncryptBitData5[PositionX2] != pEncryptBitData4[PositionX2]) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit[PositionX1] = HASH_BIT_0;
            }
            else {
                pMD5HashBit[PositionX1] = HASH_BIT_1;
            }
        }
        else {
            pMD5HashBit[PositionX1] = HASH_BIT_1;
        }
    }

    MakeColorTable(ColorTable);

	ImgWidth                            = ((ShowEndPos - CheckStartBitPos) / (MD5_HASH_BIT_NUMBER) * 3 + 3 + 3) & (~0x03);
    ImgHeight                           = 3 * MD5_HASH_BIT_NUMBER + MD5_HASH_BYTE_NUMBER;

    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;

    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;
    OutBitMapHeader.bfReserved1         = 0;
    OutBitMapHeader.bfReserved2         = 0;
    
    pOutputBmpData      = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 0x02, OutBitMapInfoHeader.biSizeImage);

    ImgPositionX = 1;
    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        ImgPositionX = 4;
        for(PositionX1 = CheckStartBitPos + i; PositionX1 < ShowEndPos; PositionX1 += MD5_HASH_BIT_NUMBER) {
            for(j = 0; j < 2; j++) {
                for(k = 0; k < 2; k++) {
                    if((pEncryptBitData5[PositionX1] & 1) != (pEncryptBitData4[PositionX1] & 1)) {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x0F;
                    }
                    else {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x00;
                    }
                }
            }
            ImgPositionX += 3;
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    sprintf(OutFileName, "file_5_r6.bmp");

    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
    
    for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);
    free(pOutputBmpData);

    Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit, pMD5HashByte);

    return 1;
}



int Find_File_4_MD5HashValue(uint08 *pEncryptBitData4, uint08 **ppEncryptBitData, uint08 *pBitmapData, uint32 BitMapDataLength, uint08 *pMD5HashBit4, uint08 *pMD5HashByte4, uint08 *pMD5HashBit)
{
    char                OutFileName[128];

    uint32              ImgWidth;
    uint32              ImgHeight;

    uint32              i, j, k;
    uint32              CheckStartBitPos;
    uint32              CheckEndBitPos;
    uint32              ShowEndBitPos;
    uint32              PositionX1;
    uint32              PositionX2;
    uint08              DataMask;

    uint32              ImgPositionX;
    uint32              ImgPositionY;

    BITMAPFILEHEADER    OutBitMapHeader;
    BITMAPINFOHEADER    OutBitMapInfoHeader;
    uint08              *pOutputBmpData    = NULL;
    FILE                *fBmp;
	uint08               ColorTable[0x400];      
    uint08              *pEncryptByteData;

    uint08              MD5HashByteSave[] = { 0x98, 0x36, 0xC9, 0xB6, 0x81, 0x45, 0x00, 0xEB, 0xDF, 0x85, 0xB3, 0x7B, 0x8C, 0xDE, 0x94, 0xD7 };

    memset(pMD5HashBit4, HASH_BIT_UNDEF, MD5_HASH_BIT_NUMBER);      

#if 1
    // 1 & 4
    CheckStartBitPos = 8  * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    CheckEndBitPos   = 32 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndBitPos    = 10 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if((pEncryptBitData4[CheckStartBitPos + PositionX1] & 1) == ((ppEncryptBitData[1])[CheckStartBitPos + PositionX1] & 1)) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if((pEncryptBitData4[PositionX2] & 1) != ((ppEncryptBitData[1])[PositionX2] & 1)) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit4[PositionX1] = HASH_BIT_0;
            }
            else if(pMD5HashBit[1 * MD5_HASH_BIT_NUMBER + PositionX1] == HASH_BIT_0) {
                pMD5HashBit4[PositionX1] = HASH_BIT_1;
            }
        }
        else if(pMD5HashBit[1 * MD5_HASH_BIT_NUMBER + PositionX1] == HASH_BIT_0) {
            pMD5HashBit4[PositionX1] = HASH_BIT_1;
        }
    }
#endif    
#if 1

    // 3 & 4
    CheckStartBitPos = 20 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    CheckEndBitPos   = 32 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndBitPos    = 22 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if((pEncryptBitData4[CheckStartBitPos + PositionX1] & 1) == ((ppEncryptBitData[3])[CheckStartBitPos + PositionX1] & 1)) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if((pEncryptBitData4[PositionX2] & 1) != ((ppEncryptBitData[3])[PositionX2] & 1)) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit4[PositionX1] = HASH_BIT_0;
            }
            else if(pMD5HashBit[3 * MD5_HASH_BIT_NUMBER + PositionX1] == HASH_BIT_0) {
                pMD5HashBit4[PositionX1] = HASH_BIT_1;
            }
        }
        else if(pMD5HashBit[3 * MD5_HASH_BIT_NUMBER + PositionX1] == HASH_BIT_0) {
            pMD5HashBit4[PositionX1] = HASH_BIT_1;
        }
    }
#endif

#if 1
    // 5 & 4
    CheckStartBitPos = 0;
    CheckEndBitPos   = 34 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;
    ShowEndBitPos    = 2 * MD5_HASH_BIT_NUMBER * HASH_BLOCK_W;

    for(PositionX1 = 0; PositionX1 < MD5_HASH_BIT_NUMBER; PositionX1++) {
        if((pEncryptBitData4[CheckStartBitPos + PositionX1] & 1) == ((ppEncryptBitData[5])[CheckStartBitPos + PositionX1] & 1)) {
            for(PositionX2 = CheckStartBitPos + PositionX1; PositionX2 < CheckEndBitPos; PositionX2 += MD5_HASH_BIT_NUMBER) {
                if((pEncryptBitData4[PositionX2] & 1) != ((ppEncryptBitData[5])[PositionX2] & 1)) {
                    break;
                }
            }
            if(PositionX2 >= CheckEndBitPos) {
                pMD5HashBit4[PositionX1] = HASH_BIT_0;
            }
            else if(pMD5HashBit[5 * MD5_HASH_BIT_NUMBER + PositionX1] == HASH_BIT_0) {
                pMD5HashBit4[PositionX1] = HASH_BIT_1;
            }
        }
        else if(pMD5HashBit[5 * MD5_HASH_BIT_NUMBER + PositionX1] == HASH_BIT_0) {
            pMD5HashBit4[PositionX1] = HASH_BIT_1;
        }
    }
#endif

    MakeColorTable(ColorTable);

	ImgWidth                            = ((ShowEndBitPos - CheckStartBitPos) / (MD5_HASH_BIT_NUMBER) * 3 + 12 + 3) & (~0x03);
    ImgHeight                           = 3 * MD5_HASH_BIT_NUMBER + MD5_HASH_BYTE_NUMBER;
    OutBitMapInfoHeader.biSize          = 0x28;
    OutBitMapInfoHeader.biWidth         = ImgWidth;
    OutBitMapInfoHeader.biHeight        = ImgHeight;
    OutBitMapInfoHeader.biPlanes        = 1;
    OutBitMapInfoHeader.biBitCount      = 8;
    OutBitMapInfoHeader.biCompression   = 0;
    OutBitMapInfoHeader.biSizeImage     = ImgWidth * ImgHeight;
    OutBitMapInfoHeader.biXPelsPerMeter = 10000;
    OutBitMapInfoHeader.biYPelsPerMeter = 10000;
    OutBitMapInfoHeader.biClrUsed       = 256;
    OutBitMapInfoHeader.biClrImportant  = 256;

    OutBitMapHeader.bfType              = 0x4D42;
    OutBitMapHeader.bfSize              = 0x0436 + OutBitMapInfoHeader.biSizeImage;
    OutBitMapHeader.bfOffBits           = 0x0436;
    OutBitMapHeader.bfReserved1         = 0;
    OutBitMapHeader.bfReserved2         = 0;
    
    pOutputBmpData      = (uint08 *)malloc(OutBitMapInfoHeader.biSizeImage);
    memset(pOutputBmpData, 0x02, OutBitMapInfoHeader.biSizeImage);

    ImgPositionX = 1;
    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit4[i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit4[i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionX = 4;
    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[1 * MD5_HASH_BIT_NUMBER + i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[1 * MD5_HASH_BIT_NUMBER + i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionX = 7;
    ImgPositionY = 1;

    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[3 * MD5_HASH_BIT_NUMBER + i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[3 * MD5_HASH_BIT_NUMBER + i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

	ImgPositionX = 10;
    ImgPositionY = 1;

    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        for(j = 0; j < 2; j++) {
            for(k = 0; k < 2; k++) {
                if(pMD5HashBit[5 * MD5_HASH_BIT_NUMBER + i] == HASH_BIT_0) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_0_INDEX + 0x0F;
                }
                else if(pMD5HashBit[5 * MD5_HASH_BIT_NUMBER + i] == HASH_BIT_1) {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = HASH_BIT_1_INDEX + 0x0F;
                }
                else {
                    pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                }
            }
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    ImgPositionY = 1;
    for(i = 0; i < MD5_HASH_BIT_NUMBER; i++) {
        ImgPositionX = 13;
        for(PositionX1 = CheckStartBitPos + i; PositionX1 < ShowEndBitPos; PositionX1 += MD5_HASH_BIT_NUMBER) {
            for(j = 0; j < 2; j++) {
                for(k = 0; k < 2; k++) {
                    if(((ppEncryptBitData[5])[PositionX1] & 1) != (pEncryptBitData4[PositionX1] & 1)) {
                        if(((ppEncryptBitData[5])[PositionX1] & _3rdEye_MODIFY_BIT) == 0) {
                            pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_BLUE_OFFSET + 0x0F;
                        }
                        else {
                            pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = IMG_RED_OFFSET + 0x0F;
                        }
                    }
                    else {
                        pOutputBmpData[(ImgPositionY + j) * ImgWidth + ImgPositionX + k] = 0x00;
                    }
                }
            }
            ImgPositionX += 3;
        }
        if((i % 8) == 7) {
            ImgPositionY += 4;
        }
        else {
            ImgPositionY += 3;
        }
    }

    sprintf(OutFileName, "file_4_r6.bmp");

    fBmp = fopen(OutFileName, "wb");
    if(fBmp == NULL) return 1;

    fwrite(&OutBitMapHeader, 1, sizeof(OutBitMapHeader), fBmp);
    fwrite(&OutBitMapInfoHeader, 1, sizeof(OutBitMapInfoHeader), fBmp);
    fwrite(ColorTable, 1, 0x400, fBmp);
    
    for(k = 0, j = OutBitMapInfoHeader.biHeight - 1; k < (uint32)OutBitMapInfoHeader.biHeight ; k++, j--) {
        fwrite(&pOutputBmpData[j * ImgWidth], 1, ImgWidth, fBmp);
    }
    fclose(fBmp);
    free(pOutputBmpData);

    for(i = 0; i < MD5_HASH_BYTE_NUMBER; i++) {
        pMD5HashByte4[i] = 0;
        for(j = 0, DataMask = 0x80; j < 8; j++) {
            if(pMD5HashBit4[j + i] == HASH_BIT_1) {
                pMD5HashByte4[i] |= DataMask;
            }
            DataMask = DataMask >> 1;
        }
        fprintf(stderr, "%02X", pMD5HashByte4[i]);
    }
    fprintf(stderr, "\n");


    pEncryptByteData = (uint08 *)malloc(BitMapDataLength);

    for(i = 0; i < (BitMapDataLength + 7) / 8; i++) {
        pEncryptByteData[i] = 0;
        for(j = 0, DataMask = 0x80; j < 8; j++) {
            if(pEncryptBitData4[i * 8 + j] == 0x01) {
                pEncryptByteData[i] |= DataMask;
            }
            DataMask = DataMask >> 1;
        }
    }

    FindMD5HashValue(pBitmapData, BitMapDataLength, pEncryptByteData, pMD5HashBit4);

    memcpy(pMD5HashByte4, MD5HashByteSave, MD5_HASH_BYTE_NUMBER);
    for(i = 0; i < MD5_HASH_BYTE_NUMBER; i++) {
        for(j = 0, DataMask = 0x80; j < 8; j++) {
            if(pMD5HashByte4[i] & DataMask) {
                pMD5HashBit4[i * 8 + j] = HASH_BIT_1;
            }
            else {
                pMD5HashBit4[i * 8 + j] = HASH_BIT_0;
            }
            DataMask = DataMask >> 1;
        }
    }

    Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit4, pMD5HashByte4);

    // Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit4, pMD5HashByte);
    // Reverse3rdEye(pBitmapData, BitMapDataLength, pMD5HashBit4, pMD5HashByte);

    free(pEncryptByteData);

    return 1;
}

int main(int argc, char* argv[])
{
    int                  i, j, k;
    FILE                *fBmp                   = NULL;
    FILE                *fTxt                   = NULL;
    uint08              **ppBitmapData          = NULL;
    uint08              *pBitmapData            = NULL;
    uint08              **ppEncryptByteData     = NULL;
	uint08              *pEncryptByteData       = NULL;
    uint08              **ppEncryptBitData      = NULL;
	uint08              *pEncryptBitData        = NULL;
    
    uint32              EncryptDataByteLength;
    uint32              EncryptDataBitLength;
    
    char                *pFileName         = NULL;
    uint08              *pOutputBmpData    = NULL;

    uint08               *pMD5HashBit       = NULL;
    uint08               *pMD5HashBit0      = NULL;
    uint08               *pMD5HashBit1      = NULL;
    uint08               *pMD5HashByte      = NULL;
	uint08               ColorTable[0x400];  
    uint16               NumofComb;
    uint32               BitMapDataLength;
    
    uint08              *pMDData;    

    uint32 data1[4]     = { 
        0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476 
    };
    uint32 data2[4]     = { 
        0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476 
    };
    uint08 indata1[0x40] = { 
        0x31, 0x32, 0x33, 0x34, 0x35, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
    };

    uint08 Key[5] = { 0x31, 0x32, 0x33, 0x34, 0x35 };


    MD5Transform(data1, (uint32 *)indata1);    


    pMDData = (uint08 *)data1;

	for(j = 0; j < 16; j++) {
		printf("%02X ", pMDData[j]);
	}
	printf("\n");

#if 0
    for(i = 1; i <= 5; i++) {
        memcpy(data1, data2, sizeof(data2));
        memset(indata1, 0, 0x40);
        indata1[i]    = 0x80;
        indata1[0x38] = (uint08)(i * 8);
        for(j = 0; j < i; j++) {
            indata1[j] = Key[j];
        }

        MD5Transform(data1, (uint32 *)indata1);

        for(j = 0; j < 16; j++) {
            printf("%02X ", pMDData[j]);
        }
        printf("\n");
    }
#endif        

    MakeColorTable(ColorTable);

    NumofComb = 0;
    for(j = 2; j < argc; j++) {
        for(k = 2; k < argc; k++) {
            if((j != k) && (j < k)) {
                NumofComb++;
            }
        }
    }
    NumofComb++;
    
    pMD5HashBit  = (uint08 *)malloc(MD5_HASH_BIT_NUMBER * argc);
    pMD5HashByte = (uint08 *)malloc(MD5_HASH_BYTE_NUMBER * argc);
    pMD5HashBit0 = (uint08 *)malloc(MD5_HASH_BIT_NUMBER * argc);
    pMD5HashBit1 = (uint08 *)malloc(MD5_HASH_BIT_NUMBER * argc);
    for(i = 0; i < (int)MD5_HASH_BIT_NUMBER * argc; i++) {
        pMD5HashBit[i]  = 0xFF;
    }
    for(i = 0; i < (int)MD5_HASH_BYTE_NUMBER * argc; i++) {
        pMD5HashByte[i] = 0xFF;
    }
    for(i = 0; i < (int)MD5_HASH_BIT_NUMBER * argc; i++) {
        pMD5HashBit0[i]  = HASH_BIT_UNDEF;
        pMD5HashBit1[i]  = HASH_BIT_UNDEF;
    }

    ppBitmapData        = (uint08 **)malloc(sizeof(uint08 *) * argc);
    ppEncryptByteData	= (uint08 **)malloc(sizeof(uint08 *) * argc);
    ppEncryptBitData	= (uint08 **)malloc(sizeof(uint08 *) * argc);
    for(i = 0; i < argc; i++) {
        ppBitmapData[i]         = NULL;
    	ppEncryptByteData[i]    = NULL;
    	ppEncryptBitData[i]     = NULL;
    }

    for(i = 1; i < argc; i++) {
        ReadBmpFile(argv[i], &BitMapDataLength, &ppBitmapData[i]);

        EncryptDataBitLength    = (BitMapDataLength + MD5_HASH_BLOCK_SIZE) & (~(MD5_HASH_BLOCK_SIZE - 1));
        pEncryptBitData         = (uint08 *)malloc(sizeof(uint08) * EncryptDataBitLength);
        ppEncryptBitData[i]     = pEncryptBitData;

        memset(pEncryptBitData, 0, EncryptDataBitLength);

        EncryptDataByteLength   = EncryptDataBitLength / 8;
        pEncryptByteData        = (uint08 *)malloc(sizeof(uint08) * EncryptDataByteLength);
        ppEncryptByteData[i]    = pEncryptByteData;
        memset(pEncryptByteData, 0, EncryptDataByteLength);

        GetEncryptData(ppBitmapData[i], BitMapDataLength, pEncryptBitData, pEncryptByteData);
#if 0        
		PrintEncryptData(argv[i], pEncryptByteData, EncryptDataByteLength);
        EncryptData2BmpFile(argv[i], ppBitmapData[i], BitMapDataLength);
#endif
       
    }

#if 0
    for(i = 1; i < argc; i++) {
        EncryptDataCheck2(argv[i], ppEncryptBitData[i], BitMapDataLength);
    }

    for(i = 1; i < argc; i++) {
        for(j = 1; j < argc; j++) {
            if(i != j) {
                EncryptDataCheck3(argv[i], ppEncryptBitData, BitMapDataLength, i, j);
            }
        }
    }
#endif

    Find_File_1_MD5HashValue(ppEncryptBitData[1], ppEncryptBitData[4], ppBitmapData[1], BitMapDataLength, &pMD5HashBit[1 * MD5_HASH_BIT_NUMBER], &pMD5HashByte[1*MD5_HASH_BYTE_NUMBER]);
    Find_File_2_MD5HashValue(ppEncryptBitData[2], ppEncryptBitData[4], ppBitmapData[2], BitMapDataLength, &pMD5HashBit[2 * MD5_HASH_BIT_NUMBER], &pMD5HashByte[2*MD5_HASH_BYTE_NUMBER]);
    Find_File_3_MD5HashValue(ppEncryptBitData[3], ppEncryptBitData[4], ppBitmapData[3], BitMapDataLength, &pMD5HashBit[3 * MD5_HASH_BIT_NUMBER], &pMD5HashByte[3*MD5_HASH_BYTE_NUMBER]);
    Find_File_5_MD5HashValue(ppEncryptBitData[5], ppEncryptBitData[4], ppBitmapData[5], BitMapDataLength, &pMD5HashBit[5 * MD5_HASH_BIT_NUMBER], &pMD5HashByte[5*MD5_HASH_BYTE_NUMBER]);
    Find_File_4_MD5HashValue(ppEncryptBitData[4], ppEncryptBitData,    ppBitmapData[4], BitMapDataLength, &pMD5HashBit[4 * MD5_HASH_BIT_NUMBER], &pMD5HashByte[4*MD5_HASH_BYTE_NUMBER], pMD5HashBit);
    
    free(pMD5HashBit);
    free(pMD5HashByte);
    free(pMD5HashBit0);
    free(pMD5HashBit1);
    if(pOutputBmpData != NULL) free(pOutputBmpData);

    for(i = 1; i < argc; i++) {
        if(ppBitmapData[i] != NULL)       free(ppBitmapData[i]);
        if(ppEncryptByteData[i] != NULL)  free(ppEncryptByteData[i]);
        if(ppEncryptBitData[i] != NULL)   free(ppEncryptBitData[i]);
    }

    free(ppBitmapData);
    free(ppEncryptByteData);
    free(ppEncryptBitData);

	return 1;
}


