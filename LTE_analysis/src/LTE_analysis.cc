/**
  ******************************************************************************
  * @file    LTE_analysis.c
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
/* Includes */

#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/poll.h>

#include <netdb.h>
#include <ifaddrs.h>
#include <time.h>

#include <fftw3.h>
#include <float.h>

#include "LTE_analysis.h"

/** @addtogroup LTE_ANALYSIS
  * @{
  */
/* Private macros ------------------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Private_Macros
  * @{
  */

#define _INPUT_         'i'
#define _OUTPUT_        'o'
#define _MOD_TYPE_      'm'
#define _TEST_          't'
#define _BW_            'b'
#define _DAS_CODE_      'c'
#define _DAS_ADC_BIT_   'a'

#define RANDOM_NUMBER_BITS      6
#define TEST_SYMBOL_COUNT       1000

#define RANDOM_INIT_VALUE       19990711


/* End of LTE_ANALYSIS_Private_Macros */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Private_Types
  * @{
  */
/* End of LTE_ANALYSIS_Private_Types */
/**
  * @}
  */

/* Private functions Prototype ------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Private_Functions
  * @{
  */

static int32_t GainBlock(int32_t N_samples, fftwf_complex *Output, fftwf_complex *Input, float Gain);
  
/* End of LTE_ANALYSIS_Private_Functions */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Private_Variables
  * @{
  */

struct LTE_Analysis_s LTE_Analysis_Context;

static struct option LTE_Analysis_Long_options[] = {
    {"input",           required_argument,      0, _INPUT_        },
    {"output",          required_argument,      0, _OUTPUT_       },
    {"mod",             required_argument,      0, _MOD_TYPE_     },
    {"test",            required_argument,      0, _TEST_         },
    {"bandwidth",       required_argument,      0, _BW_           },
    {"dascodetype",     required_argument,      0, _DAS_CODE_     },
    {"adcbits",         required_argument,      0, _DAS_ADC_BIT_  },
    {"help",            no_argument,            0, 'h'            },
    {0, 0, 0, 0}
};    


/*
    000X    :   6
    001X    :   4
    0100    :   2 
    0101    :   5
    0110    :   0
    0111    :   7
    1000    :   8
    1001    :   9
    1010    :  10 
    1011    :  11
    1100    :  12
    1101    :  1
    1110    :  3
    11110
    111110
    111111  

*/


struct code_info_s code_info_tbl_20_14_1[] =
{    

    { 0,  0x0000,  16384,  32767,      0,  32767, 0x06, 4, 7, 7, 7, 8 },         /* 0x4000 x 0x8000 */
    { 0,  0x0001,      0,  16383,  16384,  32767, 0x0D, 4, 7, 7, 7, 7 },         /* 0x4000 x 0x4000 */
    { 0,  0x0002,   8192,  16383,      0,  16383, 0x04, 4, 7, 6, 7, 7 },         /* 0x2000 x 0x4000 */
    { 0,  0x0003,      0,   8191,   8192,  16383, 0x0E, 4, 7, 6, 7, 6 },         /* 0x2000 x 0x2000 */

    { 0,  0x0004,   4096,   8191,      0,   8191, 0x02, 3, 7, 5, 8, 5 },         /* 0x1000 x 0x2000 */
    { 0,  0x0005,      0,   4095,   4096,   8191, 0x05, 4, 7, 5, 7, 5 },         /* 0x1000 x 0x1000 */
    { 0,  0x0006,   2048,   4095,      0,   4095, 0x00, 3, 7, 4, 8, 4 },         /* 0x0800 x 0x1000 */    
    { 0,  0x0007,      0,   2047,   2048,   4095, 0x07, 4, 7, 4, 7, 4 },         /* 0x0800 x 0x0800 */
    
    { 0,  0x0008,   1024,   2047,      0,   2047, 0x08, 4, 7, 3, 7, 4 },         /* 0x0400 x 0x0800 */    
    { 0,  0x0009,      0,   1023,   1024,   2047, 0x09, 4, 7, 3, 7, 3 },         /* 0x0400 x 0x0400 */
    { 0,  0x000A,    512,   1023,      0,   1023, 0x0A, 4, 7, 2, 7, 3 },         /* 0x0200 x 0x0400 */
    { 0,  0x000B,      0,    511,    512,   1023, 0x0B, 4, 7, 2, 7, 2 },         /* 0x0200 x 0x0200 */    
    
    { 0,  0x000C,      0,    511,      0,    511, 0x0C, 4, 7, 2, 7, 2 },         /* 0x0200 x 0x0200 */

    { -1,  -1, 0, 0, 0, 0 },
};

struct code_info_s code_info_tbl_20_14[] =
{    

    { 0,  0x0000,  16384,  32767,      0,  32767, 0x06, 4, 7, 7, 7, 8 },         /* 0x4000 x 0x8000 */
    { 0,  0x0001,      0,  16383,  16384,  32767, 0x0D, 4, 7, 7, 7, 7 },         /* 0x4000 x 0x4000 */
    { 0,  0x0002,   8192,  16383,      0,  16383, 0x04, 4, 7, 6, 7, 7 },         /* 0x2000 x 0x4000 */
    { 0,  0x0003,      0,   8191,   8192,  16383, 0x0E, 4, 7, 6, 7, 6 },         /* 0x2000 x 0x2000 */

    { 0,  0x0004,   4096,   8191,      0,   8191, 0x02, 4, 7, 5, 7, 6 },         /* 0x1000 x 0x2000 */
    { 0,  0x0005,      0,   4095,   4096,   8191, 0x05, 4, 7, 5, 7, 5 },         /* 0x1000 x 0x1000 */
    { 0,  0x0006,   2048,   4095,      0,   4095, 0x00, 3, 7, 4, 8, 4 },         /* 0x0800 x 0x1000 */    
    { 0,  0x0007,      0,   2047,   2048,   4095, 0x07, 4, 7, 4, 7, 4 },         /* 0x0800 x 0x0800 */
    
    { 0,  0x0008,   1024,   2047,      0,   2047, 0x08, 4, 7, 3, 8, 3 },         /* 0x0400 x 0x0800 */    
    { 0,  0x0009,      0,   1023,   1024,   2047, 0x09, 4, 7, 3, 7, 3 },         /* 0x0400 x 0x0400 */
    { 0,  0x000A,    512,   1023,      0,   1023, 0x0A, 4, 7, 2, 7, 3 },         /* 0x0200 x 0x0400 */
    { 0,  0x000B,      0,    511,    512,   1023, 0x0B, 4, 7, 2, 7, 2 },         /* 0x0200 x 0x0200 */    
    
    { 0,  0x000C,      0,    511,      0,    511, 0x0C, 4, 7, 2, 7, 2 },         /* 0x0200 x 0x0200 */

    { -1,  -1, 0, 0, 0, 0 },
};




struct code_info_s code_info_tbl_19_14[] =
{    

    { 0,  0x0000,  16384,  32767,      0,  32767, 0x06, 4, 6, 8, 7, 8 },         /* 0x4000 x 0x8000 */
    { 0,  0x0001,      0,  16383,  16384,  32767, 0x0D, 4, 7, 7, 6, 8 },         /* 0x4000 x 0x4000 */
    { 0,  0x0002,   8192,  16383,      0,  16383, 0x04, 4, 6, 7, 7, 7 },         /* 0x2000 x 0x4000 */
    { 0,  0x0003,      0,   8191,   8192,  16383, 0x0E, 4, 7, 6, 6, 7 },         /* 0x2000 x 0x2000 */

    { 0,  0x0004,   4096,   8191,      0,   8191, 0x02, 3, 7, 5, 7, 6 },         /* 0x1000 x 0x2000 */
    { 0,  0x0005,      0,   4095,   4096,   8191, 0x05, 4, 7, 5, 6, 6 },         /* 0x1000 x 0x1000 */
    { 0,  0x0006,   2048,   4095,      0,   4095, 0x00, 3, 7, 4, 7, 5 },         /* 0x0800 x 0x1000 */    
    { 0,  0x0007,      0,   2047,   2048,   4095, 0x07, 4, 7, 4, 6, 5 },         /* 0x0800 x 0x0800 */
    
    { 0,  0x0008,   1024,   2047,      0,   2047, 0x08, 4, 6, 4, 7, 4 },         /* 0x0400 x 0x0800 */    
    { 0,  0x0009,      0,   1023,   1024,   2047, 0x09, 4, 7, 3, 6, 4 },         /* 0x0400 x 0x0400 */
    { 0,  0x000A,    512,   1023,      0,   1023, 0x0A, 4, 6, 3, 7, 3 },         /* 0x0200 x 0x0400 */
    { 0,  0x000B,      0,    511,    512,   1023, 0x0B, 4, 7, 2, 6, 3 },         /* 0x0200 x 0x0200 */    
    
    { 0,  0x000C,      0,    511,      0,    511, 0x0C, 4, 7, 2, 6, 3 },         /* 0x0200 x 0x0200 */

    { -1,  -1, 0, 0, 0, 0 },
};

struct code_info_s code_area_tbl[] =
{

    { 0,  0x0000,  16384,  32767,      0,  32767, 0x06, 4, 0x7F80, 7, 0x7F00, 8 },         /* 0x4000 x 0x8000 */
    { 0,  0x0001,      0,  16383,  16384,  32767, 0x0D, 4, 0x3F80, 7, 0x7F80, 7 },         /* 0x4000 x 0x4000 */
    { 0,  0x0002,   8192,  16383,      0,  16383, 0x04, 4, 0x3FC0, 6, 0x3F80, 7 },         /* 0x2000 x 0x4000 */
    { 0,  0x0003,      0,   8191,   8192,  16383, 0x0E, 4, 0x1FC0, 6, 0x3FC0, 6 },         /* 0x2000 x 0x2000 */

    { 0,  0x0004,   4096,   8191,      0,   8191, 0x02, 3, 0x1FE0, 5, 0x1FE0, 5 },         /* 0x1000 x 0x2000 */
    { 0,  0x0005,      0,   4095,   4096,   8191, 0x05, 4, 0x0FE0, 5, 0x1FE0, 5 },         /* 0x1000 x 0x1000 */
    { 0,  0x0006,   2048,   4095,      0,   4095, 0x00, 3, 0x0FF0, 4, 0x0FF0, 4 },         /* 0x0800 x 0x1000 */    
    { 0,  0x0007,      0,   2047,   2048,   4095, 0x07, 4, 0x07F0, 4, 0x0FF0, 4 },         /* 0x0800 x 0x0800 */
    
    { 0,  0x0008,   1024,   2047,      0,   2047, 0x08, 4, 0x07F8, 3, 0x07F0, 4 },         /* 0x0400 x 0x0800 */    
    { 0,  0x0009,      0,   1023,   1024,   2047, 0x09, 4, 0x03F8, 3, 0x07F8, 3 },         /* 0x0400 x 0x0400 */
    { 0,  0x000A,    512,   1023,      0,   1023, 0x0A, 4, 0x03FC, 2, 0x03F8, 3 },         /* 0x0200 x 0x0400 */
    { 0,  0x000B,      0,    511,    512,   1023, 0x0B, 4, 0x01FC, 2, 0x03FC, 2 },         /* 0x0200 x 0x0200 */    
    
    { 0,  0x000C,      0,    511,      0,    511, 0x0C, 4, 0x01FC, 2, 0x01FC, 2 },         /* 0x0200 x 0x0200 */

    { -1,  -1, 0, 0, 0, 0 },

};


static int32_t debug_enable = 0;


static const uint8 liblte_rrc_test_load[8] = {1,0,1,0,0,1,0,1};


/* End of LTE_ANALYSIS_Private_Variables */
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Exported_Variables
  * @{
  */
/* End of LTE_ANALYSIS_Exported_Variables */
/**
  * @}
  */

/* Private functions ------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Private_Functions
  * @{
  */


static void InitCodeInfoCnt(void)
{
    int32_t i;    
    for(i = 0; code_info_tbl_20_14[i].code >= 0; i++) {
        code_info_tbl_20_14[i].cnt = 0;
    }    
}

static void UpdateCodeInfoCnt(int32_t N_samples, Fixed_Sample_t *Sample_Data)
{
    int i;
    int j;

    int16_t abs_sample_re;
    int16_t abs_sample_im;

    for(i = 0; i < N_samples; i++) {
        
        abs_sample_re = abs(Sample_Data[i].i);
        abs_sample_im = abs(Sample_Data[i].q);
        for(j = 0; code_info_tbl_20_14[j].code >= 0; j++) {
            if(abs_sample_re >= code_info_tbl_20_14[j].real_min && abs_sample_re <= code_info_tbl_20_14[j].real_max &&
                abs_sample_im >= code_info_tbl_20_14[j].imag_min && abs_sample_im <= code_info_tbl_20_14[j].imag_max) {
                code_info_tbl_20_14[j].cnt += 1;
                break;
            }
        }

        if(code_info_tbl_20_14[j].code < 0) {
            fprintf(stderr, "coudn't find i = %d, re %d, im %d, %d, %d\n", i, abs_sample_re, abs_sample_im, Sample_Data[i].i, Sample_Data[i].q);
            for(j = 0; code_info_tbl_20_14[j].code >= 0; j++) {
                fprintf(stderr, "[%d] %d, %d, %d, %d\n", j, code_info_tbl_20_14[j].real_min,  code_info_tbl_20_14[j].real_max, code_info_tbl_20_14[j].imag_min, code_info_tbl_20_14[j].imag_max);

            }
        }

    }
}


static int32_t LTE_Analysis_Init(struct LTE_Analysis_s *ctx)
{

    /* Init random */
    ctx->fs = LIBLTE_PHY_FS_30_72MHZ;
    ctx->RefRmsLevel = REFERENCE_RMS_LEVEL;

    /* For Test */
    ctx->bandwidth = 20;
    ctx->N_rb = 100;
    ctx->N_sc_per_rb = 12;
    ctx->FFT_size = 2048;
    ctx->used_subcarriers = ctx->N_rb * ctx->N_sc_per_rb;
    ctx->FFT_pad_size = (ctx->FFT_size - ctx->used_subcarriers) / 2;
    ctx->Adc_bits = USE_ADC14;
    ctx->Dac_bits = ctx->Adc_bits;

    ctx->Symbol_in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) *ctx->FFT_size);
    ctx->Sample_in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);
    ctx->Normalized_in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);
    ctx->Sample_in_fft = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Adc_data = (Fixed_Sample_t *)malloc(sizeof(Fixed_Sample_t) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);
    ctx->Adc_error = (fftwf_complex *)malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Compress_data = (Compandor_t *)malloc(sizeof(Compandor_t) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);
    ctx->Compress_Error = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Decompress_data = (Fixed_Sample_t *)malloc(sizeof(Fixed_Sample_t) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Sample_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);
    ctx->Normalized_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);
    ctx->Sample_out_fft = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Symbol_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Sample_error = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->Symbol_error = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ);

    ctx->N_measure = (PDF_RANGE / PDF_STEP) + 2;
    ctx->Power_Histogram = (int32_t *)calloc(1, sizeof(int32_t) * ctx->N_measure);
    ctx->Pdf_Of_Samples = (float *)calloc(1, sizeof(float) * ctx->N_measure);
    ctx->Ccdf_Of_Samples = (float *)calloc(1, sizeof(float) * ctx->N_measure);

    ctx->indata = NULL;
    ctx->outdata = NULL;


    return 0;
}


static int32_t LTE_Analysis_Fini(struct LTE_Analysis_s *ctx)
{

    fftwf_free(ctx->Symbol_in);
    fftwf_free(ctx->Sample_in);
    fftwf_free(ctx->Normalized_in);
    fftwf_free(ctx->Sample_in_fft);

    free(ctx->Adc_data);
    fftwf_free(ctx->Adc_error);

    free(ctx->Compress_data);
    fftwf_free(ctx->Compress_Error);

    free(ctx->Decompress_data);
    
    fftwf_free(ctx->Sample_out);
    fftwf_free(ctx->Normalized_out);
    fftwf_free(ctx->Sample_out_fft);

    fftwf_free(ctx->Symbol_out);

    fftwf_free(ctx->Sample_error);
    
    fftwf_free(ctx->Symbol_error);

    free(ctx->Power_Histogram);
    free(ctx->Pdf_Of_Samples);
    free(ctx->Ccdf_Of_Samples);

    if(ctx->indata == NULL) {
        free(ctx->indata);
    }

    if(ctx->outdata == NULL) {
        free(ctx->outdata);
    }

}


static int LTE_Analysis_Get_File_Len(char *file_name)
{
    int file_len = -1;


    FILE *fp;

    fp = fopen(file_name, "rb");
    if(fp != NULL) {
        fseek(fp, 0, SEEK_END);
        file_len = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        fclose(fp);
    }

    return file_len;
}


static int LTE_Analysis_Print_Usage(char *prgoname)
{
    fprintf(stdout, "Usage: %s\n", prgoname);
    fprintf(stdout, "Where options are:\n");
    fprintf(stdout, "  -i, --input inputfile       : input file\n");
    fprintf(stdout, "  -o, --output outputfile     : output file\n");
    fprintf(stdout, "  -m, --mod [bpsk, qpsk, 16qam, 64qam]\n");
    fprintf(stdout, "  -t, --test n\n");
    fprintf(stdout, "  -b, --bandwidth bw\n");
    fprintf(stdout, "  -c, --dascodetype n\n");
    fprintf(stdout, "  -a, --adcbit n\n");

    return 0;
}

static int LTE_Analysis_Process_Option(int argc, char *argv[], struct option *long_options, struct LTE_Analysis_s *ctx)
{

    int ch;
    int input_found = 0;
    int output_found = 0;
    int option_index = 0;

    ctx->input_file_name = NULL;
    ctx->output_file_name = NULL;

    while(1) {
        ch = getopt_long (argc, (char * const *)argv, "i:o:m:t:b:c:a:h", long_options, &option_index);
        if (ch == -1) {
            break;
        }

        switch(ch) {
        case _INPUT_:
            input_found = 1;
            ctx->input_file_name = (char *)calloc(sizeof(char), (strlen(optarg) + 1));
            if(ctx->input_file_name == NULL) {
                return -1;
            }
            strcpy(ctx->input_file_name, optarg);
            break;

        case _OUTPUT_:
            output_found = 1;
            ctx->output_file_name = (char *)calloc(sizeof(char), (strlen(optarg) + 1));
            if(ctx->output_file_name == NULL) {
                return -1;
            }
            strcpy(ctx->output_file_name, optarg);
            break;

        case _MOD_TYPE_:
            if(strcasecmp(optarg, "bpsk") == 0) {
                ctx->Modulation_Type = LIBLTE_PHY_MODULATION_TYPE_BPSK;
            }
            else if(strcasecmp(optarg, "qpsk") == 0) {
                ctx->Modulation_Type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
            }
            else if(strcasecmp(optarg, "16qam") == 0) {
                ctx->Modulation_Type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
            }
            else if(strcasecmp(optarg, "64qam") == 0) {
                ctx->Modulation_Type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
            }
            break;

        case _TEST_:
            ctx->test_code = atoi(optarg);
            break;

        case _BW_:
            ctx->bandwidth = atoi(optarg);
            break;

        case _DAS_CODE_:
            ctx->Code_type = atoi(optarg);
            break;
        
        case _DAS_ADC_BIT_:
            ctx->Adc_bits = atoi(optarg);
            break;

        case 'h':
            return -1;
        }
    }

    return 0;

}


static int LTE_Analysis_ReadInput(struct LTE_Analysis_s *ctx)
{

    FILE *infile;

    int infile_size;
    int indata_cnt;
    int read_size;
    int slot_cnt;
    int i;

    fftwf_complex *indata_tmp;

    infile_size = LTE_Analysis_Get_File_Len(ctx->input_file_name);
    if(infile_size < 0) {
        fprintf(stderr, "file error\n");
        return -1;
    }

    infile = fopen(ctx->input_file_name, "rb");
    if(infile == NULL) {
        fprintf(stderr, "open failed : %s\n", ctx->input_file_name);
        return -1;
    }

    fprintf(stderr, "infile_size = %d\n", infile_size);
    slot_cnt = (infile_size / sizeof(fftwf_complex)) / LIBLTE_PHY_N_SAMPS_PER_SLOT_30_72MHZ;
    infile_size = slot_cnt * sizeof(fftwf_complex) * LIBLTE_PHY_N_SAMPS_PER_SLOT_30_72MHZ;
    indata_cnt = infile_size / sizeof(fftwf_complex);

    fprintf(stderr, "slot_cnt = %d, modified infile_size = %d\n", slot_cnt, infile_size);
    fprintf(stderr, "indata_cnt = %d\n", indata_cnt);

    indata_tmp = (fftwf_complex *)calloc(1, sizeof(fftwf_complex) * indata_cnt);
    if(indata_tmp == NULL) {
        fprintf(stderr, "calloc failed\n");
        return -1;
    }

    /* Read File Data */
    fseek(infile, 0, SEEK_SET);

    read_size = fread(indata_tmp, 1, infile_size, infile);
    fprintf(stderr, "read_size = %d / %d\n", read_size, infile_size);
    if(infile_size != read_size) {
        return -1;
    }

    ctx->indata = indata_tmp;
    ctx->indata_cnt = indata_cnt;
    ctx->indata_pos = 0;

    return indata_cnt;

}



/* End of LTE_ANALYSIS_Private_Functions */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup LTE_ANALYSIS_Exported_Functions
  * @{
  */

int32_t LTE_Analysis_Init_Output(struct LTE_Analysis_s *ctx)
{
    fftwf_complex *outdata_tmp;    

    ctx->outdata_cnt = ctx->indata_cnt;
    ctx->outdata_pos = 0;
    ctx->outdata = (fftwf_complex *)calloc(1, sizeof(fftwf_complex) * ctx->outdata_cnt);
    if(ctx->outdata == NULL) {
        return -1;
    }

    return 0;
}

void LTE_Analysis_Write_Output(struct LTE_Analysis_s *ctx, int32_t N_samples, fftwf_complex *Output)
{
    int32_t copy_cnt;

    copy_cnt = N_samples;
    if((ctx->outdata_pos + copy_cnt) > ctx->outdata_cnt) {        
        copy_cnt = ctx->outdata_cnt - ctx->outdata_pos;
    }
    memcpy(&ctx->outdata[ctx->outdata_pos], Output, sizeof(fftwf_complex) * copy_cnt);
    ctx->outdata_pos += copy_cnt;
}

void LTE_Analysis_Write_Output_to_File(struct LTE_Analysis_s *ctx)
{

    FILE *fp;
    fp = fopen(ctx->output_file_name, "wb");
    if(fp != NULL) {
        fwrite(&ctx->outdata[0], 1, sizeof(fftwf_complex) * ctx->outdata_pos, fp);
        fclose(fp);
    }
}

struct LTE_Analysis_s *LTE_Analysis_GetContext(void)
{
    return &LTE_Analysis_Context;
}


int MakeBPSK(fftwf_complex *Symbol, uint8_t *Bits)
{
    int i;
    int j;
    int code_bit;

    float  one_over_sqrt_2  = 1/sqrt(2);
    const static int re_seq[] = {
        +1, -1
    };
    const static int im_seq[] = {
        +1, -1
    };

    code_bit = Bits[i];
    Symbol[0][0] = (float)re_seq[code_bit] * one_over_sqrt_2;
    Symbol[0][1] = (float)im_seq[code_bit] * one_over_sqrt_2;

    return 0;
}


int MakeQPSK(fftwf_complex *Symbol, uint8_t *Bits)
{
    int i;
    int code_bit;

    float  one_over_sqrt_2  = 1/sqrt(2);
    const static int re_seq[] = {
        +1, +1, -1, -1
    };
    const static int im_seq[] = {
        +1, -1, +1, -1
    };

    code_bit = 0;
    for(i = 0; i < 2; i++) {
        code_bit = (code_bit << 1) + Bits[i];
    }
    Symbol[0][0] = (float)re_seq[code_bit] * one_over_sqrt_2;
    Symbol[0][1] = (float)im_seq[code_bit] * one_over_sqrt_2;

    return 0;
}

int Make16QAM(fftwf_complex *Symbol, uint8_t *Bits)
{
    int i;
    int code_bit;
    float  one_over_sqrt_10 = 1/sqrt(10);

    const static int re_seq[] = {
        +1, +1, +3, +3, +1, +1, +3, +3, -1, -1, -3, -3, -1, -1, -3, -3
    };
    const static int im_seq[] = {
        +1, +3, +1, +3, -1, -3, -1, -3, +1, +3, +1, +3, -1, -3, -1, -3,
    };


    code_bit = 0;
    for(i = 0; i < 4; i++) {
        code_bit = (code_bit << 1) + Bits[i];
    }
    Symbol[0][0] = (float)re_seq[code_bit] * one_over_sqrt_10;
    Symbol[0][1] = (float)im_seq[code_bit] * one_over_sqrt_10;

    return 0;
}


int Make64QAM(fftwf_complex *Symbol, uint8_t *Bits)
{
    int i;
    int code_bit;
    float one_over_sqrt_42 = 1/sqrt(42);

    const static int re_seq[] = {
        +3, +3, +1, +1, +3, +3, +1, +1, +5, +5, +7, +7, +5, +5, +7, +7,
        +3, +3, +1, +1, +3, +3, +1, +1, +5, +5, +7, +7, +5, +5, +7, +7,
        -3, -3, -1, -1, -3, -3, -1, -1, -5, -5, -7, -7, -5, -5, -7, -7,
        -3, -3, -1, -1, -3, -3, -1, -1, -5, -5, -7, -7, -5, -5, -7, -7

    };
    const static int im_seq[] = {
        +3, +1, +3, +1, +5, +7, +5, +7, +3, +1, +3, +1, +5, +7, +5, +7,
        -3, -1, -3, -1, -5, -7, -5, -7, -3, -1, -3, -1, -5, -7, -5, -7,
        +3, +1, +3, +1, +5, +7, +5, +7, +3, +1, +3, +1, +5, +7, +5, +7,
        -3, -1, -3, -1, -5, -7, -5, -7, -3, -1, -3, -1, -5, -7, -5, -7
    };

    code_bit = 0;
    for(i = 0; i < 6; i++) {
        code_bit = (code_bit << 1) + Bits[i];
    }

    Symbol[0][0] = (float)re_seq[code_bit] * one_over_sqrt_42;
    Symbol[0][1] = (float)im_seq[code_bit] * one_over_sqrt_42;

    return 0;

}


void MakeRandomBits(int32_t N_Code_Bits, uint8_t *Code_Bits)
{
    int i;
    for(i = 0; i < N_Code_Bits; i++) {
        Code_Bits[i] = rand() & 0x01;
    }
}

int32_t GetBitPerSymbol(int32_t Modulation_Type)
{
    int32_t Bit_per_symbol;
    
    switch(Modulation_Type) {
    case LIBLTE_PHY_MODULATION_TYPE_BPSK:
        Bit_per_symbol = 1;
        break;
    case LIBLTE_PHY_MODULATION_TYPE_QPSK:
        Bit_per_symbol = 2;
        break;

    case LIBLTE_PHY_MODULATION_TYPE_16QAM:
        Bit_per_symbol = 4;
        break;

    case LIBLTE_PHY_MODULATION_TYPE_64QAM:
        Bit_per_symbol = 6;
        break;
    }

    return Bit_per_symbol;

}


int MakeSymbol(int32_t N_Symbols, fftwf_complex *Symbol, uint8_t *Code_bits, int32_t Bit_per_Symbol)
{
    /* Mode : QPSK, 16QAM, 64QAM */

    int32_t i;

    for(i = 0; i < N_Symbols; i++) {
        switch(Bit_per_Symbol) {
        case 1: /* LIBLTE_PHY_MODULATION_TYPE_BPSK:  */
            MakeBPSK(&Symbol[i], &Code_bits[i * Bit_per_Symbol]);
            break;

        case 2: /* LIBLTE_PHY_MODULATION_TYPE_QPSK: */
            MakeQPSK(&Symbol[i], &Code_bits[i * Bit_per_Symbol]);
            break;

        case 4: /* LIBLTE_PHY_MODULATION_TYPE_16QAM: */
            Make16QAM(&Symbol[i], &Code_bits[i * Bit_per_Symbol]);
            break;

        case 6: /* LIBLTE_PHY_MODULATION_TYPE_64QAM: */
            Make64QAM(&Symbol[i], &Code_bits[i * Bit_per_Symbol]);
            break;
        }
    }

}


int _FFT(int32_t FFT_size, fftwf_complex *Output, fftwf_complex *Input)
{
    fftwf_plan PlanFFT;
    PlanFFT = fftwf_plan_dft_1d(FFT_size, Input, Output, FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(PlanFFT);
    fftwf_destroy_plan(PlanFFT);
}

int _IFFT(int32_t FFT_size, fftwf_complex *Output, fftwf_complex *Input)
{
    fftwf_plan PlanFFT;
    PlanFFT = fftwf_plan_dft_1d(FFT_size, Input, Output, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftwf_execute(PlanFFT);
    fftwf_destroy_plan(PlanFFT);
}

int32_t ShiftSymbolforIFFT(int32_t N_output, fftwf_complex *Output, int32_t N_input, fftwf_complex *Input)
{
    int32_t i;
    int32_t In_center = N_input / 2;

    for(i = 0; i < N_output; i++) {
        Output[i][0] = 0;
        Output[i][1] = 0;
    }

    for(i = 0; i < In_center; i++) {
        // Positive spectrum
        Output[i+1][0] = Input[In_center+i][0];
        Output[i+1][1] = Input[In_center+i][1];

        // Negative spectrum
        Output[N_output-i-1][0] = Input[In_center-i-1][0];
        Output[N_output-i-1][1] = Input[In_center-i-1][1];
    }    

    return 0;

}

void ShiftFFT(int32_t FFT_Size, fftwf_complex *FFT_Data)
{

    int32_t i;
    int32_t FFT_Center = FFT_Size / 2;

    fftwf_complex *FFT_temp;

    FFT_temp = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * FFT_Size);

    for(i = 0; i < FFT_Center; i++) {        
        FFT_temp[FFT_Center + i][0] = FFT_Data[i][0];
        FFT_temp[FFT_Center + i][1] = FFT_Data[i][1];

        FFT_temp[(FFT_Center-1) - i][0] = FFT_Data[(FFT_Size- 1)-i][0];
        FFT_temp[(FFT_Center-1) - i][1] = FFT_Data[(FFT_Size- 1)-i][1];
    }

    memcpy(FFT_Data, FFT_temp, sizeof(fftwf_complex) * FFT_Size);
    
    fftwf_free(FFT_temp);

}


int32_t Symbol2Sample(int32_t N_Samples, fftwf_complex *Sample, int32_t N_symbols, fftwf_complex *Symbol)
{
    int32_t i;
    int32_t Symbol_center = N_symbols / 2;

    fftwf_complex *Symbol_in;

    Symbol_in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * N_Samples);
    ShiftSymbolforIFFT(N_Samples, Symbol_in, N_symbols, Symbol);
    _IFFT(N_Samples, Sample, Symbol_in);
    
    fftwf_free(Symbol_in);

    return 0;
}

int32_t Sample2Symbol(int32_t N_symbols, fftwf_complex *Symbol, int32_t N_Samples, fftwf_complex *Sample)
{
    int32_t i;
    int32_t Symbol_center = N_symbols / 2;
    int32_t FFT_size;

    fftwf_complex *Symbol_tmp;

    Symbol_tmp = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * N_Samples);

    FFT_size = N_Samples;
    _FFT(FFT_size, Symbol_tmp, Sample);   
    GainBlock(FFT_size, Symbol_tmp, Symbol_tmp, 1.0 / (float)FFT_size);

    for(i = 0; i < Symbol_center; i++) {
        // Positive spectrum
        Symbol[Symbol_center+i][0] = Symbol_tmp[i+1][0];
        Symbol[Symbol_center+i][1] = Symbol_tmp[i+1][1];

        // Negative spectrum]
        Symbol[Symbol_center-i-1][0] = Symbol_tmp[N_Samples-i-1][0];
        Symbol[Symbol_center-i-1][1] = Symbol_tmp[N_Samples-i-1][1];
    }  

    fftwf_free(Symbol_tmp);

    return 0;
}


int32_t NormalizeSample(int32_t N_samples, fftwf_complex *Samples, float Gain)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        Samples[i][0] = Samples[i][0] * Gain;
        Samples[i][1] = Samples[i][1] * Gain;
    }

    return 0;
}


int32_t GainBlock(int32_t N_samples, fftwf_complex *Output, fftwf_complex *Input, float Gain)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        Output[i][0] = Input[i][0] * Gain;
        Output[i][1] = Input[i][1] * Gain;
    }

    return 0;
}

float Calculate_Rms(int32_t N, fftwf_complex *Input)
{
    int i;
    float Sample_sum;
    float Sample_rms;

    Sample_rms = -1.0;
    if(N > 0) {
        Sample_sum = 0.0;
        for(i = 0; i < N; i++) {
            Sample_sum += Input[i][0] * Input[i][0] + Input[i][1] * Input[i][1];
        }
        Sample_rms = sqrt(Sample_sum / N);
    }

    return Sample_rms;
}





float GetSamplePeak(int32_t N_samples, fftwf_complex *Sample)
{
    int i;
    float sample_rms;
    float sample_max;

    sample_max = -1.0;
    if(N_samples > 0) {
        for(i = 0; i < N_samples; i++) {
            sample_rms = sqrt(Sample[i][0] * Sample[i][0] + Sample[i][1] * Sample[i][1]);
            if(sample_rms > sample_max) {
                sample_max = sample_rms;
            }
        }
    }
    
    return sample_max;
}

uint32_t Merge_IQ_Data(Fixed_Sample_t *Input)
{

    int32_t i;

    uint16_t i_data;
    uint16_t q_data;   
    uint16_t mask_bit;
    uint32_t iq_data;
    
    iq_data = 0;
    if(Input->i < 0) {
        iq_data |= COMPANDOR_I_SIGN;
    }
    i_data = abs(Input->i);
    
    if(Input->q < 0) {
        iq_data |= COMPANDOR_Q_SIGN;
    }
    q_data = abs(Input->q);

    mask_bit = 0x4000;    
    for(i = 0; i < 15; i++) {   
        iq_data = (iq_data << 1);
        if(i_data & mask_bit) {
            iq_data = iq_data | 1;
        }
        iq_data = (iq_data << 1);
        if(q_data & mask_bit) {
            iq_data = iq_data | 1;
        }
        mask_bit = mask_bit >> 1;
    }

#if 0
    fprintf(stderr, "\n");
    PrintBits(16, (uint32_t)i_data);
    PrintBits(16, (uint32_t)q_data);
    PrintBits(32, (uint32_t)iq_data);
#endif    

    return iq_data;

}



void ConvertBitString(int32_t N_bits, char *BitString, uint32_t Data)
{
    int32_t i;
    char LSBFirstString[64];
    uint32_t mask;
    int32_t bit_pos;

    mask = 0x0001;
    bit_pos = 0;
    for(i = 0; i < N_bits; i++) {       
        if(Data & mask) {
            LSBFirstString[bit_pos] = '1';
        }
        else {
            LSBFirstString[bit_pos] = '0';
        }  
        mask = mask << 1;
        bit_pos++;
        if((i % 4) == 3) {
            LSBFirstString[bit_pos] = ' ';
            bit_pos++;
        }      
    }

    if((N_bits % 4) == 0) {
        bit_pos--;
    }   

    for(i = 0; i < bit_pos; i++) {
        BitString[i] = LSBFirstString[(bit_pos - 1) - i];
    }

    BitString[i] = 0;

}


void print_rms_in_dB(float *data_buff, int data_len)
{
    int i;

    /* Print */
    for(i = 0; i < data_len; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %6.2f", i, data_buff[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %6.2f\n", data_buff[i]);
        }
        else {
            fprintf(stderr, " %6.2f", data_buff[i]);
        }
    }
    if(i % LIBLTE_PRINT_ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }    
}

void print_Encoded_Sample(int32_t Code_type, int32_t N_samples, Compandor_t *Sample)
{
    int32_t i;

    if(Code_type == DAS_CODE_16BIT || Code_type == DAS_CODE_14BIT || Code_type == DAS_CODE_12BIT) {

        for(i = 0; i < N_samples; i++) {
            if((i % ITEM_PER_LINE) == 0) {
                fprintf(stderr, "[%04d] [%5d,%5d]", i, Sample[i].u.d.i, Sample[i].u.d.q);
            }
            else if(((i + 1) % ITEM_PER_LINE) == 0) {
                fprintf(stderr, " [%5d,%5d]\n", Sample[i].u.d.i, Sample[i].u.d.q);
            }
            else {
                fprintf(stderr, " [%5d,%5d]", Sample[i].u.d.i, Sample[i].u.d.q);
            }
        }
        if(i % ITEM_PER_LINE) {
            fprintf(stderr, "\n");
        }
    }

}


void Print_Code_bits(int32_t N_Symbols, uint8_t *Code_bits, int32_t Bits_per_symbol)
{
    int32_t i;
    int32_t j;
    int32_t Code_bit;

    fprintf(stderr, "sample_code_bits = [\n");

    for(i = 0; i < N_Symbols; i++) {
        Code_bit = 0;
        for(j = 0; j < Bits_per_symbol; j++) {
            Code_bit = (Code_bit << 1) + Code_bits[i * Bits_per_symbol + j];
        }
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "    %2d,", Code_bit);
        }
        else {
            fprintf(stderr, " %2d,", Code_bit);
        }

        if(((i + 1) % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "\n");
        }
    }

    fprintf(stderr, "     0\n]\n");

}


void print_Code(int N_samples, fftwf_complex *sample)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%7.2f,%7.2f]", i, sample[i][0], sample[i][1]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%7.2f,%7.2f]\n", sample[i][0], sample[i][1]);
        }
        else {
            fprintf(stderr, " [%7.2f,%7.2f]", sample[i][0], sample[i][1]);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}



void print_Sample(int N_samples, fftwf_complex *sample)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%7.2f,%7.2f]", i, sample[i][0], sample[i][1]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%7.2f,%7.2f]\n", sample[i][0], sample[i][1]);
        }
        else {
            fprintf(stderr, " [%7.2f,%7.2f]", sample[i][0], sample[i][1]);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}

void print_Sample_Normalized(int N_samples, fftwf_complex *Sample)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%7.4f,%7.4f]", i, Sample[i][0], Sample[i][1]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%7.4f,%7.4f]\n", Sample[i][0], Sample[i][1]);
        }
        else {
            fprintf(stderr, " [%7.4f,%7.4f]", Sample[i][0], Sample[i][1]);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}

void print_Fixed_Data(int N_samples, Fixed_Sample_t *Data)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%6d,%6d]", i, Data[i].i, Data[i].q);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%6d,%6d]\n", Data[i].i, Data[i].q);
        }
        else {
            fprintf(stderr, " [%6d,%6d]", Data[i].i, Data[i].q);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}


void print_Compandor(int N_samples, Compandor_t *Compressed_data)
{
    int32_t i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%7.1f,%7.1f]", i, (float)Compressed_data[i].u.d.i, (float)Compressed_data[i].u.d.q);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%7.1f,%7.1f]\n", (float)Compressed_data[i].u.d.i, (float)Compressed_data[i].u.d.q);
        }
        else {
            fprintf(stderr, " [%7.1f,%7.1f]", (float)Compressed_data[i].u.d.i, (float)Compressed_data[i].u.d.q);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}

void Print_Adc_Bits(int32_t N_samples, Fixed_Sample_t *Adc)
{
    char I_BitsString[64];
    char Q_BitsString[64];
    char IQ_BitsString[64];

    int32_t i;
    uint16_t i_data;
    uint16_t q_data;
    uint16_t iq_data;

    for(i = 0; i < N_samples; i++) {

        uint16_t i_data = abs(Adc[i].i);
        uint16_t q_data = abs(Adc[i].q);
        uint32_t iq_data = Merge_IQ_Data(&Adc[i]);
        
        ConvertBitString(16, I_BitsString, i_data);
        ConvertBitString(16, Q_BitsString, q_data);
        ConvertBitString(32, IQ_BitsString, iq_data);
        fprintf(stderr, "%s\n", I_BitsString);
        fprintf(stderr, "%s\n", Q_BitsString);
        fprintf(stderr, "%s\n", IQ_BitsString);
    }
}

void Print_Compress_Data(int32_t Code_type, int32_t N_samples, Compandor_t *Compandor)
{
    int32_t i;
    switch(Code_type)
    {
    case DAS_CODE_RAW:
        break;

    case DAS_CODE_IQ_20_4:
        for(i = 0; i < N_samples; i++) {
            char BitString[64];  
            ConvertBitString(2, BitString, Compandor[i].u.c.sign);
            fprintf(stderr, "(%s", BitString);   
            ConvertBitString(4, BitString, Compandor[i].u.c.first_1_pos);
            fprintf(stderr, ", %s", BitString);   
            ConvertBitString(14, BitString, Compandor[i].u.c.iq_code);
            fprintf(stderr, ", %s)\n", BitString);
        }    
        break;
        
    case DAS_CODE_IQ_20_3:
        for(i = 0; i < N_samples; i++) {
            char BitString[64]; 
            ConvertBitString(2, BitString, Compandor[i].u.c.sign);
            fprintf(stderr, "(%s", BitString);   
            ConvertBitString(3, BitString, Compandor[i].u.c.first_1_pos);
            fprintf(stderr, ", %s", BitString);   
            ConvertBitString(15, BitString, Compandor[i].u.c.iq_code);
            fprintf(stderr, ", %s)\n", BitString);
        }    
        break;

    }
}

void print_Encode_Error(int N_samples, fftwf_complex *Error)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%7.2f,%7.2f]", i, Error[i][0], Error[i][1]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%7.2f,%7.2f]\n", Error[i][0], Error[i][1]);
        }
        else {
            fprintf(stderr, " [%7.2f,%7.2f]", Error[i][0], Error[i][1]);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}


void print_Complex_Number(int N_samples, fftwf_complex *Data)
{
    int i;

    for(i = 0; i < N_samples; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%7.4f,%7.4f]", i, Data[i][0], Data[i][1]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%7.4f,%7.4f]\n", Data[i][0], Data[i][1]);
        }
        else {
            fprintf(stderr, " [%7.4f,%7.4f]", Data[i][0], Data[i][1]);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}



void print_PDF(int32_t N_measure, float *Pdf)
{
    int i;

    /* Print */
    for(i = 0; i < N_measure; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %6.4f", i, Pdf[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %6.4f\n", Pdf[i]);
        }
        else {
            fprintf(stderr, " %6.4f", Pdf[i]);
        }
    }
}


void print_CCDF(int32_t N_measure, float *Ccdf)
{
    int i;

    /* Print */
    for(i = 0; i < N_measure; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %6.3f", i, Ccdf[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %6.3f\n", Ccdf[i]);
        }
        else {
            fprintf(stderr, " %6.3f", Ccdf[i]);
        }
    }
}

void print_CodeInfo(void)
{
    int32_t i;
    
    for(i = 0; code_info_tbl_20_14[i].code >= 0; i++) {

        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [0x%04X, %5d]", i, code_info_tbl_20_14[i].code, code_info_tbl_20_14[i].cnt);
        }        
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [0x%04X, %5d]\n", code_info_tbl_20_14[i].code, code_info_tbl_20_14[i].cnt);
        }
        else {
            fprintf(stderr, " [0x%04X, %5d]", code_info_tbl_20_14[i].code, code_info_tbl_20_14[i].cnt);
        }
    }
    if(i % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }


}

void print_Histogram(int32_t N_measure, int32_t *Histogram)
{

    int i;

    /* Print */
    for(i = 0; i < N_measure; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %5d", i, Histogram[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %5d\n", Histogram[i]);
        }
        else {
            fprintf(stderr, " %5d", Histogram[i]);
        }
    }

}

char *rnti_to_str(uint16 rnti)
{

    if(rnti == LIBLTE_MAC_SI_RNTI) {
        return (char *)"SI_RNTI";
    }
    if(rnti == LIBLTE_MAC_P_RNTI) {
        return (char *)"P_RNTI";
    }
    if(rnti == LIBLTE_MAC_M_RNTI) {
        return (char *)"M_RNTI";
    }    
    if(rnti >= LIBLTE_MAC_RA_RNTI_START && rnti <= LIBLTE_MAC_RA_RNTI_END) {
        return (char *)"RA_RNTI";
    }    
    if(rnti >= LIBLTE_MAC_C_RNTI_START && rnti <= LIBLTE_MAC_C_RNTI_END) {
        return (char *)"C_RNTI";
    }  

    return (char *)"unknown";
}

int UpdatePowerHistogram(int N_samples, fftwf_complex *Sample, float Rms_ref_db, int32_t *Pwr_histogram)
{
    int i;
    int index;
    float Sample_power;
    float Sample_power_db;

    for(i = 0; i < N_samples; i++) {
        Sample_power = Sample[i][0] * Sample[i][0] + Sample[i][1] * Sample[i][1];
        if(Sample_power > 0) {
            Sample_power_db = 10.0 * log10(Sample_power);
        }
        else {
            Sample_power_db = MIN_DB_VALUE;
        }
        if(Sample_power_db >= Rms_ref_db) {
            if(Sample_power_db > (Rms_ref_db + PDF_RANGE)) {
                Sample_power_db = Rms_ref_db + PDF_RANGE;
            }
            index = (int)((Sample_power_db -  Rms_ref_db) / PDF_STEP) + 1;                        
        }        
        else {
            index = 0;
        }
        Pwr_histogram[index] += 1;
    }

    return 0;
    
}

int CalculatePDF(int32_t N_measure, float *Pdf, int32_t *Pwr_histogram, int32_t N_Total_samples)
{
    int i;
    for(i = 0; i < N_measure; i++) {
        Pdf[i] = (float)Pwr_histogram[i] / (float)N_Total_samples;
    }
    return 0;
}

int CalculateCCDF(int32_t N_measure, float *Ccdf, float *Pdf)
{
    int i;
    float _ccdf;
    int index;

    _ccdf = 0.0;
    for(i = N_measure - 1; i >= 0; i--) {
        _ccdf += Pdf[i];
        Ccdf[i] = _ccdf * 100.0;
    }

    return 0;
}


int32_t LTE_Analysis_Copy_Input_to_Samplebuff(struct LTE_Analysis_s *ctx, int32_t N_req_items)
{
    int32_t i;

    int32_t ninput_items;

    ninput_items = ctx->indata_cnt - ctx->indata_pos;
    if(ninput_items > N_req_items) {
        ninput_items = N_req_items;
    }

    if(ninput_items > 0) {
        for(i = 0; i < ninput_items; i++) {
            ctx->Sample_in[i][0] = ctx->indata[ctx->indata_pos + i][0];
            ctx->Sample_in[i][1] = ctx->indata[ctx->indata_pos + i][1];
        }
        ctx->indata_pos += ninput_items;
        ctx->N_samples = ninput_items;
    }

    return ninput_items;

}

void Adc_1_sample(int32_t N_bits, int16_t *Output, float *Error, float *Input)
{
    int32_t step;
    float input_data;
    int32_t quantized_data;
    
    step = 1 << (16 - N_bits);

    input_data = *Input;    
    if(input_data >= 0) { 
        quantized_data = (int32_t)(MAX_ADC_VALUE * input_data) / step * step;  
        if(quantized_data >= MAX_ADC_VALUE) {
            quantized_data = MAX_ADC_VALUE - step;
        }
        *Output = (int16_t)quantized_data;
        *Error = (MAX_ADC_VALUE * input_data) - (float)quantized_data;            
    }
    else {
        // fprintf(stderr, "input_data = %.4f, step = %d\n", input_data, step);
        // fprintf(stderr, "input_data = %d\n", (int32_t)(MAX_ADC_VALUE * (-input_data)));        
        quantized_data = (int32_t)(MAX_ADC_VALUE * (-input_data)) / step * step;  
        if(quantized_data > MAX_ADC_VALUE) {
            quantized_data = MAX_ADC_VALUE;
        }
        *Output = (int16_t)(-quantized_data);
        *Error = (MAX_ADC_VALUE * input_data) - (float)(-quantized_data);
    }   
    
}



void Dac_1_Sample(int32_t N_bits, float *O_data, int16_t *I_data)
{
    *O_data = (float)(*I_data) / MAX_ADC_VALUE;
}

void Calculate_Sample_Error(int32_t N_samples, fftwf_complex *Error, fftwf_complex *Output, fftwf_complex *Input)
{
    int32_t i;
    for(i = 0; i < N_samples; i++) {
        Error[i][0] = Input[i][0] - Output[i][0];
        Error[i][1] = Input[i][1] - Output[i][1];
    }
}

void Adc_N_Samples(int32_t Adc_bits, int32_t N_samples, Fixed_Sample_t *Adc_Output, fftwf_complex *Adc_error, fftwf_complex *Adc_Input)
{
    int32_t i;    
    for(i = 0; i < N_samples; i++) {
        Adc_1_sample(Adc_bits, &Adc_Output[i].i, &Adc_error[i][0], &Adc_Input[i][0]);
        Adc_1_sample(Adc_bits, &Adc_Output[i].q, &Adc_error[i][1], &Adc_Input[i][1]);
    }        
}

void Dac_N_Samples(int32_t Dac_bits, int32_t N_samples, fftwf_complex *Dac_output, Fixed_Sample_t *Dac_input)
{
    int32_t i;       

    for(i = 0; i < N_samples; i++) {
        Dac_1_Sample(Dac_bits, &Dac_output[i][0], &Dac_input[i].i);
        Dac_1_Sample(Dac_bits, &Dac_output[i][1], &Dac_input[i].q);
    }            
}



void PrintBits(int32_t N_bits, uint32_t Data)
{
    char BitsString[64];

    ConvertBitString(N_bits, BitsString, Data);

    fprintf(stderr, "%s\n", BitsString);

}

void Sepeate_IQ_Data(Fixed_Sample_t *Output, uint32_t iq_data)
{
#if 0
    char BitString[64];
#endif
    int32_t i;
    
    uint16_t i_data;
    uint16_t q_data;   
    uint16_t sign_bits;
    uint32_t mask_bit;

    sign_bits = (iq_data & COMPANDOR_SIGN_MASK) >> COMPANDOR_SIGN_SHIFT;
#if 0
    ConvertBitString(2, BitString, sign_bits);
    fprintf(stderr, "sign_bits = %s\n", BitString);
#endif
    i_data = 0;
    q_data = 0;
    mask_bit = 0x20000000;
    
    for(i = 0; i < 15; i++) { 
        i_data = i_data << 1; 
        if(iq_data & mask_bit) {
            i_data |= 1; 
        }
        mask_bit = mask_bit >> 1;

        q_data = (q_data << 1); 
        if(iq_data & mask_bit) {
            q_data |= 1; 
        }
        mask_bit = mask_bit >> 1;
    }

#if 0
    ConvertBitString(16, BitString, i_data);
    fprintf(stderr, "i_data = %s\n", BitString);

    ConvertBitString(16, BitString, q_data);
    fprintf(stderr, "q_data = %s\n", BitString);
#endif

    Output->i = i_data;
    if(sign_bits & COMPANDOR_I_SIGN) {
        Output->i = -Output->i;
    }

    Output->q = q_data;
    if(sign_bits & COMPANDOR_Q_SIGN) {
        Output->q = -Output->q;
    } 

#if 0
    ConvertBitString(16, BitString, Output->i);
    fprintf(stderr, "Output->i = %s\n", BitString);

    ConvertBitString(16, BitString, Output->q);
    fprintf(stderr, "Output->q = %s\n", BitString);

#endif

}


void Compress_N_M(int32_t N, int32_t M, Compandor_t *Output, Fixed_Sample_t *Input)
{

    int32_t first_1_pos;
    uint32_t mask_bit = COMPANDOR_1_BIT_MASK;    
    uint32_t iq_data = Merge_IQ_Data(Input);    
    int32_t shift_max;
    int32_t search_max;
    uint32_t iq_mask;
    
    Output->u.c.n_bits = N;
    Output->u.c.sign = (iq_data & COMPANDOR_SIGN_MASK) >> COMPANDOR_SIGN_SHIFT;
  
#if 0
    fprintf(stderr, "\n");
    ConvertBitString(32, BitString, iq_data);
    fprintf(stderr, "iq_data = %s\n", BitString);
    
    ConvertBitString(2, BitString, Output->u.c.sign);
    fprintf(stderr, "Output->u.c.sign = %s\n", BitString);   
#endif

    search_max = (1 << M) - 1;
    iq_mask = (1 << (N - 2 - M)) - 1;
    shift_max = (30 - (N - M) + 1);
    
#if 0
    fprintf(stderr, "search_max = 0x%08X\n", iq_mask);
    fprintf(stderr, "search_max = %d\n", search_max);
    fprintf(stderr, "shift_max = %d\n", shift_max);
#endif

    
    for(first_1_pos = 0; first_1_pos < search_max; first_1_pos++) {
        if(mask_bit & iq_data) {       
            /* 0 ~ 14 */                
            Output->u.c.first_1_pos = first_1_pos;
            Output->u.c.iq_code = (iq_data >> (shift_max - first_1_pos)) & iq_mask;
#if 0        
            fprintf(stderr, "[1]first_1_pos = %d\n", first_1_pos);
            ConvertBitString(14, BitString, Output->u.c.iq_code);
            fprintf(stderr, "iq_code = %s\n", BitString);
#endif        
            
            break;
        } 
        mask_bit = mask_bit >> 1;  
    }

    if(search_max == first_1_pos) {    
        Output->u.c.first_1_pos = first_1_pos;
        Output->u.c.iq_code = (iq_data >> (shift_max - search_max + 1)) & iq_mask;

#if 0
        fprintf(stderr, "[2]first_1_pos = %d\n", first_1_pos);        
        ConvertBitString(14, BitString, Output->u.c.iq_code);
        fprintf(stderr, "iq_code = %s\n", BitString);
#endif        
        
    }

}

void Decompress_N_M(int32_t N, int32_t M, Fixed_Sample_t *Output, Compandor_t *Input)
{

#if 0    
        char BitString[64];
#endif

    int32_t first_1_pos;

    uint32_t iq_mask;
    int32_t shift_max;
    int32_t search_max;
    uint32_t mask_bit = COMPANDOR_1_BIT_MASK;    
    uint32_t iq_data;

    iq_mask = (1 << (N - 2 - M)) - 1;
    shift_max = (30 - (N - M) + 1);
    search_max = (1 << M) - 1;

    first_1_pos = Input->u.c.first_1_pos;

    /* Sign Bits */
    iq_data = ((uint32_t)Input->u.c.sign << COMPANDOR_SIGN_SHIFT) & COMPANDOR_SIGN_MASK;
    if(first_1_pos == search_max) {
        iq_data |= (Input->u.c.iq_code & iq_mask) << (shift_max - first_1_pos + 1) ;
    }
    else {
        iq_data = iq_data | (COMPANDOR_1_BIT_MASK >> first_1_pos);
        iq_data = iq_data | ((Input->u.c.iq_code & iq_mask) << (shift_max - first_1_pos));        
    }

#if 0
    ConvertBitString(32, BitString, iq_data);
    fprintf(stderr, "%s\n", BitString);  
#endif    

    Sepeate_IQ_Data(Output, iq_data);

#if 0
    ConvertBitString(16, BitString, Output->i);
    fprintf(stderr, "Output->i = %s\n", BitString);

    ConvertBitString(16, BitString, Output->q);
    fprintf(stderr, "Output->q = %s\n", BitString);
#endif

}

#define SWAP_UINT16_T(a, b) \
do { \
    uint16_t temp; \
    temp = a; \
    a = b; \
    b = temp; \
} while(0)



void Swap_IQ_N_M_X(int32_t N, int32_t M, Fixed_Sample_t *Output, Fixed_Sample_t *Input)
{

#if 1
    char I_Bitstring[64];
    char Q_Bitstring[64];
    char IQ_Bitstring[64];
#endif    
    
    int32_t max_srch_pos;
    
    uint16_t i;
    uint16_t q;
    uint16_t sign_i;
    uint16_t sign_q;

    uint16_t temp;  
    
    uint16_t mask_2_bit_i;
    uint16_t mask_2_bit_q;
    
    uint16_t mask_2_bit = 0x6000;       /* 0110 0000 0000 0000 */
    uint16_t mask_1_bit = 0x2000;       /* 0010 0000 0000 0000 */
    int16_t shift_bit;
    int16_t check_pos;

    shift_bit = 13;
       
    i = abs(Input->i);
    q = abs(Input->q);    
    sign_i = (Input->i < 0) ? 1 : 0;
    sign_q = (Input->q < 0) ? 1 : 0;

#if 0
    if(debug_enable) {
        ConvertBitString(16, I_Bitstring, Input->i);
        ConvertBitString(16, Q_Bitstring, Input->q);
        ConvertBitString(32, IQ_Bitstring, Merge_IQ_Data(Input));
        fprintf(stderr, "[I]  %s - %s - %s\n", I_Bitstring, Q_Bitstring, IQ_Bitstring);  

        ConvertBitString(16, I_Bitstring, i);
        ConvertBitString(16, Q_Bitstring, q);
        fprintf(stderr, "     %s - %s\n", I_Bitstring, Q_Bitstring);  
    }
#endif

    max_srch_pos = 1 << (M - 1);
    for(check_pos = 0; check_pos < max_srch_pos; check_pos++) {        

        mask_2_bit_i = (i & mask_2_bit) >> shift_bit;
        mask_2_bit_q = (q & mask_2_bit) >> shift_bit;    
    
        if(mask_2_bit_i >= 0x02 || mask_2_bit_q >= 0x02) {   
            
            if(mask_2_bit_i == 0x02 && mask_2_bit_q < 0x02) {
                i = i | mask_1_bit;       
                SWAP_UINT16_T(i, q);
                SWAP_UINT16_T(sign_i, sign_q);
#if 0          
                if(debug_enable) {
                    ConvertBitString(16, I_Bitstring, i);
                    ConvertBitString(16, Q_Bitstring, q);
                    fprintf(stderr, "[1]  %s - %s\n", I_Bitstring, Q_Bitstring);  
                }
#endif              
            }
            else if(mask_2_bit_q == 0x03 &&  mask_2_bit_i < 0x02) {
                q = q & ~mask_1_bit;              
                SWAP_UINT16_T(i, q);
                SWAP_UINT16_T(sign_i, sign_q);
#if 0
                if(debug_enable) {
                    ConvertBitString(16, I_Bitstring, i);
                    ConvertBitString(16, Q_Bitstring, q);
                    fprintf(stderr, "[2]  %s - %s\n", I_Bitstring, Q_Bitstring); 
                }
#endif
            }
            
            if(sign_i) {
                Output->i = -((int16_t)i);
            }
            else {
                Output->i = i;
            }            
            if(sign_q) {
                Output->q = -((int16_t)q);
            }
            else {
                Output->q = q;
            }           
            break;

        }

        mask_2_bit = mask_2_bit >> 1;
        mask_1_bit = mask_1_bit >> 1;       
        shift_bit = shift_bit - 1;
        
    }

#if 0  
    if(debug_enable) {
        ConvertBitString(16, I_Bitstring, Output->i);
        ConvertBitString(16, Q_Bitstring, Output->q);
        ConvertBitString(32, IQ_Bitstring, Merge_IQ_Data(Output));
        fprintf(stderr, "     %s - %s - %s\n", I_Bitstring, Q_Bitstring, IQ_Bitstring);  
    }
#endif

}

void Compress_N_M_X(int32_t N, int32_t M, Compandor_t *Output, Fixed_Sample_t *Input)
{
    Fixed_Sample_t Temp;

    Swap_IQ_N_M_X(N, M, &Temp, Input);
    Compress_N_M(N, M, Output, &Temp);
}


void Decompress_N_M_X(int32_t N, int32_t M, Fixed_Sample_t *Output, Compandor_t *Input)
{
    char I_Bitstring[64];
    char Q_Bitstring[64];
    char IQ_Bitstring[64];

    Fixed_Sample_t Temp;    


#if 0  
    if(debug_enable) {
        fprintf(stderr, "\n");
        fprintf(stderr, "Decompress_N_M_X\n");
    }
#endif

    Decompress_N_M(N, M, &Temp, Input);  

#if 0  
    if(debug_enable) {
        ConvertBitString(16, I_Bitstring, Temp.i);
        ConvertBitString(16, Q_Bitstring, Temp.q);
        ConvertBitString(32, IQ_Bitstring, Merge_IQ_Data(&Temp));
        fprintf(stderr, "     %s - %s - %s\n", I_Bitstring, Q_Bitstring, IQ_Bitstring); 
    }
#endif

    Swap_IQ_N_M_X(N, M, Output, &Temp);

#if 0  
    if(debug_enable) {
        ConvertBitString(16, I_Bitstring, Output->i);
        ConvertBitString(16, Q_Bitstring, Output->q);
        ConvertBitString(32, IQ_Bitstring, Merge_IQ_Data(Output));
        fprintf(stderr, "=>   %s - %s - %s\n", I_Bitstring, Q_Bitstring, IQ_Bitstring);  
        fprintf(stderr, "Decompress_N_M_X DONE\n"); 
    }
#endif

}


void Compress_Huffman_20_14(Compandor_t *Output, Fixed_Sample_t *Input)
{

    code_info_t *code_info;
    
    uint16_t i_data;
    uint16_t q_data;
    int16_t i_sign;
    int16_t q_sign;
    uint16_t i_mask;
    uint16_t q_mask;

    int32_t i;

    i_sign = (Input->i >= 0) ? 0 : 1;
    q_sign = (Input->q >= 0) ? 0 : 1;            
    i_data = abs(Input->i);
    q_data = abs(Input->q);     

    for(i = 0; code_info_tbl_20_14[i].code >= 0; i++) {
        code_info = &code_info_tbl_20_14[i];
        if(i_data >= code_info->real_min && i_data <= code_info->real_max &&
            q_data >= code_info->imag_min && q_data <= code_info->imag_max) {

            Output->u.h.h_code = code_info->h_code;
            Output->u.h.h_bits = code_info->h_bits;
            Output->u.h.i_sign = i_sign;
            Output->u.h.q_sign = q_sign;
            i_mask = (1 << code_info->i_bits) - 1;
            q_mask = (1 << code_info->q_bits) - 1;
            Output->u.h.i_data = (i_data >> code_info->i_shift) & i_mask;
            Output->u.h.q_data = (q_data >> code_info->q_shift) & q_mask; 
           
            break;
        }
    }

}

void Compress_Huffman_19_14(Compandor_t *Output, Fixed_Sample_t *Input)
{

    code_info_t *code_info;
    
    uint16_t i_data;
    uint16_t q_data;
    int16_t i_sign;
    int16_t q_sign;
    uint16_t i_mask;
    uint16_t q_mask;

    int32_t i;

    i_sign = (Input->i >= 0) ? 0 : 1;
    q_sign = (Input->q >= 0) ? 0 : 1;            
    i_data = abs(Input->i);
    q_data = abs(Input->q);     

    for(i = 0; code_info_tbl_19_14[i].code >= 0; i++) {
        code_info = &code_info_tbl_19_14[i];
        if(i_data >= code_info->real_min && i_data <= code_info->real_max &&
            q_data >= code_info->imag_min && q_data <= code_info->imag_max) {

            Output->u.h.h_code = code_info->h_code;
            Output->u.h.h_bits = code_info->h_bits;
            Output->u.h.i_sign = i_sign;
            Output->u.h.q_sign = q_sign;
            i_mask = (1 << code_info->i_bits) - 1;
            q_mask = (1 << code_info->q_bits) - 1;
            Output->u.h.i_data = (i_data >> code_info->i_shift) & i_mask;
            Output->u.h.q_data = (q_data >> code_info->q_shift) & q_mask; 
           
            break;
        }
    }

}


void Deompress_Huffman_20_14(Fixed_Sample_t *Output, Compandor_t *Input)
{

    code_info_t *code_info;
    
    int32_t i;
    uint16_t i_data;
    uint16_t q_data;
    int16_t i_sign;
    int16_t q_sign;        

    for(i = 0; code_info_tbl_20_14[i].code >= 0; i++) {         
        if(Input->u.h.h_code == code_info_tbl_20_14[i].h_code) {
            code_info = &code_info_tbl_20_14[i];
            i_sign = Input->u.h.i_sign;
            q_sign = Input->u.h.q_sign;

            if(code_info->code == 0x0C) {
                i_data = Input->u.h.i_data << code_info->i_shift;
                q_data = Input->u.h.q_data << code_info->q_shift;
            }
            else {
                if(code_info->code & 0x01) {
                    i_data = Input->u.h.i_data << code_info->i_shift;
                    q_data = (Input->u.h.q_data | (1 << code_info->q_bits)) << code_info->q_shift;
                }
                else {
                    i_data = (Input->u.h.i_data | (1 << code_info->i_bits)) << code_info->i_shift;
                    q_data = Input->u.h.q_data << code_info->q_shift;
                }
            }

            if(i_sign) {
                Output->i = -((int16_t)i_data);
            }
            else {
                Output->i = (int16_t)i_data;
            }
            if(q_sign) {
                Output->q = -((int16_t)q_data);
            }
            else {
                Output->q = (int16_t)q_data;
            }  
            break;
        }
    }

}


void Deompress_Huffman_19_14(Fixed_Sample_t *Output, Compandor_t *Input)
{

    code_info_t *code_info;
    
    int32_t i;
    uint16_t i_data;
    uint16_t q_data;
    int16_t i_sign;
    int16_t q_sign;        

    for(i = 0; code_info_tbl_19_14[i].code >= 0; i++) {         
        if(Input->u.h.h_code == code_info_tbl_19_14[i].h_code) {
            code_info = &code_info_tbl_19_14[i];
            i_sign = Input->u.h.i_sign;
            q_sign = Input->u.h.q_sign;

            if(code_info->code == 0x0C) {
                i_data = Input->u.h.i_data << code_info->i_shift;
                q_data = Input->u.h.q_data << code_info->q_shift;
            }
            else {
                if(code_info->code & 0x01) {
                    i_data = Input->u.h.i_data << code_info->i_shift;
                    q_data = (Input->u.h.q_data | (1 << code_info->q_bits)) << code_info->q_shift;
                }
                else {
                    i_data = (Input->u.h.i_data | (1 << code_info->i_bits)) << code_info->i_shift;
                    q_data = Input->u.h.q_data << code_info->q_shift;
                }
            }

            if(i_sign) {
                Output->i = -((int16_t)i_data);
            }
            else {
                Output->i = (int16_t)i_data;
            }
            if(q_sign) {
                Output->q = -((int16_t)q_data);
            }
            else {
                Output->q = (int16_t)q_data;
            }  
            break;
        }
    }

}


void Compress_Samples(int32_t Code_type, int32_t N_samples, Compandor_t *Output, Fixed_Sample_t *Input)
{

    int32_t i;
    
    switch(Code_type)
    {
    case DAS_CODE_RAW:
        for(i = 0; i < N_samples; i++) {
            Output[i].u.d.i = Input[i].i;
            Output[i].u.d.q = Input[i].q;        
        }    
        break;

    case DAS_CODE_IQ_20_4:
        for(i = 0; i < N_samples; i++) {
            Compress_N_M(20, 4, &Output[i], &Input[i]); 
        }    
        break;

    case DAS_CODE_IQ_20_3:
        for(i = 0; i < N_samples; i++) {
            Compress_N_M(20, 3, &Output[i], &Input[i]);  
        }    
        break;

    case DAS_CODE_IQ_19_4:
        for(i = 0; i < N_samples; i++) {
            Compress_N_M(19, 4, &Output[i], &Input[i]); 
        }    
        break;

    case DAS_CODE_IQ_19_3:
        for(i = 0; i < N_samples; i++) {
            Compress_N_M(19, 3, &Output[i], &Input[i]);  
        }    
        break;       

    case DAS_CODE_IQ_20_4_X:
        for(i = 0; i < N_samples; i++) {
            Compress_Huffman_20_14(&Output[i], &Input[i]);  
        }            
        break;

    case DAS_CODE_IQ_19_4_X:
        for(i = 0; i < N_samples; i++) {
            Compress_Huffman_19_14(&Output[i], &Input[i]);  
        }            
        break;
        

    case DAS_CODE_IQ_20_4_Y:
    {
        Fixed_Sample_t *Input_tmp;
        char I_Bitstring[64];
        char Q_Bitstring[64];
        char IQ_Bitstring[64];

        Input_tmp = (Fixed_Sample_t *)calloc(1, sizeof(Fixed_Sample_t) * N_samples);
#if 0        
        InitCodeInfoCnt();
        UpdateCodeInfoCnt(N_samples, Input);
        print_CodeInfo();
#endif        
        for(i = 0; i < N_samples; i++) {      
#if 0            
            ConvertBitString(16, I_Bitstring, Input[i].i);
            ConvertBitString(16, Q_Bitstring, Input[i].q);
            ConvertBitString(32, IQ_Bitstring, Merge_IQ_Data(&Input[i]));
            fprintf(stderr, "[%2d] %s - %s - %s\n", i, I_Bitstring, Q_Bitstring, IQ_Bitstring);            
#endif            
            Swap_IQ_N_M_X(20, 4, &Input_tmp[i], &Input[i]);
#if 0
            ConvertBitString(16, I_Bitstring, Input_tmp[i].i);
            ConvertBitString(16, Q_Bitstring, Input_tmp[i].q);
            ConvertBitString(32, IQ_Bitstring, Merge_IQ_Data(&Input_tmp[i]));
            fprintf(stderr, "     %s - %s - %s\n", I_Bitstring, Q_Bitstring, IQ_Bitstring);       
#endif            
        }         
#if 0        
        InitCodeInfoCnt();
        UpdateCodeInfoCnt(N_samples, Input_tmp);
        print_CodeInfo();
#endif        
        
        for(i = 0; i < N_samples; i++) {
            Compress_N_M(20, 4, &Output[i], &Input_tmp[i]); 
        }            

        free(Input_tmp);
        
        break;
    }        
    }
}

void Deompress_Samples(int32_t Code_type, int32_t N_samples, Fixed_Sample_t *Output, Compandor_t *Input)
{
    int32_t i;
    switch(Code_type)
    {
    case DAS_CODE_RAW:
        for(i = 0; i < N_samples; i++) {            
            Output[i].i = Input[i].u.d.i;
            Output[i].q = Input[i].u.d.q;        
        }    
        break;   
        
    case DAS_CODE_IQ_20_4:
        for(i = 0; i < N_samples; i++) {
            Decompress_N_M(20, 4, &Output[i], &Input[i]);
        } 
        break;

    case DAS_CODE_IQ_20_3:
        for(i = 0; i < N_samples; i++) {
            Decompress_N_M(20, 3, &Output[i], &Input[i]);
        } 
        break; 
        
    case DAS_CODE_IQ_19_4:
        for(i = 0; i < N_samples; i++) {
            Decompress_N_M(19, 4, &Output[i], &Input[i]);
        } 
        break;

    case DAS_CODE_IQ_19_3:
        for(i = 0; i < N_samples; i++) {
            Decompress_N_M(19, 3, &Output[i], &Input[i]);
        } 
        break;     

    case DAS_CODE_IQ_20_4_X:
        for(i = 0; i < N_samples; i++) {
            Deompress_Huffman_20_14(&Output[i], &Input[i]);
        } 
        break;

    case DAS_CODE_IQ_19_4_X:
        for(i = 0; i < N_samples; i++) {
            Deompress_Huffman_19_14(&Output[i], &Input[i]);
        } 
        break;

    case DAS_CODE_IQ_20_4_Y:            
        for(i = 0; i < N_samples; i++) {
            Decompress_N_M_X(20, 4, &Output[i], &Input[i]);
        } 
        break;        
    }
}

int32_t Get_Ref_Symbol_BPSK(float **i_ref_sym, float **q_ref_sym)
{

    static float i_ref[] = {
        +1/sqrt(2), -1/sqrt(2)
    };
    static float q_ref[] = {
        +1/sqrt(2), -1/sqrt(2)
    };    
        
    *i_ref_sym = i_ref;
    *q_ref_sym = q_ref;
    
    return 2;
}

int32_t Get_Ref_Symbol_QPSK(float **i_ref_sym, float **q_ref_sym)
{

    static float i_ref[] = {
        +1/sqrt(2), +1/sqrt(2), -1/sqrt(2), -1/sqrt(2)
    };
    static float q_ref[] = {
        +1/sqrt(2), -1/sqrt(2), +1/sqrt(2), -1/sqrt(2)
    };
        
    *i_ref_sym = i_ref;
    *q_ref_sym = q_ref;
    
    return 4;
}


int32_t Get_Ref_Symbol_16QAM(float **i_ref_sym, float **q_ref_sym)
{

    static float i_ref[] = {
        +1/sqrt(10), +1/sqrt(10), +3/sqrt(10), +3/sqrt(10), +1/sqrt(10), +1/sqrt(10), +3/sqrt(10), +3/sqrt(10), 
        -1/sqrt(10), -1/sqrt(10), -3/sqrt(10), -3/sqrt(10), -1/sqrt(10), -1/sqrt(10), -3/sqrt(10), -3/sqrt(10)
    };
    static float q_ref[] = {
        +1/sqrt(10), +3/sqrt(10), +1/sqrt(10), +3/sqrt(10), -1/sqrt(10), -3/sqrt(10), -1/sqrt(10), -3/sqrt(10), 
        +1/sqrt(10), +3/sqrt(10), +1/sqrt(10), +3/sqrt(10), -1/sqrt(10), -3/sqrt(10), -1/sqrt(10), -3/sqrt(10)
    };

        
    *i_ref_sym = i_ref;
    *q_ref_sym = q_ref;
    
    return 16;
}


int32_t Get_Ref_Symbol_64QAM(float **i_ref_sym, float **q_ref_sym)
{

    static float i_ref[] = {
        +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), 
        +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42), +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42),
        +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), 
        +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42), +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42),
        -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), 
        -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42), -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42),
        -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), 
        -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42), -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42)

    };
        
    static float q_ref[] = {
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42), 
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42),
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42), 
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42),
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42), 
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42),
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42), 
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42)
    };

        
    *i_ref_sym = i_ref;
    *q_ref_sym = q_ref;
    
    return 64;
}


#if 0
int32_t Calculate_EVM_BPSK(int32_t N_Samples, fftwf_complex *Symbol_error, fftwf_complex *Input)
{
    return 0;
}

int32_t Calculate_EVM_QPSK(int32_t N_Samples, fftwf_complex *Symbol_error, fftwf_complex *Input)
{
    int32_t m;
    int32_t n;

    float min_dist;
    float dist;
    float i_data;
    float q_data;
    
    const static float i_ref[] = {
        +1/sqrt(2), +1/sqrt(2), -1/sqrt(2), -1/sqrt(2)
    };
    const static float q_ref[] = {
        +1/sqrt(2), -1/sqrt(2), +1/sqrt(2), -1/sqrt(2)
    };

    for(m = 0; m < N_Samples; m++) {
        i_data = Input[m][0];
        q_data = Input[m][1];
        min_dist = FLT_MAX;
        for(n = 0; n < 4; n++) {            
            dist = (i_data - i_ref[n]) *  (i_data - i_ref[n]) + (q_data - q_ref[n]) * (q_data - q_ref[n]);
            if(dist < min_dist) {
                min_dist = dist;
                Symbol_error[m][0] = i_data - i_ref[n];
                Symbol_error[m][1] = q_data - q_ref[n];                
            }
        }     
    }  

    return 0;
    
}

int32_t Calculate_EVM_16QAM(int32_t N_Samples, fftwf_complex *Symbol_error, fftwf_complex *Input)
{

    int32_t m;
    int32_t n;

    float min_dist;
    float dist;
    float i_data;
    float q_data; 

    const static float i_ref[] = {
        +1/sqrt(10), +1/sqrt(10), +3/sqrt(10), +3/sqrt(10), +1/sqrt(10), +1/sqrt(10), +3/sqrt(10), +3/sqrt(10), 
        -1/sqrt(10), -1/sqrt(10), -3/sqrt(10), -3/sqrt(10), -1/sqrt(10), -1/sqrt(10), -3/sqrt(10), -3/sqrt(10)
    };
    const static float q_ref[] = {
        +1/sqrt(10), +3/sqrt(10), +1/sqrt(10), +3/sqrt(10), -1/sqrt(10), -3/sqrt(10), -1/sqrt(10), -3/sqrt(10), 
        +1/sqrt(10), +3/sqrt(10), +1/sqrt(10), +3/sqrt(10), -1/sqrt(10), -3/sqrt(10), -1/sqrt(10), -3/sqrt(10)
    };
    
    for(m = 0; m < N_Samples; m++) {
        i_data = Input[m][0];
        q_data = Input[m][1];
        min_dist = FLT_MAX;
        for(n = 0; n < 16; n++) {            
            dist = (i_data - i_ref[n]) *  (i_data - i_ref[n]) + (q_data - q_ref[n]) * (q_data - q_ref[n]);
            if(dist < min_dist) {
                min_dist = dist;
                Symbol_error[m][0] = i_data - i_ref[n];
                Symbol_error[m][1] = q_data - q_ref[n];                
            }
        }     
    }  

    return 16;
}

int32_t Calculate_EVM_64QAM(int32_t N_Samples, fftwf_complex *Symbol_error, fftwf_complex *Input)
{

    int32_t m;
    int32_t n;

    float min_dist;
    float dist;
    float i_data;
    float q_data;

    const static float i_ref[] = {
        +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), 
        +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42), +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42),
        +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), +3/sqrt(42), +3/sqrt(42), +1/sqrt(42), +1/sqrt(42), 
        +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42), +5/sqrt(42), +5/sqrt(42), +7/sqrt(42), +7/sqrt(42),
        -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), 
        -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42), -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42),
        -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), -3/sqrt(42), -3/sqrt(42), -1/sqrt(42), -1/sqrt(42), 
        -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42), -5/sqrt(42), -5/sqrt(42), -7/sqrt(42), -7/sqrt(42)

    };
        
    const static float q_ref[] = {
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42), 
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42),
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42), 
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42),
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42), 
        +3/sqrt(42), +1/sqrt(42), +3/sqrt(42), +1/sqrt(42), +5/sqrt(42), +7/sqrt(42), +5/sqrt(42), +7/sqrt(42),
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42), 
        -3/sqrt(42), -1/sqrt(42), -3/sqrt(42), -1/sqrt(42), -5/sqrt(42), -7/sqrt(42), -5/sqrt(42), -7/sqrt(42)
    };
    

    for(m = 0; m < N_Samples; m++) {
        i_data = Input[m][0];
        q_data = Input[m][1];
        min_dist = FLT_MAX;
        for(n = 0; n < 64; n++) {            
            dist = (i_data - i_ref[n]) *  (i_data - i_ref[n]) + (q_data - q_ref[n]) * (q_data - q_ref[n]);
            if(dist < min_dist) {
                min_dist = dist;
                Symbol_error[m][0] = i_data - i_ref[n];
                Symbol_error[m][1] = q_data - q_ref[n];                
            }
        }     
    }  

    return 0;

}
#endif


int32_t Calculate_Symbol_Error(int32_t Modulation_Type, int32_t N_Samples, fftwf_complex *Symbol_error, fftwf_complex *Symbol)
{

    int32_t m;
    int32_t n;

    float min_dist;
    float dist;
    float i_data;
    float q_data;

    int32_t n_ref_symbol;
    float *i_ref;
    float *q_ref;

    switch(Modulation_Type) {
    case LIBLTE_PHY_MODULATION_TYPE_BPSK:
        n_ref_symbol = Get_Ref_Symbol_BPSK(&i_ref, &q_ref);
        break;
    case LIBLTE_PHY_MODULATION_TYPE_QPSK:
        n_ref_symbol = Get_Ref_Symbol_QPSK(&i_ref, &q_ref);
        break;
    case LIBLTE_PHY_MODULATION_TYPE_16QAM:
        n_ref_symbol = Get_Ref_Symbol_16QAM(&i_ref, &q_ref);
        break;
    case LIBLTE_PHY_MODULATION_TYPE_64QAM:
        n_ref_symbol = Get_Ref_Symbol_64QAM(&i_ref, &q_ref);
        break;
    }    

    for(m = 0; m < N_Samples; m++) {
        i_data = Symbol[m][0];
        q_data = Symbol[m][1];
        min_dist = FLT_MAX;
        for(n = 0; n < n_ref_symbol; n++) {            
            dist = (i_data - i_ref[n]) *  (i_data - i_ref[n]) + (q_data - q_ref[n]) * (q_data - q_ref[n]);
            if(dist < min_dist) {
                min_dist = dist;
                Symbol_error[m][0] = i_data - i_ref[n];
                Symbol_error[m][1] = q_data - q_ref[n];                
            }
        }     
    }      

    return 0;    
}


float Calculate_EVM(int32_t N, fftwf_complex *Symbol, fftwf_complex *Symbol_error)
{

    int32_t i;
    
    float ref_sum;
    float error_sum; 
    float i_ref;
    float q_ref;
    float evm = 0.0;

    if(N > 0) {        
        ref_sum = 0.0;
        error_sum = 0.0;
        for(i = 0; i < N; i++) {
            error_sum += Symbol_error[i][0] *  Symbol_error[i][0] + Symbol_error[i][1] *  Symbol_error[i][1];
            i_ref = Symbol[i][0] - Symbol_error[i][0];
            q_ref = Symbol[i][1] - Symbol_error[i][1];            
            ref_sum += i_ref * i_ref + q_ref * q_ref;            
        }

        evm = sqrt(error_sum / ref_sum) * 100;
        
        fprintf(stderr, "error_sum = %.4f, ref_sum = %.4f\n", error_sum, ref_sum);
        fprintf(stderr, "error_sum / ref_sum = %.4f, evm = %.4f\n", error_sum / ref_sum, evm);
        
    }

    return evm;
    
}


float Calcuate_Channel_Power(int32_t FFT_size, int32_t Start_bin, int32_t End_Bin, fftwf_complex *FFT_Data)
{
    int32_t i;
    float Sum_power;
    
    Sum_power = 0.0;

#if 0
    fprintf(stderr, "Start_bin = %d, End_Bin = %d", Start_bin,End_Bin );
#endif

    for(i = Start_bin; i < End_Bin; i++) {
        if((i >= 0) && (i < FFT_size)) {
            Sum_power = Sum_power + FFT_Data[i][0] * FFT_Data[i][0] + FFT_Data[i][1] * FFT_Data[i][1];            
        }
    }

#if 0
    fprintf(stderr, ", Sum_power = %.4f\n", Sum_power);
#endif
    
    return Sum_power;    
}

int32_t LTE_Analysis_Test0(struct LTE_Analysis_s *ctx)
{
    char I_Bitstring[64];
    char Q_Bitstring[64];

    int32_t i;
    int32_t test_cnt;

    uint8_t *Code_Bits;
    int32_t N_Code_Bits;
    int32_t N_symbols;
    int32_t Bit_per_symbol;

    float Sample_in_rms;
    float Sample_in_peak;
    float Evm;

    float in_band_power;
    float left_band_power;
    float right_band_power;

    Bit_per_symbol = GetBitPerSymbol(ctx->Modulation_Type);
    ctx->N_symbols = ctx->N_rb * ctx->N_sc_per_rb ;
    N_Code_Bits = ctx->N_symbols * Bit_per_symbol;
    
    Code_Bits = (uint8_t *)malloc(sizeof(uint8_t) * N_Code_Bits);
    if(Code_Bits == NULL) {
        return -1;
    }

    if(ctx->test_code == 0) {
        srand(RANDOM_INIT_VALUE);
    }
    else if(ctx->test_code == 1) {
        struct timeval start_time;
        gettimeofday(&start_time, NULL);
        srand(start_time.tv_usec);
    }    

    fprintf(stderr, "bandwidth = %d, Gain = %6.2f dB\n", ctx->bandwidth, 10.0 * log10(ctx->Gain));
    ctx->N_samples_total = 0;
    // for(test_cnt = 0; test_cnt < TEST_SYMBOL_COUNT; test_cnt++) {
    for(test_cnt = 0; test_cnt < 1; test_cnt++) {

        /* Prepare Sample */
        MakeRandomBits(N_Code_Bits, Code_Bits); 
        MakeSymbol(ctx->N_symbols, ctx->Symbol_in, Code_Bits, Bit_per_symbol);
    
        ctx->N_samples = ctx->FFT_size;
        Symbol2Sample(ctx->N_samples, ctx->Sample_in, ctx->N_symbols, ctx->Symbol_in);
        ctx->Sample_in_pwr = Calculate_Rms(ctx->N_samples, ctx->Sample_in);

        GainBlock(ctx->N_samples, ctx->Normalized_in, ctx->Sample_in, ctx->Gain);
        ctx->Normalized_in_pwr = Calculate_Rms(ctx->N_samples, ctx->Normalized_in);
        Adc_N_Samples(ctx->Adc_bits, ctx->N_samples, ctx->Adc_data, ctx->Adc_error, ctx->Normalized_in);

        Compress_Samples(ctx->Code_type, ctx->N_samples, ctx->Compress_data, ctx->Adc_data);        
        Deompress_Samples(ctx->Code_type, ctx->N_samples, ctx->Decompress_data, ctx->Compress_data);

        Dac_N_Samples(ctx->Dac_bits, ctx->N_samples, ctx->Normalized_out, ctx->Decompress_data);
        GainBlock(ctx->N_samples, ctx->Sample_out, ctx->Normalized_out, 1.0/ ctx->Gain);

        Sample2Symbol(ctx->N_symbols, ctx->Symbol_out, ctx->N_samples, ctx->Sample_out);
                
        Sample_in_rms = Calculate_Rms(ctx->N_samples, ctx->Sample_in);        
        Sample_in_peak = GetSamplePeak(ctx->N_samples, ctx->Sample_in);
        _FFT(ctx->FFT_size, ctx->Sample_in_fft, ctx->Sample_in);
        GainBlock(ctx->FFT_size, ctx->Sample_in_fft, ctx->Sample_in_fft, 1.0 / (float)ctx->FFT_size);

        UpdatePowerHistogram(ctx->N_samples, ctx->Normalized_in, REFERENCE_RMS_DB, ctx->Power_Histogram);
        ctx->N_samples_total += ctx->N_samples;

        Calculate_Sample_Error(ctx->N_samples, ctx->Sample_error, ctx->Sample_out, ctx->Sample_in);

        _FFT(ctx->FFT_size, ctx->Sample_out_fft, ctx->Sample_out);
        GainBlock(ctx->FFT_size, ctx->Sample_out_fft, ctx->Sample_out_fft, 1.0 / (float)ctx->FFT_size);       
        ShiftFFT(ctx->FFT_size, ctx->Sample_out_fft);        

        int32_t Half_Band = ((float)ctx->bandwidth * 1000.0 / 15.0) / 2.0;
        int32_t FFT_Center = ctx->FFT_size/2;
       
        left_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center - 3 * Half_Band, FFT_Center - Half_Band, ctx->Sample_out_fft);
        in_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center - Half_Band, FFT_Center + Half_Band, ctx->Sample_out_fft);
        right_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center + Half_Band, FFT_Center + 3 * Half_Band, ctx->Sample_out_fft);
         
        Calculate_Symbol_Error(ctx->Modulation_Type, ctx->N_symbols, ctx->Symbol_error, ctx->Symbol_out);
        Evm = Calculate_EVM(ctx->N_symbols, ctx->Symbol_out, ctx->Symbol_error);
        

#if 1        
        InitCodeInfoCnt();
        UpdateCodeInfoCnt(ctx->N_samples, ctx->Adc_data);
        print_CodeInfo();
#endif

#if 1        
        fprintf(stderr, "\n");
        fprintf(stderr, "Code\n");
        Print_Code_bits(ctx->N_symbols, Code_Bits, Bit_per_symbol);
#endif

        fprintf(stderr, "Sample_in = %.4f\n", ctx->Gain);

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_in\n");
        fprintf(stderr, "Sample_in_pwr = %.2f\n", ctx->Sample_in_pwr);
#if 1
        print_Sample(NORMAL_PRINT_COUNT, ctx->Sample_in);
#else 
        print_Sample(ctx->N_samples, ctx->Sample_in);
#endif 
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Normalized_in\n");
        fprintf(stderr, "Normalized_in_pwr = %.2f\n", ctx->Normalized_in_pwr);
#if 1
        print_Sample_Normalized(NORMAL_PRINT_COUNT, ctx->Normalized_in);
#else 
        print_Sample(ctx->N_samples, ctx->Normalized_in);
#endif 
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "ADC\n");
#if 1        
        print_Fixed_Data(NORMAL_PRINT_COUNT, ctx->Adc_data);
#else
        print_Fixed_Data(ctx->N_samples, ctx->Adc_data);
#endif
#endif


#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "ADC Error\n");
#if 1        
        print_Sample(NORMAL_PRINT_COUNT, ctx->Adc_error);
#else
        print_Encode_Error(ctx->N_samples, ctx->Adc_error);
#endif
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Compress_data\n");
#if 1        
        Print_Compress_Data(ctx->Code_type, NORMAL_PRINT_COUNT, ctx->Compress_data);
#else
        Print_Compress_Data(ctx->Code_type, ctx->N_samples, ctx->Adc_data);
#endif
#endif
        
#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Decompress Output\n");
#if 1        
        print_Fixed_Data(NORMAL_PRINT_COUNT, ctx->Decompress_data);
#else
        print_Fixed_Data(ctx->N_samples, ctx->Decompress_data);
#endif
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_out\n");
#if 1
        print_Sample(NORMAL_PRINT_COUNT, ctx->Sample_out);
#else 
        print_Sample(ctx->N_samples, ctx->Sample_out);
#endif        
#endif

#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_out_fft\n");
#if 0
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Sample_out_fft);
#else 
        print_Complex_Number(ctx->N_samples, ctx->Sample_out_fft);
#endif  
#endif
        
#if 1       
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_error\n");
#if 1   
        print_Encode_Error(NORMAL_PRINT_COUNT, ctx->Sample_error);
#else 
        print_Encode_Error(ctx->N_samples, ctx->Sample_error);
#endif 
#endif

        
#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_in_fft\n");
#if 0
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Sample_in_fft);
#else 
        print_Complex_Number(ctx->N_samples, ctx->Sample_in_fft);
#endif   
#endif


#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Symbol_out\n");
#if 1
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Symbol_out);
#else 
        print_Complex_Number(ctx->N_symbols, ctx->Symbol_out);
#endif 
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Symbol error\n");
#if 1   
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Symbol_error);
        print_Complex_Number(NORMAL_PRINT_COUNT, &ctx->Symbol_error[ctx->N_symbols/2]);

#else 
        print_Complex_Number(ctx->N_symbols, ctx->Symbol_error);
#endif 
#endif

        fprintf(stderr, "\n");
        fprintf(stderr, "[%03d] Sample_in_peak = %6.2f, Sample_in_rms = %6.2f, PAPR = %5.2f dB, EVM = %.4f",
            test_cnt, Sample_in_peak, Sample_in_rms, 20 * log10(Sample_in_peak/Sample_in_rms), Evm);

        fprintf(stderr, ", IN = %6.2f, L %6.4f (%5.2f dBc), R %6.4f (%5.2f dBc)\n",
            in_band_power, left_band_power, 10 * log10(left_band_power / in_band_power), right_band_power, 10 * log10(right_band_power / in_band_power));

    }

#if 0
    fprintf(stderr, "\n");
    fprintf(stderr, "Power_Histogram %d\n", ctx->N_samples_total);
    print_Histogram(ctx->N_measure, ctx->Power_Histogram);
    fprintf(stderr, "\n");

    CalculatePDF(ctx->N_measure, ctx->Pdf_Of_Samples, ctx->Power_Histogram, ctx->N_samples_total);
    fprintf(stderr, "\n");
    fprintf(stderr, "PDF : %d, %d\n", ctx->N_measure, ctx->N_samples_total);
    print_PDF(ctx->N_measure - 1, &ctx->Pdf_Of_Samples[1]);
    fprintf(stderr, "\n");

    CalculateCCDF(ctx->N_measure, ctx->Ccdf_Of_Samples, ctx->Pdf_Of_Samples);
    fprintf(stderr, "\n");
    fprintf(stderr, "CCDF : %d\n", ctx->N_measure);
    print_CCDF(ctx->N_measure - 1, &ctx->Ccdf_Of_Samples[1]);
    fprintf(stderr, "\n");

#endif

    free(Code_Bits);

    return 0;

}


int32_t LTE_Analysis_Test0_save(struct LTE_Analysis_s *ctx)
{
    char I_Bitstring[64];
    char Q_Bitstring[64];

    int32_t i;
    int32_t test_cnt;

    uint8_t *Code_Bits;
    int32_t N_Code_Bits;
    int32_t N_symbols;
    int32_t Bit_per_symbol;

    float Sample_in_rms;
    float Sample_in_peak;
    float Evm;

    float in_band_power;
    float left_band_power;
    float right_band_power;

    Bit_per_symbol = GetBitPerSymbol(ctx->Modulation_Type);
    ctx->N_symbols = ctx->N_rb * ctx->N_sc_per_rb ;
    N_Code_Bits = ctx->N_symbols * Bit_per_symbol;
    
    Code_Bits = (uint8_t *)malloc(sizeof(uint8_t) * N_Code_Bits);
    if(Code_Bits == NULL) {
        return -1;
    }

    if(ctx->test_code == 0) {
        srand(RANDOM_INIT_VALUE);
    }
    else if(ctx->test_code == 1) {
        struct timeval start_time;
        gettimeofday(&start_time, NULL);
        srand(start_time.tv_usec);
    }    

    fprintf(stderr, "bandwidth = %d, Gain = %6.2f dB\n", ctx->bandwidth, 10.0 * log10(ctx->Gain));
    ctx->N_samples_total = 0;
    for(test_cnt = 0; test_cnt < TEST_SYMBOL_COUNT; test_cnt++) {
    // for(test_cnt = 0; test_cnt < 1; test_cnt++) {

        /* Prepare Sample */
        MakeRandomBits(N_Code_Bits, Code_Bits); 
        MakeSymbol(ctx->N_symbols, ctx->Symbol_in, Code_Bits, Bit_per_symbol);
    
        ctx->N_samples = ctx->FFT_size;
        Symbol2Sample(ctx->N_samples, ctx->Sample_in, ctx->N_symbols, ctx->Symbol_in);
        
        GainBlock(ctx->N_samples, ctx->Normalized_in, ctx->Sample_in, ctx->Gain);
        Adc_N_Samples(ctx->Adc_bits, ctx->N_samples, ctx->Adc_data, ctx->Adc_error, ctx->Normalized_in);

        if(test_cnt == 1) {
            debug_enable = 1;
            for(i = 0; i < ctx->N_samples; i++) {
#if 1  
                if(debug_enable) {
                    ConvertBitString(16, I_Bitstring, ctx->Adc_data[i].i);
                    ConvertBitString(16, Q_Bitstring, ctx->Adc_data[i].q);
                    fprintf(stderr, "\n"); 
                    fprintf(stderr, "[%04d] %s (%6d) - %s (%6d)\n", i, I_Bitstring, ctx->Adc_data[i].i, Q_Bitstring, ctx->Adc_data[i].q); 
                    fprintf(stderr, "\n");
                }
#endif 

                Compress_Samples(ctx->Code_type, 1, &ctx->Compress_data[i], &ctx->Adc_data[i]);
                Deompress_Samples(ctx->Code_type, 1, &ctx->Decompress_data[i], &ctx->Compress_data[i]);
            
#if 1  
                if(debug_enable) {
                    ConvertBitString(16, I_Bitstring, ctx->Decompress_data[i].i);
                    ConvertBitString(16, Q_Bitstring, ctx->Decompress_data[i].q);
                    fprintf(stderr, "\n"); 
                    fprintf(stderr, "[%04d] %s (%6d) - %s (%6d)\n", i, I_Bitstring, ctx->Decompress_data[i].i, Q_Bitstring, ctx->Decompress_data[i].q); 
                    fprintf(stderr, "\n");
                }
#endif                
            }
            

                
            
        }  
        else {

            Compress_Samples(ctx->Code_type, ctx->N_samples, ctx->Compress_data, ctx->Adc_data);
            Deompress_Samples(ctx->Code_type, ctx->N_samples, ctx->Decompress_data, ctx->Compress_data);
        }
        Dac_N_Samples(ctx->Dac_bits, ctx->N_samples, ctx->Normalized_out, ctx->Decompress_data);
        GainBlock(ctx->N_samples, ctx->Sample_out, ctx->Normalized_out, 1.0/ ctx->Gain);

        Sample2Symbol(ctx->N_symbols, ctx->Symbol_out, ctx->N_samples, ctx->Sample_out);
                
        Sample_in_rms = Calculate_Rms(ctx->N_samples, ctx->Sample_in);        
        Sample_in_peak = GetSamplePeak(ctx->N_samples, ctx->Sample_in);
        _FFT(ctx->FFT_size, ctx->Sample_in_fft, ctx->Sample_in);
        GainBlock(ctx->FFT_size, ctx->Sample_in_fft, ctx->Sample_in_fft, 1.0 / (float)ctx->FFT_size);

        UpdatePowerHistogram(ctx->N_samples, ctx->Normalized_in, REFERENCE_RMS_DB, ctx->Power_Histogram);
        ctx->N_samples_total += ctx->N_samples;

        Calculate_Sample_Error(ctx->N_samples, ctx->Sample_error, ctx->Sample_out, ctx->Sample_in);

        _FFT(ctx->FFT_size, ctx->Sample_out_fft, ctx->Sample_out);
        GainBlock(ctx->FFT_size, ctx->Sample_out_fft, ctx->Sample_out_fft, 1.0 / (float)ctx->FFT_size);       
        ShiftFFT(ctx->FFT_size, ctx->Sample_out_fft);        

        int32_t Half_Band = ((float)ctx->bandwidth * 1000.0 / 15.0) / 2.0;
        int32_t FFT_Center = ctx->FFT_size/2;
       
        left_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center - 3 * Half_Band, FFT_Center - Half_Band, ctx->Sample_out_fft);
        in_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center - Half_Band, FFT_Center + Half_Band, ctx->Sample_out_fft);
        right_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center + Half_Band, FFT_Center + 3 * Half_Band, ctx->Sample_out_fft);
        
        Calculate_Symbol_Error(ctx->Modulation_Type, ctx->N_symbols, ctx->Symbol_error, ctx->Symbol_out);
        Evm = Calculate_Rms(ctx->N_symbols, ctx->Symbol_error);

#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Symbol_out\n");
#if 1
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Symbol_out);
#else 
        print_Complex_Number(ctx->N_samples, ctx->Symbol_out);
#endif 
#endif

#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_in\n");
#if 1
        print_Sample(NORMAL_PRINT_COUNT, ctx->Sample_in);
#else 
        print_Sample(ctx->N_samples, ctx->Sample_in);
#endif 
#endif

#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_out\n");
#if 1
        print_Sample(NORMAL_PRINT_COUNT, ctx->Sample_out);
#else 
        // print_Sample(ctx->N_samples, ctx->Sample_out);
#endif        
#endif

#if 0        
        fprintf(stderr, "\n");
        fprintf(stderr, "Code_Bits\n");
        Print_Code_bits(ctx->N_symbols, Code_Bits, Bit_per_symbol);
#endif

#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Symbol_in\n");
        print_Sample(ctx->N_symbols, ctx->Symbol_in);
        fprintf(stderr, "\n");
#endif        
        
#if 0        
        print_Sample(ctx->N_samples, ctx->Sample_in);
#endif

#if 0
        print_Sample_Normalized(ctx->N_samples, ctx->Normalized_in);

        ctx->Sample_in_rms = Calculate_Rms(ctx->N_samples, ctx->Normalized_in);        
        ctx->Sample_in_peak = GetSamplePeak(ctx->N_samples, ctx->Normalized_in);

        fprintf(stderr, "[%03d] Sample_in_peak = %6.2f, Sample_in_rms = %6.2f, PAPR = %5.2f dB\n",
            test_cnt, ctx->Sample_in_peak, ctx->Sample_in_rms, 20 * log10(ctx->Sample_in_peak/ctx->Sample_in_rms));
#endif


#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "ADC Bits : Adc_bits = %d\n", ctx->Adc_bits);
#if 1        
        Print_Adc_Bits(4,  ctx->Adc_data);
#else
        Print_Adc_Bits(ctx->N_samples, ctx->Adc_data);
#endif
#endif


#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Compress_data\n");
#if 1        
        Print_Compress_Data(ctx->Code_type, 4, ctx->Compress_data);
#else
        Print_Compress_Data(ctx->Code_type, ctx->N_samples, ctx->Adc_data);
#endif
#endif


        if(debug_enable) {

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "ADC\n");
#if 0        
        print_Fixed_Data(NORMAL_PRINT_COUNT, ctx->Adc_data);
#else
        print_Fixed_Data(ctx->N_samples, ctx->Adc_data);
#endif
#endif


#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Decompress Output\n");
#if 0        
        print_Fixed_Data(NORMAL_PRINT_COUNT, ctx->Decompress_data);
#else
        print_Fixed_Data(ctx->N_samples, ctx->Decompress_data);
#endif
#endif

        fprintf(stderr, "[%03d] Sample_in_peak = %6.2f, Sample_in_rms = %6.2f, PAPR = %5.2f dB, EVM = %.4f",
            test_cnt, Sample_in_peak, Sample_in_rms, 20 * log10(Sample_in_peak/Sample_in_rms), Evm);

        fprintf(stderr, ", IN_BAND = %.2f, LEFT = %.4f (%.2f dBc), RIGHT = %.4f (%.2f dBc)\n",
            in_band_power, left_band_power, 10 * log10(left_band_power / in_band_power), right_band_power, 10 * log10(right_band_power / in_band_power));

     
        }



#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "Error Vector\n");
#if 1   
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Symbol_error);
#else 
        print_Complex_Number(ctx->N_samples, ctx->Symbol_error);
#endif 
#endif


#if 0
        fprintf(stderr, "\n");
        fprintf(stderr, "ADC Error\n");
#if 1        
        print_Encode_Error(NORMAL_PRINT_COUNT, ctx->Adc_error);
#else
        print_Encode_Error(ctx->N_samples, ctx->Adc_error);
#endif
#endif

#if 1

        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_in\n");
#if 0
        print_Sample(NORMAL_PRINT_COUNT, ctx->Sample_in);
#else 
        print_Sample(ctx->N_samples, ctx->Sample_in);
#endif 
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_out\n");
#if 0
        print_Sample(NORMAL_PRINT_COUNT, ctx->Sample_out);
#else 
        print_Sample(ctx->N_samples, ctx->Sample_out);
#endif        
#endif


#if 1       
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_error\n");
#if 0   
        print_Encode_Error(NORMAL_PRINT_COUNT, ctx->Sample_error);
#else 
        print_Encode_Error(ctx->N_samples, ctx->Sample_error);
#endif 
#endif


#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_in_fft\n");
#if 0
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Sample_in_fft);
#else 
        print_Complex_Number(ctx->N_samples, ctx->Sample_in_fft);
#endif   
#endif

#if 1
        fprintf(stderr, "\n");
        fprintf(stderr, "Sample_out_fft\n");
#if 0
        print_Complex_Number(NORMAL_PRINT_COUNT, ctx->Sample_out_fft);
#else 
        print_Complex_Number(ctx->N_samples, ctx->Sample_out_fft);
#endif  
#endif



        if(test_cnt == 1) {
            break;
        }


    }

#if 0
    fprintf(stderr, "\n");
    fprintf(stderr, "Power_Histogram %d\n", ctx->N_samples_total);
    print_Histogram(ctx->N_measure, ctx->Power_Histogram);
    fprintf(stderr, "\n");

    CalculatePDF(ctx->N_measure, ctx->Pdf_Of_Samples, ctx->Power_Histogram, ctx->N_samples_total);
    fprintf(stderr, "\n");
    fprintf(stderr, "PDF : %d, %d\n", ctx->N_measure, ctx->N_samples_total);
    print_PDF(ctx->N_measure - 1, &ctx->Pdf_Of_Samples[1]);
    fprintf(stderr, "\n");

    CalculateCCDF(ctx->N_measure, ctx->Ccdf_Of_Samples, ctx->Pdf_Of_Samples);
    fprintf(stderr, "\n");
    fprintf(stderr, "CCDF : %d\n", ctx->N_measure);
    print_CCDF(ctx->N_measure - 1, &ctx->Ccdf_Of_Samples[1]);
    fprintf(stderr, "\n");

#endif

    free(Code_Bits);

    return 0;

}


int32_t LTE_Analysis_Test2(struct LTE_Analysis_s *ctx)
{

    int32_t i;
    int32_t Symbol_seq;
    int32_t Slot_no;
    int32_t Subframe_no;
    int32_t Frame_no;
    
    int32_t N_read_request;
    int32_t N_read_result;
    float Sample_in_rms;
    float Sample_in_peak;

    int32_t FFT_Start_Position;

    float Evm;

    float in_band_power;
    float left_band_power;
    float right_band_power;

    InitCodeInfoCnt();

    ctx->indata_pos = 0;
    Symbol_seq = 0;
    Slot_no = 0;
    Subframe_no = 0;
    Frame_no = 0;

    do  {        
        if(Symbol_seq == 0) {   
            N_read_request = LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ + LIBLTE_PHY_N_SAMPS_CP_L_0_30_72MHZ;
            FFT_Start_Position = LIBLTE_PHY_N_SAMPS_CP_L_0_30_72MHZ;
        }
        else {
            N_read_request = LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ + LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ;
            FFT_Start_Position = LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ;
        }    

        /* Main */
        N_read_result = LTE_Analysis_Copy_Input_to_Samplebuff(ctx, N_read_request);
        if(N_read_result <= 0) {
            break;
        }

        ctx->N_samples = N_read_result;
        GainBlock(ctx->N_samples, ctx->Normalized_in, ctx->Sample_in, ctx->Gain);

        Adc_N_Samples(ctx->Adc_bits, ctx->N_samples, ctx->Adc_data, ctx->Adc_error, ctx->Normalized_in);

        Compress_Samples(ctx->Code_type, ctx->N_samples, ctx->Compress_data, ctx->Adc_data);
        Deompress_Samples(ctx->Code_type, ctx->N_samples, ctx->Decompress_data, ctx->Compress_data);

        Dac_N_Samples(ctx->Dac_bits, ctx->N_samples, ctx->Normalized_out, ctx->Decompress_data);
        GainBlock(ctx->N_samples, ctx->Sample_out, ctx->Normalized_out, 1.0/ ctx->Gain);
      
        LTE_Analysis_Write_Output(ctx, ctx->N_samples, ctx->Sample_out);

        Sample2Symbol(ctx->N_symbols, ctx->Symbol_out, ctx->N_samples, ctx->Sample_out);


        /* Check Point */
        Sample_in_rms = Calculate_Rms(ctx->N_samples, ctx->Sample_in);        
        Sample_in_peak = GetSamplePeak(ctx->N_samples, ctx->Sample_in);

        ctx->FFT_size = 2048;
        _FFT(ctx->FFT_size, ctx->Sample_in_fft, &ctx->Sample_in[FFT_Start_Position]);
        GainBlock(ctx->FFT_size, ctx->Sample_in_fft, ctx->Sample_in_fft, 1.0 / (float)ctx->FFT_size);

        UpdatePowerHistogram(ctx->N_samples, ctx->Normalized_in, REFERENCE_RMS_DB, ctx->Power_Histogram);
        ctx->N_samples_total += ctx->N_samples;

        Calculate_Sample_Error(ctx->N_samples, ctx->Sample_error, ctx->Sample_out, ctx->Sample_in);
        
        _FFT(ctx->FFT_size, ctx->Sample_out_fft, &ctx->Sample_out[FFT_Start_Position]);
        GainBlock(ctx->FFT_size, ctx->Sample_out_fft, ctx->Sample_out_fft, 1.0 / (float)ctx->FFT_size);       
        ShiftFFT(ctx->FFT_size, ctx->Sample_out_fft);  

        int32_t Half_Band = ((float)ctx->bandwidth * 1000.0 / 15.0) / 2.0;
        int32_t FFT_Center = ctx->FFT_size/2;


        left_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center - 3 * Half_Band, FFT_Center - Half_Band, ctx->Sample_out_fft);
        in_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center - Half_Band, FFT_Center + Half_Band, ctx->Sample_out_fft);
        right_band_power = Calcuate_Channel_Power(ctx->FFT_size, FFT_Center + Half_Band, FFT_Center + 3 * Half_Band, ctx->Sample_out_fft);

        Calculate_Symbol_Error(ctx->Modulation_Type, ctx->N_symbols, ctx->Symbol_error, ctx->Symbol_out);
        Evm = Calculate_Rms(ctx->N_symbols, ctx->Symbol_error);

#if 0        
        InitCodeInfoCnt();
        UpdateCodeInfoCnt(ctx->N_samples, ctx->Adc_data);
        print_CodeInfo();
#endif          

        fprintf(stderr, "[%03d][%d][%d][%d] Sample_in_peak = %6.2f, Sample_in_rms = %6.2f, PAPR = %5.2f dB, EVM = %6.4f",
            Frame_no, Subframe_no, Slot_no, Symbol_seq, Sample_in_peak, Sample_in_rms, 20 * log10(Sample_in_peak/Sample_in_rms), Evm);

        fprintf(stderr, ", IN %6.2f, L %6.4f (%5.2f dBc), R %6.4f (%5.2f dBc)\n",
            in_band_power, left_band_power, 10 * log10(left_band_power / in_band_power), right_band_power, 10 * log10(right_band_power / in_band_power));

        Symbol_seq++;
#if 0
        if(Symbol_seq > 1) {
            break;
        }
#endif        

        if(Symbol_seq == 7) {   
            Symbol_seq = 0;
            Slot_no++;
            if(Slot_no == 2) {
                Slot_no = 0;
                Subframe_no++;
                if(Subframe_no == 10) {
                    Subframe_no = 0;
                    Frame_no++;                   
                }
            }
        }
    } while(1);
        
    LTE_Analysis_Write_Output_to_File(ctx);

    return 0;

}



int32_t LTE_Analysis_Test3(struct LTE_Analysis_s *ctx)
{
    int test_cnt;
    int test_bits;
    int i;
    int j;

    int N_read_request;
    int N_read_result;
    
    int32_t Symbol_seq;
    int32_t Slot_no;
    int32_t Subframe_no;
    int32_t Frame_no;

    float Sample_in_rms;
    float Sample_in_peak;

    InitCodeInfoCnt();

    ctx->indata_pos = 0;
    N_read_request = LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ;

    ctx->indata_pos = 0;
    Symbol_seq = 0;
    Slot_no = 0;
    Subframe_no = 0;
    Frame_no = 0;
    
    do  {

        if(Symbol_seq == 0) {   
            N_read_request = LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ + LIBLTE_PHY_N_SAMPS_CP_L_0_30_72MHZ;
        }
        else {
            N_read_request = LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ + LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ;
        }   

        N_read_result = LTE_Analysis_Copy_Input_to_Samplebuff(ctx, N_read_request);
        if(N_read_result <= 0) {
            break;
        }

        Sample_in_rms = Calculate_Rms(ctx->N_samples, ctx->Sample_in);
        Sample_in_peak = GetSamplePeak(ctx->N_samples, ctx->Sample_in);

        fprintf(stderr, "[%03d][%d][%d][%d] Sample_in_peak = %6.2f, Sample_in_rms = %6.2f, reference = %6.2f",
            Frame_no, Subframe_no, Slot_no, Symbol_seq, 
            Sample_in_peak, Sample_in_rms, ctx->InputRefRmsLevel);

        fprintf(stderr, ", PAPR = %5.2f dB, Peak to reference rms = %5.2f dB\n",
            20 * log10(Sample_in_peak / Sample_in_rms), 20 * log10(Sample_in_peak/ctx->InputRefRmsLevel));

        GainBlock(ctx->N_samples, ctx->Normalized_in, ctx->Sample_in, ctx->Gain);
#if 0
        print_Sample_Normalized(ctx->N_samples, ctx->Normalized_in);
#endif

        UpdatePowerHistogram(ctx->N_samples, ctx->Normalized_in, REFERENCE_RMS_DB, ctx->Power_Histogram);
        ctx->N_samples_total += ctx->N_samples;

        Symbol_seq++;
        if(Symbol_seq == 7) {   
            Symbol_seq = 0;
            Slot_no++;
            if(Slot_no == 2) {
                Slot_no = 0;
                Subframe_no++;
                if(Subframe_no == 10) {
                    Subframe_no = 0;
                    Frame_no++;                   
                }
            }
        }

    } while(1);

    fprintf(stderr, "\n");
    fprintf(stderr, "Power_Histogram %d\n", ctx->N_samples_total);
    print_Histogram(ctx->N_measure, ctx->Power_Histogram);
    fprintf(stderr, "\n");

    CalculatePDF(ctx->N_measure, ctx->Pdf_Of_Samples, ctx->Power_Histogram, ctx->N_samples_total);
    fprintf(stderr, "\n");
    fprintf(stderr, "PDF : %d, %d\n", ctx->N_measure, ctx->N_samples_total);
    print_PDF(ctx->N_measure - 1, &ctx->Pdf_Of_Samples[1]);
    fprintf(stderr, "\n");

    CalculateCCDF(ctx->N_measure, ctx->Ccdf_Of_Samples, ctx->Pdf_Of_Samples);
    fprintf(stderr, "\n");
    fprintf(stderr, "CCDF : %d\n", ctx->N_measure);
    print_CCDF(ctx->N_measure - 1, &ctx->Ccdf_Of_Samples[1]);
    fprintf(stderr, "\n");

    return 0;

}



void LTE_Analysis_update_sched_info(struct LTE_Analysis_DL_s *ctx)
{
    LIBLTE_RRC_SIB_TYPE_ENUM sib_array[20];
    uint32_t num_sibs;
    uint32_t sib_idx;
    uint32_t N_sibs_to_map = 0;
    uint32_t i;
    uint32_t j;

#ifdef    LIBLTE_PHY_ANALYSIS    
    uint32 log_msg_enable = 1;
    if(log_msg_enable) {
        fprintf(stderr, "\n**************** LTE_Analysis_update_sched_info ****************\n");
    }   
#endif /* LIBLTE_PHY_ANALYSIS */ 

    // Determine which SIBs need to be mapped
    num_sibs = 0;
    if(1 == ctx->sib3_present) {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_3;
    }
    if(1 == ctx->sib4_present) {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_4;
    }
    if(1 == ctx->sib8_present) {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_8;
    }

#ifdef    LIBLTE_PHY_ANALYSIS    
    if(log_msg_enable) {
        fprintf(stderr, "Initialize the scheduling info\n");
    }   
#endif /* LIBLTE_PHY_ANALYSIS */ 

    // Initialize the scheduling info
    ctx->sib1.N_sched_info = 1;
    ctx->sib1.sched_info[0].N_sib_mapping_info = 0;
    sib_idx = 0;

    while(num_sibs > 0) {
        
        // Determine how many SIBs can be mapped to this scheduling info
        if(1 == ctx->sib1.N_sched_info)  {
            if(0 == ctx->sib1.sched_info[0].N_sib_mapping_info && LIBLTE_PHY_N_RB_DL_1_4MHZ != ctx->N_rb_dl) {
                N_sibs_to_map = 1;
#ifdef    LIBLTE_PHY_ANALYSIS    
                if(log_msg_enable) {
                    fprintf(stderr, "[001] N_sibs_to_map = 1\n");
                }   
#endif /* LIBLTE_PHY_ANALYSIS */                 
            }
            else {
                N_sibs_to_map = 2;
                ctx->sib1.sched_info[ctx->sib1.N_sched_info].N_sib_mapping_info = 0;
                ctx->sib1.sched_info[ctx->sib1.N_sched_info].si_periodicity = LIBLTE_RRC_SI_PERIODICITY_RF8;
                ctx->sib1.N_sched_info++;
            }
        }
        else {
            if(2 > ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info) {
                N_sibs_to_map = 2 - ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info;
            }
            else {
                N_sibs_to_map = 2;
                ctx->sib1.sched_info[ctx->sib1.N_sched_info].N_sib_mapping_info = 0;
                ctx->sib1.sched_info[ctx->sib1.N_sched_info].si_periodicity = LIBLTE_RRC_SI_PERIODICITY_RF8;
                ctx->sib1.N_sched_info++;
            }
        }

        // Map the SIBs for this scheduling info
#ifdef    LIBLTE_PHY_ANALYSIS    
        if(log_msg_enable) {
            fprintf(stderr, "[002] Map the SIBs for this scheduling info : N_sibs_to_map = %d\n", N_sibs_to_map);
        }   
#endif /* LIBLTE_PHY_ANALYSIS */        
        for(i = 0; i < N_sibs_to_map; i++)  {     
            int index = ctx->sib1.N_sched_info-1;
            int sched_idx = ctx->sib1.sched_info[index].N_sib_mapping_info;
            ctx->sib1.sched_info[index].sib_mapping_info[sched_idx].sib_type = sib_array[sib_idx++];
            ctx->sib1.sched_info[index].N_sib_mapping_info++;
            num_sibs--;
            if(0 == num_sibs) {
                break;
            }
        }
    }

#ifdef    LIBLTE_PHY_ANALYSIS    
    if(log_msg_enable) {
        fprintf(stderr, "\nctx->sib1.sched_info = %d\n", ctx->sib1.N_sched_info);
    }   
#endif /* LIBLTE_PHY_ANALYSIS */    
    for(i = 0; i < ctx->sib1.N_sched_info; i++) {
#ifdef    LIBLTE_PHY_ANALYSIS    
        if(log_msg_enable) {
            fprintf(stderr, "ctx->sib1.sched_info[%d]\n", i);
            fprintf(stderr, "    ctx->sib1.sched_info[%d].N_sib_mapping_info = %d\n", i, ctx->sib1.sched_info[i].N_sib_mapping_info);
        }   
#endif /* LIBLTE_PHY_ANALYSIS */        
        for(j = 0; j < ctx->sib1.sched_info[i].N_sib_mapping_info; j++) {
            fprintf(stderr, "    %d (%s)\n", 
                ctx->sib1.sched_info[i].sib_mapping_info[j].sib_type, Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[i].sib_mapping_info[j].sib_type));
        }
        fprintf(stderr, "\n");
    }
    
}


#ifdef CHECK

int32_t LTE_Analaysis_PRACH_init(struct LTE_Analysis_PRACH_s *ctx)
{



    int i;
    int j;

    // Initialize the LTE parameters
    // General
    ctx->N_rb_dl      = LIBLTE_PHY_N_RB_DL_20MHZ;
    ctx->fs           = LIBLTE_PHY_FS_30_72MHZ;
    ctx->sfn          = 0;
    ctx->N_frames     = 30;
    ctx->N_ant        = 1;
    ctx->N_id_cell    = 0;
    ctx->N_id_2       = (ctx->N_id_cell % 3);
    ctx->N_id_1       = (ctx->N_id_cell - ctx->N_id_2)/3;
    ctx->sib_tx_mode  = 1;
    ctx->percent_load = 0;

    ctx->cp_mode      = 20;
    
    // MIB
    ctx->mib.dl_bw            = LIBLTE_RRC_DL_BANDWIDTH_100;
    ctx->mib.phich_config.dur = LIBLTE_RRC_PHICH_DURATION_NORMAL;
    ctx->mib.phich_config.res = LIBLTE_RRC_PHICH_RESOURCE_1;
    ctx->phich_res            = 1;
    ctx->mib.sfn_div_4        = ctx->sfn/4;
    
    // SIB1
    ctx->sib1.N_plmn_ids                       = 1;
    ctx->sib1.plmn_id[0].id.mcc                = 0xF001;
    ctx->sib1.plmn_id[0].id.mnc                = 0xFF01;
    ctx->sib1.plmn_id[0].resv_for_oper         = LIBLTE_RRC_NOT_RESV_FOR_OPER;
    ctx->sib1.N_sched_info                     = 1;
    ctx->sib1.sched_info[0].N_sib_mapping_info = 0;
    ctx->sib1.sched_info[0].si_periodicity     = LIBLTE_RRC_SI_PERIODICITY_RF8;
    ctx->si_periodicity_T                      = 8;
    ctx->sib1.cell_barred                      = LIBLTE_RRC_CELL_NOT_BARRED;
    ctx->sib1.intra_freq_reselection           = LIBLTE_RRC_INTRA_FREQ_RESELECTION_ALLOWED;
    ctx->sib1.si_window_length                 = LIBLTE_RRC_SI_WINDOW_LENGTH_MS2;
    ctx->si_win_len                            = 2;
    ctx->sib1.tdd_cnfg.sf_assignment           = LIBLTE_RRC_SUBFRAME_ASSIGNMENT_0;
    ctx->sib1.tdd_cnfg.special_sf_patterns     = LIBLTE_RRC_SPECIAL_SUBFRAME_PATTERNS_0;
    ctx->sib1.cell_id                          = 0;
    ctx->sib1.csg_id                           = 0;
    ctx->sib1.tracking_area_code               = 0;
    ctx->sib1.q_rx_lev_min                     = -140;
    ctx->sib1.csg_indication                   = 0;
    ctx->sib1.q_rx_lev_min_offset              = 1;
    ctx->sib1.freq_band_indicator              = 1;
    ctx->sib1.system_info_value_tag            = 0;
    ctx->sib1.p_max_present                    = true;
    ctx->sib1.p_max                            = -30;
    ctx->sib1.tdd                              = false;

    // SIB2
    ctx->sib2.ac_barring_info_present                                                      = false;
    ctx->sib2.rr_config_common_sib.rach_cnfg.num_ra_preambles                              = LIBLTE_RRC_NUMBER_OF_RA_PREAMBLES_N64;
    ctx->sib2.rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present                = false;
    ctx->sib2.rr_config_common_sib.rach_cnfg.pwr_ramping_step                              = LIBLTE_RRC_POWER_RAMPING_STEP_DB6;
    ctx->sib2.rr_config_common_sib.rach_cnfg.preamble_init_rx_target_pwr                   = LIBLTE_RRC_PREAMBLE_INITIAL_RECEIVED_TARGET_POWER_DBM_N100;
    ctx->sib2.rr_config_common_sib.rach_cnfg.preamble_trans_max                            = LIBLTE_RRC_PREAMBLE_TRANS_MAX_N200;
    ctx->sib2.rr_config_common_sib.rach_cnfg.ra_resp_win_size                              = LIBLTE_RRC_RA_RESPONSE_WINDOW_SIZE_SF10;
    ctx->sib2.rr_config_common_sib.rach_cnfg.mac_con_res_timer                             = LIBLTE_RRC_MAC_CONTENTION_RESOLUTION_TIMER_SF64;
    ctx->sib2.rr_config_common_sib.rach_cnfg.max_harq_msg3_tx                              = 1;
    ctx->sib2.rr_config_common_sib.bcch_cnfg.modification_period_coeff                     = LIBLTE_RRC_MODIFICATION_PERIOD_COEFF_N2;
    ctx->sib2.rr_config_common_sib.pcch_cnfg.default_paging_cycle                          = LIBLTE_RRC_DEFAULT_PAGING_CYCLE_RF256;
    ctx->sib2.rr_config_common_sib.pcch_cnfg.nB                                            = LIBLTE_RRC_NB_ONE_T;
    ctx->sib2.rr_config_common_sib.prach_cnfg.root_sequence_index                          = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index           = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag              = false;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset            = 0;
    ctx->sib2.rr_config_common_sib.pdsch_cnfg.rs_power                                     = -60;
    ctx->sib2.rr_config_common_sib.pdsch_cnfg.p_b                                          = 0;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.n_sb                                         = 1;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.hopping_mode                                 = LIBLTE_RRC_HOPPING_MODE_INTER_SUBFRAME;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.pusch_hopping_offset                         = 0;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.enable_64_qam                                = true;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled                  = false;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch                 = 0;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled               = false;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift                           = 0;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.delta_pucch_shift                            = LIBLTE_RRC_DELTA_PUCCH_SHIFT_DS1;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.n_rb_cqi                                     = 0;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.n_cs_an                                      = 0;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.n1_pucch_an                                  = 0;
    ctx->sib2.rr_config_common_sib.srs_ul_cnfg.present                                     = false;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch                            = -70;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.alpha                                       = LIBLTE_RRC_UL_POWER_CONTROL_ALPHA_1;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch                            = -96;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1                  = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1b                 = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1B_1;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2                  = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2a                 = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2A_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2b                 = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2B_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_preamble_msg3                         = -2;
    ctx->sib2.rr_config_common_sib.ul_cp_length                                            = LIBLTE_RRC_UL_CP_LENGTH_1;
    ctx->sib2.ue_timers_and_constants.t300                                                 = LIBLTE_RRC_T300_MS1000;
    ctx->sib2.ue_timers_and_constants.t301                                                 = LIBLTE_RRC_T301_MS1000;
    ctx->sib2.ue_timers_and_constants.t310                                                 = LIBLTE_RRC_T310_MS1000;
    ctx->sib2.ue_timers_and_constants.n310                                                 = LIBLTE_RRC_N310_N20;
    ctx->sib2.ue_timers_and_constants.t311                                                 = LIBLTE_RRC_T311_MS1000;
    ctx->sib2.ue_timers_and_constants.n311                                                 = LIBLTE_RRC_N311_N10;
    ctx->sib2.arfcn_value_eutra.present                                                    = false;
    ctx->sib2.ul_bw.present                                                                = false;
    ctx->sib2.additional_spectrum_emission                                                 = 1;
    ctx->sib2.mbsfn_subfr_cnfg_list_size                                                   = 0;
    ctx->sib2.time_alignment_timer                                                         = LIBLTE_RRC_TIME_ALIGNMENT_TIMER_SF500;

    // SIB3
    ctx->sib3_present                          = 0;
    ctx->sib3.q_hyst                           = LIBLTE_RRC_Q_HYST_DB_0;
    ctx->sib3.speed_state_resel_params.present = false;
    ctx->sib3.s_non_intra_search_present       = false;
    ctx->sib3.thresh_serving_low               = 0;
    ctx->sib3.cell_resel_prio                  = 0;
    ctx->sib3.q_rx_lev_min                     = ctx->sib1.q_rx_lev_min;
    ctx->sib3.p_max_present                    = true;
    ctx->sib3.p_max                            = ctx->sib1.p_max;
    ctx->sib3.s_intra_search_present           = false;
    ctx->sib3.allowed_meas_bw_present          = false;
    ctx->sib3.presence_ant_port_1              = false;
    ctx->sib3.neigh_cell_cnfg                  = 0;
    ctx->sib3.t_resel_eutra                    = 0;
    ctx->sib3.t_resel_eutra_sf_present         = false;
    
    // SIB4
    ctx->sib4_present                         = 0;
    ctx->sib4.intra_freq_neigh_cell_list_size = 0;
    ctx->sib4.intra_freq_black_cell_list_size = 0;
    ctx->sib4.csg_phys_cell_id_range_present  = false;
    
    // SIB8
    ctx->sib8_present                 = 0;
    ctx->sib8.sys_time_info_present   = false;
    ctx->sib8.search_win_size_present = true;
    ctx->sib8.search_win_size         = 0;
    ctx->sib8.params_hrpd_present     = false;
    ctx->sib8.params_1xrtt_present    = false;
    
    // PCFICH
    ctx->pcfich.cfi = 2;
    // PHICH
    for(i=0; i<25; i++)  {
        for(j=0; j<8; j++) {
            ctx->phich.present[i][j] = false;
        }
    }

    ctx->samp_buf_idx    = 0;
    ctx->samples_ready   = false;
    ctx->last_samp_was_i = false;

    ctx->i_buf = (float *)malloc(4*LTE_FDD_DL_FS_SAMP_BUF_SIZE*sizeof(float));
    ctx->q_buf = (float *)malloc(4*LTE_FDD_DL_FS_SAMP_BUF_SIZE*sizeof(float));

    ctx->samp_buf_w_idx = 0;
    ctx->samp_buf_r_idx = 0;
    ctx->input_offset = 0;

    /**********************************************/

    // set_bandwidth
    ctx->N_rb_dl   = LIBLTE_PHY_N_RB_DL_10MHZ;
    ctx->mib.dl_bw = LIBLTE_RRC_DL_BANDWIDTH_50;

    // set_fs
    ctx->fs = LIBLTE_PHY_FS_30_72MHZ;

    // set_param(&sib1.freq_band_indicator, value, 1, 25);
    ctx->sib1.freq_band_indicator = 1;

    // set_param(&N_frames, value, 1, 1000);
    ctx->N_frames = 30;

    // set_n_ant
    ctx->N_ant = 1;
    ctx->sib_tx_mode = 1;

    // set_n_id_cell(value);
    ctx->N_id_cell = 0;
    ctx->N_id_2    = (ctx->N_id_cell % 3);
    ctx->N_id_1    = (ctx->N_id_cell - ctx->N_id_2)/3;

    // set_mcc(value);
    ctx->sib1.plmn_id[0].id.mcc = 0xF000 | 0x0001;

    // set_mnc
    ctx->sib1.plmn_id[0].id.mnc = 0xFF00 | 0x0001;

    // set_param(&sib1.cell_id, value, 0, 268435455);;
    ctx->sib1.cell_id = 0;

    // set_param(&sib1.tracking_area_code, value, 0, 65535);
    ctx->sib1.tracking_area_code = 0;

    // err = set_param(&sib1.q_rx_lev_min, value, -140, -44);
    // sib3.q_rx_lev_min = sib1.q_rx_lev_min;
    ctx->sib1.q_rx_lev_min = -140;
    ctx->sib3.q_rx_lev_min = ctx->sib1.q_rx_lev_min;

    // err = set_param(&sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch, value, -126, 24);
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch = -70;
    
    // err = set_param(&sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch, value, -127, -96);
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch = -96;

    // err = set_param(&sib3_present, value, 0, 1);
    ctx->sib3_present = 0;
    
    // err = set_q_hyst(value);
    ctx->sib3.q_hyst = (LIBLTE_RRC_Q_HYST_ENUM)LIBLTE_RRC_Q_HYST_DB_0;

    // err = set_param(&sib4_present, value, 0, 1);
    ctx->sib4_present = 0;

    // err = set_neigh_cell_list(value);
    // set_param(&sib4.intra_freq_neigh_cell_list_size, token1, 0, LIBLTE_RRC_MAX_CELL_INTRA)
    ctx->sib4.intra_freq_neigh_cell_list_size = 0;

    // err = set_param(&sib8_present, value, 0, 1);
    ctx->sib8_present = 0;

    // err = set_param(&sib8.search_win_size, value, 0, 15);
    ctx->sib8.search_win_size = 0;

    // err = set_param(&percent_load, value, 0, 66); // FIXME: Decode issues if load is greater than 66%
    ctx->percent_load = LTE_TEST_PERCENT_LOAD;   
    // 

    LTE_Fdd_Analysis_recreate_sched_info(ctx);



    return 0;
}

#endif	


uint64_t g_log_msg_code = 0;

struct LTE_Analysis_PRACH_s LTE_Analysis_PRACH_ctx;
struct LTE_Analysis_DL_s LTE_Analysis_DL_ctx;



struct LTE_Analysis_PRACH_s *LTE_Analysis_PRACH_GetContext(void)
{
	return &LTE_Analysis_PRACH_ctx;
}

struct LTE_Analysis_DL_s *LTE_Analysis_DL_GetContext(void)
{
	return &LTE_Analysis_DL_ctx;
}




int32_t LTE_Analaysis_PRACH_init(struct LTE_Analysis_PRACH_s *ctx)
{

    // Initialize the LTE parameters
    ctx->phy_struct     = NULL;

    // General
    ctx->fs             = LIBLTE_PHY_FS_30_72MHZ;

    ctx->N_id_cell      = LIBLTE_PHY_INIT_N_ID_CELL_UNKNOWN;
    ctx->N_ant          = 1;
    ctx->N_rb_dl        = LIBLTE_PHY_N_RB_DL_20MHZ;
    ctx->N_sc_rb_dl     = LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP;
    ctx->phich_res      = liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1];

    ctx->cp_mode        = LIBLTE_PHY_CP_TYPE_NORMAL;

    /* UL */
    ctx->N_rb_ul        = LIBLTE_PHY_N_RB_UL_20MHZ;

    // sib2/radioResourceConfigCommon/prach-Config/rootSequenceIndex    
    // sib2/radioResourceConfigCommon/prach-Config/prach-ConfigInfo/prach-ConfigIndex
    // sib2/radioResourceConfigCommon/prach-Config/prach-ConfigInfo/zeroCorrelationZoneConfig
    // sib2/radioResourceConfigCommon/prach-Config/prach-ConfigInfo/highSpeedFlag
    // sib2/radioResourceConfigCommon/prach-Config/prach-ConfigInfo/prach-FreqOffset
    ctx->sib2.rr_config_common_sib.prach_cnfg.root_sequence_index = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index = 0;
	ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config = 2;
	ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag = 0;
    // ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset = 10;
    // ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset = 50;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset = 0;

    // sib2 -> internal      
    ctx->prach_root_seq_idx =  ctx->sib2.rr_config_common_sib.prach_cnfg.root_sequence_index;
    PRACH_5_7_1_2_s *prach_5_7_1_2 = get_prach_5_7_1_2(ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index);
    if(prach_5_7_1_2 == NULL || prach_5_7_1_2->preamble_format == SYSTEM_FRAME_NUMBER_NA) {
        return -1;
    }    

    ctx->prach_preamble_format = prach_5_7_1_2->preamble_format;
    ctx->prach_zczc            = ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config;
    ctx->prach_hs_flag         = ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag;

    ctx->prach_preamble_index  = 4;
    ctx->prach_freq_offset     = ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset;

    ctx->prach_samps_re = (float *)malloc(sizeof(float) * PRACH_PREAMBLE_MAX_LENGTH);
    ctx->prach_samps_im = (float *)malloc(sizeof(float) * PRACH_PREAMBLE_MAX_LENGTH);
    for(int i = 0; i < PRACH_PREAMBLE_MAX_LENGTH; i++) {
        ctx->prach_samps_re[i] = 0.0;
        ctx->prach_samps_im[i] = 0.0;
    }

    ctx->group_assignment_pusch     = 0;
    ctx->group_hopping_enabled      = 0;
    ctx->sequence_hopping_enabled   = 0;
    
    ctx->cyclic_shift               = 0;
    ctx->cyclic_shift_dci           = 0;    
    ctx->N_cs_an                    = 0;
    ctx->delta_pucch_shift          = 0;

	return 0;
}



int32_t LTE_Analaysis_PRACH(struct LTE_Analysis_PRACH_s *ctx)
{

    liblte_phy_init_n(&ctx->phy_struct,
                    ctx->fs,
                    ctx->N_id_cell,
                    ctx->N_ant,
                    ctx->N_rb_dl,              
                    ctx->N_sc_rb_dl,
                    ctx->frame_struct_type,
                    ctx->cp_mode,
                    liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1]);

    // PRACH
    liblte_phy_ul_init(ctx->phy_struct,
                        ctx->N_id_cell, 
                        ctx->prach_root_seq_idx, ctx->prach_preamble_format, ctx->prach_zczc, ctx->prach_hs_flag, // PRACH
                        ctx->group_assignment_pusch, ctx->group_hopping_enabled, ctx->sequence_hopping_enabled,
                        ctx->cyclic_shift, ctx->cyclic_shift_dci, ctx->N_cs_an, ctx->delta_pucch_shift                        
                );

    liblte_phy_generate_prach_n(ctx->phy_struct, ctx->prach_preamble_index, ctx->prach_freq_offset, ctx->prach_samps_re, ctx->prach_samps_im);

    liblte_phy_detect_prach_n(ctx->phy_struct, ctx->prach_samps_re, ctx->prach_samps_im, ctx->prach_freq_offset, &ctx->N_det_pre, &ctx->det_pre, &ctx->det_ta);

	return 0;
    
}


int32_t LTE_Analaysis_DL_init(struct LTE_Analysis_DL_s *ctx)
{
    int i;
    int j;

    // Initialize the LTE parameters
    ctx->phy_struct     = NULL;

    // General
    ctx->frame_struct_type = LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD;
    
    ctx->fs             = LIBLTE_PHY_FS_30_72MHZ;

    ctx->N_id_cell      = LIBLTE_PHY_INIT_N_ID_CELL_FOR_TEST;
    ctx->N_id_2         = (ctx->N_id_cell % 3);
    ctx->N_id_1         = (ctx->N_id_cell - ctx->N_id_2)/3;
    
    ctx->N_ant          = 2;
    ctx->N_rb_dl        = LIBLTE_PHY_N_RB_DL_5MHZ;
    ctx->N_sc_rb_dl     = LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP;
    ctx->phich_res      = liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1];
    ctx->N_layer        = 2;

    ctx->cp_mode        = LIBLTE_PHY_CP_TYPE_NORMAL;

    ctx->mib.dl_bw            = LIBLTE_RRC_DL_BANDWIDTH_25;
    ctx->mib.phich_config.dur = LIBLTE_RRC_PHICH_DURATION_NORMAL;
    ctx->mib.phich_config.res = LIBLTE_RRC_PHICH_RESOURCE_1;
    ctx->phich_res = liblte_rrc_phich_resource_num[ctx->mib.phich_config.res];

     

    liblte_phy_init_n(&ctx->phy_struct,
                    ctx->fs,
                    ctx->N_id_cell,
                    ctx->N_ant,
                    ctx->N_rb_dl,              
                    ctx->N_sc_rb_dl,
                    
                    ctx->frame_struct_type,
                    ctx->cp_mode, 
                    
                    ctx->phich_res);   


    // SIB1
    ctx->si_periodicity_T                       = 8;
    ctx->si_win_len                             = 2;
    ctx->sib_tx_mode                            = 1;
    
    ctx->sib1.freq_band_indicator               = 1;

    ctx->sib1.N_plmn_ids                        = 1;
    ctx->sib1.plmn_id[0].resv_for_oper          = LIBLTE_RRC_NOT_RESV_FOR_OPER;
    ctx->sib1.plmn_id[0].id.mcc                 = 0xF000 | 0x0001;
    ctx->sib1.plmn_id[0].id.mnc                 = 0xFF00 | 0x0001;

    ctx->sib1.N_sched_info                      = 1;
    ctx->sib1.sched_info[0].N_sib_mapping_info  = 0;
    ctx->sib1.sched_info[0].si_periodicity      = LIBLTE_RRC_SI_PERIODICITY_RF8;
    
    ctx->sib1.cell_barred                       = LIBLTE_RRC_CELL_NOT_BARRED;
    ctx->sib1.intra_freq_reselection            = LIBLTE_RRC_INTRA_FREQ_RESELECTION_ALLOWED;
    ctx->sib1.si_window_length                  = LIBLTE_RRC_SI_WINDOW_LENGTH_MS2;
    ctx->sib1.tdd_cnfg.sf_assignment            = LIBLTE_RRC_SUBFRAME_ASSIGNMENT_0;
    ctx->sib1.tdd_cnfg.special_sf_patterns      = LIBLTE_RRC_SPECIAL_SUBFRAME_PATTERNS_0;
    ctx->sib1.cell_id                           = 301;
    ctx->sib1.csg_id                            = 0;
    ctx->sib1.tracking_area_code                = 0;
    ctx->sib1.q_rx_lev_min                      = -140;
    ctx->sib1.csg_indication                    = 0;
    ctx->sib1.q_rx_lev_min_offset               = 1;
    ctx->sib1.freq_band_indicator               = 1;
    ctx->sib1.system_info_value_tag             = 0;
    ctx->sib1.p_max_present                     = true;
    ctx->sib1.p_max                             = -30;
    ctx->sib1.tdd                               = false;

    // SIB2
    ctx->sib2.ac_barring_info_present                                                      = false;
    
    ctx->sib2.rr_config_common_sib.rach_cnfg.num_ra_preambles                              = LIBLTE_RRC_NUMBER_OF_RA_PREAMBLES_N64;
    ctx->sib2.rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present                = false;
    ctx->sib2.rr_config_common_sib.rach_cnfg.pwr_ramping_step                              = LIBLTE_RRC_POWER_RAMPING_STEP_DB6;
    ctx->sib2.rr_config_common_sib.rach_cnfg.preamble_init_rx_target_pwr                   = LIBLTE_RRC_PREAMBLE_INITIAL_RECEIVED_TARGET_POWER_DBM_N100;
    ctx->sib2.rr_config_common_sib.rach_cnfg.preamble_trans_max                            = LIBLTE_RRC_PREAMBLE_TRANS_MAX_N200;
    ctx->sib2.rr_config_common_sib.rach_cnfg.ra_resp_win_size                              = LIBLTE_RRC_RA_RESPONSE_WINDOW_SIZE_SF10;
    ctx->sib2.rr_config_common_sib.rach_cnfg.mac_con_res_timer                             = LIBLTE_RRC_MAC_CONTENTION_RESOLUTION_TIMER_SF64;
    ctx->sib2.rr_config_common_sib.rach_cnfg.max_harq_msg3_tx                              = 1;
    
    ctx->sib2.rr_config_common_sib.bcch_cnfg.modification_period_coeff                     = LIBLTE_RRC_MODIFICATION_PERIOD_COEFF_N2;
    ctx->sib2.rr_config_common_sib.pcch_cnfg.default_paging_cycle                          = LIBLTE_RRC_DEFAULT_PAGING_CYCLE_RF256;
    ctx->sib2.rr_config_common_sib.pcch_cnfg.nB                                            = LIBLTE_RRC_NB_ONE_T;
    
    ctx->sib2.rr_config_common_sib.prach_cnfg.root_sequence_index                          = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index           = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag              = false;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config = 0;
    ctx->sib2.rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset            = 0;
    
    ctx->sib2.rr_config_common_sib.pdsch_cnfg.rs_power                                     = -60;
    ctx->sib2.rr_config_common_sib.pdsch_cnfg.p_b                                          = 0;
    
    ctx->sib2.rr_config_common_sib.pusch_cnfg.n_sb                                         = 1;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.hopping_mode                                 = LIBLTE_RRC_HOPPING_MODE_INTER_SUBFRAME;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.pusch_hopping_offset                         = 0;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.enable_64_qam                                = true;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled                  = false;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch                 = 0;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled               = false;
    ctx->sib2.rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift                           = 0;

    ctx->sib2.rr_config_common_sib.pucch_cnfg.delta_pucch_shift                            = LIBLTE_RRC_DELTA_PUCCH_SHIFT_DS1;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.n_rb_cqi                                     = 0;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.n_cs_an                                      = 0;
    ctx->sib2.rr_config_common_sib.pucch_cnfg.n1_pucch_an                                  = 0;

    ctx->sib2.rr_config_common_sib.srs_ul_cnfg.present                                     = false;

    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch                            = -70;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.alpha                                       = LIBLTE_RRC_UL_POWER_CONTROL_ALPHA_1;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch                            = -96;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1                  = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1b                 = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1B_1;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2                  = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2a                 = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2A_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2b                 = LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2B_0;
    ctx->sib2.rr_config_common_sib.ul_pwr_ctrl.delta_preamble_msg3                         = -2;
    
    ctx->sib2.rr_config_common_sib.ul_cp_length                                            = LIBLTE_RRC_UL_CP_LENGTH_1;
    
    ctx->sib2.ue_timers_and_constants.t300                                                 = LIBLTE_RRC_T300_MS1000;
    ctx->sib2.ue_timers_and_constants.t301                                                 = LIBLTE_RRC_T301_MS1000;
    ctx->sib2.ue_timers_and_constants.t310                                                 = LIBLTE_RRC_T310_MS1000;
    ctx->sib2.ue_timers_and_constants.n310                                                 = LIBLTE_RRC_N310_N20;
    ctx->sib2.ue_timers_and_constants.t311                                                 = LIBLTE_RRC_T311_MS1000;
    ctx->sib2.ue_timers_and_constants.n311                                                 = LIBLTE_RRC_N311_N10;
    
    ctx->sib2.arfcn_value_eutra.present                                                    = false;
    ctx->sib2.ul_bw.present                                                                = false;
    ctx->sib2.additional_spectrum_emission                                                 = 1;
    ctx->sib2.mbsfn_subfr_cnfg_list_size                                                   = 0;
    ctx->sib2.time_alignment_timer                                                         = LIBLTE_RRC_TIME_ALIGNMENT_TIMER_SF500;

    // SIB3
    ctx->sib3_present                          = 0;
    ctx->sib3.q_hyst                           = LIBLTE_RRC_Q_HYST_DB_0;
    ctx->sib3.speed_state_resel_params.present = false;
    ctx->sib3.s_non_intra_search_present       = false;
    ctx->sib3.thresh_serving_low               = 0;
    ctx->sib3.cell_resel_prio                  = 0;
    ctx->sib3.q_rx_lev_min                     = ctx->sib1.q_rx_lev_min;
    ctx->sib3.p_max_present                    = true;
    ctx->sib3.p_max                            = ctx->sib1.p_max;
    ctx->sib3.s_intra_search_present           = false;
    ctx->sib3.allowed_meas_bw_present          = false;
    ctx->sib3.presence_ant_port_1              = false;
    ctx->sib3.neigh_cell_cnfg                  = 0;
    ctx->sib3.t_resel_eutra                    = 0;
    ctx->sib3.t_resel_eutra_sf_present         = false;
    
    // SIB4
    ctx->sib4_present                         = 0;
    ctx->sib4.intra_freq_neigh_cell_list_size = 0;
    ctx->sib4.intra_freq_black_cell_list_size = 0;
    ctx->sib4.csg_phys_cell_id_range_present  = false;
    
    // SIB8
    ctx->sib8_present                 = 0;
    ctx->sib8.sys_time_info_present   = false;
    ctx->sib8.search_win_size_present = true;
    ctx->sib8.search_win_size         = 0;
    ctx->sib8.params_hrpd_present     = false;
    ctx->sib8.params_1xrtt_present    = false;

    // PCFICH
    ctx->pcfich.cfi = 2;
    
    // PHICH
    for(i = 0; i < 25; i++)  {
        for(j = 0; j < 8; j++) {
            ctx->phich.present[i][j] = false;
        }
    }


    return 0;
}

int32_t LTE_Analaysis_PSCH(struct LTE_Analysis_DL_s *ctx)
{
    int p;
    int i;
    int j;

    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }    

        ctx->subframe.num = 0;
        liblte_phy_map_pss_n(ctx->phy_struct, &ctx->subframe, ctx->N_id_2, ctx->N_ant);
    }


    return 0;    
}

int32_t LTE_Analaysis_SSCH(struct LTE_Analysis_DL_s *ctx)
{
    int p;
    int i;
    int j;

    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }    

        ctx->subframe.num = 0;
        liblte_phy_map_sss_n(ctx->phy_struct, &ctx->subframe, ctx->N_id_1, ctx->N_id_2,
                           ctx->N_ant);

    }

    return 0;    
}

int32_t LTE_Analaysis_RS(struct LTE_Analysis_DL_s *ctx)
{

    int p;
    int i;
    int j;

    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }    

        liblte_phy_map_crs_n(ctx->phy_struct,
                           &ctx->subframe,
                           ctx->N_id_cell,
                           ctx->N_ant); 

    }

    return 0;
}


int32_t LTE_Analaysis_PCFICH(struct LTE_Analysis_DL_s *ctx)
{

    int p;
    int i;
    int j;

    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }    

        ctx->subframe.num = 0;
        ctx->pcfich.cfi = 3;
        // Physical control format indicator channel
        pcfich_channel_map_n(ctx->phy_struct, &ctx->pcfich, ctx->N_id_cell, ctx->N_ant,  ctx->N_layer, &ctx->subframe);

    }

    return 0;
}


int32_t LTE_Analaysis_PHICH(struct LTE_Analysis_DL_s *ctx)
{
    int p;
    int i;
    int j;

    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }  
   
        ctx->phich.present[0][0] = 1;
        ctx->phich.b[0][0] = 1;
        ctx->phich.present[0][1] = 1;
        ctx->phich.b[0][1] = 0;
        ctx->phich.present[0][2] = 0;
        ctx->phich.b[0][2] = 0;        
        ctx->phich.present[0][3] = 1;
        ctx->phich.b[0][3] = 0;

        ctx->phich.present[2][2] = 1;
        ctx->phich.b[2][2] = 1;
        ctx->phich.present[2][3] = 1;
        ctx->phich.b[2][3] = 0;

        ctx->N_layer = 2;

        phich_channel_map_n(ctx->phy_struct, &ctx->phich, &ctx->pcfich, ctx->N_id_cell, ctx->N_ant, ctx->N_layer,
            ctx->mib.phich_config.res, ctx->mib.phich_config.dur, &ctx->subframe);

    }

    return 0;
}


int32_t LTE_Analaysis_PBCH(struct LTE_Analysis_DL_s *ctx)
{
    int p;
    int i;
    int j;

#ifdef    LIBLTE_PHY_ANALYSIS    
    uint32 log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_PBCH);
    if(log_msg_enable) {
        fprintf(stderr, "\n**************** LTE_Analaysis_PBCH ****************\n");
    }   
#endif /* LIBLTE_PHY_ANALYSIS */    

    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }    

        ctx->N_ant = 2;
        ctx->N_layer = 2;
        ctx->sfn = 4;
        ctx->subframe.num  = 0;
        ctx->mib.dl_bw = LIBLTE_RRC_DL_BANDWIDTH_25;
        ctx->mib.sfn_div_4 = ctx->sfn / 4;
        ctx->mib.phich_config.dur = LIBLTE_RRC_PHICH_DURATION_NORMAL;
        ctx->mib.phich_config.res = LIBLTE_RRC_PHICH_RESOURCE_1;
        liblte_rrc_pack_bcch_bch_msg(&ctx->mib, &ctx->rrc_msg);

#ifdef    LIBLTE_PHY_ANALYSIS    
        if(log_msg_enable) {
            fprintf(stderr, "\nRRC MEssage\n");            
            fprintf(stderr, "N_bits = %d\n", ctx->rrc_msg.N_bits);            
            print_bits(ctx->rrc_msg.msg, ctx->rrc_msg.N_bits);
        }  
#endif /* LIBLTE_PHY_ANALYSIS */

        liblte_phy_bch_channel_encode_n
        (
            ctx->phy_struct, 
            ctx->rrc_msg.msg, ctx->rrc_msg.N_bits, 
            ctx->N_id_cell, ctx->N_ant, ctx->N_layer, 
            &ctx->subframe, 
            0
        );

    }

    return 0;
}


int32_t LTE_Analaysis_PDCCH(struct LTE_Analysis_DL_s *ctx)
{
    int p;
    int i;
    int j;
    uint32      max_N_prb;

    LIBLTE_PHY_ALLOCATION_STRUCT *alloc;

#ifdef    LIBLTE_PHY_ANALYSIS    
    uint32 log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_PDCCH);
    if(log_msg_enable) {
        fprintf(stderr, "\n**************** LTE_Analaysis_PDCCH ****************\n");
    }   
#endif /* LIBLTE_PHY_ANALYSIS */


    if(ctx->frame_struct_type == LIBLTE_PHY_FRAME_STRUCT_TYPE_FDD) {

        for(p = 0; p < ctx->N_ant; p++) {     
            for(i = 0; i < LIBLTE_PHY_N_SUBFRAME_MAX; i++) {
                for(j = 0; j < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; j++) {
                    ctx->subframe.tx_symb_re[p][i][j] = 0.0;
                    ctx->subframe.tx_symb_im[p][i][j] = 0.0;
                }
            }
        }    

        ctx->pdcch.N_alloc = 0;
        ctx->sfn = 100;

        // Test SIB1        
#if 0         
        ctx->subframe.num = 5; 
#endif         
        // Load
#if 1  
        ctx->subframe.num = 3; 
        ctx->modulation_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        ctx->percent_load = LTE_TEST_PERCENT_LOAD;
#endif        
        
        if(ctx->subframe.num == 5 && (ctx->sfn % 2) == 0)  {

#ifdef    LIBLTE_PHY_ANALYSIS        
            if(log_msg_enable) {
                fprintf(stderr, "\n###### SIB1 ######\n");
            }   
#endif /* LIBLTE_PHY_ANALYSIS */
                        
            // SIB1
            ctx->bcch_dlsch_msg.N_sibs           = 0;
            ctx->bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1;
            memcpy(&ctx->bcch_dlsch_msg.sibs[0].sib, &ctx->sib1, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT));

            liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg, &ctx->pdcch.alloc[ctx->pdcch.N_alloc].msg[0]);
    
            alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];                                               
#ifdef    LIBLTE_PHY_ANALYSIS   
            if(log_msg_enable) {
                fprintf(stderr, "\nctx->pdcch.alloc[%d].msg[0] - SIB1 : nbits = %d\n", ctx->pdcch.N_alloc, ctx->pdcch.alloc[ctx->pdcch.N_alloc].msg[0].N_bits);
                print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
            }  
#endif /* LIBLTE_PHY_ANALYSIS */

            liblte_phy_get_tbs_mcs_and_n_prb_for_dl_n
            (
                alloc->msg[0].N_bits,
                ctx->subframe.num,
                ctx->N_rb_dl,
                LIBLTE_MAC_SI_RNTI,
                &alloc->tbs,
                &alloc->mcs,
                &alloc->N_prb
            );

            alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
            alloc->mod_type       = LIBLTE_PHY_MODULATION_TYPE_QPSK;
            alloc->rv_idx         = (uint32)ceilf(1.5 * ((ctx->sfn / 2) % 4)) % 4; //36.321 section 5.3.1
            alloc->N_codewords    = 1;
            alloc->rnti           = LIBLTE_MAC_SI_RNTI;
            alloc->tx_mode        = ctx->sib_tx_mode;      
            if(alloc->mcs >= 10 && alloc->mcs <= 15) {
                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
            }
            else if(alloc->mcs >= 15 && alloc->mcs <= 27) {
                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
            }
                    
#ifdef    LIBLTE_PHY_ANALYSIS   
            if(log_msg_enable) {
                fprintf(stderr, "\nSystem Information Block 1\n");
                fprintf(stderr, "pdcch.alloc[%d].pre_coder_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->pre_coder_type, liblte_phy_pre_coder_type_text[alloc->pre_coder_type]);
                fprintf(stderr, "pdcch.alloc[%d].mod_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
                fprintf(stderr, "pdcch.alloc[%d].chan_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->chan_type, liblte_phy_chan_type_text[alloc->chan_type]);
                fprintf(stderr, "pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                fprintf(stderr, "pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                fprintf(stderr, "pdcch.alloc[%d].rnti = 0x%04x, %d (%s)\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti, rnti_to_str(alloc->rnti));
                fprintf(stderr, "pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                fprintf(stderr, "pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                fprintf(stderr, "pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                fprintf(stderr, "pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
            } 
#endif /* LIBLTE_PHY_ANALYSIS */
                
            ctx->pdcch.N_alloc++;            

        }

        if(ctx->subframe.num             >=  (0 * ctx->si_win_len)%10 &&
           ctx->subframe.num             <   (1 * ctx->si_win_len)%10 &&
           (ctx->sfn % ctx->si_periodicity_T) == ((0 * ctx->si_win_len)/10))   {    
            
            ctx->sib3_present = 1;
            ctx->sib4_present = 1;
            ctx->sib8_present = 1;        
            LTE_Analysis_update_sched_info(ctx);


#ifdef    LIBLTE_PHY_ANALYSIS        
            if(log_msg_enable) {
                fprintf(stderr, "\n###### SIB2 ######\n");
            }   
#endif /* LIBLTE_PHY_ANALYSIS */ 
    
            // SIs in 1st scheduling info list entry
            ctx->bcch_dlsch_msg.N_sibs           = 1;
            ctx->bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2;
            memcpy(&ctx->bcch_dlsch_msg.sibs[0].sib, &ctx->sib2, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT));
    
#ifdef    LIBLTE_PHY_ANALYSIS   
            if(log_msg_enable) {
                fprintf(stderr, "\nSIs in 1st scheduling info list entry\n");
                fprintf(stderr, "bcch_dlsch_msg.sibs[0].sib_type = %d (%s)\n", ctx->bcch_dlsch_msg.sibs[0].sib_type, Get_SIB_TYPE_Info_Str(ctx->bcch_dlsch_msg.sibs[0].sib_type));
                fprintf(stderr, "    sib1.sched_info[0].N_sib_mapping_info = %d\n", ctx->sib1.sched_info[0].N_sib_mapping_info);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            if(ctx->sib1.sched_info[0].N_sib_mapping_info != 0) {
            
#ifdef    LIBLTE_PHY_ANALYSIS   
                if(log_msg_enable) {
                    fprintf(stderr, "sib1.sched_info[0].sib_mapping_info[0].sib_type = %d (%s)\n",
                        ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type,
                        Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type));
                }
#endif /* LIBLTE_PHY_ANALYSIS */
                            
                switch(ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type) {
                case LIBLTE_RRC_SIB_TYPE_3:
                    ctx->bcch_dlsch_msg.N_sibs++;
                    ctx->bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3;
                    memcpy(&ctx->bcch_dlsch_msg.sibs[1].sib, &ctx->sib3, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT));
                    break;
                case LIBLTE_RRC_SIB_TYPE_4:
                    ctx->bcch_dlsch_msg.N_sibs++;
                    ctx->bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4;
                    memcpy(&ctx->bcch_dlsch_msg.sibs[1].sib, &ctx->sib4, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT));
                    break;
                case LIBLTE_RRC_SIB_TYPE_8:
                    ctx->bcch_dlsch_msg.N_sibs++;
                    ctx->bcch_dlsch_msg.sibs[1].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8;
                    memcpy(&ctx->bcch_dlsch_msg.sibs[1].sib, &ctx->sib8, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT));
                    break;
                default:
                    break;
                }
                                
            }
           
            alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];
            liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg,
                                                       &alloc->msg[0]);
        
#ifdef    LIBLTE_PHY_ANALYSIS   
            if(log_msg_enable) {
                fprintf(stderr, "pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
            }
#endif /* LIBLTE_PHY_ANALYSIS */
        
            if(LIBLTE_SUCCESS == liblte_phy_get_tbs_mcs_and_n_prb_for_dl_n(alloc->msg[0].N_bits,
                                                                         ctx->subframe.num,
                                                                         ctx->N_rb_dl,
                                                                         LIBLTE_MAC_SI_RNTI,
                                                                         &alloc->tbs,
                                                                         &alloc->mcs,
                                                                         &alloc->N_prb))
            {
                alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                alloc->mod_type       = LIBLTE_PHY_MODULATION_TYPE_QPSK;
                alloc->rv_idx         = 0; //36.321 section 5.3.1
                alloc->N_codewords    = 1;
                alloc->tx_mode        = ctx->sib_tx_mode;
                alloc->rnti           = LIBLTE_MAC_SI_RNTI;
                if(alloc->mcs >= 10 && alloc->mcs <= 15) {
                    alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
                }
                else if(alloc->mcs >= 15 && alloc->mcs <= 27) {
                    alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
                }
        
        
#ifdef    LIBLTE_PHY_ANALYSIS   
                if(log_msg_enable) {
                    fprintf(stderr, "\nfor SIB2\n");
                    fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->pre_coder_type, liblte_phy_pre_coder_type_text[alloc->pre_coder_type]);
                    fprintf(stderr, "    pdcch.alloc[%d].mod_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
                    fprintf(stderr, "    pdcch.alloc[%d].chan_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->chan_type, liblte_phy_chan_type_text[alloc->chan_type]);
                    fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                    fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                    fprintf(stderr, "    pdcch.alloc[%d].rnti = 0x%04x, %d (%s)\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti, rnti_to_str(alloc->rnti));
                    fprintf(stderr, "    pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                    fprintf(stderr, "    pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                    fprintf(stderr, "    pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                    fprintf(stderr, "    pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
                }
#endif /* LIBLTE_PHY_ANALYSIS */
                ctx->pdcch.N_alloc++;
            }
        }


#ifdef    LIBLTE_PHY_ANALYSIS  
        if(log_msg_enable) {
            fprintf(stderr, "\nsib1.N_sched_info = %d\n", ctx->sib1.N_sched_info);
            fprintf(stderr, "subframe.num = %d\n", ctx->subframe.num);
            fprintf(stderr, "sfn = %d\n", ctx->sfn);
            fprintf(stderr, "si_win_len = %d\n", ctx->si_win_len);
            fprintf(stderr, "si_periodicity_T = %d\n", ctx->si_periodicity_T);
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        for(j = 1; j < ctx->sib1.N_sched_info; j++) {
            if(ctx->subframe.num             ==  (j * ctx->si_win_len)%10 &&
               (ctx->sfn % ctx->si_periodicity_T) == ((j * ctx->si_win_len)/10))   {
                           
                // SIs in the jth scheduling info list entry
#ifdef    LIBLTE_PHY_ANALYSIS  
                if(log_msg_enable) {
                    fprintf(stderr, "\n[2]SIs in the jth scheduling info list entry\n");
                    fprintf(stderr, "    sib1.sched_info[%d].N_sib_mapping_info = %d\n", j, ctx->sib1.sched_info[j].N_sib_mapping_info);
                }
#endif /* LIBLTE_PHY_ANALYSIS */  
                ctx->bcch_dlsch_msg.N_sibs = ctx->sib1.sched_info[j].N_sib_mapping_info;
                for(i = 0; i < ctx->bcch_dlsch_msg.N_sibs; i++) {
        
#ifdef    LIBLTE_PHY_ANALYSIS 
                    if(log_msg_enable) {
                        fprintf(stderr, "    ctx->sib1.sched_info[%d].sib_mapping_info[%d].sib_type = %d (%s)\n", 
                            j, i, ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type,
                            Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type));
                    }
#endif /* LIBLTE_PHY_ANALYSIS */ 
                                
                    switch(ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type) {
                    case LIBLTE_RRC_SIB_TYPE_3:
                        ctx->bcch_dlsch_msg.sibs[i].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3;
                        memcpy(&ctx->bcch_dlsch_msg.sibs[i].sib, &ctx->sib3, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT));
                        break;
                    case LIBLTE_RRC_SIB_TYPE_4:
                        ctx->bcch_dlsch_msg.sibs[i].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4;
                        memcpy(&ctx->bcch_dlsch_msg.sibs[i].sib, &ctx->sib4, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT));
                        break;
                    case LIBLTE_RRC_SIB_TYPE_8:
                        ctx->bcch_dlsch_msg.sibs[i].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8;
                        memcpy(&ctx->bcch_dlsch_msg.sibs[i].sib, &ctx->sib8, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT));
                        break;
                    default:
                        break;
                    }
                }
                            
                if(0 != ctx->bcch_dlsch_msg.N_sibs) {
        
#ifdef    LIBLTE_PHY_ANALYSIS 
                    if(log_msg_enable) {
                        fprintf(stderr, "\nliblte_rrc_pack_bcch_dlsch_msg\n");
                        fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
                        fprintf(stderr, "    bcch_dlsch_msg.N_sibs = %d\n", ctx->bcch_dlsch_msg.N_sibs);
                    }
#endif /* LIBLTE_PHY_ANALYSIS */
        
                    alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];
                    liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg, &alloc->msg[0]);
        
#ifdef    LIBLTE_PHY_ANALYSIS 
                    if(log_msg_enable) {
                        fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                        print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                    }
#endif /* LIBLTE_PHY_ANALYSIS */                        
        
                    liblte_phy_get_tbs_mcs_and_n_prb_for_dl_n(alloc->msg[0].N_bits,
                                                            ctx->subframe.num,
                                                            ctx->N_rb_dl,
                                                            LIBLTE_MAC_SI_RNTI,
                                                            &alloc->tbs,
                                                            &alloc->mcs,
                                                            &alloc->N_prb);
                    
                    alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                    alloc->mod_type       = LIBLTE_PHY_MODULATION_TYPE_QPSK;
                    alloc->rv_idx         = 0; //36.321 section 5.3.1
                    alloc->N_codewords    = 1;
                    alloc->rnti           = LIBLTE_MAC_SI_RNTI;
                    alloc->tx_mode        = ctx->sib_tx_mode;
                    if(alloc->mcs >= 10 && alloc->mcs <= 15) {
                        alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
                    }
                    else if(alloc->mcs >= 15 && alloc->mcs <= 27) {
                        alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
                    }
        
        
#ifdef    LIBLTE_PHY_ANALYSIS 
                    if(log_msg_enable) {
                        fprintf(stderr, "\n");
                        fprintf(stderr, "for SIBs\n");
                        fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->pre_coder_type, liblte_phy_pre_coder_type_text[alloc->pre_coder_type]);
                        fprintf(stderr, "    pdcch.alloc[%d].mod_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
                        fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                        fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                        fprintf(stderr, "    pdcch.alloc[%d].rnti = 0x%04x (%d) (%s)\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti, rnti_to_str(alloc->rnti));
                        fprintf(stderr, "    pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                        fprintf(stderr, "    pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                        fprintf(stderr, "    pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                        fprintf(stderr, "    pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
                    }
#endif /* LIBLTE_PHY_ANALYSIS */ 
        
                    ctx->pdcch.N_alloc++;                        
                                
                }
            }
        }

        // Add test load
        if(0 == ctx->pdcch.N_alloc) {               

#ifdef    LIBLTE_PHY_ANALYSIS  
            if(log_msg_enable) {
                fprintf(stderr, "\nAdd test load\n");
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            alloc = &ctx->pdcch.alloc[ctx->pdcch.N_alloc];
            alloc->msg[0].N_bits = 0;
            alloc->N_prb         = 0;

            if(ctx->modulation_type == LIBLTE_PHY_MODULATION_TYPE_QPSK) {

                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
                liblte_phy_get_tbs_mcs_and_n_prb_for_dl_n(1480,
                                                        ctx->subframe.num,
                                                        ctx->N_rb_dl,
                                                        LIBLTE_MAC_P_RNTI,
                                                        &alloc->tbs,
                                                        &alloc->mcs,
                                                        &max_N_prb);

                while(alloc->N_prb < (uint32)((float)(max_N_prb*ctx->percent_load)/100.0))   {
                    alloc->msg[0].N_bits += 8;
                    liblte_phy_get_tbs_mcs_and_n_prb_for_dl_n(alloc->msg[0].N_bits,
                                                            ctx->subframe.num,
                                                            ctx->N_rb_dl,
                                                            LIBLTE_MAC_P_RNTI,
                                                            &alloc->tbs,
                                                            &alloc->mcs,
                                                            &alloc->N_prb);
                }

            }
            else if(ctx->modulation_type == LIBLTE_PHY_MODULATION_TYPE_16QAM) {

                alloc->msg[0].N_bits = LTE_TEST_16QAM_BITS_COUNT;
                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
            
                liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type_n(alloc->msg[0].N_bits,
                                                        ctx->subframe.num,
                                                        ctx->N_rb_dl,
                                                        LIBLTE_MAC_P_RNTI,
                                                        &alloc->tbs,
                                                        &alloc->mcs,
                                                        &alloc->N_prb,
                                                        alloc->mod_type );                    
            }
            else if(ctx->modulation_type == LIBLTE_PHY_MODULATION_TYPE_64QAM) {
                alloc->msg[0].N_bits = LTE_TEST_64QAM_BITS_COUNT;
                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;

                liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type_n(alloc->msg[0].N_bits,
                                                        ctx->subframe.num,
                                                        ctx->N_rb_dl,
                                                        LIBLTE_MAC_P_RNTI,
                                                        &alloc->tbs,
                                                        &alloc->mcs,
                                                        &alloc->N_prb,
                                                        alloc->mod_type ); 
            }

            for(i = 0; i < alloc->msg[0].N_bits; i++)  {
                alloc->msg[0].msg[i] = liblte_rrc_test_load[i%8];
            }
            
            if(0 != alloc->N_prb)  {                
                alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                alloc->N_codewords = 1;
                alloc->rnti        = LIBLTE_MAC_P_RNTI;                    
                alloc->tx_mode     = ctx->sib_tx_mode;                   

#ifdef    LIBLTE_PHY_ANALYSIS  
                if(log_msg_enable) {
                    fprintf(stderr, "\n");
                    fprintf(stderr, "for TEST LOAD\n");
                    fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->pre_coder_type, liblte_phy_pre_coder_type_text[alloc->pre_coder_type]);
                    fprintf(stderr, "    pdcch.alloc[%d].mod_type = %d (%s)\n", ctx->pdcch.N_alloc, alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
                    fprintf(stderr, "    pdcch.alloc[%d].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[ctx->pdcch.N_alloc].N_bits);
                    fprintf(stderr, "    pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
                    fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                    fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                    fprintf(stderr, "    pdcch.alloc[%d].rnti = 0x%04x, %d, (%s)\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti, rnti_to_str(alloc->rnti));
                    fprintf(stderr, "    pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                    fprintf(stderr, "    pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                    fprintf(stderr, "    pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                }
#endif /* LIBLTE_PHY_ANALYSIS */
                ctx->pdcch.N_alloc++;
            }
        }  

                    // Schedule all allocations
#ifdef    LIBLTE_PHY_ANALYSIS  
        if(log_msg_enable) {
            fprintf(stderr, "\nSchedule all allocations\n");
            fprintf(stderr, "   pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
            for(i = 0; i < ctx->pdcch.N_alloc; i++) {
                fprintf(stderr, "   pdcch.alloc[%d].N_prb = %d\n", i, ctx->pdcch.alloc[i].N_prb);
            } 
        }
#endif /* LIBLTE_PHY_ANALYSIS */            
        
        // FIXME: Scheduler
        ctx->last_prb = 0;
        for(i = 0; i < ctx->pdcch.N_alloc; i++) {
            for(j = 0; j < ctx->pdcch.alloc[i].N_prb; j++) {
                ctx->pdcch.alloc[i].prb[0][j] = ctx->last_prb;
                ctx->pdcch.alloc[i].prb[1][j] = ctx->last_prb++;
            }
        }

        if(0 != ctx->pdcch.N_alloc) {

#ifdef    LIBLTE_PHY_ANALYSIS  
            if(log_msg_enable) {
                fprintf(stderr, "\nliblte_phy_pdcch_channel_encode\n");
                fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

#if 1

            liblte_phy_pdcch_channel_encode_n(ctx->phy_struct,
                                            &ctx->pcfich,
                                            &ctx->phich,
                                            &ctx->pdcch,
                                            ctx->N_id_cell,
                                            ctx->N_ant,
                                            ctx->N_layer,
                                            ctx->mib.phich_config.res,
                                            ctx->mib.phich_config.dur,
                                            &ctx->subframe);

#else 

            liblte_phy_pdcch_channel_encode(ctx->phy_struct,
                                            &ctx->pcfich,
                                            &ctx->phich,
                                            &ctx->pdcch,
                                            ctx->N_id_cell,
                                            ctx->N_ant,
                                            ctx->phich_res,
                                            ctx->mib.phich_config.dur,
                                            &ctx->subframe);
#endif

#ifdef    LIBLTE_PHY_ANALYSIS  
            if(log_msg_enable) {
                fprintf(stderr, "\nliblte_phy_pdcch_channel_encode\n");
            }
#endif /* LIBLTE_PHY_ANALYSIS */

#if 0
            liblte_phy_pdsch_channel_encode(ctx->phy_struct,
                                            &ctx->pdcch,
                                            ctx->N_id_cell,
                                            ctx->N_ant,
                                            &ctx->subframe);
#endif
        }



#if 0

        liblte_phy_pdcch_channel_encode_n(
            ctx->phy_struct,
            &ctx->pcfich,
            &ctx->phich,
            &ctx->pdcch,
            ctx->N_id_cell,
            ctx->N_ant,
            ctx->phich_res,
            ctx->mib.phich_config.dur,
            &ctx->subframe
        );
#endif

    }

    return 0;    
}


int Msg_Check_Enabled(uint64_t log_msg_code)
{
    if((g_log_msg_code & log_msg_code) != 0) {
        return 1;
    }
    else {
        return 0;
    }   
}

int main(int argc, char *argv[])
{

    int result = 0;

    g_log_msg_code = 
          LOG_MSG_INIT | LOG_MSG_INIT_DETAIL 
        | LOG_MSG_PRACH | LOG_MSG_PRACH_DETAIL
        | LOG_MSG_UL_INIT 
        | LOG_MSG_PSCH 
        | LOG_MSG_SSCH
        | LOG_MSG_RS
        | LOG_MSG_PCFICH
        | LOG_MSG_PBCH
        | LOG_MSG_PDCCH
    ;

#ifdef    PRACH_DEBUG

    struct LTE_Analysis_PRACH_s *ctx;
    ctx = LTE_Analysis_PRACH_GetContext();
	LTE_Analaysis_PRACH_init(ctx);
    LTE_Analaysis_PRACH(ctx);
    
#endif /* PRACH_DEBUG */

    struct LTE_Analysis_DL_s *ctx;

    ctx = LTE_Analysis_DL_GetContext();    
    LTE_Analaysis_DL_init(ctx);
    
    LTE_Analaysis_PSCH(ctx);
    LTE_Analaysis_SSCH(ctx);
    LTE_Analaysis_RS(ctx);
    LTE_Analaysis_PCFICH(ctx);
    LTE_Analaysis_PHICH(ctx);
    LTE_Analaysis_PBCH(ctx);
    LTE_Analaysis_PDCCH(ctx);
    

	return result;
	
}



/* End of LTE_ANALYSIS_Exported_Functions */
/**
  * @}
  */

/* End of LTE_ANALYSIS */
/**
  * @}
  */


