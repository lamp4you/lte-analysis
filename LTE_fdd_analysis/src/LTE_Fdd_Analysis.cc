/**
  ******************************************************************************
  * @file    LTE_Fdd_Analysis.c
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

#include "LTE_Fdd_Analysis.h"

/** @addtogroup LTE_FDD_ANALYSIS
  * @{
  */
/* Private macros ------------------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Private_Macros
  * @{
  */

#define _DECODE_                       'd'
#define _ENCODE_                       'e'
#define _MOD_TYPE_                     'm'

#define _INPUT_                        'i'
#define _OUTPUT_                       'o'
#define _LOG_MSG_                      'L'

#define LTE_FDD_ANALYSIS_INPUT_FILE_NAME_DEFAULT       "LTE_Fdd_5MHz.bin"

#define COARSE_TIMING_N_SLOTS                    (160)
#define COARSE_TIMING_SEARCH_NUM_SUBFRAMES       ((COARSE_TIMING_N_SLOTS/2)+2)
#define PSS_AND_FINE_TIMING_SEARCH_NUM_SUBFRAMES (COARSE_TIMING_SEARCH_NUM_SUBFRAMES)
#define SSS_SEARCH_NUM_SUBFRAMES                 (COARSE_TIMING_SEARCH_NUM_SUBFRAMES)
#define BCH_DECODE_NUM_FRAMES                    (2)
#define PDSCH_DECODE_SIB1_NUM_FRAMES             (2)
#define PDSCH_DECODE_SI_GENERIC_NUM_FRAMES       (1)

#define INPUT_DATA_SKIP_LENGTH                  (192)
#define INPUT_DATA_OFFSET_FOR_TEST               (LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ + LIBLTE_PHY_N_SAMPS_CP_L_0_30_72MHZ + INPUT_DATA_SKIP_LENGTH)

#define LTE_TEST_16QAM_BITS_COUNT       640
#define LTE_TEST_64QAM_BITS_COUNT       1024
#define LTE_TEST_PERCENT_LOAD           30

static const float INPUT_DATA_SCALE = 2.0;


#define FILE_NAME_LENGTH        128
#define BYTES_PER_PIXEL         3
#define PIXEL_PER_RE            8
#define SUBFRAME_PER_FRAME      10

#define BLANK_COLOR             255
#define BOARDER_COLOR           40


/* End of LTE_FDD_ANALYSIS_Private_Macros */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Private_Types
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Private_Types */
/**
  * @}
  */

/* Private functions Prototype ------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Private_Functions
  * @{
  */

static int LTE_Fdd_Analysis_ReadInput(char *input_file_name, struct complex_data_f **indata);
static int LTE_Fdd_Analysis_Process_Option(int argc, char *argv[], struct LTE_Fdd_Analysis_s *ctx);

static int LTE_Fdd_Analysis_Get_File_Len(char *file_name);

static int32_t LTE_Fdd_Analysis_Copy_Input_to_Samplebuff(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_Init_Frame(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_freq_shift(struct LTE_Fdd_Analysis_s *ctx, int32_t start_idx, int32_t num_samps, float freq_offset);

static char *LTE_Fdd_Analysis_GetStateName(int32_t State);

static int32_t LTE_Fdd_Analysis_Coarse_Timing_Search(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_PSS_And_Fine_Timing_Search(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_SSS_Search(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_BCH_Decode(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_PDSCH_Decode_SIB1(struct LTE_Fdd_Analysis_s *ctx);
static int32_t LTE_Fdd_Analysis_PDSCH_Decode_SI_Generic(struct LTE_Fdd_Analysis_s *ctx);

static void LTE_Fdd_Analysis_print_mib(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_MIB_STRUCT *mib);
static void LTE_Fdd_Analysis_print_sib1(struct LTE_Fdd_Analysis_s *ctx,LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *sib1);
static void LTE_Fdd_Analysis_print_sib2(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *sib2);
static void LTE_Fdd_Analysis_print_sib3(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *sib3);
static void LTE_Fdd_Analysis_print_sib4(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *sib4);
static void LTE_Fdd_Analysis_print_sib5(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *sib5);
static void LTE_Fdd_Analysis_print_sib6(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *sib6);
static void LTE_Fdd_Analysis_print_sib7(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *sib7);
static void LTE_Fdd_Analysis_print_sib8(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *sib8);
static void LTE_Fdd_Analysis_print_sib13(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13_STRUCT *sib13);
static void LTE_Fdd_Analysis_print_page(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_PAGING_STRUCT *page);

static void LTE_Fdd_Analysis_recreate_sched_info(struct LTE_Fdd_Analysis_s *ctx);

/* End of LTE_FDD_ANALYSIS_Private_Functions */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Private_Variables
  * @{
  */


struct LTE_Fdd_Analysis_s LTE_Fdd_Analysis_context;



static struct lte_enum_info LTE_Fdd_Analysis_State_Info[] =
{
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH,        (char *)"COARSE_TIMING_SEARCH"       },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH,  (char *)"PSS_AND_FINE_TIMING_SEARCH" },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH,                  (char *)"SSS_SEARCH"                 },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE,                  (char *)"BCH_DECODE"                 },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1,           (char *)"PDSCH_DECODE_SIB1"          },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC,     (char *)"PDSCH_DECODE_SI_GENERIC"    },

    { -1,     NULL   }
};

static char *null_str = (char *)"(null)";


struct LTE_Fdd_Analysis_State_Handler_s LTE_Fdd_Analysis_State_Handler[] =
{

    { LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH,        LTE_Fdd_Analysis_Coarse_Timing_Search       },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH,  LTE_Fdd_Analysis_PSS_And_Fine_Timing_Search },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH,                  LTE_Fdd_Analysis_SSS_Search                 },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE,                  LTE_Fdd_Analysis_BCH_Decode                 },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1,           LTE_Fdd_Analysis_PDSCH_Decode_SIB1          },
    { LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC,     LTE_Fdd_Analysis_PDSCH_Decode_SI_Generic    },

    { -1,     NULL   }
};

struct PixelColor color_green = { 0, 255, 0 };
struct PixelColor color_red = { 0, 0, 255 };
struct PixelColor color_blue = { 255, 0, 0 };

float chn_6RB_filter_30_72MPS[] =
{
#if 1

// Columns  1 through 10:
  -0.00040909,  -0.00050909,  -0.00062934,  -0.00077587,  -0.00095169,  -0.00115597,  -0.00138348,  -0.00162416,  -0.00186295,  -0.00207990,
// Columns 11 through 20:
  -0.00225057,  -0.00234666,  -0.00233692,  -0.00218835,  -0.00186749,  -0.00134190,  -0.00058173,   0.00043869,   0.00173942,   0.00333347,
// Columns 21 through 30:
   0.00522566,   0.00741168,   0.00987751,   0.01259915,   0.01554274,   0.01866507,   0.02191445,   0.02523200,   0.02855317,   0.03180957,
// Columns 31 through 40:
   0.03493104,   0.03784779,   0.04049264,   0.04280316,   0.04472374,   0.04620745,   0.04721763,   0.04772913,   0.04772913,   0.04721763,
// Columns 41 through 50:
   0.04620745,   0.04472374,   0.04280316,   0.04049264,   0.03784779,   0.03493104,   0.03180957,   0.02855317,   0.02523200,   0.02191445,
// Columns 51 through 60:
   0.01866507,   0.01554274,   0.01259915,   0.00987751,   0.00741168,   0.00522566,   0.00333347,   0.00173942,   0.00043869,  -0.00058173,
// Columns 61 through 70:
  -0.00134190,  -0.00186749,  -0.00218835,  -0.00233692,  -0.00234666,  -0.00225057,  -0.00207990,  -0.00186295,  -0.00162416,  -0.00138348,
// Columns 71 through 76:
  -0.00115597,  -0.00095169,  -0.00077587,  -0.00062934,  -0.00050909,  -0.00040909

#else
// Columns 1 through 10:
     -0.00036213,  -0.00046337,  -0.00058380,  -0.00072983,  -0.00090517,  -0.00111003,  -0.00134045,  -0.00158784,  -0.00183880,  -0.00207505,
//    Columns 11 through 20:
     -0.00227380,  -0.00240823,  -0.00244836,  -0.00236203,  -0.00211622,  -0.00167837,  -0.00101792,  -0.00010782,   0.00107395,   0.00254305,
//    Columns 21 through 30:
      0.00430748,   0.00636663,   0.00871055,   0.01131955,   0.01416421,   0.01720565,   0.02039629,   0.02368092,   0.02699807,   0.03028170,
//    Columns 31 through 40:
      0.03346313,   0.03647307,   0.03924378,   0.04171124,   0.04381720,   0.04551108,   0.04675168,   0.04750852,   0.04776290,   0.04750852,
//    Columns 41 through 50:
      0.04675168,   0.04551108,   0.04381720,   0.04171124,   0.03924378,   0.03647307,   0.03346313,   0.03028170,   0.02699807,   0.02368092,
//    Columns 51 through 60:
      0.02039629,   0.01720565,   0.01416421,   0.01131955,   0.00871055,   0.00636663,   0.00430748,   0.00254305,   0.00107395,  -0.00010782,
//    Columns 61 through 70:
     -0.00101792,  -0.00167837,  -0.00211622,  -0.00236203,  -0.00244836,  -0.00240823,  -0.00227380,  -0.00207505,  -0.00183880,  -0.00158784,
//    Columns 71 through 77:
     -0.00134045,  -0.00111003,  -0.00090517,  -0.00072983,  -0.00058380,  -0.00046337,  -0.00036213
#endif

};



/* End of LTE_FDD_ANALYSIS_Private_Variables */
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Exported_Variables
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Exported_Variables */
/**
  * @}
  */

/* Private functions ------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Private_Functions
  * @{
  */

void low_pass_filter
(
    float *coef,
    int32_t filter_order,
    struct complex_data_f *input,
    struct complex_data_f *output,
    int32_t nsamples
)
{
    uint32 len = nsamples;
    uint16 len_fir = filter_order;
    uint16 len_half = (len_fir-1)/2;
    float real_acc;
    float imag_acc;

    //
    // to conform matlab filter
    //
    for (uint32 i = len_half; i < len_fir; i++) {
        real_acc = 0.0;
        imag_acc = 0.0;
        for (uint16 j = 0; j < (i+1); j++){
            real_acc = real_acc + coef[j]*input[i-j].i_data;
            imag_acc = imag_acc + coef[j]*input[i-j].q_data;
        }
        output[i-len_half].i_data = real_acc;
        output[i-len_half].q_data = imag_acc;
    }

    for (uint32 i = len_fir; i < len; i++) {
        real_acc = 0.0;
        imag_acc = 0.0;
        for (uint16 j=0; j<len_fir; j++){
            real_acc = real_acc + coef[j]*input[i-j].i_data;
            imag_acc = imag_acc + coef[j]*input[i-j].q_data;
        }
        output[i-len_half].i_data = real_acc;
        output[i-len_half].q_data = imag_acc;
    }

    for (uint32 i = len; i < (len+len_half); i++) {
        real_acc = 0.0;
        imag_acc = 0.0;
        for (uint16 j = (i-len+1); j < len_fir; j++){
            real_acc = real_acc + coef[j]*input[i-j].i_data;
            imag_acc = imag_acc + coef[j]*input[i-j].q_data;
        }
        output[i-len_half].i_data = real_acc;
        output[i-len_half].q_data = imag_acc;
    }
}



void filter_n
(
    //Inputs
    const float *coef,
    int32_t filter_order,
    float *input,
    float *output,
    int nsamples
)
{
    uint32 len = nsamples;
    uint16 len_fir = filter_order;
    uint16 len_half = (len_fir-1)/2;
    float real_acc;
    float imag_acc;

    // to conform matlab filter
    for (uint32 i=len_half; i<len_fir; i++) {
        real_acc = 0;
        imag_acc = 0;
        for (uint16 j=0; j<(i+1); j++){
            real_acc = real_acc + coef[j]*input[2*(i-j)];
            imag_acc = imag_acc + coef[j]*input[2*(i-j)+1];
        }
        output[2*(i-len_half)] = real_acc;
        output[2*(i-len_half)+1] = imag_acc;
    }

    for (uint32 i=len_fir; i<len; i++) {
        real_acc = 0;
        imag_acc = 0;
        for (uint16 j=0; j<len_fir; j++){
            real_acc = real_acc + coef[j]*input[2*(i-j)];
            imag_acc = imag_acc + coef[j]*input[2*(i-j)+1];
        }
        output[2*(i-len_half)] = real_acc;
        output[2*(i-len_half)+1] = imag_acc;
    }

    for (uint32 i=len; i<(len+len_half); i++) {
        real_acc = 0;
        imag_acc = 0;
        for (uint16 j=(i-len+1); j<len_fir; j++){
            real_acc = real_acc + coef[j]*input[2*(i-j)];
            imag_acc = imag_acc + coef[j]*input[2*(i-j)+1];
        }
        output[2*(i-len_half)] = real_acc;
        output[2*(i-len_half)+1] = imag_acc;
    }
}




void LTE_Fdd_Analysis_print_int8_data(int8_t *data, int count)
{
    int i;

    for(i = 0; i < count; i++) {
        if((i % 16) == 0) {
            fprintf(stderr, "[%04d] %4d", i, data[i]);
        }
        else if((i % 16) == 15) {
            fprintf(stderr, " %4d\n", data[i]);
        }
        else {
            fprintf(stderr, " %4d", data[i]);
        }
    }
    fprintf(stderr, "\n");
}

void LTE_Fdd_Analysis_print_bits(uint8_t *bits, int bitcount)
{
    int i;
    for(i = 0; i < bitcount; i++) {
        if((i % 50) == 0) {
            fprintf(stderr, "[%04d] %1d", i, bits[i]);
        }
        else if((i % 50) == 49) {
            fprintf(stderr, " %1d\n", bits[i]);
        }
        else {
            fprintf(stderr, " %1d", bits[i]);
        }
    }
    fprintf(stderr, "\n");
}


static char *LTE_Fdd_Analysis_GetStateName(int32_t Value)
{
    int i;

    for(i = 0; LTE_Fdd_Analysis_State_Info[i].Value >= 0; i++) {
        if(LTE_Fdd_Analysis_State_Info[i].Value == Value) {
            return LTE_Fdd_Analysis_State_Info[i].Name;
        }
    }

    return null_str;

}


static int LTE_Fdd_Analysis_Print_Usage(char *prgoname)
{
    fprintf(stdout, "Usage: %s\n", prgoname);
    fprintf(stdout, "Where options are:\n");
    fprintf(stdout, "  -d, --decode\n");
    fprintf(stdout, "  -e, --encode\n");
    fprintf(stdout, "  -m, --mod [qpsk, 16qam, 64qam]\n");
    fprintf(stdout, "  -L, --log logmsgcode\n");
    fprintf(stdout, "  -i, --input inputfile       : input file\n");
    fprintf(stdout, "  -o, --output outputfile     : output file\n");
    return 0;
}


// mode 0 - decode

static int LTE_Fdd_Analysis_Process_Option
(
    int argc,
    char *argv[],
    struct LTE_Fdd_Analysis_s *ctx
)
{
    int option_index = 0;
    char * endp;

    ctx->analysis_mode = LTE_ANALYSIS_MODE_DECODE;
    ctx->use_lowpass_filter = 0;
    char ch;

    while(1) {

        static struct option cmd_line_options[] = {
            {"mode",            required_argument,      0, 'm'            },
            {"filter",          no_argument,            0, 'f'            },
            {"help",            no_argument,            0, 'h'            },
            {0, 0, 0, 0}
        };

        ch = getopt_long (argc, (char * const *)argv, "hfm:", cmd_line_options, &option_index);
        if (ch == -1) {
            break;
        }

        switch(ch) {
        case 'm':
            ctx->analysis_mode = strtoul(optarg, &endp, 10);
            if ((optarg == endp)||(*endp != '\0')) {
                fprintf(stderr, "Error: could not parse samplerate\n");
                return -1;
            }
            break;

        case 'f':
            ctx->use_lowpass_filter = 1;
            break;

        case 'h':
            return -1;
        }

    }

    if(ctx->analysis_mode == LTE_ANALYSIS_MODE_ENCODE || ctx->analysis_mode == LTE_ANALYSIS_MODE_DECODE) {
        if (optind >= argc) {
            fprintf(stderr, "Filename Required !!!!\n");
            return -1;
        }
        snprintf(ctx->file_name, MAX_FILE_NAME_LENGTH - 1, "%s", argv[optind]);
        return 0;
    }
    else {
        return -1;
    }

}

static int LTE_Fdd_Analysis_Get_File_Len(char *file_name)
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


static int32_t LTE_Fdd_Analysis_Copy_Input_to_Samplebuff(struct LTE_Fdd_Analysis_s *ctx)
{
    int32_t i;
    int32_t n_samples;
    int32_t read_bytes;

    float total_power;
    float avg_power;
    float scale;


#ifdef    LIBLTE_PHY_ANALYSIS
    int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_LOAD_BUFF);
#endif /* LIBLTE_PHY_ANALYSIS */

    n_samples = LTE_FDD_DL_FS_SAMP_BUF_NUM_FRAMES * ctx->phy_struct->N_samps_per_frame;

    ctx->input_sample_offset = ctx->input_sample_offset - (ctx->samp_buf_w_idx - ctx->samp_buf_r_idx);

    fseek(ctx->fInput, ctx->input_sample_offset * 2 * sizeof(float), SEEK_SET);
    read_bytes = fread(ctx->in_data, 1, n_samples * 2 * sizeof(float), ctx->fInput);
    if(read_bytes < (n_samples * 2 * sizeof(float))) {
        return -1;
    }

    if(ctx->use_lowpass_filter) {
        /* Low Pass Filter */
        int32_t filter_order = sizeof(chn_6RB_filter_30_72MPS)/sizeof(float);
        filter_n(
            chn_6RB_filter_30_72MPS, filter_order,
            ctx->in_data, ctx->filtered_ata, n_samples - filter_order
        );

        for(i = 0; i < n_samples; i++) {
            ctx->i_buf[i] = ctx->filtered_ata[2*i];
            ctx->q_buf[i] = ctx->filtered_ata[2*i+1];
        }
    }
    else {
        for(i = 0; i < n_samples; i++) {
            ctx->i_buf[i] = ctx->in_data[2*i];
            ctx->q_buf[i] = ctx->in_data[2*i+1];
        }
    }

    total_power = 0.0;
    for(i = 0; i < n_samples; i++) {
        total_power +=  ctx->i_buf[i] * ctx->i_buf[i] + ctx->q_buf[i] * ctx->q_buf[i];
    }
    avg_power = total_power / n_samples;
    scale = 1.0 / sqrt(avg_power);

#ifdef    LIBLTE_PHY_ANALYSIS
    if(log_msg_enable) {
        fprintf(stderr, "[1] Avg Power : %.2f dB (%.2f)\n", 10.0 * log10(avg_power), avg_power);
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    ctx->input_sample_offset = ctx->input_sample_offset + n_samples;
    ctx->samp_buf_w_idx = n_samples;
    ctx->samp_buf_r_idx = 0;
    ctx->process_samples = true;
    ctx->copy_input = false;

#ifdef    LIBLTE_PHY_ANALYSIS
    if(log_msg_enable) {
        fprintf(stderr, "\nCopy_Input_to_Samplebuff\n");
        fprintf(stderr, "    input_sample_offset = 0x%08x (%d)\n", ctx->input_sample_offset, ctx->input_sample_offset);
        fprintf(stderr, "    samp_buf_w_idx = 0x%08x (%d)\n", ctx->samp_buf_w_idx, ctx->samp_buf_w_idx);
        fprintf(stderr, "    samp_buf_r_idx = 0x%08x (%d)\n", ctx->samp_buf_r_idx, ctx->samp_buf_r_idx);
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    return 0;

}

static int32_t LTE_Fdd_Analysis_Init_Frame(struct LTE_Fdd_Analysis_s *ctx)
{
    ctx->state                      = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
    ctx->num_samps_needed           = ctx->phy_struct->N_samps_per_subfr * PSS_AND_FINE_TIMING_SEARCH_NUM_SUBFRAMES;
    ctx->samp_buf_w_idx             = 0;
    ctx->samp_buf_r_idx             = 0;

    ctx->sfn                        = 0;
    ctx->N_sfr                      = 0;
    ctx->N_ant                      = 0;
    ctx->N_id_cell                  = 0;
    ctx->N_id_1                     = 0;
    ctx->N_id_2                     = 0;

    ctx->prev_si_value_tag          = 0;
    ctx->prev_si_value_tag_valid    = false;

    ctx->mib_printed                = false;
    ctx->sib1_printed               = false;
    ctx->sib2_printed               = false;

    ctx->sib3_printed               = false;
    ctx->sib3_expected              = false;

    ctx->sib4_printed               = false;
    ctx->sib4_expected              = false;

    ctx->sib5_printed               = false;
    ctx->sib5_expected              = false;

    ctx->sib6_printed               = false;
    ctx->sib6_expected              = false;

    ctx->sib7_printed               = false;
    ctx->sib7_expected              = false;

    ctx->sib8_printed               = false;
    ctx->sib8_expected              = false;

    ctx->sib13_printed              = false;
    ctx->sib13_expected             = false;

#ifdef    LIBLTE_PHY_ANALYSIS
    if(LOG_MSG_ENABLE(LOG_MSG_CODE_INIT)) {
        fprintf(stderr, "\ninit\n");
        fprintf(stderr, "    phich_res = %.2f\n", ctx->phich_res);
        fprintf(stderr, "    sfn = %d\n", ctx->sfn);
        fprintf(stderr, "    N_sfr = %d\n", ctx->N_sfr);
        fprintf(stderr, "    N_ant = %d\n", ctx->N_ant);
        fprintf(stderr, "    N_id_cell = %d\n", ctx->N_id_cell);
        fprintf(stderr, "    N_id_1 = %d\n", ctx->N_id_1);
        fprintf(stderr, "    N_id_2 = %d\n", ctx->N_id_2);
    }
#endif /* LIBLTE_PHY_ANALYSIS */

}

static int32_t LTE_Fdd_Analysis_freq_shift(struct LTE_Fdd_Analysis_s *ctx, int32_t start_idx, int32_t num_samps, float freq_offset)
{
    float  f_samp_re;
    float  f_samp_im;
    float  tmp_i;
    float  tmp_q;
    uint32_t i;

    for(i = start_idx; i <(start_idx + num_samps); i++) {
        f_samp_re = cosf((i+1)*(freq_offset)*2*M_PI/ctx->phy_struct->fs);
        f_samp_im = sinf((i+1)*(freq_offset)*2*M_PI/ctx->phy_struct->fs);
        tmp_i     = ctx->i_buf[i];
        tmp_q     = ctx->q_buf[i];
        ctx->i_buf[i]  = tmp_i*f_samp_re + tmp_q*f_samp_im;
        ctx->q_buf[i]  = tmp_q*f_samp_re - tmp_i*f_samp_im;
    }

    return 0;
}


static int32_t LTE_Fdd_Analysis_Coarse_Timing_Search(struct LTE_Fdd_Analysis_s *ctx)
{

    LIBLTE_ERROR_ENUM result;

#ifdef    LIBLTE_PHY_ANALYSIS
    int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_SEARCH_COARSE_TIMING);
    if(log_msg_enable) {
        fprintf(stderr, "\nCoarse_Timing_Search\n");
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    result = liblte_phy_dl_find_coarse_timing_and_freq_offset_n(
                ctx->phy_struct, ctx->i_buf, ctx->q_buf, COARSE_TIMING_N_SLOTS, &ctx->timing_struct
            );

    if(LIBLTE_SUCCESS == result) {
        /* Search for PSS and fine timing */
#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "goto  Search for SSS\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH;
        ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;

    }
    else {
        /* Stay in coarse timing search */
        ctx->samp_buf_r_idx += ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
        ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
    }

    return result;
}


static int32_t LTE_Fdd_Analysis_PSS_And_Fine_Timing_Search(struct LTE_Fdd_Analysis_s *ctx)
{

    LIBLTE_ERROR_ENUM result;
    float freq_offset;
    int32_t nid2;
    int32_t i;
    int32_t j;
    int32_t success_cnt;

#ifdef    LIBLTE_PHY_ANALYSIS
    int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_PSS_AND_FINE_TIMING);
#endif /* LIBLTE_PHY_ANALYSIS */

    fprintf(stderr, "\nLTE_Fdd_Analysis_PSS_And_Fine_Timing_Search\n");
#ifdef    LIBLTE_PHY_ANALYSIS
    if(log_msg_enable) {
        fprintf(stderr, "\n");
        fprintf(stderr, "timing_struct->n_corr_peaks = %d\n", ctx->timing_struct.n_corr_peaks);
        for(i = 0; i < ctx->timing_struct.n_corr_peaks; i ++) {
            fprintf(stderr, "PEAK %d : nid2 = %d,", i, ctx->timing_struct.nid2[i]);
            for(j = 0; j <= PSS_SYMBOL_NUM; j++) {
                fprintf(stderr, " [%d] %-8d", j, ctx->timing_struct.symb_starts[i][j]);
            }
            fprintf(stderr, "\n");
        }
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    success_cnt = 0;

    for(i = 0; i < ctx->timing_struct.n_corr_peaks; i++) {

        result = liblte_phy_find_pss_and_fine_timing_n(
                    ctx->phy_struct,
                    ctx->i_buf,
                    ctx->q_buf,
                    ctx->timing_struct.symb_starts[i],
                    &ctx->N_id_2,
                    &ctx->pss_symb,
                    &ctx->pss_thresh,
                    &freq_offset
                );

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "freq_offset = %.2f\n", freq_offset);
        }
#endif /* LIBLTE_PHY_ANALYSIS */


        if(LIBLTE_SUCCESS == result) {
            // Search for SSS
            ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH;
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * SSS_SEARCH_NUM_SUBFRAMES;

            success_cnt++;
            break;
        }
    }

#ifdef    LIBLTE_PHY_ANALYSIS
    if(log_msg_enable) {
        fprintf(stderr, "###### PSS_And_Fine_Timing_Search : DONE #####\n");
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    if(success_cnt > 0) {

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "goto  Search for SSS\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        // Search for SSS
        ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH;
        ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * SSS_SEARCH_NUM_SUBFRAMES;
    }
    else {
        // Go back to coarse timing search
#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "Go back to coarse timing search\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */
        ctx->state             = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
        ctx->samp_buf_r_idx   += ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
        ctx->num_samps_needed  = ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
    }

    return 0;

}

static int32_t LTE_Fdd_Analysis_SSS_Search(struct LTE_Fdd_Analysis_s *ctx)
{

    LIBLTE_ERROR_ENUM   result = LIBLTE_ERROR_INVALID_INPUTS;

    int32_t i;
    uint32_t frame_start_idx;
    int32_t success_cnt;
    int32_t nid2;

#ifdef    LIBLTE_PHY_ANALYSIS
    int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_SSS);
    if(log_msg_enable) {
        fprintf(stderr, "\n###### LTE_Fdd_Analysis_SSS_Search ####\n");
        fprintf(stderr, "    N_id_2 = %d\n", ctx->N_id_2);
        fprintf(stderr, "    pss_thresh = %.2f\n", ctx->pss_thresh);
        fprintf(stderr, "    N_decoded_chans = %d\n", ctx->N_decoded_chans);
    }
#endif /* LIBLTE_PHY_ANALYSIS */


    result = liblte_phy_find_sss_n(
                    ctx->phy_struct,
                    ctx->i_buf,
                    ctx->q_buf,
                    ctx->N_id_2,
                    ctx->timing_struct.symb_starts[i],
                    ctx->pss_thresh,
                    &ctx->N_id_1,
                    &frame_start_idx
                );


    if(LIBLTE_SUCCESS == result) {
        ctx->N_id_cell = 3*ctx->N_id_1 + ctx->N_id_2;

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "###### SSS_Search : DONE ####\n");
            fprintf(stderr, "    N_id_cell = %d\n", ctx->N_id_cell);
            fprintf(stderr, "    N_id_1 = %d\n", ctx->N_id_1);
            fprintf(stderr, "    N_id_2 = %d\n", ctx->N_id_2);
            fprintf(stderr, "    frame_start_idx = %d\n", frame_start_idx);
            fprintf(stderr, "    N_decoded_chans = %d\n", ctx->N_decoded_chans);
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        for(i=0; i<ctx->N_decoded_chans; i++) {
            if(ctx->N_id_cell == ctx->decoded_chans[i]) {
                break;
            }
        }

        if(i != ctx->N_decoded_chans) {
            // Go back to coarse timing search
#ifdef    LIBLTE_PHY_ANALYSIS
            if(log_msg_enable) {
                fprintf(stderr, "\nGo back to coarse timing search\n");
            }
#endif /* LIBLTE_PHY_ANALYSIS */
            ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
            ctx->corr_peak_idx++;
            LTE_Fdd_Analysis_Init_Frame(ctx);
        }
        else{

            ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE;
            while(frame_start_idx < ctx->samp_buf_r_idx) {
                frame_start_idx += ctx->phy_struct->N_samps_per_frame;
            }
            ctx->samp_buf_r_idx = frame_start_idx;
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * BCH_DECODE_NUM_FRAMES;

            // Decode BCH
#ifdef    LIBLTE_PHY_ANALYSIS
            if(log_msg_enable) {
                fprintf(stderr, "goto BCH_DECODE\n");
                fprintf(stderr, "    samp_buf_r_idx = %7d\n", ctx->samp_buf_r_idx);
                fprintf(stderr, "    samp_buf_w_idx = %7d\n", ctx->samp_buf_w_idx);
            }
#endif /* LIBLTE_PHY_ANALYSIS */
        }

    }
    else {

        // Go back to coarse timing search
        ctx->state             = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
        ctx->samp_buf_r_idx   += ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
        ctx->num_samps_needed  = ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
    }

    return result;

}

static int32_t LTE_Fdd_Analysis_BCH_Decode(struct LTE_Fdd_Analysis_s *ctx)
{

    LIBLTE_ERROR_ENUM result;
    uint32_t  N_rb_dl;

#ifdef    LIBLTE_PHY_ANALYSIS
    if(LOG_MSG_ENABLE(LOG_MSG_CODE_BCCH)) {
        fprintf(stderr, "\n###### BCH_Decode ####\n");
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    ctx->phy_struct->bch_mod_antenna = 0;

    if( LIBLTE_SUCCESS == liblte_phy_get_dl_subframe_and_ce(ctx->phy_struct,
                                                            ctx->i_buf,
                                                            ctx->q_buf,
                                                            ctx->samp_buf_r_idx,
                                                            0,
                                                            ctx->N_id_cell,
                                                            4,
                                                            &ctx->subframe ) &&

        LIBLTE_SUCCESS == liblte_phy_bch_channel_decode(ctx->phy_struct,
                                                        &ctx->subframe,
                                                        ctx->N_id_cell,
                                                        &ctx->N_ant,
                                                        ctx->rrc_msg.msg,
                                                        &ctx->rrc_msg.N_bits,
                                                        &ctx->sfn_offset ) &&

        LIBLTE_SUCCESS == liblte_rrc_unpack_bcch_bch_msg(&ctx->rrc_msg, &ctx->mib)) {

        switch(ctx->mib.dl_bw)
        {
        case LIBLTE_RRC_DL_BANDWIDTH_6:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_1_4MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_15:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_3MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_25:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_5MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_50:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_10MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_75:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_15MHZ;
            break;
        case LIBLTE_RRC_DL_BANDWIDTH_100:
            N_rb_dl = LIBLTE_PHY_N_RB_DL_20MHZ;
            break;
        }

        liblte_phy_update_n_rb_dl(ctx->phy_struct, N_rb_dl);

        ctx->sfn       = (ctx->mib.sfn_div_4 << 2) + ctx->sfn_offset;
        ctx->phich_res = liblte_rrc_phich_resource_num[ctx->mib.phich_config.res];

        LTE_Fdd_Analysis_print_mib(ctx, &ctx->mib);

        if(ctx->phy_struct->bch_mod_antenna != 0) {
            LIBLTE_PHY_STRUCT *phy_struct = ctx->phy_struct;

            int p = phy_struct->bch_mod_antenna - 1;
#if 0
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_BCCH)) {
                fprintf(stderr, "\n###### BCH SYMBOL ####\n");
                fprintf(stderr, "    num of symbols = %d\n", phy_struct->bch_n_mod_sym[p]);
                print_complex_number_normalized(phy_struct->bch_mod_sym_re[p], phy_struct->bch_mod_sym_im[p], phy_struct->bch_n_mod_sym[p], 0);
            }
#endif
        }

        // Add this channel to the list of decoded channels
        ctx->decoded_chans[ctx->N_decoded_chans++] = ctx->N_id_cell;

        if(LOG_MSG_ENABLE(LOG_MSG_CODE_BCCH)) {
            fprintf(stderr, "ctx->N_decoded_chans = %d\n", ctx->N_decoded_chans);
            for(int32_t chan = 0; chan < ctx->N_decoded_chans; chan++) {
                fprintf(stderr, "    [%02d] %4d", chan, ctx->decoded_chans[chan]);
            }
            fprintf(stderr, "\n");
        }

        if(LTE_FDD_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX == ctx->N_decoded_chans) {
            ctx->done_flag = -1;
        }

        // Decode PDSCH for SIB1
        ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1;
        if((ctx->sfn % 2) != 0)  {
            ctx->samp_buf_r_idx += ctx->phy_struct->N_samps_per_frame;
            ctx->sfn++;
        }
        ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;

#ifdef    LIBLTE_PHY_ANALYSIS
        if(LOG_MSG_ENABLE(LOG_MSG_CODE_BCCH)) {
            fprintf(stderr, "BCH_Decode : Done\n");
            fprintf(stderr, "Goto Decode PDSCH for SIB1\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

    }
    else {
#ifdef    LIBLTE_PHY_ANALYSIS
        if(LOG_MSG_ENABLE(LOG_MSG_CODE_BCCH)) {
            fprintf(stderr, "Go back to coarse timing search\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */
        ctx->state             = LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH;
        ctx->samp_buf_r_idx   += ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
        ctx->num_samps_needed  = ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
        return 0;
    }
    return 0;
}

static int32_t LTE_Fdd_Analysis_PDSCH_Decode_SIB1(struct LTE_Fdd_Analysis_s *ctx)
{
    int32_t i;
    int32_t result = LIBLTE_SUCCESS;
    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH);

    if(result == LIBLTE_SUCCESS) {

        result = liblte_phy_get_dl_subframe_and_ce(ctx->phy_struct,
                                                   ctx->i_buf,
                                                   ctx->q_buf,
                                                   ctx->samp_buf_r_idx,
                                                   5,
                                                   ctx->N_id_cell,
                                                   ctx->N_ant,
                                                   &ctx->subframe);
    }

    if(result == LIBLTE_SUCCESS) {

        ctx->phy_struct->pdcch_alloc = 0;
        result = liblte_phy_pdcch_channel_decode(ctx->phy_struct,
                                                 &ctx->subframe,
                                                 ctx->N_id_cell,
                                                 ctx->N_ant,
                                                 ctx->phich_res,
                                                 ctx->mib.phich_config.dur,
                                                 &ctx->pcfich,
                                                 &ctx->phich,
                                                 &ctx->pdcch);

        if(log_msg_enabled) {
            fprintf(stderr, "\nLTE_Fdd_Analysis_PDSCH_Decode_SIB1\n");
            fprintf(stderr, "liblte_phy_pdcch_channel_decode RESULT\n");
            fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);

            for(i = 0; i < ctx->phy_struct->pdcch_alloc; i++) {
                fprintf(stderr, "    PDCCH Modulation Symbol [%d] \n", i);
                fprintf(stderr, "        phy_struct->pdcch_mod_type = %d (%s)\n", ctx->phy_struct->pdcch_mod_type[i], liblte_phy_modulation_type_text[ctx->phy_struct->pdcch_mod_type[i]]);
                fprintf(stderr, "        Number of Symbol = %d\n", ctx->phy_struct->pdcch_n_mod_sym[i]);
#if 0
                print_complex_number_normalized(&ctx->phy_struct->pdcch_mod_sym_re[i][0], &ctx->phy_struct->pdcch_mod_sym_im[i][0], ctx->phy_struct->pdcch_n_mod_sym[i], 0);
#endif
            }
        }

    }

    if(result == LIBLTE_SUCCESS) {

        if(log_msg_enabled) {
            fprintf(stderr, "\nLTE_Fdd_Analysis_PDSCH_Decode_SIB1\n");
            fprintf(stderr, "    call liblte_phy_pdsch_channel_decode\n");
            fprintf(stderr, "    samp_buf_r_idx = %d\n", ctx->samp_buf_r_idx);
            fprintf(stderr, "    samp_buf_w_idx = %d\n", ctx->samp_buf_w_idx);
            fprintf(stderr, "    pdcch.N_symbs = %d\n", ctx->pdcch.N_symbs);
            fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
            for(i = 0; i < ctx->pdcch.N_alloc; i++) {
                LIBLTE_PHY_ALLOCATION_STRUCT *alloc;
                alloc = &ctx->pdcch.alloc[i];
                fprintf(stderr, "    alloc[%d].N_prb = %d\n", i, alloc->N_prb);
                fprintf(stderr, "    alloc[%d].mcs = %d\n", i, alloc->mcs);
                fprintf(stderr, "    alloc[%d].rv_idx = %d\n", i, alloc->rv_idx);
                fprintf(stderr, "    alloc[%d].mod_type = %d (%s)\n", i, alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
            }
        }

        ctx->phy_struct->pdsch_n_mod_sym = 0;
        result = liblte_phy_pdsch_channel_decode(ctx->phy_struct,
                                                 &ctx->subframe,
                                                 &ctx->pdcch.alloc[0],
                                                 ctx->pdcch.N_symbs,
                                                 ctx->N_id_cell,
                                                 ctx->N_ant,
                                                 ctx->rrc_msg.msg,
                                                 &ctx->rrc_msg.N_bits);

        if(log_msg_enabled) {
            if(result == LIBLTE_SUCCESS) {
#if 0                 
                if(ctx->phy_struct->pdsch_n_mod_sym != 0) {
                    fprintf(stderr, "\n    PDSCH Modulation Symbol\n");
                    fprintf(stderr, "        phy_struct->pdcch_mod_type = %s (%d)\n", liblte_phy_modulation_type_text[ctx->phy_struct->pdsch_mod_type], ctx->phy_struct->pdsch_mod_type);
                    fprintf(stderr, "        Number of Symbol = %d\n", ctx->phy_struct->pdsch_n_mod_sym);
                    print_complex_number_normalized(&ctx->phy_struct->pdsch_mod_sym_re[0], &ctx->phy_struct->pdsch_mod_sym_im[0], ctx->phy_struct->pdsch_n_mod_sym, 0);

                }
#endif
            }

        }

    }

    if(result == LIBLTE_SUCCESS) {
        result = liblte_rrc_unpack_bcch_dlsch_msg(&ctx->rrc_msg, &ctx->bcch_dlsch_msg);
    }

    if(result == LIBLTE_SUCCESS) {
        if(log_msg_enabled) {
            
            fprintf(stderr, "\nLTE_Fdd_Analysis_PDSCH_Decode_SIB1\n");
            fprintf(stderr, "    call liblte_rrc_unpack_bcch_dlsch_msg\n");
            fprintf(stderr, "    samp_buf_r_idx = %d\n", ctx->samp_buf_r_idx);
            fprintf(stderr, "    samp_buf_w_idx = %d\n", ctx->samp_buf_w_idx);

            fprintf(stderr, "\nLTE_Fdd_Analysis_PDSCH_Decode_SIB1 : RESULT\n");
            fprintf(stderr, "    rrc_msg.N_bits = %d\n", ctx->rrc_msg.N_bits);
            print_bits((uint8_t *)ctx->rrc_msg.msg, ctx->rrc_msg.N_bits);            
        }

        if(1 == ctx->bcch_dlsch_msg.N_sibs && LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1 == ctx->bcch_dlsch_msg.sibs[0].sib_type) {
            LTE_Fdd_Analysis_print_sib1(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *)&ctx->bcch_dlsch_msg.sibs[0].sib);
        }

        // Decode all PDSCHs
        if(log_msg_enabled) {
            fprintf(stderr, "\nDecode all PDSCHs\n");
        }

        ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC;
        ctx->N_sfr = 0;
        ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
        
    }
    else {
        // Try to decode SIB1 again
        if(log_msg_enabled) {
            fprintf(stderr, "\nTry to decode SIB1 again\n");
        }

        ctx->samp_buf_r_idx   += ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;
        ctx->sfn              += 2;
        ctx->num_samps_needed  = ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;

    }

    return 0;
}

static int32_t LTE_Fdd_Analysis_PDSCH_Decode_SI_Generic(struct LTE_Fdd_Analysis_s *ctx)
{

    int i;
    int32_t result;
    int32_t log_msg_enabled;

    log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH);

    result = LIBLTE_SUCCESS;
    if(result == LIBLTE_SUCCESS) {

        result = liblte_phy_get_dl_subframe_and_ce(ctx->phy_struct,
                                                   ctx->i_buf,
                                                   ctx->q_buf,
                                                   ctx->samp_buf_r_idx,
                                                   ctx->N_sfr,
                                                   ctx->N_id_cell,
                                                   ctx->N_ant,
                                                   &ctx->subframe);
    }

    if(result == LIBLTE_SUCCESS) {
        
        ctx->phy_struct->pdcch_alloc = 0;        
        check_decode_pdcch_channel
        (
            ctx->phy_struct,
            &ctx->subframe,
            ctx->N_id_cell,
            ctx->N_ant,
            ctx->phich_res,
            ctx->mib.phich_config.dur,
            &ctx->pcfich,
            &ctx->phich,
            &ctx->pdcch
        );

        check_decode_pdsch_channel(ctx->phy_struct,
                                   &ctx->subframe,
                                   &ctx->pdcch,
                                   ctx->pdcch.N_symbs,
                                   ctx->N_id_cell,
                                   ctx->N_ant);    

                                   

    }    

    if(result == LIBLTE_SUCCESS) {

        ctx->phy_struct->pdcch_alloc = 0;
        result = liblte_phy_pdcch_channel_decode(ctx->phy_struct,
                                                 &ctx->subframe,
                                                 ctx->N_id_cell,
                                                 ctx->N_ant,
                                                 ctx->phich_res,
                                                 ctx->mib.phich_config.dur,
                                                 &ctx->pcfich,
                                                 &ctx->phich,
                                                 &ctx->pdcch);
    }

    if(result == LIBLTE_SUCCESS) {

        if(log_msg_enabled) {
            fprintf(stderr, "\nLTE_Fdd_Analysis_PDSCH_Decode_SI_Generic\n");
            fprintf(stderr, "    call liblte_phy_pdsch_channel_decode\n");
            fprintf(stderr, "    samp_buf_r_idx = %d\n", ctx->samp_buf_r_idx);
            fprintf(stderr, "    samp_buf_w_idx = %d\n", ctx->samp_buf_w_idx);
        }

        ctx->phy_struct->pdsch_n_mod_sym = 0;

        result = liblte_phy_pdsch_channel_decode(ctx->phy_struct,
                                                 &ctx->subframe,
                                                 &ctx->pdcch.alloc[0],
                                                 ctx->pdcch.N_symbs,
                                                 ctx->N_id_cell,
                                                 ctx->N_ant,
                                                 ctx->rrc_msg.msg,
                                                 &ctx->rrc_msg.N_bits);

        if(ctx->phy_struct->pdsch_n_mod_sym != 0) {            
            if(log_msg_enabled) {
                fprintf(stderr, "\n    PDSCH Modulation Symbol\n");
                fprintf(stderr, "        phy_struct->pdcch_mod_type = %s (%d)\n", liblte_phy_modulation_type_text[ctx->phy_struct->pdsch_mod_type], ctx->phy_struct->pdsch_mod_type);
                fprintf(stderr, "        Number of Symbol = %d\n", ctx->phy_struct->pdsch_n_mod_sym);
            }
        }

    }

    if(result == LIBLTE_SUCCESS) {

#ifdef    LIBLTE_PHY_ANALYSIS
        if(LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH)) {
            fprintf(stderr, "pdcch.alloc[0].rnti = %d / %d\n", ctx->pdcch.alloc[0].rnti, LIBLTE_MAC_SI_RNTI);
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        if(LIBLTE_MAC_SI_RNTI == ctx->pdcch.alloc[0].rnti && LIBLTE_SUCCESS == liblte_rrc_unpack_bcch_dlsch_msg(&ctx->rrc_msg, &ctx->bcch_dlsch_msg))  {

#ifdef    LIBLTE_PHY_ANALYSIS
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH)) {
                fprintf(stderr, "bcch_dlsch_msg.N_sibs = %d\n", ctx->bcch_dlsch_msg.N_sibs);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            for(i = 0; i < ctx->bcch_dlsch_msg.N_sibs; i++) {

                if(LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH)) {
                    fprintf(stderr, "ctx->bcch_dlsch_msg.sibs[%d].sib_type = %d (%s)\n", 
                        i, ctx->bcch_dlsch_msg.sibs[i].sib_type, liblte_rrc_sys_info_block_type_text[ctx->bcch_dlsch_msg.sibs[i].sib_type]);
                }
            
                switch(ctx->bcch_dlsch_msg.sibs[i].sib_type) {
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1:
                    LTE_Fdd_Analysis_print_sib1(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2:
                    LTE_Fdd_Analysis_print_sib2(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3:
                    LTE_Fdd_Analysis_print_sib3(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4:
                    LTE_Fdd_Analysis_print_sib4(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5:
                    LTE_Fdd_Analysis_print_sib5(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6:
                    LTE_Fdd_Analysis_print_sib6(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7:
                    LTE_Fdd_Analysis_print_sib7(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8:
                    LTE_Fdd_Analysis_print_sib8(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13:
                    LTE_Fdd_Analysis_print_sib13(ctx, (LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13_STRUCT *)&ctx->bcch_dlsch_msg.sibs[i].sib);
                    break;
                default:
#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH)) {
                        fprintf(stderr, "Not handling SIB %u\n", ctx->bcch_dlsch_msg.sibs[i].sib_type);
                    }
#endif /* LIBLTE_PHY_ANALYSIS */
                    break;
                }
            }

        }
        else if(LIBLTE_MAC_P_RNTI == ctx->pdcch.alloc[0].rnti) {
            for(i = 0; i < 8; i++) {
                if(ctx->rrc_msg.msg[i] != liblte_rrc_test_fill[i]) {
                    break;
                }
            }
            if(i == 8) {
                fprintf(stderr, "\nTEST FILL RECEIVED\n");
                fprintf(stderr, "    rrc_msg.N_bits = %d\n", ctx->rrc_msg.N_bits);
                print_bits((uint8_t *)ctx->rrc_msg.msg, ctx->rrc_msg.N_bits);
            }
            else if(LIBLTE_SUCCESS == liblte_rrc_unpack_pcch_msg(&ctx->rrc_msg, &ctx->pcch_msg)) {
                LTE_Fdd_Analysis_print_page(ctx, &ctx->pcch_msg);
            }
        }
        else {
            if(log_msg_enabled) {
                fprintf(stderr, "MESSAGE RECEIVED FOR RNTI=%04X: ", ctx->pdcch.alloc[0].rnti);
                for(i = 0; i < ctx->rrc_msg.N_bits; i++) {
                    fprintf(stderr, "%u", ctx->rrc_msg.msg[i]);
                }
                fprintf(stderr, "\n");
            }
        }
    }

    // Keep trying to decode PDSCHs
    ctx->state = LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC;
    ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
    ctx->N_sfr++;
    if(ctx->N_sfr >= 10) {
        ctx->N_sfr = 0;
        ctx->sfn++;
        ctx->samp_buf_r_idx += ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
    }

#ifdef    LIBLTE_PHY_ANALYSIS
    if(log_msg_enabled) {
        fprintf(stderr, "\nKeep trying to decode PDSCH\n");
        fprintf(stderr, "    N_sfr = %d\n", ctx->N_sfr);
        fprintf(stderr, "    sfn = %d\n", ctx->sfn);
        fprintf(stderr, "    samp_buf_r_idx = %d\n", ctx->samp_buf_r_idx);
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    return 0;
}


static int32_t LTE_Fdd_Analysis_Process_State(struct LTE_Fdd_Analysis_s *ctx)
{
    int i;

    for(i = 0; LTE_Fdd_Analysis_State_Handler[i].State >= 0; i++) {
        if(LTE_Fdd_Analysis_State_Handler[i].State == ctx->state) {
            return LTE_Fdd_Analysis_State_Handler[i].Handler(ctx);
        }
    }

    return -1;
}

static void LTE_Fdd_Analysis_print_mib(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_MIB_STRUCT *mib)
{
    fprintf(stderr, "\nDL LTE Channel found [%u]:\n", ctx->corr_peak_idx);
    fprintf(stderr, "\tMIB Decoded:\n");
    fprintf(stderr, "    %-40s=%20.2f\n", "Frequency Offset", ctx->timing_struct.freq_offset[ctx->corr_peak_idx]);
    fprintf(stderr, "    %-40s=%20u\n", "System Frame Number", ctx->sfn);
    fprintf(stderr, "    %-40s=%20u\n", "Physical Cell ID", ctx->N_id_cell);
    fprintf(stderr, "    %-40s=%20u\n", "Number of TX Antennas", ctx->N_ant);
    fprintf(stderr, "    %-40s=%16s MHz\n", "Bandwidth", liblte_rrc_dl_bandwidth_text[mib->dl_bw]);
    fprintf(stderr, "    %-40s=%20s\n", "PHICH Duration", liblte_rrc_phich_duration_text[mib->phich_config.dur]);
    fprintf(stderr, "    %-40s=%20s\n", "PHICH Resource", liblte_rrc_phich_resource_text[mib->phich_config.res]);

    ctx->mib_printed = true;
}


static void LTE_Fdd_Analysis_print_sib1(struct LTE_Fdd_Analysis_s *ctx,LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *sib1)
{
    uint32 i;
    uint32 j;
    uint32 si_win_len;
    uint32 si_periodicity_T;
    uint16 mnc;

    if(true == ctx->prev_si_value_tag_valid && ctx->prev_si_value_tag != sib1->system_info_value_tag)  {        
        fprintf(stderr, "System Info value tag changed\n");
        ctx->sib1_printed  = false;
        ctx->sib2_printed  = false;
        ctx->sib3_printed  = false;
        ctx->sib4_printed  = false;
        ctx->sib5_printed  = false;
        ctx->sib6_printed  = false;
        ctx->sib7_printed  = false;
        ctx->sib8_printed  = false;
        ctx->sib13_printed = false;
    }

    if(false == ctx->sib1_printed) {
        fprintf(stderr, "SIB1 Decoded:\n");
        fprintf(stderr, "    %-40s\n", "PLMN Identity List:");
        for(i=0; i<sib1->N_plmn_ids; i++) {
            fprintf(stderr, "        %03X-", sib1->plmn_id[i].id.mcc & 0x0FFF);
            if((sib1->plmn_id[i].id.mnc & 0xFF00) == 0xFF00) {
                mnc = sib1->plmn_id[i].id.mnc & 0x00FF;
                fprintf(stderr, "%02X, ", mnc);
            }
            else {
                mnc = sib1->plmn_id[i].id.mnc & 0x0FFF;
                fprintf(stderr, "%03X, ", mnc);
            }
            for(j = 0; j < LIBLTE_MCC_MNC_LIST_N_ITEMS; j++) {
                if(liblte_mcc_mnc_list[j].mcc == (sib1->plmn_id[i].id.mcc & 0x0FFF) &&
                   liblte_mcc_mnc_list[j].mnc == mnc) {
                    fprintf(stderr, "%s, ", liblte_mcc_mnc_list[j].net_name);
                    break;
                }
            }

            if(LIBLTE_RRC_RESV_FOR_OPER == sib1->plmn_id[i].resv_for_oper) {
                fprintf(stderr, "reserved for operator use\n");
            }
            else {
                fprintf(stderr, "not reserved for operator use\n");
            }
        }
        fprintf(stderr, "    %-40s=%20u\n", "Tracking Area Code", sib1->tracking_area_code);
        fprintf(stderr, "    %-40s=%20u\n", "Cell Identity", sib1->cell_id);

        switch(sib1->cell_barred) {
        case LIBLTE_RRC_CELL_BARRED:
            fprintf(stderr, "    %-40s=%20s\n", "Cell Barred", "Barred");
            break;
        case LIBLTE_RRC_CELL_NOT_BARRED:
            fprintf(stderr, "    %-40s=%20s\n", "Cell Barred", "Not Barred");
            break;
        }
        switch(sib1->intra_freq_reselection) {
        case LIBLTE_RRC_INTRA_FREQ_RESELECTION_ALLOWED:
            fprintf(stderr, "    %-40s=%20s\n", "Intra Frequency Reselection", "Allowed");
            break;
        case LIBLTE_RRC_INTRA_FREQ_RESELECTION_NOT_ALLOWED:
            fprintf(stderr, "    %-40s=%20s\n", "Intra Frequency Reselection", "Not Allowed");
            break;
        }

        if(true == sib1->csg_indication) {
            fprintf(stderr, "    %-40s=%20s\n", "CSG Indication", "TRUE");
        }
        else{
            fprintf(stderr, "    %-40s=%20s\n", "CSG Indication", "FALSE");
        }

        if(LIBLTE_RRC_CSG_IDENTITY_NOT_PRESENT != sib1->csg_id) {
            fprintf(stderr, "    %-40s=%20u\n", "CSG Identity", sib1->csg_id);
        }
        fprintf(stderr, "    %-40s=%16d dBm\n", "Q Rx Lev Min", sib1->q_rx_lev_min);
        fprintf(stderr, "    %-40s=%17u dB\n", "Q Rx Lev Min Offset", sib1->q_rx_lev_min_offset);
        if(true == sib1->p_max_present) {
            fprintf(stderr, "    %-40s=%16d dBm\n", "P Max", sib1->p_max);
        }
        fprintf(stderr, "    %-40s=%20u\n", "Frequency Band", sib1->freq_band_indicator);
        fprintf(stderr, "    %-40s=%17s ms\n", "SI Window Length", liblte_rrc_si_window_length_text[sib1->si_window_length]);
        si_win_len = liblte_rrc_si_window_length_num[sib1->si_window_length];
        fprintf(stderr, "    %-40s\n", "Scheduling Info List:");
        for(i=0; i<sib1->N_sched_info; i++) {
            fprintf(stderr, "        %s = %s frames\n", "SI Periodicity", liblte_rrc_si_periodicity_text[sib1->sched_info[i].si_periodicity]);
            si_periodicity_T = liblte_rrc_si_periodicity_num[sib1->sched_info[i].si_periodicity];
            fprintf(stderr, "        SI Window Starts at N_subframe = %u, SFN mod %u = %u\n", (i * si_win_len) % 10, si_periodicity_T, (i * si_win_len)/10);
            if(0 == i) {
                fprintf(stderr, "            %s = %s\n", "SIB Type", "2");
            }
            for(j=0; j<sib1->sched_info[i].N_sib_mapping_info; j++) {
                fprintf(stderr, "            %s = %u\n", "SIB Type", liblte_rrc_sib_type_num[sib1->sched_info[i].sib_mapping_info[j].sib_type]);
                switch(sib1->sched_info[i].sib_mapping_info[j].sib_type) {
                case LIBLTE_RRC_SIB_TYPE_3:
                    ctx->sib3_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_4:
                    ctx->sib4_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_5:
                    ctx->sib5_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_6:
                    ctx->sib6_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_7:
                    ctx->sib7_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_8:
                    ctx->sib8_expected = true;
                    break;
                case LIBLTE_RRC_SIB_TYPE_13_v920:
                    ctx->sib13_expected = true;
                    break;
                }
            }
        }

        if(false == sib1->tdd) {
            fprintf(stderr, "    %-40s=%20s\n", "Duplexing Mode", "FDD");
        }
        else {
            fprintf(stderr, "    %-40s=%20s\n", "Duplexing Mode", "TDD");
            fprintf(stderr, "    %-40s=%20s\n", "Subframe Assignment", liblte_rrc_subframe_assignment_text[sib1->tdd_cnfg.sf_assignment]);
            fprintf(stderr, "    %-40s=%20s\n", "Special Subframe Patterns", liblte_rrc_special_subframe_patterns_text[sib1->tdd_cnfg.special_sf_patterns]);
        }

        fprintf(stderr, "    %-40s=%20u\n", "SI Value Tag", sib1->system_info_value_tag);
        ctx->prev_si_value_tag = sib1->system_info_value_tag;
        ctx->prev_si_value_tag_valid = true;
        ctx->sib1_printed = true;

    }

}

static void LTE_Fdd_Analysis_print_sib2(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *sib2)
{
    uint32 coeff = 0;
    uint32 T     = 0;
    uint32 i;

    if(false == ctx->sib2_printed) {
        fprintf(stderr, "SIB2 Decoded:\n");
        if(true == sib2->ac_barring_info_present)
        {
            if(true == sib2->ac_barring_for_emergency)
            {
                fprintf(stderr, "    %-40s=%20s\n", "AC Barring for Emergency", "Barred");
            }else{
                fprintf(stderr, "    %-40s=%20s\n", "AC Barring for Emergency", "Not Barred");
            }
            if(true == sib2->ac_barring_for_mo_signalling.enabled)
            {
                fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Signalling", "Barred");
                fprintf(stderr, "        %-40s=%20s\n", "Factor", liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_signalling.factor]);
                fprintf(stderr, "        %-40s=%19ss\n", "Time", liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_signalling.time]);
                fprintf(stderr, "        %-40s=%20u\n", "Special AC", sib2->ac_barring_for_mo_signalling.for_special_ac);
            }else{
                fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Signalling", "Not Barred");
            }
            if(true == sib2->ac_barring_for_mo_data.enabled)
            {
                fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Data", "Barred");
                fprintf(stderr, "        %-40s=%20s\n", "Factor", liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_data.factor]);
                fprintf(stderr, "        %-40s=%19ss\n", "Time", liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_data.time]);
                fprintf(stderr, "        %-40s=%20u\n", "Special AC", sib2->ac_barring_for_mo_data.for_special_ac);
            }else{
                fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Data", "Not Barred");
            }
        }
        fprintf(stderr, "    %-40s=%20s\n", "Number of RACH Preambles", liblte_rrc_number_of_ra_preambles_text[sib2->rr_config_common_sib.rach_cnfg.num_ra_preambles]);
        if(true == sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present)
        {
            fprintf(stderr, "    %-40s=%20s\n", "Size of RACH Preambles Group A", liblte_rrc_size_of_ra_preambles_group_a_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.size_of_ra]);
            fprintf(stderr, "    %-40s=%15s bits\n", "Message Size Group A", liblte_rrc_message_size_group_a_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.msg_size]);
            fprintf(stderr, "    %-40s=%18sdB\n", "Message Power Offset Group B", liblte_rrc_message_power_offset_group_b_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.msg_pwr_offset_group_b]);
        }
        fprintf(stderr, "    %-40s=%18sdB\n", "Power Ramping Step", liblte_rrc_power_ramping_step_text[sib2->rr_config_common_sib.rach_cnfg.pwr_ramping_step]);
        fprintf(stderr, "    %-40s=%17sdBm\n", "Preamble init target RX power", liblte_rrc_preamble_initial_received_target_power_text[sib2->rr_config_common_sib.rach_cnfg.preamble_init_rx_target_pwr]);
        fprintf(stderr, "    %-40s=%20s\n", "Preamble TX Max", liblte_rrc_preamble_trans_max_text[sib2->rr_config_common_sib.rach_cnfg.preamble_trans_max]);
        fprintf(stderr, "    %-40s=%10s Subframes\n", "RA Response Window Size", liblte_rrc_ra_response_window_size_text[sib2->rr_config_common_sib.rach_cnfg.ra_resp_win_size]);
        fprintf(stderr, "    %-40s=%10s Subframes\n", "MAC Contention Resolution Timer", liblte_rrc_mac_contention_resolution_timer_text[sib2->rr_config_common_sib.rach_cnfg.mac_con_res_timer]);
        fprintf(stderr, "    %-40s=%20u\n", "Max num HARQ TX for Message 3", sib2->rr_config_common_sib.rach_cnfg.max_harq_msg3_tx);
        fprintf(stderr, "    %-40s=%20s\n", "Modification Period Coeff", liblte_rrc_modification_period_coeff_text[sib2->rr_config_common_sib.bcch_cnfg.modification_period_coeff]);
        coeff = liblte_rrc_modification_period_coeff_num[sib2->rr_config_common_sib.bcch_cnfg.modification_period_coeff];
        fprintf(stderr, "    %-40s=%13s Frames\n", "Default Paging Cycle", liblte_rrc_default_paging_cycle_text[sib2->rr_config_common_sib.pcch_cnfg.default_paging_cycle]);
        T = liblte_rrc_default_paging_cycle_num[sib2->rr_config_common_sib.pcch_cnfg.default_paging_cycle];
        fprintf(stderr, "    %-40s=%13u Frames\n", "Modification Period", coeff * T);
        fprintf(stderr, "    %-40s=%13u Frames\n", "nB", (uint32)(T * liblte_rrc_nb_num[sib2->rr_config_common_sib.pcch_cnfg.nB]));
        fprintf(stderr, "    %-40s=%20u\n", "Root Sequence Index", sib2->rr_config_common_sib.prach_cnfg.root_sequence_index);
        fprintf(stderr, "    %-40s=%20u\n", "PRACH Config Index", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index);
        switch(sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index)
        {
        case 0:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 1:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 2:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 3:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 4:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 5:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 6:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 7:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 8:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 9:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 10:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 11:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 12:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 0,2,4,6,8\n");
            break;
        case 13:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 1,3,5,7,9\n");
            break;
        case 14:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Any, RACH Subframe Number = 0,1,2,3,4,5,6,7,8,9\n");
            break;
        case 15:
            fprintf(stderr, "        Preamble Format = 0, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        case 16:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 17:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 18:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 19:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 20:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 21:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 22:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 23:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 24:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 25:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 26:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 27:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 28:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 0,2,4,6,8\n");
            break;
        case 29:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Any, RACH Subframe Number = 1,3,5,7,9\n");
            break;
        case 30:
            fprintf(stderr, "        Preamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 31:
            fprintf(stderr, "        Preamble Format = 1, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        case 32:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 33:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 34:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 35:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 36:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 37:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 38:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 39:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 40:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 41:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 42:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 43:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 44:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 0,2,4,6,8\n");
            break;
        case 45:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Any, RACH Subframe Number = 1,3,5,7,9\n");
            break;
        case 46:
            fprintf(stderr, "        Preamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 47:
            fprintf(stderr, "        Preamble Format = 2, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        case 48:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Even, RACH Subframe Number = 1\n");
            break;
        case 49:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Even, RACH Subframe Number = 4\n");
            break;
        case 50:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Even, RACH Subframe Number = 7\n");
            break;
        case 51:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 1\n");
            break;
        case 52:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 4\n");
            break;
        case 53:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 7\n");
            break;
        case 54:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 1,6\n");
            break;
        case 55:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 2,7\n");
            break;
        case 56:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 3,8\n");
            break;
        case 57:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 1,4,7\n");
            break;
        case 58:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 2,5,8\n");
            break;
        case 59:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Any, RACH Subframe Number = 3,6,9\n");
            break;
        case 60:
            fprintf(stderr, "        Preamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 61:
            fprintf(stderr, "        Preamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 62:
            fprintf(stderr, "        Preamble Format = N/A, RACH SFN = N/A, RACH Subframe Number = N/A\n");
            break;
        case 63:
            fprintf(stderr, "        Preamble Format = 3, RACH SFN = Even, RACH Subframe Number = 9\n");
            break;
        }
        if(true == sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag)
        {
            fprintf(stderr, "    %-40s=%20s\n", "High Speed Flag", "Restricted Set");
        }else{
            fprintf(stderr, "    %-40s=%20s\n", "High Speed Flag", "Unrestricted Set");
        }
        fprintf(stderr, "    %-40s=%20u\n", "Ncs Configuration", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config);
        fprintf(stderr, "    %-40s=%20u\n", "PRACH Freq Offset", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset);
        fprintf(stderr, "    %-40s=%17ddBm\n", "Reference Signal Power", sib2->rr_config_common_sib.pdsch_cnfg.rs_power);
        fprintf(stderr, "    %-40s=%20u\n", "Pb", sib2->rr_config_common_sib.pdsch_cnfg.p_b);
        fprintf(stderr, "    %-40s=%20u\n", "Nsb", sib2->rr_config_common_sib.pusch_cnfg.n_sb);
        switch(sib2->rr_config_common_sib.pusch_cnfg.hopping_mode)
        {
        case LIBLTE_RRC_HOPPING_MODE_INTER_SUBFRAME:
            fprintf(stderr, "    %-40s=%20s\n", "Hopping Mode", "Inter Subframe");
            break;
        case LIBLTE_RRC_HOPPING_MODE_INTRA_AND_INTER_SUBFRAME:
            fprintf(stderr, "    %-40s= %s\n", "Hopping Mode", "Intra and Inter Subframe");
            break;
        }
        fprintf(stderr, "    %-40s=%20u\n", "PUSCH Nrb Hopping Offset", sib2->rr_config_common_sib.pusch_cnfg.pusch_hopping_offset);
        if(true == sib2->rr_config_common_sib.pusch_cnfg.enable_64_qam)
        {
            fprintf(stderr, "    %-40s=%20s\n", "64QAM", "Allowed");
        }else{
            fprintf(stderr, "    %-40s=%20s\n", "64QAM", "Not Allowed");
        }
        if(true == sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled)
        {
            fprintf(stderr, "    %-40s=%20s\n", "Group Hopping", "Enabled");
        }else{
            fprintf(stderr, "    %-40s=%20s\n", "Group Hopping", "Disabled");
        }
        fprintf(stderr, "    %-40s=%20u\n", "Group Assignment PUSCH", sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch);
        if(true == sib2->rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled)
        {
            fprintf(stderr, "    %-40s=%20s\n", "Sequence Hopping", "Enabled");
        }else{
            fprintf(stderr, "    %-40s=%20s\n", "Sequence Hopping", "Disabled");
        }
        fprintf(stderr, "    %-40s=%20u\n", "Cyclic Shift", sib2->rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift);
        fprintf(stderr, "    %-40s=%20s\n", "Delta PUCCH Shift", liblte_rrc_delta_pucch_shift_text[sib2->rr_config_common_sib.pucch_cnfg.delta_pucch_shift]);
        fprintf(stderr, "    %-40s=%20u\n", "N_rb_cqi", sib2->rr_config_common_sib.pucch_cnfg.n_rb_cqi);
        fprintf(stderr, "    %-40s=%20u\n", "N_cs_an", sib2->rr_config_common_sib.pucch_cnfg.n_cs_an);
        fprintf(stderr, "    %-40s=%20u\n", "N1 PUCCH AN", sib2->rr_config_common_sib.pucch_cnfg.n1_pucch_an);
        if(true == sib2->rr_config_common_sib.srs_ul_cnfg.present)
        {
            fprintf(stderr, "    %-40s=%20s\n", "SRS Bandwidth Config", liblte_rrc_srs_bw_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.bw_cnfg]);
            fprintf(stderr, "    %-40s=%20s\n", "SRS Subframe Config", liblte_rrc_srs_subfr_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.subfr_cnfg]);
            if(true == sib2->rr_config_common_sib.srs_ul_cnfg.ack_nack_simul_tx)
            {
                fprintf(stderr, "    %-40s=%20s\n", "Simultaneous AN and SRS", "True");
            }else{
                fprintf(stderr, "    %-40s=%20s\n", "Simultaneous AN and SRS", "False");
            }
            if(true == sib2->rr_config_common_sib.srs_ul_cnfg.max_up_pts_present)
            {
                fprintf(stderr, "    %-40s=%20s\n", "SRS Max Up PTS", "True");
            }else{
                fprintf(stderr, "    %-40s=%20s\n", "SRS Max Up PTS", "False");
            }
        }
        fprintf(stderr, "    %-40s=%17ddBm\n", "P0 Nominal PUSCH", sib2->rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch);
        fprintf(stderr, "    %-40s=%20s\n", "Alpha", liblte_rrc_ul_power_control_alpha_text[sib2->rr_config_common_sib.ul_pwr_ctrl.alpha]);
        fprintf(stderr, "    %-40s=%17ddBm\n", "P0 Nominal PUCCH", sib2->rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch);
        fprintf(stderr, "    %-40s=%18sdB\n", "Delta F PUCCH Format 1", liblte_rrc_delta_f_pucch_format_1_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1]);
        fprintf(stderr, "    %-40s=%18sdB\n", "Delta F PUCCH Format 1B", liblte_rrc_delta_f_pucch_format_1b_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1b]);
        fprintf(stderr, "    %-40s=%18sdB\n", "Delta F PUCCH Format 2", liblte_rrc_delta_f_pucch_format_2_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2]);
        fprintf(stderr, "    %-40s=%18sdB\n", "Delta F PUCCH Format 2A", liblte_rrc_delta_f_pucch_format_2a_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2a]);
        fprintf(stderr, "    %-40s=%18sdB\n", "Delta F PUCCH Format 2B", liblte_rrc_delta_f_pucch_format_2b_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2b]);
        fprintf(stderr, "    %-40s=%18ddB\n", "Delta Preamble Message 3", sib2->rr_config_common_sib.ul_pwr_ctrl.delta_preamble_msg3);
        switch(sib2->rr_config_common_sib.ul_cp_length)
        {
        case LIBLTE_RRC_UL_CP_LENGTH_1:
            fprintf(stderr, "    %-40s=%20s\n", "UL CP Length", "Normal");
            break;
        case LIBLTE_RRC_UL_CP_LENGTH_2:
            fprintf(stderr, "    %-40s=%20s\n", "UL CP Length", "Extended");
            break;
        }
        fprintf(stderr, "    %-40s=%18sms\n", "T300", liblte_rrc_t300_text[sib2->ue_timers_and_constants.t300]);
        fprintf(stderr, "    %-40s=%18sms\n", "T301", liblte_rrc_t301_text[sib2->ue_timers_and_constants.t301]);
        fprintf(stderr, "    %-40s=%18sms\n", "T310", liblte_rrc_t310_text[sib2->ue_timers_and_constants.t310]);
        fprintf(stderr, "    %-40s=%20s\n", "N310", liblte_rrc_n310_text[sib2->ue_timers_and_constants.n310]);
        fprintf(stderr, "    %-40s=%18sms\n", "T311", liblte_rrc_t311_text[sib2->ue_timers_and_constants.t311]);
        fprintf(stderr, "    %-40s=%20s\n", "N311", liblte_rrc_n311_text[sib2->ue_timers_and_constants.n311]);
        if(true == sib2->arfcn_value_eutra.present)
        {
            fprintf(stderr, "    %-40s=%20u\n", "UL ARFCN", sib2->arfcn_value_eutra.value);
        }
        if(true == sib2->ul_bw.present)
        {
            fprintf(stderr, "    %-40s=%17sMHz\n", "UL Bandwidth", liblte_rrc_ul_bw_text[sib2->ul_bw.bw]);
        }
        fprintf(stderr, "    %-40s=%20u\n", "Additional Spectrum Emission", sib2->additional_spectrum_emission);
        if(0 != sib2->mbsfn_subfr_cnfg_list_size)
        {
            fprintf(stderr, "    %s:\n", "MBSFN Subframe Config List");
        }
        for(i=0; i<sib2->mbsfn_subfr_cnfg_list_size; i++)
        {
            fprintf(stderr, "        %-40s=%20s\n", "Radio Frame Alloc Period", liblte_rrc_radio_frame_allocation_period_text[sib2->mbsfn_subfr_cnfg[i].radio_fr_alloc_period]);
            fprintf(stderr, "        %-40s=%20u\n", "Radio Frame Alloc Offset", sib2->mbsfn_subfr_cnfg[i].subfr_alloc);
            fprintf(stderr, "        Subframe Alloc%-26s=%20u\n", liblte_rrc_subframe_allocation_num_frames_text[sib2->mbsfn_subfr_cnfg[i].subfr_alloc_num_frames], sib2->mbsfn_subfr_cnfg[i].subfr_alloc);
        }
        fprintf(stderr, "    %-40s=%10s Subframes\n", "Time Alignment Timer", liblte_rrc_time_alignment_timer_text[sib2->time_alignment_timer]);

        ctx->sib2_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib3(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *sib3)
{
    if(false == ctx->sib3_printed)
    {
        fprintf(stderr, "SIB3 Decoded:\n");
        fprintf(stderr, "    %-40s=%18sdB\n", "Q-Hyst", liblte_rrc_q_hyst_text[sib3->q_hyst]);
        if(true == sib3->speed_state_resel_params.present)
        {
            fprintf(stderr, "    %-40s=%19ss\n", "T-Evaluation", liblte_rrc_t_evaluation_text[sib3->speed_state_resel_params.mobility_state_params.t_eval]);
            fprintf(stderr, "    %-40s=%19ss\n", "T-Hyst Normal", liblte_rrc_t_hyst_normal_text[sib3->speed_state_resel_params.mobility_state_params.t_hyst_normal]);
            fprintf(stderr, "    %-40s=%20u\n", "N-Cell Change Medium", sib3->speed_state_resel_params.mobility_state_params.n_cell_change_medium);
            fprintf(stderr, "    %-40s=%20u\n", "N-Cell Change High", sib3->speed_state_resel_params.mobility_state_params.n_cell_change_high);
            fprintf(stderr, "    %-40s=%18sdB\n", "Q-Hyst SF Medium", liblte_rrc_sf_medium_text[sib3->speed_state_resel_params.q_hyst_sf.medium]);
            fprintf(stderr, "    %-40s=%18sdB\n", "Q-Hyst SF High", liblte_rrc_sf_high_text[sib3->speed_state_resel_params.q_hyst_sf.high]);
        }
        if(true == sib3->s_non_intra_search_present)
        {
            fprintf(stderr, "    %-40s=%18udB\n", "S-Non Intra Search", sib3->s_non_intra_search);
        }
        fprintf(stderr, "    %-40s=%18udB\n", "Threshold Serving Low", sib3->thresh_serving_low);
        fprintf(stderr, "    %-40s=%20u\n", "Cell Reselection Priority", sib3->cell_resel_prio);
        fprintf(stderr, "    %-40s=%17ddBm\n", "Q Rx Lev Min", sib3->q_rx_lev_min);
        if(true == sib3->p_max_present)
        {
            fprintf(stderr, "    %-40s=%17ddBm\n", "P Max", sib3->p_max);
        }
        if(true == sib3->s_intra_search_present)
        {
            fprintf(stderr, "    %-40s=%18udB\n", "S-Intra Search", sib3->s_intra_search);
        }
        if(true == sib3->allowed_meas_bw_present)
        {
            fprintf(stderr, "    %-40s=%17sMHz\n", "Allowed Meas Bandwidth", liblte_rrc_allowed_meas_bandwidth_text[sib3->allowed_meas_bw]);
        }
        if(true == sib3->presence_ant_port_1)
        {
            fprintf(stderr, "    %-40s=%20s\n", "Presence Antenna Port 1", "True");
        }else{
            fprintf(stderr, "    %-40s=%20s\n", "Presence Antenna Port 1", "False");
        }
        switch(sib3->neigh_cell_cnfg)
        {
        case 0:
            fprintf(stderr, "    %-40s= %s\n", "Neighbor Cell Config", "Not all neighbor cells have the same MBSFN alloc");
            break;
        case 1:
            fprintf(stderr, "    %-40s= %s\n", "Neighbor Cell Config", "MBSFN allocs are identical for all neighbor cells");
            break;
        case 2:
            fprintf(stderr, "    %-40s= %s\n", "Neighbor Cell Config", "No MBSFN allocs are present in neighbor cells");
            break;
        case 3:
            fprintf(stderr, "    %-40s= %s\n", "Neighbor Cell Config", "Different UL/DL allocs in neighbor cells for TDD");
            break;
        }
        fprintf(stderr, "    %-40s=%19us\n", "T-Reselection EUTRA", sib3->t_resel_eutra);
        if(true == sib3->t_resel_eutra_sf_present)
        {
            fprintf(stderr, "    %-40s=%20s\n", "T-Reselection EUTRA SF Medium", liblte_rrc_sssf_medium_text[sib3->t_resel_eutra_sf.sf_medium]);
            fprintf(stderr, "    %-40s=%20s\n", "T-Reselection EUTRA SF High", liblte_rrc_sssf_high_text[sib3->t_resel_eutra_sf.sf_high]);
        }

        ctx->sib3_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib4(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *sib4)
{
    uint32 i;
    uint32 stop;

    if(false == ctx->sib4_printed) {
        fprintf(stderr, "SIB4 Decoded:\n");
        if(0 != sib4->intra_freq_neigh_cell_list_size) {
            fprintf(stderr, "    List of intra-frequency neighboring cells:\n");
        }
        for(i=0; i<sib4->intra_freq_neigh_cell_list_size; i++) {
            fprintf(stderr, "        %s = %u\n", "Physical Cell ID", sib4->intra_freq_neigh_cell_list[i].phys_cell_id);
            fprintf(stderr, "            %s = %sdB\n", "Q Offset Range", liblte_rrc_q_offset_range_text[sib4->intra_freq_neigh_cell_list[i].q_offset_range]);
        }
        if(0 != sib4->intra_freq_black_cell_list_size) {
            fprintf(stderr, "    List of blacklisted intra-frequency neighboring cells:\n");
        }
        for(i=0; i<sib4->intra_freq_black_cell_list_size; i++) {
            fprintf(stderr, "        %u - %u\n", sib4->intra_freq_black_cell_list[i].start, sib4->intra_freq_black_cell_list[i].start + liblte_rrc_phys_cell_id_range_num[sib4->intra_freq_black_cell_list[i].range]);
        }
        if(true == sib4->csg_phys_cell_id_range_present) {
            fprintf(stderr, "    %-40s= %u - %u\n", "CSG Phys Cell ID Range", sib4->csg_phys_cell_id_range.start, sib4->csg_phys_cell_id_range.start + liblte_rrc_phys_cell_id_range_num[sib4->csg_phys_cell_id_range.range]);
        }

        ctx->sib4_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib5(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *sib5)
{
    uint32 i;
    uint32 j;
    uint16 stop;

    if(false == ctx->sib5_printed) {
        fprintf(stderr, "SIB5 Decoded:\n");
        fprintf(stderr, "    List of inter-frequency neighboring cells:\n");
        for(i=0; i<sib5->inter_freq_carrier_freq_list_size; i++)  {
            fprintf(stderr, "        %-40s=%20u\n", "ARFCN", sib5->inter_freq_carrier_freq_list[i].dl_carrier_freq);
            fprintf(stderr, "        %-40s=%17ddBm\n", "Q Rx Lev Min", sib5->inter_freq_carrier_freq_list[i].q_rx_lev_min);
            if(true == sib5->inter_freq_carrier_freq_list[i].p_max_present) {
                fprintf(stderr, "        %-40s=%17ddBm\n", "P Max", sib5->inter_freq_carrier_freq_list[i].p_max);
            }
            fprintf(stderr, "        %-40s=%19us\n", "T-Reselection EUTRA", sib5->inter_freq_carrier_freq_list[i].t_resel_eutra);
            if(true == sib5->inter_freq_carrier_freq_list[i].t_resel_eutra_sf_present) {
                fprintf(stderr, "        %-40s=%20s\n", "T-Reselection EUTRA SF Medium", liblte_rrc_sssf_medium_text[sib5->inter_freq_carrier_freq_list[i].t_resel_eutra_sf.sf_medium]);
                fprintf(stderr, "        %-40s=%20s\n", "T-Reselection EUTRA SF High", liblte_rrc_sssf_high_text[sib5->inter_freq_carrier_freq_list[i].t_resel_eutra_sf.sf_high]);
            }
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib5->inter_freq_carrier_freq_list[i].threshx_high);
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib5->inter_freq_carrier_freq_list[i].threshx_low);
            fprintf(stderr, "        %-40s=%17sMHz\n", "Allowed Meas Bandwidth", liblte_rrc_allowed_meas_bandwidth_text[sib5->inter_freq_carrier_freq_list[i].allowed_meas_bw]);
            if(true == sib5->inter_freq_carrier_freq_list[i].presence_ant_port_1) {
                fprintf(stderr, "        %-40s=%20s\n", "Presence Antenna Port 1", "True");
            }
            else {
                fprintf(stderr, "        %-40s=%20s\n", "Presence Antenna Port 1", "False");
            }
            if(true == sib5->inter_freq_carrier_freq_list[i].cell_resel_prio_present) {
                fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib5->inter_freq_carrier_freq_list[i].cell_resel_prio);
            }
            
            switch(sib5->inter_freq_carrier_freq_list[i].neigh_cell_cnfg) {
            case 0:
                fprintf(stderr, "        %-40s= %s\n", "Neighbor Cell Config", "Not all neighbor cells have the same MBSFN alloc");
                break;
            case 1:
                fprintf(stderr, "        %-40s= %s\n", "Neighbor Cell Config", "MBSFN allocs are identical for all neighbor cells");
                break;
            case 2:
                fprintf(stderr, "        %-40s= %s\n", "Neighbor Cell Config", "No MBSFN allocs are present in neighbor cells");
                break;
            case 3:
                fprintf(stderr, "        %-40s= %s\n", "Neighbor Cell Config", "Different UL/DL allocs in neighbor cells for TDD");
                break;
            }
            fprintf(stderr, "        %-40s=%18sdB\n", "Q Offset Freq", liblte_rrc_q_offset_range_text[sib5->inter_freq_carrier_freq_list[i].q_offset_freq]);
            if(0 != sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list_size) {
                fprintf(stderr, "        List of inter-frequency neighboring cells with specific cell reselection parameters:\n");
                for(j=0; j<sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list_size; j++) {
                    fprintf(stderr, "            %-40s=%20u\n", "Physical Cell ID", sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list[j].phys_cell_id);
                    fprintf(stderr, "            %-40s=%18sdB\n", "Q Offset Cell", liblte_rrc_q_offset_range_text[sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list[j].q_offset_cell]);
                }
            }
            if(0 != sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list_size) {
                fprintf(stderr, "        List of blacklisted inter-frequency neighboring cells\n");
                for(j=0; j<sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list_size; j++) {
                    fprintf(stderr, "            %u - %u\n", sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].start, sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].start + liblte_rrc_phys_cell_id_range_num[sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].range]);
                }
            }
        }

        ctx->sib5_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib6(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *sib6)
{
    uint32 i;

    if(false == ctx->sib6_printed) {
        fprintf(stderr, "SIB6 Decoded:\n");
        if(0 != sib6->carrier_freq_list_utra_fdd_size) {
            fprintf(stderr, "    %s:\n", "Carrier Freq List UTRA FDD");
        }
        for(i=0; i<sib6->carrier_freq_list_utra_fdd_size; i++) {
            fprintf(stderr, "        %-40s=%20u\n", "ARFCN", sib6->carrier_freq_list_utra_fdd[i].carrier_freq);
            if(true == sib6->carrier_freq_list_utra_fdd[i].cell_resel_prio_present) {
                fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib6->carrier_freq_list_utra_fdd[i].cell_resel_prio);
            }
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib6->carrier_freq_list_utra_fdd[i].threshx_high);
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib6->carrier_freq_list_utra_fdd[i].threshx_low);
            fprintf(stderr, "        %-40s=%17ddBm\n", "Q Rx Lev Min", sib6->carrier_freq_list_utra_fdd[i].q_rx_lev_min);
            fprintf(stderr, "        %-40s=%17ddBm\n", "P Max UTRA", sib6->carrier_freq_list_utra_fdd[i].p_max_utra);
            fprintf(stderr, "        %-40s=%18dB\n", "Q Qual Min", sib6->carrier_freq_list_utra_fdd[i].q_qual_min);
        }
        if(0 != sib6->carrier_freq_list_utra_tdd_size) {
            fprintf(stderr, "    %s:\n", "Carrier Freq List UTRA TDD");
        }
        for(i=0; i<sib6->carrier_freq_list_utra_tdd_size; i++) {
            fprintf(stderr, "        %-40s=%20u\n", "ARFCN", sib6->carrier_freq_list_utra_tdd[i].carrier_freq);
            if(true == sib6->carrier_freq_list_utra_tdd[i].cell_resel_prio_present) {
                fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib6->carrier_freq_list_utra_tdd[i].cell_resel_prio);
            }
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib6->carrier_freq_list_utra_tdd[i].threshx_high);
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib6->carrier_freq_list_utra_tdd[i].threshx_low);
            fprintf(stderr, "        %-40s=%17ddBm\n", "Q Rx Lev Min", sib6->carrier_freq_list_utra_tdd[i].q_rx_lev_min);
            fprintf(stderr, "        %-40s=%17ddBm\n", "P Max UTRA", sib6->carrier_freq_list_utra_tdd[i].p_max_utra);
        }
        fprintf(stderr, "    %-40s=%19us\n", "T-Reselection UTRA", sib6->t_resel_utra);
        if(true == sib6->t_resel_utra_sf_present) {
            fprintf(stderr, "    %-40s=%20s\n", "T-Reselection UTRA SF Medium", liblte_rrc_sssf_medium_text[sib6->t_resel_utra_sf.sf_medium]);
            fprintf(stderr, "    %-40s=%20s\n", "T-Reselection UTRA SF High", liblte_rrc_sssf_high_text[sib6->t_resel_utra_sf.sf_high]);
        }

        ctx->sib6_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib7(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *sib7)
{
    uint32 i;
    uint32 j;

    if(false == ctx->sib7_printed) {
        fprintf(stderr, "SIB7 Decoded:\n");
        fprintf(stderr, "    %-40s=%19us\n", "T-Reselection GERAN", sib7->t_resel_geran);
        if(true == sib7->t_resel_geran_sf_present) {
            fprintf(stderr, "    %-40s=%20s\n", "T-Reselection GERAN SF Medium", liblte_rrc_sssf_medium_text[sib7->t_resel_geran_sf.sf_medium]);
            fprintf(stderr, "    %-40s=%20s\n", "T-Reselection GERAN SF High", liblte_rrc_sssf_high_text[sib7->t_resel_geran_sf.sf_high]);
        }
        if(0 != sib7->carrier_freqs_info_list_size) {
            fprintf(stderr, "    List of neighboring GERAN carrier frequencies\n");
        }
        for(i=0; i<sib7->carrier_freqs_info_list_size; i++) {
            fprintf(stderr, "        %-40s=%20u\n", "Starting ARFCN", sib7->carrier_freqs_info_list[i].carrier_freqs.starting_arfcn);
            fprintf(stderr, "        %-40s=%20s\n", "Band Indicator", liblte_rrc_band_indicator_geran_text[sib7->carrier_freqs_info_list[i].carrier_freqs.band_indicator]);
            if(LIBLTE_RRC_FOLLOWING_ARFCNS_EXPLICIT_LIST == sib7->carrier_freqs_info_list[i].carrier_freqs.following_arfcns) {
                fprintf(stderr, "        Following ARFCNs Explicit List\n");
                for(j=0; j<sib7->carrier_freqs_info_list[i].carrier_freqs.explicit_list_of_arfcns_size; j++) {
                    fprintf(stderr, "            %u\n", sib7->carrier_freqs_info_list[i].carrier_freqs.explicit_list_of_arfcns[j]);
                }
            }
            else if(LIBLTE_RRC_FOLLOWING_ARFCNS_EQUALLY_SPACED == sib7->carrier_freqs_info_list[i].carrier_freqs.following_arfcns) {
                fprintf(stderr, "        Following ARFCNs Equally Spaced\n");
                fprintf(stderr, "            %u, %u\n", sib7->carrier_freqs_info_list[i].carrier_freqs.equally_spaced_arfcns.arfcn_spacing, sib7->carrier_freqs_info_list[i].carrier_freqs.equally_spaced_arfcns.number_of_arfcns);
            }
            else {
                fprintf(stderr, "        Following ARFCNs Variable Bit Map\n");
                fprintf(stderr, "            %02X\n", sib7->carrier_freqs_info_list[i].carrier_freqs.variable_bit_map_of_arfcns);
            }
            if(true == sib7->carrier_freqs_info_list[i].cell_resel_prio_present) {
                fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib7->carrier_freqs_info_list[i].cell_resel_prio);
            }
            fprintf(stderr, "        %-40s=%20u\n", "NCC Permitted", sib7->carrier_freqs_info_list[i].ncc_permitted);
            fprintf(stderr, "        %-40s=%17ddBm\n", "Q Rx Lev Min", sib7->carrier_freqs_info_list[i].q_rx_lev_min);
            if(true == sib7->carrier_freqs_info_list[i].p_max_geran_present) {
                fprintf(stderr, "        %-40s=%17udBm\n", "P Max GERAN", sib7->carrier_freqs_info_list[i].p_max_geran);
            }
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib7->carrier_freqs_info_list[i].threshx_high);
            fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib7->carrier_freqs_info_list[i].threshx_low);
        }

        ctx->sib7_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib8(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *sib8)
{
    uint32 i;
    uint32 j;
    uint32 k;

    if(false == ctx->sib8_printed) {
        fprintf(stderr, "SIB8 Decoded:\n");
        if(true == sib8->sys_time_info_present) {
            if(true == sib8->sys_time_info_cdma2000.cdma_eutra_sync) {
                fprintf(stderr, "    %-40s=%20s\n", "CDMA EUTRA sync", "True");
            }
            else{
                fprintf(stderr, "    %-40s=%20s\n", "CDMA EUTRA sync", "False");
            }
            if(true == sib8->sys_time_info_cdma2000.system_time_async) {
                fprintf(stderr, "    %-40s=%14llu chips\n", "System Time", sib8->sys_time_info_cdma2000.system_time * 8);
            }
            else{
                fprintf(stderr, "    %-40s=%17llu ms\n", "System Time", sib8->sys_time_info_cdma2000.system_time * 10);
            }
        }
        if(true == sib8->search_win_size_present) {
            fprintf(stderr, "    %-40s=%20u\n", "Search Window Size", sib8->search_win_size);
        }
        if(true == sib8->params_hrpd_present) {
            if(true == sib8->pre_reg_info_hrpd.pre_reg_allowed) {
                fprintf(stderr, "    %-40s=%20s\n", "Pre Registration", "Allowed");
            }
            else{
                fprintf(stderr, "    %-40s=%20s\n", "Pre Registration", "Not Allowed");
            }
            if(true == sib8->pre_reg_info_hrpd.pre_reg_zone_id_present) {
                fprintf(stderr, "    %-40s=%20u\n", "Pre Registration Zone ID", sib8->pre_reg_info_hrpd.pre_reg_zone_id);
            }
            if(0 != sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list_size) {
                fprintf(stderr, "    Secondary Pre Registration Zone IDs:\n");
            }
            for(i=0; i<sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list_size; i++) {
                fprintf(stderr, "        %u\n", sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list[i]);
            }
            
            if(true == sib8->cell_resel_params_hrpd_present) {
                fprintf(stderr, "    Band Class List:\n");
                for(i=0; i<sib8->cell_resel_params_hrpd.band_class_list_size; i++) {
                    fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_hrpd.band_class_list[i].band_class]);
                    if(true == sib8->cell_resel_params_hrpd.band_class_list[i].cell_resel_prio_present) {
                        fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib8->cell_resel_params_hrpd.band_class_list[i].cell_resel_prio);
                    }
                    fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib8->cell_resel_params_hrpd.band_class_list[i].thresh_x_high);
                    fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib8->cell_resel_params_hrpd.band_class_list[i].thresh_x_low);
                }
                fprintf(stderr, "    Neighbor Cell List:\n");
                for(i=0; i<sib8->cell_resel_params_hrpd.neigh_cell_list_size; i++) {
                    fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_hrpd.neigh_cell_list[i].band_class]);
                    fprintf(stderr, "        Neighbor Cells Per Frequency List\n");
                    for(j=0; j<sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list_size; j++) {
                        fprintf(stderr, "            %-40s=%20u\n", "ARFCN", sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].arfcn);
                        fprintf(stderr, "            Phys Cell ID List\n");
                        for(k=0; k<sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list_size; k++) {
                            fprintf(stderr, "            \t%u\n", sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list[k]);
                        }
                    }
                }
                fprintf(stderr, "    %-40s=%19us\n", "T Reselection", sib8->cell_resel_params_hrpd.t_resel_cdma2000);
                if(true == sib8->cell_resel_params_hrpd.t_resel_cdma2000_sf_present) {
                    fprintf(stderr, "    %-40s=%20s\n", "T-Reselection Scale Factor Medium", liblte_rrc_sssf_medium_text[sib8->cell_resel_params_hrpd.t_resel_cdma2000_sf.sf_medium]);
                    fprintf(stderr, "    %-40s=%20s\n", "T-Reselection Scale Factor High", liblte_rrc_sssf_high_text[sib8->cell_resel_params_hrpd.t_resel_cdma2000_sf.sf_high]);
                }
            }
        }
        if(true == sib8->params_1xrtt_present) {
            fprintf(stderr, "    CSFB Registration Parameters\n");
            if(true == sib8->csfb_reg_param_1xrtt_present) {
                fprintf(stderr, "        %-40s=%20u\n", "SID", sib8->csfb_reg_param_1xrtt.sid);
                fprintf(stderr, "        %-40s=%20u\n", "NID", sib8->csfb_reg_param_1xrtt.nid);
                if(true == sib8->csfb_reg_param_1xrtt.multiple_sid) {
                    fprintf(stderr, "        %-40s=%20s\n", "Multiple SIDs", "True");
                }
                else {
                    fprintf(stderr, "        %-40s=%20s\n", "Multiple SIDs", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.multiple_nid) {
                    fprintf(stderr, "        %-40s=%20s\n", "Multiple NIDs", "True");
                }
                else{
                    fprintf(stderr, "        %-40s=%20s\n", "Multiple NIDs", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.home_reg) {
                    fprintf(stderr, "        %-40s=%20s\n", "Home Reg", "True");
                }
                else {
                    fprintf(stderr, "        %-40s=%20s\n", "Home Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.foreign_sid_reg) {
                    fprintf(stderr, "        %-40s=%20s\n", "Foreign SID Reg", "True");
                }
                else {
                    fprintf(stderr, "        %-40s=%20s\n", "Foreign SID Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.foreign_nid_reg) {
                    fprintf(stderr, "        %-40s=%20s\n", "Foreign NID Reg", "True");
                }
                else {
                    fprintf(stderr, "        %-40s=%20s\n", "Foreign NID Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.param_reg) {
                    fprintf(stderr, "        %-40s=%20s\n", "Parameter Reg", "True");
                }
                else {
                    fprintf(stderr, "        %-40s=%20s\n", "Parameter Reg", "False");
                }
                if(true == sib8->csfb_reg_param_1xrtt.power_up_reg) {
                    fprintf(stderr, "        %-40s=%20s\n", "Power Up Reg", "True");
                }
                else {
                    fprintf(stderr, "        %-40s=%20s\n", "Power Up Reg", "False");
                }
                fprintf(stderr, "        %-40s=%20u\n", "Registration Period", sib8->csfb_reg_param_1xrtt.reg_period);
                fprintf(stderr, "        %-40s=%20u\n", "Registration Zone", sib8->csfb_reg_param_1xrtt.reg_zone);
                fprintf(stderr, "        %-40s=%20u\n", "Total Zones", sib8->csfb_reg_param_1xrtt.total_zone);
                fprintf(stderr, "        %-40s=%20u\n", "Zone Timer", sib8->csfb_reg_param_1xrtt.zone_timer);
            }
            if(true == sib8->long_code_state_1xrtt_present) {
                fprintf(stderr, "    %-40s=%20llu\n", "Long Code State", sib8->long_code_state_1xrtt);
            }
            if(true == sib8->cell_resel_params_1xrtt_present) {
                fprintf(stderr, "    Band Class List:\n");
                for(i=0; i<sib8->cell_resel_params_1xrtt.band_class_list_size; i++) {
                    fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_1xrtt.band_class_list[i].band_class]);
                    if(true == sib8->cell_resel_params_1xrtt.band_class_list[i].cell_resel_prio_present) {
                        fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib8->cell_resel_params_1xrtt.band_class_list[i].cell_resel_prio);
                    }
                    fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib8->cell_resel_params_1xrtt.band_class_list[i].thresh_x_high);
                    fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib8->cell_resel_params_1xrtt.band_class_list[i].thresh_x_low);
                }
                fprintf(stderr, "    Neighbor Cell List:\n");
                for(i=0; i<sib8->cell_resel_params_1xrtt.neigh_cell_list_size; i++) {
                    fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_1xrtt.neigh_cell_list[i].band_class]);
                    fprintf(stderr, "        Neighbor Cells Per Frequency List\n");
                    for(j=0; j<sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list_size; j++) {
                        fprintf(stderr, "            %-40s=%20u\n", "ARFCN", sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].arfcn);
                        fprintf(stderr, "            Phys Cell ID List\n");
                        for(k=0; k<sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list_size; k++) {
                            fprintf(stderr, "            \t%u\n", sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list[k]);
                        }
                    }
                }
                fprintf(stderr, "    %-40s=%19us\n", "T Reselection", sib8->cell_resel_params_1xrtt.t_resel_cdma2000);
                if(true == sib8->cell_resel_params_1xrtt.t_resel_cdma2000_sf_present) {
                    fprintf(stderr, "    %-40s=%20s\n", "T-Reselection Scale Factor Medium", liblte_rrc_sssf_medium_text[sib8->cell_resel_params_1xrtt.t_resel_cdma2000_sf.sf_medium]);
                    fprintf(stderr, "    %-40s=%20s\n", "T-Reselection Scale Factor High", liblte_rrc_sssf_high_text[sib8->cell_resel_params_1xrtt.t_resel_cdma2000_sf.sf_high]);
                }
            }
        }

        ctx->sib8_printed = true;
    }
}

static void LTE_Fdd_Analysis_print_sib13(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13_STRUCT *sib13)
{
    uint32 i;

    if(false == ctx->sib13_printed)   {
        fprintf(stderr, "SIB13 Decoded:\n");

        fprintf(stderr, "    MBSFN Area Info List R9:\n");
        for(i=0; i<sib13->mbsfn_area_info_list_r9_size; i++) {
            fprintf(stderr, "        %-40s=%20u\n", "MBSFN Area ID R9", sib13->mbsfn_area_info_list_r9[i].mbsfn_area_id_r9);
            fprintf(stderr, "        %-40s=%20s\n", "Non-MBSFN Region Length", liblte_rrc_non_mbsfn_region_length_text[sib13->mbsfn_area_info_list_r9[i].non_mbsfn_region_length]);
            fprintf(stderr, "        %-40s=%20u\n", "Notification Indicator R9", sib13->mbsfn_area_info_list_r9[i].notification_indicator_r9);
            fprintf(stderr, "        %-40s=%20s\n", "MCCH Repetition Period R9", liblte_rrc_mcch_repetition_period_r9_text[sib13->mbsfn_area_info_list_r9[i].mcch_repetition_period_r9]);
            fprintf(stderr, "        %-40s=%20u\n", "MCCH Offset R9", sib13->mbsfn_area_info_list_r9[i].mcch_offset_r9);
            fprintf(stderr, "        %-40s=%20s\n", "MCCH Modification Period R9", liblte_rrc_mcch_modification_period_r9_text[sib13->mbsfn_area_info_list_r9[i].mcch_modification_period_r9]);
            fprintf(stderr, "        %-40s=%20u\n", "SF Alloc Info R9", sib13->mbsfn_area_info_list_r9[i].sf_alloc_info_r9);
            fprintf(stderr, "        %-40s=%20s\n", "Signalling MCS R9", liblte_rrc_mcch_signalling_mcs_r9_text[sib13->mbsfn_area_info_list_r9[i].signalling_mcs_r9]);
        }

        fprintf(stderr, "    %-40s=%20s\n", "Repetition Coeff", liblte_rrc_notification_repetition_coeff_r9_text[sib13->mbms_notification_config.repetition_coeff]);
        fprintf(stderr, "    %-40s=%20u\n", "Offset", sib13->mbms_notification_config.offset);
        fprintf(stderr, "    %-40s=%20u\n", "SF Index", sib13->mbms_notification_config.sf_index);

        ctx->sib13_printed = true;
    }
}


void LTE_Fdd_Analysis_print_page(struct LTE_Fdd_Analysis_s *ctx, LIBLTE_RRC_PAGING_STRUCT *page)
{
    uint32 i;
    uint32 j;

    fprintf(stderr, "PAGE Decoded:\n");
    if(0 != page->paging_record_list_size) {
        fprintf(stderr, "    Number of paging records: %u\n", page->paging_record_list_size);
        for(i=0; i<page->paging_record_list_size; i++) {
            fprintf(stderr, "    Page Record[%d]\n", i);
            if(LIBLTE_RRC_PAGING_UE_IDENTITY_TYPE_S_TMSI == page->paging_record_list[i].ue_identity.ue_identity_type) {
                fprintf(stderr, "        %s\n", "S-TMSI");
                fprintf(stderr, "            %-40s= %08X\n", "M-TMSI", page->paging_record_list[i].ue_identity.s_tmsi.m_tmsi);
                fprintf(stderr, "            %-40s= %u\n", "MMEC", page->paging_record_list[i].ue_identity.s_tmsi.mmec);
            }
            else {
                fprintf(stderr, "        %-40s=", "IMSI");
                for(j=0; j<page->paging_record_list[i].ue_identity.imsi_size; j++) {
                    fprintf(stderr, "%u", page->paging_record_list[i].ue_identity.imsi[j]);
                }
                fprintf(stderr, "\n");
            }
            fprintf(stderr, "        %-40s= %-20s\n", "CN Domain", liblte_rrc_cn_domain_text[page->paging_record_list[i].cn_domain]);
        }
    }
    if(true == page->system_info_modification_present) {
        fprintf(stderr, "    %-40s= %-20s (%d)\n", "System Info Modification", liblte_rrc_system_info_modification_text[page->system_info_modification], page->system_info_modification);
    }
    if(true == page->etws_indication_present) {
        fprintf(stderr, "    %-40s= %-20s\n", "ETWS Indication", liblte_rrc_etws_indication_text[page->etws_indication]);
    }
}


/* End of LTE_FDD_ANALYSIS_Private_Functions */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup LTE_FDD_ANALYSIS_Exported_Functions
  * @{
  */

uint64_t make_log_code(int log_code)
{
    return (uint64_t)1 << (log_code % 64);
}


int Msg_Check_Enabled(int log_code)
{
    struct LTE_Fdd_Analysis_s *ctx = LTE_Fdd_Analysis_GetContext();
    if(log_code >= LOG_MSG_CODE_ENCODE && log_code < LOG_MSG_CODE_MAX) {
        int32_t log_byte = log_code / 64;
        if(log_byte < MSG_LOG_CODE_COUNT) {
            if(ctx->log_msg_code[log_byte] & MAKE_LOG_CODE(log_code)) {
                if(ctx->log_msg_parent != 0) {
                    return (ctx->log_msg_code[log_byte] & MAKE_LOG_CODE(ctx->log_msg_parent)) != 0;
                }
                else {
                    return 1;
                }
            }
        }
    }
    return 0;
}

int Msg_Set_Parent(int log_code)
{
    struct LTE_Fdd_Analysis_s *ctx = LTE_Fdd_Analysis_GetContext();

    if(log_code >= LOG_MSG_CODE_ENCODE && log_code < LOG_MSG_CODE_MAX) {
        ctx->log_msg_parent = log_code;
    }
    return 0;
}

int log_msg_set(int log_code)
{
    struct LTE_Fdd_Analysis_s *ctx = LTE_Fdd_Analysis_GetContext();
    if(log_code >= LOG_MSG_CODE_ENCODE && log_code < LOG_MSG_CODE_MAX) {
        int32_t log_byte = log_code / 64;
        if(log_byte < MSG_LOG_CODE_COUNT) {
            ctx->log_msg_code[log_byte] = ctx->log_msg_code[log_byte] | MAKE_LOG_CODE(log_code);
        }
    }
    return 0;
}

int log_msg_clr(int log_code)
{
    struct LTE_Fdd_Analysis_s *ctx = LTE_Fdd_Analysis_GetContext();
    if(log_code >= LOG_MSG_CODE_ENCODE && log_code < LOG_MSG_CODE_MAX) {
        int32_t log_byte = log_code / 64;
        if(log_byte < MSG_LOG_CODE_COUNT) {
            ctx->log_msg_code[log_byte] = ctx->log_msg_code[log_byte] & (~ MAKE_LOG_CODE(log_code));
        }
    }
    return 0;
}

int InitBitmap(struct LTE_Fdd_Analysis_s *ctx)
{
    uint32 image_width;
    uint32 image_height;

    uint32 img_y;

    uint32 x;
    uint32 y;
    uint32 re_k;
    uint32 re_l;
    uint32 k_max;
    uint32 l_max;

    uint8 r;
    uint8 g;
    uint8 b;

    uint8 *bmp_buff;

    k_max = ctx->N_rb_dl * ctx->N_sc_rb_dl;
    l_max = ctx->symbol_per_slot * ctx->slot_per_subframe * SUBFRAME_PER_FRAME;
    image_height = k_max * PIXEL_PER_RE + 1;
    image_width = (l_max + 1) * PIXEL_PER_RE;

    ctx->k_max = k_max;
    ctx->l_max = l_max;
    ctx->image_height = image_height;
    ctx->image_width = image_width;

    ctx->bitmap_info_header.biSize = 40;
    ctx->bitmap_info_header.biWidth = ctx->image_width;
    ctx->bitmap_info_header.biHeight = ctx->image_height;
    ctx->bitmap_info_header.biPlanes = 1;
    ctx->bitmap_info_header.biBitCount = 24;
    ctx->bitmap_info_header.biCompression = 0;
    ctx->bitmap_info_header.biSizeImage = ctx->image_width * ctx->image_height * BYTES_PER_PIXEL;
    ctx->bitmap_info_header.biXPelsPerMeter = 1000;
    ctx->bitmap_info_header.biYPelsPerMeter = 1000;
    ctx->bitmap_info_header.biClrUsed = 0;
    ctx->bitmap_info_header.biClrImportant = 0;

    ctx->bitmap_file_header.bfType = 0x4D42;
    ctx->bitmap_file_header.bfOffBytes = sizeof(struct BitmapfileHeader) + sizeof(struct BitmapInfoHeader);
    ctx->bitmap_file_header.bfSize = ctx->bitmap_info_header.biSizeImage + ctx->bitmap_file_header.bfOffBytes;

    ctx->bitmap_buffer = (uint8 **)malloc(sizeof(uint8 *) * ctx->N_ant);

    for(int p = 0; p < ctx->N_ant; p++) {

        bmp_buff = (uint8 *)malloc(ctx->bitmap_info_header.biSizeImage);
        ctx->bitmap_buffer[p] = bmp_buff;
        memset(bmp_buff, BLANK_COLOR, ctx->bitmap_info_header.biSizeImage);

        for(re_k = 0; re_k <= k_max; re_k++) {
            img_y = re_k * PIXEL_PER_RE;
            if((re_k % ctx->N_sc_rb_dl) == 0) {
                for(re_l = 0; re_l <= (l_max * PIXEL_PER_RE); re_l++) {
                    memcpy(&bmp_buff[(img_y * ctx->image_width + re_l) * BYTES_PER_PIXEL + 0], &color_green, BYTES_PER_PIXEL);
                }
            }
            else {
                memset(&bmp_buff[img_y * ctx->image_width * BYTES_PER_PIXEL], BOARDER_COLOR, l_max * PIXEL_PER_RE * BYTES_PER_PIXEL);
            }

            for(y = 0; y < PIXEL_PER_RE; y++) {
                memset(&bmp_buff[((img_y + y) * ctx->image_width  + ctx->l_max * PIXEL_PER_RE) * BYTES_PER_PIXEL],
                    0, PIXEL_PER_RE * BYTES_PER_PIXEL);
            }
        }

        for(re_k = 0; re_k < k_max; re_k++) {
            img_y = re_k * PIXEL_PER_RE;
            for(re_l = 0; re_l <= l_max; re_l++) {
                struct PixelColor color;
                if(re_l % (ctx->symbol_per_slot * ctx->slot_per_subframe * SUBFRAME_PER_FRAME) == 0) {
                    color = color_green;
                }
                else if(re_l % (ctx->symbol_per_slot * ctx->slot_per_subframe) == 0) {
                    color = color_red;
                }
                else if(re_l % ctx->symbol_per_slot == 0) {
                    color = color_blue;
                }
                else {
                    color.b = BOARDER_COLOR; color.g = BOARDER_COLOR; color.r = BOARDER_COLOR;
                }
                for(y = 0; y < PIXEL_PER_RE; y++) {
                    memcpy(&bmp_buff[((img_y + y) * ctx->image_width + re_l * PIXEL_PER_RE) * BYTES_PER_PIXEL], &color, BYTES_PER_PIXEL);
                }
            }
        }
    }

    return 0;

}

int SaveBitmap(struct LTE_Fdd_Analysis_s *ctx)
{
    char bitmapFileName[FILE_NAME_LENGTH];

    for(int p = 0; p < ctx->N_ant; p++) {

        snprintf(bitmapFileName, FILE_NAME_LENGTH, "%s_ant%1d_frame%02d.bmp", ctx->file_name, p, ctx->sfn);
        ctx->fBitmap = fopen(bitmapFileName, "wb");
        if(ctx->fBitmap == NULL) {
            return -1;
        }

        fseek(ctx->fBitmap, 0, SEEK_SET);
        fwrite(&ctx->bitmap_file_header, 1, sizeof(struct BitmapfileHeader), ctx->fBitmap);
        fwrite(&ctx->bitmap_info_header, 1, sizeof(struct BitmapInfoHeader), ctx->fBitmap);
        fwrite(ctx->bitmap_buffer[p], 1, ctx->bitmap_info_header.biSizeImage, ctx->fBitmap);

        free(ctx->bitmap_buffer[p]);

        fclose(ctx->fBitmap);
    }

    free(ctx->bitmap_buffer);

    return 0;
}


struct LTE_Fdd_Analysis_s *LTE_Fdd_Analysis_GetContext(void)
{
    return &LTE_Fdd_Analysis_context;
}

static int32_t LTE_Fdd_Analysis_Decode_Init(struct LTE_Fdd_Analysis_s *ctx)
{

    ctx->fs = LIBLTE_PHY_FS_30_72MHZ;

    ctx->i_buf = (float *)malloc(LTE_FDD_DL_FS_SAMP_BUF_SIZE * sizeof(float));
    ctx->q_buf = (float *)malloc(LTE_FDD_DL_FS_SAMP_BUF_SIZE * sizeof(float));

    ctx->in_data = (float *)malloc(LTE_FDD_DL_FS_SAMP_BUF_SIZE * 2 * sizeof(float));
    ctx->filtered_ata = (float *)malloc(LTE_FDD_DL_FS_SAMP_BUF_SIZE * 2 * sizeof(float));

    if(ctx->i_buf == NULL || ctx->q_buf == NULL || ctx->in_data == NULL || ctx->filtered_ata == NULL) {
        if(ctx->i_buf != NULL) { free(ctx->i_buf); }
        if(ctx->q_buf != NULL) { free(ctx->q_buf); }
        if(ctx->in_data != NULL) { free(ctx->in_data); }
        if(ctx->filtered_ata != NULL) { free(ctx->filtered_ata); }
        return -1;
    }

    ctx->samp_buf_w_idx = 0;
    ctx->samp_buf_r_idx = 0;
    ctx->input_offset = 0;

    liblte_phy_init
    (
        &ctx->phy_struct,
        ctx->fs,
        LIBLTE_PHY_INIT_N_ID_CELL_UNKNOWN,
        4,
        LIBLTE_PHY_N_RB_DL_1_4MHZ, /*  LIBLTE_PHY_N_RB_DL_1_4MHZ, */
        LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP,
        liblte_rrc_phich_resource_num[LIBLTE_RRC_PHICH_RESOURCE_1]
    );

    LTE_Fdd_Analysis_Init_Frame(ctx);

    liblte_phy_dl_searcher_init(ctx->phy_struct);

    return 0;
}



int32_t LTE_Fdd_Analysis_Decode(struct LTE_Fdd_Analysis_s *ctx)
{
    int32_t result = 1;
    int32_t samps_to_copy;

    LOG_MSG_SET_PARENT(LOG_MSG_CODE_SEARCH);

    ctx->done_flag = 0;

#ifdef    LIBLTE_PHY_ANALYSIS
    int msg_log_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_INIT);
    if(msg_log_enable) {
        fprintf(stderr, "\nLTE_Fdd_Analysis_Decode\n");
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    while(1) {

        switch(ctx->state) {
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH:
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * COARSE_TIMING_SEARCH_NUM_SUBFRAMES;
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH:
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * PSS_AND_FINE_TIMING_SEARCH_NUM_SUBFRAMES;
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH:
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_subfr * SSS_SEARCH_NUM_SUBFRAMES;
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE:
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * BCH_DECODE_NUM_FRAMES;
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1:
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SIB1_NUM_FRAMES;
            break;
        case LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC:
            ctx->num_samps_needed = ctx->phy_struct->N_samps_per_frame * PDSCH_DECODE_SI_GENERIC_NUM_FRAMES;
            break;
        }

        result = LTE_Fdd_Analysis_Copy_Input_to_Samplebuff(ctx);
        if(result < 0) {
            break;
        }

#ifdef    LIBLTE_PHY_ANALYSIS
        if(msg_log_enable) {
            fprintf(stderr, "\n");
            fprintf(stderr, "Process LTE_FDD_DL_FS_SAMP_BUF_STATE_STATE_%s\n", LTE_Fdd_Analysis_GetStateName(ctx->state));
            fprintf(stderr, "    samp_buf_r_idx = %7d\n", ctx->samp_buf_r_idx);
            fprintf(stderr, "    samp_buf_w_idx = %7d\n", ctx->samp_buf_w_idx);
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        while((ctx->samp_buf_r_idx + ctx->num_samps_needed) <= ctx->samp_buf_w_idx)  {

            if(ctx->mib_printed == true &&
               ctx->sib1_printed == true &&
               ctx->sib2_printed == true &&
               ctx->sib3_printed == ctx->sib3_expected &&
               ctx->sib4_printed == ctx->sib4_expected &&
               ctx->sib5_printed == ctx->sib5_expected &&
               ctx->sib6_printed == ctx->sib6_expected &&
               ctx->sib7_printed == ctx->sib7_expected &&
               ctx->sib8_printed == ctx->sib8_expected &&
               ctx->sib13_printed == ctx->sib13_expected) {
                ctx->corr_peak_idx++;
                LTE_Fdd_Analysis_Init_Frame(ctx);
            }

#ifdef    LIBLTE_PHY_ANALYSIS
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_SEARCH)) {
                fprintf(stderr, "\n");
                fprintf(stderr, "Process LTE_FDD_DL_FS_SAMP_BUF_STATE_STATE_%s\n", LTE_Fdd_Analysis_GetStateName(ctx->state));
                fprintf(stderr, "    samp_buf_r_idx = %7d\n", ctx->samp_buf_r_idx);
                fprintf(stderr, "    samp_buf_w_idx = %7d\n", ctx->samp_buf_w_idx);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            result = LTE_Fdd_Analysis_Process_State(ctx);
            if(result < 0) {
                break;
            }

            if(ctx->state == LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH) {
                ctx->done_flag = -1;
            }

            if(-1 == ctx->done_flag)  {
                break;
            }

        }

        if(-1 == ctx->done_flag)  {
            break;
        }

    }

    return 0;

}

static int LTE_Fdd_Analysis_Set_BW(struct LTE_Fdd_Analysis_s *ctx)
{

    if(ctx->bandwidth == 5) {
        ctx->N_rb_dl        = LIBLTE_PHY_N_RB_DL_5MHZ;
        ctx->mib.dl_bw      = LIBLTE_RRC_DL_BANDWIDTH_25;
    }
    else if(ctx->bandwidth == 10) {
        ctx->N_rb_dl        = LIBLTE_PHY_N_RB_DL_10MHZ;
        ctx->mib.dl_bw      = LIBLTE_RRC_DL_BANDWIDTH_50;
    }
    else if(ctx->bandwidth == 15) {
        ctx->N_rb_dl        = LIBLTE_PHY_N_RB_DL_15MHZ;
        ctx->mib.dl_bw      = LIBLTE_RRC_DL_BANDWIDTH_75;
    }
    else if(ctx->bandwidth == 20) {
        ctx->N_rb_dl        = LIBLTE_PHY_N_RB_DL_20MHZ;
        ctx->mib.dl_bw      = LIBLTE_RRC_DL_BANDWIDTH_100;
    }
    else {
        ctx->bandwidth = 5;
        ctx->N_rb_dl   = LIBLTE_PHY_N_RB_DL_5MHZ;
        ctx->mib.dl_bw = LIBLTE_RRC_DL_BANDWIDTH_25;
    }

    return 0;

}

static void LTE_Fdd_Analysis_recreate_sched_info(struct LTE_Fdd_Analysis_s *ctx)
{
    LIBLTE_RRC_SIB_TYPE_ENUM sib_array[20];
    uint32_t num_sibs = 0;
    uint32_t sib_idx = 0;
    uint32_t N_sibs_to_map = 0;
    uint32_t i;
    uint32_t j;

    fprintf(stderr, "\nLTE_Fdd_Analysis_recreate_sched_info\n");

    // Determine which SIBs need to be mapped
#if 0
    fprintf(stderr, "    Determine which SIBs need to be mapped\n");
#endif

    if(1 == ctx->sib3_present) {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_3;
    }
    if(1 == ctx->sib4_present) {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_4;
    }
    if(1 == ctx->sib8_present) {
        sib_array[num_sibs++] = LIBLTE_RRC_SIB_TYPE_8;
    }

    // Initialize the scheduling info
    fprintf(stderr, "    Initialize the scheduling info\n");
    ctx->sib1.N_sched_info = 1;
    ctx->sib1.sched_info[0].N_sib_mapping_info = 0;

    // Map the SIBs
    while(num_sibs > 0) {

        // Determine how many SIBs can be mapped to this scheduling info
        fprintf(stderr, "    Determine how many SIBs can be mapped to this scheduling info\n");
        if(1 == ctx->sib1.N_sched_info)  {
            if(0 == ctx->sib1.sched_info[0].N_sib_mapping_info && LIBLTE_PHY_N_RB_DL_1_4MHZ != ctx->N_rb_dl) {
                N_sibs_to_map = 1;
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
        fprintf(stderr, "    Map the SIBs for this scheduling info\n");
        for(i = 0; i < N_sibs_to_map; i++)  {

#if 0
            fprintf(stder, "ctx->sib1.N_sched_info = %d\n", ctx->sib1.N_sched_info);
            fprintf(stder, "ctx->sib1.sched_info[%d].N_sib_mapping_info = %d\n", ctx->sib1.N_sched_info - 1, ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info);
#endif
            ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].sib_mapping_info[ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info].sib_type = sib_array[sib_idx++];

#if 0
            fprintf(stder, " ctx->sib1.sched_info[%d].sib_mapping_info[%d].sib_type = %d\n",
                ctx->sib1.N_sched_info - 1,
                ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info,
                ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].sib_mapping_info[ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info].sib_type
            );
#endif

            ctx->sib1.sched_info[ctx->sib1.N_sched_info-1].N_sib_mapping_info++;
            num_sibs--;
            if(0 == num_sibs) {
                break;
            }
        }

    }

    fprintf(stderr, "\nctx->sib1.sched_info = %d\n", ctx->sib1.N_sched_info);

    for(i = 0; i < ctx->sib1.N_sched_info; i++) {
        fprintf(stderr, "ctx->sib1.sched_info[%d]\n", i);
        fprintf(stderr, "    ctx->sib1.sched_info[%d].N_sib_mapping_info = %d\n", i, ctx->sib1.sched_info[i].N_sib_mapping_info);
        for(j = 0; j < ctx->sib1.sched_info[i].N_sib_mapping_info; j++) {
            fprintf(stderr, "    %s (%d)\n",
                Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[i].sib_mapping_info[j].sib_type),
                ctx->sib1.sched_info[i].sib_mapping_info[j].sib_type);
        }
        fprintf(stderr, "\n");
    }

}

int32_t LTE_Fdd_Analysis_Draw_Bitmap(struct LTE_Fdd_Analysis_s *ctx, uint8 *bmp_buffer, int re_k, int re_l, int lte_ch)
{
    struct PixelColor *color = NULL;
    int i;
    int j;
    int x;
    int y;

    static struct LTEChannel2Color colortable[] =
    {
        { LTE_PSS,     { 255,   0,   0 } },
        { LTE_SSS,     {   0, 255,   0 } },
        { LTE_CRS,     {   0,   0, 200 } },
        { LTE_BCCH,    { 255, 255,   0 } },
        { LTE_PCFICH,  { 255,   0, 255 } },
        { LTE_PHICH,   {   0, 255, 255 } },
        { LTE_PDCCH,   { 255, 100, 200 } },
        { LTE_PDSCH,   { 255,   0,   0 } },
        { LTE_CH_MAX,  {   0,   0,   0 } },
    };

    for(i = 0; colortable[i].lte_ch != LTE_CH_MAX; i++ ) {
        if(colortable[i].lte_ch == lte_ch) {
            color = &colortable[i].color;
            break;
        }
    }

    if(color != NULL) {
        for(i = 1, y = re_k * PIXEL_PER_RE + 1; i < PIXEL_PER_RE; i++, y++) {
            for(j = 1, x = re_l * PIXEL_PER_RE + 1; j < PIXEL_PER_RE; j++, x++) {
                memcpy(&bmp_buffer[((ctx->image_height - y) * ctx->image_width + x) * BYTES_PER_PIXEL], color, BYTES_PER_PIXEL);
            }
        }
    }

    return 0;
}



int32_t LTE_Fdd_Analysis_Draw_Frame(struct LTE_Fdd_Analysis_s *ctx)
{

    uint32 p;
    uint32 s;
    uint32 re_k;
    uint32 re_l;
    uint32 base_l = ctx->subframe.num * ctx->symbol_per_slot * ctx->slot_per_subframe;

    for(p = 0; p < ctx->N_ant; p++) {
        for(re_l = 0; re_l < ctx->symbol_per_slot * ctx->slot_per_subframe; re_l++) {
            for(re_k = 0; re_k < ctx->N_rb_dl * ctx->N_sc_rb_dl; re_k++) {
                if(ctx->subframe.tx_symb[p][re_l][re_k] != 0) {
                    LTE_Fdd_Analysis_Draw_Bitmap(ctx, ctx->bitmap_buffer[p], re_k, base_l + re_l, ctx->subframe.tx_symb[p][re_l][re_k]);
                }
            }
        }
    }

    return 0;
}

static int32_t LTE_Fdd_Analysis_Encode_Init(struct LTE_Fdd_Analysis_s *ctx)
{
    int i;
    int j;

    ctx->fs     = LIBLTE_PHY_FS_30_72MHZ;
    ctx->i_buf  = (float *)malloc(LTE_FDD_DL_FS_SAMP_BUF_SIZE*sizeof(float));
    ctx->q_buf  = (float *)malloc(LTE_FDD_DL_FS_SAMP_BUF_SIZE*sizeof(float));

    ctx->samp_buf_w_idx = 0;
    ctx->samp_buf_r_idx = 0;

    ctx->input_offset = 0;

    LTE_Fdd_Analysis_Init_Frame(ctx);

    // Parse the inputs
    ctx->out_size = LTE_FDD_DL_FG_OUT_SIZE_GR_COMPLEX;
    ctx->log_msg_parent = 0;

    // LOG_MSG_CODE_PSS /
    // LOG_MSG_CODE_SSS

    ctx->symbol_per_slot = 7;
    ctx->slot_per_subframe = 2;


    // Initialize the LTE parameters
    // General
    ctx->bandwidth    = 5;
    ctx->fs           = LIBLTE_PHY_FS_30_72MHZ;
    ctx->sfn          = 0;
    ctx->N_frames     = 30;
    ctx->N_ant        = 1;
    ctx->N_id_cell    = 123;    // Configure
    ctx->N_id_2       = (ctx->N_id_cell % 3);
    ctx->N_id_1       = (ctx->N_id_cell - ctx->N_id_2)/3;
    ctx->sib_tx_mode  = 1;
    ctx->percent_load = 0;      // Configure
    ctx->N_sc_rb_dl   = 12;

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
    ctx->bandwidth = 10;
    ctx->N_rb_dl   = LIBLTE_PHY_N_RB_DL_10MHZ;
    ctx->mib.dl_bw = LIBLTE_RRC_DL_BANDWIDTH_50;

    // set_fs
    ctx->fs = LIBLTE_PHY_FS_30_72MHZ;

    // set_param(&sib1.freq_band_indicator, value, 1, 25);
    ctx->sib1.freq_band_indicator = 1;

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
    LTE_Fdd_Analysis_Set_BW(ctx);

    return 0;
}



int32_t LTE_Fdd_Analysis_Encode(struct LTE_Fdd_Analysis_s *ctx)
{
    float       i_samp;
    float       q_samp;
    int32       act_noutput_items;
    uint32      out_idx;
    uint32      loop_cnt;
    uint32      i;
    uint32      j;
    uint32      k;
    uint32      p;
    uint32      N_sfr;
    uint32      last_prb;
    uint32      max_N_prb;
    size_t      line_size = LINE_MAX;
    ssize_t     N_line_chars;
    char       *line;
    bool        done = false;

    LIBLTE_PHY_ALLOCATION_STRUCT *alloc;

    ctx->output_offset = 0;

    int msg_log_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_ENCODE);
    if(msg_log_enable) {
        fprintf(stderr, "\nLTE_Fdd_Analysis_Encode\n");
    }

    liblte_phy_init(&ctx->phy_struct,
                    ctx->fs,
                    ctx->N_id_cell,
                    ctx->N_ant,
                    ctx->N_rb_dl,
                    LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP,
                    ctx->phich_res);


    for(ctx->sfn = 0; ctx->sfn < ctx->N_frames; ctx->sfn += 1) {

        InitBitmap(ctx);

        for(N_sfr=0; N_sfr < 10; N_sfr++) {

            // Initialize the output to all zeros
            for(p = 0; p < ctx->N_ant; p++) {
                for(j = 0; j < 16; j++) {
                    for(k = 0; k < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; k++) {
                        ctx->subframe.tx_symb_re[p][j][k] = 0;
                        ctx->subframe.tx_symb_im[p][j][k] = 0;

                        ctx->subframe.tx_symb[p][j][k] = 0;
                    }
                }
            }

            ctx->subframe.num = N_sfr;

            if(ctx->subframe.num == 0 || ctx->subframe.num == 5) {

                liblte_phy_map_pss(ctx->phy_struct,
                                   &ctx->subframe,
                                   ctx->N_id_2,
                                   ctx->N_ant);

                liblte_phy_map_sss(ctx->phy_struct,
                                   &ctx->subframe,
                                   ctx->N_id_1,
                                   ctx->N_id_2,
                                   ctx->N_ant);
            }

            liblte_phy_map_crs(ctx->phy_struct,
                               &ctx->subframe,
                               ctx->N_id_cell,
                               ctx->N_ant);

            if(ctx->subframe.num == 0) {

                ctx->mib.sfn_div_4 = ctx->sfn / 4;

                liblte_rrc_pack_bcch_bch_msg(&ctx->mib, &ctx->rrc_msg);

                liblte_phy_bch_channel_encode(ctx->phy_struct,
                                              ctx->rrc_msg.msg,
                                              ctx->rrc_msg.N_bits,
                                              ctx->N_id_cell,
                                              ctx->N_ant,
                                              &ctx->subframe,
                                              ctx->sfn);

            }

            LOG_MSG_SET_PARENT(LOG_MSG_CODE_SIB_INFO);

            ctx->pdcch.N_alloc = 0;
            if(ctx->subframe.num == 5 && (ctx->sfn % 2) == 0)  {

                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "\n###### SIB1 ######\n");
                }

                // SIB1
                ctx->bcch_dlsch_msg.N_sibs           = 0;
                ctx->bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1;
                memcpy(&ctx->bcch_dlsch_msg.sibs[0].sib, &ctx->sib1, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT));

                liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg,
                                               &ctx->pdcch.alloc[ctx->pdcch.N_alloc].msg[0]);

                alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];

                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                    print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                }

                liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
                                                        ctx->subframe.num,
                                                        ctx->N_rb_dl,
                                                        LIBLTE_MAC_SI_RNTI,
                                                        &alloc->tbs,
                                                        &alloc->mcs,
                                                        &alloc->N_prb);

                alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
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

                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                    fprintf(stderr, "pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                    fprintf(stderr, "pdcch.alloc[%d].chan_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_chan_type_text[alloc->chan_type], alloc->chan_type);
                    fprintf(stderr, "pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                    fprintf(stderr, "pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                    fprintf(stderr, "pdcch.alloc[%d].rnti = %d\n", ctx->pdcch.N_alloc, alloc->rnti);
                    fprintf(stderr, "pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                    fprintf(stderr, "pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                    fprintf(stderr, "pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                    fprintf(stderr, "pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
                }

                ctx->pdcch.N_alloc++;

            }

            if(ctx->subframe.num             >=  (0 * ctx->si_win_len)%10 &&
               ctx->subframe.num             <   (1 * ctx->si_win_len)%10 &&
               (ctx->sfn % ctx->si_periodicity_T) == ((0 * ctx->si_win_len)/10))   {

                // SIs in 1st scheduling info list entry
                ctx->bcch_dlsch_msg.N_sibs           = 1;
                ctx->bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2;

                memcpy(&ctx->bcch_dlsch_msg.sibs[0].sib, &ctx->sib2, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT));

#ifdef    LIBLTE_PHY_ANALYSIS
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "\n[1]SIs in 1st scheduling info list entry\n");
                    fprintf(stderr, "bcch_dlsch_msg.sibs[0].sib_type = %s (%d)\n", Get_SIB_TYPE_Info_Str(ctx->bcch_dlsch_msg.sibs[0].sib_type), ctx->bcch_dlsch_msg.sibs[0].sib_type);
                    fprintf(stderr, "    sib1.sched_info[0].N_sib_mapping_info = %d\n", ctx->sib1.sched_info[0].N_sib_mapping_info);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                if(ctx->sib1.sched_info[0].N_sib_mapping_info != 0) {

#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                        fprintf(stderr, "    sib1.sched_info[0].sib_mapping_info[0].sib_type = %s (%d)\n",
                            Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type),
                            ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type);
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
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                    print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                // FIXME: This was a hack to allow SIB2 decoding with 1.4MHz BW due to overlap with MIB
                if(LIBLTE_SUCCESS == liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
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
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                        fprintf(stderr, "\nfor SIB2\n");
                        fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                        fprintf(stderr, "    pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                        fprintf(stderr, "    pdcch.alloc[%d].chan_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_chan_type_text[alloc->chan_type], alloc->chan_type);
                        fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                        fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                        fprintf(stderr, "    pdcch.alloc[%d].rnti = %d\n", ctx->pdcch.N_alloc, alloc->rnti);
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
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                if(ctx->sib1.N_sched_info) {
                    fprintf(stderr, "\nsib1.N_sched_info = %d\n", ctx->sib1.N_sched_info);
                    fprintf(stderr, "    subframe.num = %d\n", ctx->subframe.num);
                    fprintf(stderr, "    sfn = %d\n", ctx->sfn);
                    fprintf(stderr, "    si_win_len = %d\n", ctx->si_win_len);
                    fprintf(stderr, "    si_periodicity_T = %d\n", ctx->si_periodicity_T);
                }
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            for(j = 1; j < ctx->sib1.N_sched_info; j++) {

                if(ctx->subframe.num             ==  (j * ctx->si_win_len)%10 &&
                   (ctx->sfn % ctx->si_periodicity_T) == ((j * ctx->si_win_len)/10))   {

                    // SIs in the jth scheduling info list entry
#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                        fprintf(stderr, "\n[2]SIs in the jth scheduling info list entry\n");
                        fprintf(stderr, "    sib1.sched_info[%d].N_sib_mapping_info = %d\n", j, ctx->sib1.sched_info[j].N_sib_mapping_info);
                    }
#endif /* LIBLTE_PHY_ANALYSIS */
                    ctx->bcch_dlsch_msg.N_sibs = ctx->sib1.sched_info[j].N_sib_mapping_info;

                    for(i = 0; i < ctx->bcch_dlsch_msg.N_sibs; i++) {

#ifdef    LIBLTE_PHY_ANALYSIS
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "    ctx->sib1.sched_info[%d].sib_mapping_info[%d].sib_type = %s (%d)\n", j, i,
                                Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type),
                                ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type);
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
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "\nliblte_rrc_pack_bcch_dlsch_msg\n");
                            fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
                            fprintf(stderr, "    bcch_dlsch_msg.N_sibs = %d\n", ctx->bcch_dlsch_msg.N_sibs);
                        }
#endif /* LIBLTE_PHY_ANALYSIS */

                        alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];

                        liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg,
                                                       &alloc->msg[0]);

#ifdef    LIBLTE_PHY_ANALYSIS
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                            print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                        }
#endif /* LIBLTE_PHY_ANALYSIS */

                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
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
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "\n");
                            fprintf(stderr, "for SIBs\n");
                            fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                            fprintf(stderr, "    pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                            fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                            fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                            fprintf(stderr, "    pdcch.alloc[%d].rnti = 0x%04x, %d\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti);
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

#ifdef    LIBLTE_PHY_ANALYSIS
            LOG_MSG_SET_PARENT(0);
#endif /* LIBLTE_PHY_ANALYSIS */

            // Add test load
            if(0 == ctx->pdcch.N_alloc) {

#ifdef    LIBLTE_PHY_ANALYSIS
                LOG_MSG_SET_PARENT(LOG_MSG_CODE_PAYLOAD);
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD)) {
                    fprintf(stderr, "\nAdd test load\n");
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                alloc = &ctx->pdcch.alloc[ctx->pdcch.N_alloc];
                alloc->msg[0].N_bits = 0;
                alloc->N_prb         = 0;

                if(ctx->Modulation_Type == LIBLTE_PHY_MODULATION_TYPE_QPSK) {

                    liblte_phy_get_tbs_mcs_and_n_prb_for_dl(1480,
                                                            ctx->subframe.num,
                                                            ctx->N_rb_dl,
                                                            LIBLTE_MAC_P_RNTI,
                                                            &alloc->tbs,
                                                            &alloc->mcs,
                                                            &max_N_prb);

                    while(alloc->N_prb < (uint32)((float)(max_N_prb*ctx->percent_load)/100.0))   {
                        alloc->msg[0].N_bits += 8;
                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
                                                                ctx->subframe.num,
                                                                ctx->N_rb_dl,
                                                                LIBLTE_MAC_P_RNTI,
                                                                &alloc->tbs,
                                                                &alloc->mcs,
                                                                &alloc->N_prb);
                    }

                }
                else {

                    if(ctx->Modulation_Type == LIBLTE_PHY_MODULATION_TYPE_16QAM) {

                        alloc->msg[0].N_bits = LTE_TEST_16QAM_BITS_COUNT;
                        alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;

                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type(alloc->msg[0].N_bits,
                                                                ctx->subframe.num,
                                                                ctx->N_rb_dl,
                                                                LIBLTE_MAC_P_RNTI,
                                                                &alloc->tbs,
                                                                &alloc->mcs,
                                                                &alloc->N_prb,
                                                                alloc->mod_type );
                    }
                    else if(ctx->Modulation_Type == LIBLTE_PHY_MODULATION_TYPE_64QAM) {
                        alloc->msg[0].N_bits = LTE_TEST_64QAM_BITS_COUNT;
                        alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;

                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type(alloc->msg[0].N_bits,
                                                                ctx->subframe.num,
                                                                ctx->N_rb_dl,
                                                                LIBLTE_MAC_P_RNTI,
                                                                &alloc->tbs,
                                                                &alloc->mcs,
                                                                &alloc->N_prb,
                                                                alloc->mod_type );

                    }

                }

                for(i=0; i<alloc->msg[0].N_bits; i++)
                {
                    alloc->msg[0].msg[i] = liblte_rrc_test_fill[i%8];
                }

                if(0 != alloc->N_prb)  {

                    alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                    alloc->N_codewords = 1;
                    alloc->rnti        = LIBLTE_MAC_P_RNTI;
                    alloc->tx_mode     = ctx->sib_tx_mode;

#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD)) {
                         fprintf(stderr, "\n");
                         fprintf(stderr, "for TEST LOAD\n");
                         fprintf(stderr, "pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                         fprintf(stderr, "pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                         fprintf(stderr, "pdcch.alloc[%d].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[ctx->pdcch.N_alloc].N_bits);
                         fprintf(stderr, "pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
                         fprintf(stderr, "pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                         fprintf(stderr, "pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                         fprintf(stderr, "pdcch.alloc[%d].rnti = 0x%04x, %d\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti);
                         fprintf(stderr, "pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                         fprintf(stderr, "pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                         fprintf(stderr, "pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                     }
#endif /* LIBLTE_PHY_ANALYSIS */

                    ctx->pdcch.N_alloc++;
                }
#ifdef    LIBLTE_PHY_ANALYSIS
                LOG_MSG_SET_PARENT(0);
#endif /* LIBLTE_PHY_ANALYSIS */

            }

            // Schedule all allocations
#ifdef    LIBLTE_PHY_ANALYSIS
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO) || LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD)) {
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
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO) || LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD) || LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH)) {
                    fprintf(stderr, "\nliblte_phy_pdcch_channel_encode\n");
                    fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                liblte_phy_pdcch_channel_encode(ctx->phy_struct,
                                                &ctx->pcfich,
                                                &ctx->phich,
                                                &ctx->pdcch,
                                                ctx->N_id_cell,
                                                ctx->N_ant,
                                                ctx->phich_res,
                                                ctx->mib.phich_config.dur,
                                                &ctx->subframe);

#ifdef    LIBLTE_PHY_ANALYSIS
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO) || LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD) || LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH) || LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH)) {
                    fprintf(stderr, "\nliblte_phy_pdsch_channel_encode\n");
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                liblte_phy_pdsch_channel_encode(ctx->phy_struct,
                                                &ctx->pdcch,
                                                ctx->N_id_cell,
                                                ctx->N_ant,
                                                &ctx->subframe);
            }

            // Construct the output
            for(p = 0; p < ctx->N_ant; p++) {
                liblte_phy_create_dl_subframe(ctx->phy_struct,
                                              &ctx->subframe,
                                              p,
                                              &ctx->i_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)],
                                              &ctx->q_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)]);

#ifdef    LIBLTE_PHY_ANALYSIS
                int32_t display_len;
                if(ctx->subframe.num == 0) {
                    display_len = ctx->phy_struct->N_samps_cp_l_0 + ctx->phy_struct->N_samps_per_symb;
                }
                else {
                    display_len = ctx->phy_struct->N_samps_cp_l_else + ctx->phy_struct->N_samps_per_symb;
                }
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_CREATE_DL_FRAME)) {
                    fprintf(stderr, "\nliblte_phy_create_dl_subframe\n");
                    print_complex_number(&ctx->i_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)],
                        &ctx->q_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)],
                        display_len, 0);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

            }

            LTE_Fdd_Analysis_Draw_Frame(ctx);


        }

        if(ctx->sfn < ctx->N_frames) {
            ctx->outdata_cnt = 0;
            ctx->samp_buf_idx = 0;
            loop_cnt = ctx->phy_struct->N_samps_per_frame;
            for(i = 0; i < loop_cnt; i++) {
                i_samp = 0;
                q_samp = 0;
                for(p = 0; p < ctx->N_ant; p++) {
                    i_samp += ctx->i_buf[(p*ctx->phy_struct->N_samps_per_frame) + ctx->samp_buf_idx];
                    q_samp += ctx->q_buf[(p*ctx->phy_struct->N_samps_per_frame) + ctx->samp_buf_idx];
                }
                ctx->outdata[ctx->outdata_cnt].i_data = i_samp;
                ctx->outdata[ctx->outdata_cnt].q_data = q_samp;
                ctx->samp_buf_idx++;
                ctx->outdata_cnt++;
            }

            if(ctx->outf != NULL) {
                // fprintf(stderr, "ctx->sfn = %d, file offset : 0x%08x, %d\n", ctx->sfn, ctx->output_offset, ctx->output_offset);
                fwrite(ctx->outdata, 1, sizeof(struct complex_data_f) * ctx->outdata_cnt, ctx->outf);
                ctx->output_offset += sizeof(struct complex_data_f) * ctx->outdata_cnt;
            }
        }

        SaveBitmap(ctx);

    }

    fprintf(stderr, "LTE_Fdd_Analysis_Encode : DONE\n");

    return 0;
}



int32_t LTE_Fdd_Analysis_Encode2(struct LTE_Fdd_Analysis_s *ctx)
{
    float       i_samp;
    float       q_samp;
    int32       act_noutput_items;
    uint32      out_idx;
    uint32      loop_cnt;
    uint32      i;
    uint32      j;
    uint32      k;
    uint32      p;
    uint32      N_sfr;
    uint32      last_prb;
    uint32      max_N_prb;
    size_t      line_size = LINE_MAX;
    ssize_t     N_line_chars;
    char       *line;
    bool        done = false;

    LIBLTE_PHY_ALLOCATION_STRUCT *alloc;

    ctx->output_offset = 0;

#ifdef    LIBLTE_PHY_ANALYSIS
    int msg_log_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_ENCODE);
    if(msg_log_enable) {
        fprintf(stderr, "\nLTE_Fdd_Analysis_Encode\n");
    }
#endif /* LIBLTE_PHY_ANALYSIS */

    liblte_phy_init(&ctx->phy_struct,
                    ctx->fs,
                    ctx->N_id_cell,
                    ctx->N_ant,
                    ctx->N_rb_dl,
                    LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP,
                    ctx->phich_res);


    for(ctx->sfn = 0; ctx->sfn < ctx->N_frames; ctx->sfn += 1) {

        InitBitmap(ctx);

        for(N_sfr=0; N_sfr < 10; N_sfr++) {

            // Initialize the output to all zeros
            for(p = 0; p < ctx->N_ant; p++) {
                for(j = 0; j < 16; j++) {
                    for(k = 0; k < LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP; k++) {
                        ctx->subframe.tx_symb_re[p][j][k] = 0;
                        ctx->subframe.tx_symb_im[p][j][k] = 0;

                        ctx->subframe.tx_symb[p][j][k] = 0;
                    }
                }
            }

            ctx->subframe.num = N_sfr;

            if(ctx->subframe.num == 0 || ctx->subframe.num == 5) {

                liblte_phy_map_pss(ctx->phy_struct,
                                   &ctx->subframe,
                                   ctx->N_id_2,
                                   ctx->N_ant);

                liblte_phy_map_sss(ctx->phy_struct,
                                   &ctx->subframe,
                                   ctx->N_id_1,
                                   ctx->N_id_2,
                                   ctx->N_ant);
            }

            liblte_phy_map_crs(ctx->phy_struct,
                               &ctx->subframe,
                               ctx->N_id_cell,
                               ctx->N_ant);

            if(ctx->subframe.num == 0) {

                ctx->mib.sfn_div_4 = ctx->sfn / 4;

                liblte_rrc_pack_bcch_bch_msg(&ctx->mib, &ctx->rrc_msg);

                liblte_phy_bch_channel_encode(ctx->phy_struct,
                                              ctx->rrc_msg.msg,
                                              ctx->rrc_msg.N_bits,
                                              ctx->N_id_cell,
                                              ctx->N_ant,
                                              &ctx->subframe,
                                              ctx->sfn);

            }

#ifdef    LIBLTE_PHY_ANALYSIS
            LOG_MSG_SET_PARENT(LOG_MSG_CODE_SIB_INFO);
#endif /* LIBLTE_PHY_ANALYSIS */

            ctx->pdcch.N_alloc = 0;
            if(ctx->subframe.num == 5 && (ctx->sfn % 2) == 0)  {

#ifdef    LIBLTE_PHY_ANALYSIS
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "\n###### SIB1 ######\n");
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                // SIB1
                ctx->bcch_dlsch_msg.N_sibs           = 0;
                ctx->bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1;
                memcpy(&ctx->bcch_dlsch_msg.sibs[0].sib, &ctx->sib1, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT));

                liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg,
                                               &ctx->pdcch.alloc[ctx->pdcch.N_alloc].msg[0]);

                 alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];

#ifdef    LIBLTE_PHY_ANALYSIS
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                    print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
                                                        ctx->subframe.num,
                                                        ctx->N_rb_dl,
                                                        LIBLTE_MAC_SI_RNTI,
                                                        &alloc->tbs,
                                                        &alloc->mcs,
                                                        &alloc->N_prb);

                alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
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
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                    fprintf(stderr, "pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                    fprintf(stderr, "pdcch.alloc[%d].chan_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_chan_type_text[alloc->chan_type], alloc->chan_type);
                    fprintf(stderr, "pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                    fprintf(stderr, "pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                    fprintf(stderr, "pdcch.alloc[%d].rnti = %d\n", ctx->pdcch.N_alloc, alloc->rnti);
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

                // SIs in 1st scheduling info list entry
                ctx->bcch_dlsch_msg.N_sibs           = 1;
                ctx->bcch_dlsch_msg.sibs[0].sib_type = LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2;

                memcpy(&ctx->bcch_dlsch_msg.sibs[0].sib, &ctx->sib2, sizeof(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT));

#ifdef    LIBLTE_PHY_ANALYSIS
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "\n[1]SIs in 1st scheduling info list entry\n");
                    fprintf(stderr, "bcch_dlsch_msg.sibs[0].sib_type = %s (%d)\n", Get_SIB_TYPE_Info_Str(ctx->bcch_dlsch_msg.sibs[0].sib_type), ctx->bcch_dlsch_msg.sibs[0].sib_type);
                    fprintf(stderr, "    sib1.sched_info[0].N_sib_mapping_info = %d\n", ctx->sib1.sched_info[0].N_sib_mapping_info);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                if(ctx->sib1.sched_info[0].N_sib_mapping_info != 0) {

#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                        fprintf(stderr, "    sib1.sched_info[0].sib_mapping_info[0].sib_type = %s (%d)\n",
                            Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type),
                            ctx->sib1.sched_info[0].sib_mapping_info[0].sib_type);
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
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                    fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                    print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                // FIXME: This was a hack to allow SIB2 decoding with 1.4MHz BW due to overlap with MIB
                if(LIBLTE_SUCCESS == liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
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
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                        fprintf(stderr, "\nfor SIB2\n");
                        fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                        fprintf(stderr, "    pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                        fprintf(stderr, "    pdcch.alloc[%d].chan_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_chan_type_text[alloc->chan_type], alloc->chan_type);
                        fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                        fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                        fprintf(stderr, "    pdcch.alloc[%d].rnti = %d\n", ctx->pdcch.N_alloc, alloc->rnti);
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
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                if(ctx->sib1.N_sched_info) {
                    fprintf(stderr, "\nsib1.N_sched_info = %d\n", ctx->sib1.N_sched_info);
                    fprintf(stderr, "    subframe.num = %d\n", ctx->subframe.num);
                    fprintf(stderr, "    sfn = %d\n", ctx->sfn);
                    fprintf(stderr, "    si_win_len = %d\n", ctx->si_win_len);
                    fprintf(stderr, "    si_periodicity_T = %d\n", ctx->si_periodicity_T);
                }
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            for(j = 1; j < ctx->sib1.N_sched_info; j++) {

                if(ctx->subframe.num             ==  (j * ctx->si_win_len)%10 &&
                   (ctx->sfn % ctx->si_periodicity_T) == ((j * ctx->si_win_len)/10))   {

                    // SIs in the jth scheduling info list entry
#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                        fprintf(stderr, "\n[2]SIs in the jth scheduling info list entry\n");
                        fprintf(stderr, "    sib1.sched_info[%d].N_sib_mapping_info = %d\n", j, ctx->sib1.sched_info[j].N_sib_mapping_info);
                    }
#endif /* LIBLTE_PHY_ANALYSIS */
                    ctx->bcch_dlsch_msg.N_sibs = ctx->sib1.sched_info[j].N_sib_mapping_info;

                    for(i = 0; i < ctx->bcch_dlsch_msg.N_sibs; i++) {

#ifdef    LIBLTE_PHY_ANALYSIS
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "    ctx->sib1.sched_info[%d].sib_mapping_info[%d].sib_type = %s (%d)\n", j, i,
                                Get_Rrc_Sib_Type_Str(ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type),
                                ctx->sib1.sched_info[j].sib_mapping_info[i].sib_type);
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
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "\nliblte_rrc_pack_bcch_dlsch_msg\n");
                            fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
                            fprintf(stderr, "    bcch_dlsch_msg.N_sibs = %d\n", ctx->bcch_dlsch_msg.N_sibs);
                        }
#endif /* LIBLTE_PHY_ANALYSIS */

                        alloc =  &ctx->pdcch.alloc[ctx->pdcch.N_alloc];

                        liblte_rrc_pack_bcch_dlsch_msg(&ctx->bcch_dlsch_msg,
                                                       &alloc->msg[0]);

#ifdef    LIBLTE_PHY_ANALYSIS
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "    pdcch.alloc[%d].msg[0].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[0].N_bits);
                            print_bits((uint8_t *) alloc->msg[0].msg, alloc->msg[0].N_bits);
                        }
#endif /* LIBLTE_PHY_ANALYSIS */

                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
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
                        if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO)) {
                            fprintf(stderr, "\n");
                            fprintf(stderr, "for SIBs\n");
                            fprintf(stderr, "    pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                            fprintf(stderr, "    pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                            fprintf(stderr, "    pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                            fprintf(stderr, "    pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                            fprintf(stderr, "    pdcch.alloc[%d].rnti = 0x%04x, %d\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti);
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

#ifdef    LIBLTE_PHY_ANALYSIS
            LOG_MSG_SET_PARENT(0);
#endif /* LIBLTE_PHY_ANALYSIS */

#ifdef CHECK_CODE
            // Add test load
            if(0 == ctx->pdcch.N_alloc) {

#ifdef    LIBLTE_PHY_ANALYSIS
                LOG_MSG_SET_PARENT(LOG_MSG_CODE_PAYLOAD);
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD)) {
                    fprintf(stderr, "\nAdd test load\n");
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                alloc = &ctx->pdcch.alloc[ctx->pdcch.N_alloc];
                alloc->msg[0].N_bits = 0;
                alloc->N_prb         = 0;

                if(ctx->Modulation_Type == LIBLTE_PHY_MODULATION_TYPE_QPSK) {

                    liblte_phy_get_tbs_mcs_and_n_prb_for_dl(1480,
                                                            ctx->subframe.num,
                                                            ctx->N_rb_dl,
                                                            LIBLTE_MAC_P_RNTI,
                                                            &alloc->tbs,
                                                            &alloc->mcs,
                                                            &max_N_prb);

                    while(alloc->N_prb < (uint32)((float)(max_N_prb*ctx->percent_load)/100.0))   {
                        alloc->msg[0].N_bits += 8;
                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl(alloc->msg[0].N_bits,
                                                                ctx->subframe.num,
                                                                ctx->N_rb_dl,
                                                                LIBLTE_MAC_P_RNTI,
                                                                &alloc->tbs,
                                                                &alloc->mcs,
                                                                &alloc->N_prb);
                    }

                }
                else {

                    if(ctx->Modulation_Type == LIBLTE_PHY_MODULATION_TYPE_16QAM) {

                        alloc->msg[0].N_bits = LTE_TEST_16QAM_BITS_COUNT;
                        alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;

                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type(alloc->msg[0].N_bits,
                                                                ctx->subframe.num,
                                                                ctx->N_rb_dl,
                                                                LIBLTE_MAC_P_RNTI,
                                                                &alloc->tbs,
                                                                &alloc->mcs,
                                                                &alloc->N_prb,
                                                                alloc->mod_type );
                    }
                    else if(ctx->Modulation_Type == LIBLTE_PHY_MODULATION_TYPE_64QAM) {
                        alloc->msg[0].N_bits = LTE_TEST_64QAM_BITS_COUNT;
                        alloc->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;

                        liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type(alloc->msg[0].N_bits,
                                                                ctx->subframe.num,
                                                                ctx->N_rb_dl,
                                                                LIBLTE_MAC_P_RNTI,
                                                                &alloc->tbs,
                                                                &alloc->mcs,
                                                                &alloc->N_prb,
                                                                alloc->mod_type );

                    }

                }

                for(i=0; i<alloc->msg[0].N_bits; i++)
                {
                    alloc->msg[0].msg[i] = liblte_rrc_test_fill[i%8];
                }

                if(0 != alloc->N_prb)  {

                    alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
                    alloc->N_codewords = 1;
                    alloc->rnti        = LIBLTE_MAC_P_RNTI;
                    alloc->tx_mode     = ctx->sib_tx_mode;

#ifdef    LIBLTE_PHY_ANALYSIS
                    if(LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD)) {
                         fprintf(stderr, "\n");
                         fprintf(stderr, "for TEST LOAD\n");
                         fprintf(stderr, "pdcch.alloc[%d].pre_coder_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_pre_coder_type_text[alloc->pre_coder_type], alloc->pre_coder_type);
                         fprintf(stderr, "pdcch.alloc[%d].mod_type = %s (%d)\n", ctx->pdcch.N_alloc, liblte_phy_modulation_type_text[alloc->mod_type], alloc->mod_type);
                         fprintf(stderr, "pdcch.alloc[%d].N_bits = %d\n", ctx->pdcch.N_alloc, alloc->msg[ctx->pdcch.N_alloc].N_bits);
                         fprintf(stderr, "pdcch.alloc[%d].N_prb = %d\n", ctx->pdcch.N_alloc, alloc->N_prb);
                         fprintf(stderr, "pdcch.alloc[%d].rv_idx = %d\n", ctx->pdcch.N_alloc, alloc->rv_idx);
                         fprintf(stderr, "pdcch.alloc[%d].N_codewords = %d\n", ctx->pdcch.N_alloc, alloc->N_codewords);
                         fprintf(stderr, "pdcch.alloc[%d].rnti = 0x%04x, %d\n", ctx->pdcch.N_alloc, alloc->rnti, alloc->rnti);
                         fprintf(stderr, "pdcch.alloc[%d].tx_mode = %d\n", ctx->pdcch.N_alloc, alloc->tx_mode);
                         fprintf(stderr, "pdcch.alloc[%d].tbs = %d\n", ctx->pdcch.N_alloc, alloc->tbs);
                         fprintf(stderr, "pdcch.alloc[%d].mcs = %d\n", ctx->pdcch.N_alloc, alloc->mcs);
                     }
#endif /* LIBLTE_PHY_ANALYSIS */

                    ctx->pdcch.N_alloc++;
                }
#ifdef    LIBLTE_PHY_ANALYSIS
                LOG_MSG_SET_PARENT(0);
#endif /* LIBLTE_PHY_ANALYSIS */

            }

#endif /* CHECK_CODE */

            // Schedule all allocations
#ifdef    LIBLTE_PHY_ANALYSIS
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO) || LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD)) {
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
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO) || LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD) || LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH)) {
                    fprintf(stderr, "\nliblte_phy_pdcch_channel_encode\n");
                    fprintf(stderr, "    pdcch.N_alloc = %d\n", ctx->pdcch.N_alloc);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                liblte_phy_pdcch_channel_encode(ctx->phy_struct,
                                                &ctx->pcfich,
                                                &ctx->phich,
                                                &ctx->pdcch,
                                                ctx->N_id_cell,
                                                ctx->N_ant,
                                                ctx->phich_res,
                                                ctx->mib.phich_config.dur,
                                                &ctx->subframe);

#ifdef    LIBLTE_PHY_ANALYSIS
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_SIB_INFO) || LOG_MSG_ENABLE(LOG_MSG_CODE_PAYLOAD) || LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH) || LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH)) {
                    fprintf(stderr, "\nliblte_phy_pdsch_channel_encode\n");
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                liblte_phy_pdsch_channel_encode(ctx->phy_struct,
                                                &ctx->pdcch,
                                                ctx->N_id_cell,
                                                ctx->N_ant,
                                                &ctx->subframe);
            }

            // Construct the output
            for(p = 0; p < ctx->N_ant; p++) {
                liblte_phy_create_dl_subframe(ctx->phy_struct,
                                              &ctx->subframe,
                                              p,
                                              &ctx->i_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)],
                                              &ctx->q_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)]);

#ifdef    LIBLTE_PHY_ANALYSIS
                int32_t display_len;
                if(ctx->subframe.num == 0) {
                    display_len = ctx->phy_struct->N_samps_cp_l_0 + ctx->phy_struct->N_samps_per_symb;
                }
                else {
                    display_len = ctx->phy_struct->N_samps_cp_l_else + ctx->phy_struct->N_samps_per_symb;
                }
                if(LOG_MSG_ENABLE(LOG_MSG_CODE_CREATE_DL_FRAME)) {
                    fprintf(stderr, "\nliblte_phy_create_dl_subframe\n");
                    print_complex_number(&ctx->i_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)],
                        &ctx->q_buf[(p*ctx->phy_struct->N_samps_per_frame) + (ctx->subframe.num*ctx->phy_struct->N_samps_per_subfr)],
                        display_len, 0);
                }
#endif /* LIBLTE_PHY_ANALYSIS */

            }

            LTE_Fdd_Analysis_Draw_Frame(ctx);


        }

        if(ctx->sfn < ctx->N_frames) {
            ctx->outdata_cnt = 0;
            ctx->samp_buf_idx = 0;
            loop_cnt = ctx->phy_struct->N_samps_per_frame;
            for(i = 0; i < loop_cnt; i++) {
                i_samp = 0;
                q_samp = 0;
                for(p = 0; p < ctx->N_ant; p++) {
                    i_samp += ctx->i_buf[(p*ctx->phy_struct->N_samps_per_frame) + ctx->samp_buf_idx];
                    q_samp += ctx->q_buf[(p*ctx->phy_struct->N_samps_per_frame) + ctx->samp_buf_idx];
                }
                ctx->outdata[ctx->outdata_cnt].i_data = i_samp;
                ctx->outdata[ctx->outdata_cnt].q_data = q_samp;
                ctx->samp_buf_idx++;
                ctx->outdata_cnt++;
            }

            if(ctx->outf != NULL) {
                // fprintf(stderr, "ctx->sfn = %d, file offset : 0x%08x, %d\n", ctx->sfn, ctx->output_offset, ctx->output_offset);
                fwrite(ctx->outdata, 1, sizeof(struct complex_data_f) * ctx->outdata_cnt, ctx->outf);
                ctx->output_offset += sizeof(struct complex_data_f) * ctx->outdata_cnt;
            }
        }

        SaveBitmap(ctx);

    }

    fprintf(stderr, "LTE_Fdd_Analysis_Encode : DONE\n");

    return 0;
}

int init_log_feature(void)
{
    log_msg_set(LOG_MSG_CODE_INIT);

    /* PSS */
    // log_msg_set(LOG_MSG_CODE_PSS_INIT);
    // log_msg_set(LOG_MSG_CODE_PSS_DATA);

    /* SSS */
    // log_msg_set(LOG_MSG_CODE_SSS_INIT);
    // log_msg_set(LOG_MSG_CODE_SSS_DATA);
    log_msg_set(LOG_MSG_CODE_SSS_FIND);

    // log_msg_set(LOG_MSG_CODE_DECODE_INPUT);
    log_msg_set(LOG_MSG_CODE_LOAD_BUFF);
    log_msg_set(LOG_MSG_CODE_SEARCH);
    log_msg_set(LOG_MSG_CODE_SEARCH_COARSE_TIMING);
    log_msg_set(LOG_MSG_CODE_PSS_AND_FINE_TIMING);
    log_msg_set(LOG_MSG_CODE_SSS);
    log_msg_set(LOG_MSG_CODE_BCCH);

    log_msg_set(LOG_MSG_CODE_PRS_C);
    log_msg_set(LOG_MSG_CODE_PDSCH);
    log_msg_set(LOG_MSG_CODE_PDCCH);


}

int main(int argc, char *argv[])
{

    struct LTE_Fdd_Analysis_s *ctx;
    int result;

    ctx = LTE_Fdd_Analysis_GetContext();
    init_log_feature();

    result = LTE_Fdd_Analysis_Process_Option(argc, argv, ctx);
    if(result < 0) {
        LTE_Fdd_Analysis_Print_Usage(argv[0]);
        exit(1);
    }

    if(ctx->analysis_mode == LTE_ANALYSIS_MODE_ENCODE) {
        if(ctx->file_name != NULL) {
            LTE_Fdd_Analysis_Encode_Init(ctx);
            LTE_Fdd_Analysis_Encode(ctx);
        }
        if(ctx->outf != NULL) {
            fclose(ctx->outf);
            ctx->outf = NULL;
        }
    }

    if(ctx->analysis_mode == LTE_ANALYSIS_MODE_DECODE) {
        if(ctx->file_name != NULL) {

            ctx->fInput = fopen(ctx->file_name, "rb");
            if( ctx->fInput == NULL) {
                fprintf(stderr, "Open failed : %s\n", ctx->file_name);
                return -1;
            }

            if(LTE_Fdd_Analysis_Decode_Init(ctx) < 0) {
                fprintf(stderr, "Decode init failed\n");
                return -1;
            }

            LTE_Fdd_Analysis_Decode(ctx);

            fclose(ctx->fInput);

        }
    }

    return 1;

}


/* End of LTE_FDD_ANALYSIS_Exported_Functions */
/**
  * @}
  */

/* End of LTE_FDD_ANALYSIS */
/**
  * @}
  */


