/**
  ******************************************************************************
  * @file    LTE_Fdd_Analysis.h
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
#ifndef   _LTE_FDD_ANALYSIS_H_
#define   _LTE_FDD_ANALYSIS_H_

/* Includes */
#include "liblte_phy.h"
#include "liblte_rrc.h"
#include "liblte_mac.h"
#include "liblte_mcc_mnc_list.h"
#include <gnuradio/sync_block.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LTE_FDD_ANALYSIS LTE Fdd Analysis
  * @{
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Private_Macros LTE Fdd Analysis Private Macros
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Private_Macros */
/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Exported_Macros LTE Fdd Analysis Exported Macros
  * @{
  */

#define LTE_FDD_DL_FS_SAMP_BUF_SIZE       (LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ*10)
#define LTE_FDD_DL_FS_SAMP_BUF_NUM_FRAMES (10)

#define LTE_FDD_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX 10

// Configurable Parameters
#define FS_PARAM "fs"

#define LTE_ANALYSIS_MODE_ENCODE        1
#define LTE_ANALYSIS_MODE_DECODE        2

#define LTE_INPUT_ITME_COUNT            8191


/* End of LTE_FDD_ANALYSIS_Exported_Macros */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Private_Types LTE Fdd Analysis Private Types
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Private_Types */
/**
  * @}
  */

/* Exported types -------------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Exported_Types LTE Fdd Analysis Exported Types
  * @{
  */

struct LTE_Fdd_Analysis_s;


typedef enum {
    LTE_FDD_DL_FS_IN_SIZE_INT8 = 0,
    LTE_FDD_DL_FS_IN_SIZE_GR_COMPLEX,
} LTE_FDD_DL_FS_IN_SIZE_ENUM;

typedef enum {
    LTE_FDD_DL_FS_SAMP_BUF_STATE_COARSE_TIMING_SEARCH = 0,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_PSS_AND_FINE_TIMING_SEARCH,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_SSS_SEARCH,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_BCH_DECODE,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SIB1,
    LTE_FDD_DL_FS_SAMP_BUF_STATE_PDSCH_DECODE_SI_GENERIC,
} LTE_FDD_DL_FS_SAMP_BUF_STATE_ENUM;

typedef enum{
    LTE_FDD_DL_FG_OUT_SIZE_INT8 = 0,
    LTE_FDD_DL_FG_OUT_SIZE_GR_COMPLEX,
} LTE_FDD_DL_FG_OUT_SIZE_ENUM;



struct complex_data_f {
    float i_data;
    float q_data;
};

struct complex_data_i {
    int8_t i_data;
    int8_t q_data;
};

struct symbol_pos_s {
    int start;
    int end;
};

struct LTE_Fdd_Analysis_s {

    int32_t Mode;

    char *input_file_name;
    char *output_file_name;

    struct complex_data_f   *indata;
    int32_t indata_cnt;
    int32_t indata_pos;

    FILE *outf;    
    struct complex_data_f *outdata;
    int32_t outdata_cnt;
    uint32_t output_offset;
    
    // LTE library
    LIBLTE_PHY_STRUCT                *phy_struct;
    LIBLTE_PHY_COARSE_TIMING_STRUCT   timing_struct;
    LIBLTE_BIT_MSG_STRUCT             rrc_msg;
    LIBLTE_RRC_MIB_STRUCT             mib;
    LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT  bcch_dlsch_msg;
    LIBLTE_RRC_PCCH_MSG_STRUCT        pcch_msg;
    LIBLTE_PHY_FS_ENUM                fs;

    // Sample buffer
    float  *i_buf;
    float  *q_buf;
    uint32  samp_buf_w_idx;
    uint32  samp_buf_r_idx;
    bool    last_samp_was_i;
    
    // Variables
    LTE_FDD_DL_FS_SAMP_BUF_STATE_ENUM state;
    float                             phich_res;
    uint32_t                          sfn;
    uint32_t                          N_sfr;
    uint32_t                          N_id_cell;
    uint32_t                          N_id_1;
    uint32_t                          N_id_2;
    uint32_t                          corr_peak_idx;
    uint32_t                          decoded_chans[LTE_FDD_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX];
    uint32_t                          N_decoded_chans;
    uint8_t                           N_ant;
    uint8_t                           prev_si_value_tag;
    int8_t                            prev_si_value_tag_valid;
    int8_t                            mib_printed;
    int8_t                            sib1_printed;
    int8_t                            sib2_printed;
    int8_t                            sib3_printed;
    int8_t                            sib3_expected;
    int8_t                            sib4_printed;
    int8_t                            sib4_expected;
    int8_t                            sib5_printed;
    int8_t                            sib5_expected;
    int8_t                            sib6_printed;
    int8_t                            sib6_expected;
    int8_t                            sib7_printed;
    int8_t                            sib7_expected;
    int8_t                            sib8_printed;
    int8_t                            sib8_expected;
    int8_t                            sib13_printed;
    int8_t                            sib13_expected;

    int32_t num_samps_needed;
    int32_t done_flag;

    uint32_t pss_symb;

    float pss_thresh;
    uint8_t sfn_offset;

    LIBLTE_PHY_SUBFRAME_STRUCT  subframe;
    LIBLTE_PHY_PCFICH_STRUCT    pcfich;
    LIBLTE_PHY_PHICH_STRUCT     phich;
    LIBLTE_PHY_PDCCH_STRUCT     pdcch;

    int32_t input_offset;

    // Input parameters
    LTE_FDD_DL_FG_OUT_SIZE_ENUM out_size;

    // Sample buffer
#if 0    
    float  *i_buf;
    float  *q_buf;
#endif

    uint32  samp_buf_idx;
    bool    samples_ready;

#if 0    
    bool    last_samp_was_i;
#endif

    // LTE parameters
#if 0    
    LIBLTE_BIT_MSG_STRUCT                    rrc_msg;
    LIBLTE_RRC_MIB_STRUCT                    mib;
    LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT         bcch_dlsch_msg;
#endif 
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT  sib1;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT  sib2;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT  sib3;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT  sib4;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT  sib8;

#if 0    
    LIBLTE_PHY_STRUCT                       *phy_struct;
    LIBLTE_PHY_PCFICH_STRUCT                 pcfich;
    LIBLTE_PHY_PHICH_STRUCT                  phich;
    LIBLTE_PHY_PDCCH_STRUCT                  pdcch;
    LIBLTE_PHY_SUBFRAME_STRUCT               subframe;
    LIBLTE_PHY_FS_ENUM                       fs;
    float                                    phich_res;
#endif

    float                                    bandwidth;
#if 0
    uint32                                   sfn;
#endif
    uint32_t                                 N_frames;
#if 0
    uint32_t                                   N_id_cell;
    uint32_t                                   N_id_1;
    uint32_t                                   N_id_2;
#endif    
    uint32_t                                N_rb_dl;
    uint32_t                                si_periodicity_T;
    uint32_t                                si_win_len;
    uint32_t                                sib_tx_mode;
    uint32_t                                percent_load;
#if 0    
    uint8_t                                 N_ant;
#endif
    uint8_t                                 sib3_present;
    uint8_t                                 sib4_present;
    uint8_t                                 sib8_present;

    uint32                                  last_prb;
    int32                                   noutput_items;

    int8       *int8_out;

    int32_t process_samples;
    int32_t copy_input;

    int32_t modulation_type;
    

};


struct LTE_Fdd_Analysis_State_Handler_s {
    int32_t State;
    int32_t (*Handler)(LTE_Fdd_Analysis_s *ctx);
};


/* End of LTE_FDD_ANALYSIS_Exported_Types */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Private_Variables LTE Fdd Analysis Private Variables
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Private_Variables */
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Exported_Variables LTE Fdd Analysis Exported Variables
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Exported_Variables */
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Private_Functions LTE Fdd Analysis Private Functions
  * @{
  */
/* End of LTE_FDD_ANALYSIS_Private_Functions */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup LTE_FDD_ANALYSIS_Exported_Functions LTE Fdd Analysis Exported Functions
  * @{
  */

struct LTE_Fdd_Analysis_s *GetLTEFddAnalysisContext(void);
int LTE_Fdd_Analysis_Process_Option(int argc, char *argv[], struct option *long_options, struct LTE_Fdd_Analysis_s *ctx);
int32_t LTE_Fdd_Analysis(struct LTE_Fdd_Analysis_s *ctx);



/* End of LTE_FDD_ANALYSIS_Exported_Functions */
/**
  * @}
  */

/* End of LTE_FDD_ANALYSIS */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _LTE_FDD_ANALYSIS_H_ */

