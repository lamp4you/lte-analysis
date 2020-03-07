/**
  ******************************************************************************
  * @file    LTE_analysis.h
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
#ifndef   _LTE_ANALYSIS_H_
#define   _LTE_ANALYSIS_H_

/* Includes */

#include "liblte_phy.h"
#include "liblte_phy_n.h"
#include "liblte_rrc.h"
#include "liblte_mac.h"
#include "liblte_mcc_mnc_list.h"
#include <gnuradio/sync_block.h>


#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LTE_ANALYSIS LTE Analysis
  * @{
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Private_Macros LTE Analysis Private Macros
  * @{
  */
/* End of LTE_ANALYSIS_Private_Macros */
/**
  * @}
  */

/* Exported macros ------------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Exported_Macros LTE Analysis Exported Macros
  * @{
  */
#define LTE_DL_FS_SAMP_BUF_SIZE       (LIBLTE_PHY_N_SAMPS_PER_FRAME_30_72MHZ*10)
#define LTE_DL_FS_SAMP_BUF_NUM_FRAMES (10)

#define LTE_DL_FS_SAMP_BUF_N_DECODED_CHANS_MAX 10


#define PDF_STEP            (0.01)      /* 0.01 dB */
#define PDF_RANGE           (20.0)

#define REFERENCE_RMS_DB        (-20.0)     /* - 20.0 dBFS */
#define REFERENCE_RMS_LEVEL     pow(10.0, REFERENCE_RMS_DB / 20.0)

#define _100RB_RMS_LEVEL        sqrt(1200)
#define _100RB_GAIN             (REFERENCE_RMS_LEVEL/_100RB_RMS_LEVEL)

#define _50RB_RMS_LEVEL         sqrt(600)
#define _50RB_GAIN              (REFERENCE_RMS_LEVEL/_50RB_RMS_LEVEL)

#define _25RB_RMS_LEVEL         sqrt(300)
#define _25RB_GAIN              (REFERENCE_RMS_LEVEL/_50RB_RMS_LEVEL)

#define MAX_ADC_VALUE           (32768)
#define USE_ADC16               16
#define USE_ADC14               14
#define USE_ADC12               12

#define MIN_DB_VALUE            -1000.0

#define DAS_CODE_16BIT          1
#define DAS_CODE_14BIT          2
#define DAS_CODE_12BIT          3
#define DAS_CODE_10BIT          4

#define DAS_CODE_RAW            0
#define DAS_CODE_IQ_20_4        1
#define DAS_CODE_IQ_20_3        2
#define DAS_CODE_IQ_19_4        3
#define DAS_CODE_IQ_19_3        4
#define DAS_CODE_IQ_20_4_X      5
#define DAS_CODE_IQ_19_4_X      6
#define DAS_CODE_IQ_20_4_Y      99

#define NORMAL_PRINT_COUNT      20

#define COMPANDOR_SIGN_SHIFT    30
#define COMPANDOR_SIGN_MASK     0xC0000000

#define COMPANDOR_SIGN(iq_data)         ((iq_data & COMPANDOR_SIGN_MASK) >> COMPANDOR_SIGN_SHIFT)

#define COMPANDOR_1_BIT_MASK            0x20000000
#define COMPANDOR_2_BIT_MASK            0x30000000
#define COMPANDOR_4_BIT_MASK            0x3C000000

#define COMPANDOR_4_BIT_SHIFT_MAX       (26)

#define COMPANDOR_20_4_MASK             0x00003FFF
#define COMPANDOR_20_4_SHIFT_MAX        15
#define COMPANDOR_20_4_SEARCH_1_MAX     15

#define COMPANDOR_20_3_MASK             0x00007FFF
#define COMPANDOR_20_3_SHIFT_MAX        14
#define COMPANDOR_20_3_SEARCH_1_MAX     7

#define COMPANDOR_I_SIGN                0x0002
#define COMPANDOR_Q_SIGN                0x0001

#define LTE_TEST_16QAM_BITS_COUNT       640
#define LTE_TEST_64QAM_BITS_COUNT       1024
#define LTE_TEST_PERCENT_LOAD           50


/* End of LTE_ANALYSIS_Exported_Macros */
/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Private_Types LTE Analysis Private Types
  * @{
  */
/* End of LTE_ANALYSIS_Private_Types */
/**
  * @}
  */

/* Exported types -------------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Exported_Types LTE Analysis Exported Types
  * @{
  */


struct iq_sample_s {
    int16_t i;
    int16_t q;
};

typedef struct iq_sample_s iq_sample_t;

struct iq_code_s {
    int16_t n_bits;
    int16_t sign;
    int16_t first_1_pos;
    int32_t iq_code;   
};

typedef struct iq_code_s iq_code_t;


struct huff_code_s {
    
    int16_t h_code;
    int16_t h_bits;

    int16_t i_sign;
    int16_t q_sign;    
    uint16_t i_data;
    uint16_t q_data;
    
};

typedef struct huff_code_s huff_code_t;

struct Compandor_s {
    union {
        iq_sample_t d;
        huff_code_t h;
        iq_code_t c;   
    } u;
};

typedef struct Compandor_s Compandor_t;

struct Fixed_Sample_s {
    int16_t i;
    int16_t q;
};

typedef struct Fixed_Sample_s Fixed_Sample_t;

struct LTE_Analysis_s {

    LIBLTE_PHY_FS_ENUM fs;

    char *input_file_name;
    char *output_file_name;

    fftwf_complex *indata;
    int32_t indata_cnt;
    int32_t indata_pos;

    fftwf_complex *outdata;
    int32_t outdata_cnt;
    int32_t outdata_pos;

    fftwf_complex *Sample_in;    
    fftwf_complex *Normalized_in;    
    fftwf_complex *Sample_in_fft;    
    int32_t N_samples;
    float Sample_in_pwr;
    float Normalized_in_pwr;    
 
    fftwf_complex *Symbol_in;    
    int32_t N_symbols;
  
    int32_t test_code;
    int Modulation_Type;
    int bandwidth;
    int N_rb;
    int N_sc_per_rb;
    int used_subcarriers;
    int FFT_size;
    int FFT_pad_size;
    float Gain;
    float InputRefRmsLevel;

    float RefRmsLevel;    

    int32_t N_samples_total;
    int32_t N_measure;
    int32_t *Power_Histogram;
    float *Pdf_Of_Samples;
    float *Ccdf_Of_Samples;

    int32_t Adc_bits;
    Fixed_Sample_t *Adc_data;
    fftwf_complex *Adc_error;    

    int32_t Dac_bits;
    int32_t Code_type;
    
    Compandor_t *Compress_data;
    fftwf_complex *Compress_Error;    

    Fixed_Sample_t *Decompress_data;

    fftwf_complex *Normalized_out;
    fftwf_complex *Sample_out;
    fftwf_complex *Sample_error;

    fftwf_complex *Symbol_out;

    fftwf_complex *Sample_out_fft;
    fftwf_complex *Symbol_error;  


    

};


struct code_info_s {
    
    int32_t cnt;
    int32_t code;
    uint16_t real_min;
    uint16_t real_max;
    uint16_t imag_min;
    uint16_t imag_max;  

    uint32_t h_code;
    int32_t h_bits;
    int32_t i_bits;
    int32_t i_shift;
    int32_t q_bits;
    int32_t q_shift;    
    
};

typedef struct code_info_s code_info_t;


struct LTE_Analysis_PRACH_s {

    // 
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT  sib2;

    // 
    LIBLTE_PHY_STRUCT       *phy_struct;
    LIBLTE_PHY_FS_ENUM      fs;

    int                     frame_struct_type;
    int                     cp_mode;
    
    float                   bandwidth;

    uint16                  N_id_cell;
    uint8                   N_ant;
    uint32                  N_rb_dl;
    uint32                  N_sc_rb_dl;
    float                   phich_res;


    /* UL */
    uint32                  N_rb_ul;

    /* PRACH */
    int                     prach_root_seq_idx;
    int                     prach_preamble_format;
    int                     prach_zczc;
    bool                    prach_hs_flag;

    int                     prach_preamble_index;
    int                     prach_freq_offset;

    float                   *prach_samps_re;
    float                   *prach_samps_im;

    uint32                  N_det_pre;
    uint32                  det_pre;
    uint32                  det_ta;
    
    uint8              group_assignment_pusch;
    bool               group_hopping_enabled;
    bool               sequence_hopping_enabled;
    
    uint8              cyclic_shift;
    uint8              cyclic_shift_dci;    
    uint8              N_cs_an;
    uint8              delta_pucch_shift;


};


struct LTE_Analysis_DL_s {

    LIBLTE_PHY_STRUCT       *phy_struct;
    LIBLTE_PHY_FS_ENUM      fs;

    int                     frame_struct_type;

    uint16                  N_id_cell;
    uint32                  N_id_1;
    uint32                  N_id_2;
    
    uint8                   N_ant;
    uint8                   N_layer;
    uint32                  N_rb_dl;
    uint32                  N_sc_rb_dl;
    float                   phich_res;
    int                     cp_mode;

    int                     modulation_type;
    int                     percent_load;

    uint32                              sfn;
    uint32                              si_periodicity_T;
    uint32                              si_win_len;
    uint32                              sib_tx_mode;
    uint32                              last_prb;

    LIBLTE_PHY_SUBFRAME_STRUCT          subframe;
    LIBLTE_PHY_PCFICH_STRUCT            pcfich;
    LIBLTE_PHY_PHICH_STRUCT             phich;
    LIBLTE_PHY_PDCCH_STRUCT             pdcch;
    
    LIBLTE_RRC_MIB_STRUCT               mib;
    LIBLTE_BIT_MSG_STRUCT               rrc_msg;
    LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT    bcch_dlsch_msg;

    uint8                                    sib3_present;
    uint8                                    sib4_present;
    uint8                                    sib8_present;

    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT  sib1;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT  sib2;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT  sib3;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT  sib4;
    LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT  sib8;


#ifdef CHECK
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
        LIBLTE_PHY_COARSE_TIMING_STRUCT   timing_struct;
        LIBLTE_RRC_PCCH_MSG_STRUCT        pcch_msg;
    
        // Sample buffer
        float  *i_buf;
        float  *q_buf;
        uint32  samp_buf_w_idx;
        uint32  samp_buf_r_idx;
        bool    last_samp_was_i;
        
        // Variables
        LTE_FDD_DL_FS_SAMP_BUF_STATE_ENUM state;
        float                             phich_res;
        
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
#if 0    
        uint8_t                                 N_ant;
#endif
    
        int32                                   noutput_items;
    
        int8       *int8_out;
    
        int32_t process_samples;
        int32_t copy_input;
    
        int32_t modulation_type;
        

#endif


};



/* End of LTE_ANALYSIS_Exported_Types */
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Private_Variables LTE Analysis Private Variables
  * @{
  */
/* End of LTE_ANALYSIS_Private_Variables */
/**
  * @}
  */

/* Exported variables ---------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Exported_Variables LTE Analysis Exported Variables
  * @{
  */
/* End of LTE_ANALYSIS_Exported_Variables */
/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Private_Functions LTE Analysis Private Functions
  * @{
  */
/* End of LTE_ANALYSIS_Private_Functions */
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup LTE_ANALYSIS_Exported_Functions LTE Analysis Exported Functions
  * @{
  */

#ifdef __cplusplus 
extern "C" {
#endif  

struct LTE_Analysis_s *LTE_Analysis_GetContext(void);

#ifdef __cplusplus 
}
#endif  


/* End of LTE_ANALYSIS_Exported_Functions */
/**
  * @}
  */

/* End of LTE_ANALYSIS */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _LTE_ANALYSIS_H_ */

