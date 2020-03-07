
/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "liblte_phy.h"
#include "liblte_mac.h"
#include "liblte_mcc_mnc_list.h"

#include <math.h>
#include <stdint.h>
#include <unistd.h>


int32_t dci_format_0_test = 1;

char *null_str = (char *)"(null)";

struct lte_enum_info SIB_TYPE_Info_Str[] =
{
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_2"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_3"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_4"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_5"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_6"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_7"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_8"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_9,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_9"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_10,          (char *)"RRC_SYS_INFO_BLOCK_TYPE_10"        },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_11,          (char *)"RRC_SYS_INFO_BLOCK_TYPE_11"        },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_12,          (char *)"RRC_SYS_INFO_BLOCK_TYPE_12"        },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13,          (char *)"RRC_SYS_INFO_BLOCK_TYPE_13"        },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_SPARE1,      (char *)"RRC_SYS_INFO_BLOCK_TYPE_SPARE1"    },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_SPARE2,      (char *)"RRC_SYS_INFO_BLOCK_TYPE_SPARE2"    },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_SPARE3,      (char *)"RRC_SYS_INFO_BLOCK_TYPE_SPARE3"    },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_SPARE4,      (char *)"RRC_SYS_INFO_BLOCK_TYPE_SPARE4"    },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1,           (char *)"RRC_SYS_INFO_BLOCK_TYPE_1"         },
    {  LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_N_ITEMS,     (char *)"RRC_SYS_INFO_BLOCK_TYPE_N_ITEMS"   },

    { -1,     NULL   }
};

struct lte_enum_info RRC_SIB_TYPE_Info_str[] =
{
    { LIBLTE_RRC_SIB_TYPE_3,         (char *)"RRC_SIB_TYPE_3"       },
    { LIBLTE_RRC_SIB_TYPE_4,         (char *)"RRC_SIB_TYPE_4"       },
    { LIBLTE_RRC_SIB_TYPE_5,         (char *)"RRC_SIB_TYPE_5"       },

    { LIBLTE_RRC_SIB_TYPE_6,         (char *)"RRC_SIB_TYPE_6"       },
    { LIBLTE_RRC_SIB_TYPE_7,         (char *)"RRC_SIB_TYPE_7"       },
    { LIBLTE_RRC_SIB_TYPE_8,         (char *)"RRC_SIB_TYPE_8"       },
    { LIBLTE_RRC_SIB_TYPE_9,         (char *)"RRC_SIB_TYPE_9"       },
    { LIBLTE_RRC_SIB_TYPE_10,        (char *)"RRC_SIB_TYPE_10"      },

    { LIBLTE_RRC_SIB_TYPE_11,        (char *)"RRC_SIB_TYPE_11"      },
    { LIBLTE_RRC_SIB_TYPE_12_v920,   (char *)"RRC_SIB_TYPE_12_v920" },
    { LIBLTE_RRC_SIB_TYPE_13_v920,   (char *)"RRC_SIB_TYPE_13_v920" },
    { LIBLTE_RRC_SIB_TYPE_SPARE_5,   (char *)"RRC_SIB_TYPE_SPARE_5" },
    { LIBLTE_RRC_SIB_TYPE_SPARE_4,   (char *)"RRC_SIB_TYPE_SPARE_4" },
    { LIBLTE_RRC_SIB_TYPE_SPARE_3,   (char *)"RRC_SIB_TYPE_SPARE_3" },
    { LIBLTE_RRC_SIB_TYPE_SPARE_2,   (char *)"RRC_SIB_TYPE_SPARE_2" },
    { LIBLTE_RRC_SIB_TYPE_SPARE_1,   (char *)"RRC_SIB_TYPE_SPARE_1" },

    { -1,     NULL   }
};

struct lte_enum_info PHICH_DURATION_str[] =
{

    {   LIBLTE_RRC_PHICH_DURATION_NORMAL,          (char *)"RRC_PHICH_DURATION_NORMAL",       },
    {   LIBLTE_RRC_PHICH_DURATION_EXTENDED,        (char *)"RRC_PHICH_DURATION_EXTENDED",     },
    {   LIBLTE_RRC_PHICH_DURATION_N_ITEMS,         (char *)"RRC_PHICH_DURATION_N_ITEMS",      },

    { -1,     NULL   }
};


LIBLTE_ERROR_ENUM liblte_phy_pss_init(LIBLTE_PHY_STRUCT *phy_struct)
{
    LIBLTE_ERROR_ENUM result = LIBLTE_ERROR_INVALID_INPUTS;

    int32_t i;
    int32_t j;
    int32_t k;
    int32_t nid1;
    int32_t nid2;

    float   total_power;
    float   avg_power;
    float   scale;
    float   pss_re[63];
    float   pss_im[63];

    int32_t samples_pss_symbol;

    samples_pss_symbol = phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb;

    if(phy_struct != NULL) {

        static float pss_symb_re[LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP];
        static float pss_symb_im[LIBLTE_PHY_N_RB_DL_20MHZ*LIBLTE_PHY_N_SC_RB_DL_NORMAL_CP];

#ifdef    LIBLTE_PHY_ANALYSIS
        if(LOG_MSG_ENABLE(LOG_MSG_CODE_INIT)) {
            fprintf(stderr, "\nGenerates an LTE Primary Synchronization Signal (PSS)\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {

            for(i = 0; i < (phy_struct->N_rb_dl * phy_struct->N_sc_rb_dl); i++) {
                pss_symb_re[i] = 0.0;
                pss_symb_im[i] = 0.0;
            }

            generate_pss(nid2, pss_re, pss_im);

            for(i = 0; i < PSS_SEQ_LENGTH; i++) {
                k              = i - PSS_SEQ_LENGTH/2 + (phy_struct->N_rb_dl * phy_struct->N_sc_rb_dl)/2;
                pss_symb_re[k] = pss_re[i];
                pss_symb_im[k] = pss_im[i];
            }

            symbols_to_samples_dl(
                phy_struct,
                pss_symb_re, pss_symb_re, PSS_SYMBOL_NUM,
                phy_struct->pss_sample_re[nid2], phy_struct->pss_sample_im[nid2], NULL
            );

            total_power = 0.0;
            for(i = 0; i < samples_pss_symbol; i++) {
                total_power += phy_struct->pss_sample_re[nid2][i] * phy_struct->pss_sample_re[nid2][i]
                             + phy_struct->pss_sample_im[nid2][i] * phy_struct->pss_sample_im[nid2][i];
            }
            avg_power = total_power / samples_pss_symbol;
            scale = 1.0 / sqrt(avg_power);

#ifdef    LIBLTE_PHY_ANALYSIS
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_PSS_INIT)) {
                fprintf(stderr, "\nNID2 = %d\n", nid2);
                fprintf(stderr, "Total Power = %.2f dB (%.2f)\n", 10.0 * log10(total_power), total_power);
                fprintf(stderr, "Avg Power = %.2f dB (%.2f)\n", 10.0 * log10(avg_power), avg_power);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            total_power = 0.0;
            for(i = 0; i < samples_pss_symbol; i++) {
                phy_struct->pss_sample_re[nid2][i] = phy_struct->pss_sample_re[nid2][i] * scale;
                phy_struct->pss_sample_im[nid2][i] = phy_struct->pss_sample_im[nid2][i] * scale;
                total_power += phy_struct->pss_sample_re[nid2][i] * phy_struct->pss_sample_re[nid2][i]
                             + phy_struct->pss_sample_im[nid2][i] * phy_struct->pss_sample_im[nid2][i];
            }
            avg_power = total_power / samples_pss_symbol;

#ifdef    LIBLTE_PHY_ANALYSIS
            if(LOG_MSG_ENABLE(LOG_MSG_CODE_PSS_DATA)) {
                fprintf(stderr, "Total Power = %.2f dB (%.2f)\n", 10.0 * log10(total_power), total_power);
                fprintf(stderr, "Avg Power = %.2f dB (%.2f)\n", 10.0 * log10(avg_power), avg_power);
                print_complex_number(phy_struct->pss_sample_re[nid2], phy_struct->pss_sample_im[nid2], samples_pss_symbol, 0);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

        }

        result = LIBLTE_SUCCESS;

    }

    return result;

}



LIBLTE_ERROR_ENUM liblte_phy_sss_init(LIBLTE_PHY_STRUCT *phy_struct)
{
    LIBLTE_ERROR_ENUM result = LIBLTE_ERROR_INVALID_INPUTS;

    int32_t i;
    int32_t j;
    int32_t k;
    int32_t nid1;
    int32_t nid2;

    float   total_power;
    float   avg_power;
    float   scale;

    float *abs_corr_0x0_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];
    float *abs_corr_0x5_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];
    float *abs_corr_5x0_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];
    float *abs_corr_5x5_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];

    float *abs_conv_0x0_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];
    float *abs_conv_0x5_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];
    float *abs_conv_5x0_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];
    float *abs_conv_5x5_in_time[LTE_NUM_OF_NID][LTE_NUM_OF_NID];

    float corr_max;
    float corr_max_tmp;
    float conv_max;
    float conv_max_tmp;

    float max_value;
    float corr_re;
    float corr_im;
    float conv_re;
    float conv_im;
    float *x_re;
    float *x_im;
    float *y_re;
    float *y_im;

    int32_t nid;
    int32_t nid_1;
    int32_t t;

    int32_t num_of_sub_carriers;
    int32_t samples_per_sss_symbol;
    int32_t max_nid;

    static float *sss_re_0[LTE_NUM_OF_NID];

    num_of_sub_carriers = phy_struct->N_rb_dl*phy_struct->N_sc_rb_dl;
    samples_per_sss_symbol = phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb;
    max_nid = LIBLTE_NUM_OF_CELL_GROUP_ID * LTE_NUM_OF_CELL_ID;

    for(nid = 0; nid < max_nid; nid++) {
        for(nid_1 = 0; nid_1 < max_nid; nid_1++) {

            abs_corr_0x0_in_time[nid][nid_1] = (float *)malloc(2*sizeof(float)*(LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ+LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ));
            abs_corr_0x5_in_time[nid][nid_1] = (float *)malloc(2*sizeof(float)*(LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ+LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ));
            abs_corr_5x5_in_time[nid][nid_1] = (float *)malloc(2*sizeof(float)*(LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ+LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ));

            abs_conv_0x0_in_time[nid][nid_1] = (float *)malloc(2*sizeof(float)*(LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ+LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ));
            abs_conv_0x5_in_time[nid][nid_1] = (float *)malloc(2*sizeof(float)*(LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ+LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ));
            abs_conv_5x5_in_time[nid][nid_1] = (float *)malloc(2*sizeof(float)*(LIBLTE_PHY_N_SAMPS_PER_SYMB_30_72MHZ+LIBLTE_PHY_N_SAMPS_CP_L_ELSE_30_72MHZ));
        }
    }

    for(nid = 0; nid < max_nid; nid++) {

        for(j = 0; j < samples_per_sss_symbol; j++) {
            phy_struct->sss_mod_re_0[nid][j] = 0.0;
            phy_struct->sss_mod_im_0[nid][j] = 0.0;
            phy_struct->sss_mod_re_5[nid][j] = 0.0;
            phy_struct->sss_mod_im_5[nid][j] = 0.0;
        }

        generate_sss(
            phy_struct,
            nid / LTE_NUM_OF_CELL_ID,
            nid % LTE_NUM_OF_CELL_ID,
            phy_struct->sss_re_0,
            phy_struct->sss_im_0,
            phy_struct->sss_re_5,
            phy_struct->sss_im_5
        );

        for(j = 0; j < LIBLTE_NUM_OF_SSS_SEQUENCE; j++) {
            k = num_of_sub_carriers/2 - LIBLTE_NUM_OF_SSS_SEQUENCE/2 + j;
            phy_struct->sss_mod_re_0[nid][k] = phy_struct->sss_re_0[j];
            phy_struct->sss_mod_im_0[nid][k] = phy_struct->sss_im_0[j];
            phy_struct->sss_mod_re_5[nid][k] = phy_struct->sss_re_5[j];
            phy_struct->sss_mod_im_5[nid][k] = phy_struct->sss_im_5[j];
        }

        symbols_to_samples_dl(
            phy_struct,
            phy_struct->sss_mod_re_0[nid], phy_struct->sss_mod_im_0[nid], PSS_SYMBOL_NUM,
            phy_struct->sss_sample_re_0[nid], phy_struct->sss_sample_im_0[nid], NULL
        );

        symbols_to_samples_dl(
            phy_struct,
            phy_struct->sss_mod_re_5[nid], phy_struct->sss_mod_im_5[nid], PSS_SYMBOL_NUM,
            phy_struct->sss_sample_re_5[nid], phy_struct->sss_sample_im_5[nid], NULL
        );

    }


    for(nid = 0; nid < max_nid; nid++) {
        for(nid_1 = 0; nid_1 < max_nid; nid_1++) {

            if(abs_corr_0x0_in_time[nid][nid_1] != NULL) { free(abs_corr_0x0_in_time[nid][nid_1]); }
            if(abs_corr_0x5_in_time[nid][nid_1] != NULL) { free(abs_corr_0x5_in_time[nid][nid_1]); }
            if(abs_corr_5x5_in_time[nid][nid_1] != NULL) { free(abs_corr_5x5_in_time[nid][nid_1]); }

            if(abs_conv_0x0_in_time[nid][nid_1] != NULL) { free(abs_conv_0x0_in_time[nid][nid_1]); }
            if(abs_conv_0x5_in_time[nid][nid_1] != NULL) { free(abs_conv_0x5_in_time[nid][nid_1]); }
            if(abs_conv_5x5_in_time[nid][nid_1] != NULL) { free(abs_conv_5x5_in_time[nid][nid_1]); }
        }
    }

    if(phy_struct != NULL) {

        result = LIBLTE_SUCCESS;

    }

    return result;

}

LIBLTE_ERROR_ENUM liblte_phy_dl_searcher_init(LIBLTE_PHY_STRUCT *phy_struct)
{
    LIBLTE_ERROR_ENUM result = LIBLTE_ERROR_INVALID_INPUTS;

    if(phy_struct != NULL) {

        liblte_phy_pss_init(phy_struct);
        // liblte_phy_sss_init(phy_struct);

        result = LIBLTE_SUCCESS;

    }

    return result;
}

int do_correlation
(
    float *x_re,
    float *x_im,
    int32_t num_of_sample_x,
    float *y_re,
    float *y_im,
    int32_t num_of_sample_y,
    float *result
)
{
    int32_t idx;
    int32_t i;
    int32_t t;
    float corr_re;
    float corr_im;

    if(num_of_sample_x > num_of_sample_y) {

        float *y_re_tmp;
        float *y_im_tmp;
        float *result_tmp;

        y_re_tmp = (float *)malloc(num_of_sample_x * sizeof(float));
        y_im_tmp = (float *)malloc(num_of_sample_x * sizeof(float));
        result_tmp = (float *)malloc(2 * num_of_sample_x * sizeof(float));

        memcpy(y_re_tmp, y_re, num_of_sample_y * sizeof(float));
        memcpy(y_im_tmp, y_im, num_of_sample_y * sizeof(float));
        for(i = num_of_sample_y; i < num_of_sample_x; i++) {
            y_re_tmp[i] = 0.0;
            y_im_tmp[i] = 0.0;
        }

        do_correlation(x_re, x_im, num_of_sample_x, y_re_tmp, y_im_tmp, num_of_sample_x, result_tmp);
        memcpy(result, &result_tmp[num_of_sample_x - num_of_sample_y], (num_of_sample_x + num_of_sample_y) * sizeof(float));

        free(y_re_tmp);
        free(y_im_tmp);
        free(result_tmp);

    }
    else if(num_of_sample_x < num_of_sample_y) {

        float *x_re_tmp;
        float *x_im_tmp;
        float *result_tmp;

        x_re_tmp = (float *)malloc(num_of_sample_y * sizeof(float));
        x_im_tmp = (float *)malloc(num_of_sample_y * sizeof(float));
        result_tmp = (float *)malloc(2 * num_of_sample_y * sizeof(float));

        memcpy(x_re_tmp, x_re, num_of_sample_x * sizeof(float));
        memcpy(x_im_tmp, x_im, num_of_sample_x * sizeof(float));
        for(i = num_of_sample_x; i < num_of_sample_y; i++) {
            x_re_tmp[i] = 0.0;
            x_im_tmp[i] = 0.0;
        }

        do_correlation(x_re_tmp, x_im_tmp, num_of_sample_y, y_re, y_im, num_of_sample_y, result_tmp);

        memcpy(result, result_tmp, (num_of_sample_x + num_of_sample_y) * sizeof(float));

        free(x_re_tmp);
        free(x_im_tmp);
        free(result_tmp);

    }
    else {
//                            0 1 2 3 0 1 2 3
//    - - - - - - - - - - - - - - - - 0 1 2 3
//                            0 1 2 3
//                              0 1 2 3
//                                0 1 2 3
//                                  0 1 2 3
//                                    0 1 2 3
//                                      0 1 2
//                                        0 1
//                                          0
//                                ....
//                                                        0 1 2 3 4 5 6 7 8 9 - - -
//
        int y_idx;
        idx = 0;
        for(i = 0; i < num_of_sample_x; i++) {
            corr_re = 0.0;
            corr_im = 0.0;
            for(t = 0; t < i; t++) {
                y_idx = num_of_sample_y-(i-t);
                corr_re += x_re[t] * y_re[y_idx] - x_im[t] * y_im[y_idx];
                corr_im += x_im[t] * y_re[y_idx] + x_re[t] * y_im[y_idx];
            }
            result[idx] = sqrt(corr_re*corr_re + corr_im*corr_im);
            idx++;
        }

        for(i = 0; i < num_of_sample_x; i++) {
            corr_re = 0.0;
            corr_im = 0.0;
            for(t = i; t < num_of_sample_y; t++) {
                y_idx = t-i;
                corr_re += x_re[t] * y_re[y_idx] - x_im[t] * y_im[y_idx];
                corr_im += x_im[t] * y_re[y_idx] + x_re[t] * y_im[y_idx];
            }
            result[idx] = sqrt(corr_re*corr_re + corr_im*corr_im);
            idx++;
        }
    }

    return 1;

}

int do_convolution
(
    float *x_re,
    float *x_im,
    int32_t num_of_sample_x,
    float *y_re,
    float *y_im,
    int32_t num_of_sample_y,
    float *result
)
{
    int32_t i;
    int32_t t;
    float conv_re;
    float conv_im;
    int32_t idx;

    if(num_of_sample_x > num_of_sample_y) {

        float *y_re_tmp;
        float *y_im_tmp;
        float *result_tmp;

        y_re_tmp = (float *)malloc(num_of_sample_x * sizeof(float));
        y_im_tmp = (float *)malloc(num_of_sample_x * sizeof(float));
        result_tmp = (float *)malloc(2 * num_of_sample_x * sizeof(float));

        for(i = 0; i < (num_of_sample_x - num_of_sample_y); i++) {
            y_re_tmp[i] = 0.0;
            y_im_tmp[i] = 0.0;
        }
        memcpy(&y_re_tmp[num_of_sample_x - num_of_sample_y], y_re, num_of_sample_y * sizeof(float));
        memcpy(&y_im_tmp[num_of_sample_x - num_of_sample_y], y_im, num_of_sample_y * sizeof(float));

        do_convolution(x_re, x_im, num_of_sample_x, y_re_tmp, y_im_tmp, num_of_sample_x, result_tmp);
        memcpy(result, &result_tmp[num_of_sample_x - num_of_sample_y], (num_of_sample_x + num_of_sample_y) * sizeof(float));

        free(y_re_tmp);
        free(y_im_tmp);
        free(result_tmp);

    }
    else if(num_of_sample_x < num_of_sample_y) {

        float *x_re_tmp;
        float *x_im_tmp;
        float *result_tmp;

        x_re_tmp = (float *)malloc(num_of_sample_y * sizeof(float));
        x_im_tmp = (float *)malloc(num_of_sample_y * sizeof(float));
        result_tmp = (float *)malloc(2 * num_of_sample_y * sizeof(float));

        memcpy(x_re_tmp, x_re, num_of_sample_x * sizeof(float));
        memcpy(x_im_tmp, x_im, num_of_sample_x * sizeof(float));
        for(i = num_of_sample_x; i < num_of_sample_y; i++) {
            x_re_tmp[i] = 0.0;
            x_im_tmp[i] = 0.0;
        }

        do_convolution(x_re_tmp, x_im_tmp, num_of_sample_y, y_re, y_im, num_of_sample_y, result_tmp);
        memcpy(result, result_tmp, (num_of_sample_x + num_of_sample_y) * sizeof(float));

        free(x_re_tmp);
        free(x_im_tmp);
        free(result_tmp);

    }
    else {
//    - - - - - - - - - - - - - - - - 0 1 2 3
//                            3 2 1 0
//                              3 2 1 0
//                                3 2 1 0
//                                  3 2 1 0

//                                    3 2 1 0
//                                      3 2 1 -
//                                        3 2 - -
//                                          3 - - -
//                                                                  - - - - - - - - - - - - - - -
        int32_t y_idx;
        idx = 0;
        for(i = 0; i < num_of_sample_x; i++) {
            conv_re = 0.0;
            conv_im = 0.0;
            for(t = 0; t < i; t++) {
                y_idx = i-t-1;
                conv_re += x_re[t] * y_re[y_idx] - x_im[t] * y_im[y_idx];
                conv_im += x_im[t] * y_re[y_idx] + x_re[t] * y_im[y_idx];
            }
            result[idx] = sqrt(conv_re*conv_re + conv_im*conv_im);
            idx++;
        }

        for(i = 0; i < num_of_sample_x; i++) {
            conv_re = 0.0;
            conv_im = 0.0;
            for(t = i; t < num_of_sample_y; t++) {
                y_idx = num_of_sample_x-1-(t-i);
                conv_re += x_re[t] * y_re[y_idx] - x_im[t] * y_im[y_idx];
                conv_im += x_im[t] * y_re[y_idx] + x_re[t] * y_im[y_idx];
            }
            result[idx] = sqrt(conv_re*conv_re + conv_im*conv_im);
            idx++;
        }
    }

    return 1;

}


int32_t bytes_to_bits(uint8_t *bytes, uint8_t *bits, int32_t bit_len)
{
    int32_t i;
    int32_t j;   
    int32_t byte_count;
    uint8_t bit_mask;
    uint8_t byte_data;

    byte_count = (bit_len + 7) / 8;

    for(i = 0; i < byte_count; i++) {
        bit_mask = 0x80;
        byte_data = bytes[i];
        for(j = 0; j < 8 && ((i * 8 + j) < bit_len); j++) {
            if(byte_data & bit_mask) {
                bits[i*8+j] = 1;
            }
            else {
                bits[i*8+j] = 0;
            }
            bit_mask = bit_mask >> 1;
        }
    }

    return 0;
}

int32_t uint64_to_bits(uint64_t data, uint8_t *bits, int32_t bit_len)
{
    int32_t i;
    uint64_t bit_mask;

    bit_mask = 1;
    bit_mask = bit_mask << 63;
    for(i = 0; i < bit_len; i++) {
        if(data & bit_mask) {
            bits[i] = 1;
        }
        else {
            bits[i] = 0;
        }        
        bit_mask = bit_mask >> 1;
    }
    
    return 0;

}

int32_t uint32_to_bits(uint32_t data, uint8_t *bits, int32_t bit_len)
{
    int32_t i;
    uint32_t bit_mask;

    bit_mask = 1;
    bit_mask = bit_mask << 31;
    for(i = 0; i < bit_len; i++) {
        if(data & bit_mask) {
            bits[i] = 1;
        }
        else {
            bits[i] = 0;
        }        
        bit_mask = bit_mask >> 1;
    }
    return 0;
}


void print_quantize_sample(int16_t *re, int16_t *im, int count, int offset)
{
    int i;

    for(i = 0; i < count; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%6d,%6d]", offset + i, re[offset + i], im[offset + i]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%6d,%6d]\n", re[offset + i], im[offset + i]);
        }
        else {
            fprintf(stderr, " [%6d,%6d]", re[offset + i], im[offset + i]);
        }
    }
    if((i + 1) % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}


void print_complex_number(float *re, float *im, int count, int offset)
{
    int i;

    for(i = 0; i < count; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%8.2f,%8.2f]", offset + i, re[offset + i], im[offset + i]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%8.2f,%8.2f]\n", re[offset + i], im[offset + i]);
        }
        else {
            fprintf(stderr, " [%8.2f,%8.2f]", re[offset + i], im[offset + i]);
        }
    }
    if((i + 1) % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}

void print_complex_number_normalized(float *re, float *im, int count, int offset)
{
    int i;

    for(i = 0; i < count; i++) {
        if((i % ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] [%5.2f,%5.2f]", offset + i, re[offset + i], im[offset + i]);
        }
        else if(((i + 1) % ITEM_PER_LINE) == 0) {
            fprintf(stderr, " [%5.2f,%5.2f]\n", re[offset + i], im[offset + i]);
        }
        else {
            fprintf(stderr, " [%5.2f,%5.2f]", re[offset + i], im[offset + i]);
        }
    }
    if((i + 1) % ITEM_PER_LINE) {
        fprintf(stderr, "\n");
    }
}



void print_iqdata(float *symb_re, float *symb_im, int symbol_len)
{
    int i;
    for(i = 0;i < symbol_len; i++) {
        if((i % 8) == 0) {
            fprintf(stderr, "[%5d] [%5d,%5d]", i, (int)symb_re[i], (int)symb_im[i]);
        }
        else if((i % 8) == 7) {
            fprintf(stderr, " [%5d,%5d]\n", (int)symb_re[i], (int)symb_im[i]);
        }
        else {
            fprintf(stderr, " [%5d,%5d]", (int)symb_re[i], (int)symb_im[i]);
        }
    }
    fprintf(stderr, "\n");

}

void print_iqdata_f(float *symb_re, float *symb_im, int symbol_len)
{
    int i;
    for(i = 0;i < symbol_len; i++) {
        if((i % 8) == 0) {
            fprintf(stderr, "[%5d] [%5.2f,%5.2f]", i, symb_re[i], symb_im[i]);
        }
        else if((i % 8) == 7) {
            fprintf(stderr, " [%5.2f,%5.2f]\n", symb_re[i], symb_im[i]);
        }
        else {
            fprintf(stderr, " [%5.2f,%5.2f]", symb_re[i], symb_im[i]);
        }
    }
    fprintf(stderr, "\n");
}

void print_float_scaled(float *data_buff, int data_len, float scale)
{
    float *normalized_data;
    int i;

    normalized_data = (float *)calloc(sizeof(float), data_len);
    if(normalized_data == NULL) {
        return;
    }

    /* Normalize to Max Value */
    if(scale > 0) {
        for(i=0; i<data_len; i++) {
            normalized_data[i] = data_buff[i] / scale;
        }
    }
    else {
        for(i=0; i<data_len; i++) {
            normalized_data[i] = data_buff[i];
        }
    }

    /* Print */
    for(i = 0; i < data_len; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            if(data_len < 10000) {
                fprintf(stderr, "[%04d] %6.4f", i, normalized_data[i]);
            }
            else {
                fprintf(stderr, "[%05d] %6.4f", i, normalized_data[i]);
            }
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %6.4f\n", normalized_data[i]);
        }
        else {
            fprintf(stderr, " %6.4f", normalized_data[i]);
        }
    }

    if((i % LIBLTE_PRINT_ITEM_PER_LINE) != 0) {
        fprintf(stderr, "\n");
    }

    free(normalized_data);

}


void print_float_scaled2(float *data_buff, int data_len, float scale)
{
    float *normalized_data;
    int i;
    int j;

    normalized_data = (float *)calloc(sizeof(float), data_len);
    if(normalized_data == NULL) {
        return;
    }

    /* Normalize to Max Value */
    if(scale > 0) {
        for(i = 0; i < data_len; i++) {
            normalized_data[i] = data_buff[i] / scale;
        }
    }
    else {
        for(i = 0; i < data_len; i++) {
            normalized_data[i] = data_buff[i];
        }
    }

    /* Print */
    for(i = 0; i < data_len; i += 20) {
        for(j = 0; (j < 20) && ((i+j) < data_len); j++) {
            if(normalized_data[i+j] > 0.5) {
                break;
            }
        }
        if(j == 20) {
            continue;
        }

        for(j = 0; (j < 20) && ((i+j) < data_len); j++) {
            if(((i + j) % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
                if(data_len < 10000) {
                    fprintf(stderr, "[%04d] %6.4f", (i + j), normalized_data[(i + j)]);
                }
                else if(data_len < 100000) {
                    fprintf(stderr, "[%05d] %6.4f", (i + j), normalized_data[(i + j)]);
                }
                else {
                    fprintf(stderr, "[%06d] %6.4f", (i + j), normalized_data[(i + j)]);
                }
            }
            else if(((i + j) % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
                fprintf(stderr, " %6.4f\n", normalized_data[(i + j)]);
            }
            else {
                fprintf(stderr, " %6.4f", normalized_data[(i + j)]);
            }
        }

        if((j % 20) !=  0) {
            fprintf(stderr, "\n");
        }

    }

    free(normalized_data);

}



void liblte_print_complex_number_polar(float *mag, float *phase, int data_len)
{
    int i;

    /* Print */
    for(i = 0; i < data_len; i++) {
        if((i % 10) == 0) {
            fprintf(stderr, "[%04d] [%8.2f,%6.2f]", i, mag[i], phase[i]);
        }
        else if((i % 10) == 9) {
            fprintf(stderr, " [%8.2f,%6.2f]\n", mag[i], phase[i]);
        }
        else {
            fprintf(stderr, " [%8.2f,%6.2f]", mag[i], phase[i]);
        }
    }
}

void print_pss(int N_id_2, float *pss_re, float *pss_im)
{
    int i;

    fprintf(stderr, "PSS (Primary synchronization Signal)\n");
    fprintf(stderr, "N_id_2 = %d\n", N_id_2);

    print_complex_number(pss_re, pss_im, PSS_SEQ_LENGTH, 0);

    fprintf(stderr, "\n");
}


void print_int8_data(int8_t *data, int count)
{
    int i;

    for(i = 0; i < count; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %4d", i, data[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %4d\n", data[i]);
        }
        else {
            fprintf(stderr, " %4d", data[i]);
        }
    }
    fprintf(stderr, "\n");
}

void liblte_print_float(float *data_buff, int data_len)
{
    int i;

    /* Print */
    for(i = 0; i < data_len; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %6.1f", i, data_buff[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %6.1f\n", data_buff[i]);
        }
        else {
            fprintf(stderr, " %6.1f", data_buff[i]);
        }
    }
    fprintf(stderr, "\n");

}

void print_uint32_data(uint32_t *data, int count)
{
    int i;

    for(i = 0; i < count; i++) {
        if((i % LIBLTE_PRINT_ITEM_PER_LINE) == 0) {
            fprintf(stderr, "[%04d] %4d", i, data[i]);
        }
        else if((i % LIBLTE_PRINT_ITEM_PER_LINE) == (LIBLTE_PRINT_ITEM_PER_LINE - 1)) {
            fprintf(stderr, " %4d\n", data[i]);
        }
        else {
            fprintf(stderr, " %4d", data[i]);
        }
    }
    fprintf(stderr, "\n");
}

void print_bits(uint8_t *bits, int bitcount)
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

void print_bits_to_bytes(uint8_t *bits, int bitcount)
{
    int i;
    int j;
    int byte_count;
    uint8_t *bytes_array;
    uint8_t byte_data;
    uint8_t bit_mask;

    byte_count = (bitcount + 7) / 8;
    bytes_array = (uint8_t *)malloc(byte_count);

    for(i = 0; i < byte_count; i++) {
        byte_data = 0;
        bit_mask = 0x80;
        for(j = 0; j < 8 && ((i * 8 + j) < bitcount); j++) {
            if(bits[i*8 + j] != 0) {
                byte_data = byte_data | bit_mask;
            }
            bit_mask = bit_mask >> 1;
        }
        bytes_array[i] = byte_data;
    }

    fprintf(stderr, " 0x");
    for(i = 0; i < byte_count; i++) {
        fprintf(stderr, "%02X", bytes_array[i]);
    }

    fprintf(stderr, " : ");
    for(i = 0; i < byte_count; i++) {
        fprintf(stderr, " %02X", bytes_array[i]);
    }
    fprintf(stderr, "\n");

}



char *Get_SIB_TYPE_Info_Str(int32_t Value)
{
    int i;

    for(i = 0; SIB_TYPE_Info_Str[i].Value >= 0; i++) {
        if(SIB_TYPE_Info_Str[i].Value == Value) {
            return SIB_TYPE_Info_Str[i].Name;
        }
    }

    return null_str;

}

char *Get_Rrc_Sib_Type_Str(int32_t Value)
{
    int i;

    for(i = 0; RRC_SIB_TYPE_Info_str[i].Value >= 0; i++) {
        if(RRC_SIB_TYPE_Info_str[i].Value == Value) {
            return RRC_SIB_TYPE_Info_str[i].Name;
        }
    }

    return null_str;
}

char *Get_PHICH_DURATION_str(int32_t Value)
{
    int i;

    for(i = 0; PHICH_DURATION_str[i].Value >= 0; i++) {
        if(PHICH_DURATION_str[i].Value == Value) {
            return PHICH_DURATION_str[i].Name;
        }
    }

    return null_str;
}

float liblte_get_max_value(float *data_buff, int data_len)
{
    float max_data;
    int i;

    max_data = 0.0;
    for(i = 0; i < data_len; i++) {
        if(data_buff[i] > max_data) {
            max_data = data_buff[i];
        }
    }

    return max_data;
}

void print_0x4_corr(LIBLTE_PHY_STRUCT *phy_struct)
{
    int i;
    int N_print_sample;

    float *dl_timing_abs_corr;
    float abs_corr_max;

    dl_timing_abs_corr = (float *)calloc(sizeof(float), phy_struct->N_samps_per_slot + phy_struct->N_samps_cp_l_else);
    if(dl_timing_abs_corr == NULL) {
        return;
    }

    for(i = 0; i < phy_struct->N_samps_per_slot; i++) {
        dl_timing_abs_corr[i] = phy_struct->dl_timing_abs_corr[i] * phy_struct->dl_timing_abs_corr[(phy_struct->N_samps_cp_l_0 + phy_struct->N_samps_per_symb + (phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb)*3)+i];
    }
    for(; i < (phy_struct->N_samps_per_slot + phy_struct->N_samps_cp_l_else); i++) {
        dl_timing_abs_corr[i] = phy_struct->dl_timing_abs_corr[i] * phy_struct->dl_timing_abs_corr[(phy_struct->N_samps_cp_l_0 + phy_struct->N_samps_per_symb + (phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb)*3)+i];
    }

    N_print_sample = phy_struct->N_samps_per_slot;
    abs_corr_max = liblte_get_max_value(dl_timing_abs_corr, N_print_sample);

#if 1
    fprintf(stderr, "0 by 4th : abs_corr_max = %.2f\n", abs_corr_max);
    print_float_scaled(dl_timing_abs_corr, N_print_sample, abs_corr_max);
#endif

}

float get_channel_power(float *i_data, float *q_data, int32_t count)
{
    float sum;
    int32_t i;

    sum = 0.0;
    for(i = 0; i < count; i++) {
        sum += i_data[i] * i_data[i] + q_data[i]*q_data[i];
    }
    return sum;
}



/*

    // 6 RB
    { 21, 1, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 19, 1, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 21, 1, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    {  8, 1, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 21, 1, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 21, 1, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 21, 2, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 19, 2, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 21, 2, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 22, 2, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    {  8, 2, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 22, 2, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 31, 2, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 28, 2, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 21, 2, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 21, 2, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 21, 4, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 19, 4, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 21, 4, LTE_DCI_FORMAT_1A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 4, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    {  8, 4, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 4, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 34, 4, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 30, 4, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 21, 4, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 21, 4, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    // 15 RB
    { 22, 1, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 23, 1, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 21, 1, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 10, 1, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 22, 1, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 22, 1, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 22, 2, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 23, 2, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 22, 2, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 2, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 10, 2, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 2, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 34, 2, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 31, 2, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 22, 2, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 22, 2, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 22, 4, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 23, 4, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 22, 4, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 10, 4, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 37, 4, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 33, 4, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 22, 4, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 22, 4, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    // 25 RB
    { 25, 1, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 25, 1, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 12, 1, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 1, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 1, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 25, 2, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 25, 2, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 12, 2, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 39, 2, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 36, 2, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 25, 2, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 2, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 25, 4, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 25, 4, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 4, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 12, 4, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 4, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 42, 4, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 38, 4, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 25, 4, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 25, 4, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    // 50 RB
    { 27, 1, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 31, 1, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 13, 1, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 27, 2, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 31, 2, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 2, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 13, 2, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 2, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 43, 2, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 41, 2, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 27, 2, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 27, 4, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 31, 4, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 30, 4, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 13, 4, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 30, 4, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 46, 4, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 42, 4, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 27, 4, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    // 75 RB
    { 27, 1, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 33, 1, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 14, 1, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 1, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 27, 2, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 33, 2, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 29, 2, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 14, 2, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 29, 2, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 45, 2, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 42, 2, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 27, 2, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 2, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 27, 4, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 33, 4, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 31, 4, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 14, 4, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 31, 4, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 48, 4, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 45, 4, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 27, 4, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 27, 4, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    // 100 RB
    { 28, 1, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 39, 1, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 28, 1, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 15, 1, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 1, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 1, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 28, 2, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 39, 2, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 28, 2, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 30, 2, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 15, 2, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 30, 2, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 51, 2, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 48, 2, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 28, 2, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 2, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },

    { 28, 4, LTE_DCI_FORMAT_0,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 39, 4, LTE_DCI_FORMAT_1,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2                                             | LTE_TM_7 },
    { 28, 4, LTE_DCI_FORMAT_1A, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 33, 4, LTE_DCI_FORMAT_1B, LTE_C_RNTIS,                                                        LTE_TM_6            },
    { 15, 4, LTE_DCI_FORMAT_1C, LTE_S_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 33, 4, LTE_DCI_FORMAT_1D, LTE_C_RNTIS,                                             LTE_TM_5                       },
    { 54, 4, LTE_DCI_FORMAT_2,  LTE_C_RNTIS,                                  LTE_TM_4                                  },
    { 50, 4, LTE_DCI_FORMAT_2A, LTE_C_RNTIS,                       LTE_TM_3                                             },
    { 28, 4, LTE_DCI_FORMAT_3,  LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },
    { 28, 4, LTE_DCI_FORMAT_3A, LTE_C_RNTIS, LTE_TM_1 | LTE_TM_2 | LTE_TM_3 | LTE_TM_4 | LTE_TM_5 | LTE_TM_6 | LTE_TM_7 },


*/

int32_t dci_format_0_1a_3_3a_fdd_len(int32_t N_rb_dl)
{
    struct nrb2bits_s dci_format_0_1A_3_3A_len_tbl[] = {
        {   6, 21},
        {  15, 22},
        {  25, 25},
        {  50, 27},
        {  75, 27},
        { 100, 28},
        {  -1, -1},
    };

    for(int x = 0; dci_format_0_1A_3_3A_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_0_1A_3_3A_len_tbl[x].nrb) {
            return dci_format_0_1A_3_3A_len_tbl[x].bits;
        }
    }
    return -1;
    
}

int32_t dci_format_1_fdd_len(int32_t N_rb_dl)
{
    struct nrb2bits_s dci_format_1_len_tbl[] = {
        {   6, 19},
        {  15, 23},
        {  25, 27},
        {  50, 31},
        {  75, 33},
        { 100, 39},
        {  -1, -1},
    };

    for(int x = 0; dci_format_1_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_1_len_tbl[x].nrb) {
            return dci_format_1_len_tbl[x].bits;
        }
    }
    return -1;

}

int32_t dci_format_1c_fdd_len(int32_t N_rb_dl)
{
    struct nrb2bits_s dci_format_1C_len_tbl[] = {
        {   6,  8},
        {  15, 10},
        {  25, 12},
        {  50, 13},
        {  75, 14},
        { 100, 15},
        {  -1, -1},
    };

    for(int x = 0; dci_format_1C_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_1C_len_tbl[x].nrb) {
            return dci_format_1C_len_tbl[x].bits;
        }
    }
    return -1;
}


int32_t dci_format_2_fdd_len(int32_t N_rb_dl, int32_t N_ant)
{

    struct nrb2_bits_per_ant_s dci_format_2_len_tbl[] = {
        {   6, { -1, 31, -1, 34 } },
        {  15, { -1, 34, -1, 37 } },
        {  25, { -1, 39, -1, 42 } },
        {  50, { -1, 43, -1, 46 } },
        {  75, { -1, 45, -1, 48 } },
        { 100, { -1, 51, -1, 54 } },
        {  -1, { -1,            } },
    };

    for(int x = 0; dci_format_2_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_2_len_tbl[x].nrb) {
            if(N_ant <= MAX_ANTENNA) {
                return dci_format_2_len_tbl[x].bits[N_ant - 1];
            }
        }
    }
    return -1;

}


int32_t dci_format_2a_fdd_len(int32_t N_rb_dl, int32_t N_ant)
{

    struct nrb2_bits_per_ant_s dci_format_2_len_tbl[] = {
        {   6, { -1, 28, -1, 30 } },
        {  15, { -1, 31, -1, 33 } },
        {  25, { -1, 36, -1, 38 } },
        {  50, { -1, 41, -1, 42 } },
        {  75, { -1, 42, -1, 45 } },
        { 100, { -1, 48, -1, 50 } },
        {  -1, { -1,            } },
    };

    for(int x = 0; dci_format_2_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_2_len_tbl[x].nrb) {
            if(N_ant <= MAX_ANTENNA) {
                return dci_format_2_len_tbl[x].bits[N_ant - 1];
            }
        }
    }
    return -1;

}


int32_t ra_type0_len(int32_t N_rb_dl)
{
    struct nrb2bits_s dci_format_1C_len_tbl[] = {
        {   6,  6},
        {  15,  8},
        {  25, 13},
        {  50, 17},
        {  75, 19},
        { 100, 25},
        {  -1, -1},
    };

    int32_t dci_len = -1;
    for(int x = 0; dci_format_1C_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_1C_len_tbl[x].nrb) {
            dci_len = dci_format_1C_len_tbl[x].bits;
        }
    }
    return dci_len;
}

int32_t ra_n_rb_rbg_subset(int32_t N_rb_dl, uint32_t subset)
{

    uint32_t P;
    uint32_t decision;

    P = RBG_size(N_rb_dl);

    decision = (N_rb_dl - 1) % P;
    if(subset < decision) {
        return ((N_rb_dl - 1) / (P * P)) * P + P;
    }
    else if(subset == decision) {
        return ((N_rb_dl - 1) / (P * P)) * P + (N_rb_dl - 1) % P + 1;
    }
    else {
        return ((N_rb_dl - 1) / (P * P)) * P;
    }

}


int32_t ra_type0_shift_bit(int32_t N_rb_dl, int32_t subset)
{

    // 15 RB, P = 2, 6
    // 012345678901234
    // 01  23  45  67
    //   01  23  45  6
    // 2, 1
    
    // 25 RB, 25, P = 2, 11
    // 0123456789012345678901234
    // 01  23  45  67  89  AB  C
    //   01  23  45  67  89  AB 
    // 2, 1
                
    // 50 RB, 25, P = 3, 14
    // 17 (18), 16 (17), 14 (15)
    //  4     ,       3,      1
    // 01234567890123456789012345678901234567890123456789
    // 012      345      678      9AB      CDE      F01
    //    012      345      678      9AB      CDE      F0
    //       012      345      678      9AB      CDE     
    // 4,3,1
    
    // 75 RB, 25, P = 4, 16
    // 012345678901234567890123456789012345678901234567890123456789012345678901234
    // 0123            4567            89AB            CDEF            0123              
    //     0123            4567            89AB            CDEF            0123         
    //         0123            4567            89AB            CDEF            012  
    //             0123            4567            89AB            CDEF           
    // 4, 4, 3, 0
    
    // 100 RB, 25, P = 4, 22
    // 0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
    // 0123            4567            89AB            CDEF            0123            4567            89AB              
    //     0123            4567            89AB            CDEF            0123            4567                
    //         0123            4567            89AB            CDEF            0123            4567              
    //             0123            4567            89AB            CDEF            0123            4567                    
    // 6, 2, 2, 2

    struct nrb2rashift_bit_s nrb2rashift_bits[] = {
        {  15,  2, {2, 1,      }},
        {  25,  2, {2, 1,      }},
        {  50,  3, {4, 3, 1,   }},
        {  75,  4, {4, 4, 3,   }},
        { 100,  4, {6, 2, 2, 2 }},
        {  -1, -1, {-1,        }},
    };

    int32_t shift_bits = -1;
    for(int x = 0; nrb2rashift_bits[x].nrb > 0; x++) {
        if(N_rb_dl == nrb2rashift_bits[x].nrb) {
            if(subset < nrb2rashift_bits[x].nsubset) {
                return nrb2rashift_bits[x].shit_bits[subset];
            }
            else {
                return -1;
            }
        }       
    }

    return -1;

}

int32_t RBG_size(int32_t N_rb_dl)
{
    struct nrb2bits_s dci_format_1C_len_tbl[] = {
        {   6,  1},
        {  15,  2},
        {  25,  2},
        {  50,  3},
        {  75,  4},
        { 100,  4},
        {  -1, -1},
    };

    int32_t dci_len = -1;
    for(int x = 0; dci_format_1C_len_tbl[x].nrb > 0; x++) {
        if(N_rb_dl == dci_format_1C_len_tbl[x].nrb) {
            dci_len = dci_format_1C_len_tbl[x].bits;
        }
    }
    return dci_len;
}

int32_t dci_0_1a_unpack_fdd
(
    uint8                           *in_bits,
    uint32                           N_in_bits,
    int32_t                          ca_presence,
    uint16                           rnti,
    uint32                           N_rb_dl,
    uint32                           N_rb_ul,
    uint8                            N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,    
    struct dci_format_s             *dci    
)
{
    uint8_t             *dci_bits = in_bits;
    struct dci_format_0_1a_hdr_s *dci_0_1a_hdr;;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);

    dci_0_1a_hdr = (struct dci_format_0_1a_hdr_s *)&dci->d;

    // Carrier indicator
    if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
        dci_0_1a_hdr->ca_ind = liblte_bits_2_value(&dci_bits, 3);
    }

    // Check DCI 0/1A flag 3GPP TS 36.212 v10.1.0 section 5.3.3.1.3
    dci_0_1a_hdr->dci_0_1a_flag = liblte_bits_2_value(&dci_bits, 1);
    if(DCI_0_1A_FLAG_0 == dci_0_1a_hdr->dci_0_1a_flag) {
        dci->format = LTE_DCI_FORMAT_0;
        return dci_0_unpack_fdd(in_bits, N_in_bits, ca_presence, rnti, N_rb_dl, N_ant, alloc, (struct dci_format_0_fdd_s *)&dci->d);
    }
    else {
        dci->format = LTE_DCI_FORMAT_1A;
        return dci_1a_unpack_fdd(in_bits, N_in_bits, ca_presence, rnti, N_rb_ul, N_ant, alloc, (struct dci_format_1a_fdd_s *)&dci->d);
    }

}

int32_t dci_0_unpack_fdd
(
    uint8_t                         *in_bits,
    uint32_t                        N_in_bits,
    int32_t                         ca_presence,
    uint16_t                        rnti,
    uint32_t                        N_rb_ul,
    uint8_t                         N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,    
    struct dci_format_0_fdd_s       *dci_0
)
{

    int32_t  result = LIBLTE_SUCCESS;
    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);
    uint8_t             *dci_bits = in_bits;

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "dci_0_unpack\n");
        fprintf(stderr, "    Unpacks all of the fields from the Downlink Control Information format 0\n");
        fprintf(stderr, "    3GPP TS 36.212 v10.1.0 section 5.3.3.1.1\n");
        fprintf(stderr, "    N_in_bits = %d\n", N_in_bits);
        fprintf(stderr, "    N_rb_ul = %d\n", N_rb_ul);
        fprintf(stderr, "    N_ant = %d\n", N_ant);
    }

    // Carrier indicator
    if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
        dci_0->ca_ind = liblte_bits_2_value(&dci_bits, 3);
    }

    // Flag for format0/format1A differentiation – 1 bit
    dci_0->dci_0_1a_flag = liblte_bits_2_value(&dci_bits, 1);

    // Frequency hopping flag – 1 bit as defined in section 8.4 of [3]
    dci_0->freq_hopping_flag = liblte_bits_2_value(&dci_bits, 1);

    dci_0->riv_length = (int32_t)ceilf(logf(N_rb_ul*(N_rb_ul+1)/2)/logf(2));
    if(dci_0->freq_hopping_flag) {

        // For PUSCH hopping (single cluster allocation only)
        if(N_rb_ul >= 50) {
            dci_0->N_ul_hop = liblte_bits_2_value(&dci_bits, 2);
            dci_0->riv = liblte_bits_2_value(&dci_bits, dci_0->riv_length - 2);
            if(dci_0->N_ul_hop == 3) {
                dci_0->hopping_type = 2;
            }
            else {
                dci_0->hopping_type = 1;
            }
        }
        else {
            dci_0->N_ul_hop = liblte_bits_2_value(&dci_bits, 1);
            dci_0->riv = liblte_bits_2_value(&dci_bits, dci_0->riv_length - 1);
            if(dci_0->N_ul_hop == 1) {
                dci_0->hopping_type = 2;
            }
            else {
                dci_0->hopping_type = 1;
            }           
        }   
        
        // Modulation and coding scheme and redundancy version – 5 bits as defined in section 8.6 of 3GPP TS 36.213
        dci_0->mcs_and_redundancy_ver = liblte_bits_2_value(&dci_bits, 5);

        // New data indicator – 1 bit
        dci_0->ndi = liblte_bits_2_value(&dci_bits, 1);

        // TPC command for scheduled PUSCH – 2 bits as defined in section 5.1.1.1 of 3GPP TS 36.213
        dci_0->tpc = liblte_bits_2_value(&dci_bits, 2);

        // Cyclic shift for DM RS and OCC index – 3 bits as defined in section 5.5.2.1.1 of 3GPP TS 36.211
        dci_0->cyclic_shift_for_dm_rs = liblte_bits_2_value(&dci_bits, 3);

        // CQI request – 1 or 2 bits as defined in section 7.2.1 of 3GPP TS 36.213. The 2-bit field only applies to UEs that are configured
        // with more than one DL cell and when the corresponding DCI is mapped onto the UE specific by C-RNTI search
        // space as defined in 3GPP TS 36.213
        dci_0->cqi_req = liblte_bits_2_value(&dci_bits, 1);

        dci_0->pad_byte = liblte_bits_2_value(&dci_bits, 1);

        // ---------------------------------------------
        // RIV = N_rb_ul * (L_CRBs - 1) + RB_START
        dci_0->RB_start[0] = dci_0->riv % N_rb_ul;
        dci_0->L_crbs[0] = dci_0->riv/N_rb_ul + 1;

        // RIV = N_rb_ul * (N_rb_ul - L_CRBs + 1) + (N_ul_rb - 1 - RB_START)
        // RB_START = N_rb_ul - 1 - RIV % N_rb_ul
        // L_CRBs = N_rb_ul + 1 - RIV / N_rb_ul
        dci_0->RB_start[1] = N_rb_ul - 1 -  (dci_0->riv % N_rb_ul);
        dci_0->L_crbs[1] = N_rb_ul - dci_0->riv/N_rb_ul + 1;        

        fprintf(stderr, "--------------------------------\n");
        if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
            fprintf(stderr, "WARNING: Not handling carrier indicator : ca_ind = %d\n", dci_0->ca_ind);
        }
        fprintf(stderr, "    Check DCI 0/1A flag 3GPP TS 36.212 v10.1.0 section 5.3.3.1.3\n");
        fprintf(stderr, "        dci_0_1a_flag = %d\n", dci_0->dci_0_1a_flag);
        fprintf(stderr, "    rnti = 0x%04x (%d)\n", rnti, rnti);
        fprintf(stderr, "\n");
        fprintf(stderr, "    freq_hopping_flag = %d\n", dci_0->freq_hopping_flag);
        fprintf(stderr, "    N_ul_hop = %d (0x%02x)\n", dci_0->N_ul_hop, dci_0->N_ul_hop);
        fprintf(stderr, "    Type %d PUSCH Hopping\n", dci_0->hopping_type);
        fprintf(stderr, "    RIV_length = %d\n", dci_0->riv_length);
        fprintf(stderr, "    RIV = %d (0x%04x)\n", dci_0->riv, dci_0->riv);
        fprintf(stderr, "    RB_start[0] = %d, L_crbs[0] = %d\n", dci_0->RB_start[0], dci_0->L_crbs[0]);
        fprintf(stderr, "    RB_start[1] = %d, L_crbs[1] = %d\n", dci_0->RB_start[1], dci_0->L_crbs[1]);
        fprintf(stderr, "\n");
        fprintf(stderr, "Extract the rest of the fields\n");
        fprintf(stderr, "    mcs and redundancy version = %d (0x%04x)\n", dci_0->mcs_and_redundancy_ver, dci_0->mcs_and_redundancy_ver);
        fprintf(stderr, "    New data indicator = %d (0x%04x)\n", dci_0->ndi, dci_0->ndi);
        fprintf(stderr, "    TPC command for scheduled PUSCH  = %d (0x%04x)\n", dci_0->tpc, dci_0->tpc);
        fprintf(stderr, "    Cyclic shift for DM RS and OCC index = %d (0x%04x)\n", dci_0->cyclic_shift_for_dm_rs, dci_0->cyclic_shift_for_dm_rs);
        fprintf(stderr, "    CQI request = %d (0x%04x)\n", dci_0->cqi_req, dci_0->cqi_req);
        fprintf(stderr, "    Padding bytes = %d (0x%04x)\n", dci_0->pad_byte, dci_0->pad_byte);

        return LIBLTE_SUCCESS;
    }
    else {

        // Resource block assignment and hopping resource allocation
        dci_0->riv = liblte_bits_2_value(&dci_bits, dci_0->riv_length);

        // Modulation and coding scheme and redundancy version – 5 bits as defined in section 8.6 of 3GPP TS 36.213
        dci_0->mcs_and_redundancy_ver = liblte_bits_2_value(&dci_bits, 5);

        // New data indicator – 1 bit
        dci_0->ndi = liblte_bits_2_value(&dci_bits, 1);

        // TPC command for scheduled PUSCH – 2 bits as defined in section 5.1.1.1 of 3GPP TS 36.213
        dci_0->tpc = liblte_bits_2_value(&dci_bits, 2);

        // Cyclic shift for DM RS and OCC index – 3 bits as defined in section 5.5.2.1.1 of 3GPP TS 36.211
        dci_0->cyclic_shift_for_dm_rs = liblte_bits_2_value(&dci_bits, 3);

        // CQI request – 1 or 2 bits as defined in section 7.2.1 of 3GPP TS 36.213. The 2-bit field only applies to UEs that are configured
        // with more than one DL cell and when the corresponding DCI is mapped onto the UE specific by C-RNTI search
        // space as defined in 3GPP TS 36.213
        dci_0->cqi_req = liblte_bits_2_value(&dci_bits, 1);

        dci_0->pad_byte = liblte_bits_2_value(&dci_bits, 1);

        // SRS request – 0 or 1 bit. This field can only be present in DCI formats scheduling PUSCH which are mapped onto
        // the UE specific by C-RNTI search space as defined in 3GPP TS 36.213.

        // ---------------------------------------------
        // RIV = N_rb_ul * (L_CRBs - 1) + RB_START
        dci_0->RB_start[0] = dci_0->riv % N_rb_ul;
        dci_0->L_crbs[0] = dci_0->riv/N_rb_ul + 1;

        // RIV = N_rb_ul * (N_rb_ul - L_CRBs + 1) + (N_ul_rb - 1 - RB_START)
        // RB_START = N_rb_ul - 1 - RIV % N_rb_ul
        // L_CRBs = N_rb_ul + 1 - RIV / N_rb_ul
        dci_0->RB_start[1] = N_rb_ul - 1 -  (dci_0->riv % N_rb_ul);
        dci_0->L_crbs[1] = N_rb_ul - dci_0->riv/N_rb_ul + 1;

        if(log_msg_enabled) {

            fprintf(stderr, "--------------------------------\n");
            if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
                fprintf(stderr, "WARNING: Not handling carrier indicator : ca_ind = %d\n", dci_0->ca_ind);
            }
            fprintf(stderr, "    Check DCI 0/1A flag 3GPP TS 36.212 v10.1.0 section 5.3.3.1.3\n");
            fprintf(stderr, "        dci_0_1a_flag = %d\n", dci_0->dci_0_1a_flag);
            fprintf(stderr, "    rnti = 0x%04x (%d)\n", rnti, rnti);
            fprintf(stderr, "\n");
            fprintf(stderr, "    freq_hopping_flag = %d\n", dci_0->freq_hopping_flag);
            fprintf(stderr, "    RIV_length = %d\n", dci_0->riv_length);
            fprintf(stderr, "    RIV = %d\n", dci_0->riv);
            fprintf(stderr, "    N_rb_ul / 2 = %d\n", N_rb_ul / 2);
            fprintf(stderr, "    RB_start[0] = %d, L_crbs[0] = %d\n", dci_0->RB_start[0], dci_0->L_crbs[0]);
            fprintf(stderr, "    RB_start[1] = %d, L_crbs[1] = %d\n", dci_0->RB_start[1], dci_0->L_crbs[1]);
            fprintf(stderr, "\n");
            fprintf(stderr, "Extract the rest of the fields\n");
            fprintf(stderr, "    mcs and redundancy version = %d (0x%04x)\n", dci_0->mcs_and_redundancy_ver, dci_0->mcs_and_redundancy_ver);
            fprintf(stderr, "    New data indicator = %d (0x%04x)\n", dci_0->ndi, dci_0->ndi);
            fprintf(stderr, "    TPC command for scheduled PUSCH  = %d (0x%04x)\n", dci_0->tpc, dci_0->tpc);
            fprintf(stderr, "    Cyclic shift for DM RS and OCC index = %d (0x%04x)\n", dci_0->cyclic_shift_for_dm_rs, dci_0->cyclic_shift_for_dm_rs);
            fprintf(stderr, "    CQI request = %d (0x%04x)\n", dci_0->cqi_req, dci_0->cqi_req);
            fprintf(stderr, "    Padding bytes = %d (0x%04x)\n", dci_0->pad_byte, dci_0->pad_byte);


        }

        return LIBLTE_SUCCESS;

    }

}

int32_t dci_1_unpack_fdd
(
    uint8                           *in_bits,
    uint32                           N_in_bits,
    int32_t                          ca_presence,
    uint16                           rnti,
    uint32                           N_rb_dl,
    uint8                            N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,    
    struct dci_format_1_fdd_s        *dci_1
)
{
    int32_t     i;
    int32_t     j;
    uint8_t     *dci_bits = in_bits;
    int32_t     ra_len;
    int32_t     type1_only_field_len;
    int32_t     P;
    uint32_t    bit_mask;
    int32_t     k;
    int32_t     decision_p;
    int32_t     p;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "dci_1_unpack\n");
        fprintf(stderr, "    Unpacks all of the fields from the Downlink Control Information format 1\n");
        fprintf(stderr, "    3GPP TS 36.212 v10.1.0 section 5.3.3.1.2\n");
        fprintf(stderr, "    N_in_bits = %d\n", N_in_bits);
        fprintf(stderr, "    N_rb_dl = %d\n", N_rb_dl);
        fprintf(stderr, "    N_ant = %d\n", N_ant);
    }

    // Carrier indicator
    if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
        dci_1->ca_ind = liblte_bits_2_value(&dci_bits, 3);
    }

    // Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.212 [3]
    if(N_rb_dl < 10) {
        dci_1->resource_alloc_type = 0;
    }
    else {
        dci_1->resource_alloc_type = liblte_bits_2_value(&dci_bits, 1);
    }

    ra_len = ra_type0_len(N_rb_dl);
    P = RBG_size(N_rb_dl);

    dci_1->delta_shift = 0;
    type1_only_field_len = 0;
    if(dci_1->resource_alloc_type == 1) {
        
        dci_1->N_rb_type1 = (N_rb_dl + (P - 1)) / P ;
        if(N_rb_dl < 50) {
            dci_1->subset = liblte_bits_2_value(&dci_bits, 1);
            type1_only_field_len = 2;
        }
        else {
            dci_1->subset = liblte_bits_2_value(&dci_bits, 2);
            type1_only_field_len = 3;
        }
        
        dci_1->shift = liblte_bits_2_value(&dci_bits, 1);
        ra_len = ra_len - type1_only_field_len;
        
        if(dci_1->subset>= P) {
            if(log_msg_enabled) {
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    RESULT : Invalid Contents\n");
            }
            return LIBLTE_ERROR_INVALID_CONTENTS;    
        }         

        dci_1->N_rb_type1 = (N_rb_dl + (P - 1)) / P - type1_only_field_len;
        dci_1->N_rb_rbg_subset = ra_n_rb_rbg_subset(N_rb_dl, dci_1->subset);
        if(dci_1->shift) {        
            dci_1->delta_shift = dci_1->N_rb_rbg_subset - dci_1->N_rb_type1;
        }

    }

    dci_1->resource_block_assignment = liblte_bits_2_value(&dci_bits, ra_len);
    
    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    dci_1->mcs = liblte_bits_2_value(&dci_bits, 5);

    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    dci_1->harq_process = liblte_bits_2_value(&dci_bits, 3);

    // New data indicator – 1 bit
    dci_1->ndi = liblte_bits_2_value(&dci_bits, 1);

    // Redundancy version – 2 bits
    dci_1->redundancy_version = liblte_bits_2_value(&dci_bits, 2);

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    dci_1->tpc = liblte_bits_2_value(&dci_bits, 2);

    dci_1->prb_allocated = 0;
    for(i = 0; i < N_rb_dl; i++) {
        dci_1->prb[i] = 0; 
    }    

    bit_mask = 1 << (ra_len - 1);
    if(dci_1->resource_alloc_type == 0) {
        for(i = 0; i < ra_len; i++) {
            if(dci_1->resource_block_assignment & bit_mask) {
                for(j = 0; j < P && (i * P + j) < N_rb_dl; j++) {
                    dci_1->prb[i * P + j] = 1;
                }
                dci_1->prb_allocated = dci_1->prb_allocated + j;
            }
            bit_mask = bit_mask >> 1;
        } 
    }
    else {                
        
        // 7.1.6.2 Resource allocation type 1 in TS 36.213         
        for(i = 0; i < ra_len; i++) {
            if(dci_1->resource_block_assignment & bit_mask) {                    
                dci_1->prb[((i + dci_1->delta_shift) / P) * P * P +  dci_1->subset * P + (i + dci_1->delta_shift) % P] = 1;
                dci_1->prb_allocated = dci_1->prb_allocated + 1;
            }
            bit_mask = bit_mask >> 1;
        }
    }

    // Table 7.1.7.1-1: Modulation and TBS index table for PDSCH
    // Qm
    if(dci_1->mcs <= 9) {
        dci_1->mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        dci_1->i_tbs = dci_1->mcs;
    }
    else if(dci_1->mcs <= 16) {
        dci_1->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
        dci_1->i_tbs = dci_1->mcs - 1;
    }
    else if(dci_1->mcs <= 28) {
        dci_1->i_tbs = dci_1->mcs - 2;
        dci_1->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
    }    
    else if(dci_1->mcs == 29) {
        dci_1->mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    }
    else if(dci_1->mcs == 30) {
        dci_1->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
    }
    else {
        dci_1->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
    }

    dci_1->rnti = rnti;
    dci_1->tbs = TBS_71721[dci_1->i_tbs][dci_1->prb_allocated-1];

    if(N_ant == 1) {
        dci_1->tx_mode = 1;
    }
    else{
        dci_1->tx_mode = 2;
    }
    // dci_1->N_codewords = 1;

    // Copy dci_1a to alloc
    alloc->mcs = dci_1->mcs;
    alloc->rv_idx = dci_1->redundancy_version;
    alloc->mod_type = dci_1->mod_type;
    alloc->tbs = dci_1->tbs;
    alloc->rnti = dci_1->rnti;
    alloc->dci_format = LTE_DCI_FORMAT_1;    
    alloc->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
    alloc->tx_mode = dci_1->tx_mode;
    alloc->N_codewords = 1;
    
    bit_mask = 1 << (ra_len - 1);
    alloc->N_prb = 0;
    if(dci_1->resource_alloc_type == 0) {
        for(i = 0; i < ra_len; i++) {
            if(dci_1->resource_block_assignment & bit_mask) {
                for(j = 0; j < P && (i * P + j) < N_rb_dl; j++) {
                    alloc->prb[0][alloc->N_prb] = i * P + j;
                    alloc->prb[1][alloc->N_prb] = i * P + j;
                    alloc->N_prb = alloc->N_prb + 1;
                }
            }
            bit_mask = bit_mask >> 1;
        } 
    }
    else {                
        // 7.1.6.2 Resource allocation type 1 in TS 36.213         
        for(i = 0; i < ra_len; i++) {
            if(dci_1->resource_block_assignment & bit_mask) {                    
                dci_1->prb[((i + dci_1->delta_shift) / P) * P * P +  dci_1->subset * P + (i + dci_1->delta_shift) % P] = 1;
                alloc->prb[0][alloc->N_prb] = ((i + dci_1->delta_shift) / P) * P * P +  dci_1->subset * P + (i + dci_1->delta_shift) % P;
                alloc->prb[1][alloc->N_prb] = ((i + dci_1->delta_shift) / P) * P * P +  dci_1->subset * P + (i + dci_1->delta_shift) % P;
                alloc->N_prb = alloc->N_prb + 1;
            }
            bit_mask = bit_mask >> 1;
        }
    }
    
    if(log_msg_enabled) {

        fprintf(stderr, "--------------------------------\n");
        if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
            fprintf(stderr, "WARNING: Not handling carrier indicator : ca_ind : %d\n", dci_1->ca_ind);
        }

        fprintf(stderr, "    Resource allocation type : %d\n", dci_1->resource_alloc_type);
        fprintf(stderr, "    Resource block group size : %d\n", P);
        if(dci_1->resource_alloc_type == 1) {
            fprintf(stderr, "    subset : %d\n", dci_1->subset);
            if(dci_1->shift) {
                fprintf(stderr, "    RBG subset PRBs shifted : Yes (end at highest PRB)\n");
            }
            else {
                fprintf(stderr, "    RBG subset PRBs shifted : No (start at lowest PRB)\n");
            }
        }

        fprintf(stderr, "    Resource block assignment = %d (0x%04x)\n", dci_1->resource_block_assignment, dci_1->resource_block_assignment);
        fprintf(stderr, "       %d PRBs was allocated.\n", dci_1->prb_allocated);
        fprintf(stderr, "       Value in binary :");
        print_bits(dci_1->prb, N_rb_dl);

        fprintf(stderr, "    Modulation and coding scheme = %d\n", dci_1->mcs);
        fprintf(stderr, "    HARQ process number = %d\n", dci_1->harq_process);
        fprintf(stderr, "    New data indicator = %d\n", dci_1->ndi);
        fprintf(stderr, "    Redundancy version = %d\n", dci_1->redundancy_version);
        fprintf(stderr, "    TPC command for PUCCH = %d\n", dci_1->tpc);
        
        fprintf(stderr, "--------------------------------\n");
        fprintf(stderr, "    mod_type = %d (%s)\n", dci_1->mod_type, liblte_phy_modulation_type_text[dci_1->mod_type]);
        fprintf(stderr, "    Transport block size = %d\n", dci_1->tbs);

    }

    return LIBLTE_SUCCESS;

}


int32_t dci_1a_unpack_fdd
(
    uint8                           *in_bits,
    uint32                           N_in_bits,
    int32_t                          ca_presence,
    uint16                           rnti,
    uint32                           N_rb_dl,
    uint8                            N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,    
    struct dci_format_1a_fdd_s       *dci_1a
)
{
    uint32_t    i;
    uint32_t    j;
    int32_t     N_prb_1a;
    uint8_t     *dci_bits = in_bits;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "dci_1a_unpack\n");
        fprintf(stderr, "    Unpacks all of the fields from the Downlink Control Information format 1A\n");
        fprintf(stderr, "    3GPP TS 36.212 v10.1.0 section 5.3.3.1.3\n");
        fprintf(stderr, "    3GPP TS 36.213 v10.3.0 section 7.1.6.3\n");
        fprintf(stderr, "    3GPP TS 36.213 v10.3.0 section 7.1.7\n");
        fprintf(stderr, "    N_in_bits = %d\n", N_in_bits);
        fprintf(stderr, "    N_rb_dl = %d\n", N_rb_dl);
        fprintf(stderr, "    N_ant = %d\n", N_ant);
    }

    // Carrier indicator
    if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
        dci_1a->ca_ind = liblte_bits_2_value(&dci_bits, 3);
    }

    // Check DCI 0/1A flag 3GPP TS 36.212 v10.1.0 section 5.3.3.1.3
    dci_1a->dci_0_1a_flag = liblte_bits_2_value(&dci_bits, 1);

    // Determine if RIV uses local or distributed VRBs
    dci_1a->loc_or_dist = liblte_bits_2_value(&dci_bits, 1);
    if(LIBLTE_MAC_SI_RNTI == rnti || LIBLTE_MAC_P_RNTI == rnti || (LIBLTE_MAC_RA_RNTI_START <= rnti && LIBLTE_MAC_RA_RNTI_END >= rnti)) {

        if(DCI_VRB_TYPE_LOCALIZED == dci_1a->loc_or_dist) {

            // Find the RIV that was sent 3GPP TS 36.213 v10.3.0 section 7.1.6.3
            dci_1a->riv_length = (uint32)ceilf(logf(N_rb_dl*(N_rb_dl+1)/2)/logf(2));

            // Resource block assignment for Localized DRB
            dci_1a->riv = liblte_bits_2_value(&dci_bits, dci_1a->riv_length);

            // MCS
            dci_1a->mcs = liblte_bits_2_value(&dci_bits, 5);

            // HARQ Process
            dci_1a->harq_process = liblte_bits_2_value(&dci_bits, 3);

            // New data indicator
            dci_1a->ndi = liblte_bits_2_value(&dci_bits, 1);

            // Redundancy version
            dci_1a->redundancy_version = liblte_bits_2_value(&dci_bits, 2);

            // TPC command for PUCCH
            dci_1a->tpc = liblte_bits_2_value(&dci_bits, 2);

            // ---------------------------------------------
            // RIV = N_rb_dl * (L_CRBs - 1) + RB_START
            dci_1a->L_crbs[0] = dci_1a->riv / N_rb_dl + 1;
            dci_1a->RB_start[0] = dci_1a->riv % N_rb_dl;

            // RIV = N_rb_dl * (N_rb_dl - L_CRBs + 1) + (N_rb_dl - 1 - RB_START)
            // RB_START = N_rb_dl - 1 - RIV % N_rb_dl
            // L_CRBs = N_rb_dl + 1 - RIV / N_rb_dl
            dci_1a->RB_start[1] = N_rb_dl - 1 -  (dci_1a->riv % N_rb_dl);
            dci_1a->L_crbs[1] = N_rb_dl - dci_1a->riv/N_rb_dl + 1;

            if(N_ant == 1) {
                dci_1a->tx_mode = 1;
            }
            else{
                dci_1a->tx_mode = 2;
            }

            // Parse the data
            if((dci_1a->tpc % 2) == 0)  {
                N_prb_1a = 2;
            }
            else{
                N_prb_1a = 3;
            }            

            dci_1a->mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
            dci_1a->pre_coder_type = LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY;
            if(dci_1a->mcs >= 10 && dci_1a->mcs <= 15) {
                dci_1a->mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
            }
            else if(dci_1a->mcs >= 15 && dci_1a->mcs <= 27) {
                dci_1a->mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
            }
            dci_1a->N_codewords = 1;
            dci_1a->rnti = rnti;

            if(27 > dci_1a->mcs) {
                if(log_msg_enabled) {
                    fprintf(stderr, "    dci_1a->mcs = %d, dci_1a->L_crbs[0] = %d, dci_1a->riv/N_rb_dl + 1 = %d\n", dci_1a->mcs, dci_1a->L_crbs[0], dci_1a->riv/N_rb_dl + 1);
                }                
                dci_1a->tbs = TBS_71721[dci_1a->mcs][N_prb_1a - 1];
            }
            else {

                if(log_msg_enabled) {
                    fprintf(stderr, "--------------------------------\n");
                    fprintf(stderr, "    RESULT : Invalid Contents\n");
                }

                return LIBLTE_ERROR_INVALID_CONTENTS;
            }

            // Copy dci_1a to alloc
            alloc->N_prb = dci_1a->riv/N_rb_dl + 1;
            alloc->mcs = dci_1a->mcs;
            alloc->rv_idx = dci_1a->redundancy_version;

            // Convert allocation into array of prbs
            for(i = 0; i < alloc->N_prb; i++) {
                alloc->prb[0][i] = dci_1a->RB_start[0] + i;
                alloc->prb[1][i] = dci_1a->RB_start[0] + i;
            }
  
            // Fill in the allocation structure 3GPP TS 36.213 v10.3.0 section 7.1.7
            alloc->mod_type = dci_1a->mod_type;
            alloc->pre_coder_type = dci_1a->pre_coder_type;
            alloc->tx_mode = dci_1a->tx_mode;
            alloc->N_codewords = 1;
            alloc->tbs = dci_1a->tbs;
            alloc->rnti = dci_1a->rnti;
            alloc->dci_format = LTE_DCI_FORMAT_1A;

            if(log_msg_enabled) {

                fprintf(stderr, "--------------------------------\n");
                if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
                    fprintf(stderr, "WARNING: Not handling carrier indicator : ca_ind = %d\n", dci_1a->ca_ind);
                }
                fprintf(stderr, "    Check DCI 0/1A flag 3GPP TS 36.212 v10.1.0 section 5.3.3.1.3\n");
                fprintf(stderr, "        dci_0_1a_flag = %d\n", dci_1a->dci_0_1a_flag);
                fprintf(stderr, "    rnti = 0x%04x (%d)\n", rnti, rnti);
                fprintf(stderr, "\n");
                fprintf(stderr, "Determine if RIV uses local or distributed VRBs\n");
                fprintf(stderr, "    loc_or_dist = %d\n", dci_1a->loc_or_dist);
                fprintf(stderr, "\n");
                fprintf(stderr, "Find the RIV that was sent 3GPP TS 36.213 v10.3.0 section 7.1.6.3\n");
                fprintf(stderr, "    RIV_length = %d\n", dci_1a->riv_length);
                fprintf(stderr, "    RIV = %d\n", dci_1a->riv);
                fprintf(stderr, "    N_rb_dl / 2 = %d\n", N_rb_dl/2);
                fprintf(stderr, "    RB_start[0] = %d, L_crbs[0] = %d\n", dci_1a->RB_start[0], dci_1a->L_crbs[0]);
                fprintf(stderr, "    RB_start[1] = %d, L_crbs[1] = %d\n", dci_1a->RB_start[1], dci_1a->L_crbs[1]);
                fprintf(stderr, "\n");
                fprintf(stderr, "Extract the rest of the fields\n");
                fprintf(stderr, "    mcs = %d\n", dci_1a->mcs);
                fprintf(stderr, "    harq_process = %d\n", dci_1a->harq_process);
                fprintf(stderr, "    new_data_ind = %d\n", dci_1a->ndi);
                fprintf(stderr, "    redundancy version = %d\n", dci_1a->redundancy_version);
                fprintf(stderr, "    tpc  = %d\n", dci_1a->tpc);
                fprintf(stderr, "\n");
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    mod_type = %d (%s)\n", dci_1a->mod_type, liblte_phy_modulation_type_text[dci_1a->mod_type]);
                fprintf(stderr, "    tx_mode = %d\n", dci_1a->tx_mode);
                fprintf(stderr, "    N_codewords = %d\n", dci_1a->N_codewords);
                fprintf(stderr, "    Transport block size = %d\n", dci_1a->tbs);
            }

            return LIBLTE_SUCCESS;

        }
        else {
            if(log_msg_enabled) {
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    rnti = 0x%04x (%d)\n", rnti, rnti);
                fprintf(stderr, "    RESULT : Not Implemented 'Distributed Allocation VRB' yet in DCI Format 1A !!! \n");
            }
            return LIBLTE_ERROR_NOT_DEFINED;
        }
    }
    else {
        // Format 1A is used for random access procedure initiated by a PDCCH order only if format 1A CRC is scrambled with C-RNTI and
        // all the remaining fields are set as follows:
        if(dci_1a->loc_or_dist == DCI_VRB_TYPE_LOCALIZED) {
            // Resource block assignment
            dci_1a->riv_length = (int32_t)ceilf(logf(N_rb_dl*(N_rb_dl+1)/2)/logf(2));
            dci_1a->riv = liblte_bits_2_value(&dci_bits, dci_1a->riv_length);

            // Preamble Index
            dci_1a->preamble_index = liblte_bits_2_value(&dci_bits, 6);
            // PRACH Mask Index
            dci_1a->prach_mask_index = liblte_bits_2_value(&dci_bits, 4);      

            int32_t remain_size = N_in_bits - (int32_t)(dci_bits - in_bits);
            int32_t remain_bits = liblte_bits_2_value(&dci_bits, remain_size);

            // Resource block assignment – ceil(log2(N_rb_dl * (N_rb_dl+1) / 2)) bits, where all bits shall be set to 1
            // All the remaining bits in format 1A for compact scheduling assignment of one PDSCH codeword are set to zero
            if(dci_1a->riv != ((1 << dci_1a->riv_length) - 1) || remain_bits != 0) {
                if(log_msg_enabled) {
                    fprintf(stderr, "--------------------------------\n");
                    fprintf(stderr, "    RIV = 0x%04X (%d)\n", dci_1a->riv, dci_1a->riv);
                    fprintf(stderr, "    remain_bits = 0x%04X\n", remain_bits);
                    fprintf(stderr, "    RESULT : Invalid Contents in DCI Format 1A\n");
                }                
                return LIBLTE_ERROR_INVALID_CONTENTS;
            }            

            if(log_msg_enabled) {
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    RIV_length = %d\n", dci_1a->riv_length);
                fprintf(stderr, "    RIV = 0x%04X (%d)\n", dci_1a->riv, dci_1a->riv);
                fprintf(stderr, "    preamble_index = 0x%04X (%d)\n", dci_1a->preamble_index, dci_1a->preamble_index);
                fprintf(stderr, "    prach_mask_index = 0x%04X (%d)\n", dci_1a->prach_mask_index, dci_1a->prach_mask_index);
            }

            return LIBLTE_SUCCESS;
        }
        else {

            if(log_msg_enabled) {
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    RESULT : Invalid Contents\n");
            }

            return LIBLTE_ERROR_INVALID_CONTENTS;
        }
    }

}


int32_t dci_2_unpack_fdd
(
    uint8                           *in_bits,
    uint32                           N_in_bits,
    int32_t                          ca_presence,
    uint16                           rnti,
    uint32                           N_rb_dl,
    uint8                            N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,    
    struct dci_format_2_fdd_s       *dci_2
)
{

    int32_t     i;
    int32_t     j;
    uint8_t     *dci_bits = in_bits;
    int32_t     ra_len;
    int32_t     type1_only_field_len;
    int32_t     P;
    uint32_t    bit_mask;
    int32_t     k;
    int32_t     decision_p;
    int32_t     p;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "dci_2_unpack\n");
        fprintf(stderr, "    Unpacks all of the fields from the Downlink Control Information format 2\n");
        fprintf(stderr, "    3GPP TS 36.212 v10.1.0 section 5.3.3.1.2\n");
        fprintf(stderr, "    N_in_bits = %d\n", N_in_bits);
        fprintf(stderr, "    N_rb_dl = %d\n", N_rb_dl);
        fprintf(stderr, "    N_ant = %d\n", N_ant);
    }

    dci_2->prb_allocated = 0;
    for(i = 0; i < N_rb_dl; i++) {
        dci_2->prb[i] = 0; 
    } 

    // Carrier indicator
    if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
        dci_2->ca_ind = liblte_bits_2_value(&dci_bits, 3);
    }

    // Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.212 [3]
    if(N_rb_dl < 10) {
        dci_2->resource_alloc_type = 0;
    }
    else {
        dci_2->resource_alloc_type = liblte_bits_2_value(&dci_bits, 1);
    }

    ra_len = ra_type0_len(N_rb_dl);
    P = RBG_size(N_rb_dl);

    dci_2->delta_shift = 0;
    type1_only_field_len = 0;
    if(dci_2->resource_alloc_type == 1) {
        
        dci_2->N_rb_type1 = (N_rb_dl + (P - 1)) / P ;
        if(N_rb_dl < 50) {
            dci_2->subset = liblte_bits_2_value(&dci_bits, 1);
            type1_only_field_len = 2;
        }
        else {
            dci_2->subset = liblte_bits_2_value(&dci_bits, 2);
            type1_only_field_len = 3;
        }
        
        dci_2->shift = liblte_bits_2_value(&dci_bits, 1);
        ra_len = ra_len - type1_only_field_len;
        
        if(dci_2->subset>= P) {
            if(log_msg_enabled) {
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    RESULT : Invalid Contents\n");
            }
            return LIBLTE_ERROR_INVALID_CONTENTS;    
        }         

        dci_2->N_rb_type1 = (N_rb_dl + (P - 1)) / P - type1_only_field_len;
        dci_2->N_rb_rbg_subset = ra_n_rb_rbg_subset(N_rb_dl, dci_2->subset);
        if(dci_2->shift) {        
            dci_2->delta_shift = dci_2->N_rb_rbg_subset - dci_2->N_rb_type1;
        }

    }

    dci_2->resource_block_assignment = liblte_bits_2_value(&dci_bits, ra_len);

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    dci_2->tpc = liblte_bits_2_value(&dci_bits, 2);

    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    dci_2->harq_process = liblte_bits_2_value(&dci_bits, 3);

    // Transport block to codeword swap flag – 1 bit
    dci_2->tb_codeword_swap_flag = liblte_bits_2_value(&dci_bits, 1);

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    dci_2->tb1_mcs = liblte_bits_2_value(&dci_bits, 5);

    // New data indicator – 1 bit
    dci_2->tb1_ndi = liblte_bits_2_value(&dci_bits, 1);

    // Redundancy version – 2 bits
    dci_2->tb1_redundancy_version = liblte_bits_2_value(&dci_bits, 2);

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    dci_2->tb2_mcs = liblte_bits_2_value(&dci_bits, 5);

    // New data indicator – 1 bit
    dci_2->tb2_ndi = liblte_bits_2_value(&dci_bits, 1);

    // Redundancy version – 2 bits
    dci_2->tb2_redundancy_version = liblte_bits_2_value(&dci_bits, 2);


    // Table 7.1.7.1-1: Modulation and TBS index table for PDSCH
    // Qm
    dci_2->tb1_i_tbs = 0;
    
    if(dci_2->tb1_mcs <= 9) {
        dci_2->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        dci_2->tb1_i_tbs = dci_2->tb1_mcs;
    }
    else if(dci_2->tb1_mcs <= 16) {
        dci_2->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
        dci_2->tb1_i_tbs = dci_2->tb1_mcs - 1;
    }
    else if(dci_2->tb1_mcs <= 28) {
        dci_2->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
        dci_2->tb1_i_tbs = dci_2->tb1_mcs - 2;
    }    
    else if(dci_2->tb1_mcs == 29) {
        dci_2->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    }
    else if(dci_2->tb1_mcs == 30) {
        dci_2->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
    }
    else {
        dci_2->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
    }

    dci_2->tb2_i_tbs = 0;
    if(dci_2->tb2_mcs <= 9) {
        dci_2->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        dci_2->tb2_i_tbs = dci_2->tb2_mcs;
    }
    else if(dci_2->tb2_mcs <= 16) {
        dci_2->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
        dci_2->tb2_i_tbs = dci_2->tb2_mcs - 1;
    }
    else if(dci_2->tb2_mcs <= 28) {
        dci_2->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
        dci_2->tb2_i_tbs = dci_2->tb2_mcs - 2;
    }    
    else if(dci_2->tb2_mcs == 29) {
        dci_2->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    }
    else if(dci_2->tb2_mcs == 30) {
        dci_2->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
    }
    else {
        dci_2->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
    }

    bit_mask = 1 << (ra_len - 1);
    if(dci_2->resource_alloc_type == 0) {
        for(i = 0; i < ra_len; i++) {
            if(dci_2->resource_block_assignment & bit_mask) {
                for(j = 0; j < P && (i * P + j) < N_rb_dl; j++) {
                    dci_2->prb[i * P + j] = 1;
                }
                dci_2->prb_allocated = dci_2->prb_allocated + j;
            }
            bit_mask = bit_mask >> 1;
        } 
    }
    else {                
        
        // 7.1.6.2 Resource allocation type 1 in TS 36.213         
        for(i = 0; i < ra_len; i++) {
            if(dci_2->resource_block_assignment & bit_mask) {                    
                dci_2->prb[((i + dci_2->delta_shift) / P) * P * P +  dci_2->subset * P + (i + dci_2->delta_shift) % P] = 1;
                dci_2->prb_allocated = dci_2->prb_allocated + 1;
            }
            bit_mask = bit_mask >> 1;
        }
    }

    if(dci_2->tb1_i_tbs < 27 &&  dci_2->prb_allocated < 110) {
        dci_2->tb1_tbs = TBS_71721[dci_2->tb1_i_tbs][dci_2->prb_allocated-1];
    }

    if(dci_2->tb2_i_tbs < 27 &&  dci_2->prb_allocated < 110) {
        dci_2->tb2_tbs = TBS_71721[dci_2->tb2_i_tbs][dci_2->prb_allocated-1];
    }

    dci_2->rnti = rnti;

    if(log_msg_enabled) {

        fprintf(stderr, "--------------------------------\n");
        if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
            fprintf(stderr, "WARNING: Not handling carrier indicator : ca_ind : %d\n", dci_2->ca_ind);
        }

        fprintf(stderr, "    Resource allocation type : %d\n", dci_2->resource_alloc_type);
        fprintf(stderr, "    Resource block group size : %d\n", P);
        if(dci_2->resource_alloc_type == 1) {
            fprintf(stderr, "    subset : %d\n", dci_2->subset);
            if(dci_2->shift) {
                fprintf(stderr, "    RBG subset PRBs shifted : Yes (end at highest PRB)\n");
            }
            else {
                fprintf(stderr, "    RBG subset PRBs shifted : No (start at lowest PRB)\n");
            }
        }

        fprintf(stderr, "    Resource block assignment = %d (0x%04x)\n", dci_2->resource_block_assignment, dci_2->resource_block_assignment);
        fprintf(stderr, "       %d PRBs was allocated.\n", dci_2->prb_allocated);
        fprintf(stderr, "       Value in binary :");
        print_bits(dci_2->prb, N_rb_dl);

        fprintf(stderr, "    TPC command for PUCCH = %d\n", dci_2->tpc);
        fprintf(stderr, "    HARQ process number = %d\n", dci_2->harq_process);
        
        if(dci_2->tb_codeword_swap_flag) {
            fprintf(stderr, "    Codewords are swapped : Yes\n");
        }
        else {
            fprintf(stderr, "    Codewords are swapped : No\n");
        }

        fprintf(stderr, "    Transport block 1\n");
        fprintf(stderr, "        Modulation and coding scheme index = %d\n", dci_2->tb1_mcs);
        fprintf(stderr, "        mod_type = %d (%s)\n", dci_2->tb1_mod_type, liblte_phy_modulation_type_text[dci_2->tb1_mod_type]);
        fprintf(stderr, "        Transport block size = %d\n", dci_2->tb1_tbs);
        fprintf(stderr, "        New data indicator = %d\n", dci_2->tb1_ndi);
        fprintf(stderr, "        Redundancy version = %d\n", dci_2->tb1_redundancy_version);

        fprintf(stderr, "    Transport block 2\n");
        fprintf(stderr, "        Modulation and coding scheme index = %d\n", dci_2->tb2_mcs);
        fprintf(stderr, "        mod_type = %d (%s)\n", dci_2->tb2_mod_type, liblte_phy_modulation_type_text[dci_2->tb2_mod_type]);
        fprintf(stderr, "        Transport block size = %d\n", dci_2->tb2_tbs);
        fprintf(stderr, "        New data indicator = %d\n", dci_2->tb2_ndi);
        fprintf(stderr, "        Redundancy version = %d\n", dci_2->tb2_redundancy_version);

    }

    return LIBLTE_SUCCESS;

}


int32_t dci_2a_unpack_fdd
(
    uint8                           *in_bits,
    uint32                           N_in_bits,
    int32_t                          ca_presence,
    uint16                           rnti,
    uint32                           N_rb_dl,
    uint8                            N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,    
    struct dci_format_2a_fdd_s       *dci_2a
)
{


    int32_t     i;
    int32_t     j;
    uint8_t     *dci_bits = in_bits;
    int32_t     ra_len;
    int32_t     type1_only_field_len;
    int32_t     P;
    uint32_t    bit_mask;
    int32_t     k;
    int32_t     decision_p;
    int32_t     p;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "dci_2a_unpack\n");
        fprintf(stderr, "    Unpacks all of the fields from the Downlink Control Information format 2A\n");
        fprintf(stderr, "    3GPP TS 36.212 v10.1.0 section 5.3.3.1.2\n");
        fprintf(stderr, "    N_in_bits = %d\n", N_in_bits);
        fprintf(stderr, "    N_rb_dl = %d\n", N_rb_dl);
        fprintf(stderr, "    N_ant = %d\n", N_ant);
    }

    dci_2a->prb_allocated = 0;
    for(i = 0; i < N_rb_dl; i++) {
        dci_2a->prb[i] = 0; 
    } 

    // Carrier indicator
    if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
        dci_2a->ca_ind = liblte_bits_2_value(&dci_bits, 3);
    }

    // Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.212 [3]
    if(N_rb_dl < 10) {
        dci_2a->resource_alloc_type = 0;
    }
    else {
        dci_2a->resource_alloc_type = liblte_bits_2_value(&dci_bits, 1);
    }

    ra_len = ra_type0_len(N_rb_dl);
    P = RBG_size(N_rb_dl);

    dci_2a->delta_shift = 0;
    type1_only_field_len = 0;
    if(dci_2a->resource_alloc_type == 1) {
        
        dci_2a->N_rb_type1 = (N_rb_dl + (P - 1)) / P ;
        if(N_rb_dl < 50) {
            dci_2a->subset = liblte_bits_2_value(&dci_bits, 1);
            type1_only_field_len = 2;
        }
        else {
            dci_2a->subset = liblte_bits_2_value(&dci_bits, 2);
            type1_only_field_len = 3;
        }
        
        dci_2a->shift = liblte_bits_2_value(&dci_bits, 1);
        ra_len = ra_len - type1_only_field_len;
        
        if(dci_2a->subset >= P) {
            if(log_msg_enabled) {
                fprintf(stderr, "--------------------------------\n");
                fprintf(stderr, "    RESULT : Invalid Contents\n");
            }
            return LIBLTE_ERROR_INVALID_CONTENTS;    
        }         

        dci_2a->N_rb_type1 = (N_rb_dl + (P - 1)) / P - type1_only_field_len;
        dci_2a->N_rb_rbg_subset = ra_n_rb_rbg_subset(N_rb_dl, dci_2a->subset);
        if(dci_2a->shift) {        
            dci_2a->delta_shift = dci_2a->N_rb_rbg_subset - dci_2a->N_rb_type1;
        }

    }

    dci_2a->resource_block_assignment = liblte_bits_2_value(&dci_bits, ra_len);

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    dci_2a->tpc = liblte_bits_2_value(&dci_bits, 2);

    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    dci_2a->harq_process = liblte_bits_2_value(&dci_bits, 3);

    // Transport block to codeword swap flag – 1 bit
    dci_2a->tb_codeword_swap_flag = liblte_bits_2_value(&dci_bits, 1);

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    dci_2a->tb1_mcs = liblte_bits_2_value(&dci_bits, 5);

    // New data indicator – 1 bit
    dci_2a->tb1_ndi = liblte_bits_2_value(&dci_bits, 1);

    // Redundancy version – 2 bits
    dci_2a->tb1_redundancy_version = liblte_bits_2_value(&dci_bits, 2);

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    dci_2a->tb2_mcs = liblte_bits_2_value(&dci_bits, 5);

    // New data indicator – 1 bit
    dci_2a->tb2_ndi = liblte_bits_2_value(&dci_bits, 1);

    // Redundancy version – 2 bits
    dci_2a->tb2_redundancy_version = liblte_bits_2_value(&dci_bits, 2);

    // Precoding information – number of bits as specified in Table 5.3.3.1.5A-1 in 3GPP TS 36.212
    if(N_ant == 4) {
        dci_2a->precoding_info = liblte_bits_2_value(&dci_bits, 2);
    }

    // Table 7.1.7.1-1: Modulation and TBS index table for PDSCH
    // Qm
    dci_2a->tb1_i_tbs = 0;
    
    if(dci_2a->tb1_mcs <= 9) {
        dci_2a->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        dci_2a->tb1_i_tbs = dci_2a->tb1_mcs;
    }
    else if(dci_2a->tb1_mcs <= 16) {
        dci_2a->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
        dci_2a->tb1_i_tbs = dci_2a->tb1_mcs - 1;
    }
    else if(dci_2a->tb1_mcs <= 28) {
        dci_2a->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
        dci_2a->tb1_i_tbs = dci_2a->tb1_mcs - 2;
    }    
    else if(dci_2a->tb1_mcs == 29) {
        dci_2a->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    }
    else if(dci_2a->tb1_mcs == 30) {
        dci_2a->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
    }
    else {
        dci_2a->tb1_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
    }

    dci_2a->tb2_i_tbs = 0;
    if(dci_2a->tb2_mcs <= 9) {
        dci_2a->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
        dci_2a->tb2_i_tbs = dci_2a->tb2_mcs;
    }
    else if(dci_2a->tb2_mcs <= 16) {
        dci_2a->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
        dci_2a->tb2_i_tbs = dci_2a->tb2_mcs - 1;
    }
    else if(dci_2a->tb2_mcs <= 28) {
        dci_2a->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
        dci_2a->tb2_i_tbs = dci_2a->tb2_mcs - 2;
    }    
    else if(dci_2a->tb2_mcs == 29) {
        dci_2a->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_QPSK;
    }
    else if(dci_2a->tb2_mcs == 30) {
        dci_2a->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_16QAM;
    }
    else {
        dci_2a->tb2_mod_type = LIBLTE_PHY_MODULATION_TYPE_64QAM;
    }

    bit_mask = 1 << (ra_len - 1);
    if(dci_2a->resource_alloc_type == 0) {
        for(i = 0; i < ra_len; i++) {
            if(dci_2a->resource_block_assignment & bit_mask) {
                for(j = 0; j < P && (i * P + j) < N_rb_dl; j++) {
                    dci_2a->prb[i * P + j] = 1;
                }
                dci_2a->prb_allocated = dci_2a->prb_allocated + j;
            }
            bit_mask = bit_mask >> 1;
        } 
    }
    else {                
        
        // 7.1.6.2 Resource allocation type 1 in TS 36.213         
        for(i = 0; i < ra_len; i++) {
            if(dci_2a->resource_block_assignment & bit_mask) {                    
                dci_2a->prb[((i + dci_2a->delta_shift) / P) * P * P +  dci_2a->subset * P + (i + dci_2a->delta_shift) % P] = 1;
                dci_2a->prb_allocated = dci_2a->prb_allocated + 1;
            }
            bit_mask = bit_mask >> 1;
        }
    }

    if(dci_2a->tb1_i_tbs < 27 &&  dci_2a->prb_allocated < 110) {
        dci_2a->tb1_tbs = TBS_71721[dci_2a->tb1_i_tbs][dci_2a->prb_allocated-1];
    }

    if(dci_2a->tb2_i_tbs < 27 &&  dci_2a->prb_allocated < 110) {
        dci_2a->tb2_tbs = TBS_71721[dci_2a->tb2_i_tbs][dci_2a->prb_allocated-1];
    }

    dci_2a->rnti = rnti;



    if(log_msg_enabled) {

        fprintf(stderr, "--------------------------------\n");
        if(LIBLTE_PHY_DCI_CA_PRESENT == ca_presence) {
            fprintf(stderr, "WARNING: Not handling carrier indicator : ca_ind : %d\n", dci_2a->ca_ind);
        }

        fprintf(stderr, "    Resource allocation type : %d\n", dci_2a->resource_alloc_type);
        fprintf(stderr, "    Resource block group size : %d\n", P);
        if(dci_2a->resource_alloc_type == 1) {
            fprintf(stderr, "    subset : %d\n", dci_2a->subset);
            if(dci_2a->shift) {
                fprintf(stderr, "    RBG subset PRBs shifted : Yes (end at highest PRB)\n");
            }
            else {
                fprintf(stderr, "    RBG subset PRBs shifted : No (start at lowest PRB)\n");
            }
        }

        fprintf(stderr, "    Resource block assignment = %d (0x%04x)\n", dci_2a->resource_block_assignment, dci_2a->resource_block_assignment);
        fprintf(stderr, "       %d PRBs was allocated.\n", dci_2a->prb_allocated);
        fprintf(stderr, "       Value in binary :");
        print_bits(dci_2a->prb, N_rb_dl);

        fprintf(stderr, "    TPC command for PUCCH = %d\n", dci_2a->tpc);
        fprintf(stderr, "    HARQ process number = %d\n", dci_2a->harq_process);
        
        if(dci_2a->tb_codeword_swap_flag) {
            fprintf(stderr, "    Codewords are swapped : Yes\n");
        }
        else {
            fprintf(stderr, "    Codewords are swapped : No\n");
        }

        fprintf(stderr, "    Transport block 1\n");
        fprintf(stderr, "        Modulation and coding scheme index = %d\n", dci_2a->tb1_mcs);
        fprintf(stderr, "        mod_type = %d (%s)\n", dci_2a->tb1_mod_type, liblte_phy_modulation_type_text[dci_2a->tb1_mod_type]);
        fprintf(stderr, "        Transport block size = %d\n", dci_2a->tb1_tbs);
        fprintf(stderr, "        New data indicator = %d\n", dci_2a->tb1_ndi);
        fprintf(stderr, "        Redundancy version = %d\n", dci_2a->tb1_redundancy_version);        

        fprintf(stderr, "    Transport block 2\n");
        fprintf(stderr, "        Modulation and coding scheme index = %d\n", dci_2a->tb2_mcs);
        fprintf(stderr, "        mod_type = %d (%s)\n", dci_2a->tb2_mod_type, liblte_phy_modulation_type_text[dci_2a->tb2_mod_type]);
        fprintf(stderr, "        Transport block size = %d\n", dci_2a->tb2_tbs);
        fprintf(stderr, "        New data indicator = %d\n", dci_2a->tb2_ndi);
        fprintf(stderr, "        Redundancy version = %d\n", dci_2a->tb2_redundancy_version);
        
        if(N_ant == 4) {
        
        }

    }

    return LIBLTE_SUCCESS;

}


int32_t liblte_phy_get_pdcch_descrambled_bits
(
    LIBLTE_PHY_STRUCT              *phy_struct,
    uint8                           N_ant,
    uint32_t                        M_ap_symb,
    uint32_t                        *pdcch_c,
    uint32_t                        *N_bits
)
{
    uint32_t i;
    uint32_t M_layer_symb = 0;
    uint32_t M_symb = 0;

    pre_decoder_and_matched_filter_dl(phy_struct->pdcch_y_est_re,
                                      phy_struct->pdcch_y_est_im,
                                      phy_struct->pdcch_c_est_re[0],
                                      phy_struct->pdcch_c_est_im[0],
                                      576,
                                      M_ap_symb,
                                      N_ant,
                                      LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY,
                                      phy_struct->pdcch_x_re,
                                      phy_struct->pdcch_x_im,
                                      &M_layer_symb);

    layer_demapper_dl(phy_struct->pdcch_x_re,
                      phy_struct->pdcch_x_im,
                      M_layer_symb,
                      N_ant,
                      1,
                      LIBLTE_PHY_PRE_CODER_TYPE_TX_DIVERSITY,
                      phy_struct->pdcch_d_re,
                      phy_struct->pdcch_d_im,
                      &M_symb);

    modulation_demapper(phy_struct->pdcch_d_re,
                        phy_struct->pdcch_d_im,
                        M_symb,
                        LIBLTE_PHY_MODULATION_TYPE_QPSK,
                        phy_struct->pdcch_soft_bits,
                        N_bits);

    for(i = 0; i < *N_bits; i++) {
        phy_struct->pdcch_descramb_bits[i] = (float)phy_struct->pdcch_soft_bits[i]*(1-2*(float)pdcch_c[i]);
    }

    return LIBLTE_SUCCESS;


}


int32_t liblte_phy_get_pdcch_y_est
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             agg_lvl,
    int32_t             start_cce_n
)
{
    int32_t j;
    int32_t k;
    int32_t p;
    int32_t idx;

    idx = 0;
    for(j = 0; j < agg_lvl; j++) {
        for(k = 0;  k < LIBLTE_PHY_PDCCH_N_RE_CCE; k++) {
            phy_struct->pdcch_y_est_re[idx] = phy_struct->pdcch_cce_y_est_re[start_cce_n+j][k];
            phy_struct->pdcch_y_est_im[idx] = phy_struct->pdcch_cce_y_est_im[start_cce_n+j][k];
            idx++;
        }
    }

    return LIBLTE_SUCCESS;

}


int32_t liblte_phy_get_pdcch_c_est
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             agg_lvl,
    int32_t             start_cce_n
)
{
    int32_t j;
    int32_t k;
    int32_t p;
    int32_t idx;

    idx = 0;
    for(j = 0; j < agg_lvl; j++) {
        for(k = 0;  k < LIBLTE_PHY_PDCCH_N_RE_CCE; k++) {
            for(p = 0; p < N_ant; p++) {
                phy_struct->pdcch_c_est_re[p][idx] = phy_struct->pdcch_cce_c_est_re[p][start_cce_n+j][k];
                phy_struct->pdcch_c_est_im[p][idx] = phy_struct->pdcch_cce_c_est_im[p][start_cce_n+j][k];
            }
            idx++;
        }
    }

    return LIBLTE_SUCCESS;

}



void rate_unmatch_conv_n
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *e_bits,
    uint32             N_e_bits,
    uint32             N_c_bits,
    float             *d_bits,
    uint32            *N_d_bits
)
{
    uint32 C_cc_sb = 32; // Step 1: Assign C_cc_sb to 32
    uint32 R_cc_sb;
    uint32 w_idx = 0;
    uint32 d_idx;
    uint32 N_dummy;
    uint32 idx;
    uint32 K_pi;
    uint32 K_w;
    uint32 i;
    uint32 j;
    uint32 k;
    uint32 x;
    int32_t log_msg_enabled = 0;

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "rate_unmatch_conv\n");
        fprintf(stderr, "    Rate unmatches convolutionally encoded data\n");
        fprintf(stderr, "    3GPP TS 36.212 v10.1.0 section 5.1.4.2\n");
    }

    // In order to undo bit collection, selection, and transmission
    // a dummy block must be sub-block interleaved to determine
    // where NULL bits are to be inserted
    // Sub-block interleaving
    // Step 2: Determine the number of rows
    R_cc_sb = 0;
    while(N_c_bits > (C_cc_sb*R_cc_sb)) {
        R_cc_sb++;
    }

    if(log_msg_enabled) {
        fprintf(stderr, "N_e_bits = %d, N_c_bits = %d, C_cc_sb = %d, R_cc_sb = %d\n", N_e_bits, N_c_bits, C_cc_sb, R_cc_sb);
    }

    // Steps 3, 4, and 5
    for(x = 0; x < 3; x++) {
        // Step 3: Pack data into matrix and pad with dummy
        if(N_c_bits < (C_cc_sb*R_cc_sb)) {
            N_dummy = C_cc_sb*R_cc_sb - N_c_bits;
        }
        else{
            N_dummy = 0;
        }

        for(i = 0; i < N_dummy; i++) {
            phy_struct->ruc_tmp[i] = RX_NULL_BIT;
        }
        for(i = N_dummy; i < C_cc_sb*R_cc_sb; i++) {
            phy_struct->ruc_tmp[i] = 0;
        }

        idx = 0;
        for(i = 0; i < R_cc_sb; i++) {
            for(j = 0; j < C_cc_sb; j++) {
                phy_struct->ruc_sb_mat[i][j] = phy_struct->ruc_tmp[idx++];
            }
        }

        // Step 4: Inter-column permutation
        for(i = 0; i < R_cc_sb; i++) {
            for(j = 0; j < C_cc_sb; j++) {
                phy_struct->ruc_sb_perm_mat[i][j] = phy_struct->ruc_sb_mat[i][IC_PERM_CC[j]];
            }
        }

        // Step 5: Read out the bits
        for(j = 0; j < C_cc_sb; j++) {
            for(i = 0; i < R_cc_sb; i++) {
                phy_struct->ruc_w_dum[w_idx] = phy_struct->ruc_sb_perm_mat[i][j];
                phy_struct->ruc_w[w_idx]     = RX_NULL_BIT;
                w_idx++;
            }
        }
    }

    // Undo bit collection, selection, and transmission by
    // recreating the circular buffer
    K_pi = R_cc_sb*C_cc_sb;
    K_w  = 3*K_pi;
    k    = 0;
    j    = 0;
    while(k < N_e_bits) {
        if(phy_struct->ruc_w_dum[j % K_w] != RX_NULL_BIT) {
            // Soft combine the inputs
            if(phy_struct->ruc_w[j % K_w] == RX_NULL_BIT) {
                phy_struct->ruc_w[j % K_w] = e_bits[k];
            }
            else if(e_bits[k] != RX_NULL_BIT) {
                phy_struct->ruc_w[j % K_w] += e_bits[k];
            }
            k++;
        }
        j++;
    }

    // Recreate the sub-block interleaver output
    for(i = 0; i < K_pi; i++) {
        phy_struct->ruc_v[0][i] = phy_struct->ruc_w[i];
        phy_struct->ruc_v[1][i] = phy_struct->ruc_w[i+K_pi];
        phy_struct->ruc_v[2][i] = phy_struct->ruc_w[i+2*K_pi];
    }

    // Sub-block deinterleaving
    // Steps 5, 4, and 3
    for(x = 0; x < 3; x++) {
        // Step 5: Load the permuted matrix
        idx = 0;
        for(j = 0; j < C_cc_sb; j++) {
            for(i = 0; i < R_cc_sb; i++) {
                phy_struct->ruc_sb_perm_mat[i][j] = phy_struct->ruc_v[x][idx++];
            }
        }

        // Step 4: Undo permutation
        for(i = 0; i < R_cc_sb; i++) {
            for(j = 0; j < C_cc_sb; j++) {
                phy_struct->ruc_sb_mat[i][IC_PERM_CC[j]] = phy_struct->ruc_sb_perm_mat[i][j];
            }
        }

        // Step 3: Unpack the data and remove dummy
        if(N_c_bits < (C_cc_sb*R_cc_sb)) {
            N_dummy = C_cc_sb*R_cc_sb - N_c_bits;
        }
        else{
            N_dummy = 0;
        }

        idx = 0;
        for(i = 0; i < R_cc_sb; i++) {
            for(j = 0; j < C_cc_sb; j++) {
                phy_struct->ruc_tmp[idx++] = phy_struct->ruc_sb_mat[i][j];
            }
        }

        d_idx = 0;
        for(i = N_dummy; i < C_cc_sb*R_cc_sb; i++) {
            d_bits[d_idx*3+x] = phy_struct->ruc_tmp[i];
            d_idx++;
        }

    }

    *N_d_bits = d_idx*3;

    if(log_msg_enabled) {
        fprintf(stderr, "*N_d_bits = %d\n", *N_d_bits);
    }

}

LIBLTE_ERROR_ENUM dci_channel_decode_n
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *in_bits,
    uint32             N_in_bits,
    uint8              ue_ant,
    uint8             *out_bits,
    uint32             N_out_bits,
    uint16            *rnti_found
)
{
    LIBLTE_ERROR_ENUM  err = LIBLTE_ERROR_INVALID_CRC;
    uint32             i;
    uint32             j;
    uint32             N_d_bits;
    uint32             N_c_bits;
    uint32             ber;
    uint32             g[3] = {0133, 0171, 0165}; // Numbers are in octal
    uint16             rnti;
    uint8              x_rnti_bits[16];
    uint8              x_as_bits[16];
    uint8             *a_bits;
    uint8             *p_bits;
    uint8              calc_p_bits[16];

    uint16_t          expect_rnti;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);

    // Construct UE antenna mask
    memset(x_as_bits, 0, sizeof(uint8)*16);
    if(ue_ant == 1) {
        x_as_bits[15] = 1;
    }

    // Rate unmatch to get the d_bits
    rate_unmatch_conv_n(phy_struct,
                      in_bits,
                      N_in_bits,
                      N_out_bits+16,
                      phy_struct->dci_rx_d_bits,
                      &N_d_bits);

    // Viterbi decode the d_bits to get the c_bits
    viterbi_decode(phy_struct,
                   phy_struct->dci_rx_d_bits,
                   N_d_bits,
                   7,
                   3,
                   g,
                   phy_struct->dci_c_bits,
                   &N_c_bits);

    phy_struct->N_c_bits = N_c_bits;

    // Recover a_bits and p_bits
    a_bits = &phy_struct->dci_c_bits[0];
    p_bits = &phy_struct->dci_c_bits[N_out_bits];

    // Calculate p_bits
    calc_crc(a_bits, N_out_bits, CRC16, calc_p_bits, 16);

    expect_rnti = 0;
    for(i = 0; i < 16; i++) {
        if(p_bits[i] ^ calc_p_bits[i] ^ x_as_bits[i]) {
            expect_rnti = (expect_rnti << 1) | 1;
        }
        else {
            expect_rnti = (expect_rnti << 1) | 0;
        }
    }

    *rnti_found = expect_rnti;
    for(i = 0; i < N_out_bits + 16; i++) {
        out_bits[i] = a_bits[i];
    }

    return(err);
}



int32_t decode_cce
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             L,
    int32_t             cce_index,
    int32_t             dci_len,
    uint16_t            *rnti_found
)
{

    uint32_t N_bits;

    *rnti_found = 0;

    liblte_phy_get_pdcch_y_est(phy_struct, N_ant, L, cce_index);
    liblte_phy_get_pdcch_c_est(phy_struct, N_ant, L, cce_index);
    liblte_phy_get_pdcch_descrambled_bits(phy_struct, N_ant, L * LIBLTE_PHY_PDCCH_N_RE_CCE,
        &phy_struct->pdcch_c[cce_index*LIBLTE_PHY_PDCCH_N_RE_CCE*2], &N_bits);

    dci_channel_decode_n(phy_struct,
                     phy_struct->pdcch_descramb_bits,
                     N_bits,
                     0,
                     phy_struct->pdcch_dci,
                     dci_len,
                     rnti_found);

    return 0;

}

int32_t check_cce_index(int32_t cce_index, int32_t rnti, int32_t L, int32_t Ncce, int32_t subframe_no)
{
    int32_t num_of_pdcch_candidates_tbl[] = { 6, 6, 2, 2 };
    uint32_t M_L;
    uint32_t m_prime;
    uint32_t A = 39827;
    uint32_t D = 65537;
    int32_t k_;
    uint32_t Y_k;
    int32_t m;
    int32_t search_cce_index;
    int32_t agg_lvl_idx;
    int32_t Ncce_div_L;
    int32_t log_msg_enabled = 1;

    Ncce_div_L = Ncce / L;

    for(agg_lvl_idx = 0; agg_lvl_idx < 4; agg_lvl_idx++) {
        if(L == (1 << agg_lvl_idx)) {
            break;
        }
    }

    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "    cce_index = %3d, rnti = 0x%04x (%d)\n", cce_index, rnti, rnti);
    }
    M_L = num_of_pdcch_candidates_tbl[agg_lvl_idx];
    if(log_msg_enabled) {
        fprintf(stderr, "    M_L = %d\n", M_L);
    }

    Y_k = rnti;
    if(log_msg_enabled) {
        fprintf(stderr, "    Find Yk\n");
        fprintf(stderr, "        Y(-1) = %5d\n", Y_k);
    }
    for(k_ = 0; k_ <= subframe_no; k_++) {
        Y_k = (A * Y_k) % D;
        if(log_msg_enabled) {
            fprintf(stderr, "        Y(%d)  = %5d\n", k_, Y_k);
        }
    }

    if(log_msg_enabled) {
        fprintf(stderr, "    Search cce_index \n");
    }

    for(m = 0; (m < M_L) && (m  < Ncce_div_L); m++) {
        m_prime = m;
        search_cce_index = L * ((Y_k + m_prime) % Ncce_div_L);
        fprintf(stderr, "        [%d] cce_index = %2d\n", m, search_cce_index);
    }

    for(m = 0; (m < M_L) && (m  < Ncce_div_L); m++) {
        m_prime = m;
        search_cce_index = L * ((Y_k + m_prime) % Ncce_div_L);
        if(search_cce_index == cce_index) {
            return 1;
        }
    }

    return 0;

}

int32_t check_common_search_space
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             agg_level_min,
    int32_t             agg_level_max,
    int32_t             dci_format,
    int32_t             dci_len
)
{
    int32_t i;
    int32_t j;
    int32_t L;
    int32_t Ncce_div_L;
    int32_t cce_start_index;
    uint16_t rnti;
    int32_t pdcch_cce_allocated;

    // uint32_t N_bits;
    // int32_t result;

    for(L = agg_level_min; L <= agg_level_max; L = L * 2) {
        Ncce_div_L = phy_struct->n_cce_pdcch / L;
        for(i = 0; (i < Ncce_div_L) && (i < N_CCE_OF_COMMON_SEARCH_SPACE / L); i++) {
            cce_start_index = i * L;
            for(j = 0; j < phy_struct->pdcch_cce_allocated; j++) {
                if(phy_struct->pdcch_cce_alloc_type[j] == 1) {
                    if((cce_start_index >= phy_struct->pdcch_cce_start_index[j]) &&
                        (cce_start_index < (phy_struct->pdcch_cce_start_index[j] + phy_struct->pdcch_cce_agg_level[pdcch_cce_allocated]))) {
                        break;
                    }
                }
            }
            if(j < phy_struct->pdcch_cce_allocated) {
                continue;
            }

            decode_cce(phy_struct, N_ant, L, cce_start_index, dci_len, &rnti);
            if(rnti == LIBLTE_MAC_SI_RNTI || rnti == LIBLTE_MAC_P_RNTI || (rnti >= LIBLTE_MAC_RA_RNTI_START && rnti <= LIBLTE_MAC_RA_RNTI_END)) {
                pdcch_cce_allocated = phy_struct->pdcch_cce_allocated;

                phy_struct->pdcch_cce_alloc_type[pdcch_cce_allocated] = 1;
                phy_struct->pdcch_cce_start_index[pdcch_cce_allocated] = cce_start_index;
                phy_struct->pdcch_cce_rnti[pdcch_cce_allocated] = rnti;
                phy_struct->pdcch_cce_agg_level[pdcch_cce_allocated] = L;
                phy_struct->pdcch_dci_format[pdcch_cce_allocated] = dci_format;
                phy_struct->pdcch_dci_len[pdcch_cce_allocated] = dci_len;

                phy_struct->pdcch_cce_allocated = pdcch_cce_allocated + 1;

            }
        }
    }

    return 0;

}

int32_t check_ue_specific_search_space
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             subframe_no,
    int32_t             agg_level_min,
    int32_t             agg_level_max,
    int32_t             dci_format,
    int32_t             dci_len
)
{
    int32_t i;
    int32_t j;
    int32_t L;
    int32_t Ncce_div_L;
    int32_t cce_start_index;
    // uint32_t N_bits;
    // int32_t result;
    uint16_t rnti;
    int32_t pdcch_cce_allocated;

    for(L = agg_level_min; L <= agg_level_max; L = L * 2) {

        if(L*LIBLTE_PHY_PDCCH_N_RE_CCE*2 < (dci_len + 16) * 3) {
            continue;
        }

        Ncce_div_L = phy_struct->n_cce_pdcch / L;
        for(i = 0; (i < Ncce_div_L) && (i < N_CCE_OF_COMMON_SEARCH_SPACE / L); i++) {

            cce_start_index = i * L;
            for(j = 0; j < phy_struct->pdcch_cce_allocated; j++) {
                if(phy_struct->pdcch_cce_alloc_type[j] == 1) {
                    if((cce_start_index >= phy_struct->pdcch_cce_start_index[j]) &&
                        (cce_start_index < (phy_struct->pdcch_cce_start_index[j] + phy_struct->pdcch_cce_agg_level[pdcch_cce_allocated]))) {
                        break;
                    }
                }
            }
            if(j < phy_struct->pdcch_cce_allocated) {
                continue;
            }

            decode_cce(phy_struct, N_ant, L, cce_start_index, dci_len, &rnti);

            if(rnti != LIBLTE_MAC_SI_RNTI && rnti != LIBLTE_MAC_P_RNTI && (rnti < LIBLTE_MAC_RA_RNTI_START || rnti > LIBLTE_MAC_RA_RNTI_END)) {
                if(check_cce_index(cce_start_index, rnti, L, phy_struct->n_cce_pdcch, subframe_no)) {

                    pdcch_cce_allocated = phy_struct->pdcch_cce_allocated;

                    phy_struct->pdcch_cce_alloc_type[pdcch_cce_allocated] = 2;
                    phy_struct->pdcch_cce_start_index[pdcch_cce_allocated] = cce_start_index;
                    phy_struct->pdcch_cce_rnti[pdcch_cce_allocated] = rnti;
                    phy_struct->pdcch_cce_agg_level[pdcch_cce_allocated] = L;
                    phy_struct->pdcch_dci_format[pdcch_cce_allocated] = dci_format;
                    phy_struct->pdcch_dci_len[pdcch_cce_allocated] = dci_len;

                    phy_struct->pdcch_cce_allocated = pdcch_cce_allocated + 1;

                }
            }
        }
    }

    return 0;

}


LIBLTE_ERROR_ENUM liblte_phy_find_sss_n
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *i_samps,
    float             *q_samps,
    uint32             N_id_2,
    uint32            *symb_starts,
    float              pss_thresh,
    uint32            *N_id_1,
    uint32            *frame_start_idx
)
{
    LIBLTE_ERROR_ENUM result = LIBLTE_ERROR_INVALID_INPUTS;
    float             sss_thresh;
    float             abs_corr;
    float             abs_corr_0;
    float             abs_corr_5;
    int32_t           i;
    int32_t           j;
    int32_t           k;
    uint32            N_samps_per_symb_else = phy_struct->N_samps_per_symb + phy_struct->N_samps_cp_l_else;
    uint32            N_samps_per_symb_0 = phy_struct->N_samps_per_symb + phy_struct->N_samps_cp_l_0;

    float corr_re;
    float corr_im;
    float corr_max;

    int32_t num_of_sub_carriers;
    int32_t samples_per_sss_symbol;
    int32_t max_nid;

    num_of_sub_carriers = phy_struct->N_rb_dl*phy_struct->N_sc_rb_dl;
    samples_per_sss_symbol = phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb;
    max_nid = LIBLTE_NUM_OF_CELL_GROUP_ID * LTE_NUM_OF_CELL_ID;

    if(phy_struct != NULL && i_samps != NULL && q_samps != NULL && symb_starts != NULL && N_id_1 != NULL && frame_start_idx != NULL) {

#ifdef    LIBLTE_PHY_ANALYSIS
        int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_SSS);
        if(log_msg_enable) {
            fprintf(stderr, "\nliblte_phy_find_sss\n");
            fprintf(stderr, "    Searches for the Secondary Synchronization Signal.\n");
            fprintf(stderr, "    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.11.2\n");
            fprintf(stderr, "    N_id_2 = %d\n", N_id_2);
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        // Generate secondary synchronization signals
#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "\nGenerate Secondary Synchronization Signals (SSS)\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        for(i = 0; i < LIBLTE_NUM_OF_CELL_GROUP_ID; i++) {
            for(j = 0; j < num_of_sub_carriers; j++) {
                phy_struct->sss_mod_re_0[i][j] = 0.0;
                phy_struct->sss_mod_im_0[i][j] = 0.0;
                phy_struct->sss_mod_re_5[i][j] = 0.0;
                phy_struct->sss_mod_im_5[i][j] = 0.0;
            }
        }

        /* Number of CELL Group ID : 168 (0 ~ 167) */
        for(i = 0; i < LIBLTE_NUM_OF_CELL_GROUP_ID; i++) {

            generate_sss(phy_struct,
                         i,
                         N_id_2,
                         phy_struct->sss_re_0,
                         phy_struct->sss_im_0,
                         phy_struct->sss_re_5,
                         phy_struct->sss_im_5);

            /* Made up of 62 Scrambling Sequence */
            for(j = 0; j < LIBLTE_NUM_OF_SSS_SEQUENCE; j++)  {
                k                              = num_of_sub_carriers/2 - LIBLTE_NUM_OF_SSS_SEQUENCE/2 + j;
                phy_struct->sss_mod_re_0[i][k] = phy_struct->sss_re_0[j];
                phy_struct->sss_mod_im_0[i][k] = phy_struct->sss_im_0[j];
                phy_struct->sss_mod_re_5[i][k] = phy_struct->sss_re_5[j];
                phy_struct->sss_mod_im_5[i][k] = phy_struct->sss_im_5[j];
            }

        }

        sss_thresh = pss_thresh * SSS_TO_PSS_THRESHOLD;

        // Demod symbol and search for secondary synchronization signals
#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "\nDemod symbol and search for secondary synchronization signals\n");
            fprintf(stderr, "    sss_thresh = %.2f\n", sss_thresh);
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        samples_to_symbols_dl(phy_struct,
                              i_samps,
                              q_samps,
                              symb_starts[0],
                              SSS_SYMBOL_NUM,
                              0,
                              phy_struct->rx_symb_re,
                              phy_struct->rx_symb_im);
#ifdef SHOW_SSS_CORRELATION
        for(i = 0; i < LIBLTE_NUM_OF_CELL_GROUP_ID; i++) {
            corr_re = 0.0;
            corr_im = 0.0;
            for(j = 0; j < num_of_sub_carriers; j++)  {
                corr_re += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_re_0[i][j] +
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_im_0[i][j]);
                corr_im += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_im_0[i][j] -
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_re_0[i][j]);
            }
            abs_corr_0 = sqrt(corr_re*corr_re + corr_im*corr_im);

            corr_re = 0.0;
            corr_im = 0.0;
            for(j = 0; j < num_of_sub_carriers; j++)  {
                corr_re += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_re_5[i][j] +
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_im_5[i][j]);
                corr_im += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_im_5[i][j] -
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_re_5[i][j]);
            }
            abs_corr_5 = sqrt(corr_re*corr_re + corr_im*corr_im);
#ifdef    LIBLTE_PHY_ANALYSIS
            if(log_msg_enable) {
                fprintf(stderr, "[%3d] %7.2f, %7.2f\n", i, abs_corr_0, abs_corr_5);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

        }
#endif

        for(i = 0; i < LIBLTE_NUM_OF_CELL_GROUP_ID; i++) {
            corr_re = 0.0;
            corr_im = 0.0;
            for(j = 0; j < num_of_sub_carriers; j++)  {
                corr_re += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_re_0[i][j] +
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_im_0[i][j]);
                corr_im += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_im_0[i][j] -
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_re_0[i][j]);
            }
            abs_corr = sqrt(corr_re*corr_re + corr_im*corr_im);
            if(abs_corr >= sss_thresh) {
                *N_id_1          = i;
                *frame_start_idx = symb_starts[0];
                result           = LIBLTE_SUCCESS;
                break;
            }

            corr_re = 0.0;
            corr_im = 0.0;
            for(j = 0; j < num_of_sub_carriers; j++)  {
                corr_re += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_re_5[i][j] +
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_im_5[i][j]);
                corr_im += (phy_struct->rx_symb_re[j]*phy_struct->sss_mod_im_5[i][j] -
                            phy_struct->rx_symb_im[j]*phy_struct->sss_mod_re_5[i][j]);
            }
            abs_corr = sqrt(corr_re*corr_re + corr_im*corr_im);
            if(abs_corr >= sss_thresh) {
                *N_id_1          = i;
                if(symb_starts[0] > phy_struct->N_samps_per_subfr * NUM_OF_SUBFRAME_PER_PSS_PERIOD) {
                    *frame_start_idx = symb_starts[0] - phy_struct->N_samps_per_subfr * NUM_OF_SUBFRAME_PER_PSS_PERIOD;
                }
                else {
                    *frame_start_idx = symb_starts[0] + phy_struct->N_samps_per_subfr * NUM_OF_SUBFRAME_PER_PSS_PERIOD;
                }
                break;
            }

        }

        if(i < LIBLTE_NUM_OF_CELL_GROUP_ID) {
            result = LIBLTE_SUCCESS;
#ifdef    LIBLTE_PHY_ANALYSIS
            if(log_msg_enable) {
                fprintf(stderr, "N_id_1 = %d\n", *N_id_1);
                fprintf(stderr, "frame_start_idx = %d\n", *frame_start_idx);
            }
#endif /* LIBLTE_PHY_ANALYSIS */

        }
        else {

#ifdef    LIBLTE_PHY_ANALYSIS
            if(log_msg_enable) {
                fprintf(stderr, "Could not find SSS\n");
            }
#endif /* LIBLTE_PHY_ANALYSIS */

        }

    }

    return result;
}

LIBLTE_ERROR_ENUM liblte_phy_find_pss_and_fine_timing_n
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *i_samps,
    float             *q_samps,
    uint32            *symb_starts,
    uint32            *N_id_2,
    uint32            *pss_symb,
    float             *pss_thresh,
    float             *freq_offset
)
{

    LIBLTE_ERROR_ENUM   result = LIBLTE_ERROR_INVALID_INPUTS;

    float              corr_re_n1;
    float              corr_im_n1;
    float              corr_re;
    float              corr_im;
    float              corr_re_p1;
    float              corr_im_p1;
    float              abs_corr_n1;
    float              abs_corr;
    float              abs_corr_p1;

    float               corr_max;
    static float        pss_re[63];
    static float        pss_im[63];
    float               *pss_mod_re;
    float               *pss_mod_im;
    int32_t             i;
    int32_t             j;
    int32_t             k;
    int32_t             idx;
    int32_t             j_prime;
    int32_t             k_prime;
    int32_t             z;
    uint32              pss_timing_idx;
    int8                timing;

    int32_t             num_of_sub_carriers;
    int32_t             num_samples_symbol_0;
    int32_t             num_samples_symbol_else;

    static float abs_corr_max[LTE_NUM_OF_CELL_ID];
    static int32_t abs_corr_timing[LTE_NUM_OF_CELL_ID];
    static int32_t abs_corr_idx[LTE_NUM_OF_CELL_ID];

    static float corr_re_n[3];
    static float corr_im_n[3];
    static float abs_corr_n[3][2*SEARCH_FINE_LIMIT + 1];

    num_of_sub_carriers = phy_struct->N_rb_dl * phy_struct->N_sc_rb_dl;
    num_samples_symbol_0 = phy_struct->N_samps_cp_l_0 + phy_struct->N_samps_per_symb;
    num_samples_symbol_else = phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb;

    if(phy_struct != NULL && i_samps != NULL && q_samps != NULL && symb_starts != NULL && N_id_2 != NULL &&
       pss_symb != NULL && pss_thresh != NULL) {


        int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_PSS_AND_FINE_TIMING);
        if(log_msg_enable) {
            fprintf(stderr, "\nliblte_phy_find_pss_and_fine_timing\n");
            fprintf(stderr, "    Searches for the Primary Synchronization Signal and determines fine timing.\n");
            fprintf(stderr, "    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.11.1\n");
            fprintf(stderr, "\nGenerate PSS\n");
        }

        // Generate PSS
        for(i = 0; i < LTE_NUM_OF_CELL_ID; i++) {

            for(j = 0; j < num_of_sub_carriers; j++) {
                phy_struct->pss_mod_re[i][j] = 0;
                phy_struct->pss_mod_im[i][j] = 0;
            }

            generate_pss(i, pss_re, pss_im);

            for(j = 0; j < PSS_SEQ_LENGTH; j++) {
                k = (phy_struct->N_rb_dl*phy_struct->N_sc_rb_dl)/2 - (PSS_SEQ_LENGTH/2) + j;
                phy_struct->pss_mod_re_n1[i][k-1] = pss_re[j];
                phy_struct->pss_mod_im_n1[i][k-1] = pss_im[j];
                phy_struct->pss_mod_re[i][k]      = pss_re[j];
                phy_struct->pss_mod_im[i][k]      = pss_im[j];
                phy_struct->pss_mod_re_p1[i][k+1] = pss_re[j];
                phy_struct->pss_mod_im_p1[i][k+1] = pss_im[j];
            }

        }

        // Demod symbols and correlate with PSS
        if(log_msg_enable) {
            fprintf(stderr, "Demod symbols and correlate with PSS\n");
            fprintf(stderr, "Sample Start\n   ");
            for(j = 0; j < N_SYMB_DL_NORMAL_CP; j++) {
                fprintf(stderr, " %d", symb_starts[j]);
            }
            fprintf(stderr, "\n");
        }

        samples_to_symbols_dl(
            phy_struct,
            i_samps,
            q_samps,
            symb_starts[0],
            PSS_SYMBOL_NUM,
            0,
            phy_struct->rx_symb_re,
            phy_struct->rx_symb_im
        );

        for(k = 0; k < LTE_NUM_OF_CELL_ID; k++) {
            corr_re_n1 = 0.0;
            corr_im_n1 = 0.0;
            corr_re    = 0.0;
            corr_im    = 0.0;
            corr_re_p1 = 0.0;
            corr_im_p1 = 0.0;
            for(z = 0; z < num_of_sub_carriers; z++) {
                corr_re_n1 += (phy_struct->rx_symb_re[z]*phy_struct->pss_mod_re_n1[k][z] +
                               phy_struct->rx_symb_im[z]*phy_struct->pss_mod_im_n1[k][z]);
                corr_im_n1 += (phy_struct->rx_symb_re[z]*phy_struct->pss_mod_im_n1[k][z] -
                               phy_struct->rx_symb_im[z]*phy_struct->pss_mod_re_n1[k][z]);
                corr_re    += (phy_struct->rx_symb_re[z]*phy_struct->pss_mod_re[k][z] +
                               phy_struct->rx_symb_im[z]*phy_struct->pss_mod_im[k][z]);
                corr_im    += (phy_struct->rx_symb_re[z]*phy_struct->pss_mod_im[k][z] -
                               phy_struct->rx_symb_im[z]*phy_struct->pss_mod_re[k][z]);
                corr_re_p1 += (phy_struct->rx_symb_re[z]*phy_struct->pss_mod_re_p1[k][z] +
                               phy_struct->rx_symb_im[z]*phy_struct->pss_mod_im_p1[k][z]);
                corr_im_p1 += (phy_struct->rx_symb_re[z]*phy_struct->pss_mod_im_p1[k][z] -
                               phy_struct->rx_symb_im[z]*phy_struct->pss_mod_re_p1[k][z]);
            }
            abs_corr_n1 = sqrt(corr_re_n1*corr_re_n1 + corr_im_n1*corr_im_n1);
            abs_corr    = sqrt(corr_re*corr_re + corr_im*corr_im);
            abs_corr_p1 = sqrt(corr_re_p1*corr_re_p1 + corr_im_p1*corr_im_p1);

            if(abs_corr > abs_corr_n1) {
                if(abs_corr > abs_corr_p1) {
                    if(abs_corr > corr_max) {
                        idx       = 0;
                        corr_max  = abs_corr;
                        *pss_symb = (i*N_SYMB_DL_NORMAL_CP)+j;
                        *N_id_2   = k;
                    }
                }
                else {
                    if(abs_corr_p1 > corr_max) {
                        idx       = 1;
                        corr_max  = abs_corr_p1;
                        *pss_symb = (i*N_SYMB_DL_NORMAL_CP)+j;
                        *N_id_2   = k;
                    }
                }
            }
            else {
                if(abs_corr_n1 > abs_corr_p1) {
                    if(abs_corr_n1 > corr_max) {
                        idx       = -1;
                        corr_max  = abs_corr_n1;
                        *pss_symb = (i*N_SYMB_DL_NORMAL_CP)+j;
                        *N_id_2   = k;
                    }
                }
                else {
                    if(abs_corr_p1 > corr_max) {
                        idx       = 1;
                        corr_max  = abs_corr_p1;
                        *pss_symb = (i*N_SYMB_DL_NORMAL_CP)+j;
                        *N_id_2   = k;

                    }
                }
            }
        }

        if(log_msg_enable) {
            fprintf(stderr, "corr_max = %8.2f, N_id_2 = %d\n", corr_max, *N_id_2);
        }

        if(-1 == idx) {
            pss_mod_re   = &phy_struct->pss_mod_re_n1[*N_id_2][0];
            pss_mod_im   = &phy_struct->pss_mod_im_n1[*N_id_2][0];
            *freq_offset = -15000; // FIXME
        }
        else if(0 == idx) {
            pss_mod_re   = &phy_struct->pss_mod_re[*N_id_2][0];
            pss_mod_im   = &phy_struct->pss_mod_im[*N_id_2][0];
            *freq_offset = 0;
        }
        else{
            pss_mod_re   = &phy_struct->pss_mod_re_p1[*N_id_2][0];
            pss_mod_im   = &phy_struct->pss_mod_im_p1[*N_id_2][0];
            *freq_offset = 15000; // FIXME
        }

        corr_max = 0.0;
        timing   = 0;
        for(i = -SEARCH_FINE_LIMIT; i <= SEARCH_FINE_LIMIT; i++) {

            samples_to_symbols_dl(phy_struct,
                                  i_samps,
                                  q_samps,
                                  symb_starts[0] + i,
                                  PSS_SYMBOL_NUM,
                                  0,
                                  phy_struct->rx_symb_re,
                                  phy_struct->rx_symb_im);

            corr_re = 0.0;
            corr_im = 0.0;
            for(j = 0; j < num_of_sub_carriers; j++) {
                corr_re += (phy_struct->rx_symb_re[j]*pss_mod_re[j] +
                            phy_struct->rx_symb_im[j]*pss_mod_im[j]);
                corr_im += (phy_struct->rx_symb_re[j]*pss_mod_im[j] -
                            phy_struct->rx_symb_im[j]*pss_mod_re[j]);
            }
            abs_corr = sqrt(corr_re*corr_re + corr_im*corr_im);
            if(abs_corr > corr_max) {
                corr_max = abs_corr;
                timing   = i;
            }

            if(log_msg_enable) {
                fprintf(stderr, "[%3d] abs_corr = %7.2f, corr_max[%3d] = %7.2f\n", i, abs_corr, timing, corr_max);
            }

        }

        *pss_thresh = corr_max;

        // Construct fine symbol start locations
        pss_timing_idx = symb_starts[PSS_SYMBOL_NUM] + timing;

        if(log_msg_enable) {
            fprintf(stderr, "\nConstruct fine symbol start locations\n");
            fprintf(stderr, "timing = %d, pss_timing_idx = %d, *pss_thresh = %.2f\n", timing, pss_timing_idx, *pss_thresh);
        }

        if(pss_timing_idx > phy_struct->N_samps_per_slot) {
            symb_starts[0] = pss_timing_idx + num_samples_symbol_else - phy_struct->N_samps_per_slot;
        }
        else {
            symb_starts[0] = pss_timing_idx + num_samples_symbol_else + phy_struct->N_samps_per_subfr * NUM_OF_SUBFRAME_PER_PSS_PERIOD - phy_struct->N_samps_per_slot;
        }
        for(j = 1; j <= PSS_SYMBOL_NUM; j++) {
            symb_starts[j] = symb_starts[0] + num_samples_symbol_0 + (j-1) * num_samples_symbol_else;
        }

        if(log_msg_enable) {

            fprintf(stderr, "\n");
            fprintf(stderr, "Construct fine symbol start locations\n");
            fprintf(stderr, "N_id_2 = %d, timing = %d, pss_timing_idx = %d, *pss_thresh = %.2f\n", *N_id_2, timing, pss_timing_idx, *pss_thresh);
            fprintf(stderr, "Sample Start\n   ");
            for(j = 0; j < N_SYMB_DL_NORMAL_CP; j++) {
                fprintf(stderr, " %d", symb_starts[j]);
            }
            fprintf(stderr, "\n");
        }

        result = LIBLTE_SUCCESS;

    }

    return result;
}




LIBLTE_ERROR_ENUM liblte_phy_dl_find_coarse_timing_and_freq_offset_n
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    float               *i_samps,
    float               *q_samps,
    uint32               N_slots,
    LIBLTE_PHY_COARSE_TIMING_STRUCT *timing_struct
)
{

    LIBLTE_ERROR_ENUM   result = LIBLTE_ERROR_INVALID_INPUTS;
    int32_t             i;
    int32_t             j;
    int32_t             k;
    int32_t             l;
    float               corr_re;
    float               corr_im;
    uint32              idx;
    float               abs_corr_max;
    static float        abs_corr_max_by_nid2[LTE_NUM_OF_CELL_ID];
    int32_t             nid2;
    int32_t             delta_idx;
    int32_t             max_idx;

    static int32        abs_corr_idx[LTE_NUM_OF_CELL_ID][NUM_OF_MAX_SEARCH_PSS_CORR_PEAK];
    static float        abs_corr[LTE_NUM_OF_CELL_ID][NUM_OF_MAX_SEARCH_PSS_CORR_PEAK];
    static int32        abs_corr_bin_width[LTE_NUM_OF_CELL_ID][NUM_OF_MAX_SEARCH_PSS_CORR_PEAK];
    int32_t             n_corr_peaks[LTE_NUM_OF_CELL_ID];
    int32_t             corr_peak_count;

    static int32        abs_corr_idx_tmp[LTE_NUM_OF_CELL_ID][NUM_OF_MAX_SEARCH_PSS_CORR_PEAK];
    static float        abs_corr_tmp[LTE_NUM_OF_CELL_ID][NUM_OF_MAX_SEARCH_PSS_CORR_PEAK];
    static int32        abs_corr_bin_width_tmp[LTE_NUM_OF_CELL_ID][NUM_OF_MAX_SEARCH_PSS_CORR_PEAK];
    int32_t             n_corr_peaks_tmp[LTE_NUM_OF_CELL_ID];

    int32_t             pss_period;
    int32_t             samples_symbol_0;
    int32_t             samples_symbol_else;
    int32_t             symbol_0_start;

#ifdef    LIBLTE_PHY_ANALYSIS
    int log_msg_enable = LOG_MSG_ENABLE(LOG_MSG_CODE_SEARCH_COARSE_TIMING);
#endif /* LIBLTE_PHY_ANALYSIS */

    samples_symbol_0 = phy_struct->N_samps_cp_l_0 + phy_struct->N_samps_per_symb;
    samples_symbol_else = phy_struct->N_samps_cp_l_else + phy_struct->N_samps_per_symb;
    pss_period = phy_struct->N_samps_per_subfr * NUM_OF_SUBFRAME_PER_PSS_PERIOD;

    if(phy_struct != NULL && i_samps != NULL && q_samps != NULL && timing_struct != NULL)  {

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "\nliblte_phy_dl_find_coarse_timing_and_freq_offset\n");
            fprintf(stderr, "    Finds coarse time syncronization and frequency offset by auto-correlating to find the cyclic prefix on\n");
            fprintf(stderr, "    reference signal symbols of the downlink\n");
            fprintf(stderr, "    Document Reference: 3GPP TS 36.211 v10.1.0\n");
            fprintf(stderr, "    N_slots = %d\n", N_slots);
            fprintf(stderr, "    N_samps_per_slot = %d\n", phy_struct->N_samps_per_slot);
            fprintf(stderr, "    N_samps_cp_l_0 = %d\n", phy_struct->N_samps_cp_l_0);
            fprintf(stderr, "    N_samps_cp_l_else = %d \n", phy_struct->N_samps_cp_l_else);
            fprintf(stderr, "    N_samps_per_symb = %d\n", phy_struct->N_samps_per_symb);
            fprintf(stderr, "\nTiming correlation\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        /* CROSS CORRELATION DETECTION ALGORITHM */
        uint32_t correlation_search_len = phy_struct->N_samps_per_frame * NUM_OF_SEARCH_FRAME;

        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
            fprintf(stderr, "NID2 = %d\n", nid2);
            for(i = 0; i < correlation_search_len; i++) {
                phy_struct->dl_timing_cross_corr[nid2][i] = 0.0;
            }
            for(i = 0; i < correlation_search_len; i++) {
                corr_re = 0.0;
                corr_im = 0.0;
                for(j = 0; j < samples_symbol_else; j++) {
                    idx      = i + j;
                    corr_re += i_samps[idx] * phy_struct->pss_sample_re[nid2][j] + q_samps[idx] * phy_struct->pss_sample_im[nid2][j];
                    corr_im += i_samps[idx] * phy_struct->pss_sample_im[nid2][j] - q_samps[idx] * phy_struct->pss_sample_re[nid2][j];
                }
                phy_struct->dl_timing_cross_corr[nid2][i] += corr_re * corr_re + corr_im * corr_im;
            }
        }

        abs_corr_max = 0.0;
        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
            abs_corr_max_by_nid2[nid2] = liblte_get_max_value(phy_struct->dl_timing_cross_corr[nid2], correlation_search_len);
            if(abs_corr_max < abs_corr_max_by_nid2[nid2]) {
                abs_corr_max = abs_corr_max_by_nid2[nid2];
            }
        }

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "\n");
            fprintf(stderr, "Find abs_corr_max : abs_corr_max = %.2f\n", abs_corr_max);
            fprintf(stderr, "Remove Low Power Bin\n");
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
            if(abs_corr_max_by_nid2[nid2] < abs_corr_max * PSS_POWER_MAX_TO_THRESHOLD_RATIO) {
                abs_corr_max_by_nid2[nid2] = -1.0;
                for(i = 0; i < correlation_search_len; i++) {
                    phy_struct->dl_timing_cross_corr[nid2][i] = 0.0;
                }
            }
            else {
                for(i = 0; i < correlation_search_len; i++) {
                    if(phy_struct->dl_timing_cross_corr[nid2][i] < abs_corr_max * SEARCH_CUT_OFF_CORRELATION_RATIO) {
                        phy_struct->dl_timing_cross_corr[nid2][i] = 0.0;
                    }
                    else {
                        phy_struct->dl_timing_cross_corr[nid2][i] = phy_struct->dl_timing_cross_corr[nid2][i] / abs_corr_max;
                    }
                }
            }
        }

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
                if(abs_corr_max_by_nid2[nid2] <= 0) {
                    continue;
                }
                fprintf(stderr, "\n");
                fprintf(stderr, "-------------- Normalized Correlation for NID %d\n", nid2);
                print_float_scaled2(phy_struct->dl_timing_cross_corr[nid2], correlation_search_len, 1.0);
            }
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
            if(abs_corr_max_by_nid2[nid2] <= 0) {
                continue;
            }

            /* find Local Maximum */
            n_corr_peaks[nid2] = 0;

            for(i = 1; i < (correlation_search_len - 1); i++) {
                if((phy_struct->dl_timing_cross_corr[nid2][i+1] < phy_struct->dl_timing_cross_corr[nid2][i]) &&
                    (phy_struct->dl_timing_cross_corr[nid2][i-1] < phy_struct->dl_timing_cross_corr[nid2][i])) {
                    corr_peak_count = n_corr_peaks[nid2];
                    if(corr_peak_count == 0) {
                        abs_corr_idx[nid2][corr_peak_count] = i;
                        abs_corr[nid2][corr_peak_count] = phy_struct->dl_timing_cross_corr[nid2][i];
                        n_corr_peaks[nid2] = corr_peak_count + 1;
                    }
                    else  {
                        if((i - abs_corr_idx[nid2][corr_peak_count - 1]) < SEARCH_COARSE_LIMIT) {
                            /* Merge Narrow Bin */
                            if(abs_corr[nid2][corr_peak_count - 1] <= phy_struct->dl_timing_cross_corr[nid2][i]) {
                                abs_corr_idx[nid2][corr_peak_count - 1] = i;
                                abs_corr[nid2][corr_peak_count - 1] = phy_struct->dl_timing_cross_corr[nid2][i];
                            }
                        }
                        else {
                            abs_corr_idx[nid2][corr_peak_count] = i;
                            abs_corr[nid2][corr_peak_count] = phy_struct->dl_timing_cross_corr[nid2][i];
                            n_corr_peaks[nid2] = corr_peak_count + 1;
                        }
                    }
                }
            }

#ifdef    LIBLTE_PHY_ANALYSIS
            if(log_msg_enable) {
                fprintf(stderr, "\n");
                fprintf(stderr, "Find Local Maximum : Nid2 = %d, count = %d\n", nid2, n_corr_peaks[nid2]);
                for(i = 0; i < n_corr_peaks[nid2];i++) {
                    fprintf(stderr, "[%02d] abs_corr_idx[%d][%02d] = %7d, abs_cor[%d][%2d] = %.2f\n",
                        i, nid2, i, abs_corr_idx[nid2][i], nid2, i, abs_corr[nid2][i]);
                }
                fprintf(stderr, "\n");
            }
#endif /* LIBLTE_PHY_ANALYSIS */

            n_corr_peaks_tmp[nid2] = 0;
            while(1) {

                /* Find Max Correlation */
                abs_corr_max = 0.0;
                max_idx = -1;
                for(i = 0; i < n_corr_peaks[nid2]; i++) {
                    if(abs_corr[nid2][i] > abs_corr_max) {
                        abs_corr_max = abs_corr[nid2][i];
                        max_idx = i;
                    }
                }
                if(max_idx < 0) {
                    break;
                }

#ifdef    LIBLTE_PHY_ANALYSIS
                if(log_msg_enable) {
                    fprintf(stderr, "\n");
                    fprintf(stderr, "Find Max abs_corr[%d][%02d] = %.2f at max_idx = %02d, abs_corr_idx[%d][%02d] = %07d\n",
                        nid2, max_idx, abs_corr[nid2][max_idx], max_idx, nid2, max_idx, abs_corr_idx[nid2][max_idx]);
                    fprintf(stderr, "\n");
                }
#endif /* LIBLTE_PHY_ANALYSIS */

                abs_corr_bin_width[nid2][max_idx] = 1;
                /* Clear Other Peaks */
                for(i = 0; i < n_corr_peaks[nid2]; i++) {
                    if(i != max_idx && abs_corr[nid2][i] > 0.0) {
                        delta_idx = abs(abs_corr_idx[nid2][i] - abs_corr_idx[nid2][max_idx]);
                        while(delta_idx >= (pss_period - SEARCH_COARSE_LIMIT)) {
                            delta_idx -= pss_period;
                            if((-SEARCH_COARSE_LIMIT <= delta_idx) && (delta_idx <= SEARCH_COARSE_LIMIT)) {
                                break;
                            }
                        }
                        if((-SEARCH_COARSE_LIMIT <= delta_idx) && (delta_idx <= SEARCH_COARSE_LIMIT)) {

                            abs_corr_idx[nid2][i] = -1;
                            abs_corr[nid2][i] = -1.0;
                            abs_corr_bin_width[nid2][max_idx] = abs_corr_bin_width[nid2][max_idx] + 1;
                        }
                    }
                }

                // Add New ..
                corr_peak_count = n_corr_peaks_tmp[nid2];
                abs_corr_idx_tmp[nid2][corr_peak_count] = abs_corr_idx[nid2][max_idx];
                abs_corr_tmp[nid2][corr_peak_count] = abs_corr[nid2][max_idx];
                abs_corr_bin_width_tmp[nid2][corr_peak_count] = abs_corr_bin_width[nid2][max_idx];
                abs_corr[nid2][max_idx] = -1.0;
                n_corr_peaks_tmp[nid2] = corr_peak_count + 1;
            }
        }

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
                if(abs_corr_max_by_nid2[nid2] <= 0) {
                    continue;
                }
                fprintf(stderr, "\n");
                fprintf(stderr, "Find Local Maximum : Nid2 = %d, count = %d\n", nid2, n_corr_peaks_tmp[nid2]);
                for(i = 0; i < n_corr_peaks_tmp[nid2];i++) {
                    fprintf(stderr, "[%02d] abs_corr_idx[%d][%02d] = %7d, abs_cor[%d][%2d] = %.2f, n_abs_corr_bin[%d][%2d] = %d\n",
                        i, nid2, i, abs_corr_idx_tmp[nid2][i], nid2, i, abs_corr_tmp[nid2][i], nid2, i, abs_corr_bin_width_tmp[nid2][i]);
                }
            }
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        /* Remove lonely bin */
        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
            if(abs_corr_max_by_nid2[nid2] <= 0) {
                continue;
            }
            n_corr_peaks[nid2] = 0;
            for(i = 0; i < n_corr_peaks_tmp[nid2]; i++) {
                if(abs_corr_bin_width_tmp[nid2][i] > 1) {
                    int32_t temp_idx = n_corr_peaks[nid2];
                    abs_corr_idx[nid2][temp_idx] = abs_corr_idx_tmp[nid2][i];
                    abs_corr[nid2][temp_idx] = abs_corr_tmp[nid2][i];
                    abs_corr_bin_width[nid2][temp_idx] = abs_corr_bin_width_tmp[nid2][i];
                    n_corr_peaks[nid2] = temp_idx + 1;
                }
            }
        }

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
                if(abs_corr_max_by_nid2[nid2] <= 0) {
                    continue;
                }
                fprintf(stderr, "\n");
                fprintf(stderr, "After removing lonely BIN : Nid2 = %d, count = %d\n", nid2, n_corr_peaks[nid2]);
                for(i = 0; i < n_corr_peaks[nid2];i++) {
                    fprintf(stderr, "[%02d] abs_corr_idx[%d][%02d] = %7d, abs_cor[%d][%2d] = %.2f, n_abs_corr_bin[%d][%2d] = %d\n",
                        i, nid2, i, abs_corr_idx[nid2][i], nid2, i, abs_corr[nid2][i], nid2, i, abs_corr_bin_width[nid2][i]);
                }
            }
        }
#endif /* LIBLTE_PHY_ANALYSIS */

        timing_struct->n_corr_peaks = 0;
        for(nid2 = 0; nid2 < LTE_NUM_OF_CELL_ID; nid2++) {
            if(abs_corr_max_by_nid2[nid2] <= 0) {
                continue;
            }

            for(i = 0; i < n_corr_peaks[nid2]; i++) {

                for(j = 0; j < timing_struct->n_corr_peaks; j++) {
                    delta_idx = abs(timing_struct->abs_corr_idx[j] - abs_corr_idx[nid2][i]);
                    if(delta_idx < SEARCH_COARSE_LIMIT) {
                        if(timing_struct->abs_corr[j] < abs_corr[nid2][i]) {
                            timing_struct->abs_corr[j] = abs_corr[nid2][i];
                            timing_struct->abs_corr_idx[j] = abs_corr_idx[nid2][i];
                            timing_struct->nid2[j] = nid2;
                        }
                        break;
                    }
                }

                if(j < timing_struct->n_corr_peaks) {
                    continue;
                }

                timing_struct->nid2[timing_struct->n_corr_peaks] = nid2;
                timing_struct->abs_corr[timing_struct->n_corr_peaks] = abs_corr[nid2][i];
                timing_struct->abs_corr_idx[timing_struct->n_corr_peaks] = abs_corr_idx[nid2][i];

                /* FDD Normal CP */
                if(abs_corr_idx[nid2][i] > phy_struct->N_samps_per_slot) {
                    symbol_0_start = abs_corr_idx[nid2][i] + samples_symbol_else - phy_struct->N_samps_per_slot;
                }
                else {
                    symbol_0_start = abs_corr_idx[nid2][i] + samples_symbol_else + pss_period - phy_struct->N_samps_per_slot;
                }
                for(j = 0; j <= PSS_SYMBOL_NUM; j++) {
                    if(j == 0) {
                        timing_struct->symb_starts[timing_struct->n_corr_peaks+i][0] = symbol_0_start;
                    }
                    else {
                        timing_struct->symb_starts[timing_struct->n_corr_peaks+i][j] = symbol_0_start + samples_symbol_0 + (j-1) * samples_symbol_else;
                    }
                }
                timing_struct->n_corr_peaks = timing_struct->n_corr_peaks + 1;
            }
        }

#ifdef    LIBLTE_PHY_ANALYSIS
        if(log_msg_enable) {
            fprintf(stderr, "\n");
            fprintf(stderr, "timing_struct->n_corr_peaks = %d\n", timing_struct->n_corr_peaks);
            for(i = 0; i < timing_struct->n_corr_peaks; i ++) {
                fprintf(stderr, "PEAK %d : nid2 = %d,", i, timing_struct->nid2[i]);
                for(j = 0; j <= PSS_SYMBOL_NUM; j++) {
                    fprintf(stderr, " [%d] %-8d", j, timing_struct->symb_starts[i][j]);
                }
                fprintf(stderr, "\n");
            }
        }
#endif /* LIBLTE_PHY_ANALYSIS */


        result = LIBLTE_SUCCESS;

    }

    return result;

}




LIBLTE_ERROR_ENUM liblte_phy_pdcch_channel_decode_n2
(
    LIBLTE_PHY_STRUCT              *phy_struct,
    LIBLTE_PHY_SUBFRAME_STRUCT     *subframe,
    uint32                          N_id_cell,
    uint8                           N_ant,
    LIBLTE_PHY_PCFICH_STRUCT       *pcfich,
    LIBLTE_PHY_PHICH_STRUCT        *phich,
    LIBLTE_PHY_PDCCH_STRUCT        *pdcch
)
{
    LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;

    uint32_t            N_reg_rb;
    uint32_t            N_reg_pdcch;
    uint32_t            N_cce_pdcch;

    uint32_t            k_prime;
    uint32_t            m_prime;
    uint32_t            l_prime;
    bool                valid_reg;

    int32_t             num_of_sub_carriers;
    int32_t             i;
    int32_t             j;
    int32_t             k;
    int32_t             idx;
    int32_t             p;
    int32_t             shift_idx;
    uint32_t            C_cc_sb;
    uint32_t            R_cc_sb;
    uint32_t            N_dummy;
    uint32_t            K_pi;
    uint32_t            c_init;

    int32_t result;
    int32_t log_msg_enabled;

    log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);
    num_of_sub_carriers = phy_struct->N_rb_dl*phy_struct->N_sc_rb_dl;

    if(phy_struct != NULL &&  subframe != NULL && pcfich != NULL && phich != NULL && pdcch != NULL) {

        // PDCCH
        // Calculate number of symbols, 3GPP TS 36.211 v10.1.0 section 6.7
        // Calculate resources, 3GPP TS 36.211 v10.1.0 section 6.8.1

        N_reg_rb = 3;
        N_reg_pdcch = pdcch->N_symbs*(phy_struct->N_rb_dl * N_reg_rb) - phy_struct->N_rb_dl - pcfich->N_reg - phich->N_reg;
        if(N_ant == 4) {
            // Remove CRS
            N_reg_pdcch -= phy_struct->N_rb_dl;
        }
        N_cce_pdcch = N_reg_pdcch/9;
        phy_struct->n_cce_pdcch = N_cce_pdcch;

        if(log_msg_enabled) {
            fprintf(stderr, "pdcch->N_symbs = %d, phy_struct->N_rb_dl = %d, pcfich->N_reg = %d, phich->N_reg = %d, N_reg_pdcch = %d, N_cce_pdcch = %d\n",
                pdcch->N_symbs, phy_struct->N_rb_dl, pcfich->N_reg, phich->N_reg, N_reg_pdcch, N_cce_pdcch);
        }

        // Step 1 and 2
        m_prime = 0;
        k_prime = 0;
        // Step 10
        while(k_prime < num_of_sub_carriers) {
            // Step 3
            l_prime = 0;
            // Step 8
            while(l_prime < pdcch->N_symbs)  {
                if(l_prime == 0) {
                    // Step 4
                    // Avoid PCFICH and PHICH
                    valid_reg = true;
                    for(i = 0; i < pcfich->N_reg; i++) {
                        if(k_prime == pcfich->k[i]) {
                            valid_reg = false;
                        }
                    }
                    for(i = 0; i < phich->N_reg; i++) {
                        if(k_prime == phich->k[i]) {
                            valid_reg = false;
                        }
                    }
                    if((k_prime % 6) == 0 && valid_reg == true) {
                        if(m_prime < N_reg_pdcch) {
                            // Step 5
                            idx = 0;
                            for(i = 0; i < 6; i++) {
                                // Avoid CRS
                                if((N_id_cell % 3) != (i % 3)) {
                                    phy_struct->pdcch_reg_y_est_re[m_prime][idx] = subframe->rx_symb_re[l_prime][k_prime+i];
                                    phy_struct->pdcch_reg_y_est_im[m_prime][idx] = subframe->rx_symb_im[l_prime][k_prime+i];
                                    for(p = 0; p < N_ant; p++) {
                                        phy_struct->pdcch_reg_c_est_re[p][m_prime][idx] = subframe->rx_ce_re[p][l_prime][k_prime+i];
                                        phy_struct->pdcch_reg_c_est_im[p][m_prime][idx] = subframe->rx_ce_im[p][l_prime][k_prime+i];
                                    }
                                    idx++;
                                }
                            }
                            // Step 6
                            m_prime++;
                        }
                    }
                }
                else if(l_prime == 1 && N_ant == 4) {
                    // Step 4
                    if((k_prime % 6) == 0) {
                        if(m_prime < N_reg_pdcch) {
                            // Step 5
                            idx = 0;
                            for(i = 0; i < 6; i++) {
                                // Avoid CRS
                                if((N_id_cell % 3) != (i % 3))  {
                                    /* COPY 4 Samples.. */
                                    phy_struct->pdcch_reg_y_est_re[m_prime][idx] = subframe->rx_symb_re[l_prime][k_prime+i];
                                    phy_struct->pdcch_reg_y_est_im[m_prime][idx] = subframe->rx_symb_im[l_prime][k_prime+i];
                                    for(p=0; p<N_ant; p++) {
                                        phy_struct->pdcch_reg_c_est_re[p][m_prime][idx] = subframe->rx_ce_re[p][l_prime][k_prime+i];
                                        phy_struct->pdcch_reg_c_est_im[p][m_prime][idx] = subframe->rx_ce_im[p][l_prime][k_prime+i];
                                    }
                                    idx++;
                                }
                            }
                            // Step 6
                            m_prime++;
                        }
                    }
                }
                else {
                    // Step 4
                    if((k_prime % 4) == 0) {
                        if(m_prime < N_reg_pdcch) {
                            // Step 5
                            for(i = 0; i < 4; i++) {
                                phy_struct->pdcch_reg_y_est_re[m_prime][i] = subframe->rx_symb_re[l_prime][k_prime+i];
                                phy_struct->pdcch_reg_y_est_im[m_prime][i] = subframe->rx_symb_im[l_prime][k_prime+i];
                                for(p = 0; p < N_ant; p++) {
                                    phy_struct->pdcch_reg_c_est_re[p][m_prime][i] = subframe->rx_ce_re[p][l_prime][k_prime+i];
                                    phy_struct->pdcch_reg_c_est_im[p][m_prime][i] = subframe->rx_ce_im[p][l_prime][k_prime+i];
                                }
                            }
                            m_prime++;
                        }
                    }
                }
                // Step 7
                l_prime++;
            }
            // Step 9
            k_prime++;
        }

        // Undo cyclic shift of the REGs
        for(i = 0; i < N_reg_pdcch; i++) {
            shift_idx = (i+N_id_cell) % N_reg_pdcch;
            for(j = 0; j < 4; j++) {
                phy_struct->pdcch_shift_y_est_re[shift_idx][j] = phy_struct->pdcch_reg_y_est_re[i][j];
                phy_struct->pdcch_shift_y_est_im[shift_idx][j] = phy_struct->pdcch_reg_y_est_im[i][j];
                for(p = 0; p < N_ant; p++) {
                    phy_struct->pdcch_shift_c_est_re[p][shift_idx][j] = phy_struct->pdcch_reg_c_est_re[p][i][j];
                    phy_struct->pdcch_shift_c_est_im[p][shift_idx][j] = phy_struct->pdcch_reg_c_est_im[p][i][j];
                }
            }
        }

        // Undo permutation of the REGs, 3GPP TS 36.212 v10.1.0 section 5.1.4.2.1
        for(i = 0; i < N_reg_pdcch; i++) {
            phy_struct->pdcch_reg_vec[i] = i;
        }

        // In order to recreate circular buffer, a dummy block must be
        // sub block interleaved to determine where NULL bits are to be
        // inserted
        // Step 1
        C_cc_sb = 32;
        // Step 2
        R_cc_sb = 0;
        while(N_reg_pdcch > (C_cc_sb*R_cc_sb)) {
            R_cc_sb++;
        }
        // Step 3
        if(N_reg_pdcch < (C_cc_sb*R_cc_sb)) {
            N_dummy = C_cc_sb*R_cc_sb - N_reg_pdcch;
        }
        else {
            N_dummy = 0;
        }

        for(i=0; i<N_dummy; i++) {
            phy_struct->ruc_tmp[i] = RX_NULL_BIT;
        }
        for(i=N_dummy; i<C_cc_sb*R_cc_sb; i++) {
            phy_struct->ruc_tmp[i] = 0;
        }
        idx = 0;
        for(i=0; i<R_cc_sb; i++) {
            for(j=0; j<C_cc_sb; j++) {
                phy_struct->ruc_sb_mat[i][j] = phy_struct->ruc_tmp[idx++];
            }
        }
        // Step 4
        for(i=0; i<R_cc_sb; i++) {
            for(j=0; j<C_cc_sb; j++) {
                phy_struct->ruc_sb_perm_mat[i][j] = phy_struct->ruc_sb_mat[i][IC_PERM_CC[j]];
            }
        }
        // Step 5
        idx = 0;
        for(j=0; j<C_cc_sb; j++) {
            for(i=0; i<R_cc_sb; i++) {
                phy_struct->ruc_v[0][idx++] = phy_struct->ruc_sb_perm_mat[i][j];
            }
        }
        // Recreate circular buffer
        K_pi = R_cc_sb*C_cc_sb;
        k    = 0;
        j    = 0;
        while(k < N_reg_pdcch) {
            if(phy_struct->ruc_v[0][j%K_pi] != RX_NULL_BIT) {
                phy_struct->ruc_v[0][j%K_pi] = phy_struct->pdcch_reg_vec[k++];
            }
            j++;
        }
        // Sub block deinterleaving
        // Step 5
        idx = 0;
        for(j=0; j<C_cc_sb; j++) {
            for(i=0; i<R_cc_sb; i++) {
                phy_struct->ruc_sb_perm_mat[i][j] = phy_struct->ruc_v[0][idx++];
            }
        }
        // Step 4
        for(i=0; i<R_cc_sb; i++) {
            for(j=0; j<C_cc_sb; j++) {
                phy_struct->ruc_sb_mat[i][IC_PERM_CC[j]] = phy_struct->ruc_sb_perm_mat[i][j];
            }
        }
        // Step 3
        idx = 0;
        for(i=0; i<R_cc_sb; i++) {
            for(j=0; j<C_cc_sb; j++) {
                phy_struct->ruc_tmp[idx++] = phy_struct->ruc_sb_mat[i][j];
            }
        }
        for(i=0; i < N_reg_pdcch; i++) {
            for(j=0; j<4; j++) {
                phy_struct->pdcch_perm_y_est_re[i][j] = phy_struct->pdcch_shift_y_est_re[(uint32)phy_struct->ruc_tmp[N_dummy+i]][j];
                phy_struct->pdcch_perm_y_est_im[i][j] = phy_struct->pdcch_shift_y_est_im[(uint32)phy_struct->ruc_tmp[N_dummy+i]][j];
                for(p=0; p<N_ant; p++)
                {
                    phy_struct->pdcch_perm_c_est_re[p][i][j] = phy_struct->pdcch_shift_c_est_re[p][(uint32)phy_struct->ruc_tmp[N_dummy+i]][j];
                    phy_struct->pdcch_perm_c_est_im[p][i][j] = phy_struct->pdcch_shift_c_est_im[p][(uint32)phy_struct->ruc_tmp[N_dummy+i]][j];
                }
            }
        }
        // Construct CCEs
        for(i=0; i<N_cce_pdcch; i++)
        {
            for(j=0; j<LIBLTE_PHY_PDCCH_N_REG_CCE; j++)
            {
                for(k=0; k<4; k++)
                {
                    phy_struct->pdcch_cce_y_est_re[i][j*4+k] = phy_struct->pdcch_perm_y_est_re[i*LIBLTE_PHY_PDCCH_N_REG_CCE+j][k];
                    phy_struct->pdcch_cce_y_est_im[i][j*4+k] = phy_struct->pdcch_perm_y_est_im[i*LIBLTE_PHY_PDCCH_N_REG_CCE+j][k];
                    for(p = 0; p < N_ant; p++) {
                        phy_struct->pdcch_cce_c_est_re[p][i][j*4+k] = phy_struct->pdcch_perm_c_est_re[p][i*LIBLTE_PHY_PDCCH_N_REG_CCE+j][k];
                        phy_struct->pdcch_cce_c_est_im[p][i][j*4+k] = phy_struct->pdcch_perm_c_est_im[p][i*LIBLTE_PHY_PDCCH_N_REG_CCE+j][k];
                    }
                }
            }
        }

        // Generate the scrambling sequence
        c_init = (subframe->num << 9) + N_id_cell;
        generate_prs_c(c_init, LIBLTE_PHY_PDCCH_N_BITS_MAX * 2, phy_struct->pdcch_c);


    }

    return(err);

}

LIBLTE_ERROR_ENUM liblte_phy_get_tbs_mcs_and_n_prb_for_dl_by_mod_type
(
    uint32_t  N_bits,
    uint32_t  N_subframe,
    uint32_t  N_rb_dl,
    uint16_t  rnti,
    uint32_t *tbs,
    uint8_t  *mcs,
    uint32_t *N_prb,
    LIBLTE_PHY_MODULATION_TYPE_ENUM mod_type
)
{
    LIBLTE_ERROR_ENUM err = LIBLTE_ERROR_INVALID_INPUTS;
    uint32            i;
    uint32            j;
    uint32            code_rate;
    uint32            N_prb_tmp;
    uint32            N_bits_per_prb;
    uint32            I_tbs;

    uint32            tbs_index_start;
    uint32            tbs_index_end;

    if(tbs   != NULL &&
       mcs   != NULL &&
       N_prb != NULL)
    {

        if(mod_type == LIBLTE_PHY_MODULATION_TYPE_QPSK) {
            tbs_index_start = 0;
            tbs_index_end = 10;
        }
        else if(mod_type == LIBLTE_PHY_MODULATION_TYPE_16QAM) {
            tbs_index_start = 9;
            tbs_index_end = 16;
        }
        else if(mod_type == LIBLTE_PHY_MODULATION_TYPE_64QAM) {
            tbs_index_start = 15;
            tbs_index_end = 27;
        }
        else {
            tbs_index_start = 0;
            tbs_index_end = 27;
        }

        if(LIBLTE_MAC_SI_RNTI        == rnti ||
           LIBLTE_MAC_P_RNTI         == rnti ||
           (LIBLTE_MAC_RA_RNTI_START <= rnti &&
            LIBLTE_MAC_RA_RNTI_END   >= rnti))  {

            // Choose N_prb == 2 to give the largest possible tbs
            N_prb_tmp = 2;
            for(i = tbs_index_start; i < tbs_index_end; i++) {
                if(N_bits <= TBS_71721[i][N_prb_tmp]) {
                    *tbs = TBS_71721[i][N_prb_tmp];
                    *mcs = i;
                    break;
                }
            }

            // Target coding rate is 4:1, will allow down to 3:1
            N_bits_per_prb = get_num_bits_in_prb(N_subframe, 3, N_rb_dl/2, N_rb_dl, 2, mod_type);
            *N_prb         = 0;
            for(code_rate = 4; code_rate > 2; code_rate--) {
                for(i=1; i<=N_rb_dl; i++) {
                    if((*tbs * code_rate) < (N_bits_per_prb*i)) {
                        *N_prb = i;
                        break;
                    }
                }
                if(*N_prb != 0) {
                    break;
                }
            }

            if(*N_prb != 0) {
                err = LIBLTE_SUCCESS;
            }
        }
        else {
            *N_prb = 0;
            for(i = tbs_index_start; i < tbs_index_end; i++) {

                for(j = 0; j < N_rb_dl; j++)  {
                    if(N_bits <= TBS_71721[i][j]) {
                        *tbs   = TBS_71721[i][j];
                        I_tbs  = i;
                        *N_prb = j + 1;
                        break;
                    }
                }

                if(*N_prb != 0) {
                    if(mod_type == LIBLTE_PHY_MODULATION_TYPE_QPSK) {
                        *mcs = I_tbs;
                    }
                    else if(mod_type == LIBLTE_PHY_MODULATION_TYPE_16QAM) {
                        *mcs = I_tbs + 1;
                    }
                    else if(mod_type == LIBLTE_PHY_MODULATION_TYPE_64QAM) {
                        *mcs = I_tbs + 2;
                    }
                    else {
                        *mcs = I_tbs;
                    }
                    break;
                }
            }

            if(*N_prb != 0) {
                err = LIBLTE_SUCCESS;
            }

        }
    }

    return(err);
}


int32_t check_decode_pdcch_channel
(
    LIBLTE_PHY_STRUCT              *phy_struct,
    LIBLTE_PHY_SUBFRAME_STRUCT     *subframe,
    uint32                          N_id_cell,
    uint8                           N_ant,
    float                           phich_res,
    LIBLTE_RRC_PHICH_DURATION_ENUM  phich_dur,
    LIBLTE_PHY_PCFICH_STRUCT       *pcfich,
    LIBLTE_PHY_PHICH_STRUCT        *phich,
    LIBLTE_PHY_PDCCH_STRUCT        *pdcch
)
{
    int32_t result = LIBLTE_ERROR_INVALID_INPUTS;

    int32_t         i;
    int32_t         j;
    int32_t         k;
    int32_t         p;

    uint32_t        N_bits;
    uint32_t        idx;

    uint16          rnti = 0;
    int32_t         dci_1a_size;
    int32_t         dci_1c_size;
    int32_t         dci_len;
    int32_t         dci_foramt;
    int32_t         conv_code_rate = 3;

    int32_t         L;
    int32_t         agg_lvl_idx;
    int32_t         show_bits;
    int32_t         cce_start_index;
    int32_t         cce_index;
    int32_t         Ncce_div_L;

    int32_t         num_of_sub_carriers;
    struct dci_format_s dci;

    int32_t         log_msg_enabled;
    int32_t         pdcch_cce_alloc_idx;
    int32_t         ca_presence;

    log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDCCH);
    num_of_sub_carriers = phy_struct->N_rb_dl*phy_struct->N_sc_rb_dl;

    if(phy_struct != NULL &&  subframe != NULL && pcfich != NULL && phich != NULL && pdcch != NULL) {

        pcfich_channel_demap(phy_struct, subframe, N_id_cell, N_ant, pcfich, &N_bits);
        if(LIBLTE_SUCCESS != cfi_channel_decode(phy_struct, phy_struct->pdcch_descramb_bits, N_bits, &pcfich->cfi))  {
            if(log_msg_enabled) {
                fprintf(stderr, "LIBLTE_ERROR_INVALID_CRC\n");
            }
            result = LIBLTE_ERROR_INVALID_CRC;
            return result;
        }

        // PHICH
        phich_channel_demap(phy_struct, pcfich, subframe, N_id_cell, N_ant, phich_res, phich_dur, phich);
        pdcch->N_symbs = pcfich->cfi;
        if(phy_struct->N_rb_dl <= 10) {
            pdcch->N_symbs++;
        }

        // PDCCH
        // Calculate number of symbols, 3GPP TS 36.211 v10.1.0 section 6.7
        // Calculate resources, 3GPP TS 36.211 v10.1.0 section 6.8.1
        liblte_phy_pdcch_channel_decode_n2
        (
            phy_struct,
            subframe,
            N_id_cell,
            N_ant,
            pcfich,
            phich,
            pdcch
        );

        if(log_msg_enabled) {
            fprintf(stderr, "\n");
            fprintf(stderr, "[DCI]subframe = %d, n_cce_pdcch = %d\n", subframe->num, phy_struct->n_cce_pdcch);
        }

        phy_struct->pdcch_cce_allocated = 0;

        /* DCI Format 0, Format 1A, Format 3 or Format 3A */
        if(log_msg_enabled) {
            fprintf(stderr, "[DCI]Find DCI Format 0, Format 1A, Format 3 or Format 3A in a common search space\n");
        }

        ca_presence = LIBLTE_PHY_DCI_CA_NOT_PRESENT;

        dci_len = dci_format_0_1a_3_3a_fdd_len(phy_struct->N_rb_dl);
        check_common_search_space(phy_struct, N_ant, 4, 8, LTE_DCI_FORMAT_1A, dci_len);

        if(log_msg_enabled) {
            fprintf(stderr, "[DCI]Find DCI Format 1C in a common search space\n");
        }
        dci_len = dci_format_1c_fdd_len(phy_struct->N_rb_dl);
        check_common_search_space(phy_struct, N_ant, 4, 8, LTE_DCI_FORMAT_1C, dci_len);

#if 0
        if(log_msg_enabled) {
            fprintf(stderr, "[DCI]Find DCI Format 0, Format 1A, Format 3 or Format 3A in an UE specific search space\n");
        }        
        dci_len = dci_format_0_1a_3_3a_fdd_len(phy_struct->N_rb_dl);
        check_ue_specific_search_space(phy_struct, N_ant, subframe->num, 1, 8, LTE_DCI_FORMAT_0, dci_len);
#endif        

        dci_len = dci_format_1_fdd_len(phy_struct->N_rb_dl);
        check_ue_specific_search_space(phy_struct, N_ant, subframe->num, 1, 8, LTE_DCI_FORMAT_1, dci_len);

#if 0
        dci_len = dci_format_2_fdd_len(phy_struct->N_rb_dl, N_ant);
        check_ue_specific_search_space(phy_struct, N_ant, subframe->num, 1, 8, LTE_DCI_FORMAT_2, dci_len);
        
        dci_len = dci_format_2a_fdd_len(phy_struct->N_rb_dl, N_ant);
        check_ue_specific_search_space(phy_struct, N_ant, subframe->num, 1, 8, LTE_DCI_FORMAT_2A, dci_len);
#endif

        pdcch->N_alloc = 0;
        for(pdcch_cce_alloc_idx = 0; pdcch_cce_alloc_idx < phy_struct->pdcch_cce_allocated; pdcch_cce_alloc_idx++ ) {

            cce_index = phy_struct->pdcch_cce_start_index[pdcch_cce_alloc_idx];
            rnti = phy_struct->pdcch_cce_rnti[pdcch_cce_alloc_idx];
            L = phy_struct->pdcch_cce_agg_level[pdcch_cce_alloc_idx];
            dci_foramt = phy_struct->pdcch_dci_format[pdcch_cce_alloc_idx];
            dci_len = phy_struct->pdcch_dci_len[pdcch_cce_alloc_idx];

            if(pdcch->N_alloc >= LIBLTE_PHY_PDCCH_MAX_ALLOC) {
                fprintf(stderr, "\n");
                fprintf(stderr, "Too many DCI\n");
                continue;
            }            

            result = LIBLTE_ERROR_DECODE_FAIL;
            decode_cce(phy_struct, N_ant, L, cce_index, dci_len, &rnti);

            fprintf(stderr, "\n");
            fprintf(stderr, "DCI bits, bit_len = %d : ", dci_len);
            print_bits_to_bytes(phy_struct->pdcch_dci, dci_len);
            print_bits(phy_struct->pdcch_dci, dci_len);

            fprintf(stderr, "cce_start[%03d] ~ cce_end[%03d], rnti = 0x%04x (%d)\n", cce_index, cce_index + L - 1, rnti, rnti);
            fprintf(stderr, "    DCI length = %d\n", dci_len);
            if(dci_foramt == LTE_DCI_FORMAT_1A) {
                
                int32_t dci_format_1a_len = dci_format_0_1a_3_3a_fdd_len(phy_struct->N_rb_dl);

                fprintf(stderr, "    DCI fomat 0 or DCI fomat 1A or DCI fomat 3 or DCI fomat 3A\n");

                result = dci_0_1a_unpack_fdd(phy_struct->pdcch_dci,
                                    dci_len,
                                    LIBLTE_PHY_DCI_CA_NOT_PRESENT,
                                    rnti,
                                    phy_struct->N_rb_dl,
                                    phy_struct->N_rb_ul,
                                    N_ant,
                                    &pdcch->alloc[pdcch->N_alloc],
                                    &dci);

                                                   

            }
            else if(dci_foramt == LTE_DCI_FORMAT_1C) {
                fprintf(stderr, "    DCI fomat 1C\n");
            }
            else if(dci_foramt == LTE_DCI_FORMAT_0) {

                result = dci_0_1a_unpack_fdd(phy_struct->pdcch_dci,
                                    dci_len,
                                    LIBLTE_PHY_DCI_CA_NOT_PRESENT,
                                    rnti,
                                    phy_struct->N_rb_dl,
                                    phy_struct->N_rb_ul,
                                    N_ant,
                                    &pdcch->alloc[pdcch->N_alloc],
                                    &dci);

            }
            else if(dci_foramt == LTE_DCI_FORMAT_1) {
                fprintf(stderr, "    DCI fomat 1\n");
                result = dci_1_unpack_fdd(phy_struct->pdcch_dci,
                                    dci_len,
                                    LIBLTE_PHY_DCI_CA_NOT_PRESENT,
                                    rnti,
                                    phy_struct->N_rb_dl,
                                    N_ant,
                                    &pdcch->alloc[pdcch->N_alloc],
                                    (struct dci_format_1_fdd_s *)&dci.d);


            }            
            else if(dci_foramt == LTE_DCI_FORMAT_2) {
                
                fprintf(stderr, "    DCI fomat 2\n");
                result = dci_2_unpack_fdd(phy_struct->pdcch_dci,
                                    dci_len,
                                    LIBLTE_PHY_DCI_CA_NOT_PRESENT,
                                    rnti,
                                    phy_struct->N_rb_dl,
                                    N_ant,
                                    &pdcch->alloc[pdcch->N_alloc],
                                    (struct dci_format_2_fdd_s *)&dci.d);

            }
            else if(dci_foramt == LTE_DCI_FORMAT_2A) {
                fprintf(stderr, "    DCI fomat 2A\n");                
                result = dci_2a_unpack_fdd(phy_struct->pdcch_dci,
                                    dci_len,
                                    LIBLTE_PHY_DCI_CA_NOT_PRESENT,
                                    rnti,
                                    phy_struct->N_rb_dl,
                                    N_ant,
                                    &pdcch->alloc[pdcch->N_alloc],
                                    (struct dci_format_2a_fdd_s *)&dci.d);
            }

            if(result == LIBLTE_SUCCESS) {
                if(dci_foramt == LTE_DCI_FORMAT_1A || dci_foramt == LTE_DCI_FORMAT_1C || dci_foramt == LTE_DCI_FORMAT_1) {
                    pdcch->N_alloc = pdcch->N_alloc + 1;
                }
            }
            else {
                phy_struct->pdcch_cce_alloc_type[pdcch_cce_alloc_idx] = 3;
            }

        }

    }

    return result;

}

int32_t get_psch_first_sc(LIBLTE_PHY_STRUCT *phy_struct)
{
    int32_t first_sc;
    
    if(phy_struct->N_rb_dl == 6) {
        first_sc = 0;
    }
    else if(phy_struct->N_rb_dl == 15) {
        first_sc = (4*phy_struct->N_sc_rb_dl)+6;
    }
    else if(phy_struct->N_rb_dl == 25) {
        first_sc = (9*phy_struct->N_sc_rb_dl)+6;
    }
    else if(phy_struct->N_rb_dl == 50) {
        first_sc = 22*phy_struct->N_sc_rb_dl;
    }
    else if(phy_struct->N_rb_dl == 75) {
        first_sc = (34*phy_struct->N_sc_rb_dl)+6;
    }
    else { // phy_struct->N_rb_dl == 100
        first_sc = 47*phy_struct->N_sc_rb_dl;
    }
    return first_sc;
}

int32_t get_psch_last_sc(LIBLTE_PHY_STRUCT *phy_struct)
{
    int32_t last_sc;
    
    if(phy_struct->N_rb_dl == 6) {
        last_sc  = (6*phy_struct->N_sc_rb_dl)-1;
    }
    else if(phy_struct->N_rb_dl == 15) {
        last_sc  = (11*phy_struct->N_sc_rb_dl)-7;
    }
    else if(phy_struct->N_rb_dl == 25) {
        last_sc  = (16*phy_struct->N_sc_rb_dl)-7;
    }
    else if(phy_struct->N_rb_dl == 50) {
        last_sc  = (28*phy_struct->N_sc_rb_dl)-1;
    }
    else if(phy_struct->N_rb_dl == 75) {
        last_sc  = (41*phy_struct->N_sc_rb_dl)-7;
    }
    else { // phy_struct->N_rb_dl == 100
        last_sc  = (53*phy_struct->N_sc_rb_dl)-1;
    }

    return last_sc;
    
}

int32_t liblte_phy_get_pdsch_est
(
    LIBLTE_PHY_STRUCT               *phy_struct,
    LIBLTE_PHY_SUBFRAME_STRUCT      *subframe,
    uint32_t                        N_pdcch_symbs,
    uint32                          N_id_cell,
    uint8                           N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc
)
{
    uint32_t    first_sc;
    uint32_t    last_sc;
    uint32_t    idx;
    int32_t     L;
    uint32_t    prb_idx;
    int32_t     i;
    int32_t     j;
    uint32_t    p;

    first_sc = get_psch_first_sc(phy_struct);
    last_sc = get_psch_last_sc(phy_struct);

    idx = 0;
    for(L = N_pdcch_symbs; L < 14; L++) {                
        for(prb_idx = 0; prb_idx < alloc->N_prb; prb_idx++) {
            i = alloc->prb[L/7][prb_idx];                    
            for(j = 0; j < phy_struct->N_sc_rb_dl; j++)   {
                if(N_ant == 1 && (L % 7)  == 0 && (N_id_cell % 6) == (j % 6)) {
                    // Skip CRS
                }
                else if(N_ant == 1 &&  (L % 7)  == 4 &&  ((N_id_cell+3) % 6) == (j % 6)) {
                    // Skip CRS
                }
                else if((N_ant  == 2 || N_ant == 4) && ((L % 7) == 0  || (L % 7) == 4) && (N_id_cell % 3) == (j % 3)) {
                    // Skip CRS
                }
                else if(N_ant == 4 && (L % 7) == 1 && (N_id_cell % 3) == (j % 3)) {
                    // Skip CRS
                }
                else if(subframe->num == 0 && (i*phy_struct->N_sc_rb_dl+j) >= first_sc && (i*phy_struct->N_sc_rb_dl+j) <= last_sc  &&
                         L >= 7 && L <= 10) {
                    // Skip PBCH
                }
                else if((subframe->num == 0 || subframe->num == 5) && (i*phy_struct->N_sc_rb_dl+j) >= first_sc &&
                         (i*phy_struct->N_sc_rb_dl+j) <= last_sc && L == 6) {
                    // Skip PSS
                }
                else if((subframe->num == 0 || subframe->num == 5) && (i*phy_struct->N_sc_rb_dl+j) >= first_sc 
                    && (i*phy_struct->N_sc_rb_dl+j) <= last_sc && L == 5) {
                    // Skip SSS
                }
                else {
                    phy_struct->pdsch_y_est_re[idx] = subframe->rx_symb_re[L][i*phy_struct->N_sc_rb_dl+j];
                    phy_struct->pdsch_y_est_im[idx] = subframe->rx_symb_im[L][i*phy_struct->N_sc_rb_dl+j];
                    for(p = 0; p < N_ant; p++) {
                        phy_struct->pdsch_c_est_re[p][idx] = subframe->rx_ce_re[p][L][i*phy_struct->N_sc_rb_dl+j];
                        phy_struct->pdsch_c_est_im[p][idx] = subframe->rx_ce_im[p][L][i*phy_struct->N_sc_rb_dl+j];
                    }                
                    idx++;
                }
            }
        }
    }

    return idx;
    
}

int32_t check_decode_pdsch_channel
(
    LIBLTE_PHY_STRUCT             *phy_struct,
    LIBLTE_PHY_SUBFRAME_STRUCT    *subframe,
    LIBLTE_PHY_PDCCH_STRUCT       *pdcch,
    uint32_t                      N_pdcch_symbs,
    uint32_t                      N_id_cell,
    uint8_t                       N_ant
)
{

    uint32_t          p;
    uint32_t          prb_idx;
    uint32_t          c_init;
    uint32_t          M_layer_symb;
    uint32_t          M_symb;
    uint32_t          N_bits;

    int32_t pdcch_cce_alloc_idx;
    int32_t pdcch_alloc_type;
    int32_t cce_index;
    int32_t rnti;
    int32_t L;
    int32_t dci_foramt;
    int32_t dci_len;
    int32_t agg_level;
    uint32_t first_sc;
    uint32_t last_sc;
    uint32_t idx;
    int32_t i;
    int32_t j;
    int32_t show_bits;
    uint8_t *out_bits;
    uint32_t *N_out_bits;
    
    int32_t alloc_idx;
    LIBLTE_PHY_ALLOCATION_STRUCT  *alloc;
    LIBLTE_BIT_MSG_STRUCT         *rrc_msg;

    int32_t result = LIBLTE_ERROR_INVALID_INPUTS;

    int32_t log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_BCCH) | LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH);

    rrc_msg = (LIBLTE_BIT_MSG_STRUCT *)malloc(sizeof(LIBLTE_BIT_MSG_STRUCT));
    if(rrc_msg == NULL) {
        return LIBLTE_ERROR_INVALID_INPUTS;
    }

    result = LIBLTE_ERROR_DECODE_FAIL;
    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "check_decode_pdsch_channel\n");
        fprintf(stderr, "    Demodulates and decodes the Physical Downlink Shared Channel\n");
        fprintf(stderr, "    Document Reference: 3GPP TS 36.211 v10.1.0 sections 6.3 and 6.4\n");
        fprintf(stderr, "    N_pdcch_symbs = %d\n", N_pdcch_symbs);
        fprintf(stderr, "    N_id_cell = %d\n", N_id_cell);
        fprintf(stderr, "    N_ant = %d\n", N_ant);
    }
    
    if(log_msg_enabled) {
        fprintf(stderr, "\n");
        fprintf(stderr, "Found CCE for sumframe %d\n", subframe->num);
        fprintf(stderr, "   pdcch->N_alloc = %d\n", pdcch->N_alloc);
    }

    for(alloc_idx = 0; alloc_idx < pdcch->N_alloc; alloc_idx++)  {

        alloc = &pdcch->alloc[alloc_idx];
        if(log_msg_enabled) {
            fprintf(stderr, "\n");
            fprintf(stderr, "   alloc[%d], N_prb = %d\n", alloc_idx, alloc->N_prb);
        }        
        
        for(i = 0; i < alloc->N_prb; i++) {
            fprintf(stderr, "    [%2d] prb[0] = %d, prb[1] = %d\n", i, alloc->prb[0][i], alloc->prb[1][i]);
        }

        if(alloc->dci_format == LTE_DCI_FORMAT_1) {

            // Extract resource elements and channel estimate 3GPP TS 36.211 v10.1.0 section 6.3.5
            if(log_msg_enabled) {
                fprintf(stderr, "\nExtract resource elements and channel estimate 3GPP TS 36.211 v10.1.0 section 6.3.5\n");
                fprintf(stderr, "Determine first and last PBCH, PSS, and SSS subcarriers\n");
            }
            
            idx = liblte_phy_get_pdsch_est(phy_struct, subframe, N_pdcch_symbs, N_id_cell, N_ant, alloc);
            if(log_msg_enabled) {
                fprintf(stderr, "    Total = %d\n", idx);
            }
            
            pre_decoder_and_matched_filter_dl(phy_struct->pdsch_y_est_re,
                                              phy_struct->pdsch_y_est_im,
                                              phy_struct->pdsch_c_est_re[0],
                                              phy_struct->pdsch_c_est_im[0],
                                              5000,
                                              idx,
                                              N_ant,
                                              alloc->pre_coder_type,
                                              phy_struct->pdsch_x_re,
                                              phy_struct->pdsch_x_im,
                                              &M_layer_symb);           
            
            if(log_msg_enabled) {
                fprintf(stderr, "    M_layer_symb = %d\n", M_layer_symb);
            }

            
            layer_demapper_dl(phy_struct->pdsch_x_re,
                              phy_struct->pdsch_x_im,
                              M_layer_symb,
                              N_ant,
                              alloc->N_codewords,
                              alloc->pre_coder_type,
                              phy_struct->pdsch_d_re,
                              phy_struct->pdsch_d_im,
                              &M_symb);
            
            if(log_msg_enabled) {
                fprintf(stderr, "    M_symb = %d\n", M_symb);
            }        

            phy_struct->pdsch_n_mod_sym = M_symb;
            phy_struct->pdsch_mod_type = alloc->mod_type;
            memcpy(phy_struct->pdsch_mod_sym_re, phy_struct->pdsch_d_re, M_symb * sizeof(float));
            memcpy(phy_struct->pdsch_mod_sym_im, phy_struct->pdsch_d_im, M_symb * sizeof(float));
            
            modulation_demapper(phy_struct->pdsch_d_re,
                                phy_struct->pdsch_d_im,
                                M_symb,
                                alloc->mod_type,
                                phy_struct->pdsch_soft_bits,
                                &N_bits);
            
            if(log_msg_enabled) {
                fprintf(stderr, "    N_bits = %d, alloc->rnti = %d (0x%04x)\n", N_bits, alloc->rnti, alloc->rnti);
            }      
            
            c_init = (alloc->rnti << 14) | (0 << 13) | (subframe->num << 9) | N_id_cell;
            generate_prs_c(c_init, N_bits, phy_struct->pdsch_c);
            for(i = 0; i < N_bits; i++) {
                phy_struct->pdsch_descramb_bits[i] = (float)phy_struct->pdsch_soft_bits[i]*(1-2*(float)phy_struct->pdsch_c[i]);
            }

#if 1             
            if(log_msg_enabled) {
                fprintf(stderr, "    subframe->num = %d, N_id_cell = %d, alloc->tbs = %d, alloc->tx_mode = %d, alloc->rv_idx = %d\n", 
                    subframe->num, N_id_cell, alloc->tbs, alloc->tx_mode, alloc->rv_idx);
            }         
            
            result = check_decode_dlsch_channel(phy_struct,
                                                phy_struct->pdsch_descramb_bits,
                                                N_bits,
                                                alloc->tbs,
                                                alloc->tx_mode,
                                                alloc->rv_idx,
                                                8,
                                                250368, // FIXME: Using N_soft from a cat 1 UE (3GPP TS 36.306)
                                                rrc_msg->msg,
                                                &rrc_msg->N_bits);  

#endif

            if(log_msg_enabled) {
                fprintf(stderr, "\n");
                fprintf(stderr, "LTE_DCI_FORMAT_1\n");
                fprintf(stderr, "    rnti = 0x%04x (%d)\n", alloc->rnti, alloc->rnti);
                fprintf(stderr, "\n");
                fprintf(stderr, "    MCS = %d\n", alloc->mcs);
                fprintf(stderr, "    Modulation Type = %d (%s)\n", alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
                fprintf(stderr, "    Transport block size = %d\n", alloc->tbs);
                fprintf(stderr, "\n");
                fprintf(stderr, "    Redundancy version = %d\n", alloc->rv_idx);               
            }
            
            if(result != LIBLTE_SUCCESS) {
                continue;
            }
            
        }
    
        if(alloc->dci_format == LTE_DCI_FORMAT_1C) {

            if(log_msg_enabled) {
                fprintf(stderr, "\n");
                fprintf(stderr, "LTE_DCI_FORMAT_1C\n");
            }
            
            if(result != LIBLTE_SUCCESS) {
                continue;
            }
        }

        if(alloc->dci_format == LTE_DCI_FORMAT_1A) {

            // Extract resource elements and channel estimate 3GPP TS 36.211 v10.1.0 section 6.3.5
            if(log_msg_enabled) {
                fprintf(stderr, "\nExtract resource elements and channel estimate 3GPP TS 36.211 v10.1.0 section 6.3.5\n");
                fprintf(stderr, "Determine first and last PBCH, PSS, and SSS subcarriers\n");
            }
            
            idx = liblte_phy_get_pdsch_est(phy_struct, subframe, N_pdcch_symbs, N_id_cell, N_ant, alloc);
            if(log_msg_enabled) {
                fprintf(stderr, "    Total = %d\n", idx);
            }
            
            pre_decoder_and_matched_filter_dl(phy_struct->pdsch_y_est_re,
                                              phy_struct->pdsch_y_est_im,
                                              phy_struct->pdsch_c_est_re[0],
                                              phy_struct->pdsch_c_est_im[0],
                                              5000,
                                              idx,
                                              N_ant,
                                              alloc->pre_coder_type,
                                              phy_struct->pdsch_x_re,
                                              phy_struct->pdsch_x_im,
                                              &M_layer_symb);           
            
            if(log_msg_enabled) {
                fprintf(stderr, "    M_layer_symb = %d\n", M_layer_symb);
            }
            
            layer_demapper_dl(phy_struct->pdsch_x_re,
                              phy_struct->pdsch_x_im,
                              M_layer_symb,
                              N_ant,
                              alloc->N_codewords,
                              alloc->pre_coder_type,
                              phy_struct->pdsch_d_re,
                              phy_struct->pdsch_d_im,
                              &M_symb);
            
            if(log_msg_enabled) {
                fprintf(stderr, "    M_symb = %d\n", M_symb);
            }        
            
            phy_struct->pdsch_n_mod_sym = M_symb;
            phy_struct->pdsch_mod_type = alloc->mod_type;
            memcpy(phy_struct->pdsch_mod_sym_re, phy_struct->pdsch_d_re, M_symb * sizeof(float));
            memcpy(phy_struct->pdsch_mod_sym_im, phy_struct->pdsch_d_im, M_symb * sizeof(float));
            
            modulation_demapper(phy_struct->pdsch_d_re,
                                phy_struct->pdsch_d_im,
                                M_symb,
                                alloc->mod_type,
                                phy_struct->pdsch_soft_bits,
                                &N_bits);
            
            if(log_msg_enabled) {
                fprintf(stderr, "    N_bits = %d, alloc->rnti = %d (0x%04x)\n", N_bits, alloc->rnti, alloc->rnti);
            }      
            
            c_init = (alloc->rnti << 14) | (0 << 13) | (subframe->num << 9) | N_id_cell;
            generate_prs_c(c_init, N_bits, phy_struct->pdsch_c);
            for(i = 0; i < N_bits; i++) {
                phy_struct->pdsch_descramb_bits[i] = (float)phy_struct->pdsch_soft_bits[i]*(1-2*(float)phy_struct->pdsch_c[i]);
            }
            
            if(log_msg_enabled) {
                fprintf(stderr, "    subframe->num = %d, N_id_cell = %d, alloc->tbs = %d, alloc->tx_mode = %d, alloc->rv_idx = %d\n", 
                    subframe->num, N_id_cell, alloc->tbs, alloc->tx_mode, alloc->rv_idx);
            }         
            
            result = check_decode_dlsch_channel(phy_struct,
                                                phy_struct->pdsch_descramb_bits,
                                                N_bits,
                                                alloc->tbs,
                                                alloc->tx_mode,
                                                alloc->rv_idx,
                                                8,
                                                250368, // FIXME: Using N_soft from a cat 1 UE (3GPP TS 36.306)
                                                rrc_msg->msg,
                                                &rrc_msg->N_bits);        

            if(log_msg_enabled) {
                fprintf(stderr, "\n");
                fprintf(stderr, "LTE_DCI_FORMAT_1A\n");
                fprintf(stderr, "    rnti = 0x%04x (%d)\n", alloc->rnti, alloc->rnti);
                fprintf(stderr, "\n");
                fprintf(stderr, "    MCS = %d\n", alloc->mcs);
                fprintf(stderr, "    N_codewords = %d\n", alloc->N_codewords);
                fprintf(stderr, "    Modulation Type = %d (%s)\n", alloc->mod_type, liblte_phy_modulation_type_text[alloc->mod_type]);
                fprintf(stderr, "    Transport block size = %d\n", alloc->tbs);
                fprintf(stderr, "    N_prb = %d, %02d to %02d \n", alloc->N_prb, alloc->prb[0][0], alloc->prb[0][0] + alloc->N_prb - 1);
                fprintf(stderr, "\n");
                fprintf(stderr, "    Redundancy version = %d\n", alloc->rv_idx);
                fprintf(stderr, "    TX mode = %d\n", alloc->tx_mode);
                fprintf(stderr, "    New Data Indicator = %d\n", alloc->ndi);
            }

            if(result != LIBLTE_SUCCESS) {
                continue;
            }

            if(alloc->rnti == LIBLTE_MAC_SI_RNTI) {       

                LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT  *bcch_dlsch_msg;
                bcch_dlsch_msg = (LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT *)malloc(sizeof(LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT));

                if(log_msg_enabled) {
                    fprintf(stderr, "alloc->rnti : LIBLTE_MAC_SI_RNTI\n");
                }                

                if(bcch_dlsch_msg != NULL) {
                    result = liblte_rrc_unpack_bcch_dlsch_msg(rrc_msg, bcch_dlsch_msg);
                    if(result == LIBLTE_SUCCESS) {
                        if(log_msg_enabled) {
                            fprintf(stderr, "bcch_dlsch_msg.N_sibs = %d\n", bcch_dlsch_msg->N_sibs);
                        }

                        for(i = 0; i < bcch_dlsch_msg->N_sibs; i++) {
                            if(log_msg_enabled) {
                                fprintf(stderr, "ctx->bcch_dlsch_msg.sibs[%d].sib_type = %d (%s)\n", 
                                    i, bcch_dlsch_msg->sibs[i].sib_type, liblte_rrc_sys_info_block_type_text[bcch_dlsch_msg->sibs[i].sib_type]);
                            }
                            switch(bcch_dlsch_msg->sibs[i].sib_type) {
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1:
                                print_bcch_dlsch_msg_sib1((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2:
                                print_bcch_dlsch_msg_sib2((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3:
                                print_bcch_dlsch_msg_sib3((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4:
                                print_bcch_dlsch_msg_sib4((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5:
                                print_bcch_dlsch_msg_sib5((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6:
                                print_bcch_dlsch_msg_sib6((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7:
                                print_bcch_dlsch_msg_sib7((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8:
                                print_bcch_dlsch_msg_sib8((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13:
                                print_bcch_dlsch_msg_sib13((LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13_STRUCT *)&bcch_dlsch_msg->sibs[i].sib);
                                break;
                            default:
                                if(log_msg_enabled) {
                                    fprintf(stderr, "Not handling SIB %u\n", bcch_dlsch_msg->sibs[i].sib_type);
                                }
                                break;
                            }
                        }

                    }

                    free(bcch_dlsch_msg);
                }
            }
            else if(alloc->rnti == LIBLTE_MAC_P_RNTI) {

                LIBLTE_RRC_PCCH_MSG_STRUCT    *pcch_msg;

                if(log_msg_enabled) {
                    fprintf(stderr, "alloc->rnti : LIBLTE_MAC_P_RNTI\n");
                }
                
                pcch_msg = (LIBLTE_RRC_PCCH_MSG_STRUCT *)malloc(sizeof(LIBLTE_RRC_PCCH_MSG_STRUCT));
                if(pcch_msg != NULL) {

                    result = liblte_rrc_unpack_pcch_msg(rrc_msg, pcch_msg);
                    if(result == LIBLTE_SUCCESS) {
                        print_pcch_page(pcch_msg);
                    }
                
                    free(pcch_msg);
                }
            }
            else if(alloc->rnti >= LIBLTE_MAC_RA_RNTI_START && alloc->rnti <= LIBLTE_MAC_RA_RNTI_END)  {

                if(log_msg_enabled) {
                    fprintf(stderr, "LIBLTE_MAC_RA_RNTI_START <= alloc->rnti < LIBLTE_MAC_RA_RNTI_END\n");
                }

            }
        }
    }

    if(rrc_msg != NULL) {
        free(rrc_msg);
    }
    
    return result;

}


int32_t check_decode_dlsch_channel
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *in_bits,
    uint32             N_in_bits,
    uint32             tbs,
    uint32             tx_mode,
    uint32             rv_idx,
    uint32             M_dl_harq,
    uint32             N_soft,
    uint8             *out_bits,
    uint32            *N_out_bits
)
{
    int32_t result = LIBLTE_ERROR_INVALID_CRC;
    uint32             i;
    uint32             cb;
    uint32             ber;
    uint32             N_b_bits;
    uint32             N_d_bits;
    uint32             N_fill_bits;
    uint32             N_codeblocks;
    uint8              calc_p_bits[24];
    uint8             *a_bits;
    uint8             *p_bits;

    int32_t log_msg_enabled;
    log_msg_enabled = LOG_MSG_ENABLE(LOG_MSG_CODE_PDSCH);


    // In order to decode a DLSCH message, the NULL bit pattern must be
    // determined by encoding a sequence of zeros
    N_b_bits = tbs+24;
    memset(phy_struct->dlsch_b_bits, 0, sizeof(uint8)*N_b_bits);
    liblte_phy_code_block_segmentation(phy_struct->dlsch_b_bits,
                                       N_b_bits,
                                       &N_codeblocks,
                                       &N_fill_bits,
                                       phy_struct->dlsch_c_bits[0],
                                       LIBLTE_PHY_MAX_CODE_BLOCK_SIZE,
                                       phy_struct->dlsch_N_c_bits);

    // Determine e_bits
    code_block_deconcatenation(in_bits,
                               N_in_bits,
                               tbs,
                               phy_struct->dlsch_rx_e_bits[0],
                               phy_struct->dlsch_N_e_bits,
                               18432,
                               &N_codeblocks);

    for(cb = 0; cb < N_codeblocks; cb++) {
        // Construct dummy_d_bits
        turbo_encode(phy_struct,
                     phy_struct->dlsch_c_bits[cb],
                     phy_struct->dlsch_N_c_bits[cb],
                     N_fill_bits,
                     phy_struct->dlsch_tx_d_bits,
                     &N_d_bits);

        // Determine d_bits
        liblte_phy_rate_unmatch_turbo(phy_struct,
                                      phy_struct->dlsch_rx_e_bits[cb],
                                      phy_struct->dlsch_N_e_bits[cb],
                                      phy_struct->dlsch_tx_d_bits,
                                      N_d_bits/3,
                                      N_codeblocks,
                                      tx_mode,
                                      N_soft,
                                      M_dl_harq,
                                      LIBLTE_PHY_CHAN_TYPE_DLSCH,
                                      rv_idx,
                                      phy_struct->dlsch_rx_d_bits,
                                      &N_d_bits);

        // Determine c_bits
        turbo_decode(phy_struct,
                     phy_struct->dlsch_rx_d_bits,
                     N_d_bits,
                     N_fill_bits,
                     phy_struct->dlsch_c_bits[cb],
                     &phy_struct->dlsch_N_c_bits[cb]);
    }

    // Determine b_bits
    liblte_phy_code_block_desegmentation(phy_struct->dlsch_c_bits[0],
                                         phy_struct->dlsch_N_c_bits,
                                         LIBLTE_PHY_MAX_CODE_BLOCK_SIZE,
                                         tbs,
                                         phy_struct->dlsch_b_bits,
                                         N_b_bits);

    // Recover a_bits and p_bits
    a_bits = &phy_struct->dlsch_b_bits[0];
    p_bits = &phy_struct->dlsch_b_bits[tbs];

    // Calculate p_bits
    calc_crc(a_bits, tbs, CRC24A, calc_p_bits, 24);

    // Check CRC
    ber = 0;
    for(i = 0; i < 24; i++) {
        ber += p_bits[i] ^ calc_p_bits[i];
    }

    if(log_msg_enabled) {
        fprintf(stderr, "[DLSCH] ber = %d\n", ber);
    }

    if(ber == 0) {
        for(i = 0; i < tbs; i++) {
            out_bits[i] = a_bits[i];
        }
        *N_out_bits = tbs;
        result         = LIBLTE_SUCCESS;

        if(log_msg_enabled) {
            fprintf(stderr, " N_out_bits = %d\n", *N_out_bits);
            print_bits(out_bits, *N_out_bits);            
        }
    }

    return result;
    
}

void print_bcch_dlsch_msg_sib1(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *sib1)
{
    uint32 i;
    uint32 j;
    uint32 si_win_len;
    uint32 si_periodicity_T;
    uint16 mnc;

    fprintf(stderr, "SIB1 Decoded:\n");
    fprintf(stderr, "    %-40s\n", "PLMN Identity List:");
    for(i = 0; i < sib1->N_plmn_ids; i++) {
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

}


void print_bcch_dlsch_msg_sib2(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *sib2)
{
    uint32 coeff = 0;
    uint32 T     = 0;
    uint32 i;


    fprintf(stderr, "SIB2 Decoded:\n");
    if(true == sib2->ac_barring_info_present) {
        if(true == sib2->ac_barring_for_emergency) {
            fprintf(stderr, "    %-40s=%20s\n", "AC Barring for Emergency", "Barred");
        }
        else {
            fprintf(stderr, "    %-40s=%20s\n", "AC Barring for Emergency", "Not Barred");
        }
        if(true == sib2->ac_barring_for_mo_signalling.enabled) {
            fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Signalling", "Barred");
            fprintf(stderr, "        %-40s=%20s\n", "Factor", liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_signalling.factor]);
            fprintf(stderr, "        %-40s=%19ss\n", "Time", liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_signalling.time]);
            fprintf(stderr, "        %-40s=%20u\n", "Special AC", sib2->ac_barring_for_mo_signalling.for_special_ac);
        }
        else {
            fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Signalling", "Not Barred");
        }

        if(true == sib2->ac_barring_for_mo_data.enabled) {
            fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Data", "Barred");
            fprintf(stderr, "        %-40s=%20s\n", "Factor", liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_data.factor]);
            fprintf(stderr, "        %-40s=%19ss\n", "Time", liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_data.time]);
            fprintf(stderr, "        %-40s=%20u\n", "Special AC", sib2->ac_barring_for_mo_data.for_special_ac);
        }
        else {
            fprintf(stderr, "    %-40s=%20s\n", "AC Barring for MO Data", "Not Barred");
        }
    }
    
    fprintf(stderr, "    %-40s=%20s\n", "Number of RACH Preambles", liblte_rrc_number_of_ra_preambles_text[sib2->rr_config_common_sib.rach_cnfg.num_ra_preambles]);
    if(true == sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present) {
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
    switch(sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index) {
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
    
    if(true == sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag) {
        fprintf(stderr, "    %-40s=%20s\n", "High Speed Flag", "Restricted Set");
    }
    else {
        fprintf(stderr, "    %-40s=%20s\n", "High Speed Flag", "Unrestricted Set");
    }
    fprintf(stderr, "    %-40s=%20u\n", "Ncs Configuration", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config);
    fprintf(stderr, "    %-40s=%20u\n", "PRACH Freq Offset", sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset);
    fprintf(stderr, "    %-40s=%17ddBm\n", "Reference Signal Power", sib2->rr_config_common_sib.pdsch_cnfg.rs_power);
    fprintf(stderr, "    %-40s=%20u\n", "Pb", sib2->rr_config_common_sib.pdsch_cnfg.p_b);
    fprintf(stderr, "    %-40s=%20u\n", "Nsb", sib2->rr_config_common_sib.pusch_cnfg.n_sb);
    switch(sib2->rr_config_common_sib.pusch_cnfg.hopping_mode) {
    case LIBLTE_RRC_HOPPING_MODE_INTER_SUBFRAME:
        fprintf(stderr, "    %-40s=%20s\n", "Hopping Mode", "Inter Subframe");
        break;
    case LIBLTE_RRC_HOPPING_MODE_INTRA_AND_INTER_SUBFRAME:
        fprintf(stderr, "    %-40s= %s\n", "Hopping Mode", "Intra and Inter Subframe");
        break;
    }
    fprintf(stderr, "    %-40s=%20u\n", "PUSCH Nrb Hopping Offset", sib2->rr_config_common_sib.pusch_cnfg.pusch_hopping_offset);
    if(true == sib2->rr_config_common_sib.pusch_cnfg.enable_64_qam) {
        fprintf(stderr, "    %-40s=%20s\n", "64QAM", "Allowed");
    }
    else {
        fprintf(stderr, "    %-40s=%20s\n", "64QAM", "Not Allowed");
    }
    
    if(true == sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled) {
        fprintf(stderr, "    %-40s=%20s\n", "Group Hopping", "Enabled");
    }
    else {
        fprintf(stderr, "    %-40s=%20s\n", "Group Hopping", "Disabled");
    }
    fprintf(stderr, "    %-40s=%20u\n", "Group Assignment PUSCH", sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch);
    if(true == sib2->rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled) {
        fprintf(stderr, "    %-40s=%20s\n", "Sequence Hopping", "Enabled");
    }
    else {
        fprintf(stderr, "    %-40s=%20s\n", "Sequence Hopping", "Disabled");
    }
    fprintf(stderr, "    %-40s=%20u\n", "Cyclic Shift", sib2->rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift);
    fprintf(stderr, "    %-40s=%20s\n", "Delta PUCCH Shift", liblte_rrc_delta_pucch_shift_text[sib2->rr_config_common_sib.pucch_cnfg.delta_pucch_shift]);
    fprintf(stderr, "    %-40s=%20u\n", "N_rb_cqi", sib2->rr_config_common_sib.pucch_cnfg.n_rb_cqi);
    fprintf(stderr, "    %-40s=%20u\n", "N_cs_an", sib2->rr_config_common_sib.pucch_cnfg.n_cs_an);
    fprintf(stderr, "    %-40s=%20u\n", "N1 PUCCH AN", sib2->rr_config_common_sib.pucch_cnfg.n1_pucch_an);
    if(true == sib2->rr_config_common_sib.srs_ul_cnfg.present) {
        fprintf(stderr, "    %-40s=%20s\n", "SRS Bandwidth Config", liblte_rrc_srs_bw_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.bw_cnfg]);
        fprintf(stderr, "    %-40s=%20s\n", "SRS Subframe Config", liblte_rrc_srs_subfr_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.subfr_cnfg]);
        if(true == sib2->rr_config_common_sib.srs_ul_cnfg.ack_nack_simul_tx) {
            fprintf(stderr, "    %-40s=%20s\n", "Simultaneous AN and SRS", "True");
        }
        else {
            fprintf(stderr, "    %-40s=%20s\n", "Simultaneous AN and SRS", "False");
        }
        if(true == sib2->rr_config_common_sib.srs_ul_cnfg.max_up_pts_present) {
            fprintf(stderr, "    %-40s=%20s\n", "SRS Max Up PTS", "True");
        }
        else {
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
    switch(sib2->rr_config_common_sib.ul_cp_length) {
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
    if(true == sib2->arfcn_value_eutra.present) {
        fprintf(stderr, "    %-40s=%20u\n", "UL ARFCN", sib2->arfcn_value_eutra.value);
    }
    if(true == sib2->ul_bw.present) {
        fprintf(stderr, "    %-40s=%17sMHz\n", "UL Bandwidth", liblte_rrc_ul_bw_text[sib2->ul_bw.bw]);
    }
    fprintf(stderr, "    %-40s=%20u\n", "Additional Spectrum Emission", sib2->additional_spectrum_emission);
    if(0 != sib2->mbsfn_subfr_cnfg_list_size) {
        fprintf(stderr, "    %s:\n", "MBSFN Subframe Config List");
    }
    for(i=0; i<sib2->mbsfn_subfr_cnfg_list_size; i++) {
        fprintf(stderr, "        %-40s=%20s\n", "Radio Frame Alloc Period", liblte_rrc_radio_frame_allocation_period_text[sib2->mbsfn_subfr_cnfg[i].radio_fr_alloc_period]);
        fprintf(stderr, "        %-40s=%20u\n", "Radio Frame Alloc Offset", sib2->mbsfn_subfr_cnfg[i].subfr_alloc);
        fprintf(stderr, "        Subframe Alloc%-26s=%20u\n", liblte_rrc_subframe_allocation_num_frames_text[sib2->mbsfn_subfr_cnfg[i].subfr_alloc_num_frames], sib2->mbsfn_subfr_cnfg[i].subfr_alloc);
    }
    fprintf(stderr, "    %-40s=%10s Subframes\n", "Time Alignment Timer", liblte_rrc_time_alignment_timer_text[sib2->time_alignment_timer]);

}


void print_bcch_dlsch_msg_sib3(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *sib3)
{

    fprintf(stderr, "SIB3 Decoded:\n");
    fprintf(stderr, "    %-40s=%18sdB\n", "Q-Hyst", liblte_rrc_q_hyst_text[sib3->q_hyst]);
    if(true == sib3->speed_state_resel_params.present) {
        fprintf(stderr, "    %-40s=%19ss\n", "T-Evaluation", liblte_rrc_t_evaluation_text[sib3->speed_state_resel_params.mobility_state_params.t_eval]);
        fprintf(stderr, "    %-40s=%19ss\n", "T-Hyst Normal", liblte_rrc_t_hyst_normal_text[sib3->speed_state_resel_params.mobility_state_params.t_hyst_normal]);
        fprintf(stderr, "    %-40s=%20u\n", "N-Cell Change Medium", sib3->speed_state_resel_params.mobility_state_params.n_cell_change_medium);
        fprintf(stderr, "    %-40s=%20u\n", "N-Cell Change High", sib3->speed_state_resel_params.mobility_state_params.n_cell_change_high);
        fprintf(stderr, "    %-40s=%18sdB\n", "Q-Hyst SF Medium", liblte_rrc_sf_medium_text[sib3->speed_state_resel_params.q_hyst_sf.medium]);
        fprintf(stderr, "    %-40s=%18sdB\n", "Q-Hyst SF High", liblte_rrc_sf_high_text[sib3->speed_state_resel_params.q_hyst_sf.high]);
    }
    if(true == sib3->s_non_intra_search_present) {
        fprintf(stderr, "    %-40s=%18udB\n", "S-Non Intra Search", sib3->s_non_intra_search);
    }
    fprintf(stderr, "    %-40s=%18udB\n", "Threshold Serving Low", sib3->thresh_serving_low);
    fprintf(stderr, "    %-40s=%20u\n", "Cell Reselection Priority", sib3->cell_resel_prio);
    fprintf(stderr, "    %-40s=%17ddBm\n", "Q Rx Lev Min", sib3->q_rx_lev_min);
    if(true == sib3->p_max_present) {
        fprintf(stderr, "    %-40s=%17ddBm\n", "P Max", sib3->p_max);
    }
    if(true == sib3->s_intra_search_present) {
        fprintf(stderr, "    %-40s=%18udB\n", "S-Intra Search", sib3->s_intra_search);
    }
    if(true == sib3->allowed_meas_bw_present) {
        fprintf(stderr, "    %-40s=%17sMHz\n", "Allowed Meas Bandwidth", liblte_rrc_allowed_meas_bandwidth_text[sib3->allowed_meas_bw]);
    }
    if(true == sib3->presence_ant_port_1) {
        fprintf(stderr, "    %-40s=%20s\n", "Presence Antenna Port 1", "True");
    }
    else{
        fprintf(stderr, "    %-40s=%20s\n", "Presence Antenna Port 1", "False");
    }
    switch(sib3->neigh_cell_cnfg) {
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
    if(true == sib3->t_resel_eutra_sf_present) {
        fprintf(stderr, "    %-40s=%20s\n", "T-Reselection EUTRA SF Medium", liblte_rrc_sssf_medium_text[sib3->t_resel_eutra_sf.sf_medium]);
        fprintf(stderr, "    %-40s=%20s\n", "T-Reselection EUTRA SF High", liblte_rrc_sssf_high_text[sib3->t_resel_eutra_sf.sf_high]);
    }

}

void print_bcch_dlsch_msg_sib4(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *sib4)
{
    uint32 i;
    uint32 stop;

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
    
}

void print_bcch_dlsch_msg_sib5(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *sib5)
{
    uint32_t i;
    uint32_t j;
    uint16_t stop;

    fprintf(stderr, "SIB5 Decoded:\n");
    fprintf(stderr, "    List of inter-frequency neighboring cells:\n");
    for(i = 0; i < sib5->inter_freq_carrier_freq_list_size; i++) {
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
            for(j = 0; j < sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list_size; j++) {
                fprintf(stderr, "            %-40s=%20u\n", "Physical Cell ID", sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list[j].phys_cell_id);
                fprintf(stderr, "            %-40s=%18sdB\n", "Q Offset Cell", liblte_rrc_q_offset_range_text[sib5->inter_freq_carrier_freq_list[i].inter_freq_neigh_cell_list[j].q_offset_cell]);
            }
        }
        if(0 != sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list_size) {
            fprintf(stderr, "        List of blacklisted inter-frequency neighboring cells\n");
            for(j = 0; j < sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list_size; j++) {
                fprintf(stderr, "            %u - %u\n", sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].start, sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].start + liblte_rrc_phys_cell_id_range_num[sib5->inter_freq_carrier_freq_list[i].inter_freq_black_cell_list[j].range]);
            }
        }
    }

}

void print_bcch_dlsch_msg_sib6(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *sib6)
{
    uint32 i;

    fprintf(stderr, "SIB6 Decoded:\n");
    
    if(0 != sib6->carrier_freq_list_utra_fdd_size) {
        fprintf(stderr, "    %s:\n", "Carrier Freq List UTRA FDD");
    }
    for(i = 0; i < sib6->carrier_freq_list_utra_fdd_size; i++) {
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

    for(i = 0; i < sib6->carrier_freq_list_utra_tdd_size; i++) {
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

}

void print_bcch_dlsch_msg_sib7(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *sib7)
{
    uint32_t i;
    uint32_t j;

    fprintf(stderr, "SIB7 Decoded:\n");
    fprintf(stderr, "    %-40s=%19us\n", "T-Reselection GERAN", sib7->t_resel_geran);
    if(true == sib7->t_resel_geran_sf_present) {
        fprintf(stderr, "    %-40s=%20s\n", "T-Reselection GERAN SF Medium", liblte_rrc_sssf_medium_text[sib7->t_resel_geran_sf.sf_medium]);
        fprintf(stderr, "    %-40s=%20s\n", "T-Reselection GERAN SF High", liblte_rrc_sssf_high_text[sib7->t_resel_geran_sf.sf_high]);
    }
    if(0 != sib7->carrier_freqs_info_list_size) {
        fprintf(stderr, "    List of neighboring GERAN carrier frequencies\n");
    }
    
    for(i = 0; i < sib7->carrier_freqs_info_list_size; i++) {
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

}

void print_bcch_dlsch_msg_sib8(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *sib8)
{
    uint32_t i;
    uint32_t j;
    uint32_t k;


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
        else {
            fprintf(stderr, "    %-40s=%20s\n", "Pre Registration", "Not Allowed");
        }
        if(true == sib8->pre_reg_info_hrpd.pre_reg_zone_id_present) {
            fprintf(stderr, "    %-40s=%20u\n", "Pre Registration Zone ID", sib8->pre_reg_info_hrpd.pre_reg_zone_id);
        }
        if(0 != sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list_size) {
            fprintf(stderr, "    Secondary Pre Registration Zone IDs:\n");
        }
        for(i = 0; i < sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list_size; i++) {
            fprintf(stderr, "        %u\n", sib8->pre_reg_info_hrpd.secondary_pre_reg_zone_id_list[i]);
        }
        
        if(true == sib8->cell_resel_params_hrpd_present) {
            fprintf(stderr, "    Band Class List:\n");
            for(i = 0; i < sib8->cell_resel_params_hrpd.band_class_list_size; i++) {
                fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_hrpd.band_class_list[i].band_class]);
                if(true == sib8->cell_resel_params_hrpd.band_class_list[i].cell_resel_prio_present) {
                    fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib8->cell_resel_params_hrpd.band_class_list[i].cell_resel_prio);
                }
                fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib8->cell_resel_params_hrpd.band_class_list[i].thresh_x_high);
                fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib8->cell_resel_params_hrpd.band_class_list[i].thresh_x_low);
            }
            fprintf(stderr, "    Neighbor Cell List:\n");
            for(i = 0; i < sib8->cell_resel_params_hrpd.neigh_cell_list_size; i++) {
                fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_hrpd.neigh_cell_list[i].band_class]);
                fprintf(stderr, "        Neighbor Cells Per Frequency List\n");
                for(j = 0; j < sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list_size; j++) {
                    fprintf(stderr, "            %-40s=%20u\n", "ARFCN", sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].arfcn);
                    fprintf(stderr, "            Phys Cell ID List\n");
                    for(k = 0; k < sib8->cell_resel_params_hrpd.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list_size; k++) {
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
            for(i = 0; i < sib8->cell_resel_params_1xrtt.band_class_list_size; i++) {
                fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_1xrtt.band_class_list[i].band_class]);
                if(true == sib8->cell_resel_params_1xrtt.band_class_list[i].cell_resel_prio_present) {
                    fprintf(stderr, "        %-40s=%20u\n", "Cell Reselection Priority", sib8->cell_resel_params_1xrtt.band_class_list[i].cell_resel_prio);
                }
                fprintf(stderr, "        %-40s=%20u\n", "Threshold X High", sib8->cell_resel_params_1xrtt.band_class_list[i].thresh_x_high);
                fprintf(stderr, "        %-40s=%20u\n", "Threshold X Low", sib8->cell_resel_params_1xrtt.band_class_list[i].thresh_x_low);
            }
            fprintf(stderr, "    Neighbor Cell List:\n");
            for(i = 0; i < sib8->cell_resel_params_1xrtt.neigh_cell_list_size; i++) {
                fprintf(stderr, "        %-40s=%20s\n", "Band Class", liblte_rrc_band_class_cdma2000_text[sib8->cell_resel_params_1xrtt.neigh_cell_list[i].band_class]);
                fprintf(stderr, "        Neighbor Cells Per Frequency List\n");
                for(j = 0; j < sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list_size; j++) {
                    fprintf(stderr, "            %-40s=%20u\n", "ARFCN", sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].arfcn);
                    fprintf(stderr, "            Phys Cell ID List\n");
                    for(k = 0; k < sib8->cell_resel_params_1xrtt.neigh_cell_list[i].neigh_cells_per_freq_list[j].phys_cell_id_list_size; k++) {
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

}

void print_bcch_dlsch_msg_sib13(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13_STRUCT *sib13)
{
    uint32_t i;

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

}


void print_pcch_page(LIBLTE_RRC_PAGING_STRUCT *page)
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

