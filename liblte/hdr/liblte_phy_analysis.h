/*******************************************************************************

    Copyright 2012-2017 Ben Wojtowicz

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*******************************************************************************

    File: liblte_phy.h

    Description: Contains all the definitions for the LTE Physical Layer
                 library.

    Revision History
    ----------    -------------    --------------------------------------------
    02/26/2012    Ben Wojtowicz    Created file.
    04/21/2012    Ben Wojtowicz    Added Turbo encode/decode and PDSCH decode
    05/28/2012    Ben Wojtowicz    Rearranged to match spec order, added
                                   initial transmit functionality, and fixed
                                   a bug related to PDCCH sizes.
    06/11/2012    Ben Wojtowicz    Enabled fftw input, output, and plan in
                                   phy_struct.
    10/06/2012    Ben Wojtowicz    Added random access and paging PDCCH
                                   decoding, soft turbo decoding, and PDSCH
                                   decoding per allocation.
    11/10/2012    Ben Wojtowicz    Added TBS, MCS, and N_prb calculations for
                                   SI PDSCH messages, added more sample defines,
                                   and re-factored the coarse timing and freq
                                   search to find more than 1 eNB.
    12/01/2012    Ben Wojtowicz    Added ability to preconfigure CRS.
    12/26/2012    Ben Wojtowicz    Started supporting N_sc_rb for normal and
                                   extended CP and fixed several FIXMEs.
    03/03/2013    Ben Wojtowicz    Added support for multiple sampling rates,
                                   pre-permutation of PDCCH encoding, and
                                   integer frequency offset detection.
    03/17/2013    Ben Wojtowicz    Moved to single float version of fftw.
    07/21/2013    Ben Wojtowicz    Added routines for determining TBS, MCS,
                                   N_prb, and N_cce.
    08/26/2013    Ben Wojtowicz    Added PRACH generation and detection support
                                   and changed ambiguous routines/variables to
                                   be non-ambiguous.
    09/16/2013    Ben Wojtowicz    Implemented routines for determine TBS, MCS,
                                   N_prb, and N_cce.
    09/28/2013    Ben Wojtowicz    Reordered sample rate enum and added a text
                                   version.
    11/13/2013    Ben Wojtowicz    Started adding PUSCH functionality.
    03/26/2014    Ben Wojtowicz    Added PUSCH functionality.
    04/12/2014    Ben Wojtowicz    Added support for PRB allocation differences
                                   in each slot.
    05/04/2014    Ben Wojtowicz    Added PHICH and TPC support.
    06/15/2014    Ben Wojtowicz    Added TPC values for DCI 0, 3, and 4.
    07/14/2015    Ben Wojtowicz    Added a constant definition of Fs as an
                                   integer.
    12/06/2015    Ben Wojtowicz    Added defines for many of the array sizes in
                                   LIBLTE_PHY_STRUCT and corrected the order of
                                   sizes in the rate match/unmatch arrays in
                                   LIBLTE_PHY_STRUCT (thanks to Ziming He for
                                   finding this).
    02/13/2016    Ben Wojtowicz    Moved turbo coder rate match/unmatch and
                                   code block segmentation/desegmentation to
                                   globally available routines to support unit
                                   tests.
    03/12/2016    Ben Wojtowicz    Added PUCCH channel decode support.
    07/03/2016    Ben Wojtowicz    Added PDCCH size defines.
    07/31/2016    Ben Wojtowicz    Added harq_retx_count to allocation struct.
    07/29/2017    Ben Wojtowicz    Added two codeword support, refactored PUCCH
                                   channel decoding for PUCCH types 1, 1A, and
                                   1B, and added a function to map the SR
                                   configuration index.

*******************************************************************************/

#ifndef __LIBLTE_PHY_ANALYSIS_H__
#define __LIBLTE_PHY_ANALYSIS_H__

/*******************************************************************************
                              INCLUDES
*******************************************************************************/

#include "liblte_common.h"
#include "liblte_rrc.h"
#include "fftw3.h"

#include <math.h>
#include <stdint.h>
#include <unistd.h>


#define N_SYMB_DL_NORMAL_CP 7


/*********************************************************************
    Name: dci_0_pack

    Description: Packs all of the fields into the Downlink Control
                 Information format 0

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.3.3.1.1
                        3GPP TS 36.213 v10.3.0 section 8.1.1
                        3GPP TS 36.213 v10.3.0 section 8.6

    Notes: Currently only handles non-hopping single-cluster
           assignments
*********************************************************************/
// Defines
#define DCI_0_1A_FLAG_0          0
#define DCI_0_1A_FLAG_1A         1
#define DCI_VRB_TYPE_LOCALIZED   0
#define DCI_VRB_TYPE_DISTRIBUTED 1

/*********************************************************************
    Name: calc_crc

    Description: Calculates one of the LTE CRCs

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.1
*********************************************************************/

#define CRC24A 0x01864CFB
#define CRC24B 0x01800063
#define CRC16  0x00011021
#define CRC8   0x0000019B

/*********************************************************************
    Name: rate_unmatch_conv

    Description: Rate unmatches convolutionally encoded data

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.4.2
*********************************************************************/
// Defines
#define RX_NULL_BIT 10000
#define TX_NULL_BIT 100



#define LTE_S_RNTIS     1
#define LTE_C_RNTIS     2

#define LTE_TM_1        0x0001
#define LTE_TM_2        0x0002
#define LTE_TM_3        0x0004
#define LTE_TM_4        0x0008
#define LTE_TM_5        0x0010
#define LTE_TM_6        0x0020
#define LTE_TM_7        0x0040

#define LTE_DCI_FORMAT_0    0x0000
#define LTE_DCI_FORMAT_1    0x0100
#define LTE_DCI_FORMAT_1A   0x0101
#define LTE_DCI_FORMAT_1B   0x0102
#define LTE_DCI_FORMAT_1C   0x0103
#define LTE_DCI_FORMAT_1D   0x0104
#define LTE_DCI_FORMAT_2    0x0200
#define LTE_DCI_FORMAT_2A   0x0201
#define LTE_DCI_FORMAT_3    0x0300
#define LTE_DCI_FORMAT_3A   0x0301

#define MAX_TPC_COMMAND         100
#define MAX_ANTENNA         4

extern uint32 TBS_71721[27][110];
extern uint8 IC_PERM_CC[32];
extern uint8 IC_PERM_TC[32];


struct nrb2bits_s {
    int32_t nrb;
    int32_t bits;
};

struct nrb2_bits_per_ant_s {
    int32_t nrb;
    int32_t bits[MAX_ANTENNA];
};


struct nrb2rashift_bit_s {
    int32_t nrb;
    int32_t nsubset;
    int32_t shit_bits[4];
};

#pragma pack(push, 1)

struct dci_format_0_1a_hdr_s {
    int16_t     ca_ind;                     // Carrier indicator
    int16_t     dci_0_1a_flag;              // Flag for format0/format1A differentiation
};


//
// 5.3.3.1.1 Format 0  in 3GPP TS 36.212
//
struct dci_format_0_fdd_s {

    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3].
    int16_t     ca_ind;                     //  3                : Carrier indicator

    // Flag for format0/format1A differentiation – 1 bit, where value 0 indicates format 0 and value 1 indicates format 1A
    int16_t     dci_0_1a_flag;              //  1                : Flag for format0/format1A differentiation

    // Frequency hopping flag – 1 bit as defined in section 8.4 of 3GPP TS 36.213 [3] . 
    // This field is used for multi-cluster allocations as the MSB of the corresponding resource allocation if needed.
    int32_t     freq_hopping_flag;          //  1                : Frequency hopping flag

    // Resource block assignment and hopping resource allocation –
    // For PUSCH hopping (single cluster allocation only):
    int32_t     N_ul_hop;                   //  1/ 1/ 1/ 2/ 2/ 2 : N_ULhop
    int32_t     riv;                        //  5/ 7/ 7/11/12/13 : Resource block assignment and hopping resource allocation

    // Modulation and coding scheme and redundancy version – 5 bits as defined in section 8.6 of 3GPP TS 36.213 [3]
    int32_t     mcs_and_redundancy_ver;     //  5                : Modulation and coding scheme and redundancy version

    // New data indicator – 1 bit    
    int32_t     ndi;                        //  1                : New data indicator

    // TPC command for scheduled PUSCH – 2 bits as defined in section 5.1.1.1 of [3]
    int32_t     tpc;                        //  2                : TPC command for scheduled PUSCH

    // Cyclic shift for DM RS and OCC index – 3 bits as defined in section 5.5.2.1.1 of 3GPP TS 36.211 [2]
    int32_t     cyclic_shift_for_dm_rs;     //  3                : Cyclic shift for DM RS and OCC index

    // CQI request – 1 or 2 bits as defined in section 7.2.1 of 3GPP TS 36.213 [3]. 
    // The 2-bit field only applies to UEs that are configured with more than one DL cell and when the corresponding DCI is mapped onto the UE specific by C-RNTI search
    // space as defined in 3GPP TS 36.213 [3]
    int32_t     cqi_req;                    //  1 or 2           : CQI request

    // SRS request – 0 or 1 bit. This field can only be present in DCI formats scheduling PUSCH which are mapped onto
    // the UE specific by C-RNTI search space as defined in 3GPP TS 36.213 [3]. The interpretation of this field is provided in section
    // [x] of 3GPP TS 36.213 [3]
    int32_t     srs_request;                //  0 or 1           : SRS request
    
    int32_t     multi_cluster_flag;         //  1                : Multi-cluster flag
                                            // 17/19/19/24/25/26
                                            // 17/18/19/20
                                            // 19/20/21/22
                                            // 24/25/26/27

    // -------------------------------------------------------  
    int32_t     hopping_type;

    int32_t     pad_byte;

    int32_t     riv_length; 
    int32_t     RB_start[2];
    int32_t     L_crbs[2];

                                            
};

//
// 5.3.3.1.2 Format 1 in 3GPP TS 36.212
// 
struct dci_format_1_fdd_s {
    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    int16_t     ca_ind;                     

    // Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.213 [3]
    int16_t     resource_alloc_type;              

    // Resource block assignment
    // For resource allocation type 0 as defined in section 7.1.6.1 of 3GPP TS 36.213 [3]:
    uint32_t    resource_block_assignment;

    // For resource allocation type 1 as defined in section 7.1.6.2 of 3GPP TS 36.213 [3]:
    uint16_t    subset;
    uint16_t    shift;

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    uint16_t    mcs;       
    
    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    uint16_t    harq_process;    

    // New data indicator – 1 bit
    uint16_t    ndi;                        

    // Redundancy version – 2 bits
    uint16_t    redundancy_version;         

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    uint16_t    tpc;

    // -------------------------------------------------------   
    uint16_t    N_rb_type1;
    uint16_t    N_rb_rbg_subset;
    uint16_t    delta_shift;
    uint16_t    prb_allocated;
    uint8_t     prb[LIBLTE_PHY_N_RB_DL_MAX];

    LIBLTE_PHY_MODULATION_TYPE_ENUM mod_type;

    // int32_t     N_codewords;
    uint16_t    rnti;
    uint16_t    i_tbs;
    int32_t     tbs;
    uint32_t    tx_mode; 

};

//
// 5.3.3.1.3 Format 1A in 3GPP TS 36.212
//
struct dci_format_1a_fdd_s {

    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    uint16_t    ca_ind; 

    // Flag for format0/format1A differentiation – 1 bit, where value 0 indicates format 0 and value 1 indicates format 1A
    // Format 1A is used for random access procedure initiated by a PDCCH order only if format 1A CRC is scrambled
    // with C-RNTI and all the remaining fields are set as follows:
    uint16_t    dci_0_1a_flag;              // Flag for format0/format1A differentiation

    // Localized/Distributed VRB assignment flag – 1 bit is set to ‘0’
    // Resource block assignment –  bits, where all bits shall be set to 1
    // Preamble Index – 6 bits
    // PRACH Mask Index – 4 bits, [5]


    // Localized/Distributed VRB assignment flag – 1 bit as defined in 7.1.6.3 of 3GPP TS 36.213 [3]
    uint16_t    loc_or_dist;                // Localized/Distributed VRB assignment flag

    // 1 bit, the MSB indicates the gap value, where value 0 indicates
    uint16_t    n_gap;                      // N_Gap
    uint32_t    riv;                        // resource indication value
    uint32_t    rb_assign_distributed;      // Resource block assignment for Distributed DRB

    // Modulation and coding scheme – 5bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    uint16_t    mcs;                        // MCS

    // HARQ process number – 3 bits (FDD) , 4 bits (TDD)
    uint16_t    harq_process;               // HARQ Process

    // New data indicator – 1 bit
    uint16_t    ndi;                        // New data indicator

    // Redundancy version – 2 bits
    uint16_t    redundancy_version;         // Redundancy version

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    uint16_t    tpc;                        // TPC command for PUCCH

    // -------------------------------------------------------    

    int32_t     srs_request;                // SRS Request
    int32_t     harq_ack_resource_offset;   // HARQ-ACK resource offset   
    int32_t     preamble_index;             // Preamble Index
    int32_t     prach_mask_index;           // PRACH Mask Index

    int32_t     riv_length; 
    int32_t     RB_start[2];
    int32_t     L_crbs[2];

    uint16_t    rnti;
    uint32_t    tx_mode; 
    int32_t     N_codewords;
    LIBLTE_PHY_MODULATION_TYPE_ENUM mod_type;
    LIBLTE_PHY_PRE_CODER_TYPE_ENUM  pre_coder_type;    
    int32_t     tbs;

};


//
// 5.3.3.1.3A Format 1B in 3GPP TS 36.212
//
struct dci_format_1b_fdd_s {
    
    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    uint16_t    ca_ind;                     

    // Localized/Distributed VRB assignment flag – 1 bit as defined in section 7.1.6.3 of 3GPP TS 36.213 [3]
    uint16_t    loc_or_dist; 

    // 5/ 7/ 9/10/11/12
    uint16_t    n_gap;                      // N_Gap
    uint32_t    rb_assign_local;            // Resource block assignment for Localized DRB
    uint32_t    rb_assign_distributed;      // Resource block assignment for Distributed DRB

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of [3]
    uint16_t    mcs;                    

    // HARQ process number – 3 bits (FDD) , 4 bits (TDD)
    uint16_t    harq_process;

    // New data indicator – 1 bit
    uint16_t    ndi;                        // New data indicator

    // Redundancy version – 2 bits
    uint16_t    redundancy_version;         // Redundancy version

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of [3]
    uint16_t    tpc;                        // TPC command for PUCCH

    // TPMI information for precoding – number of bits as specified in Table 5.3.3.1.3A-1
    uint16_t    tpmi;

    // PMI confirmation for precoding – 1 bit as specified in Table 5.3.3.1.3A-2
    uint16_t    pmi;    

    // 20/22/24/25/26/27
    // 22/24/26/27/28/29  ->  22/25/27/28/29/30
    // 24/26/28/29/30/31  ->  25/27/28/30/31/33   
    
};

//
// 5.3.3.1.4 Format 1C in 3GPP TS 36.212
//
struct dci_format_1c_fdd_s {
    
    // 1 bit indicates the gap value, where value 0 indicates Ngap = Ngap,1 and value 1 indicates Ngap = Ngap,2
    int32_t     n_gap;                      // N_Gap
    
    int32_t     rb_assign_local;            // Resource block assignment for Localized DRB
    int32_t     rb_assign_distributed;      // Resource block assignment for Distributed DRB

    // - Modulation and coding scheme – 5bits as defined in section 7.1.7 of [3]
    int32_t     mcs;                    
    
};

//
// 5.3.3.1.4 Format 1D in 3GPP TS 36.212
//
struct dci_format_1d_fdd_s {

    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    int16_t     ca_ind;                     

    // Localized/Distributed VRB assignment flag – 1 bit as defined in section 7.1.6.3 of 3GPP TS 36.213 [3]
    int32_t     loc_or_dist; 


    // 1 bit indicates the gap value, where value 0 indicates Ngap = Ngap,1 and value 1 indicates Ngap = Ngap,2
    int32_t     n_gap;                      // N_Gap
    
    int32_t     rb_assign_local;            // Resource block assignment for Localized DRB
    int32_t     rb_assign_distributed;      // Resource block assignment for Distributed DRB

    // - Modulation and coding scheme – 5bits as defined in section 7.1.7 of 36.213 [3]
    uint16_t    mcs;                    

    // HARQ process number – 3 bits (FDD) , 4 bits (TDD)
    uint16_t    harq_process;

    // New data indicator – 1 bit
    uint16_t    ndi;                        // New data indicator

    // Redundancy version – 2 bits
    uint16_t    redundancy_version;         // Redundancy version

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    uint16_t    tpc;                        // TPC command for PUCCH

    // TPMI information for precoding – number of bits as specified in Table 5.3.3.1.3A-1
    uint16_t    tpmi;

    // Downlink power offset – 1 bit as defined in section 7.1.5 of 3GPP TS 36.213 [3]
    uint16_t    dl_power_offset;
    
};

//
// 5.3.3.1.5 Format 2 in 3GPP TS 36.212
//
struct dci_format_2_fdd_s {

    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    uint16_t    ca_ind;                     

    // - Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.213 [3]
    // If downlink bandwidth is less than or equal to 10 PRBs, there is no resource allocation header and resource
    // allocation type 0 is assumed.
    uint16_t    resource_alloc_type;      

    // Resource block assignment
    // For resource allocation type 0 as defined in section 7.1.6.1 of 3GPP TS 36.213 [3]:
    uint32_t    resource_block_assignment;

    // For resource allocation type 1 as defined in section 7.1.6.2 of 3GPP TS 36.213 [3]:
    uint16_t    subset;
    uint16_t    shift;

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    uint16_t    tpc;

    // HARQ process number - 3 bits (FDD), 4 bits (TDD)
    uint16_t    harq_process;

    // Transport block to codeword swap flag – 1 bit
    uint16_t    tb_codeword_swap_flag;

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb1_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb1_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb1_redundancy_version;       

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb2_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb2_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb2_redundancy_version;  
    

    // -------------------------------------------------------   
    int32_t     N_rb_type1;
    int32_t     N_rb_rbg_subset;
    int32_t     delta_shift;
    int32_t     prb_allocated;
    uint8_t     prb[LIBLTE_PHY_N_RB_DL_MAX];

    LIBLTE_PHY_MODULATION_TYPE_ENUM tb1_mod_type;
    int32_t     tb1_i_tbs;
    int32_t     tb1_tbs;

    LIBLTE_PHY_MODULATION_TYPE_ENUM tb2_mod_type;
    int32_t     tb2_i_tbs;
    int32_t     tb2_tbs;

    uint16_t    rnti;

};

//
//5.3.3.1.5 Format 2 in 3GPP TS 36.212
// 
struct dci_format_2a_fdd_s {
    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    int16_t     ca_ind;                     

    // - Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.213 [3]
    // If downlink bandwidth is less than or equal to 10 PRBs, there is no resource allocation header and resource
    // allocation type 0 is assumed.
    int16_t     resource_alloc_type;      

    // Resource block assignment
    // For resource allocation type 0 as defined in section 7.1.6.1 of 3GPP TS 36.213 [3]:
    uint32_t    resource_block_assignment;

    // For resource allocation type 1 as defined in section 7.1.6.2 of 3GPP TS 36.213 [3]:
    int32_t     subset;
    int32_t     shift;

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    int32_t     tpc;

    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    int32_t     harq_process;    

    // Transport block to codeword swap flag – 1 bit
    int32_t tb_codeword_swap_flag;

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb1_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb1_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb1_redundancy_version;       

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb2_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb2_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb2_redundancy_version;  

    // Precoding information – number of bits as specified in Table 5.3.3.1.5A-1 in 3GPP TS 36.212
    int32_t     precoding_info;

    // -------------------------------------------------------   
    int32_t     N_rb_type1;
    int32_t     N_rb_rbg_subset;
    int32_t     delta_shift;
    int32_t     prb_allocated;
    uint8_t     prb[LIBLTE_PHY_N_RB_DL_MAX];

    LIBLTE_PHY_MODULATION_TYPE_ENUM tb1_mod_type;
    int32_t     tb1_i_tbs;
    int32_t     tb1_tbs;

    LIBLTE_PHY_MODULATION_TYPE_ENUM tb2_mod_type;
    int32_t     tb2_i_tbs;
    int32_t     tb2_tbs;

    uint16_t    rnti;
    

};

//
// 5.3.3.1.5B Format 2B in 3GPP TS 36.212
// 
struct dci_format_2b_fdd_s {
    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    int16_t     ca_ind;                     

    // - Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.213 [3]
    // If downlink bandwidth is less than or equal to 10 PRBs, there is no resource allocation header and resource
    // allocation type 0 is assumed.
    int16_t     resource_alloc_type;      

    // Resource block assignment
    // For resource allocation type 0 as defined in section 7.1.6.1 of 3GPP TS 36.213 [3]:
    uint32_t    resource_block_assignment;

    // For resource allocation type 1 as defined in section 7.1.6.2 of 3GPP TS 36.213 [3]:
    int32_t     subset;
    int32_t     shift;

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    int32_t     tpc;

    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    int32_t     harq_process;    

    // Scrambling identity– 1 bit as defined in section 6.10.3.1 of [2]
    int32_t     scrambling_id;    

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb1_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb1_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb1_redundancy_version;       

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb2_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb2_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb2_redundancy_version;  

};


//
// 5.3.3.1.5C Format 2C in 3GPP TS 36.212
// 
struct dci_format_2c_fdd_s {
    // Carrier indicator – 0 or 3 bits. This field is present according to the definitions in 3GPP TS 36.213 [3]
    int16_t     ca_ind;                     

    // - Resource allocation header (resource allocation type 0 / type 1) – 1 bit as defined in section 7.1.6 of 3GPP TS 36.213 [3]
    // If downlink bandwidth is less than or equal to 10 PRBs, there is no resource allocation header and resource
    // allocation type 0 is assumed.
    int16_t     resource_alloc_type;      

    // Resource block assignment
    // For resource allocation type 0 as defined in section 7.1.6.1 of 3GPP TS 36.213 [3]:
    uint32_t    resource_block_assignment;

    // For resource allocation type 1 as defined in section 7.1.6.2 of 3GPP TS 36.213 [3]:
    int32_t     subset;
    int32_t     shift;

    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    int32_t     tpc;

    // HARQ process number – 3 bits (FDD), 4 bits (TDD)
    int32_t     harq_process;    

    // Antenna port(s), scrambling identity and number of layers – 3 bits as specified in Table 5.3.3.1.5C-1 where nSCID is
    // the scrambling identity for antenna ports 7 and 8 defined in section 6.10.3.1 of [2]
    int32_t    antenna_port;
    
    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb1_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb1_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb1_redundancy_version;       

    // Modulation and coding scheme – 5 bits as defined in section 7.1.7 of 3GPP TS 36.213 [3]
    int32_t     tb2_mcs;                        

    // New data indicator – 1 bit
    int32_t     tb2_ndi;                        // New data indicator

    // Redundancy version – 2 bits
    int32_t     tb2_redundancy_version;  

};

//
// 5.3.3.1.6 Format 3 in 3GPP TS 36.212
// 
struct dci_format_3_fdd_s {
    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    int32_t     tpc[MAX_TPC_COMMAND];
};


//
// 5.3.3.1.6 Format 3 in 3GPP TS 36.212
// 
struct dci_format_3A_fdd_s {
    // TPC command for PUCCH – 2 bits as defined in section 5.1.2.1 of 3GPP TS 36.213 [3]
    int32_t     tpc[MAX_TPC_COMMAND];
};

struct dci_format_s {
    int32_t     format;
    union {
        struct dci_format_0_1a_hdr_s dci_0_1a_hdr;
        struct dci_format_0_fdd_s  dci_0_fdd;
        struct dci_format_1_fdd_s  dci_1_fdd;
        struct dci_format_1a_fdd_s dci_1a_fdd;
        struct dci_format_1b_fdd_s dci_1b_fdd;
        struct dci_format_1c_fdd_s dci_1c_fdd;
        struct dci_format_1d_fdd_s dci_1d_fdd;
        struct dci_format_2_fdd_s dci_2_fdd;
        struct dci_format_2a_fdd_s dci_2a_fdd;        
        struct dci_format_2b_fdd_s dci_2b_fdd;        
        struct dci_format_2c_fdd_s dci_2c_fdd;        
        struct dci_format_3_fdd_s dci_3_fdd;   
        struct dci_format_3_fdd_s dci_3a_fdd;   
    } d;
};

#pragma pack(pop)

extern int32_t dci_format_0_test;



/*********************************************************************
    Name: pre_decoder_and_matched_filter_dl

    Description: Matched filters and unmaps a block of vectors from
                 resources on each downlink antenna port

    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.3.4

    NOTES: Currently only supports single antenna or TX diversity
*********************************************************************/
void pre_decoder_and_matched_filter_dl(float                          *y_re,
                                       float                          *y_im,
                                       float                          *h_re,
                                       float                          *h_im,
                                       uint32                          h_len,
                                       uint32                          M_ap_symb,
                                       uint8                           N_ant,
                                       LIBLTE_PHY_PRE_CODER_TYPE_ENUM  type,
                                       float                          *x_re,
                                       float                          *x_im,
                                       uint32                         *M_layer_symb);

/*********************************************************************
    Name: layer_demapper_dl

    Description: De-maps one or several downlink layers into complex-
                 valued modulation symbols

    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.3.3

    NOTES: Currently only supports single antenna or TX diversity
*********************************************************************/
void layer_demapper_dl(float                          *x_re,
                       float                          *x_im,
                       uint32                          M_layer_symb,
                       uint8                           N_ant,
                       uint32                          N_codewords,
                       LIBLTE_PHY_PRE_CODER_TYPE_ENUM  type,
                       float                          *d_re,
                       float                          *d_im,
                       uint32                         *M_symb);


/*********************************************************************
    Name: modulation_demapper

    Description: Maps complex-valued modulation symbols to binary
                 digits

    Document Reference: 3GPP TS 36.211 v10.1.0 section 7.1

    NOTES: Currently only supports BPSK and QPSK
*********************************************************************/
void modulation_demapper(float                           *d_re,
                         float                           *d_im,
                         uint32                           M_symb,
                         LIBLTE_PHY_MODULATION_TYPE_ENUM  type,
                         int8                            *bits,
                         uint32                          *N_bits);

/*********************************************************************
    Name: generate_prs_c

    Description: Generates the psuedo random sequence c

    Document Reference: 3GPP TS 36.211 v10.1.0 section 7.2
*********************************************************************/
void generate_prs_c(uint32  c_init,
                    uint32  len,
                    uint32 *c);

/*********************************************************************
    Name: code_block_deconcatenation

    Description: Performs code block deconcatenation for turbo coded
                 channels

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.5
*********************************************************************/
void code_block_deconcatenation
(
    float  *f_bits,
    uint32  N_f_bits,
    uint32  tbs,
    float  *e_bits,
    uint32 *N_e_bits,
    uint32  N_e_bits_max,
    uint32 *N_codeblocks
);

/*********************************************************************
    Name: turbo_encode

    Description: Turbo encodes a bit array using the LTE Parallel
                 Concatenated Convolutional Code

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.3.2

    Notes: Currently not handling filler bits
*********************************************************************/
void turbo_encode(LIBLTE_PHY_STRUCT *phy_struct,
                  uint8             *c_bits,
                  uint32             N_c_bits,
                  uint32             N_fill_bits,
                  uint8             *d_bits,
                  uint32            *N_d_bits);


/*********************************************************************
    Name: turbo_decode

    Description: Turbo decodes data according to the LTE Parallel
                 Concatenated Convolutional Code.  The design of this
                 decoder is based on the conversion of the constituent
                 coder from:
                                   -------->+---------------->+---- out
                                   |        ^                 ^
                           in_act  |   |-|  |   |-|      |-|  |
                 in --->+------------->|D|----->|D|----->|D|---
                        ^              |-|      |-|  |   |-|  |
                        |                            v        |
                        -----------------------------+<--------
                 to:
                           ------->+---------------->+------------- out
                           |       ^                 ^
                           |  |-|  |   |-|      |-|  |
                 in_act ------|D|----->|D|----->|D|---
                           |  |-|      |-|  |   |-|  |
                           |                v        v
                           ---------------->+------->+------------- in
                 in_act can be determined using viterbi decoding and
                 a second copy of in can be calculated using in_act

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.3.2

    Notes: Currently not handling filler bits
*********************************************************************/
void turbo_decode(LIBLTE_PHY_STRUCT *phy_struct,
                  float             *d_bits,
                  uint32             N_d_bits,
                  uint32             N_fill_bits,
                  uint8             *c_bits,
                  uint32            *N_c_bits);


/*********************************************************************
    Name: calc_crc

    Description: Calculates one of the LTE CRCs

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.1
*********************************************************************/
void calc_crc(uint8  *a_bits,
              uint32  N_a_bits,
              uint32  crc,
              uint8  *p_bits,
              uint32  N_p_bits);

/*********************************************************************
    Name: pcfich_channel_demap

    Description: Channel demaps the PCFICH

    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.7
*********************************************************************/
void pcfich_channel_demap(LIBLTE_PHY_STRUCT          *phy_struct,
                          LIBLTE_PHY_SUBFRAME_STRUCT *subframe,
                          uint32                      N_id_cell,
                          uint8                       N_ant,
                          LIBLTE_PHY_PCFICH_STRUCT   *pcfich,
                          uint32                     *N_bits);

/*********************************************************************
    Name: phich_channel_demap

    Description: Channel demaps the PHICH

    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.9
*********************************************************************/
void phich_channel_demap(LIBLTE_PHY_STRUCT              *phy_struct,
                         LIBLTE_PHY_PCFICH_STRUCT       *pcfich,
                         LIBLTE_PHY_SUBFRAME_STRUCT     *subframe,
                         uint32                          N_id_cell,
                         uint8                           N_ant,
                         float                           phich_res,
                         LIBLTE_RRC_PHICH_DURATION_ENUM  phich_dur,
                         LIBLTE_PHY_PHICH_STRUCT        *phich);

/*********************************************************************
    Name: viterbi_decode

    Description: Viterbi decodes a convolutionally coded input bit
                 array using the provided parameters

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.1.3.1
*********************************************************************/
void viterbi_decode(LIBLTE_PHY_STRUCT *phy_struct,
                    float             *d_bits,
                    uint32             N_d_bits,
                    uint32             constraint_len,
                    uint32             rate,
                    uint32            *g,
                    uint8             *c_bits,
                    uint32            *N_c_bits);

/*********************************************************************
    Name: cfi_channel_decode

    Description: Channel decodes the Control Format Indicator channel

    Document Reference: 3GPP TS 36.212 v10.1.0 section 5.3.4
*********************************************************************/
LIBLTE_ERROR_ENUM cfi_channel_decode
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *in_bits,
    uint32             N_in_bits,
    uint32            *cfi
);

/*********************************************************************
    Name: generate_sss

    Description: Generates LTE secondary synchronization signals

    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.11.2.1
*********************************************************************/
void generate_sss
(
    LIBLTE_PHY_STRUCT *phy_struct,
     uint32             N_id_1,
     uint32             N_id_2,
     float             *sss_re_0,
     float             *sss_im_0,
     float             *sss_re_5,
     float             *sss_im_5
);


/*********************************************************************
    Name: samples_to_symbols_dl

    Description: Converts I/Q samples to subcarrier symbols for the
                 downlink

    Document Reference: 3GPP TS 36.211 v10.1.0 section 6.12
*********************************************************************/
void samples_to_symbols_dl(LIBLTE_PHY_STRUCT *phy_struct,
                           float             *samps_re,
                           float             *samps_im,
                           uint32             slot_start_idx,
                           uint32             symbol_offset,
                           uint8              scale,
                           float             *symb_re,
                           float             *symb_im);

/*********************************************************************
    Name: get_num_bits_in_prb

    Description: Determines the number of bits available in a
                 particular PRB with a particular modulation type

    Document Reference: N/A
*********************************************************************/
uint32 get_num_bits_in_prb(uint32                          N_subframe,
                           uint32                          N_ctrl_symbs,
                           uint32                          prb,
                           uint32                          N_rb_dl,
                           uint8                           N_ant,
                           LIBLTE_PHY_MODULATION_TYPE_ENUM mod_type);


LIBLTE_ERROR_ENUM liblte_phy_pss_init(LIBLTE_PHY_STRUCT *phy_struct);
LIBLTE_ERROR_ENUM liblte_phy_sss_init(LIBLTE_PHY_STRUCT *phy_struct);
LIBLTE_ERROR_ENUM liblte_phy_dl_searcher_init(LIBLTE_PHY_STRUCT *phy_struct);

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
);


int32_t dci_format_0_1a_3_3a_fdd_len(int32_t N_rb_dl);
int32_t dci_format_1_fdd_len(int32_t N_rb_dl);
int32_t dci_format_1c_fdd_len(int32_t N_rb_dl);
int32_t dci_format_2_fdd_len(int32_t N_rb_dl, int32_t N_ant);
int32_t dci_format_2a_fdd_len(int32_t N_rb_dl, int32_t N_ant);


int32_t ra_type0_len(int32_t N_rb_dl);
int32_t RBG_size(int32_t N_rb_dl);
int32_t ra_type0_shift_bit(int32_t N_rb_dl, int32_t subset);
int32_t ra_n_rb_rbg_subset(int32_t N_rb_dl, uint32_t subset);


int do_correlation
(
    float *x_re,
    float *x_im,
    int32_t num_of_sample_x,
    float *y_re,
    float *y_im,
    int32_t num_of_sample_y,
    float *result
);

int do_convolution
(
    float *x_re,
    float *x_im,
    int32_t num_of_sample_x,
    float *y_re,
    float *y_im,
    int32_t num_of_sample_y,
    float *result
);

LIBLTE_ERROR_ENUM liblte_phy_dl_find_coarse_timing_and_freq_offset_n
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    float               *i_samps,
    float               *q_samps,
    uint32               N_slots,
    LIBLTE_PHY_COARSE_TIMING_STRUCT *timing_struct
);

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
);

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
);


LIBLTE_ERROR_ENUM dci_channel_decode_n
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *in_bits,
    uint32             N_in_bits,
    uint8              ue_ant,
    uint8             *out_bits,
    uint32             N_out_bits,
    uint16            *rnti_found
);

LIBLTE_ERROR_ENUM liblte_phy_pdcch_channel_decode_n2
(
    LIBLTE_PHY_STRUCT              *phy_struct,
    LIBLTE_PHY_SUBFRAME_STRUCT     *subframe,
    uint32                          N_id_cell,
    uint8                           N_ant,
    LIBLTE_PHY_PCFICH_STRUCT       *pcfich,
    LIBLTE_PHY_PHICH_STRUCT        *phich,
    LIBLTE_PHY_PDCCH_STRUCT        *pdcch
);

void rate_unmatch_conv_n
(
    LIBLTE_PHY_STRUCT *phy_struct,
    float             *e_bits,
    uint32             N_e_bits,
    uint32             N_c_bits,
    float             *d_bits,
    uint32            *N_d_bits
);


int32_t dci_0_1a_unpack_fdd
(
    uint8_t                        *in_bits,
    uint32_t                        N_in_bits,
    int32_t                         ca_presence,
    uint16_t                        rnti,
    uint32_t                        N_rb_dl,
    uint32_t                        N_rb_ul,
    uint8_t                         N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,   
    struct dci_format_s             *dci
);

int32_t dci_0_unpack_fdd
(
    uint8_t                         *in_bits,
    uint32_t                         N_in_bits,
    int32_t                          ca_presence,
    uint16_t                         rnti,
    uint32_t                         N_rb_dl,
    uint8_t                          N_ant,
    LIBLTE_PHY_ALLOCATION_STRUCT    *alloc,   
    struct dci_format_0_fdd_s       *dci_0
);

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
);

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
);

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
);

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
);


int32_t check_common_search_space
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             agg_level_min,
    int32_t             agg_level_max,
    int32_t             dci_format,
    int32_t             dci_len
);

int32_t check_ue_specific_search_space
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             subframe_no,
    int32_t             agg_level_min,
    int32_t             agg_level_max,
    int32_t             dci_format,
    int32_t             dci_len
);

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
);


int32_t bytes_to_bits(uint8_t *bytes, uint8_t *bits, int32_t bit_len);
int32_t uint64_to_bits(uint64_t data, uint8_t *bits, int32_t bit_len);
int32_t uint32_to_bits(uint32_t data, uint8_t *bits, int32_t bit_len);
void print_float_scaled2(float *data_buff, int data_len, float scale);

int32_t liblte_phy_get_pdcch_descrambled_bits
(
    LIBLTE_PHY_STRUCT              *phy_struct,
    uint8                           N_ant,
    uint32_t                        M_ap_symb,
    uint32_t                        *pdcch_c,
    uint32_t                        *N_bits
);

int32_t liblte_phy_get_pdsch_est
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             agg_lvl,
    int32_t             start_cce_n
);

int32_t get_psch_first_sc(LIBLTE_PHY_STRUCT *phy_struct);
int32_t get_psch_last_sc(LIBLTE_PHY_STRUCT *phy_struct);



int32_t decode_cce
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             L,
    int32_t             cce_index,
    int32_t             dci_len,
    uint16_t            *rnti_found
);

int32_t check_cce_index(int32_t cce_index, int32_t rnti, int32_t L, int32_t Ncce, int32_t subframe_no);

int32_t check_common_search_space
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             agg_level_min,
    int32_t             agg_level_max,
    int32_t             dci_format,
    int32_t             dci_len
);

int32_t check_ue_specific_search_space
(
    LIBLTE_PHY_STRUCT   *phy_struct,
    int32_t             N_ant,
    int32_t             subframe_no,
    int32_t             agg_level_min,
    int32_t             agg_level_max,
    int32_t             dci_format,
    int32_t             dci_len
);


int32_t check_decode_pdsch_channel
(
    LIBLTE_PHY_STRUCT             *phy_struct,
    LIBLTE_PHY_SUBFRAME_STRUCT    *subframe,
    LIBLTE_PHY_PDCCH_STRUCT       *pdcch,
    uint32_t                      N_pdcch_symbs,
    uint32_t                      N_id_cell,
    uint8_t                       N_ant
);

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
);

void print_bcch_dlsch_msg_sib1(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT *sib1);
void print_bcch_dlsch_msg_sib2(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT *sib2);
void print_bcch_dlsch_msg_sib3(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3_STRUCT *sib3);
void print_bcch_dlsch_msg_sib4(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4_STRUCT *sib4);
void print_bcch_dlsch_msg_sib5(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5_STRUCT *sib5);
void print_bcch_dlsch_msg_sib6(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6_STRUCT *sib6);
void print_bcch_dlsch_msg_sib7(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7_STRUCT *sib7);
void print_bcch_dlsch_msg_sib8(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8_STRUCT *sib8);
void print_bcch_dlsch_msg_sib13(LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13_STRUCT *sib13);

void print_pcch_page(LIBLTE_RRC_PAGING_STRUCT *page);



typedef enum {
    LTE_PSS = 1,
    LTE_SSS,
    LTE_CRS,
    LTE_BCCH,
    LTE_PCFICH,
    LTE_PHICH,
    LTE_PDCCH,
    LTE_PDSCH,
    
    LTE_CH_MAX
} enum_lte_channel;

uint64_t make_log_code(int log_code);
#define MAKE_LOG_CODE(log_code)         ((uint64)make_log_code(log_code))

typedef enum {

    LOG_MSG_CODE_ENCODE = 1,
    LOG_MSG_CODE_INIT,
    LOG_MSG_CODE_PSS,
    LOG_MSG_CODE_PSS_INIT,
    LOG_MSG_CODE_PSS_DATA,
    
    LOG_MSG_CODE_SSS,
    LOG_MSG_CODE_SSS_INIT,
    LOG_MSG_CODE_SSS_DATA,
    LOG_MSG_CODE_SSS_FIND,
    
    LOG_MSG_CODE_CRS,
    LOG_MSG_CODE_PRS_C,

    LOG_MSG_CODE_BCCH,
    LOG_MSG_CODE_PDCCH,
    LOG_MSG_CODE_PCFICH,
    LOG_MSG_CODE_PHICH,
    LOG_MSG_CODE_PDSCH,

    LOG_MSG_CODE_CONV,
    LOG_MSG_CODE_RATE_MATCH,
    LOG_MSG_CODE_MODULATION_MAPPER,
    LOG_MSG_CODE_LAYER_MAPPER_DL,
    LOG_MSG_CODE_PRE_CODER_DL,

    LOG_MSG_CODE_SIB_INFO,
    LOG_MSG_CODE_BCCH_DLSCH, 
    LOG_MSG_CODE_TBS_MCS, 

    LOG_MSG_CODE_DCI,
    LOG_MSG_CODE_CHECK_PRB,
    LOG_MSG_CODE_CODE_BLOCK_SEGMENT,
    LOG_MSG_CODE_TURBO_ENCODE,
    LOG_MSG_CODE_SYMBOL_2_SAMPLE,
    LOG_MSG_CODE_CODE_BLOCK_CONCATENATION,

    LOG_MSG_CODE_PAYLOAD,
    LOG_MSG_CODE_CREATE_DL_FRAME,

    LOG_MSG_CODE_DECODE_INPUT,
    LOG_MSG_CODE_LOAD_BUFF,
    LOG_MSG_CODE_SEARCH,
    LOG_MSG_CODE_SEARCH_COARSE_TIMING,
    LOG_MSG_CODE_PSS_AND_FINE_TIMING,

    LOG_MSG_CODE_MAX,
} enum_log_msg_code;


#define LOG_MSG_ENABLE(log_msg_code)        Msg_Check_Enabled(log_msg_code)
#define LOG_MSG_SET_PARENT(log_msg_code)    Msg_Set_Parent(log_msg_code)

struct lte_enum_info {
    int32_t Value;
    char *Name;
};


void print_iqdata(float *symb_re, float *symb_im, int symbol_len);
void print_int8_data(int8_t *data, int count);
void print_complex_number(float *re, float *im, int count, int offset);
void print_complex_number_normalized(float *re, float *im, int count, int offset);
void print_iqdata_f(float *symb_re, float *symb_im, int symbol_len);
void print_float_scaled(float *data_buff, int data_len, float scale);
void liblte_print_complex_number_polar(float *mag, float *phase, int data_len);
void print_pss(int N_id_2, float *pss_re, float *pss_im);
void print_int8_data(int8_t *data, int count);
void liblte_print_float(float *data_buff, int data_len);
void print_uint32_data(uint32_t *data, int count);
void print_bits(uint8_t *bits, int bitcount);
void print_bits_to_bytes(uint8_t *bits, int bitcount);

char *Get_SIB_TYPE_Info_Str(int32_t Value);
char *Get_Rrc_Sib_Type_Str(int32_t Value);
char *Get_PHICH_DURATION_str(int32_t Value);

float liblte_get_max_value(float *data_buff, int data_len);
void print_0x4_corr(LIBLTE_PHY_STRUCT *phy_struct);

float get_channel_power(float *i_data, float *q_data, int32_t count);
void print_quantize_sample(int16_t *re, int16_t *im, int count, int offset);

int32_t Msg_Check_Enabled(int log_code);
int32_t Msg_Set_Parent(int log_code);

int log_msg_set(int log_code);
int log_msg_clr(int log_code);



#endif /* __LIBLTE_PHY_ANALYSIS_H__ */
