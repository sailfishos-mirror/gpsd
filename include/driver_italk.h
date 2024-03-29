/*
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#ifndef _GPSD_ITALK_H_
#define _GPSD_ITALK_H_

// 0 and 1 are responses to the <?> ping for iTalk and NMEA respectively
#define PROTO_ITALK 0
#define PROTO_NMEA 1

/*
 * Assistance from Timo Ylhainen of Fastrax is acknowledged and appreciated.
 *
 * iTalk is a messaging system which communicates between tasks, which may
 * be running on different devices (nodes). For our purposes (receiver
 * configuration), we will probably be sending to the SYSTEM task.
 */

#define TASK_MASK       0x1f    // 5 low bits of src/dst fields
#define NODE_MASK       0xe0    // 3 high bits of src/dst fields
#define NODE_UNDEF      0x00    // Used in message routing
#define NODE_ITRAX      0x20    // The receiver
#define NODE_HOST       0x40    // Software on your computer
#define NODE_GPSWB      0x60    // GPSWorkbench seems to be HOST|ITRAX

/* FIX-ME: These defines will likely be replaced by an enum
 * once I map every message to the task that sent it.
 */
// System controller on the receiver
#define TASK_SYSTEM     0
// Acquisition & Tracking messages (PD)
#define TASK_TRACK1     2
#define TASK_TRACK2     3
// Data decoding messages (PD)
#define TASK_DATA       4
// Navigation messages are sent by these tasks (PD)
#define TASK_NAV1       7
#define TASK_NAV2       8
#define TASK_NAV3       9
// Host controller software (PD)
#define TASK_HOST       31

#define MAX_NR_VISIBLE_PRNS 16

// iTalk Message IDs - isuite.fastrax.fi/sdk/331/Protocols/PRO_MsgId.html
#define ITALK_ACQ_DATA          1
#define ITALK_PRN_STATUS        2
#define ITALK_TRACK             3
#define ITALK_PSEUDO            4
#define ITALK_AGC               6
#define ITALK_NAV_FIX           7
#define ITALK_RAW_ALMANAC       9
#define ITALK_RAW_EPHEMERIS     10
#define ITALK_SV_HEALTH         11
#define ITALK_UTC_IONO_MODEL    12
#define ITALK_PRN_PRED          13
#define ITALK_FREQ_PRED         14
#define ITALK_SUBFRAME          15
#define ITALK_BIT_STREAM        18
#define ITALK_DBGTRACE          19
#define ITALK_START             64
#define ITALK_STOP              65
#define ITALK_SLEEP             66
#define ITALK_STATUS            67
#define ITALK_ITALK_CONF        68
#define ITALK_SYSINFO           69
#define ITALK_ITALK_TASK_ROUTE  70
#define ITALK_PARAM_CTRL        71
#define ITALK_PARAMS_CHANGED    72
#define ITALK_START_COMPLETED   73
#define ITALK_STOP_COMPLETED    74
#define ITALK_LOG_CMD           75
#define ITALK_SYSTEM_START      76
#define ITALK_STOP_SEARCH       79
#define ITALK_SEARCH            80
#define ITALK_PRED_SEARCH       81
#define ITALK_SEARCH_DONE       82
#define ITALK_TRACK_DROP        88
#define ITALK_TRACK_STATUS      90
#define ITALK_HANDOVER_DATA     92
#define ITALK_CORE_SYNC         93
#define ITALK_WAAS_RAWDATA      96
#define ITALK_ASSISTANCE        98
#define ITALK_PULL_FIX          99
#define ITALK_MEMCTRL           112
#define ITALK_STOP_TASK         255

// NAV_FIX
#define FIX_CONV_VEL_VALID              0x0002
#define FIX_CONV_ACC_VALID              0x0004
#define FIX_CONV_DOP_VALID              0x0010
#define FIX_CONV_ERR_VALID              0x0020
#define FIX_CONV_UTC_VALID              0x0040
#define FIX_CONV_UND_VALID              0x0080
#define FIX_CONV_MAG_VALID              0x0100
#define FIX_CONV_GRID_VALID             0x0200
#define FIX_CONV_VEL_ESTIMATED          0x0400

#define FIX_FLAG_POS_REJECT_FOM         0x0003
#define FIX_FLAG_POS_REJECT_DOP         0x0004
#define FIX_FLAG_POS_PINNING            0x0020

#define FIX_FLAG_VEL_REJECT_RES         0x0003
#define FIX_FLAG_ACCELERATION           0x4000
#define FIX_FLAG_VEL_RELIABLE           0x0020
#define FIX_FLAG_VEL_RELIABLE_3D        0x0040

#define FIX_FLAG_MASK_INVALID           0x0007
#define FIX_FLAG_REJECT_NUM_SV          0x0001
#define FIX_FLAG_REJECT_POSTRAIM        0x0002
#define FIX_FLAG_REJECT_OTHER           0x0007
#define FIX_FLAG_RELIABLE               0x0008
#define FIX_FLAG_PF_RAIM                0x0010
#define FIX_FLAG_3DFIX                  0x0100
#define FIX_FLAG_DGPS_CORRECTION        0x0200
#define FIX_FLAG_TROPO                  0x0400
#define FIX_FLAG_IONO                   0x0800
#define FIX_FLAG_INS                    0x2000

#define FIXINFO_FLAG_VALID              0x0002
#define FIXINFO_FLAG_NEW_FIX            0x0004
#define FIXINFO_FLAG_SKY_FIX            0x0008
#define FIXINFO_FLAG_AID_GPSTIME        0x0010
#define FIXINFO_FLAG_AID_TIMESTAMP      0x0020
#define FIXINFO_FLAG_AID_EPHEMERIS      0x0040
#define FIXINFO_FLAG_AID_ALTITUDE       0x0080
#define FIXINFO_FLAG_KALMAN             0x1000
#define FIXINFO_FLAG_INTERNAL           0x2000
#define FIXINFO_FLAG_FIRSTFIX           0x4000

// PRN_STATUS
#define PRN_FLAG_FOUND                  0x0001
#define PRN_FLAG_TRACKING               0x0002
#define PRN_FLAG_USE_IN_NAV             0x0004

// UTC_IONO_MODEL
#define UTC_IONO_MODEL_UTCVALID         0x0001
#define UTC_IONO_MODEL_IONOVALID        0x0002

// SUBFRAME
#define SUBFRAME_WORD_FLAG_MASK         0x03ff
#define SUBFRAME_GPS_PREAMBLE_INVERTED  0x0400

// PSEUDO
#define PSEUDO_OBS_DOPPLER_OK                   0x0001
#define PSEUDO_OBS_PSEUDORANGE_OK               0x0002
#define PSEUDO_OBS_TOW_OK                       0x0004
#define PSEUDO_OBS_PRN_OK                       0x0008
#define PSEUDO_OBS_ELEV_OK                      0x0010
#define PSEUDO_OBS_SNR_OK                       0x0020
#define PSEUDO_OBS_SV_HEALTHY                   0x0040
#define PSEUDO_OBS_NO_CROSS_CORR                0x0080
#define PSEUDO_OBS_DATA_EXISTS                  0x0100
#define PSEUDO_OBS_DATA_GOOD                    0x0200
#define PSEUDO_OBS_BIT_LOCK                     0x0400
#define PSEUDO_OBS_FIRST_MEAS                   0x0800
#define PSEUDO_OBS_RAIM_P_OK                    0x1000
#define PSEUDO_OBS_RAIM_V_OK                    0x2000
#define PSEUDO_OBS_RAIM_T_OK                    0x4000
#define PSEUDO_OBS_PLL                          0x8000
#define PSEUDO_OBS_MEAS_OK              ( PSEUDO_OBS_ELEV_OK | PSEUDO_OBS_SNR_OK | PSEUDO_OBS_PRN_OK | PSEUDO_OBS_NO_CROSS_CORR | PSEUDO_OBS_SV_HEALTHY | PSEUDO_OBS_DATA_EXISTS | PSEUDO_OBS_DATA_GOOD | PSEUDO_OBS_PSEUDORANGE_OK )
#define PSEUDO_OBS_DOPPLER_MEAS_OK      ( PSEUDO_OBS_ELEV_OK | PSEUDO_OBS_SNR_OK | PSEUDO_OBS_PRN_OK | PSEUDO_OBS_NO_CROSS_CORR | PSEUDO_OBS_SV_HEALTHY | PSEUDO_OBS_DATA_EXISTS | PSEUDO_OBS_DATA_GOOD | PSEUDO_OBS_DOPPLER_OK )

#define PSEUDO_TOW_WEEK_OK                      0x0001
#define PSEUDO_TOW_OK                           0x0002
#define PSEUDO_RESYNCH                          0x0004
#define PSEUDO_FIRST_MEAS                       0x0008
#define PSEUDO_UNSCHEDULED                      0x0010

#define PSEUDO_OBS_CORRECTED_AMBIGUOUS          0x0001
#define PSEUDO_OBS_CORRECTED_BY_SMOOTHING       0x0002
#define PSEUDO_OBS_CORRECTED_BY_IONO            0x0008
#define PSEUDO_OBS_CORRECTED_BY_TROPO           0x0010
#define PSEUDO_OBS_CORRECTED_BY_FAST_CORR       0x0020
#define PSEUDO_OBS_CORRECTED_BY_DGPS            0x0040
#define PSEUDO_OBS_CORRECTED_BY_SLOW_CORR       0x0080
#define PSEUDO_OBS_CORRECTED_BY_WAAS_IONO       0x0100
#define PSEUDO_OBS_CORR_POSSIBLE_XCORR          0x4000
#define PSEUDO_OBS_CORR_FRAME_LOCK              0x8000
#define PSEUDO_OBS_CORRECTED_BY_WAAS   (PSEUDO_OBS_CORRECTED_BY_WAAS_IONO | \
                                        PSEUDO_OBS_CORRECTED_BY_FAST_CORR)

// MEMCTRL
#define MEM_WRITE               0x0002
#define MEM_READD               0x0003
#define MEM_BOOT                0x0004
#define MEM_ERASE               0x0006
#define MEM_XTAL_CALIBRATE      0x000a
// BOOT flags based on isuite.fastrax.fi/sdk/331/Protocols/PRO_NMEA.html
#define MEM_BOOT_NORMAL         0x0000
#define MEM_BOOT_INT_FWLOADER   0x0001
#define MEM_BOOT_DL_FWLOADER    0x0002
#define MEM_BOOT_RELOC_ALTFW    0x0003

// Config Parameters - isuite.fastrax.fi/sdk/331/System/SYS_Parameters.html
// System parameters
#define SYS_SET_ID                      0x0001
#define SYS_FACTORY_SET_ID              0x0002
#define SYS_AUTOSTART                   0x0380
#define START_MODE_AUTO                 0x0301
#define SYS_LKG_SAVE_TIME_LIMIT         0x0008
#define SYS_LKG_SAVE_DIST_LIMIT         0x0009
#define SYS_LKG_SAVE_STOP_TIME_LIMIT    0x000a
#define SYS_WATCHDOG                    0x0028
#define SYS_WATCHDOG_TIMEOUT            0x0029
#define SYS_BOOT_ERASE_PARAMS           0x0080
#define SYS_ENABLE_UI_LEDS              0x0081

// Protocols parameters
#define SYS_ITALK_PORT                  0x0010
#define SYS_ITALK_SPEED                 0x0011
#define SYS_ITALK_MASK                  0x0012
#define SYS_NMEA_PORT                   0x0020
#define SYS_NMEA_SPEED                  0x0021
#define SYS_NMEA_MASK                   0x0022
#define TRACK_ALT_MSG_ROUTING           0x047f
#define OBS_ALT_MSG_ROUTING             0x047e

// Fix Conversion parameters
#define NAV_DATUM_ID                    0x0b08
#define NAV_GRID_ID                     0x0b09
#define NAV_GRID_NUMBER                 0x0b0a
#define NAV_HEAD_VEL_THR                0x0b0b
#define NAV_HEAD_VEL_FILTER             0x0b0c
#define NAV_HEAD_VEL_THRMAX             0x0b0d
#define NAV_HEAD_VEL_THR_PLL            0x0b0e
#define NAV_HEAD_VEL_THRMAX_PLL         0x0b0f
#define NAV_HOLD_HEADING_IF_NO_FIX      0x0bd0

// General navigation parameters
#define NAV_MODE                        0x0b01
#define NAV_FIX_INTERVAL                0x0b02
#define NAV_OUTPUT_INTERVAL             0x0b03
#define NAV_FOM_LIMIT                   0x0b10
#define NAV_VEL_FOM_LIMIT               0x0b15
#define NAV_HDOP_LIMIT                  0x0b11
#define NAV_VDOP_LIMIT                  0x0b12
#define NAV_ALT_LIMIT                   0x0b13
#define NAV_VEL_LIMIT                   0x0b14
#define NAV_EXT_AIDING_ALT              0x0b20
#define NAV_CS_INIT_VAR                 0x0b30
#define NAV_CS_PROC_VAR                 0x0b31
#define NAV_CS_MEAS_VAR                 0x0b32
#define NAV_FILTER_VEL_LOW              0x0b33
#define NAV_FILTER_VEL_HIGH             0x0b34
#define NAV_MAX_LKGAGE                  0x0b40
#define NAV_MAX_2D_FIX_SEC              0x0b41
#define NAV_CARRIERSMOOTHING_ENA        0x0b81
#define NAV_OLD_DATA_ENA                0x0b82
#define NAV_SNR_WEIGHTING_ENA           0x0b83
#define NAV_NORMAL_ENV_ENA              0x0b84
#define NAV_IONO_ENA                    0x0b85
#define NAV_TROPO_ENA                   0x0b87
#define NAV_DGPS_ENA                    0x0b88
#define NAV_VEL_FILTER_ENA              0x0b8b
#define NAV_ALT_LIMIT_ENA               0x0b8c
#define NAV_VEL_LIMIT_ENA               0x0b8d
#define NAV_EXT_AIDING_ALT_ENA          0x0b8e
#define NAV_FOM_ENA                     0x0b8f
#define NAV_HDOP_ENA                    0x0b90
#define NAV_VDOP_ENA                    0x0b91
#define NAV_TENTATIVE_ENA               0x0b96
#define NAV_PULLFIX_ENA                 0x0b97
#define NAV_2D_FIX_ENA                  0x0ba0
#define NAV_RESERVED_001                0x0ba1
#define NAV_OUTPUT_LAST_POS_IF_NO_FIX   0x0bb0
#define NAV_ESTIMATE_VEL_WITHOUT_PLL    0x0bb1
#define NAV_OUTPUT_LAST_VEL_IF_NO_FIX   0x0bb2

// Position pinning parameters
#define NAV_PIN_VEL                     0x0b35
#define NAV_PIN_DRIFT_ERR               0x0b36
#define NAV_PIN_XYZ_ERR                 0x0b37
#define NAV_PIN_TIMEOUT                 0x0b38
#define NAV_PIN_START_DELAY             0x0b39
#define NAV_PINNING_ENA                 0x0b8a

// Interval mode parameters
#define NAV_INTMODE_NBR_FIXES           0x0b22
#define NAV_INTMODE_FIX_INTERVAL        0x0b23
#define NAV_INTMODE_TRY_FIND_SV         0x0b24
#define NAV_INTMODE_TRY_GET_FIX         0x0b25
#define NAV_INTMODE_MAX_STAY_UP         0x0b26
#define NAV_INTMODE_NUM_IGNORED_FIXES   0x0b27
#define NAV_INTERVAL_MODE_ENA           0x0ba2

// Kalman navigation parameters
#define KLM_MODE                        0x0801
#define KLM_MAX_NUM_STATES              0x0802
#define KLM_START_FLAGS                 0x0803
#define KLM_OUTPUT_FLAGS                0x0804
#define KLM_NUM_OBS_LIMIT               0x0805
#define KLM_MEAS_FLAGS                  0x0806
#define KLM_COV_LIMIT                   0x0807
#define KLM_DOPPLER_NOISE               0x0810
#define KLM_RANGE_NOISE                 0x0811
#define KLM_DOPPLER_NOISE_LOW           0x0812
#define KLM_RANGE_NOISE_LOW             0x0813
#define KLM_NOISE_SNR_LOW               0x0814
#define KLM_DOPPLER_NOISE_PLL           0x0815
#define KLM_RANGE_NOISE_PLL             0x0816
#define KLM_CLOCK_OFFSET_NOISE          0x0820
#define KLM_CLOCK_DRIFT_NOISE           0x0821
#define KLM_POS_NOISE                   0x0822
#define KLM_POS_NOISE_VERT              0x0823
#define KLM_VEL_NOISE                   0x0824
#define KLM_VEL_NOISE_VERT              0x0825
#define KLM_ACC_NOISE                   0x0826
#define KLM_ACC_NOISE_VERT              0x0827
#define KLM_ACC_NOISE_PARAM             0x0828
#define KLM_POS_INIT_UNC                0x0830
#define KLM_VEL_INIT_UNC                0x0831
#define KLM_CLOCK_OFFSET_INIT_UNC       0x0832
#define KLM_CLOCK_DRIFT_INIT_UNC        0x0833
#define KLM_RESERVED_001                0x0841
#define KLM_RESERVED_002                0x0842
#define KLM_RESERVED_003                0x0843
#define KLM_RESERVED_004                0x0844
#define KLM_RESERVED_005                0x0845
#define KLM_RESERVED_006                0x0846
#define KLM_RESERVED_007                0x0847
#define KLM_RESERVED_008                0x0848

// Observation parameters
#define TRACK_MEAS_INTERVAL             0x0420
#define TRACK_CHANNELS                  0x041d
#define OBS_ELEV_LIMIT                  0x0101
#define OBS_SNR_LIMIT                   0x0102
#define OBS_SNR_RAIM_LIMIT              0x0103
#define OBS_CROSS_CORR_SNR_DIFF         0x0120
#define OBS_MAX_SNR                     0x0121
#define OBS_PLL_CROSS_CORR_THR          0x0122
#define OBS_FLL_CROSS_CORR_THR          0x0123
#define OBS_FREQ_CROSS_CORR_THR         0x0124
#define OBS_EPOCH_LIMIT                 0x0130
#define OBS_ELEV_LIMIT_ENA              0x0181
#define OBS_SNR_LIMIT_ENA               0x0182
#define OBS_SNR_RAIM_ENA                0x0183
#define SAT_ORBIT_FIT_UPDATE            0x0203
#define SAT_FIRST_WEEK                  0x0204
#define SAT_NUM_LEAP                    0x0205
#define SAT_PRED_MAX_LKGAGE             0x0220
#define SAT_PRED_PHASE_TIMEOUT          0x0221
#define SAT_PRED_LKG_TIMEOUT            0x0222
#define SAT_ORBIT_CHECK                 0x0281

// Unav Tracking parameters
#define TRACK_DLL_ALPHA                 0x0401
#define TRACK_DLL_BETA                  0x0402
#define TRACK_DLL_THR_HIGH              0x0403
#define TRACK_DLL_THR_LOW               0x0404
#define TRACK_DLL_POW_NARROW            0x0405
#define TRACK_DLL_POW_WIDE              0x0406
#define TRACK_FLL_RESPONSE_TIME         0x0407
#define TRACK_POW_CALIBRATION           0x0408
#define TRACK_FLL_THR                   0x0409
#define TRACK_FLL_POW_NARROW            0x040b
#define TRACK_FLL_POW_WIDE              0x040c
#define TRACK_PLL_CTH                   0x040d
#define TRACK_PLL_CDTH                  0x040e
#define TRACK_PLL_CD2TH                 0x040f
#define TRACK_RESERVED_000              0x0410
#define TRACK_RESERVED_001              0x0411
#define TRACK_RESERVED_002              0x0412
#define TRACK_RESERVED_003              0x0413
#define TRACK_RESERVED_004              0x0414
#define TRACK_RESERVED_005              0x0415
#define TRACK_RESERVED_006              0x0416
#define TRACK_RESERVED_007              0x0417
#define TRACK_RESERVED_008              0x0418
#define TRACK_RESERVED_009              0x0419
#define TRACK_RESERVED_010              0x0425
#define TRACK_RESERVED_011              0x0426
#define TRACK_RESERVED_012              0x0427
#define TRACK_RESERVED_013              0x0428
#define TRACK_RESERVED_014              0x0429
#define TRACK_RESERVED_016              0x042a
#define TRACK_RESERVED_017              0x042b
#define TRACK_RESERVED_015              0x0483
#define SUBF_CHECK_FLAGS                0x0432

// Unav Track task parameters
#define TRACK_GROUP_1                   0x041a
#define TRACK_GROUP_2                   0x041b
#define TRACK_GROUP_2_DELAY             0x041c
#define TRACK_CC_DELAY                  0x041e
#define TRACK_CC_THR                    0x041f
#define TRACK_PLL_ENA                   0x0480
#define TRACK_NAVAID_ENA                0x0482
#define TRACK_SHIFT_REG                 0x0421

// Agc config parameters
#define TRACK_AGC_LO                    0x0422
#define TRACK_AGC_HI                    0x0423
#define TRACK_AGC_MAX_HI                0x0424
#define TRACK_AGC_ENA                   0x0481

// PPS parameters
#define PPS_DUTYCYCLE                   0x0440
#define PPS_FREQ                        0x0441
#define PPS_DELAY                       0x0442
#define PPS_SURVEYLEN                   0x0443
#define PPS_MEAS_MS                     0x0444
#define PPS_ENA                         0x0490
#define PPS_SYNC_TRACK                  0x0491
#define PPS_ENA_PRED                    0x0492
#define PPS_INVERT                      0x0493

// Frequency plan parameters
#define FREQ_XTAL                       0x0501
#define FREQ_MCLK_NOM                   0x0502
#define FREQ_MCLK_DENOM                 0x0503
#define FREQ_RF_NOM                     0x0504
#define FREQ_RF_DENOM                   0x0505
#define FREQ_MIXER_OFFSET               0x0506
#define FREQ_TME2                       0x0507
#define FREQ_PARAM_ENA                  0x0581

// Search parameters
#define SEARCH_XTAL_UNC                 0x0701
#define SEARCH_DOPPLER_UNC              0x0702
#define SEARCH_WIN_PRED_EVEN            0x0703
#define SEARCH_WIN_PRED_ODD             0x0704
#define SEARCH_SENS_FULL_R1             0x0705
#define SEARCH_SENS_FULL_R2             0x0706
#define SEARCH_SENS_FULL_R3             0x0707
#define SEARCH_SENS_PRED_EVEN           0x0708
#define SEARCH_SENS_PRED_ODD            0x0709
#define SEARCH_PRED_ROUNDS              0x070a
#define SEARCH_BACK_PRNS                0x070b
#define SEARCH_GPS_MASK                 0x070c
#define SEARCH_WAAS_MASK                0x070d
#define SEARCH_AUTO_PD_ROUNDS           0x070e
#define SEARCH_FLAGS                    0x070f
#define SEARCH_PREC_PRED_TIMEOUT        0x0710
#define SEARCH_PRED_TIMEOUT             0x0711
#define SEARCH_FERRY_COND               0x0712
#define SEARCH_IFFERRY_PRED_COND        0x0713
#define SEARCH_TUNNEL_IN_SNR            0x0714
#define SEARCH_TUNNEL_OUT_SNR           0x0715
#define SEARCH_PRED_ENA                 0x0781
#define SEARCH_BITSYNC_ENA              0x0782
#define SEARCH_AUTO_PRED_ENA            0x0783
#define SEARCH_AUTO_PD_ENA              0x0784
#define SEARCH_SE_PD                    0x0785

// Unav Acquisition parameters
#define ACQ_SENS_9_COH                  0x0901
#define ACQ_SENS_9_NONCOH               0x0902
#define ACQ_SENS_9_THR                  0x0903
#define ACQ_SENS_9_BIN                  0x0904
#define ACQ_SENS_10_COH                 0x0905
#define ACQ_SENS_10_NONCOH              0x0906
#define ACQ_SENS_10_THR                 0x0907
#define ACQ_SENS_10_BIN                 0x0908
#define ACQ_MSG_ENA                     0x0981
#define ACQ_QUICK_SEARCH_ENA            0x0982
#define SE_NONCOH_SHIFT                 0x0940
#define SE_IR_SHIFT                     0x0941
#define SE_THR                          0x0942
#define SE_INT_ENA                      0x09a0

// Logging parameters
#define LOG_MODE                        0x0d01
#define LOG_INTERVAL_MIN                0x0d02
#define LOG_INTERVAL_MAX                0x0d03
#define LOG_MOVE_MIN                    0x0d04
#define LOG_MOVE_MAX                    0x0d05
#define LOG_VELOCITY_MIN                0x0d06
#define LOG_VELOCITY_MAX                0x0d07
#define LOG_MAXITEMS                    0x0d08
#define LOG_STORE_LAT_LONG              0x0d80
#define LOG_STORE_ALT                   0x0d81
#define LOG_STORE_ALT_FULL              0x0d82
#define LOG_STORE_GPSTIME               0x0d83
#define LOG_STORE_GPSTIME_MS            0x0d84
#define LOG_STORE_DIRECTION             0x0d85
#define LOG_STORE_VEL                   0x0d86
#define LOG_STORE_VEL_VERT              0x0d87
#define LOG_STORE_FIXINFO               0x0d88

// SBAS parameters
#define WAAS_TIMEOUT_MODE               0x0b60
#define WAAS_MAX_CHANNELS               0x0b61
#define WAAS_ENA                        0x0bc0
#define WAAS_MSG_0_ENA                  0x0bc1
#define WAAS_STRICT_ENA                 0x0bc2

// Sony Track parameters
#define TRACK_DLL_COEFF_GPS             0x0f01
#define TRACK_DLL_COEFF_DISCR           0x0f02
#define TRACK_DLL_LIM_GPS               0x0f03
#define TRACK_DLL4_COEFF_A              0x0f04
#define TRACK_DLL4_COEFF_B              0x0f05
#define TRACK_DLL4_COEFF_C              0x0f06
#define TRACK_DLL4_COEFF_D              0x0f07
#define TRACK_DLL4_FASTADJ_THRES        0x0f08
#define TRACK_ELGATE_NARROW             0x0f09
#define TRACK_COSTASLF_GPS              0x0f0a
#define TRACK_COSTASLF_WAAS             0x0f0b
#define TRACK_LPF_GPS_ACQ               0x0f0c
#define TRACK_LPF_GPS_LOCK              0x0f0d
#define TRACK_LPF_WAAS_LOCK             0x0f0e
#define TRACK_LPF_NOISE                 0x0f0f
#define TRACK_SIGDETECT_TH              0x0f10
#define TRACK_SIGDETECT_TH_HS           0x0f11
#define TRACK_TIMEOUT_ACQ               0x0f12
#define TRACK_TIMEOUT_ACQHS             0x0f13
#define TRACK_TIMEOUT_REACQ             0x0f14
#define TRACK_HANDOVER_OFFSET           0x0f15
#define TRACK_CROSSCORR_THRES           0x0f16
#define TRACK_DLLCTRL_INTERVAL          0x0f17
#define TRACK_BITEXTRACT                0x0f18
#define TRACK_RESERVED001               0x0f19
#define TRACK_RESERVED002               0x0f1a
#define TRACK_WAAS_PRN_BITSTREAM        0x0f1b
#define TRACK_COSTAS_ERROR_TH           0x0f1d
#define TRACK_CARRFLT_OUT_TH            0x0f1e
#define TRACK_CARRFLT_MIDDLE_TH         0x0f1f
#define TRACK_CARRFLT_OUT_DIV           0x0f20
#define TRACK_CARRFLT_MIDDLE_DIV        0x0f21
#define TRACK_CARRFLT_INBAND_DIV        0x0f22
#define TRACK_LATCHTIME_OFFSET          0x0f23
#define TRACK_DIRECTHANDOVER_OFFSET     0x0f24
#define TRACK_EN_HS                     0x0f80
#define TRACK_CARR_AID                  0x0f81
#define WAAS_EN_DECODE                  0x0f82
#define TRACK_CARRCHKATLOCK             0x0f83
#define TRACK_BL_REACQ                  0x0f84

// Sony Test parameters
#define SONYTEST_DISABLE_PORTS          0x0f85

// Sony Acq parameters
#define SACQ_SEARCH_CH_NUM              0x0f30
#define SACQ_NOISE_COUNT_NUM            0x0f31
#define SACQ_NOISE_VALID_TIME           0x0f32
#define SACQ_NOISE_K                    0x0f33
#define SACQ_PEAK_FD                    0x0f34
#define SACQ_PEAK_NFD                   0x0f35
#define SACQ_RESERVE                    0x0f36
#define SACQ_SEARCH_CH_NUM_VALID        0x0f96

#endif // _GPSD_ITALK_H_
// vim: set expandtab shiftwidth=4
