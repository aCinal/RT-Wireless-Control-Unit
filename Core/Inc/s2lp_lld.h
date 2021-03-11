/**
 * @file s2lp_lld.h
 * @author Adrian Cinal
 * @brief S2-LP LLD header file
 */

#ifndef __S2LP_LLD_H_
#define __S2LP_LLD_H_

/*---------------------------------------------- Includes ----------------------------------------------*/

#include <stdint.h>

/*---------------------------------------------- Typedefs ----------------------------------------------*/

typedef uint8_t TByte;
typedef uint16_t THalfWord;
typedef uint32_t TWord;
typedef uint32_t TSize;

/**
 * @brief LLD functions return value enumeration
 */
typedef enum ES2lpLldRet {
	ES2lpLldRet_Ok = 0, ES2lpLldRet_Error
} ES2lpLldRet;

/*---------------------------------------------- Defines ----------------------------------------------*/

/* S2-LP command list */
#define S2LP_TX_COMMAND                             ( (TByte) 0x60 )
#define S2LP_RX_COMMAND                             ( (TByte) 0x61 )
#define S2LP_READY_COMMAND                          ( (TByte) 0x62 )
#define S2LP_STANDBY_COMMAND                        ( (TByte) 0x63 )
#define S2LP_SLEEP_COMMAND                          ( (TByte) 0x64 )
#define S2LP_LOCKRX_COMMAND                         ( (TByte) 0x65 )
#define S2LP_LOCKTX_COMMAND                         ( (TByte) 0x66 )
#define S2LP_SABORT_COMMAND                         ( (TByte) 0x67 )
#define S2LP_LDC_RELOAD_COMMAND                     ( (TByte) 0x68 )
#define S2LP_SRES_COMMAND                           ( (TByte) 0x70 )
#define S2LP_FLUSHRXFIFO_COMMAND                    ( (TByte) 0x71 )
#define S2LP_FLUSHTXFIFO_COMMAND                    ( (TByte) 0x72 )
#define S2LP_SEQUENCE_UPDATE_COMMAND                ( (TByte) 0x73 )

/* S2-LP register addresses */
#define S2LP_GPIO0_CONF_ADDR                        ( (TByte) 0x00 )
#define S2LP_GPIO1_CONF_ADDR                        ( (TByte) 0x01 )
#define S2LP_GPIO2_CONF_ADDR                        ( (TByte) 0x02 )
#define S2LP_GPIO3_CONF_ADDR                        ( (TByte) 0x03 )
#define S2LP_SYNT3_ADDR                             ( (TByte) 0x05 )
#define S2LP_SYNT2_ADDR                             ( (TByte) 0x06 )
#define S2LP_SYNT1_ADDR                             ( (TByte) 0x07 )
#define S2LP_SYNT0_ADDR                             ( (TByte) 0x08 )
#define S2LP_IF_OFFSET_ANA_ADDR                     ( (TByte) 0x09 )
#define S2LP_IF_OFFSET_DIG_ADDR                     ( (TByte) 0x0A )
#define S2LP_CHSPACE_ADDR                           ( (TByte) 0x0C )
#define S2LP_CHNUM_ADDR                             ( (TByte) 0x0D )
#define S2LP_MOD4_ADDR                              ( (TByte) 0x0E )
#define S2LP_MOD3_ADDR                              ( (TByte) 0x0F )
#define S2LP_MOD2_ADDR                              ( (TByte) 0x10 )
#define S2LP_MOD1_ADDR                              ( (TByte) 0x11 )
#define S2LP_MOD0_ADDR                              ( (TByte) 0x12 )
#define S2LP_CHFLT_ADDR                             ( (TByte) 0x13 )
#define S2LP_AFC2_ADDR                              ( (TByte) 0x14 )
#define S2LP_AFC1_ADDR                              ( (TByte) 0x15 )
#define S2LP_AFC0_ADDR                              ( (TByte) 0x16 )
#define S2LP_RSSI_FLT_ADDR                          ( (TByte) 0x17 )
#define S2LP_RSSI_TH_ADDR                           ( (TByte) 0x18 )
#define S2LP_AGCCTRL4_ADDR                          ( (TByte) 0x1A )
#define S2LP_AGCCTRL3_ADDR                          ( (TByte) 0x1B )
#define S2LP_AGCCTRL2_ADDR                          ( (TByte) 0x1C )
#define S2LP_AGCCTRL1_ADDR                          ( (TByte) 0x1D )
#define S2LP_AGCCTRL0_ADDR                          ( (TByte) 0x1E )
#define S2LP_ANT_SELECT_CONF_ADDR                   ( (TByte) 0x1F )
#define S2LP_CLOCKREC2_ADDR                         ( (TByte) 0x20 )
#define S2LP_CLOCKREC1_ADDR                         ( (TByte) 0x21 )
#define S2LP_PCKTCTRL6_ADDR                         ( (TByte) 0x2B )
#define S2LP_PCKTCTRL5_ADDR                         ( (TByte) 0x2C )
#define S2LP_PCKTCTRL4_ADDR                         ( (TByte) 0x2D )
#define S2LP_PCKTCTRL3_ADDR                         ( (TByte) 0x2E )
#define S2LP_PCKTCTRL2_ADDR                         ( (TByte) 0x2F )
#define S2LP_PCKTCTRL1_ADDR                         ( (TByte) 0x30 )
#define S2LP_PCKTLEN1_ADDR                          ( (TByte) 0x31 )
#define S2LP_PCKTLEN0_ADDR                          ( (TByte) 0x32 )
#define S2LP_SYNC3_ADDR                             ( (TByte) 0x33 )
#define S2LP_SYNC2_ADDR                             ( (TByte) 0x34 )
#define S2LP_SYNC1_ADDR                             ( (TByte) 0x35 )
#define S2LP_SYNC0_ADDR                             ( (TByte) 0x36 )
#define S2LP_QI_ADDR                                ( (TByte) 0x37 )
#define S2LP_PCKT_PSTMBL_ADDR                       ( (TByte) 0x38 )
#define S2LP_PROTOCOL2_ADDR                         ( (TByte) 0x39 )
#define S2LP_PROTOCOL1_ADDR                         ( (TByte) 0x3A )
#define S2LP_PROTOCOL0_ADDR                         ( (TByte) 0x3B )
#define S2LP_FIFO_CONFIG3_ADDR                      ( (TByte) 0x3C )
#define S2LP_FIFO_CONFIG2_ADDR                      ( (TByte) 0x3D )
#define S2LP_FIFO_CONFIG1_ADDR                      ( (TByte) 0x3E )
#define S2LP_FIFO_CONFIG0_ADDR                      ( (TByte) 0x3F )
#define S2LP_PCKT_FLT_OPTIONS_ADDR                  ( (TByte) 0x40 )
#define S2LP_PCKT_FLT_GOALS4_ADDR                   ( (TByte) 0x41 )
#define S2LP_PCKT_FLT_GOALS3_ADDR                   ( (TByte) 0x42 )
#define S2LP_PCKT_FLT_GOALS2_ADDR                   ( (TByte) 0x43 )
#define S2LP_PCKT_FLT_GOALS1_ADDR                   ( (TByte) 0x44 )
#define S2LP_PCKT_FLT_GOALS0_ADDR                   ( (TByte) 0x45 )
#define S2LP_TIMERS5_ADDR                           ( (TByte) 0x46 )
#define S2LP_TIMERS4_ADDR                           ( (TByte) 0x47 )
#define S2LP_TIMERS3_ADDR                           ( (TByte) 0x48 )
#define S2LP_TIMERS2_ADDR                           ( (TByte) 0x49 )
#define S2LP_TIMERS1_ADDR                           ( (TByte) 0x4A )
#define S2LP_TIMERS0_ADDR                           ( (TByte) 0x4B )
#define S2LP_CSMA_CONF3_ADDR                        ( (TByte) 0x4C )
#define S2LP_CSMA_CONF2_ADDR                        ( (TByte) 0x4D )
#define S2LP_CSMA_CONF1_ADDR                        ( (TByte) 0x4E )
#define S2LP_CSMA_CONF0_ADDR                        ( (TByte) 0x4F )
#define S2LP_IRQ_MASK3_ADDR                         ( (TByte) 0x50 )
#define S2LP_IRQ_MASK2_ADDR                         ( (TByte) 0x51 )
#define S2LP_IRQ_MASK1_ADDR                         ( (TByte) 0x52 )
#define S2LP_IRQ_MASK0_ADDR                         ( (TByte) 0x53 )
#define S2LP_FAST_RX_TIMER_ADDR                     ( (TByte) 0x54 )
#define S2LP_PA_POWER8_ADDR                         ( (TByte) 0x5A )
#define S2LP_PA_POWER7_ADDR                         ( (TByte) 0x5B )
#define S2LP_PA_POWER6_ADDR                         ( (TByte) 0x5C )
#define S2LP_PA_POWER5_ADDR                         ( (TByte) 0x5D )
#define S2LP_PA_POWER4_ADDR                         ( (TByte) 0x5E )
#define S2LP_PA_POWER3_ADDR                         ( (TByte) 0x5F )
#define S2LP_PA_POWER2_ADDR                         ( (TByte) 0x60 )
#define S2LP_PA_POWER1_ADDR                         ( (TByte) 0x61 )
#define S2LP_PA_POWER0_ADDR                         ( (TByte) 0x62 )
#define S2LP_PA_CONFIG1_ADDR                        ( (TByte) 0x63 )
#define S2LP_PA_CONFIG0_ADDR                        ( (TByte) 0x64 )
#define S2LP_SYNTH_CONFIG2_ADDR                     ( (TByte) 0x65 )
#define S2LP_VCO_CONFIG_ADDR                        ( (TByte) 0x68 )
#define S2LP_VCO_CALIBR_IN2_ADDR                    ( (TByte) 0x69 )
#define S2LP_VCO_CALIBR_IN1_ADDR                    ( (TByte) 0x6A )
#define S2LP_VCO_CALIBR_IN0_ADDR                    ( (TByte) 0x6B )
#define S2LP_XO_RCO_CONF1_ADDR                      ( (TByte) 0x6C )
#define S2LP_XO_RCO_CONF0_ADDR                      ( (TByte) 0x6D )
#define S2LP_RCO_CALIBR_CONF3_ADDR                  ( (TByte) 0x6E )
#define S2LP_RCO_CALIBR_CONF2_ADDR                  ( (TByte) 0x6F )
#define S2LP_PM_CONF4_ADDR                          ( (TByte) 0x75 )
#define S2LP_PM_CONF3_ADDR                          ( (TByte) 0x76 )
#define S2LP_PM_CONF2_ADDR                          ( (TByte) 0x77 )
#define S2LP_PM_CONF1_ADDR                          ( (TByte) 0x78 )
#define S2LP_PM_CONF0_ADDR                          ( (TByte) 0x79 )
#define S2LP_MC_STATE1_ADDR                         ( (TByte) 0x8D )
#define S2LP_MC_STATE0_ADDR                         ( (TByte) 0x8E )
#define S2LP_TX_FIFO_STATUS_ADDR                    ( (TByte) 0x8F )
#define S2LP_RX_FIFO_STATUS_ADDR                    ( (TByte) 0x90 )
#define S2LP_RCO_CALIBR_OUT4_ADDR                   ( (TByte) 0x94 )
#define S2LP_RCO_CALIBR_OUT3_ADDR                   ( (TByte) 0x95 )
#define S2LP_VCO_CALIBR_OUT1_ADDR                   ( (TByte) 0x99 )
#define S2LP_VCO_CALIBR_OUT0_ADDR                   ( (TByte) 0x9A )
#define S2LP_TX_PCKT_INFO_ADDR                      ( (TByte) 0x9C )
#define S2LP_RX_PCKT_INFO_ADDR                      ( (TByte) 0x9D )
#define S2LP_AFC_CORR_ADDR                          ( (TByte) 0x9E )
#define S2LP_LINK_QUALIF2_ADDR                      ( (TByte) 0x9F )
#define S2LP_LINK_QUALIF1_ADDR                      ( (TByte) 0xA0 )
#define S2LP_RSSI_LEVEL_ADDR                        ( (TByte) 0xA2 )
#define S2LP_RX_PCKT_LEN1_ADDR                      ( (TByte) 0xA4 )
#define S2LP_RX_PCKT_LEN0_ADDR                      ( (TByte) 0xA5 )
#define S2LP_CRC_FIELD3_ADDR                        ( (TByte) 0xA6 )
#define S2LP_CRC_FIELD2_ADDR                        ( (TByte) 0xA7 )
#define S2LP_CRC_FIELD1_ADDR                        ( (TByte) 0xA8 )
#define S2LP_CRC_FIELD0_ADDR                        ( (TByte) 0xA9 )
#define S2LP_RX_ADDRE_FIELD1_ADDR                   ( (TByte) 0xAA )
#define S2LP_RX_ADDRE_FIELD0_ADDR                   ( (TByte) 0xAB )
#define S2LP_RSSI_LEVEL_RUN_ADDR                    ( (TByte) 0xEF )
#define S2LP_DEVICE_INFO1_ADDR                      ( (TByte) 0xF0 )
#define S2LP_DEVICE_INFO0_ADDR                      ( (TByte) 0xF1 )
#define S2LP_IRQ_STATUS3_ADDR                       ( (TByte) 0xFA )
#define S2LP_IRQ_STATUS2_ADDR                       ( (TByte) 0xFB )
#define S2LP_IRQ_STATUS1_ADDR                       ( (TByte) 0xFC )
#define S2LP_IRQ_STATUS0_ADDR                       ( (TByte) 0xFD )
#define S2LP_FIFO_ADDR                              ( (TByte) 0xFF )

/* S2-LP default register values */
#define S2LP_GPIO0_CONF_DEFAULT                     ( (TByte) 0x0A )
#define S2LP_GPIO1_CONF_DEFAULT                     ( (TByte) 0xA2 )
#define S2LP_GPIO2_CONF_DEFAULT                     ( (TByte) 0xA2 )
#define S2LP_GPIO3_CONF_DEFAULT                     ( (TByte) 0xA2 )
#define S2LP_SYNT3_DEFAULT                          ( (TByte) 0x42 )
#define S2LP_SYNT2_DEFAULT                          ( (TByte) 0x16 )
#define S2LP_SYNT1_DEFAULT                          ( (TByte) 0x27 )
#define S2LP_SYNT0_DEFAULT                          ( (TByte) 0x62 )
#define S2LP_IF_OFFSET_ANA_DEFAULT                  ( (TByte) 0x2A )
#define S2LP_IF_OFFSET_DIG_DEFAULT                  ( (TByte) 0xB8 )
#define S2LP_CHSPACE_DEFAULT                        ( (TByte) 0x3F )
#define S2LP_CHNUM_DEFAULT                          ( (TByte) 0x00 )
#define S2LP_MOD4_DEFAULT                           ( (TByte) 0x83 )
#define S2LP_MOD3_DEFAULT                           ( (TByte) 0x2B )
#define S2LP_MOD2_DEFAULT                           ( (TByte) 0x77 )
#define S2LP_MOD1_DEFAULT                           ( (TByte) 0x03 )
#define S2LP_MOD0_DEFAULT                           ( (TByte) 0x93 )
#define S2LP_CHFLT_DEFAULT                          ( (TByte) 0x23 )
#define S2LP_AFC2_DEFAULT                           ( (TByte) 0xC8 )
#define S2LP_AFC1_DEFAULT                           ( (TByte) 0x18 )
#define S2LP_AFC0_DEFAULT                           ( (TByte) 0x25 )
#define S2LP_RSSI_FLT_DEFAULT                       ( (TByte) 0xE3 )
#define S2LP_RSSI_TH_DEFAULT                        ( (TByte) 0x28 )
#define S2LP_AGCCTRL4_DEFAULT                       ( (TByte) 0x54 )
#define S2LP_AGCCTRL3_DEFAULT                       ( (TByte) 0x10 )
#define S2LP_AGCCTRL2_DEFAULT                       ( (TByte) 0x22 )
#define S2LP_AGCCTRL1_DEFAULT                       ( (TByte) 0x59 )
#define S2LP_AGCCTRL0_DEFAULT                       ( (TByte) 0x8C )
#define S2LP_ANT_SELECT_CONF_DEFAULT                ( (TByte) 0x45 )
#define S2LP_CLOCKREC2_DEFAULT                      ( (TByte) 0xC0 )
#define S2LP_CLOCKREC1_DEFAULT                      ( (TByte) 0x58 )
#define S2LP_PCKTCTRL6_DEFAULT                      ( (TByte) 0x80 )
#define S2LP_PCKTCTRL5_DEFAULT                      ( (TByte) 0x10 )
#define S2LP_PCKTCTRL4_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_PCKTCTRL3_DEFAULT                      ( (TByte) 0x20 )
#define S2LP_PCKTCTRL2_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_PCKTCTRL1_DEFAULT                      ( (TByte) 0x2C )
#define S2LP_PCKTLEN1_DEFAULT                       ( (TByte) 0x00 )
#define S2LP_PCKTLEN0_DEFAULT                       ( (TByte) 0x14 )
#define S2LP_SYNC3_DEFAULT                          ( (TByte) 0x88 )
#define S2LP_SYNC2_DEFAULT                          ( (TByte) 0x88 )
#define S2LP_SYNC1_DEFAULT                          ( (TByte) 0x88 )
#define S2LP_SYNC0_DEFAULT                          ( (TByte) 0x88 )
#define S2LP_QI_DEFAULT                             ( (TByte) 0x01 )
#define S2LP_PCKT_PSTMBL_DEFAULT                    ( (TByte) 0x00 )
#define S2LP_PROTOCOL2_DEFAULT                      ( (TByte) 0x40 )
#define S2LP_PROTOCOL1_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_PROTOCOL0_DEFAULT                      ( (TByte) 0x08 )
#define S2LP_FIFO_CONFIG3_DEFAULT                   ( (TByte) 0x30 )
#define S2LP_FIFO_CONFIG2_DEFAULT                   ( (TByte) 0x30 )
#define S2LP_FIFO_CONFIG1_DEFAULT                   ( (TByte) 0x30 )
#define S2LP_FIFO_CONFIG0_DEFAULT                   ( (TByte) 0x30 )
#define S2LP_PCKT_FLT_OPTIONS_DEFAULT               ( (TByte) 0x40 )
#define S2LP_PCKT_FLT_GOALS4_DEFAULT                ( (TByte) 0x00 )
#define S2LP_PCKT_FLT_GOALS3_DEFAULT                ( (TByte) 0x00 )
#define S2LP_PCKT_FLT_GOALS2_DEFAULT                ( (TByte) 0x00 )
#define S2LP_PCKT_FLT_GOALS1_DEFAULT                ( (TByte) 0x00 )
#define S2LP_PCKT_FLT_GOALS0_DEFAULT                ( (TByte) 0x00 )
#define S2LP_TIMERS5_DEFAULT                        ( (TByte) 0x01 )
#define S2LP_TIMERS4_DEFAULT                        ( (TByte) 0x00 )
#define S2LP_TIMERS3_DEFAULT                        ( (TByte) 0x01 )
#define S2LP_TIMERS2_DEFAULT                        ( (TByte) 0x00 )
#define S2LP_TIMERS1_DEFAULT                        ( (TByte) 0x01 )
#define S2LP_TIMERS0_DEFAULT                        ( (TByte) 0x00 )
#define S2LP_CSMA_CONF3_DEFAULT                     ( (TByte) 0x4C )
#define S2LP_CSMA_CONF2_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_CSMA_CONF1_DEFAULT                     ( (TByte) 0x04 )
#define S2LP_CSMA_CONF0_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_IRQ_MASK3_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_IRQ_MASK2_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_IRQ_MASK1_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_IRQ_MASK0_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_FAST_RX_TIMER_DEFAULT                  ( (TByte) 0x28 )
#define S2LP_PA_POWER8_DEFAULT                      ( (TByte) 0x01 )
#define S2LP_PA_POWER7_DEFAULT                      ( (TByte) 0x0C )
#define S2LP_PA_POWER6_DEFAULT                      ( (TByte) 0x18 )
#define S2LP_PA_POWER5_DEFAULT                      ( (TByte) 0x24 )
#define S2LP_PA_POWER4_DEFAULT                      ( (TByte) 0x30 )
#define S2LP_PA_POWER3_DEFAULT                      ( (TByte) 0x48 )
#define S2LP_PA_POWER2_DEFAULT                      ( (TByte) 0x60 )
#define S2LP_PA_POWER1_DEFAULT                      ( (TByte) 0x00 )
#define S2LP_PA_POWER0_DEFAULT                      ( (TByte) 0x47 )
#define S2LP_PA_CONFIG1_DEFAULT                     ( (TByte) 0x03 )
#define S2LP_PA_CONFIG0_DEFAULT                     ( (TByte) 0x8A )
#define S2LP_SYNTH_CONFIG2_DEFAULT                  ( (TByte) 0xD0 )
#define S2LP_VCO_CONFIG_DEFAULT                     ( (TByte) 0x03 )
#define S2LP_VCO_CALIBR_IN2_DEFAULT                 ( (TByte) 0x88 )
#define S2LP_VCO_CALIBR_IN1_DEFAULT                 ( (TByte) 0x40 )
#define S2LP_VCO_CALIBR_IN0_DEFAULT                 ( (TByte) 0x40 )
#define S2LP_XO_RCO_CONF1_DEFAULT                   ( (TByte) 0x45 )
#define S2LP_XO_RCO_CONF0_DEFAULT                   ( (TByte) 0x30 )
#define S2LP_RCO_CALIBR_CONF3_DEFAULT               ( (TByte) 0x70 )
#define S2LP_RCO_CALIBR_CONF2_DEFAULT               ( (TByte) 0x4D )
#define S2LP_PM_CONF4_DEFAULT                       ( (TByte) 0x17 )
#define S2LP_PM_CONF3_DEFAULT                       ( (TByte) 0x20 )
#define S2LP_PM_CONF2_DEFAULT                       ( (TByte) 0x00 )
#define S2LP_PM_CONF1_DEFAULT                       ( (TByte) 0x39 )
#define S2LP_PM_CONF0_DEFAULT                       ( (TByte) 0x42 )
#define S2LP_MC_STATE1_DEFAULT                      ( (TByte) 0x52 )
#define S2LP_MC_STATE0_DEFAULT                      ( (TByte) 0x07 )
#define S2LP_TX_FIFO_STATUS_DEFAULT                 ( (TByte) 0x00 )
#define S2LP_RX_FIFO_STATUS_DEFAULT                 ( (TByte) 0x00 )
#define S2LP_RCO_CALIBR_OUT4_DEFAULT                ( (TByte) 0x70 )
#define S2LP_RCO_CALIBR_OUT3_DEFAULT                ( (TByte) 0x00 )
#define S2LP_VCO_CALIBR_OUT1_DEFAULT                ( (TByte) 0x00 )
#define S2LP_VCO_CALIBR_OUT0_DEFAULT                ( (TByte) 0x00 )
#define S2LP_TX_PCKT_INFO_DEFAULT                   ( (TByte) 0x00 )
#define S2LP_RX_PCKT_INFO_DEFAULT                   ( (TByte) 0x00 )
#define S2LP_AFC_CORR_DEFAULT                       ( (TByte) 0x00 )
#define S2LP_LINK_QUALIF2_DEFAULT                   ( (TByte) 0x00 )
#define S2LP_LINK_QUALIF1_DEFAULT                   ( (TByte) 0x00 )
#define S2LP_RSSI_LEVEL_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_RX_PCKT_LEN1_DEFAULT                   ( (TByte) 0x00 )
#define S2LP_RX_PCKT_LEN0_DEFAULT                   ( (TByte) 0x00 )
#define S2LP_CRC_FIELD3_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_CRC_FIELD2_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_CRC_FIELD1_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_CRC_FIELD0_DEFAULT                     ( (TByte) 0x00 )
#define S2LP_RX_ADDRE_FIELD1_DEFAULT                ( (TByte) 0x00 )
#define S2LP_RX_ADDRE_FIELD0_DEFAULT                ( (TByte) 0x00 )
#define S2LP_RSSI_LEVEL_RUN_DEFAULT                 ( (TByte) 0x00 )
#define S2LP_DEVICE_INFO1_DEFAULT                   ( (TByte) 0x03 )
#define S2LP_DEVICE_INFO0_DEFAULT                   ( (TByte) 0xC1 )
#define S2LP_IRQ_STATUS3_DEFAULT                    ( (TByte) 0x00 )
#define S2LP_IRQ_STATUS2_DEFAULT                    ( (TByte) 0x09 )
#define S2LP_IRQ_STATUS1_DEFAULT                    ( (TByte) 0x05 )
#define S2LP_IRQ_STATUS0_DEFAULT                    ( (TByte) 0x00 )

/* Interrupt bits mapping in the IRQ_STATUS word */
#define S2LP_EVENT_RX_DATA_READY                    ( (TWord) 0x00000001 )
#define S2LP_EVENT_RX_DATA_DISCARDED                ( (TWord) 0x00000002 )
#define S2LP_EVENT_TX_DATA_SENT                     ( (TWord) 0x00000004 )
#define S2LP_EVENT_MAX_RE_TX_REACHED                ( (TWord) 0x00000008 )
#define S2LP_EVENT_CRC_ERROR                        ( (TWord) 0x00000010 )
#define S2LP_EVENT_TX_FIFO_UF_OF_ERROR              ( (TWord) 0x00000020 )
#define S2LP_EVENT_RX_FIFO_UF_OF_ERROR              ( (TWord) 0x00000040 )
#define S2LP_EVENT_TX_FIFO_ALMOST_FULL              ( (TWord) 0x00000080 )
#define S2LP_EVENT_TX_FIFO_ALMOST_EMPTY             ( (TWord) 0x00000100 )
#define S2LP_EVENT_RX_FIFO_ALMOST_FULL              ( (TWord) 0x00000200 )
#define S2LP_EVENT_RX_FIFO_ALMOST_EMPTY             ( (TWord) 0x00000400 )
#define S2LP_EVENT_MAX_NUM_OF_BACK_OFF_CCA          ( (TWord) 0x00000800 )
#define S2LP_EVENT_VALID_PREAMBLE_DETECTED          ( (TWord) 0x00001000 )
#define S2LP_EVENT_SYNC_WORD_DETECTED               ( (TWord) 0x00002000 )
#define S2LP_EVENT_RSSI_ABOVE_THRESHOLD             ( (TWord) 0x00004000 )
#define S2LP_EVENT_WAKE_UP_TIMEOUT_LDCR             ( (TWord) 0x00008000 )
#define S2LP_EVENT_READY                            ( (TWord) 0x00010000 )
#define S2LP_EVENT_STANDBY_STATE_SWITCHING          ( (TWord) 0x00020000 )
#define S2LP_EVENT_LOW_BATTERY_LEVEL                ( (TWord) 0x00040000 )
#define S2LP_EVENT_POWER_ON_RESET                   ( (TWord) 0x00080000 )
#define S2LP_EVENT_RX_TIMER_TIMEOUT                 ( (TWord) 0x10000000 )
#define S2LP_EVENT_SNIFF_TIMER_TIMEOUT              ( (TWord) 0x20000000 )

/* Modulation scheme bits mapping in the MOD2 register */
#define S2LP_MOD_2_FSK                              ( (TByte) (0b000 << 4) )
#define S2LP_MOD_4_FSK                              ( (TByte) (0b001 << 4) )
#define S2LP_MOD_2_GFSK                             ( (TByte) (0b010 << 4) )
#define S2LP_MOD_4_GFSK                             ( (TByte) (0b011 << 4) )
#define S2LP_MOD_ASK_OOK                            ( (TByte) (0b101 << 4) )
#define S2LP_MOD_CW                                 ( (TByte) (0b111 << 4) )

/* GPIO signal select bits mapping in the GPIOx_CONF registers */
#define S2LP_GPIO_SEL_NIRQ                          ( (TByte) (0 << 3) )
#define S2LP_GPIO_SEL_POR_INVERTED                  ( (TByte) (1 << 3) )
#define S2LP_GPIO_SEL_WUT_EXPIRATION                ( (TByte) (2 << 3) )
#define S2LP_GPIO_SEL_LOW_BATTERY                   ( (TByte) (3 << 3) )
#define S2LP_GPIO_SEL_TX_DATA_INTERNAL_CLOCK        ( (TByte) (4 << 3) )
#define S2LP_GPIO_SEL_TX_STATE_RADIO_TX             ( (TByte) (5 << 3) )
#define S2LP_GPIO_SEL_TX_RX_FIFO_ALMOST_EMPTY       ( (TByte) (6 << 3) )
#define S2LP_GPIO_SEL_TX_RX_FIFO_ALMOST_FULL        ( (TByte) (7 << 3) )
#define S2LP_GPIO_SEL_RX_DATA_OUTPUT                ( (TByte) (8 << 3) )
#define S2LP_GPIO_SEL_RX_CLOCK_OUTPUT               ( (TByte) (9 << 3) )
#define S2LP_GPIO_SEL_RX_STATE_INDICATION           ( (TByte) (10 << 3) )
#define S2LP_GPIO_SEL_NOT_SLEEP_OR_STANDBY          ( (TByte) (11 << 3) )
#define S2LP_GPIO_SEL_DEVICE_IN_STANDBY_STATE       ( (TByte) (12 << 3) )
#define S2LP_GPIO_SEL_ANTENNA_SWITCH_SIGNAL         ( (TByte) (13 << 3) )
#define S2LP_GPIO_SEL_VALID_PREAMBLE_DETECTED       ( (TByte) (14 << 3) )
#define S2LP_GPIO_SEL_SYNC_WORD_DETECTED            ( (TByte) (15 << 3) )
#define S2LP_GPIO_SEL_RSSI_ABOVE_THRESHOLD          ( (TByte) (16 << 3) )
#define S2LP_GPIO_SEL_TX_RX_MODE_INDICATOR          ( (TByte) (18 << 3) )
#define S2LP_GPIO_SEL_VDD                           ( (TByte) (19 << 3) )
#define S2LP_GPIO_SEL_GND                           ( (TByte) (20 << 3) )
#define S2LP_GPIO_SEL_SMPS_ENABLE_SIGNAL            ( (TByte) (21 << 3) )
#define S2LP_GPIO_SEL_DEVICE_IN_SLEEP_STATE         ( (TByte) (22 << 3) )
#define S2LP_GPIO_SEL_DEVICE_IN_READY_STATE         ( (TByte) (23 << 3) )
#define S2LP_GPIO_SEL_DEVICE_IN_LOCK_STATE          ( (TByte) (24 << 3) )
#define S2LP_GPIO_SEL_WAIT_FOR_LOCK_DETECTOR        ( (TByte) (25 << 3) )
#define S2LP_GPIO_SEL_TX_DATA_OOK_SIGNAL            ( (TByte) (26 << 3) )
#define S2LP_GPIO_SEL_WAIT_FOR_XO_READY_SIGNAL      ( (TByte) (27 << 3) )
#define S2LP_GPIO_SEL_WAIT_FOR_TIMER_EXP            ( (TByte) (28 << 3) )
#define S2LP_GPIO_SEL_WAIT_FOR_VCO_CALIB            ( (TByte) (29 << 3) )
#define S2LP_GPIO_SEL_SYNTH_BLOCK_ENABLED           ( (TByte) (30 << 3) )
#define S2LP_GPIO_SEL_TX_COMMAND                    ( (TByte) (0 << 3) )
#define S2LP_GPIO_SEL_RX_COMMAND                    ( (TByte) (1 << 3) )
#define S2LP_GPIO_SEL_TX_DATA_INPUT                 ( (TByte) (2 << 3) )
#define S2LP_GPIO_SEL_WAKE_UP_FROM_EXT_INPUT        ( (TByte) (3 << 3) )
#define S2LP_GPIO_SEL_EXT_CLOCK_AT_34_7_Hz          ( (TByte) (4 << 3) )

/* GPIO mode bits mapping in the GPIOx_CONF registers */
#define S2LP_GPIO_MODE_INPUT                        ( (TByte) 0b01 )
#define S2LP_GPIO_MODE_OUTPUT_LOW_POWER             ( (TByte) 0b10 )
#define S2LP_GPIO_MODE_OUTPUT_HIGH_POWER            ( (TByte) 0b11 )

/*---------------------------------------------- Macros ----------------------------------------------*/

#define IS_S2LP_COMMAND(command)                    ( (command) == S2LP_TX_COMMAND || \
                                                      (command) == S2LP_RX_COMMAND || \
                                                      (command) == S2LP_READY_COMMAND || \
                                                      (command) == S2LP_STANDBY_COMMAND || \
                                                      (command) == S2LP_SLEEP_COMMAND || \
                                                      (command) == S2LP_LOCKRX_COMMAND || \
                                                      (command) == S2LP_LOCKTX_COMMAND || \
                                                      (command) == S2LP_SABORT_COMMAND || \
                                                      (command) == S2LP_LDC_RELOAD_COMMAND || \
                                                      (command) == S2LP_SRES_COMMAND || \
                                                      (command) == S2LP_FLUSHRXFIFO_COMMAND || \
                                                      (command) == S2LP_FLUSHTXFIFO_COMMAND || \
                                                      (command) == S2LP_SEQUENCE_UPDATE_COMMAND )

/* Set bits in a register based on a mask */
#define SET_BITS(REG, MASK, BITS)                     ( (REG) = ( ( (TByte)(REG) & ~( (TByte)(MASK) ) ) | \
                                                                ( ( (TByte)(MASK) ) & ( (TByte)(BITS) ) ) ) )

/* Change byte order in a word */
#define SWAP_ENDIAN_32(word)                          ( ( (word) & 0xFF000000 ) >> 24 | \
                                                        ( (word) & 0x00FF0000 ) >> 8 | \
                                                        ( (word) & 0x0000FF00 ) << 8 | \
                                                        ( (word) & 0x000000FF ) << 24 )

/* Change byte order in a halfword */
#define SWAP_ENDIAN_16(halfword)                      ( ( (halfword) & 0xFF00 ) >> 8 | \
                                                        ( (halfword) & 0x00FF ) << 8  )

#define S2lpLld_WriteRegByValue(address, value)  do { \
    TByte temp = value; \
    S2lpLld_WriteReg(address, &temp, 1, NULL); \
} while(0)

/*---------------------------------------------- Function prototypes ----------------------------------------------*/

/**
 * @brief Reset the device
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_Reset(void);

/**
 * @brief Write data to S2-LP's registers
 * @param address S2-LP memory map address
 * @param bufPtr Data to be written
 * @param numOfBytes Number of bytes to be written
 * @param s2lpStatusBitsBufPtr Buffer to pass the status register contents out of the function (should be NULL if not used)
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_WriteReg(TByte address, TByte *bufPtr, TSize numOfBytes, THalfWord* s2lpStatusBitsBufPtr);

/**
 * @brief Read data from S2-LP's registers
 * @param address S2-LP memory map address
 * @param bufPtr Buffer for the read data
 * @param numOfBytes Number of bytes to be read
 * @param s2lpStatusBitsBufPtr Buffer to pass the status register contents out of the function (should be NULL if not used)
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_ReadReg(TByte address, TByte *bufPtr, TSize numOfBytes, THalfWord* s2lpStatusBitsBufPtr);

/**
 * @brief Send a command to S2-LP via SPI
 * @param command S2-LP command code
 * @param s2lpStatusBitsBufPtr Buffer to pass the status register contents out of the function (should be NULL if not used)
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_SendCommand(TByte command, THalfWord* s2lpStatusBitsBufPtr);

#endif /* __S2LP_LLD_H_ */
