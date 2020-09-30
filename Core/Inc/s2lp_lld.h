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
	ES2lpLldRet_Ok = 0,
	ES2lpLldRet_Error
} ES2lpLldRet;


/*---------------------------------------------- Defines ----------------------------------------------*/

/* S2-LP command list */
#define TX_COMMAND                ((TByte) 0x60)
#define RX_COMMAND                ((TByte) 0x61)
#define READY_COMMAND             ((TByte) 0x62)
#define STANDBY_COMMAND           ((TByte) 0x63)
#define SLEEP_COMMAND             ((TByte) 0x64)
#define LOCKRX_COMMAND            ((TByte) 0x65)
#define LOCKTX_COMMAND            ((TByte) 0x66)
#define SABORT_COMMAND            ((TByte) 0x67)
#define LDC_RELOAD_COMMAND        ((TByte) 0x68)
#define SRES_COMMAND              ((TByte) 0x70)
#define FLUSHRXFIFO_COMMAND       ((TByte) 0x71)
#define FLUSHTXFIFO_COMMAND       ((TByte) 0x72)
#define SEQUENCE_UPDATE_COMMAND   ((TByte) 0x73)

/* S2-LP register addresses */
#define GPIO0_CONF_ADDR           ((TByte) 0x00)
#define GPIO1_CONF_ADDR           ((TByte) 0x01)
#define GPIO2_CONF_ADDR           ((TByte) 0x02)
#define GPIO3_CONF_ADDR           ((TByte) 0x03)
#define SYNT3_ADDR                ((TByte) 0x05)
#define SYNT2_ADDR                ((TByte) 0x06)
#define SYNT1_ADDR                ((TByte) 0x07)
#define SYNT0_ADDR                ((TByte) 0x08)
#define IF_OFFSET_ANA_ADDR        ((TByte) 0x09)
#define IF_OFFSET_DIG_ADDR        ((TByte) 0x0A)
#define CHSPACE_ADDR              ((TByte) 0x0C)
#define CHNUM_ADDR                ((TByte) 0x0D)
#define MOD4_ADDR                 ((TByte) 0x0E)
#define MOD3_ADDR                 ((TByte) 0x0F)
#define MOD2_ADDR                 ((TByte) 0x10)
#define MOD1_ADDR                 ((TByte) 0x11)
#define MOD0_ADDR                 ((TByte) 0x12)
#define CHFLT_ADDR                ((TByte) 0x13)
#define AFC2_ADDR                 ((TByte) 0x14)
#define AFC1_ADDR                 ((TByte) 0x15)
#define AFC0_ADDR                 ((TByte) 0x16)
#define RSSI_FLT_ADDR             ((TByte) 0x17)
#define RSSI_TH_ADDR              ((TByte) 0x18)
#define AGCCTRL4_ADDR             ((TByte) 0x1A)
#define AGCCTRL3_ADDR             ((TByte) 0x1B)
#define AGCCTRL2_ADDR             ((TByte) 0x1C)
#define AGCCTRL1_ADDR             ((TByte) 0x1D)
#define AGCCTRL0_ADDR             ((TByte) 0x1E)
#define ANT_SELECT_CONF_ADDR      ((TByte) 0x1F)
#define CLOCKREC2_ADDR            ((TByte) 0x20)
#define CLOCKREC1_ADDR            ((TByte) 0x21)
#define PCKTCTRL6_ADDR            ((TByte) 0x2B)
#define PCKTCTRL5_ADDR            ((TByte) 0x2C)
#define PCKTCTRL4_ADDR            ((TByte) 0x2D)
#define PCKTCTRL3_ADDR            ((TByte) 0x2E)
#define PCKTCTRL2_ADDR            ((TByte) 0x2F)
#define PCKTCTRL1_ADDR            ((TByte) 0x30)
#define PCKTLEN1_ADDR             ((TByte) 0x31)
#define PCKTLEN0_ADDR             ((TByte) 0x32)
#define SYNC3_ADDR                ((TByte) 0x33)
#define SYNC2_ADDR                ((TByte) 0x34)
#define SYNC1_ADDR                ((TByte) 0x35)
#define SYNC0_ADDR                ((TByte) 0x36)
#define QI_ADDR                   ((TByte) 0x37)
#define PCKT_PSTMBL_ADDR          ((TByte) 0x38)
#define PROTOCOL2_ADDR            ((TByte) 0x39)
#define PROTOCOL1_ADDR            ((TByte) 0x3A)
#define PROTOCOL0_ADDR            ((TByte) 0x3B)
#define FIFO_CONFIG3_ADDR         ((TByte) 0x3C)
#define FIFO_CONFIG2_ADDR         ((TByte) 0x3D)
#define FIFO_CONFIG1_ADDR         ((TByte) 0x3E)
#define FIFO_CONFIG0_ADDR         ((TByte) 0x3F)
#define PCKT_FLT_OPTIONS_ADDR     ((TByte) 0x40)
#define PCKT_FLT_GOALS4_ADDR      ((TByte) 0x41)
#define PCKT_FLT_GOALS3_ADDR      ((TByte) 0x42)
#define PCKT_FLT_GOALS2_ADDR      ((TByte) 0x43)
#define PCKT_FLT_GOALS1_ADDR      ((TByte) 0x44)
#define PCKT_FLT_GOALS0_ADDR      ((TByte) 0x45)
#define TIMERS5_ADDR              ((TByte) 0x46)
#define TIMERS4_ADDR              ((TByte) 0x47)
#define TIMERS3_ADDR              ((TByte) 0x48)
#define TIMERS2_ADDR              ((TByte) 0x49)
#define TIMERS1_ADDR              ((TByte) 0x4A)
#define TIMERS0_ADDR              ((TByte) 0x4B)
#define CSMA_CONF3_ADDR           ((TByte) 0x4C)
#define CSMA_CONF2_ADDR           ((TByte) 0x4D)
#define CSMA_CONF1_ADDR           ((TByte) 0x4E)
#define CSMA_CONF0_ADDR           ((TByte) 0x4F)
#define IRQ_MASK3_ADDR            ((TByte) 0x50)
#define IRQ_MASK2_ADDR            ((TByte) 0x51)
#define IRQ_MASK1_ADDR            ((TByte) 0x52)
#define IRQ_MASK0_ADDR            ((TByte) 0x53)
#define FAST_RX_TIMER_ADDR        ((TByte) 0x54)
#define PA_POWER8_ADDR            ((TByte) 0x5A)
#define PA_POWER7_ADDR            ((TByte) 0x5B)
#define PA_POWER6_ADDR            ((TByte) 0x5C)
#define PA_POWER5_ADDR            ((TByte) 0x5D)
#define PA_POWER4_ADDR            ((TByte) 0x5E)
#define PA_POWER3_ADDR            ((TByte) 0x5F)
#define PA_POWER2_ADDR            ((TByte) 0x60)
#define PA_POWER1_ADDR            ((TByte) 0x61)
#define PA_POWER0_ADDR            ((TByte) 0x62)
#define PA_CONFIG1_ADDR           ((TByte) 0x63)
#define PA_CONFIG0_ADDR           ((TByte) 0x64)
#define SYNTH_CONFIG2_ADDR        ((TByte) 0x65)
#define VCO_CONFIG_ADDR           ((TByte) 0x68)
#define VCO_CALIBR_IN2_ADDR       ((TByte) 0x69)
#define VCO_CALIBR_IN1_ADDR       ((TByte) 0x6A)
#define VCO_CALIBR_IN0_ADDR       ((TByte) 0x6B)
#define XO_RCO_CONF1_ADDR         ((TByte) 0x6C)
#define XO_RCO_CONF0_ADDR         ((TByte) 0x6D)
#define RCO_CALIBR_CONF3_ADDR     ((TByte) 0x6E)
#define RCO_CALIBR_CONF2_ADDR     ((TByte) 0x6F)
#define PM_CONF4_ADDR             ((TByte) 0x75)
#define PM_CONF3_ADDR             ((TByte) 0x76)
#define PM_CONF2_ADDR             ((TByte) 0x77)
#define PM_CONF1_ADDR             ((TByte) 0x78)
#define PM_CONF0_ADDR             ((TByte) 0x79)
#define MC_STATE1_ADDR            ((TByte) 0x8D)
#define MC_STATE0_ADDR            ((TByte) 0x8E)
#define TX_FIFO_STATUS_ADDR       ((TByte) 0x8F)
#define RX_FIFO_STATUS_ADDR       ((TByte) 0x90)
#define RCO_CALIBR_OUT4_ADDR      ((TByte) 0x94)
#define RCO_CALIBR_OUT3_ADDR      ((TByte) 0x95)
#define VCO_CALIBR_OUT1_ADDR      ((TByte) 0x99)
#define VCO_CALIBR_OUT0_ADDR      ((TByte) 0x9A)
#define TX_PCKT_INFO_ADDR         ((TByte) 0x9C)
#define RX_PCKT_INFO_ADDR         ((TByte) 0x9D)
#define AFC_CORR_ADDR             ((TByte) 0x9E)
#define LINK_QUALIF2_ADDR         ((TByte) 0x9F)
#define LINK_QUALIF1_ADDR         ((TByte) 0xA0)
#define RSSI_LEVEL_ADDR           ((TByte) 0xA2)
#define RX_PCKT_LEN1_ADDR         ((TByte) 0xA4)
#define RX_PCKT_LEN0_ADDR         ((TByte) 0xA5)
#define CRC_FIELD3_ADDR           ((TByte) 0xA6)
#define CRC_FIELD2_ADDR           ((TByte) 0xA7)
#define CRC_FIELD1_ADDR           ((TByte) 0xA8)
#define CRC_FIELD0_ADDR           ((TByte) 0xA9)
#define RX_ADDRE_FIELD1_ADDR      ((TByte) 0xAA)
#define RX_ADDRE_FIELD0_ADDR      ((TByte) 0xAB)
#define RSSI_LEVEL_RUN_ADDR       ((TByte) 0xEF)
#define DEVICE_INFO1_ADDR         ((TByte) 0xF0)
#define DEVICE_INFO0_ADDR         ((TByte) 0xF1)
#define IRQ_STATUS3_ADDR          ((TByte) 0xFA)
#define IRQ_STATUS2_ADDR          ((TByte) 0xFB)
#define IRQ_STATUS1_ADDR          ((TByte) 0xFC)
#define IRQ_STATUS0_ADDR          ((TByte) 0xFD)
#define FIFO_ADDR                 ((TByte) 0xFF)

/* S2-LP default register values */
#define GPIO0_CONF_DEFAULT        ((TByte) 0x0A)
#define GPIO1_CONF_DEFAULT        ((TByte) 0xA2)
#define GPIO2_CONF_DEFAULT        ((TByte) 0xA2)
#define GPIO3_CONF_DEFAULT        ((TByte) 0xA2)
#define SYNT3_DEFAULT             ((TByte) 0x42)
#define SYNT2_DEFAULT             ((TByte) 0x16)
#define SYNT1_DEFAULT             ((TByte) 0x27)
#define SYNT0_DEFAULT             ((TByte) 0x62)
#define IF_OFFSET_ANA_DEFAULT     ((TByte) 0x2A)
#define IF_OFFSET_DIG_DEFAULT     ((TByte) 0xB8)
#define CHSPACE_DEFAULT           ((TByte) 0x3F)
#define CHNUM_DEFAULT             ((TByte) 0x00)
#define MOD4_DEFAULT              ((TByte) 0x83)
#define MOD3_DEFAULT              ((TByte) 0x2B)
#define MOD2_DEFAULT              ((TByte) 0x77)
#define MOD1_DEFAULT              ((TByte) 0x03)
#define MOD0_DEFAULT              ((TByte) 0x93)
#define CHFLT_DEFAULT             ((TByte) 0x23)
#define AFC2_DEFAULT              ((TByte) 0xC8)
#define AFC1_DEFAULT              ((TByte) 0x18)
#define AFC0_DEFAULT              ((TByte) 0x25)
#define RSSI_FLT_DEFAULT          ((TByte) 0xE3)
#define RSSI_TH_DEFAULT           ((TByte) 0x28)
#define AGCCTRL4_DEFAULT          ((TByte) 0x54)
#define AGCCTRL3_DEFAULT          ((TByte) 0x10)
#define AGCCTRL2_DEFAULT          ((TByte) 0x22)
#define AGCCTRL1_DEFAULT          ((TByte) 0x59)
#define AGCCTRL0_DEFAULT          ((TByte) 0x8C)
#define ANT_SELECT_CONF_DEFAULT   ((TByte) 0x45)
#define CLOCKREC2_DEFAULT         ((TByte) 0xC0)
#define CLOCKREC1_DEFAULT         ((TByte) 0x58)
#define PCKTCTRL6_DEFAULT         ((TByte) 0x80)
#define PCKTCTRL5_DEFAULT         ((TByte) 0x10)
#define PCKTCTRL4_DEFAULT         ((TByte) 0x00)
#define PCKTCTRL3_DEFAULT         ((TByte) 0x20)
#define PCKTCTRL2_DEFAULT         ((TByte) 0x00)
#define PCKTCTRL1_DEFAULT         ((TByte) 0x2C)
#define PCKTLEN1_DEFAULT          ((TByte) 0x00)
#define PCKTLEN0_DEFAULT          ((TByte) 0x14)
#define SYNC3_DEFAULT             ((TByte) 0x88)
#define SYNC2_DEFAULT             ((TByte) 0x88)
#define SYNC1_DEFAULT             ((TByte) 0x88)
#define SYNC0_DEFAULT             ((TByte) 0x88)
#define QI_DEFAULT                ((TByte) 0x01)
#define PCKT_PSTMBL_DEFAULT       ((TByte) 0x00)
#define PROTOCOL2_DEFAULT         ((TByte) 0x40)
#define PROTOCOL1_DEFAULT         ((TByte) 0x00)
#define PROTOCOL0_DEFAULT         ((TByte) 0x08)
#define FIFO_CONFIG3_DEFAULT      ((TByte) 0x30)
#define FIFO_CONFIG2_DEFAULT      ((TByte) 0x30)
#define FIFO_CONFIG1_DEFAULT      ((TByte) 0x30)
#define FIFO_CONFIG0_DEFAULT      ((TByte) 0x30)
#define PCKT_FLT_OPTIONS_DEFAULT  ((TByte) 0x40)
#define PCKT_FLT_GOALS4_DEFAULT   ((TByte) 0x00)
#define PCKT_FLT_GOALS3_DEFAULT   ((TByte) 0x00)
#define PCKT_FLT_GOALS2_DEFAULT   ((TByte) 0x00)
#define PCKT_FLT_GOALS1_DEFAULT   ((TByte) 0x00)
#define PCKT_FLT_GOALS0_DEFAULT   ((TByte) 0x00)
#define TIMERS5_DEFAULT           ((TByte) 0x01)
#define TIMERS4_DEFAULT           ((TByte) 0x00)
#define TIMERS3_DEFAULT           ((TByte) 0x01)
#define TIMERS2_DEFAULT           ((TByte) 0x00)
#define TIMERS1_DEFAULT           ((TByte) 0x01)
#define TIMERS0_DEFAULT           ((TByte) 0x00)
#define CSMA_CONF3_DEFAULT        ((TByte) 0x4C)
#define CSMA_CONF2_DEFAULT        ((TByte) 0x00)
#define CSMA_CONF1_DEFAULT        ((TByte) 0x04)
#define CSMA_CONF0_DEFAULT        ((TByte) 0x00)
#define IRQ_MASK3_DEFAULT         ((TByte) 0x00)
#define IRQ_MASK2_DEFAULT         ((TByte) 0x00)
#define IRQ_MASK1_DEFAULT         ((TByte) 0x00)
#define IRQ_MASK0_DEFAULT         ((TByte) 0x00)
#define FAST_RX_TIMER_DEFAULT     ((TByte) 0x28)
#define PA_POWER8_DEFAULT         ((TByte) 0x01)
#define PA_POWER7_DEFAULT         ((TByte) 0x0C)
#define PA_POWER6_DEFAULT         ((TByte) 0x18)
#define PA_POWER5_DEFAULT         ((TByte) 0x24)
#define PA_POWER4_DEFAULT         ((TByte) 0x30)
#define PA_POWER3_DEFAULT         ((TByte) 0x48)
#define PA_POWER2_DEFAULT         ((TByte) 0x60)
#define PA_POWER1_DEFAULT         ((TByte) 0x00)
#define PA_POWER0_DEFAULT         ((TByte) 0x47)
#define PA_CONFIG1_DEFAULT        ((TByte) 0x03)
#define PA_CONFIG0_DEFAULT        ((TByte) 0x8A)
#define SYNTH_CONFIG2_DEFAULT     ((TByte) 0xD0)
#define VCO_CONFIG_DEFAULT        ((TByte) 0x03)
#define VCO_CALIBR_IN2_DEFAULT    ((TByte) 0x88)
#define VCO_CALIBR_IN1_DEFAULT    ((TByte) 0x40)
#define VCO_CALIBR_IN0_DEFAULT    ((TByte) 0x40)
#define XO_RCO_CONF1_DEFAULT      ((TByte) 0x45)
#define XO_RCO_CONF0_DEFAULT      ((TByte) 0x30)
#define RCO_CALIBR_CONF3_DEFAULT  ((TByte) 0x70)
#define RCO_CALIBR_CONF2_DEFAULT  ((TByte) 0x4D)
#define PM_CONF4_DEFAULT          ((TByte) 0x17)
#define PM_CONF3_DEFAULT          ((TByte) 0x20)
#define PM_CONF2_DEFAULT          ((TByte) 0x00)
#define PM_CONF1_DEFAULT          ((TByte) 0x39)
#define PM_CONF0_DEFAULT          ((TByte) 0x42)
#define MC_STATE1_DEFAULT         ((TByte) 0x52)
#define MC_STATE0_DEFAULT         ((TByte) 0x07)
#define TX_FIFO_STATUS_DEFAULT    ((TByte) 0x00)
#define RX_FIFO_STATUS_DEFAULT    ((TByte) 0x00)
#define RCO_CALIBR_OUT4_DEFAULT   ((TByte) 0x70)
#define RCO_CALIBR_OUT3_DEFAULT   ((TByte) 0x00)
#define VCO_CALIBR_OUT1_DEFAULT   ((TByte) 0x00)
#define VCO_CALIBR_OUT0_DEFAULT   ((TByte) 0x00)
#define TX_PCKT_INFO_DEFAULT      ((TByte) 0x00)
#define RX_PCKT_INFO_DEFAULT      ((TByte) 0x00)
#define AFC_CORR_DEFAULT          ((TByte) 0x00)
#define LINK_QUALIF2_DEFAULT      ((TByte) 0x00)
#define LINK_QUALIF1_DEFAULT      ((TByte) 0x00)
#define RSSI_LEVEL_DEFAULT        ((TByte) 0x00)
#define RX_PCKT_LEN1_DEFAULT      ((TByte) 0x00)
#define RX_PCKT_LEN0_DEFAULT      ((TByte) 0x00)
#define CRC_FIELD3_DEFAULT        ((TByte) 0x00)
#define CRC_FIELD2_DEFAULT        ((TByte) 0x00)
#define CRC_FIELD1_DEFAULT        ((TByte) 0x00)
#define CRC_FIELD0_DEFAULT        ((TByte) 0x00)
#define RX_ADDRE_FIELD1_DEFAULT   ((TByte) 0x00)
#define RX_ADDRE_FIELD0_DEFAULT   ((TByte) 0x00)
#define RSSI_LEVEL_RUN_DEFAULT    ((TByte) 0x00)
#define DEVICE_INFO1_DEFAULT      ((TByte) 0x03)
#define DEVICE_INFO0_DEFAULT      ((TByte) 0xC1)
#define IRQ_STATUS3_DEFAULT       ((TByte) 0x00)
#define IRQ_STATUS2_DEFAULT       ((TByte) 0x09)
#define IRQ_STATUS1_DEFAULT       ((TByte) 0x05)
#define IRQ_STATUS0_DEFAULT       ((TByte) 0x00)

/*---------------------------------------------- Function prototypes ----------------------------------------------*/
/**
 * @brief Write data to S2-LP's registers
 * @param address S2-LP memory map address
 * @param bufPtr Data to be written
 * @param numOfBytes Number of bytes to be written
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_WriteReg(TByte address, TByte* bufPtr, TSize numOfBytes);

/**
 * @brief Read data from S2-LP's registers
 * @param address S2-LP memory map address
 * @param[out] bufPtr Buffer for the read data
 * @param numOfBytes Number of bytes to be read
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_ReadReg(TByte address, TByte* bufPtr, TSize numOfBytes);

/**
 * @brief Send a command to S2-LP via SPI
 * @param command S2-LP command code
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_SendCommand(TByte command);

#endif /* __S2LP_LLD_H_ */
