/*********************************************************************** 
*Copyright(c)2008,by Gohigh Data Networks Technology Co.,LTD 
*All rights reserved.
*
*Appiled body:
*Project name:	
*Project function:
*Header file:
*File list:
*Author:vehicle to everything
*Date:
*Version:			v0.0.1
*Version history:	None
*说明：
*
**********************************************************************/
#ifndef __SDIO_H__
#define __SDIO_H__

/* MMC Register defines */
#define DD_SDMMC_TIMEOUT                        (-290002)

#define TF_card          0
#define SDIO_card     2
#define EMMC             1

#define ON                  1   
#define OFF                0   
#define PMU_I2C       0
#define CMD6             6

#define MMCI_RCA        0x12340000

/* define controller bit for CPR_SDMMCCLKCTL register */
#define CPR_SDMMCCLKCTL_SDMMC_CLK_EN                                9
#define CPR_SDMMCCLKCTL_SDMMC_CLK_DIV                               6
#define CPR_SDMMCCLKCTL_SDMMC_CLK1_DELAY                        3
#define CPR_SDMMCCLKCTL_SDMMC_CLK2_DELAY                        0

/*
 * If the command response transmission does not contain the command index
 * field (long response), the RespCmd fiels is unknown, although it must
 * contain 111111 (the value of the reserved field from the response".
 * This is the case for the SEND_OP_CMD 
 */
#define MMCI_RESP_CMD_FOR_SEND_OP_CMD         63

#define MMC_TOP_CLOCK_SPEED_KHERTZ                 25000  /* 25 MHz in high speed mode */
#define MMC_FAST_CLOCK_DIVIDER                           1
#define MMC_SLOW_CLOCK_DIVIDER                          129  /* for 100 kHz start-up speed */
                  
#define MMC_CMD_MASK                    (3 << 5)            /* non-SPI command type */
#define MMC_CMD_AC                         (0 << 5)
#define MMC_CMD_ADTC                     (1 << 5)
#define MMC_CMD_BC                          (2 << 5)
#define MMC_CMD_BCR                       (3 << 5)

/***************************************************************************
 * MMC defined response Lengths in bytes.  (whole length including headers)
 **************************************************************************/
//定义响应子节的长度
#define MMC_NO_RESPONSE                 0    // 无响应
#define MMC_R1_LENGTH                   6    // 47,Start bit ; 46,Transmission bit; [45:40], Command index ;[39:8], Card status ;[7:1] ,CRC7 ;0 ,End bit
#define MMC_R1B_LENGTH                  6    
#define MMC_R2_LENGTH                   17   //135,Start bit ; 134,Transmission bit; [133:128],Check bits ;[127:1], CID or CSD register incl.internal  ;0 ,End bit
#define MMC_R3_LENGTH                   6    // 47,Start bit ; 46,Transmission bit; [45:40], Check bits ;[39:8], OCR register ;[7:1] ,Check bits ;0 ,End bit
#define MMC_R4_LENGTH                   6    // 47,Start bit ; 46,Transmission bit; [45:40], CMD39 ;[39:8], Argument field ;[7:1] ,CRC7 ;0 ,End bit
#define MMC_R5_LENGTH                   6    // 47,Start bit ; 46,Transmission bit; [45:40], CMD40 ;[39:8], OCR register ;[7:1] ,CRC7 ;0 ,End bit
#define MMC_R7_LENGTH                   6    // 47,Start bit ; 46,Transmission bit; [45:40], Command index ;[39:20], Reserved bits ;[19:16]Voltage accepted [15:8] Echo-back of check pattern[7:1] ,CRC7 ;0 ,End bit

#define SPI_R1_LENGTH                   1    
#define SPI_R3_LENGTH                   5

/***************************************************************************
 * MMC commands and responses.  Format = Command Number, response length (bytes)
 * MMC的命令格式.  Format = 命令索引, 响应长度 (bytes)
 * Use the macros:
 * MMC_CMD_IDX( cMD_iDX, rSP_lEN )
 * MMC_CMD_RSP( cMD_iDX, rSP_lEN )
 **************************************************************************/
#define mmcCmdIdx(cMDiDX, rSPlEN)     (cMDiDX)
#define M_mmcCmdIdx(nUMBER)          mmcCmdIdx(nUMBER)

#define mmcCmdRsp(cMDiDX, rSPlEN)     (rSPlEN)
#define M_mmcCmdRsp(nUMBER)          mmcCmdRsp(nUMBER)

/* 无响应*/
#define MMC_GO_IDLE_STATE           0,    MMC_NO_RESPONSE

/* CMD1  设置及返回卡的OCR*/
#define MMC_SEND_OP_COND            1,    MMC_R3_LENGTH

/* CMD2  要求卡返回ID Asks the card to send its CID number on the CMD line*/
#define MMC_ALL_SEND_CID            2,    MMC_R2_LENGTH

/* CMD3  设置卡的地址*/
#define MMC_SET_RELATIVE_ADDR       3,    MMC_R1_LENGTH

/* CMD4  设置卡的DSR寄存器*/
#define MMC_SET_DSR                 4,    MMC_NO_RESPONSE

/* CMD7 SELECT/DESELECT_CARD  R1 while selecting from Stand-By State to Transfer State; R1b while selecting from Disconnected State to Programming State.*/
#define MMC_SEL_DESEL_CARD          7,    MMC_R1B_LENGTH

/* CMD8 Sends SD Memory Card interface condition, which includes host supply voltage information and asks the card whether card supports voltage.*/
#define MMC_SEND_IF_COND            8,    MMC_R7_LENGTH   

/* CMD9  Addressed card sends its card-specific data (CSD) on the CMD line.*/
#define MMC_SEND_CSD                9,    MMC_R2_LENGTH

/* CMD10 Addressed card sends its card identification (CID) on CMD the line.*/
#define MMC_SEND_CID                10,   MMC_R2_LENGTH

/* CMD11 Reads data stream from the card, starting at the given address, until a STOP_TRANSMISSION follows.*/
#define MMC_READ_DAT_UNTIL_STOP     11,   MMC_R1_LENGTH

/* CMD12 Forces the card to stop transmission. R1 for read cases and R1b for write cases.*/
#define MMC_STOP_TRANSMISSION       12,   MMC_R1_LENGTH

/* CMD13 Addressed card sends its status register. */
#define MMC_SEND_STATUS             13,   MMC_R1_LENGTH

/* CMD15 Sets the card to inactive state*/
#define MMC_GO_INACTIVE_STATE       15,   MMC_NO_RESPONSE

/* CMD16 Sets the block length (in bytes) for all following block commands (read and write). Default block length is specified in the CSD.*/
#define MMC_SET_BLOCKLEN            16,   MMC_R1_LENGTH

/* CMD17 Reads a block of the size selected by the SET_BLOCKLEN command.*/
#define MMC_READ_SINGLE_BLOCK       17,   MMC_R1_LENGTH

/* CMD18 READ_MULTIPLE_BLOCK*/
#define MMC_READ_MULTIPLE_BLOCK     18,   MMC_R1_LENGTH

/* CMD20 Writes a data stream from the host,starting at the given address, until a STOP_TRANSMISSION follows.*/
#define MMC_WRITE_DAT_UNTIL_STOP    20,   MMC_R1_LENGTH

/* CMD23 Defines the number of blocks (read/write) and the reliable writer parameter (write) for a block read or write command.*/
#define MMC_SET_BLOCK_COUNT         23,   MMC_R1_LENGTH

/* CMD24 Writes a block of the size selected by the SET_BLOCKLEN command.*/
#define MMC_WRITE_BLOCK             24,   MMC_R1_LENGTH

/* CMD25 Continuously writes blocks of data until a STOP_TRANSMISSION follows or the requested number of block received.*/
#define MMC_WRITE_MULTIPLE_BLOCK    25,   MMC_R1_LENGTH

/* CMD27 Programming of the programmable bits of the CSD.*/
#define MMC_PROGRAM_CSD             27,   MMC_R1_LENGTH

/* CMD28 If the card has write protection features, this command sets the write protection bit of the addressed group. The properties of write protection are coded in the card specific data*/
#define MMC_SET_WRITE_PROT          28,   MMC_R1B_LENGTH

/* CMD29 If the card provides write protection features, this command clears the write protection bit of the addressed group.*/
#define MMC_CLR_WRITE_PROT          29,   MMC_R1B_LENGTH

/* CMD30 If the card provides write protection features,this command asks the card to send the status of the write protection bits.*/
#define MMC_SEND_WRITE_PROT         30,   MMC_R1_LENGTH

/* CMD35 Sets the address of the first erase group within a range to be selected for erase*/
#define MMC_TAG_ERASE_GROUP_START   35,   MMC_R1_LENGTH

/* CMD36 Sets the address of the last erase group within a continuous range to be selected for erase*/
#define MMC_TAG_ERASE_GROUP_END     36,   MMC_R1_LENGTH

/* CMD38 Erases all previously selected write blocks according to argument bits.*/
#define MMC_ERASE                   38,   MMC_R1B_LENGTH

/* CMD42 Used to set/reset the password or lock/unlock the card.*/
#define MMC_LOCK_UNLOCK             42,   MMC_R1_LENGTH

/* CMD55 Indicates to the card that the next command is an application specific command rather than a standard command*/
#define MMC_APP_CMD                 55,   MMC_R1_LENGTH

/* CMD56 Used either to transfer a data block to the card or to get a data block from the card for general purpose / application specific commands.*/
#define MMC_GEN_CMD                 56,   MMC_R1B_LENGTH


/* these last 2 commands are specific to SPI mode */
#define MMC_READ_OCR                58,   SPI_R3_LENGTH
#define MMC_CRC_ON_OFF              59,   SPI_R1_LENGTH

/* CMD32  sets the address of the first writeblock to be erased.*/
#define SD_ERASE_WR_BLK_START       32,   MMC_R1_LENGTH

/* CMD33  sets the address of the last write block of the continuous range to be erased.*/
#define SD_ERASE_WR_BLK_END         33,   MMC_R1_LENGTH


#define SD_SWITCH_FUNC           0xfe,    MMC_R1_LENGTH			//CMD6 is same as ACMD6, so set CMD6 as 0xfe
/* ACMD6 Defines the data bus width (’00’=1bit or ’10’=4 bits bus) to be used for data transfer. The allowed data bus widths are given in SCR register.*/
#define SD_SET_BUS_WIDTH             6,   MMC_R1_LENGTH

/* ACMD41  设置及返回卡的OCR
#define SD_SEND_OP_COND             41,   MMC_R3_LENGTH

/* ACMD42  Connect[1]/Disconnect[0] the 50KOhm pull-up resistor on CD/DAT3 (pin 1) of the card. The pull-up may be used for card detection.  */
#define SET_CLR_CARD_DETECT         42,   MMC_R1_LENGTH


/* EMMC相对于MMC新增加的命令 CMD8和SD中的CM8重合  用以下来表示*/
#define EMMC_SEND_EXT_CSD           0xfd, MMC_R1_LENGTH

/* CMD8 The card sends its EXT_CSD register as a block of data.*/
#define EMMC_SEND_EXT_CSD_REAL      8,    MMC_R1_LENGTH

/* CMD6  Switches the mode of operation of the selected card or modifies the EXT_CSD registers.*/
#define EMMC_SWITCH		            6,    MMC_R1B_LENGTH


/* 以下是SDIO时用的命令*/
/* CMD5  设置及返回卡的OCR  相当于ACMD41 CMD1*/
#define SD_IO_SEND_OP_COND            5,    MMC_R4_LENGTH

/* CMD52 功能命令*/
#define SD_IO_RW_DIRECT               52,   MMC_R5_LENGTH

/* CMD53 读块操作  相当于CMD17*/
#define SD_IO_RW_EXTENDED             53,   MMC_R5_LENGTH


/* 卡的状态的标志*/
/** FTL Device bit-map flag : Device is not present */
#define FTL_NOT_PRESENT         0x00000000

/** FTL Device bit-map flag : Device is present */
#define FTL_IS_PRESENT          0x00000001

/** FTL Device bit-map flag : Device is is an SD card */
#define FTL_IS_SD_CARD          0x00000002

/** FTL Device bit-map flag : Device is an MMC card */
#define FTL_IS_MMC_CARD         0x00000004

/** FTL Device bit-map flag : Device is write protected */
#define FTL_IS_WRITE_PROTECTED  0x00000008

/** FTL Device bit-map flag : Device is removable */
#define FTL_IS_REMOVABLE        0x00000010

/** FTL Device bit-map flag : Device initialisation failed  */
#define FTL_INIT_FAILED         0x00000020

/** FTL Device bit-map flag : Device is an HCSD card  */
#define FTL_IS_HCSD_CARD         0x00000040

/** FTL Device bit-map flag : Device is an HCSD card  */
#define FTL_IS_EMMC_CARD         0x00000080


/** FTL Device bit-map flag : Device requires regular timeout polling */
#define FTL_TIMEOUT_POLLING     0x00000100

/* Private flags that are not reported to the upper layers */
/** FTL Device bit-map flag : (Internal use only) Device is asleep */
#define FTL_IS_INACTIVE         0x00010000

/** FTL Device bit-map flag : Mask to hide internal-only flags */
#define FTL_PRIVATE_FLAGS_MASK  0xFF00FFFF

/** FTL Device bit-map flag : Device Error */
#define FTL_ERROR               0x80000000


/* Card Status -- from Table 16 MMC spec version 3.1 */
#define CARD_STATUS_OUT_OF_RANGE          0x80000000 /* argument out of range for card */
#define CARD_STATUS_ADDRESS_ERROR         0x40000000 /* address not aligned with block length */
#define CARD_STATUS_BLOCK_LEN_ERROR       0x20000000 /* block length not allowed or number of transferred bytes doesn't match block length */
#define CARD_STATUS_ERASE_SEQ_ERROR       0x10000000 /* An error in the sequence of erase commmands has occurred */
#define CARD_STATUS_ERASE_PARAM           0x08000000 /* invalid selection of erase groups */
#define CARD_STATUS_WP_VIOLATION          0x04000000 /* Attempt to program write protected block */
#define CARD_STATUS_CARD_IS_LOCKED        0x02000000 /* card is locked by the host */
#define CARD_STATUS_LOCK_UNLOCK_FAILED    0x01000000 /* either an error in the unlocking process or an attempt to access a locked card */
#define CARD_STATUS_COM_CRC_ERROR         0x00800000
#define CARD_STATUS_ILLEGAL_COMMAND       0x00400000 /* command not legal for the card state */
#define CARD_STATUS_CARD_ECC_FAILED       0x00200000 /* internal card ECC was applied but failed to fix the error */
#define CARD_STATUS_CC_ERROR              0x00100000 /* Internal Card Controller error */
#define CARD_STATUS_ERROR                 0x00080000
#define CARD_STATUS_UNDERRUN              0x00040000 /* card could not sustain data transfer in stream read mode */
#define CARD_STATUS_OVERRUN               0x00020000 /* card could not sustain data programming in stream write mode */
#define CARD_STATUS_CID_CSD_OVERWRITE     0x00010000
#define CARD_STATUS_WP_ERASE_SKIP         0x00008000 /* only partial address space was erased due to existing write protected blocks */
#define CARD_STATUS_CARD_ECC_DISABLED     0x00004000 /* the command was executed without using the internal ECC */
#define CARD_STATUS_ERASE_RESET           0x00002000 /* erase sequence was cleared because an out of erase sequence command was rx'd */
#define CARD_STATUS_CURRENT_STATE_MASK    0x00001e00 //*CURRENT_STATE
#define CARD_STATUS_READ_FOR_DATA         0x00000100
#define CARD_STATUS_APP_CMD               0x00000020
#define CARD_STATUS_CURRENT_STATE_IDLE    0
#define CARD_STATUS_CURRENT_STATE_READY   1
#define CARD_STATUS_CURRENT_STATE_IDENT   2
#define CARD_STATUS_CURRENT_STATE_STDBY   3
#define CARD_STATUS_CURRENT_STATE_TRAN    4
#define CARD_STATUS_CURRENT_STATE_DATA    5
#define CARD_STATUS_CURRENT_STATE_RCV     6
#define CARD_STATUS_CURRENT_STATE_PRG     7
#define CARD_STATUS_CURRENT_STATE_DIS     8
#define CARD_STATUS_CURRENT_STATE_BIT_POS 9
/* Current States */


/* 以下是OCR寄存器 的电压设定范围值*/ /* OCR */
#define OCR_CARD_POWER_UP_BUSY        0x80000000
#define MMC_VDD_165_195     0x00000080  /* VDD voltage 1.65 - 1.95 */ //只有emmc
#define MMC_VDD_20_21       0x00000100  /* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22       0x00000200  /* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23       0x00000400  /* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24       0x00000800  /* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25       0x00001000  /* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26       0x00002000  /* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27       0x00004000  /* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28       0x00008000  /* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29       0x00010000  /* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30       0x00020000  /* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31       0x00040000  /* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32       0x00080000  /* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33       0x00100000  /* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34       0x00200000  /* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35       0x00400000  /* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36       0x00800000  /* VDD voltage 3.5 ~ 3.6 */
#define OCR_AVAIL                0x00010000

#define MMC_CAP_4_BIT_DATA      (1 << 0)    /* Can the host do 4 bit transfers */
#define MMC_CAP_MMC_HIGHSPEED   (1 << 1)    /* Can do MMC high-speed timing */
#define MMC_CAP_SD_HIGHSPEED    (1 << 2)    /* Can do SD high-speed timing */
#define MMC_CAP_SDIO_IRQ        (1 << 3)    /* Can signal pending SDIO IRQs */
#define MMC_CAP_SPI             (1 << 4)    /* Talks only SPI protocols */
#define MMC_CAP_NEEDS_POLL      (1 << 5)    /* Needs polling for card-detection */
#define MMC_CAP_8_BIT_DATA      (1 << 6)    /* Can the host do 8 bit transfers */
#define MMC_CAP_DISABLE         (1 << 7)    /* Can the host be disabled */
#define MMC_CAP_NONREMOVABLE    (1 << 8)    /* Nonremovable e.g. eMMC */
#define MMC_CAP_WAIT_WHILE_BUSY (1 << 9)    /* Waits while card is busy */



/* SDIO 协议中的 CMD52 Response*/
#define R5_COM_CRC_ERROR        (1 << 15)   /* er, b */
#define R5_ILLEGAL_COMMAND      (1 << 14)   /* er, b */
#define R5_ERROR                (1 << 11)   /* erx, c */
#define R5_FUNCTION_NUMBER      (1 << 9)    /* er, c */
#define R5_OUT_OF_RANGE         (1 << 8)    /* er, c */
#define R5_STATUS(x)            (x & 0xCB00)
#define R5_IO_CURRENT_STATE(x)  ((x & 0x3000) >> 12) /* s, b */

/* CCCR寄存器的变量*/
#define SDIO_CCCR_CCCR          0x00
#define SDIO_CCCR_REV_1_00      0   /* CCCR/FBR Version 1.00 */
#define SDIO_CCCR_REV_1_10      1   /* CCCR/FBR Version 1.10 */
#define SDIO_CCCR_REV_1_20      2   /* CCCR/FBR Version 1.20 */
#define SDIO_SDIO_REV_1_00      0   /* SDIO Spec Version 1.00 */
#define SDIO_SDIO_REV_1_10      1   /* SDIO Spec Version 1.10 */
#define SDIO_SDIO_REV_1_20      2   /* SDIO Spec Version 1.20 */
#define SDIO_SDIO_REV_2_00      3   /* SDIO Spec Version 2.00 */
#define SDIO_CCCR_SD            0x01
#define SDIO_SD_REV_1_01        0   /* SD Physical Spec Version 1.01 */
#define SDIO_SD_REV_1_10        1   /* SD Physical Spec Version 1.10 */
#define SDIO_SD_REV_2_00        2   /* SD Physical Spec Version 2.00 */
#define SDIO_CCCR_IOEx          0x02
#define SDIO_CCCR_IORx          0x03
#define SDIO_CCCR_IENx          0x04    /* Function/Master Interrupt Enable */
#define SDIO_CCCR_INTx          0x05    /* Function Interrupt Pending */
#define SDIO_CCCR_ABORT         0x06    /* function abort/card reset */
#define SDIO_CCCR_IF            0x07    /* bus interface controls */
#define SDIO_BUS_WIDTH_1BIT     0x00
#define SDIO_BUS_WIDTH_4BIT     0x02
#define SDIO_BUS_CD_DISABLE     0x80   /* disable pull-up on DAT3 (pin 1) */
#define SDIO_CCCR_CAPS          0x08
#define SDIO_CCCR_CAP_SDC       0x01    /* can do CMD52 while data transfer */
#define SDIO_CCCR_CAP_SMB       0x02    /* can do multi-block xfers (CMD53) */
#define SDIO_CCCR_CAP_SRW       0x04    /* supports read-wait protocol */
#define SDIO_CCCR_CAP_SBS       0x08    /* supports suspend/resume */
#define SDIO_CCCR_CAP_S4MI      0x10    /* interrupt during 4-bit CMD53 */
#define SDIO_CCCR_CAP_E4MI      0x20    /* enable ints during 4-bit CMD53 */
#define SDIO_CCCR_CAP_LSC       0x40    /* low speed card */
#define SDIO_CCCR_CAP_4BLS      0x80    /* 4 bit low speed card */
#define SDIO_CCCR_CIS           0x09    /* common CIS pointer (3 bytes) */

/* Following 4 regs are valid only if SBS is set */
#define SDIO_CCCR_SUSPEND       0x0c
#define SDIO_CCCR_SELx          0x0d
#define SDIO_CCCR_EXECx         0x0e
#define SDIO_CCCR_READYx        0x0f
#define SDIO_CCCR_BLKSIZE       0x10
#define SDIO_CCCR_POWER         0x12
#define SDIO_POWER_SMPC         0x01    /* Supports Master Power Control */
#define SDIO_POWER_EMPC         0x02    /* Enable Master Power Control */
#define SDIO_CCCR_SPEED         0x13
#define SDIO_SPEED_SHS          0x01    /* Supports High-Speed mode */
#define SDIO_SPEED_EHS          0x02    /* Enable High-Speed mode */

/*Function Basic Registers (FBR) */
#define SDIO_FBR_BASE(f)        ((f) * 0x100) /* base of function f's FBRs */
#define SDIO_FBR_STD_IF         0x00
#define SDIO_FBR_SUPPORTS_CSA   0x40    /* supports Code Storage Area */
#define SDIO_FBR_ENABLE_CSA     0x80    /* enable Code Storage Area */
#define SDIO_FBR_STD_IF_EXT     0x01
#define SDIO_FBR_POWER          0x02
#define SDIO_FBR_POWER_SPS      0x01    /* Supports Power Selection */
#define SDIO_FBR_POWER_EPS      0x02    /* Enable (low) Power Selection */
#define SDIO_FBR_CIS            0x09    /* CIS pointer (3 bytes) */
#define SDIO_FBR_CSA            0x0C    /* CSA pointer (3 bytes) */
#define SDIO_FBR_CSA_DATA       0x0F
#define SDIO_FBR_BLKSIZE        0x10    /* block size (2 bytes) */

typedef enum DlMmcCmdResultTag
{
    MMC_CMD_OK = 0,
    MMC_CDT_INTR,
    MMC_RE_INTR,
    MMC_CMD_RESP_END,       /* MMC_CD_INTR,*/
    MMC_DTO_INTR,
    MMC_TXDR_INTR,
    MMC_RXDR_INTR,
    MMC_RCRC_INTR,
    MMC_DCRC_INTR,
    MMC_RTO_INTR,
    MMC_DRTO_INTR,
    MMC_HTO_INTR,
    MMC_FRUN_INTR,
    MMC_HLE_INTR,
    MMC_SBE_INTR,
    MMC_ACD_INTR,
    MMC_CRCERR_INTR,
    MMC_FAIL,
    MMC_CARD_REMOVED,
    MMC_RESERVED_ENUM = MMC_CDT_INTR,
}DlMmcCmdResult;

#endif

