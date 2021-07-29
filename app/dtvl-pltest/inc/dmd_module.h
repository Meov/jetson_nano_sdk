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
*ËµÃ÷£º
*
**********************************************************************/
#ifndef __DMD_MODULE_H__
#define __DMD_MODULE_H__
/*******************************************************************************
Base addresses for standard memory-mapped peripherals for LC1860
********************************************************************************/
#if 0
#define DDR0_MEMORY_BASE                                       0x00000000
#define DDR1_MEMORY_BASE                                       0x40000000
#define AP_SW0_GPV_BASE                                        0xA0A00000
#define CCI_AXI_GPV_BASE                                       0xA0000000
#define GPU_BASE                                               0xA0100000
#define CTL_BASE                                               0xA0110000
#define AP_A7_GIC_BASE                                         0xA0118000
#define AP_PERI_SW0_GPV_BASE                                   0xA0200000
#define X2DACC_BASE                                            0xA0300000
#define CODEC_BASE                                             0xA0340000
#define ENCODER1_BASE                                          0xA0342000
#define SMMU0_BASE                                             0xA0350000
#define USB_OTG_BASE                                           0xA0400000
#define USB_HSIC_BASE                                          0xA0440000
#define USB_CTL_BASE                                           0xA0480000
#define ISP_BASE                                               0xA0500000
#define SMMU1_BASE                                             0xA0580000
#define LCDC0_BASE                                             0xA0590000
#define LCDC1_BASE                                             0xA0591000
#define MIPI_BASE                                              0xA05A0000
#define DSI_ISP_BASE                                           0xA05B0000
#define AP_DMAG_BASE                                           0xA0600000
#define AP_DMAS_BASE                                           0xA0608000
#define AP_DMAC_BASE                                           0xA060C000
#define AES_BASE                                               0xA0610000
#define SHA_BASE                                               0xA0610800
#define NFC_BASE                                               0xA0620000
#define SDMMC1_BASE                                            0xA0630000
#define AP_PERI_SW4_GPV_BASE                                   0xA0700000
#define ROM_BASE                                               0xFFFF0000
#define SEC_RAM_BASE                                           0xA0800000
#define SECURITY_BASE                                          0xA0820000
#define HPI_BASE                                               0xA0830000
#define MEM_REGION_BASE                                        0xA08C0000
#define SDMMC0_BASE                                            0xA0860000
#define SDMMC2_BASE                                            0xA0861000
#define SSI0_BASE                                              0xA0880000
#define SSI1_BASE                                              0xA0881000
#define UART0_BASE                                             0xA0882000
#define UART1_BASE                                             0xA0883000
#define UART2_BASE                                             0xA0884000
#define AP_I2S_BASE                                            0xA0885000
#define AP_WDT0_BASE                                           0xA0890000
#define AP_WDT1_BASE                                           0xA0891000
#define AP_WDT2_BASE                                           0xA0892000
#define AP_WDT3_BASE                                           0xA0893000
#define AP_TIMER_BASE                                          0xA0894000
#define I2C0_BASE                                              0xA0895000
#define I2C1_BASE                                              0xA0896000
#define I2C3_BASE                                              0xA0897000
#define PWM_BASE                                               0xA0898000
#define AP_PWR_BASE                                            0xA0899000
#define AP_WDT4_BASE                                           0xA089A000
#define I2C2_BASE                                              0xA08A0000
#define KBS_BASE                                               0xA08A1000
#define SSI2_BASE                                              0xA08A2000
#define BP147_BASE                                             0xA08A3000
#define TPZCTL_BASE                                            0xA08A7000
#define AP_PERI_SW5_GPV_BASE                                   0xA0900000
#define TOP_RAM_BASE                                           0xE0040000
#define TL420_RAM_BASE                                         0xE0400000
#define CP_BOOT_RAM_BASE                                       0xEFFF0000
#define TOP_MAILBOX_BASE                                       0xE1000000
//#define MEMCTL0_BASE                                           0xE1001000
//#define MEMCTL1_BASE                                           0xE1002000
#define TOP_DMAS_BASE                                          0xE1003000
#define TOP_DMAG_BASE                                          0xE1004000
#define COM_I2C_BASE                                           0xE1006000
#define COM_PCM_BASE                                           0xE1007000
#define MUXPIN_BASE                                            0xE1009000
#define DDR_PWR_BASE                                           0xE100A000
#define COM_UART_BASE                                          0xE100B000
#define GPIO_BASE                                              0xE100C000
#define COM_I2S_BASE                                           0xE100E000
#define TL420_ICTL_BASE                                        0xE1010000
#define DEBUG_APB_BASE                                         0xE1080000
#define TOP_CTRL_GPV_BASE                                      0xE1100000
#define CP_A7_GIC_BASE                                         0x80020000
#define CP_SHRAM0_BASE                                         0x81000000
#define CP_SHRAM1_BASE                                         0x81100000
#define XC4210_RAM_BASE                                        0x82000000
#define X1643_RAM_BASE                                         0x82800000
#define CP_GPV_BASE                                            0x83000000
#define CP_MAILBOX_BASE                                        0x84000000
#define XC4210_ICTL_BASE                                       0x84100000
#define X1643_ICTL_BASE                                        0x84200000
#define LTEU_BASE                                              0x85000000
#define LTET_BASE                                              0x85800000
#define RFIF_BASE                                              0x85900000
#define DIGRFV4_BASE                                           0x85A00000
#define RF_DMAD_BASE                                           0x85B00000
#define THU_BASE                                               0x86000000
#define CP_DMAD_BASE                                           0x86200000
#define CP_DMAG_BASE                                           0x86300000
#define IPHWA_BASE                                             0x86400000
#define HSL_BASE                                               0x86500000
#define A5_BASE                                                0x86700000
#define GEA_BASE                                               0x86800000
#define CIPHERHWA_BASE                                         0x86900000
#define XCDMA_BASE                                             0x86A00000
#define SIM0_BASE                                              0x87000000
#define SIM1_BASE                                              0x87010000
#define CP_TIMER_BASE                                          0x87020000
#define CP_WDT_BASE                                            0x87030000
#define CP_PWR_BASE                                            0x87040000
#define RTC_BASE                                               0x87050000
#define ISP_BASE                                               0xA0500000
#else
#define DDR0_MEMORY_BASE                                       0
#define DDR1_MEMORY_BASE                                       0
#define AP_SW0_GPV_BASE                                        0
#define CCI_AXI_GPV_BASE                                       0
#define GPU_BASE                                               0
#define CTL_BASE                                               0
#define AP_A7_GIC_BASE                                         0
#define AP_PERI_SW0_GPV_BASE                                   0
#define X2DACC_BASE                                            0
#define CODEC_BASE                                             0
#define ENCODER1_BASE                                          0
#define SMMU0_BASE                                             0
#define USB_OTG_BASE                                           0
#define USB_HSIC_BASE                                          0
#define USB_CTL_BASE                                           0
#define ISP_BASE                                               0
#define SMMU1_BASE                                             0
#define LCDC0_BASE                                             0
#define LCDC1_BASE                                             0
#define MIPI_BASE                                              0
#define DSI_ISP_BASE                                           0
#define AP_DMAG_BASE                                           0
#define AP_DMAS_BASE                                           0
#define AP_DMAC_BASE                                           0
#define AES_BASE                                               0
#define SHA_BASE                                               0
#define NFC_BASE                                               0
#define SDMMC1_BASE                                            0
#define AP_PERI_SW4_GPV_BASE                                   0
#define ROM_BASE                                               0
#define SEC_RAM_BASE                                           0
#define SECURITY_BASE                                          0
#define HPI_BASE                                               0
#define MEM_REGION_BASE                                        0
#define SDMMC0_BASE                                            0
#define SDMMC2_BASE                                            0
#define SSI0_BASE                                              0
#define SSI1_BASE                                              0
#define UART0_BASE                                             0
#define UART1_BASE                                             0
#define UART2_BASE                                             0
#define AP_I2S_BASE                                            0
#define AP_WDT0_BASE                                           0
#define AP_WDT1_BASE                                           0
#define AP_WDT2_BASE                                           0
#define AP_WDT3_BASE                                           0
#define AP_TIMER_BASE                                          0
#define I2C0_BASE                                              0
#define I2C1_BASE                                              0
#define I2C3_BASE                                              0
#define PWM_BASE                                               0
#define AP_PWR_BASE                                            0
#define AP_WDT4_BASE                                           0
#define I2C2_BASE                                              0
#define KBS_BASE                                               0
#define SSI2_BASE                                              0
#define BP147_BASE                                             0
#define TPZCTL_BASE                                            0
#define AP_PERI_SW5_GPV_BASE                                   0
#define TOP_RAM_BASE                                           0
#define TL420_RAM_BASE                                         0
#define CP_BOOT_RAM_BASE                                       0
#define TOP_MAILBOX_BASE                                       0
//#define MEMCTL0_BASE                                           0
//#define MEMCTL1_BASE                                           0
#define TOP_DMAS_BASE                                          0
#define TOP_DMAG_BASE                                          0
#define COM_I2C_BASE                                           0
#define COM_PCM_BASE                                           0
#define MUXPIN_BASE                                            0
#define DDR_PWR_BASE                                           0
#define COM_UART_BASE                                          0
#define GPIO_BASE                                              0
#define COM_I2S_BASE                                           0
#define TL420_ICTL_BASE                                        0
#define DEBUG_APB_BASE                                         0
#define TOP_CTRL_GPV_BASE                                      0
#define CP_A7_GIC_BASE                                         0
#define CP_SHRAM0_BASE                                         0
#define CP_SHRAM1_BASE                                         0
#define XC4210_RAM_BASE                                        0
#define X1643_RAM_BASE                                         0
#define CP_GPV_BASE                                            0
#define CP_MAILBOX_BASE                                        0
#define XC4210_ICTL_BASE                                       0
#define X1643_ICTL_BASE                                        0
#define LTEU_BASE                                              0
#define LTET_BASE                                              0
#define RFIF_BASE                                              0
#define DIGRFV4_BASE                                           0
#define RF_DMAD_BASE                                           0
#define THU_BASE                                               0
#define CP_DMAD_BASE                                           0
#define CP_DMAG_BASE                                           0
#define IPHWA_BASE                                             0
#define HSL_BASE                                               0
#define A5_BASE                                                0
#define GEA_BASE                                               0
#define CIPHERHWA_BASE                                         0
#define XCDMA_BASE                                             0
#define SIM0_BASE                                              0
#define SIM1_BASE                                              0
#define CP_TIMER_BASE                                          0
#define CP_WDT_BASE                                            0
#define CP_PWR_BASE                                            0
#define RTC_BASE                                               0
#define ISP_BASE                                               0
#endif

/*******************************************************************************
CTL registers' address on LC1860
********************************************************************************/
/*AP²àA7¿ØÖÆ¼°Æô¶¯µØÖ·¼Ä´æÆ÷*/
#define CTL_AP_HA7_CORE0_WBOOT_ADDR                       CTL_BASE + 0x0000
#define CTL_GEN_REG0                                      CTL_BASE + 0x0004
#define CTL_AP_HA7_CORE1_WBOOT_ADDR                       CTL_BASE + 0x0008
#define CTL_GEN_REG1                                      CTL_BASE + 0x000C
#define CTL_AP_HA7_CORE2_WBOOT_ADDR                       CTL_BASE + 0x0010
#define CTL_GEN_REG2                                      CTL_BASE + 0x0014
#define CTL_AP_HA7_CORE3_WBOOT_ADDR                       CTL_BASE + 0x0018
#define CTL_AP_SA7_CORE0_WBOOT_ADDR                       CTL_BASE + 0x001C
#define CTL_AP_SA7_CTRL                                   CTL_BASE + 0x00E4
#define CTL_AP_HA7_CTRL                                   CTL_BASE + 0x00E8
/*CTLµÍ¹¦ºÄ¿ØÖÆ¼Ä´æÆ÷*/
#define CTL_LP_MODE_CTRL                                  CTL_BASE + 0x00A0
/*SSIÐ­Òé¿ØÖÆ¼Ä´æÆ÷*/
#define CTL_SSI0_PROTOCOL_CTRL                            CTL_BASE + 0x00B0
#define CTL_SSI1_PROTOCOL_CTRL                            CTL_BASE + 0x00B4
#define CTL_SSI2_PROTOCOL_CTRL                            CTL_BASE + 0x00B8
/*TIMER¿ØÖÆ¼Ä´æÆ÷*/
#define CTL_TIMER1_PAUSE_CTRL                             CTL_BASE + 0x00BC
#define CTL_TIMER2_PAUSE_CTRL                             CTL_BASE + 0x00C0
#define CTL_TIMER3_PAUSE_CTRL                             CTL_BASE + 0x00C4
#define CTL_TIMER4_PAUSE_CTRL                             CTL_BASE + 0x00C8
#define CTL_TIMER5_PAUSE_CTRL                             CTL_BASE + 0x00CC
#define CTL_TIMER6_PAUSE_CTRL                             CTL_BASE + 0x00D0
#define CTL_TIMER7_PAUSE_CTRL                             CTL_BASE + 0x00D4
#define CTL_TIMER8_PAUSE_CTRL                             CTL_BASE + 0x00D8
/*GPU¿ØÖÆ¼Ä´æÆ÷*/
#define CTL_GPU_CTRL                                      CTL_BASE + 0x00EC
/*ON2¿ØÖÆ¼Ä´æÆ÷*/
#define CTL_ON2_RAM_CLKGATE_CTRL                          CTL_BASE + 0x0120
/*×ÜÏß¿ØÖÆ¼Ä´æÆ÷*/
#define CTL_BUS_CFG                                       CTL_BASE + 0x0124

/*******************************************************************************
USB_CTL registers' address on LC1860
********************************************************************************/
/*USBºÍHSIC PHY¿ØÖÆ¼Ä´æÆ÷*/
#define USB_CTL_POR_OTGPHY_CTRL                           USB_CTL_BASE + 0x0020
#define USB_CTL_PORTRESET_OTGPHY_CTRL                     USB_CTL_BASE + 0x0024
#define USB_CTL_OTGPHY_SUSPENDM_CTRL                      USB_CTL_BASE + 0x0028
#define USB_CTL_OTGPHY_CTRL                               USB_CTL_BASE + 0x002C
#define USB_CTL_OTGPHY_TEST_CTRL0                         USB_CTL_BASE + 0x0030
#define USB_CTL_OTGPHY_TEST_CTRL1                         USB_CTL_BASE + 0x0034
#define USB_CTL_OTGPHY_TEST_CTRL2                         USB_CTL_BASE + 0x0038
#define USB_CTL_OTGPHY_CHARGE_CTRL                        USB_CTL_BASE + 0x003C
#define USB_CTL_OTGPHY_PARAM_OVERRIDE                     USB_CTL_BASE + 0x0040
#define USB_CTL_OTG_CORE_CTRL                             USB_CTL_BASE + 0x0044
#define USB_CTL_OTG_PHY_STATUS                            USB_CTL_BASE + 0x0048
#define USB_CTL_OTGPHY_INTR_RAW                           USB_CTL_BASE + 0x004C
#define USB_CTL_OTGPHY_INTR_STA                           USB_CTL_BASE + 0x0050
#define USB_CTL_OTGPHY_INTR_EN                            USB_CTL_BASE + 0x0054
#define USB_CTL_HSIC_PHY_POR_CTRL                         USB_CTL_BASE + 0x0060
#define USB_CTL_HSICPHY_PORTRESET_CTRL                    USB_CTL_BASE + 0x0064
#define USB_CTL_HSIC_PHY_SUSPEND_CTRL                     USB_CTL_BASE + 0x0068
#define USB_CTL_HSIC_SCALEDOWN_MODE_CTRL                  USB_CTL_BASE + 0x006C
#define USB_CTL_HSIC_PHY_STATUS                           USB_CTL_BASE + 0x0070
#define USB_CTL_HSICPHY_CTRL                              USB_CTL_BASE + 0x0074
#define USB_CTL_HSICPHY_TEST_CTRL0                        USB_CTL_BASE + 0x0078
#define USB_CTL_HSICPHY_TEST_CTRL1                        USB_CTL_BASE + 0x007C
#define USB_CTL_HSICPHY_TEST_CTRL2                        USB_CTL_BASE + 0x0080
#define USB_CTL_HSICPHY_PARAM_OVERRIDE                    USB_CTL_BASE + 0x0084
#define USB_CTL_HSICPHY_INTR_RAW                          USB_CTL_BASE + 0x0088
#define USB_CTL_HSICPHY_INTR_STA                          USB_CTL_BASE + 0x008C
#define USB_CTL_HSICPHY_INTR_EN                           USB_CTL_BASE + 0x0090
#define USB_CTL_OTG_CORE_RST_CTRL                         USB_CTL_BASE + 0x0094
#define USB_CTL_HSIC_CORE_RST_CTRL                        USB_CTL_BASE + 0x0098
#define USB_CTL_OTGPHY_SUSPEND_STATUS                     USB_CTL_BASE + 0x00DC
#define USB_CTL_HSICPHY_SUSPEND_STATUS                    USB_CTL_BASE + 0x00E0
/*USB_CTLµÍ¹¦ºÄ¿ØÖÆ¼Ä´æÆ÷*/
#define USB_CTL_LP_MODE_CTRL                              USB_CTL_BASE + 0x00A0

/*******************************************************************************
LCDC0 registers' address on LC1860
********************************************************************************/
/*LCDC¿ØÖÆÆ÷¼Ä´æÆ÷µØÖ·Ó³Éä*/
#define LCDC0_MOD                                         LCDC0_BASE + 0x0000
#define LCDC0_INTC                                        LCDC0_BASE + 0x0014
#define LCDC0_INTF                                        LCDC0_BASE + 0x0018
#define LCDC0_INTE                                        LCDC0_BASE + 0x001C
#define LCDC0_TIM1000US                                   LCDC0_BASE + 0x0028
#define LCDC0_HV_CTL                                      LCDC0_BASE + 0x0030
#define LCDC0_HV_P                                        LCDC0_BASE + 0x0034
#define LCDC0_HV_HCTL                                     LCDC0_BASE + 0x003C
#define LCDC0_HV_VCTL0                                    LCDC0_BASE + 0x0040
#define LCDC0_HV_HS                                       LCDC0_BASE + 0x0048
#define LCDC0_HV_VS                                       LCDC0_BASE + 0x004C
#define LCDC0_HV_F                                        LCDC0_BASE + 0x0050
#define LCDC0_HV_VINT                                     LCDC0_BASE + 0x0054
#define LCDC0_HV_DM                                       LCDC0_BASE + 0x0058
#define LCDC0_HV_CNTNOW                                   LCDC0_BASE + 0x005C
#define LCDC0_MIPI_DSI_AUTORESETTIME                      LCDC0_BASE + 0x0060
#define LCDC0_MIPI_DSI_AUTORESETCTL                       LCDC0_BASE + 0x0064
#define LCDC0_MWIN_SIZE                                   LCDC0_BASE + 0x0074
#define LCDC0_MWIN_BG                                     LCDC0_BASE + 0x0078
#define LCDC0_DISP_CTRL                                   LCDC0_BASE + 0x007C
#define LCDC0_L0_WIN_CTRL                                 LCDC0_BASE + 0x0094
#define LCDC0_L0_SA0                                      LCDC0_BASE + 0x0098
#define LCDC0_L0_KEY_COLOR                                LCDC0_BASE + 0x009C
#define LCDC0_L0_WIN_SIZE                                 LCDC0_BASE + 0x00A0
#define LCDC0_L0_WIN_OFS                                  LCDC0_BASE + 0x00A4
#define LCDC0_L0_SRC_WID                                  LCDC0_BASE + 0x00A8
#define LCDC0_L1_WIN_CTRL                                 LCDC0_BASE + 0x00AC
#define LCDC0_L1_SA0                                      LCDC0_BASE + 0x00B0
#define LCDC0_L1_KEY_COLOR                                LCDC0_BASE + 0x00B4
#define LCDC0_L1_WIN_SIZE                                 LCDC0_BASE + 0x00B8
#define LCDC0_L1_WIN_OFS                                  LCDC0_BASE + 0x00BC
#define LCDC0_L1_SRC_WID                                  LCDC0_BASE + 0x00C0
#define LCDC0_DMA_STA                                     LCDC0_BASE + 0x0128
#define LCDC0_FIFO_STATUS                                 LCDC0_BASE + 0x0148
#define LCDC0_RF_PER                                      LCDC0_BASE + 0x0154
#define LCDC0_L0_SA1                                      LCDC0_BASE + 0x0160
#define LCDC0_L1_SA1                                      LCDC0_BASE + 0x0164
#define LCDC0_LP_CTRL                                     LCDC0_BASE + 0x01C0
#define LCDC0_EDPI_IF_MODE                                LCDC0_BASE + 0x01D0
#define LCDC0_MIPI_DSI_CTRL                               LCDC0_BASE + 0x01D8
#define LCDC0_AXIDMA_PRIOR                                LCDC0_BASE + 0x01F0
#define LCDC0_DMA_CTRL                                    LCDC0_BASE + 0x01F4
#define LCDC0_DMA_READ_PERIOD                             LCDC0_BASE + 0x01F8

/*******************************************************************************
LCDC1 registers' address on LC1860
********************************************************************************/
/*LCDC¿ØÖÆÆ÷¼Ä´æÆ÷µØÖ·Ó³Éä*/
#define LCDC1_MOD                                         LCDC1_BASE + 0x0000
#define LCDC1_INTC                                        LCDC1_BASE + 0x0014
#define LCDC1_INTF                                        LCDC1_BASE + 0x0018
#define LCDC1_INTE                                        LCDC1_BASE + 0x001C
#define LCDC1_TIM1000US                                   LCDC1_BASE + 0x0028
#define LCDC1_HV_CTL                                      LCDC1_BASE + 0x0030
#define LCDC1_HV_P                                        LCDC1_BASE + 0x0034
#define LCDC1_HV_HCTL                                     LCDC1_BASE + 0x003C
#define LCDC1_HV_VCTL0                                    LCDC1_BASE + 0x0040
#define LCDC1_HV_HS                                       LCDC1_BASE + 0x0048
#define LCDC1_HV_VS                                       LCDC1_BASE + 0x004C
#define LCDC1_HV_F                                        LCDC1_BASE + 0x0050
#define LCDC1_HV_VINT                                     LCDC1_BASE + 0x0054
#define LCDC1_HV_DM                                       LCDC1_BASE + 0x0058
#define LCDC1_HV_CNTNOW                                   LCDC1_BASE + 0x005C
#define LCDC1_MIPI_DSI_AUTORESETTIME                      LCDC1_BASE + 0x0060
#define LCDC1_MIPI_DSI_AUTORESETCTL                       LCDC1_BASE + 0x0064
#define LCDC1_MWIN_SIZE                                   LCDC1_BASE + 0x0074
#define LCDC1_MWIN_BG                                     LCDC1_BASE + 0x0078
#define LCDC1_DISP_CTRL                                   LCDC1_BASE + 0x007C
#define LCDC1_L0_WIN_CTRL                                 LCDC1_BASE + 0x0094
#define LCDC1_L0_SA0                                      LCDC1_BASE + 0x0098
#define LCDC1_L0_KEY_COLOR                                LCDC1_BASE + 0x009C
#define LCDC1_L0_WIN_SIZE                                 LCDC1_BASE + 0x00A0
#define LCDC1_L0_WIN_OFS                                  LCDC1_BASE + 0x00A4
#define LCDC1_L0_SRC_WID                                  LCDC1_BASE + 0x00A8
#define LCDC1_L1_WIN_CTRL                                 LCDC1_BASE + 0x00AC
#define LCDC1_L1_SA0                                      LCDC1_BASE + 0x00B0
#define LCDC1_L1_KEY_COLOR                                LCDC1_BASE + 0x00B4
#define LCDC1_L1_WIN_SIZE                                 LCDC1_BASE + 0x00B8
#define LCDC1_L1_WIN_OFS                                  LCDC1_BASE + 0x00BC
#define LCDC1_L1_SRC_WID                                  LCDC1_BASE + 0x00C0
#define LCDC1_DMA_STA                                     LCDC1_BASE + 0x0128
#define LCDC1_FIFO_STATUS                                 LCDC1_BASE + 0x0148
#define LCDC1_RF_PER                                      LCDC1_BASE + 0x0154
#define LCDC1_L0_SA1                                      LCDC1_BASE + 0x0160
#define LCDC1_L1_SA1                                      LCDC1_BASE + 0x0164
#define LCDC1_LP_CTRL                                     LCDC1_BASE + 0x01C0
#define LCDC1_EDPI_IF_MODE                                LCDC1_BASE + 0x01D0
#define LCDC1_MIPI_DSI_CTRL                               LCDC1_BASE + 0x01D8
#define LCDC1_AXIDMA_PRIOR                                LCDC1_BASE + 0x01F0
#define LCDC1_DMA_CTRL                                    LCDC1_BASE + 0x01F4
#define LCDC1_DMA_READ_PERIOD                             LCDC1_BASE + 0x01F8

/*******************************************************************************
MIPI registers' address on LC1860
********************************************************************************/
/*mipi¿ØÖÆÆ÷¼Ä´æÆ÷µØÖ·Ó³Éä*/
#define MIPI_VERSION                                      MIPI_BASE + 0x0000
#define MIPI_PWR_UP                                       MIPI_BASE + 0x0004
#define MIPI_CLKMGR_CFG                                   MIPI_BASE + 0x0008
#define MIPI_DPI_VCID                                     MIPI_BASE + 0x000C
#define MIPI_DPI_COLOR_CODING                             MIPI_BASE + 0x0010
#define MIPI_DPI_CFG_POL                                  MIPI_BASE + 0x0014
#define MIPI_DPI_LP_CMD_TIM                               MIPI_BASE + 0x0018
#define MIPI_PCKHDL_CFG                                   MIPI_BASE + 0x002C
#define MIPI_GEN_VCID                                     MIPI_BASE + 0x0030
#define MIPI_MODE_CFG                                     MIPI_BASE + 0x0034
#define MIPI_VID_MODE_CFG                                 MIPI_BASE + 0x0038
#define MIPI_VID_PKG_SIZE                                 MIPI_BASE + 0x003C
#define MIPI_VID_NUM_CHUNKS                               MIPI_BASE + 0x0040
#define MIPI_VID_NULL_SIZE                                MIPI_BASE + 0x0044
#define MIPI_VID_HSA_TIME                                 MIPI_BASE + 0x0048
#define MIPI_VID_HBP_TIME                                 MIPI_BASE + 0x004C
#define MIPI_VID_HLINE_TIME                               MIPI_BASE + 0x0050
#define MIPI_VID_VSA_LINES                                MIPI_BASE + 0x0054
#define MIPI_VID_VBP_LINES                                MIPI_BASE + 0x0058
#define MIPI_VID_VFP_LINES                                MIPI_BASE + 0x005C
#define MIPI_VID_VACT_LINES                               MIPI_BASE + 0x0060
#define MIPI_EDPI_CMD_SIZE                                MIPI_BASE + 0x0064
#define MIPI_CMD_MODE_CFG                                 MIPI_BASE + 0x0068
#define MIPI_GEN_HDR                                      MIPI_BASE + 0x006C
#define MIPI_GEN_PLD_DATA                                 MIPI_BASE + 0x0070
#define MIPI_CMD_PKT_STATUS                               MIPI_BASE + 0x0074
#define MIPI_TO_CNT_CFG                                   MIPI_BASE + 0x0078
#define MIPI_HS_RD_TO_CNT                                 MIPI_BASE + 0x007C
#define MIPI_LP_RD_TO_CNT                                 MIPI_BASE + 0x0080
#define MIPI_HS_WR_TO_CNT                                 MIPI_BASE + 0x0084
#define MIPI_LP_WR_TO_CNT                                 MIPI_BASE + 0x0088
#define MIPI_BTA_TO_CNT                                   MIPI_BASE + 0x008C
#define MIPI_SDF_3D                                       MIPI_BASE + 0x0090
#define MIPI_LPCLK_CTRL                                   MIPI_BASE + 0x0094
#define MIPI_PHY_TMR_LPCLK_CFG                            MIPI_BASE + 0x0098
#define MIPI_PHY_TMR_CFG                                  MIPI_BASE + 0x009C
#define MIPI_PHY_RSTZ                                     MIPI_BASE + 0x00A0
#define MIPI_PHY_IF_CFG                                   MIPI_BASE + 0x00A4
#define MIPI_PHY_ULPS_CTRL                                MIPI_BASE + 0x00A8
#define MIPI_PHY_TX_TRIGGERS                              MIPI_BASE + 0x00AC
#define MIPI_PHY_STATUS                                   MIPI_BASE + 0x00B0
#define MIPI_PHY_TEST_CTRL0                               MIPI_BASE + 0x00B4
#define MIPI_PHY_TEST_CTRL1                               MIPI_BASE + 0x00B8
#define MIPI_ERROR_ST0                                    MIPI_BASE + 0x00BC
#define MIPI_ERROR_ST1                                    MIPI_BASE + 0x00C0
#define MIPI_ERROR_MSK0                                   MIPI_BASE + 0x00C4
#define MIPI_ERROR_MSK1                                   MIPI_BASE + 0x00C8

/*******************************************************************************
DSI_ISP registers' address on LC1860
********************************************************************************/
/*DSI_ISP_CTLµÍ¹¦ºÄ¿ØÖÆ¼Ä´æÆ÷*/
#define DSI_ISP_CTL_LP_MODE_CTRL                          DSI_ISP_BASE + 0x00A0
/*ISP PHY¿ØÖÆ¼Ä´æÆ÷*/
#define DSI_ISP_CTL_CPHY0_MODE_CTRL                       DSI_ISP_BASE + 0x00F0
#define DSI_ISP_CTL_CPHY0_RSTZ_CTRL                       DSI_ISP_BASE + 0x00F4
#define DSI_ISP_CTL_CPHY0_CTRL0                           DSI_ISP_BASE + 0x00F8
#define DSI_ISP_CTL_CPHY0_CTRL1                           DSI_ISP_BASE + 0x00FC
#define DSI_ISP_CTL_CPHY0_ERROR_STAT                      DSI_ISP_BASE + 0x0100
#define DSI_ISP_CTL_CPHY0_STAT                            DSI_ISP_BASE + 0x0104
#define DSI_ISP_CTL_CPHY1_MODE_CTRL                       DSI_ISP_BASE + 0x0108
#define DSI_ISP_CTL_CPHY1_RSTZ_CTRL                       DSI_ISP_BASE + 0x010C
#define DSI_ISP_CTL_CPHY1_CTRL0                           DSI_ISP_BASE + 0x0110
#define DSI_ISP_CTL_CPHY1_CTRL1                           DSI_ISP_BASE + 0x0114
#define DSI_ISP_CTL_CPHY1_ERROR_STAT                      DSI_ISP_BASE + 0x0118
#define DSI_ISP_CTL_CPHY1_STAT                            DSI_ISP_BASE + 0x011C

/*******************************************************************************
AP_DMAG registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define AP_DMAG_CH_STATUS                                 AP_DMAG_BASE + 0x0000
#define AP_DMAG_CH_PRIOR0                                 AP_DMAG_BASE + 0x0020
#define AP_DMAG_CH_PRIOR1                                 AP_DMAG_BASE + 0x0024
#define AP_DMAG_CH_PRIOR2                                 AP_DMAG_BASE + 0x0028
#define AP_DMAG_CH_INTR_EN0                               AP_DMAG_BASE + 0x0040
#define AP_DMAG_CH_INTR_MASK0                             AP_DMAG_BASE + 0x0044
#define AP_DMAG_CH_INTR_STATUS0                           AP_DMAG_BASE + 0x0048
#define AP_DMAG_CH_LP_EN0                                 AP_DMAG_BASE + 0x0080
#define AP_DMAG_CH_BUS_LP_EN                              AP_DMAG_BASE + 0x0088
#define AP_DMAG_RAM_ADDR_BOUNDARY0                        AP_DMAG_BASE + 0x00A0
#define AP_DMAG_RAM_ADDR_BOUNDARY1                        AP_DMAG_BASE + 0x00A4
#define AP_DMAG_RAM_ADDR_BOUNDARY2                        AP_DMAG_BASE + 0x00A8
#define AP_DMAG_RAM_ADDR_BOUNDARY3                        AP_DMAG_BASE + 0x00AC
#define AP_DMAG_RAM_ADDR_BOUNDARY4                        AP_DMAG_BASE + 0x00B0
#define AP_DMAG_RAM_ADDR_BOUNDARY5                        AP_DMAG_BASE + 0x00B4
#define AP_DMAG_RAM_ADDR_BOUNDARY6                        AP_DMAG_BASE + 0x00B8
#define AP_DMAG_RAM_ADDR_BOUNDARY7                        AP_DMAG_BASE + 0x00BC
#define AP_DMAG_RAM_ADDR_BOUNDARY8                        AP_DMAG_BASE + 0x00C0
#define AP_DMAG_RAM_ADDR_BOUNDARY9                        AP_DMAG_BASE + 0x00C4
#define AP_DMAG_RAM_ADDR_BOUNDARY10                       AP_DMAG_BASE + 0x00C8
#define AP_DMAG_RAM_ADDR_BOUNDARY11                       AP_DMAG_BASE + 0x00CC
/*Í¨µÀ0¼Ä´æÆ÷*/
#define AP_DMAG_CH0_CTRL                                  AP_DMAG_BASE + 0x0100
#define AP_DMAG_CH0_CONFIG                                AP_DMAG_BASE + 0x0104
#define AP_DMAG_CH0_SRC_ADDR                              AP_DMAG_BASE + 0x0108
#define AP_DMAG_CH0_DST_ADDR                              AP_DMAG_BASE + 0x0110
#define AP_DMAG_CH0_SIZE                                  AP_DMAG_BASE + 0x0118
#define AP_DMAG_CH0_LINK_ADDR                             AP_DMAG_BASE + 0x0124
#define AP_DMAG_CH0_LINK_NUM                              AP_DMAG_BASE + 0x0128
#define AP_DMAG_CH0_INTR_EN                               AP_DMAG_BASE + 0x012C
#define AP_DMAG_CH0_INTR_STATUS                           AP_DMAG_BASE + 0x0130
#define AP_DMAG_CH0_INTR_RAW                              AP_DMAG_BASE + 0x0134
#define AP_DMAG_CH0_MONITOR_CTRL                          AP_DMAG_BASE + 0x0138
#define AP_DMAG_CH0_MONITOR_OUT                           AP_DMAG_BASE + 0x013C
/*Í¨µÀ1¼Ä´æÆ÷*/
#define AP_DMAG_CH1_CTRL                                  AP_DMAG_BASE + 0x0140
#define AP_DMAG_CH1_CONFIG                                AP_DMAG_BASE + 0x0144
#define AP_DMAG_CH1_SRC_ADDR                              AP_DMAG_BASE + 0x0148
#define AP_DMAG_CH1_DST_ADDR                              AP_DMAG_BASE + 0x0150
#define AP_DMAG_CH1_SIZE                                  AP_DMAG_BASE + 0x0158
#define AP_DMAG_CH1_LINK_ADDR                             AP_DMAG_BASE + 0x0164
#define AP_DMAG_CH1_LINK_NUM                              AP_DMAG_BASE + 0x0168
#define AP_DMAG_CH1_INTR_EN                               AP_DMAG_BASE + 0x016C
#define AP_DMAG_CH1_INTR_STATUS                           AP_DMAG_BASE + 0x0170
#define AP_DMAG_CH1_INTR_RAW                              AP_DMAG_BASE + 0x0174
#define AP_DMAG_CH1_MONITOR_CTRL                          AP_DMAG_BASE + 0x0178
#define AP_DMAG_CH1_MONITOR_OUT                           AP_DMAG_BASE + 0x017C
/*Í¨µÀ2¼Ä´æÆ÷*/
#define AP_DMAG_CH2_CTRL                                  AP_DMAG_BASE + 0x0180
#define AP_DMAG_CH2_CONFIG                                AP_DMAG_BASE + 0x0184
#define AP_DMAG_CH2_SRC_ADDR                              AP_DMAG_BASE + 0x0188
#define AP_DMAG_CH2_DST_ADDR                              AP_DMAG_BASE + 0x0190
#define AP_DMAG_CH2_SIZE                                  AP_DMAG_BASE + 0x0198
#define AP_DMAG_CH2_LINK_ADDR                             AP_DMAG_BASE + 0x01A4
#define AP_DMAG_CH2_LINK_NUM                              AP_DMAG_BASE + 0x01A8
#define AP_DMAG_CH2_INTR_EN                               AP_DMAG_BASE + 0x01AC
#define AP_DMAG_CH2_INTR_STATUS                           AP_DMAG_BASE + 0x01B0
#define AP_DMAG_CH2_INTR_RAW                              AP_DMAG_BASE + 0x01B4
#define AP_DMAG_CH2_MONITOR_CTRL                          AP_DMAG_BASE + 0x01B8
#define AP_DMAG_CH2_MONITOR_OUT                           AP_DMAG_BASE + 0x01BC
/*Í¨µÀ3¼Ä´æÆ÷*/
#define AP_DMAG_CH3_CTRL                                  AP_DMAG_BASE + 0x01C0
#define AP_DMAG_CH3_CONFIG                                AP_DMAG_BASE + 0x01C4
#define AP_DMAG_CH3_SRC_ADDR                              AP_DMAG_BASE + 0x01C8
#define AP_DMAG_CH3_DST_ADDR                              AP_DMAG_BASE + 0x01D0
#define AP_DMAG_CH3_SIZE                                  AP_DMAG_BASE + 0x01D8
#define AP_DMAG_CH3_LINK_ADDR                             AP_DMAG_BASE + 0x01E4
#define AP_DMAG_CH3_LINK_NUM                              AP_DMAG_BASE + 0x01E8
#define AP_DMAG_CH3_INTR_EN                               AP_DMAG_BASE + 0x01EC
#define AP_DMAG_CH3_INTR_STATUS                           AP_DMAG_BASE + 0x01F0
#define AP_DMAG_CH3_INTR_RAW                              AP_DMAG_BASE + 0x01F4
#define AP_DMAG_CH3_MONITOR_CTRL                          AP_DMAG_BASE + 0x01F8
#define AP_DMAG_CH3_MONITOR_OUT                           AP_DMAG_BASE + 0x01FC
/*Í¨µÀ4¼Ä´æÆ÷*/
#define AP_DMAG_CH4_CTRL                                  AP_DMAG_BASE + 0x0200
#define AP_DMAG_CH4_CONFIG                                AP_DMAG_BASE + 0x0204
#define AP_DMAG_CH4_SRC_ADDR                              AP_DMAG_BASE + 0x0208
#define AP_DMAG_CH4_DST_ADDR                              AP_DMAG_BASE + 0x0210
#define AP_DMAG_CH4_SIZE                                  AP_DMAG_BASE + 0x0218
#define AP_DMAG_CH4_LINK_ADDR                             AP_DMAG_BASE + 0x0224
#define AP_DMAG_CH4_LINK_NUM                              AP_DMAG_BASE + 0x0228
#define AP_DMAG_CH4_INTR_EN                               AP_DMAG_BASE + 0x022C
#define AP_DMAG_CH4_INTR_STATUS                           AP_DMAG_BASE + 0x0230
#define AP_DMAG_CH4_INTR_RAW                              AP_DMAG_BASE + 0x0234
#define AP_DMAG_CH4_MONITOR_CTRL                          AP_DMAG_BASE + 0x0238
#define AP_DMAG_CH4_MONITOR_OUT                           AP_DMAG_BASE + 0x023C
/*Í¨µÀ5¼Ä´æÆ÷*/
#define AP_DMAG_CH5_CTRL                                  AP_DMAG_BASE + 0x0240
#define AP_DMAG_CH5_CONFIG                                AP_DMAG_BASE + 0x0244
#define AP_DMAG_CH5_SRC_ADDR                              AP_DMAG_BASE + 0x0248
#define AP_DMAG_CH5_DST_ADDR                              AP_DMAG_BASE + 0x0250
#define AP_DMAG_CH5_SIZE                                  AP_DMAG_BASE + 0x0258
#define AP_DMAG_CH5_LINK_ADDR                             AP_DMAG_BASE + 0x0264
#define AP_DMAG_CH5_LINK_NUM                              AP_DMAG_BASE + 0x0268
#define AP_DMAG_CH5_INTR_EN                               AP_DMAG_BASE + 0x026C
#define AP_DMAG_CH5_INTR_STATUS                           AP_DMAG_BASE + 0x0270
#define AP_DMAG_CH5_INTR_RAW                              AP_DMAG_BASE + 0x0274
#define AP_DMAG_CH5_MONITOR_CTRL                          AP_DMAG_BASE + 0x0278
#define AP_DMAG_CH5_MONITOR_OUT                           AP_DMAG_BASE + 0x027C
/*Í¨µÀ6¼Ä´æÆ÷*/
#define AP_DMAG_CH6_CTRL                                  AP_DMAG_BASE + 0x0280
#define AP_DMAG_CH6_CONFIG                                AP_DMAG_BASE + 0x0284
#define AP_DMAG_CH6_SRC_ADDR                              AP_DMAG_BASE + 0x0288
#define AP_DMAG_CH6_DST_ADDR                              AP_DMAG_BASE + 0x0290
#define AP_DMAG_CH6_SIZE                                  AP_DMAG_BASE + 0x0298
#define AP_DMAG_CH6_LINK_ADDR                             AP_DMAG_BASE + 0x02A4
#define AP_DMAG_CH6_LINK_NUM                              AP_DMAG_BASE + 0x02A8
#define AP_DMAG_CH6_INTR_EN                               AP_DMAG_BASE + 0x02AC
#define AP_DMAG_CH6_INTR_STATUS                           AP_DMAG_BASE + 0x02B0
#define AP_DMAG_CH6_INTR_RAW                              AP_DMAG_BASE + 0x02B4
#define AP_DMAG_CH6_MONITOR_CTRL                          AP_DMAG_BASE + 0x02B8
#define AP_DMAG_CH6_MONITOR_OUT                           AP_DMAG_BASE + 0x02BC
/*Í¨µÀ7¼Ä´æÆ÷*/
#define AP_DMAG_CH7_CTRL                                  AP_DMAG_BASE + 0x02C0
#define AP_DMAG_CH7_CONFIG                                AP_DMAG_BASE + 0x02C4
#define AP_DMAG_CH7_SRC_ADDR                              AP_DMAG_BASE + 0x02C8
#define AP_DMAG_CH7_DST_ADDR                              AP_DMAG_BASE + 0x02D0
#define AP_DMAG_CH7_SIZE                                  AP_DMAG_BASE + 0x02D8
#define AP_DMAG_CH7_LINK_ADDR                             AP_DMAG_BASE + 0x02E4
#define AP_DMAG_CH7_LINK_NUM                              AP_DMAG_BASE + 0x02E8
#define AP_DMAG_CH7_INTR_EN                               AP_DMAG_BASE + 0x02EC
#define AP_DMAG_CH7_INTR_STATUS                           AP_DMAG_BASE + 0x02F0
#define AP_DMAG_CH7_INTR_RAW                              AP_DMAG_BASE + 0x02F4
#define AP_DMAG_CH7_MONITOR_CTRL                          AP_DMAG_BASE + 0x02F8
#define AP_DMAG_CH7_MONITOR_OUT                           AP_DMAG_BASE + 0x02FC
/*Í¨µÀ8¼Ä´æÆ÷*/
#define AP_DMAG_CH8_CTRL                                  AP_DMAG_BASE + 0x0300
#define AP_DMAG_CH8_CONFIG                                AP_DMAG_BASE + 0x0304
#define AP_DMAG_CH8_SRC_ADDR                              AP_DMAG_BASE + 0x0308
#define AP_DMAG_CH8_DST_ADDR                              AP_DMAG_BASE + 0x0310
#define AP_DMAG_CH8_SIZE                                  AP_DMAG_BASE + 0x0318
#define AP_DMAG_CH8_LINK_ADDR                             AP_DMAG_BASE + 0x0324
#define AP_DMAG_CH8_LINK_NUM                              AP_DMAG_BASE + 0x0328
#define AP_DMAG_CH8_INTR_EN                               AP_DMAG_BASE + 0x032C
#define AP_DMAG_CH8_INTR_STATUS                           AP_DMAG_BASE + 0x0330
#define AP_DMAG_CH8_INTR_RAW                              AP_DMAG_BASE + 0x0334
#define AP_DMAG_CH8_MONITOR_CTRL                          AP_DMAG_BASE + 0x0338
#define AP_DMAG_CH8_MONITOR_OUT                           AP_DMAG_BASE + 0x033C
/*Í¨µÀ9¼Ä´æÆ÷*/
#define AP_DMAG_CH9_CTRL                                  AP_DMAG_BASE + 0x0340
#define AP_DMAG_CH9_CONFIG                                AP_DMAG_BASE + 0x0344
#define AP_DMAG_CH9_SRC_ADDR                              AP_DMAG_BASE + 0x0348
#define AP_DMAG_CH9_DST_ADDR                              AP_DMAG_BASE + 0x0350
#define AP_DMAG_CH9_SIZE                                  AP_DMAG_BASE + 0x0358
#define AP_DMAG_CH9_LINK_ADDR                             AP_DMAG_BASE + 0x0364
#define AP_DMAG_CH9_LINK_NUM                              AP_DMAG_BASE + 0x0368
#define AP_DMAG_CH9_INTR_EN                               AP_DMAG_BASE + 0x036C
#define AP_DMAG_CH9_INTR_STATUS                           AP_DMAG_BASE + 0x0370
#define AP_DMAG_CH9_INTR_RAW                              AP_DMAG_BASE + 0x0374
#define AP_DMAG_CH9_MONITOR_CTRL                          AP_DMAG_BASE + 0x0378
#define AP_DMAG_CH9_MONITOR_OUT                           AP_DMAG_BASE + 0x037C
/*Í¨µÀ10¼Ä´æÆ÷*/
#define AP_DMAG_CH10_CTRL                                 AP_DMAG_BASE + 0x0380
#define AP_DMAG_CH10_CONFIG                               AP_DMAG_BASE + 0x0384
#define AP_DMAG_CH10_SRC_ADDR                             AP_DMAG_BASE + 0x0388
#define AP_DMAG_CH10_DST_ADDR                             AP_DMAG_BASE + 0x0390
#define AP_DMAG_CH10_SIZE                                 AP_DMAG_BASE + 0x0398
#define AP_DMAG_CH10_LINK_ADDR                            AP_DMAG_BASE + 0x03A4
#define AP_DMAG_CH10_LINK_NUM                             AP_DMAG_BASE + 0x03A8
#define AP_DMAG_CH10_INTR_EN                              AP_DMAG_BASE + 0x03AC
#define AP_DMAG_CH10_INTR_STATUS                          AP_DMAG_BASE + 0x03B0
#define AP_DMAG_CH10_INTR_RAW                             AP_DMAG_BASE + 0x03B4
#define AP_DMAG_CH10_MONITOR_CTRL                         AP_DMAG_BASE + 0x03B8
#define AP_DMAG_CH10_MONITOR_OUT                          AP_DMAG_BASE + 0x03BC
/*Í¨µÀ11¼Ä´æÆ÷*/
#define AP_DMAG_CH11_CTRL                                 AP_DMAG_BASE + 0x03C0
#define AP_DMAG_CH11_CONFIG                               AP_DMAG_BASE + 0x03C4
#define AP_DMAG_CH11_SRC_ADDR                             AP_DMAG_BASE + 0x03C8
#define AP_DMAG_CH11_DST_ADDR                             AP_DMAG_BASE + 0x03D0
#define AP_DMAG_CH11_SIZE                                 AP_DMAG_BASE + 0x03D8
#define AP_DMAG_CH11_LINK_ADDR                            AP_DMAG_BASE + 0x03E4
#define AP_DMAG_CH11_LINK_NUM                             AP_DMAG_BASE + 0x03E8
#define AP_DMAG_CH11_INTR_EN                              AP_DMAG_BASE + 0x03EC
#define AP_DMAG_CH11_INTR_STATUS                          AP_DMAG_BASE + 0x03F0
#define AP_DMAG_CH11_INTR_RAW                             AP_DMAG_BASE + 0x03F4
#define AP_DMAG_CH11_MONITOR_CTRL                         AP_DMAG_BASE + 0x03F8
#define AP_DMAG_CH11_MONITOR_OUT                          AP_DMAG_BASE + 0x03FC
/*ÄÚ²¿RAM½Ó¿Ú*/
#define AP_DMAG_RAM_DATA                                  AP_DMAG_BASE + 0x4000

/*******************************************************************************
AP_DMAS registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define AP_DMAS_EN                                        AP_DMAS_BASE + 0x0000
#define AP_DMAS_CLR                                       AP_DMAS_BASE + 0x0004
#define AP_DMAS_STA                                       AP_DMAS_BASE + 0x0008
#define AP_DMAS_INT_RAW0                                  AP_DMAS_BASE + 0x000C
#define AP_DMAS_INT_EN0_AP_A7                             AP_DMAS_BASE + 0x0010
#define AP_DMAS_INT0_AP_A7                                AP_DMAS_BASE + 0x001C
#define AP_DMAS_INT_CLR0                                  AP_DMAS_BASE + 0x0028
#define AP_DMAS_INT_RAW1                                  AP_DMAS_BASE + 0x030C
#define AP_DMAS_INT_EN1_AP_A7                             AP_DMAS_BASE + 0x0310
#define AP_DMAS_INT1_AP_A7                                AP_DMAS_BASE + 0x031C
#define AP_DMAS_INT_CLR1                                  AP_DMAS_BASE + 0x0328
#define AP_DMAS_INTV_UNIT                                 AP_DMAS_BASE + 0x002C
#define AP_DMAS_LP_CTL                                    AP_DMAS_BASE + 0x03FC
/*·¢ËÍÍ¨µÀ¼Ä´æÆ÷*/
#define AP_DMAS_CH0_SAR                                   AP_DMAS_BASE + 0x0040
#define AP_DMAS_CH0_DAR                                   AP_DMAS_BASE + 0x0044
#define AP_DMAS_CH0_CTL0                                  AP_DMAS_BASE + 0x0048
#define AP_DMAS_CH0_CTL1                                  AP_DMAS_BASE + 0x004C
#define AP_DMAS_CH0_WD                                    AP_DMAS_BASE + 0x0240
#define AP_DMAS_CH0_CA                                    AP_DMAS_BASE + 0x0050
#define AP_DMAS_CH0_INTA                                  AP_DMAS_BASE + 0x0054
#define AP_DMAS_CH1_SAR                                   AP_DMAS_BASE + 0x0060
#define AP_DMAS_CH1_DAR                                   AP_DMAS_BASE + 0x0064
#define AP_DMAS_CH1_CTL0                                  AP_DMAS_BASE + 0x0068
#define AP_DMAS_CH1_CTL1                                  AP_DMAS_BASE + 0x006C
#define AP_DMAS_CH1_WD                                    AP_DMAS_BASE + 0x0244
#define AP_DMAS_CH1_CA                                    AP_DMAS_BASE + 0x0070
#define AP_DMAS_CH1_INTA                                  AP_DMAS_BASE + 0x0074
#define AP_DMAS_CH2_SAR                                   AP_DMAS_BASE + 0x0080
#define AP_DMAS_CH2_DAR                                   AP_DMAS_BASE + 0x0084
#define AP_DMAS_CH2_CTL0                                  AP_DMAS_BASE + 0x0088
#define AP_DMAS_CH2_CTL1                                  AP_DMAS_BASE + 0x008C
#define AP_DMAS_CH2_WD                                    AP_DMAS_BASE + 0x0248
#define AP_DMAS_CH2_CA                                    AP_DMAS_BASE + 0x0090
#define AP_DMAS_CH2_INTA                                  AP_DMAS_BASE + 0x0094
#define AP_DMAS_CH3_SAR                                   AP_DMAS_BASE + 0x00A0
#define AP_DMAS_CH3_DAR                                   AP_DMAS_BASE + 0x00A4
#define AP_DMAS_CH3_CTL0                                  AP_DMAS_BASE + 0x00A8
#define AP_DMAS_CH3_CTL1                                  AP_DMAS_BASE + 0x00AC
#define AP_DMAS_CH3_WD                                    AP_DMAS_BASE + 0x024C
#define AP_DMAS_CH3_CA                                    AP_DMAS_BASE + 0x00B0
#define AP_DMAS_CH3_INTA                                  AP_DMAS_BASE + 0x00B4
#define AP_DMAS_CH4_SAR                                   AP_DMAS_BASE + 0x00C0
#define AP_DMAS_CH4_DAR                                   AP_DMAS_BASE + 0x00C4
#define AP_DMAS_CH4_CTL0                                  AP_DMAS_BASE + 0x00C8
#define AP_DMAS_CH4_CTL1                                  AP_DMAS_BASE + 0x00CC
#define AP_DMAS_CH4_WD                                    AP_DMAS_BASE + 0x0250
#define AP_DMAS_CH4_CA                                    AP_DMAS_BASE + 0x00D0
#define AP_DMAS_CH4_INTA                                  AP_DMAS_BASE + 0x00D4
#define AP_DMAS_CH5_SAR                                   AP_DMAS_BASE + 0x00E0
#define AP_DMAS_CH5_DAR                                   AP_DMAS_BASE + 0x00E4
#define AP_DMAS_CH5_CTL0                                  AP_DMAS_BASE + 0x00E8
#define AP_DMAS_CH5_CTL1                                  AP_DMAS_BASE + 0x00EC
#define AP_DMAS_CH5_WD                                    AP_DMAS_BASE + 0x0254
#define AP_DMAS_CH5_CA                                    AP_DMAS_BASE + 0x00F0
#define AP_DMAS_CH5_INTA                                  AP_DMAS_BASE + 0x00F4
#define AP_DMAS_CH6_SAR                                   AP_DMAS_BASE + 0x0100
#define AP_DMAS_CH6_DAR                                   AP_DMAS_BASE + 0x0104
#define AP_DMAS_CH6_CTL0                                  AP_DMAS_BASE + 0x0108
#define AP_DMAS_CH6_CTL1                                  AP_DMAS_BASE + 0x010C
#define AP_DMAS_CH6_WD                                    AP_DMAS_BASE + 0x0258
#define AP_DMAS_CH6_CA                                    AP_DMAS_BASE + 0x0110
#define AP_DMAS_CH6_INTA                                  AP_DMAS_BASE + 0x0114
#define AP_DMAS_CH7_SAR                                   AP_DMAS_BASE + 0x0120
#define AP_DMAS_CH7_DAR                                   AP_DMAS_BASE + 0x0124
#define AP_DMAS_CH7_CTL0                                  AP_DMAS_BASE + 0x0128
#define AP_DMAS_CH7_CTL1                                  AP_DMAS_BASE + 0x012C
#define AP_DMAS_CH7_WD                                    AP_DMAS_BASE + 0x025C
#define AP_DMAS_CH7_CA                                    AP_DMAS_BASE + 0x0130
#define AP_DMAS_CH7_INTA                                  AP_DMAS_BASE + 0x0134
/*½ÓÊÕÍ¨µÀ¼Ä´æÆ÷*/
#define AP_DMAS_CH8_SAR                                   AP_DMAS_BASE + 0x0140
#define AP_DMAS_CH8_DAR                                   AP_DMAS_BASE + 0x0144
#define AP_DMAS_CH8_CTL0                                  AP_DMAS_BASE + 0x0148
#define AP_DMAS_CH8_CTL1                                  AP_DMAS_BASE + 0x014C
#define AP_DMAS_CH8_CA                                    AP_DMAS_BASE + 0x0150
#define AP_DMAS_CH8_INTA                                  AP_DMAS_BASE + 0x0154
#define AP_DMAS_CH9_SAR                                   AP_DMAS_BASE + 0x0160
#define AP_DMAS_CH9_DAR                                   AP_DMAS_BASE + 0x0164
#define AP_DMAS_CH9_CTL0                                  AP_DMAS_BASE + 0x0168
#define AP_DMAS_CH9_CTL1                                  AP_DMAS_BASE + 0x016C
#define AP_DMAS_CH9_CA                                    AP_DMAS_BASE + 0x0170
#define AP_DMAS_CH9_INTA                                  AP_DMAS_BASE + 0x0174
#define AP_DMAS_CH10_SAR                                  AP_DMAS_BASE + 0x0180
#define AP_DMAS_CH10_DAR                                  AP_DMAS_BASE + 0x0184
#define AP_DMAS_CH10_CTL0                                 AP_DMAS_BASE + 0x0188
#define AP_DMAS_CH10_CTL1                                 AP_DMAS_BASE + 0x018C
#define AP_DMAS_CH10_CA                                   AP_DMAS_BASE + 0x0190
#define AP_DMAS_CH10_INTA                                 AP_DMAS_BASE + 0x0194
#define AP_DMAS_CH11_SAR                                  AP_DMAS_BASE + 0x01A0
#define AP_DMAS_CH11_DAR                                  AP_DMAS_BASE + 0x01A4
#define AP_DMAS_CH11_CTL0                                 AP_DMAS_BASE + 0x01A8
#define AP_DMAS_CH11_CTL1                                 AP_DMAS_BASE + 0x01AC
#define AP_DMAS_CH11_CA                                   AP_DMAS_BASE + 0x01B0
#define AP_DMAS_CH11_INTA                                 AP_DMAS_BASE + 0x01B4
#define AP_DMAS_CH12_SAR                                  AP_DMAS_BASE + 0x01C0
#define AP_DMAS_CH12_DAR                                  AP_DMAS_BASE + 0x01C4
#define AP_DMAS_CH12_CTL0                                 AP_DMAS_BASE + 0x01C8
#define AP_DMAS_CH12_CTL1                                 AP_DMAS_BASE + 0x01CC
#define AP_DMAS_CH12_CA                                   AP_DMAS_BASE + 0x01D0
#define AP_DMAS_CH12_INTA                                 AP_DMAS_BASE + 0x01D4
#define AP_DMAS_CH13_SAR                                  AP_DMAS_BASE + 0x01E0
#define AP_DMAS_CH13_DAR                                  AP_DMAS_BASE + 0x01E4
#define AP_DMAS_CH13_CTL0                                 AP_DMAS_BASE + 0x01E8
#define AP_DMAS_CH13_CTL1                                 AP_DMAS_BASE + 0x01EC
#define AP_DMAS_CH13_CA                                   AP_DMAS_BASE + 0x01F0
#define AP_DMAS_CH13_INTA                                 AP_DMAS_BASE + 0x01F4
#define AP_DMAS_CH14_SAR                                  AP_DMAS_BASE + 0x0200
#define AP_DMAS_CH14_DAR                                  AP_DMAS_BASE + 0x0204
#define AP_DMAS_CH14_CTL0                                 AP_DMAS_BASE + 0x0208
#define AP_DMAS_CH14_CTL1                                 AP_DMAS_BASE + 0x020C
#define AP_DMAS_CH14_CA                                   AP_DMAS_BASE + 0x0210
#define AP_DMAS_CH14_INTA                                 AP_DMAS_BASE + 0x0214
#define AP_DMAS_CH15_SAR                                  AP_DMAS_BASE + 0x0220
#define AP_DMAS_CH15_DAR                                  AP_DMAS_BASE + 0x0224
#define AP_DMAS_CH15_CTL0                                 AP_DMAS_BASE + 0x0228
#define AP_DMAS_CH15_CTL1                                 AP_DMAS_BASE + 0x022C
#define AP_DMAS_CH15_CA                                   AP_DMAS_BASE + 0x0230
#define AP_DMAS_CH15_INTA                                 AP_DMAS_BASE + 0x0234

/*******************************************************************************
AP_DMAC registers' address on LC1860
********************************************************************************/
/*AP_DMACÍ¨µÀ¼Ä´æÆ÷*/
#define AP_DMAC_SAR0                                      AP_DMAC_BASE + 0x0000
#define AP_DMAC_DAR0                                      AP_DMAC_BASE + 0x0008
#define AP_DMAC_LLP0                                      AP_DMAC_BASE + 0x0010
#define AP_DMAC_CTL0_L                                    AP_DMAC_BASE + 0x0018
#define AP_DMAC_CTL0_H                                    AP_DMAC_BASE + 0x001C
#define AP_DMAC_CFG0_L                                    AP_DMAC_BASE + 0x0040
#define AP_DMAC_CFG0_H                                    AP_DMAC_BASE + 0x0044
#define AP_DMAC_SGR0                                      AP_DMAC_BASE + 0x0048
#define AP_DMAC_DSR0                                      AP_DMAC_BASE + 0x0050
#define AP_DMAC_SAR1                                      AP_DMAC_BASE + 0x0058
#define AP_DMAC_DAR1                                      AP_DMAC_BASE + 0x0060
#define AP_DMAC_LLP1                                      AP_DMAC_BASE + 0x0068
#define AP_DMAC_CTL1_L                                    AP_DMAC_BASE + 0x0070
#define AP_DMAC_CTL1_H                                    AP_DMAC_BASE + 0x0074
#define AP_DMAC_CFG1_L                                    AP_DMAC_BASE + 0x0098
#define AP_DMAC_CFG1_H                                    AP_DMAC_BASE + 0x009C
#define AP_DMAC_SGR1                                      AP_DMAC_BASE + 0x00A0
#define AP_DMAC_DSR1                                      AP_DMAC_BASE + 0x00A8
/*AP_DMACÔ­Ê¼ÖÐ¶Ï×´Ì¬¼Ä´æÆ÷*/
#define AP_DMAC_RAWTFR                                    AP_DMAC_BASE + 0x02C0
#define AP_DMAC_RAWBLOCK                                  AP_DMAC_BASE + 0x02C8
#define AP_DMAC_RAWERR                                    AP_DMAC_BASE + 0x02E0
/*AP_DMACÖÐ¶Ï×´Ì¬¼Ä´æÆ÷*/
#define AP_DMAC_STATUSTFR                                 AP_DMAC_BASE + 0x02E8
#define AP_DMAC_STATUSBLOCK                               AP_DMAC_BASE + 0x02F0
#define AP_DMAC_STATUSERR                                 AP_DMAC_BASE + 0x0308
/*AP_DMACÖÐ¶ÏÊ¹ÄÜ¼Ä´æÆ÷*/
#define AP_DMAC_EXTFR                                     AP_DMAC_BASE + 0x0310
#define AP_DMAC_EXBLOCK                                   AP_DMAC_BASE + 0x0318
#define AP_DMAC_EXERR                                     AP_DMAC_BASE + 0x0330
/*AP_DMACÖÐ¶ÏÇå³ý¼Ä´æÆ÷*/
#define AP_DMAC_CLEARTFR                                  AP_DMAC_BASE + 0x0338
#define AP_DMAC_CLEARBLOCK                                AP_DMAC_BASE + 0x0340
#define AP_DMAC_CLEARERR                                  AP_DMAC_BASE + 0x0358
/*AP_DMACºÏ²¢ÖÐ¶Ï×´Ì¬¼Ä´æÆ÷*/
#define AP_DMAC_STATUSIXT                                 AP_DMAC_BASE + 0x0360
/*AP_DMACÆäËü¼Ä´æÆ÷*/
#define AP_DMAC_DMACFGREG                                 AP_DMAC_BASE + 0x0398
#define AP_DMAC_CHEXREG                                   AP_DMAC_BASE + 0x03A0

/*******************************************************************************
AES registers' address on LC1860
********************************************************************************/
#define AES_START                                         AES_BASE + 0x0000
#define AES_CTL                                           AES_BASE + 0x0004
#define AES_INTR                                          AES_BASE + 0x0008
#define AES_INTE                                          AES_BASE + 0x000C
#define AES_INTS                                          AES_BASE + 0x0010
#define AES_KEY_LEN                                       AES_BASE + 0x0014
#define AES_KEY_UPDATE                                    AES_BASE + 0x0018
#define AES_BLK_NUM                                       AES_BASE + 0x001C
#define AES_SADDR                                         AES_BASE + 0x0020
#define AES_CUR_SAR                                       AES_BASE + 0x0024
#define AES_TADDR                                         AES_BASE + 0x0030
#define AES_CUR_TAR                                       AES_BASE + 0x0034
#define AES_IN0                                           AES_BASE + 0x0040
#define AES_IN1                                           AES_BASE + 0x0044
#define AES_IN2                                           AES_BASE + 0x0048
#define AES_IN3                                           AES_BASE + 0x004C
#define AES_OUT0                                          AES_BASE + 0x0050
#define AES_OUT1                                          AES_BASE + 0x0054
#define AES_OUT2                                          AES_BASE + 0x0058
#define AES_OUT3                                          AES_BASE + 0x005C
#define AES_KEY0                                          AES_BASE + 0x0060
#define AES_KEY1                                          AES_BASE + 0x0064
#define AES_KEY2                                          AES_BASE + 0x0068
#define AES_KEY3                                          AES_BASE + 0x006C
#define AES_KEY4                                          AES_BASE + 0x0070
#define AES_KEY5                                          AES_BASE + 0x0074
#define AES_KEY6                                          AES_BASE + 0x0078
#define AES_KEY7                                          AES_BASE + 0x007C
#define AES_KEY_MASK0                                     AES_BASE + 0x0080
#define AES_KEY_MASK1                                     AES_BASE + 0x0084
#define AES_KEY_MASK2                                     AES_BASE + 0x0088
#define AES_KEY_MASK3                                     AES_BASE + 0x008C
#define AES_KEY_MASK4                                     AES_BASE + 0x0090
#define AES_KEY_MASK5                                     AES_BASE + 0x0094
#define AES_KEY_MASK6                                     AES_BASE + 0x0098
#define AES_KEY_MASK7                                     AES_BASE + 0x009C
#define AES_LP_CTRL                                       AES_BASE + 0x00A0

/*******************************************************************************
SHA registers' address on LC1860
********************************************************************************/
#define SHA_CTL                                           SHA_BASE + 0x0000
#define SHA_IN_TYPE                                       SHA_BASE + 0x0004
#define SHA_MOD                                           SHA_BASE + 0x0008
#define SHA_INTR                                          SHA_BASE + 0x000C
#define SHA_INTE                                          SHA_BASE + 0x0010
#define SHA_INTS                                          SHA_BASE + 0x0014
#define SHA_SADDR                                         SHA_BASE + 0x0018
#define SHA_CUR_SAR                                       SHA_BASE + 0x001C
#define SHA_LINE_NUM                                      SHA_BASE + 0x0020
#define SHA_W0                                            SHA_BASE + 0x0024
#define SHA_W1                                            SHA_BASE + 0x0028
#define SHA_W2                                            SHA_BASE + 0x002C
#define SHA_W3                                            SHA_BASE + 0x0030
#define SHA_W4                                            SHA_BASE + 0x0034
#define SHA_W5                                            SHA_BASE + 0x0038
#define SHA_W6                                            SHA_BASE + 0x003C
#define SHA_W7                                            SHA_BASE + 0x0040
#define SHA_W8                                            SHA_BASE + 0x0044
#define SHA_W9                                            SHA_BASE + 0x0048
#define SHA_W10                                           SHA_BASE + 0x004C
#define SHA_W11                                           SHA_BASE + 0x0050
#define SHA_W12                                           SHA_BASE + 0x0054
#define SHA_W13                                           SHA_BASE + 0x0058
#define SHA_W14                                           SHA_BASE + 0x005C
#define SHA_W15                                           SHA_BASE + 0x0060
#define SHA_H0                                            SHA_BASE + 0x0064
#define SHA_H1                                            SHA_BASE + 0x0068
#define SHA_H2                                            SHA_BASE + 0x006C
#define SHA_H3                                            SHA_BASE + 0x0070
#define SHA_H4                                            SHA_BASE + 0x0074
#define SHA_H5                                            SHA_BASE + 0x0078
#define SHA_H6                                            SHA_BASE + 0x007C
#define SHA_H7                                            SHA_BASE + 0x0080
#define SHA_LP_CTRL                                       SHA_BASE + 0x0084

/*******************************************************************************
NFC registers' address on LC1860
********************************************************************************/
#define NFC_CMDCTL                                        NFC_BASE + 0x0000
#define NFC_DBACTL                                        NFC_BASE + 0x0004
#define NFC_AUTCTL                                        NFC_BASE + 0x0008
#define NFC_SPR_CMDCTL                                    NFC_BASE + 0x000C
#define NFC_ECCTL                                         NFC_BASE + 0x0010
#define NFC_DSEL                                          NFC_BASE + 0x0014
#define NFC_ACOMM_1                                       NFC_BASE + 0x0018
#define NFC_ACOMM_2                                       NFC_BASE + 0x001C
#define NFC_A0                                            NFC_BASE + 0x0020
#define NFC_A1                                            NFC_BASE + 0x0024
#define NFC_A2                                            NFC_BASE + 0x0028
#define NFC_A3                                            NFC_BASE + 0x002C
#define NFC_A4                                            NFC_BASE + 0x0030
#define NFC_A5                                            NFC_BASE + 0x0034
#define NFC_A6                                            NFC_BASE + 0x0038
#define NFC_A7                                            NFC_BASE + 0x003C
#define NFC_BCOMM_1                                       NFC_BASE + 0x0040
#define NFC_BCOMM_2                                       NFC_BASE + 0x0044
#define NFC_A_B0                                          NFC_BASE + 0x0048
#define NFC_A_B1                                          NFC_BASE + 0x004C
#define NFC_A_B2                                          NFC_BASE + 0x0050
#define NFC_A_B3                                          NFC_BASE + 0x0054
#define NFC_A_B4                                          NFC_BASE + 0x0058
#define NFC_CTL                                           NFC_BASE + 0x005C
#define NFC_SEQ                                           NFC_BASE + 0x0060
#define NFC_DATA_LB                                       NFC_BASE + 0x0068
#define NFC_DATA_HB                                       NFC_BASE + 0x006C
#define NFC_TYPE                                          NFC_BASE + 0x0070
#define NFC_BUSY_CNT                                      NFC_BASE + 0x0074
#define NFC_MAP                                           NFC_BASE + 0x0078
#define NFC_FUNC_CTL                                      NFC_BASE + 0x007C
#define NFC_STB_LWIDTH                                    NFC_BASE + 0x0080
#define NFC_STB_HWIDTH                                    NFC_BASE + 0x0084
#define NFC_SPEC                                          NFC_BASE + 0x0088
#define NFC_STAT                                          NFC_BASE + 0x008C
#define NFC_ID1                                           NFC_BASE + 0x0090
#define NFC_ID2                                           NFC_BASE + 0x0094
#define NFC_ID3                                           NFC_BASE + 0x0098
#define NFC_ID4                                           NFC_BASE + 0x009C
#define NFC_ID5                                           NFC_BASE + 0x00A0
#define NFC_SPR_CNT                                       NFC_BASE + 0x00A4
#define NFC_CMPCTL                                        NFC_BASE + 0x00A8
#define NFC_CMPST                                         NFC_BASE + 0x00AC
#define NFC_ECCTL1                                        NFC_BASE + 0x00B0
#define NFC_SECT_NUM                                      NFC_BASE + 0x00B4
#define NFC_ID6                                           NFC_BASE + 0x00B8
#define NFC_ID7                                           NFC_BASE + 0x00BC
#define NFC_ID8                                           NFC_BASE + 0x00C0
#define NFC_SPR_REG0                                      NFC_BASE + 0x00C4
#define NFC_SPR_REG1                                      NFC_BASE + 0x00C8
#define NFC_SPR_REG2                                      NFC_BASE + 0x00CC
#define NFC_SPR_REG3                                      NFC_BASE + 0x00D0
#define NFC_SPR_REG4                                      NFC_BASE + 0x00D4
#define NFC_SPR_REG5                                      NFC_BASE + 0x00D8
#define NFC_SPR_REG6                                      NFC_BASE + 0x00DC
#define NFC_SPR_REG7                                      NFC_BASE + 0x00E0
#define NFC_SPR_REG8                                      NFC_BASE + 0x00E4
#define NFC_SPR_REG9                                      NFC_BASE + 0x00E8
#define NFC_SPR_REG10                                     NFC_BASE + 0x00EC
#define NFC_SPR_REG11                                     NFC_BASE + 0x00F0
#define NFC_SPR_REG12                                     NFC_BASE + 0x00F4
#define NFC_SPR_REG13                                     NFC_BASE + 0x00F8
#define NFC_SPR_REG14                                     NFC_BASE + 0x00FC
#define NFC_SPR_REG15                                     NFC_BASE + 0x0100
#define NFC_SPR_REG16                                     NFC_BASE + 0x0104
#define NFC_SPR_REG17                                     NFC_BASE + 0x0108
#define NFC_SPR_REG18                                     NFC_BASE + 0x010C
#define NFC_SPR_REG19                                     NFC_BASE + 0x0110
#define NFC_SPR_REG20                                     NFC_BASE + 0x0114
#define NFC_SPR_REG21                                     NFC_BASE + 0x0118
#define NFC_SPR_REG22                                     NFC_BASE + 0x011C
#define NFC_SPR_REG23                                     NFC_BASE + 0x0120
#define NFC_SPR_REG24                                     NFC_BASE + 0x0124
#define NFC_SPR_REG25                                     NFC_BASE + 0x0128
#define NFC_SPR_REG26                                     NFC_BASE + 0x012C
#define NFC_SPR_REG27                                     NFC_BASE + 0x0130
#define NFC_SPR_REG28                                     NFC_BASE + 0x0134
#define NFC_SPR_REG29                                     NFC_BASE + 0x0138
#define NFC_SPR_REG30                                     NFC_BASE + 0x013C
#define NFC_SPR_REG31                                     NFC_BASE + 0x0140
#define NFC_SPR_REG32                                     NFC_BASE + 0x0144
#define NFC_SPR_REG33                                     NFC_BASE + 0x0148
#define NFC_SPR_REG34                                     NFC_BASE + 0x014C
#define NFC_SPR_REG35                                     NFC_BASE + 0x0150
#define NFC_SPR_REG36                                     NFC_BASE + 0x0154
#define NFC_SPR_REG37                                     NFC_BASE + 0x0158
#define NFC_SPR_REG38                                     NFC_BASE + 0x015C
#define NFC_SPR_REG39                                     NFC_BASE + 0x0160
#define NFC_SPR_REG40                                     NFC_BASE + 0x0164
#define NFC_SPR_REG41                                     NFC_BASE + 0x0168
#define NFC_SPR_REG42                                     NFC_BASE + 0x016C
#define NFC_SPR_REG43                                     NFC_BASE + 0x0170
#define NFC_SPR_REG44                                     NFC_BASE + 0x0174
#define NFC_SPR_REG45                                     NFC_BASE + 0x0178
#define NFC_SPR_REG46                                     NFC_BASE + 0x017C
#define NFC_SPR_REG47                                     NFC_BASE + 0x0180
#define NFC_SPR_REG48                                     NFC_BASE + 0x0184
#define NFC_SPR_REG49                                     NFC_BASE + 0x0188
#define NFC_SPR_REG50                                     NFC_BASE + 0x018C
#define NFC_SPR_REG51                                     NFC_BASE + 0x0190
#define NFC_SPR_REG52                                     NFC_BASE + 0x0194
#define NFC_SPR_REG53                                     NFC_BASE + 0x0198
#define NFC_SPR_REG54                                     NFC_BASE + 0x019C
#define NFC_SPR_REG55                                     NFC_BASE + 0x01A0
#define NFC_SPR_REG56                                     NFC_BASE + 0x01A4
#define NFC_SPR_REG57                                     NFC_BASE + 0x01A8
#define NFC_SPR_REG58                                     NFC_BASE + 0x01AC
#define NFC_SPR_REG59                                     NFC_BASE + 0x01B0
#define NFC_SPR_REG60                                     NFC_BASE + 0x01B4
#define NFC_SPR_REG61                                     NFC_BASE + 0x01B8
#define NFC_SPR_REG62                                     NFC_BASE + 0x01BC
#define NFC_SPR_REG63                                     NFC_BASE + 0x01C0
#define NFC_SPR_REG64                                     NFC_BASE + 0x01C4
#define NFC_SPR_REG65                                     NFC_BASE + 0x01C8
#define NFC_SPR_REG66                                     NFC_BASE + 0x01CC
#define NFC_SPR_REG67                                     NFC_BASE + 0x01D0
#define NFC_SPR_REG68                                     NFC_BASE + 0x01D4
#define NFC_SPR_REG69                                     NFC_BASE + 0x01D8
#define NFC_SPR_REG70                                     NFC_BASE + 0x01DC
#define NFC_SPR_REG71                                     NFC_BASE + 0x01E0
#define NFC_SPR_REG72                                     NFC_BASE + 0x01E4
#define NFC_SPR_REG73                                     NFC_BASE + 0x01E8
#define NFC_SPR_REG74                                     NFC_BASE + 0x01EC
#define NFC_SPR_REG75                                     NFC_BASE + 0x01F0
#define NFC_SPR_REG76                                     NFC_BASE + 0x01F4
#define NFC_SPR_REG77                                     NFC_BASE + 0x01F8
#define NFC_SPR_REG78                                     NFC_BASE + 0x01FC
#define NFC_SPR_REG79                                     NFC_BASE + 0x0200
#define NFC_SPR_REG80                                     NFC_BASE + 0x0204
#define NFC_SPR_REG81                                     NFC_BASE + 0x0208
#define NFC_SPR_REG82                                     NFC_BASE + 0x020C
#define NFC_SPR_REG83                                     NFC_BASE + 0x0210
#define NFC_SPR_REG84                                     NFC_BASE + 0x0214
#define NFC_SPR_REG85                                     NFC_BASE + 0x0218
#define NFC_SPR_REG86                                     NFC_BASE + 0x021C
#define NFC_SPR_REG87                                     NFC_BASE + 0x0220
#define NFC_SPR_REG88                                     NFC_BASE + 0x0224
#define NFC_SPR_REG89                                     NFC_BASE + 0x0228
#define NFC_SPR_REG90                                     NFC_BASE + 0x022C
#define NFC_SPR_REG91                                     NFC_BASE + 0x0230
#define NFC_SPR_REG92                                     NFC_BASE + 0x0234
#define NFC_SPR_REG93                                     NFC_BASE + 0x0238
#define NFC_SPR_REG94                                     NFC_BASE + 0x023C
#define NFC_SPR_REG95                                     NFC_BASE + 0x0240
#define NFC_SPR_REG96                                     NFC_BASE + 0x0244
#define NFC_SPR_REG97                                     NFC_BASE + 0x0248
#define NFC_SPR_REG98                                     NFC_BASE + 0x024C
#define NFC_SPR_REG99                                     NFC_BASE + 0x0250
#define NFC_SPR_REG100                                    NFC_BASE + 0x0254
#define NFC_SPR_REG101                                    NFC_BASE + 0x0258
#define NFC_SPR_REG102                                    NFC_BASE + 0x025C
#define NFC_SPR_REG103                                    NFC_BASE + 0x0260
#define NFC_SPR_REG104                                    NFC_BASE + 0x0264
#define NFC_INT_EN                                        NFC_BASE + 0x02C0
#define NFC_INT_MASK                                      NFC_BASE + 0x02C4
#define NFC_INT                                           NFC_BASE + 0x02C8
#define NFC_MSKED_INT                                     NFC_BASE + 0x02CC
#define NFC_INT_CLR                                       NFC_BASE + 0x02D0
#define NFC_CONFIG                                        NFC_BASE + 0x02D4
#define NFC_FIFO_REG                                      NFC_BASE + 0x0300

/*******************************************************************************
SDMMC1 registers' address on LC1860
********************************************************************************/
#define SDMMC1_CTRL                                       SDMMC1_BASE + 0x0000
#define SDMMC1_CLKDIV                                     SDMMC1_BASE + 0x0008
#define SDMMC1_CLKENA                                     SDMMC1_BASE + 0x0010
#define SDMMC1_TMOUT                                      SDMMC1_BASE + 0x0014
#define SDMMC1_CTYPE                                      SDMMC1_BASE + 0x0018
#define SDMMC1_BLKSIZ                                     SDMMC1_BASE + 0x001C
#define SDMMC1_BYTCNT                                     SDMMC1_BASE + 0x0020
#define SDMMC1_INTEN                                      SDMMC1_BASE + 0x0024
#define SDMMC1_CMDARG                                     SDMMC1_BASE + 0x0028
#define SDMMC1_CMD                                        SDMMC1_BASE + 0x002C
#define SDMMC1_RESP0                                      SDMMC1_BASE + 0x0030
#define SDMMC1_RESP1                                      SDMMC1_BASE + 0x0034
#define SDMMC1_RESP2                                      SDMMC1_BASE + 0x0038
#define SDMMC1_RESP3                                      SDMMC1_BASE + 0x003C
#define SDMMC1_MINTSTS                                    SDMMC1_BASE + 0x0040
#define SDMMC1_RINTSTS                                    SDMMC1_BASE + 0x0044
#define SDMMC1_STATUS                                     SDMMC1_BASE + 0x0048
#define SDMMC1_FIFOTH                                     SDMMC1_BASE + 0x004C
#define SDMMC1_CDETECT                                    SDMMC1_BASE + 0x0050
#define SDMMC1_WRTPRT                                     SDMMC1_BASE + 0x0054
#define SDMMC1_TCBCNT                                     SDMMC1_BASE + 0x005C
#define SDMMC1_TBBCNT                                     SDMMC1_BASE + 0x0060
#define SDMMC1_DEBNCE                                     SDMMC1_BASE + 0x0064
#define SDMMC1_UHSREG                                     SDMMC1_BASE + 0x0074
#define SDMMC1_BMOD                                       SDMMC1_BASE + 0x0080
#define SDMMC1_PLDMND                                     SDMMC1_BASE + 0x0084
#define SDMMC1_DBADDR                                     SDMMC1_BASE + 0x0088
#define SDMMC1_IDSTS                                      SDMMC1_BASE + 0x008C
#define SDMMC1_IDINTEN                                    SDMMC1_BASE + 0x0090
#define SDMMC1_DSCADDR                                    SDMMC1_BASE + 0x0094
#define SDMMC1_BUFADDR                                    SDMMC1_BASE + 0x0098
#define SDMMC1_CARDTHRCTL                                 SDMMC1_BASE + 0x0100

/*******************************************************************************
SECURITY registers' address on LC1860
********************************************************************************/
#define SECURITY_CTRL                                     SECURITY_BASE + 0x0000
#define SECURITY_AP_PWR_CTRL                              SECURITY_BASE + 0x0004
#define SECURITY_FSM_MODE                                 SECURITY_BASE + 0x0010
#define SECURITY_KEY_CTRL                                 SECURITY_BASE + 0x0014
#define SECURITY_CONVS0_NUM                               SECURITY_BASE + 0x0018
#define SECURITY_CONVS1_NUM                               SECURITY_BASE + 0x001C
#define SECURITY_INTR_EN                                  SECURITY_BASE + 0x0020
#define SECURITY_INTR_STATUS                              SECURITY_BASE + 0x0024
#define SECURITY_INTR_RAW                                 SECURITY_BASE + 0x0028
#define SECURITY_LP_EN                                    SECURITY_BASE + 0x0030
#define SECURITY_DEBUG_CTRL0                              SECURITY_BASE + 0x0034
#define SECURITY_DEBUG_CTRL1                              SECURITY_BASE + 0x0038
#define SECURITY_SMMU_CTRL                                SECURITY_BASE + 0x003C
#define SECURITY_EFUSE_CTRL                               SECURITY_BASE + 0x0200
#define SECURITY_EFUSE_MODE                               SECURITY_BASE + 0x0204
#define SECURITY_EFUSE_ADDR                               SECURITY_BASE + 0x0208
#define SECURITY_EFUSE_LENGTH                             SECURITY_BASE + 0x020C
#define SECURITY_EFUSE_CNT                                SECURITY_BASE + 0x0210
#define SECURITY_EFUSE_DATA                               SECURITY_BASE + 0x0220
#define SECURITY_EFUSE_PGM_DATA                           SECURITY_BASE + 0x0320

/*******************************************************************************
HPI registers' address on LC1860
********************************************************************************/
#define HPI_MOD                                           HPI_BASE + 0x0000
#define HPI_MUXED_WR_TIM                                  HPI_BASE + 0x0010
#define HPI_MUXED_RD_TIM                                  HPI_BASE + 0x0014
#define HPI_LP_CTRL                                       HPI_BASE + 0x0018

/*******************************************************************************
SDMMC0 registers' address on LC1860
********************************************************************************/
#define SDMMC0_CTRL                                       SDMMC0_BASE + 0x0000
#define SDMMC0_CLKDIV                                     SDMMC0_BASE + 0x0008
#define SDMMC0_CLKENA                                     SDMMC0_BASE + 0x0010
#define SDMMC0_TMOUT                                      SDMMC0_BASE + 0x0014
#define SDMMC0_CTYPE                                      SDMMC0_BASE + 0x0018
#define SDMMC0_BLKSIZ                                     SDMMC0_BASE + 0x001C
#define SDMMC0_BYTCNT                                     SDMMC0_BASE + 0x0020
#define SDMMC0_INTEN                                      SDMMC0_BASE + 0x0024
#define SDMMC0_CMDARG                                     SDMMC0_BASE + 0x0028
#define SDMMC0_CMD                                        SDMMC0_BASE + 0x002C
#define SDMMC0_RESP0                                      SDMMC0_BASE + 0x0030
#define SDMMC0_RESP1                                      SDMMC0_BASE + 0x0034
#define SDMMC0_RESP2                                      SDMMC0_BASE + 0x0038
#define SDMMC0_RESP3                                      SDMMC0_BASE + 0x003C
#define SDMMC0_MINTSTS                                    SDMMC0_BASE + 0x0040
#define SDMMC0_RINTSTS                                    SDMMC0_BASE + 0x0044
#define SDMMC0_STATUS                                     SDMMC0_BASE + 0x0048
#define SDMMC0_FIFOTH                                     SDMMC0_BASE + 0x004C
#define SDMMC0_CDETECT                                    SDMMC0_BASE + 0x0050
#define SDMMC0_WRTPRT                                     SDMMC0_BASE + 0x0054
#define SDMMC0_TCBCNT                                     SDMMC0_BASE + 0x005C
#define SDMMC0_TBBCNT                                     SDMMC0_BASE + 0x0060
#define SDMMC0_DEBNCE                                     SDMMC0_BASE + 0x0064
#define SDMMC0_UHSREG                                     SDMMC0_BASE + 0x0074
#define SDMMC0_BMOD                                       SDMMC0_BASE + 0x0080
#define SDMMC0_PLDMND                                     SDMMC0_BASE + 0x0084
#define SDMMC0_DBADDR                                     SDMMC0_BASE + 0x0088
#define SDMMC0_IDSTS                                      SDMMC0_BASE + 0x008C
#define SDMMC0_IDINTEN                                    SDMMC0_BASE + 0x0090
#define SDMMC0_DSCADDR                                    SDMMC0_BASE + 0x0094
#define SDMMC0_BUFADDR                                    SDMMC0_BASE + 0x0098
#define SDMMC0_CARDTHRCTL                                 SDMMC0_BASE + 0x0100

/*******************************************************************************
SDMMC2 registers' address on LC1860
********************************************************************************/
#define SDMMC2_CTRL                                       SDMMC2_BASE + 0x0000
#define SDMMC2_CLKDIV                                     SDMMC2_BASE + 0x0008
#define SDMMC2_CLKENA                                     SDMMC2_BASE + 0x0010
#define SDMMC2_TMOUT                                      SDMMC2_BASE + 0x0014
#define SDMMC2_CTYPE                                      SDMMC2_BASE + 0x0018
#define SDMMC2_BLKSIZ                                     SDMMC2_BASE + 0x001C
#define SDMMC2_BYTCNT                                     SDMMC2_BASE + 0x0020
#define SDMMC2_INTEN                                      SDMMC2_BASE + 0x0024
#define SDMMC2_CMDARG                                     SDMMC2_BASE + 0x0028
#define SDMMC2_CMD                                        SDMMC2_BASE + 0x002C
#define SDMMC2_RESP0                                      SDMMC2_BASE + 0x0030
#define SDMMC2_RESP1                                      SDMMC2_BASE + 0x0034
#define SDMMC2_RESP2                                      SDMMC2_BASE + 0x0038
#define SDMMC2_RESP3                                      SDMMC2_BASE + 0x003C
#define SDMMC2_MINTSTS                                    SDMMC2_BASE + 0x0040
#define SDMMC2_RINTSTS                                    SDMMC2_BASE + 0x0044
#define SDMMC2_STATUS                                     SDMMC2_BASE + 0x0048
#define SDMMC2_FIFOTH                                     SDMMC2_BASE + 0x004C
#define SDMMC2_CDETECT                                    SDMMC2_BASE + 0x0050
#define SDMMC2_WRTPRT                                     SDMMC2_BASE + 0x0054
#define SDMMC2_TCBCNT                                     SDMMC2_BASE + 0x005C
#define SDMMC2_TBBCNT                                     SDMMC2_BASE + 0x0060
#define SDMMC2_DEBNCE                                     SDMMC2_BASE + 0x0064
#define SDMMC2_UHSREG                                     SDMMC2_BASE + 0x0074
#define SDMMC2_BMOD                                       SDMMC2_BASE + 0x0080
#define SDMMC2_PLDMND                                     SDMMC2_BASE + 0x0084
#define SDMMC2_DBADDR                                     SDMMC2_BASE + 0x0088
#define SDMMC2_IDSTS                                      SDMMC2_BASE + 0x008C
#define SDMMC2_IDINTEN                                    SDMMC2_BASE + 0x0090
#define SDMMC2_DSCADDR                                    SDMMC2_BASE + 0x0094
#define SDMMC2_BUFADDR                                    SDMMC2_BASE + 0x0098
#define SDMMC2_CARDTHRCTL                                 SDMMC2_BASE + 0x0100

/* define controller bit for SDMMC_CTRL */
#define SDMMC_CTRL_ENABLE_OD_PULLUP						24
#define SDMMC_CTRL_ABORT_READ_DATA						8
#define SDMMC_CTRL_SEND_IRQ_RESPONSE					7
#define SDMMC_CTRL_READ_WAIT							6
#define SDMMC_CTRL_DMA_ENABLE							5
#define SDMMC_CTRL_INT_ENABLE							4
#define SDMMC_CTRL_DMA_RESET							2
#define SDMMC_CTRL_FIFO_RESET							1
#define SDMMC_CTRL_CONTROLLER_RESET						0

/* define controller bit for SDMMC_CLKENA */
#define SDMMC_CLKENA_CCLK_LOW_POWER						16
#define SDMMC_CLKENA_CCLK_EN							0

/* define controller bit for SDMMC_TMOUT */
#define SDMMC_TMOUT_DATA_TIMEOUT						8
#define SDMMC_TMOUT_RESP_TIMEOUT						0

/* define controller bit for SDMMC_CTYPE */
#define SDMMC_CTYPE_CARD_WIDTH							0
#define SDMMC_CTYPE_CARD_WIDTH_1BIT						0
#define SDMMC_CTYPE_CARD_WIDTH_4BIT						1

/* define all interrupt types for SDMMC module */
#define SDMMC_CRCERR_INTR								15
#define SDMMC_ACD_INTR									14
#define SDMMC_SBE_INTR									13
#define SDMMC_HLE_INTR									12
#define SDMMC_FRUN_INTR									11
#define SDMMC_HTO_INTR									10
#define SDMMC_DRTO_INTR									9
#define SDMMC_RTO_INTR									8
#define SDMMC_DCRC_INTR									7
#define SDMMC_RCRC_INTR									6
#define SDMMC_RXDR_INTR									5
#define SDMMC_TXDR_INTR									4
#define SDMMC_DTO_INTR									3
#define SDMMC_CD_INTR									2
#define SDMMC_RE_INTR									1
#define SDMMC_CDT_INTR									0

/* define controller bit for SDMMC_CMD */
#define SDMMC_CMD_START_CMD								31
#define SDMMC_CMD_UPDATE_CLOCK_REGISTERS_ONLY			21
#define SDMMC_CMD_SEND_INITIALIZATION					15
#define SDMMC_CMD_STOP_ABORT_CMD						14
#define SDMMC_CMD_WAIT_PRVDATA_COMPLETE					13
#define SDMMC_CMD_SEND_AUTO_STOP						12
#define SDMMC_CMD_TRANSFER_MODE							11
#define SDMMC_CMD_READ_WRITE							10
#define SDMMC_CMD_DATA_TRANSFER_EXPECTED				9
#define SDMMC_CMD_CHECK_RESPONSE_CRC					8
#define SDMMC_CMD_RESPONSE_LENGTH						7
#define SDMMC_CMD_RESPONSE_EXPECT						6
#define SDMMC_CMD_CMD_INDEX								0

/* define controller bit for MMC_FIFOTH */
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE		28
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_1		0
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_4		1
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_8		2
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_16	3
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_32	4
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_64	5
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_128	6
#define SDMMC_FIFOTH_DMA_MUTIPLE_TRANSACTION_SIZE_256	7

#define SDMMC_FIFOTH_RX_WMARK							16
#define SDMMC_FIFOTH_TX_WMARK							0

#define SDMMC_STATUS_DATA_BUSY						    9

#define SDIO0_PCLK_EN	                                5
#define SDIO1_PCLK_EN	                                6
#define SDIO2_PCLK_EN	                                7
#define SDIO3_PCLK_EN	                                8
#define SEC_PCLK_GR                                     0


#define SDIO_CLK_DIV                                   8
#define SDIO_CLK1_DELAY                                4
#define SDIO_CLK2_DELAY                                4

/*******************************************************************************
SSI0 registers' address on LC1860
********************************************************************************/
#define SSI0_CTRL0                                        SSI0_BASE + 0x0000
#define SSI0_CTRL1                                        SSI0_BASE + 0x0004
#define SSI0_EN                                           SSI0_BASE + 0x0008
#define SSI0_SE                                           SSI0_BASE + 0x0010
#define SSI0_BAUD                                         SSI0_BASE + 0x0014
#define SSI0_TXFTL                                        SSI0_BASE + 0x0018
#define SSI0_RXFTL                                        SSI0_BASE + 0x001C
#define SSI0_TXFL                                         SSI0_BASE + 0x0020
#define SSI0_RXFL                                         SSI0_BASE + 0x0024
#define SSI0_STS                                          SSI0_BASE + 0x0028
#define SSI0_IE                                           SSI0_BASE + 0x002C
#define SSI0_IS                                           SSI0_BASE + 0x0030
#define SSI0_RIS                                          SSI0_BASE + 0x0034
#define SSI0_TXOIC                                        SSI0_BASE + 0x0038
#define SSI0_RXOIC                                        SSI0_BASE + 0x003C
#define SSI0_RXUIC                                        SSI0_BASE + 0x0040
#define SSI0_IC                                           SSI0_BASE + 0x0048
#define SSI0_DMAC                                         SSI0_BASE + 0x004C
#define SSI0_DMATDL                                       SSI0_BASE + 0x0050
#define SSI0_DMARDL                                       SSI0_BASE + 0x0054
#define SSI0_DATA                                         SSI0_BASE + 0x0060

/*******************************************************************************
SSI1 registers' address on LC1860
********************************************************************************/
#define SSI1_CTRL0                                        SSI1_BASE + 0x0000
#define SSI1_CTRL1                                        SSI1_BASE + 0x0004
#define SSI1_EN                                           SSI1_BASE + 0x0008
#define SSI1_SE                                           SSI1_BASE + 0x0010
#define SSI1_BAUD                                         SSI1_BASE + 0x0014
#define SSI1_TXFTL                                        SSI1_BASE + 0x0018
#define SSI1_RXFTL                                        SSI1_BASE + 0x001C
#define SSI1_TXFL                                         SSI1_BASE + 0x0020
#define SSI1_RXFL                                         SSI1_BASE + 0x0024
#define SSI1_STS                                          SSI1_BASE + 0x0028
#define SSI1_IE                                           SSI1_BASE + 0x002C
#define SSI1_IS                                           SSI1_BASE + 0x0030
#define SSI1_RIS                                          SSI1_BASE + 0x0034
#define SSI1_TXOIC                                        SSI1_BASE + 0x0038
#define SSI1_RXOIC                                        SSI1_BASE + 0x003C
#define SSI1_RXUIC                                        SSI1_BASE + 0x0040
#define SSI1_IC                                           SSI1_BASE + 0x0048
#define SSI1_DMAC                                         SSI1_BASE + 0x004C
#define SSI1_DMATDL                                       SSI1_BASE + 0x0050
#define SSI1_DMARDL                                       SSI1_BASE + 0x0054
#define SSI1_DATA                                         SSI1_BASE + 0x0060

/*******************************************************************************
UART0 registers' address on LC1860
********************************************************************************/
#define UART0_RBR                                         UART0_BASE + 0x0000
#define UART0_THR                                         UART0_BASE + 0x0000
#define UART0_DLL                                         UART0_BASE + 0x0000
#define UART0_IER                                         UART0_BASE + 0x0004
#define UART0_DLH                                         UART0_BASE + 0x0004
#define UART0_IIR                                         UART0_BASE + 0x0008
#define UART0_FCR                                         UART0_BASE + 0x0008
#define UART0_TCR                                         UART0_BASE + 0x000C
#define UART0_MCR                                         UART0_BASE + 0x0010
#define UART0_TSR                                         UART0_BASE + 0x0014
#define UART0_MSR                                         UART0_BASE + 0x0018
#define UART0_USR                                         UART0_BASE + 0x007C

/*******************************************************************************
UART1 registers' address on LC1860
********************************************************************************/
#define UART1_RBR                                         UART1_BASE + 0x0000
#define UART1_THR                                         UART1_BASE + 0x0000
#define UART1_DLL                                         UART1_BASE + 0x0000
#define UART1_IER                                         UART1_BASE + 0x0004
#define UART1_DLH                                         UART1_BASE + 0x0004
#define UART1_IIR                                         UART1_BASE + 0x0008
#define UART1_FCR                                         UART1_BASE + 0x0008
#define UART1_TCR                                         UART1_BASE + 0x000C
#define UART1_MCR                                         UART1_BASE + 0x0010
#define UART1_TSR                                         UART1_BASE + 0x0014
#define UART1_MSR                                         UART1_BASE + 0x0018
#define UART1_USR                                         UART1_BASE + 0x007C

/*******************************************************************************
UART2 registers' address on LC1860
********************************************************************************/
#define UART2_RBR                                         UART2_BASE + 0x0000
#define UART2_THR                                         UART2_BASE + 0x0000
#define UART2_DLL                                         UART2_BASE + 0x0000
#define UART2_IER                                         UART2_BASE + 0x0004
#define UART2_DLH                                         UART2_BASE + 0x0004
#define UART2_IIR                                         UART2_BASE + 0x0008
#define UART2_FCR                                         UART2_BASE + 0x0008
#define UART2_TCR                                         UART2_BASE + 0x000C
#define UART2_MCR                                         UART2_BASE + 0x0010
#define UART2_TSR                                         UART2_BASE + 0x0014
#define UART2_MSR                                         UART2_BASE + 0x0018
#define UART2_USR                                         UART2_BASE + 0x007C

/*******************************************************************************
AP_I2S registers' address on LC1860
********************************************************************************/
#define AP_I2S_SCLK_CFG                                   AP_I2S_BASE + 0x0000
#define AP_I2S_FSYNC_CFG                                  AP_I2S_BASE + 0x0004
#define AP_I2S_FIFO_STA                                   AP_I2S_BASE + 0x0008
#define AP_I2S_MODE                                       AP_I2S_BASE + 0x0010
#define AP_I2S_REC_FIFO                                   AP_I2S_BASE + 0x0014
#define AP_I2S_TRAN_FIFO                                  AP_I2S_BASE + 0x0018

/*******************************************************************************
AP_WDT0 registers' address on LC1860
********************************************************************************/
#define AP_WDT0_CR                                        AP_WDT0_BASE + 0x0000
#define AP_WDT0_TORR                                      AP_WDT0_BASE + 0x0004
#define AP_WDT0_CCVR                                      AP_WDT0_BASE + 0x0008
#define AP_WDT0_CRR                                       AP_WDT0_BASE + 0x000C
#define AP_WDT0_STAT                                      AP_WDT0_BASE + 0x0010
#define AP_WDT0_ICR                                       AP_WDT0_BASE + 0x0014

/*******************************************************************************
AP_WDT1 registers' address on LC1860
********************************************************************************/
#define AP_WDT1_CR                                        AP_WDT1_BASE + 0x0000
#define AP_WDT1_TORR                                      AP_WDT1_BASE + 0x0004
#define AP_WDT1_CCVR                                      AP_WDT1_BASE + 0x0008
#define AP_WDT1_CRR                                       AP_WDT1_BASE + 0x000C
#define AP_WDT1_STAT                                      AP_WDT1_BASE + 0x0010
#define AP_WDT1_ICR                                       AP_WDT1_BASE + 0x0014

/*******************************************************************************
AP_WDT2 registers' address on LC1860
********************************************************************************/
#define AP_WDT2_CR                                        AP_WDT2_BASE + 0x0000
#define AP_WDT2_TORR                                      AP_WDT2_BASE + 0x0004
#define AP_WDT2_CCVR                                      AP_WDT2_BASE + 0x0008
#define AP_WDT2_CRR                                       AP_WDT2_BASE + 0x000C
#define AP_WDT2_STAT                                      AP_WDT2_BASE + 0x0010
#define AP_WDT2_ICR                                       AP_WDT2_BASE + 0x0014

/*******************************************************************************
AP_WDT3 registers' address on LC1860
********************************************************************************/
#define AP_WDT3_CR                                        AP_WDT3_BASE + 0x0000
#define AP_WDT3_TORR                                      AP_WDT3_BASE + 0x0004
#define AP_WDT3_CCVR                                      AP_WDT3_BASE + 0x0008
#define AP_WDT3_CRR                                       AP_WDT3_BASE + 0x000C
#define AP_WDT3_STAT                                      AP_WDT3_BASE + 0x0010
#define AP_WDT3_ICR                                       AP_WDT3_BASE + 0x0014

/*******************************************************************************
AP_TIMER registers' address on LC1860
********************************************************************************/
/*AP_TIMER0µÄ¼Ä´æÆ÷*/
#define AP_TIMER0_TLC                                     AP_TIMER_BASE + 0x0000
#define AP_TIMER0_TCV                                     AP_TIMER_BASE + 0x0004
#define AP_TIMER0_TCR                                     AP_TIMER_BASE + 0x0008
#define AP_TIMER0_TIC                                     AP_TIMER_BASE + 0x000C
#define AP_TIMER0_TIS                                     AP_TIMER_BASE + 0x0010
/*AP_TIMER1µÄ¼Ä´æÆ÷*/
#define AP_TIMER1_TLC                                     AP_TIMER_BASE + 0x0014
#define AP_TIMER1_TCV                                     AP_TIMER_BASE + 0x0018
#define AP_TIMER1_TCR                                     AP_TIMER_BASE + 0x001C
#define AP_TIMER1_TIC                                     AP_TIMER_BASE + 0x0020
#define AP_TIMER1_TIS                                     AP_TIMER_BASE + 0x0024
/*AP_TIMER2µÄ¼Ä´æÆ÷*/
#define AP_TIMER2_TLC                                     AP_TIMER_BASE + 0x0028
#define AP_TIMER2_TCV                                     AP_TIMER_BASE + 0x002C
#define AP_TIMER2_TCR                                     AP_TIMER_BASE + 0x0030
#define AP_TIMER2_TIC                                     AP_TIMER_BASE + 0x0034
#define AP_TIMER2_TIS                                     AP_TIMER_BASE + 0x0038
/*AP_TIMER3µÄ¼Ä´æÆ÷*/
#define AP_TIMER3_TLC                                     AP_TIMER_BASE + 0x003C
#define AP_TIMER3_TCV                                     AP_TIMER_BASE + 0x0040
#define AP_TIMER3_TCR                                     AP_TIMER_BASE + 0x0044
#define AP_TIMER3_TIC                                     AP_TIMER_BASE + 0x0048
#define AP_TIMER3_TIS                                     AP_TIMER_BASE + 0x004C
/*AP_TIMER4µÄ¼Ä´æÆ÷*/
#define AP_TIMER4_TLC                                     AP_TIMER_BASE + 0x0050
#define AP_TIMER4_TCV                                     AP_TIMER_BASE + 0x0054
#define AP_TIMER4_TCR                                     AP_TIMER_BASE + 0x0058
#define AP_TIMER4_TIC                                     AP_TIMER_BASE + 0x005C
#define AP_TIMER4_TIS                                     AP_TIMER_BASE + 0x0060
/*AP_TIMER5µÄ¼Ä´æÆ÷*/
#define AP_TIMER5_TLC                                     AP_TIMER_BASE + 0x0064
#define AP_TIMER5_TCV                                     AP_TIMER_BASE + 0x0068
#define AP_TIMER5_TCR                                     AP_TIMER_BASE + 0x006C
#define AP_TIMER5_TIC                                     AP_TIMER_BASE + 0x0070
#define AP_TIMER5_TIS                                     AP_TIMER_BASE + 0x0074
/*AP_TIMER6µÄ¼Ä´æÆ÷*/
#define AP_TIMER6_TLC                                     AP_TIMER_BASE + 0x0078
#define AP_TIMER6_TCV                                     AP_TIMER_BASE + 0x007C
#define AP_TIMER6_TCR                                     AP_TIMER_BASE + 0x0080
#define AP_TIMER6_TIC                                     AP_TIMER_BASE + 0x0084
#define AP_TIMER6_TIS                                     AP_TIMER_BASE + 0x0088
/*AP_TIMER7µÄ¼Ä´æÆ÷*/
#define AP_TIMER7_TLC                                     AP_TIMER_BASE + 0x008C
#define AP_TIMER7_TCV                                     AP_TIMER_BASE + 0x0090
#define AP_TIMER7_TCR                                     AP_TIMER_BASE + 0x0094
#define AP_TIMER7_TIC                                     AP_TIMER_BASE + 0x0098
#define AP_TIMER7_TIS                                     AP_TIMER_BASE + 0x009C
/*AP_TIMERµÄÈ«¾Ö¼Ä´æÆ÷*/
#define AP_TIMER_TIS                                      AP_TIMER_BASE + 0x00A0
#define AP_TIMER_TIC                                      AP_TIMER_BASE + 0x00A4
#define AP_TIMER_RTIS                                     AP_TIMER_BASE + 0x00A8

/*******************************************************************************
I2C0 registers' address on LC1860
********************************************************************************/
#define I2C0_CON                                          I2C0_BASE + 0x0000
#define I2C0_TAR                                          I2C0_BASE + 0x0004
#define I2C0_HS_MADDR                                     I2C0_BASE + 0x000C
#define I2C0_DATA_CMD                                     I2C0_BASE + 0x0010
#define I2C0_SS_SCL_HCNT                                  I2C0_BASE + 0x0014
#define I2C0_SS_SCL_LCNT                                  I2C0_BASE + 0x0018
#define I2C0_FS_SCL_HCNT                                 I2C0_BASE + 0x001C
#define I2C0_FS_SCL_LCNT                                  I2C0_BASE + 0x0020
#define I2C0_HS_SCL_HCNT                                  I2C0_BASE + 0x0024
#define I2C0_HS_SCL_LCNT                                  I2C0_BASE + 0x0028
#define I2C0_INTR_STAT                                    I2C0_BASE + 0x002C
#define I2C0_INTR_EN                                     I2C0_BASE + 0x0030
#define I2C0_RAW_INTR_STAT                                I2C0_BASE + 0x0034
#define I2C0_RX_TL                                        I2C0_BASE + 0x0038
#define I2C0_TX_TL                                        I2C0_BASE + 0x003C
#define I2C0_CLR_INTR                                    I2C0_BASE + 0x0040
#define I2C0_CLR_RX_UNDER                                 I2C0_BASE + 0x0044
#define I2C0_CLR_RX_OVER                                  I2C0_BASE + 0x0048
#define I2C0_CLR_TX_OVER                                 I2C0_BASE + 0x004C
#define I2C0_CLR_TX_ABRT                                  I2C0_BASE + 0x0054
#define I2C0_CLR_ACTIVITY                                 I2C0_BASE + 0x005C
#define I2C0_CLR_STOP_DET                                 I2C0_BASE + 0x0060
#define I2C0_CLR_START_DET                                I2C0_BASE + 0x0064
#define I2C0_CLR_GEN_CALL                                 I2C0_BASE + 0x0068
#define I2C0_ENABLE                                      I2C0_BASE + 0x006C
#define I2C0_STATUS                                       I2C0_BASE + 0x0070
#define I2C0_TXFLR                                        I2C0_BASE + 0x0074
#define I2C0_RXFLR                                        I2C0_BASE + 0x0078
#define I2C0_SDA_HOLD                                     I2C0_BASE + 0x007C
#define I2C0_TX_ABRT_SOURCE                               I2C0_BASE + 0x0080
#define I2C0_FS_SPKLEN                                   I2C0_BASE + 0x00A0
#define I2C0_HS_SPKLEN                                    I2C0_BASE + 0x00A4

/*******************************************************************************
I2C1 registers' address on LC1860
********************************************************************************/
#define I2C1_CON                                         I2C1_BASE + 0x0000
#define I2C1_TAR                                         I2C1_BASE + 0x0004
#define I2C1_HS_MADDR                                     I2C1_BASE + 0x000C
#define I2C1_DATA_CMD                                     I2C1_BASE + 0x0010
#define I2C1_SS_SCL_HCNT                                 I2C1_BASE + 0x0014
#define I2C1_SS_SCL_LCNT                                  I2C1_BASE + 0x0018
#define I2C1_FS_SCL_HCNT                                  I2C1_BASE + 0x001C
#define I2C1_FS_SCL_LCNT                                  I2C1_BASE + 0x0020
#define I2C1_HS_SCL_HCNT                                  I2C1_BASE + 0x0024
#define I2C1_HS_SCL_LCNT                                  I2C1_BASE + 0x0028
#define I2C1_INTR_STAT                                  I2C1_BASE + 0x002C
#define I2C1_INTR_EN                                      I2C1_BASE + 0x0030
#define I2C1_RAW_INTR_STAT                              I2C1_BASE + 0x0034
#define I2C1_RX_TL                                       I2C1_BASE + 0x0038
#define I2C1_TX_TL                                       I2C1_BASE + 0x003C
#define I2C1_CLR_INTR                                     I2C1_BASE + 0x0040
#define I2C1_CLR_RX_UNDER                                 I2C1_BASE + 0x0044
#define I2C1_CLR_RX_OVER                            I2C1_BASE + 0x0048
#define I2C1_CLR_TX_OVER                                  I2C1_BASE + 0x004C
#define I2C1_CLR_TX_ABRT                                 I2C1_BASE + 0x0054
#define I2C1_CLR_ACTIVITY                                 I2C1_BASE + 0x005C
#define I2C1_CLR_STOP_DET                                 I2C1_BASE + 0x0060
#define I2C1_CLR_START_DET                              I2C1_BASE + 0x0064
#define I2C1_CLR_GEN_CALL                                 I2C1_BASE + 0x0068
#define I2C1_ENABLE                                     I2C1_BASE + 0x006C
#define I2C1_STATUS                                      I2C1_BASE + 0x0070
#define I2C1_TXFLR                                        I2C1_BASE + 0x0074
#define I2C1_RXFLR                                        I2C1_BASE + 0x0078
#define I2C1_SDA_HOLD                                     I2C1_BASE + 0x007C
#define I2C1_TX_ABRT_SOURCE                              I2C1_BASE + 0x0080
#define I2C1_FS_SPKLEN                                  I2C1_BASE + 0x00A0
#define I2C1_HS_SPKLEN                                   I2C1_BASE + 0x00A4

/*******************************************************************************
I2C3 registers' address on LC1860
********************************************************************************/
#define I2C3_CON                                          I2C3_BASE + 0x0000
#define I2C3_TAR                                          I2C3_BASE + 0x0004
#define I2C3_HS_MADDR                                    I2C3_BASE + 0x000C
#define I2C3_DATA_CMD                                     I2C3_BASE + 0x0010
#define I2C3_SS_SCL_HCNT                                 I2C3_BASE + 0x0014
#define I2C3_SS_SCL_LCNT                                 I2C3_BASE + 0x0018
#define I2C3_FS_SCL_HCNT                                  I2C3_BASE + 0x001C
#define I2C3_FS_SCL_LCNT                                  I2C3_BASE + 0x0020
#define I2C3_HS_SCL_HCNT                                  I2C3_BASE + 0x0024
#define I2C3_HS_SCL_LCNT                                I2C3_BASE + 0x0028
#define I2C3_INTR_STAT                                    I2C3_BASE + 0x002C
#define I2C3_INTR_EN                                     I2C3_BASE + 0x0030
#define I2C3_RAW_INTR_STAT                                I2C3_BASE + 0x0034
#define I2C3_RX_TL                                        I2C3_BASE + 0x0038
#define I2C3_TX_TL                                        I2C3_BASE + 0x003C
#define I2C3_CLR_INTR                                     I2C3_BASE + 0x0040
#define I2C3_CLR_RX_UNDER                                 I2C3_BASE + 0x0044
#define I2C3_CLR_RX_OVER                                  I2C3_BASE + 0x0048
#define I2C3_CLR_TX_OVER                                 I2C3_BASE + 0x004C
#define I2C3_CLR_TX_ABRT                                  I2C3_BASE + 0x0054
#define I2C3_CLR_ACTIVITY                                 I2C3_BASE + 0x005C
#define I2C3_CLR_STOP_DET                                 I2C3_BASE + 0x0060
#define I2C3_CLR_START_DET                               I2C3_BASE + 0x0064
#define I2C3_CLR_GEN_CALL                                 I2C3_BASE + 0x0068
#define I2C3_ENABLE                                       I2C3_BASE + 0x006C
#define I2C3_STATUS                                       I2C3_BASE + 0x0070
#define I2C3_TXFLR                                        I2C3_BASE + 0x0074
#define I2C3_RXFLR                                        I2C3_BASE + 0x0078
#define I2C3_SDA_HOLD                                     I2C3_BASE + 0x007C
#define I2C3_TX_ABRT_SOURCE                               I2C3_BASE + 0x0080
#define I2C3_FS_SPKLEN                                  I2C3_BASE + 0x00A0
#define I2C3_HS_SPKLEN                                    I2C3_BASE + 0x00A4

/*******************************************************************************
PWM registers' address on LC1860
********************************************************************************/
#define PWM0_EN                                           PWM_BASE + 0x0000
#define PWM0_UP                                           PWM_BASE + 0x0004
#define PWM0_RST                                          PWM_BASE + 0x0008
#define PWM0_P                                            PWM_BASE + 0x000C
#define PWM0_OCPY                                         PWM_BASE + 0x0010
#define PWM1_EN                                           PWM_BASE + 0x0040
#define PWM1_UP                                           PWM_BASE + 0x0044
#define PWM1_RST                                          PWM_BASE + 0x0048
#define PWM1_P                                            PWM_BASE + 0x004C
#define PWM1_OCPY                                         PWM_BASE + 0x0050

/*******************************************************************************
AP_PWR registers' address on LC1860
********************************************************************************/
/*Ë¯ÃßÏà¹Ø¼Ä´æÆ÷*/
#define AP_PWR_SLPCTL0                                    AP_PWR_BASE + 0x0000
#define AP_PWR_SLPCTL1                                   AP_PWR_BASE + 0x0004
#define AP_PWR_SLPCNT_LIMIT                               AP_PWR_BASE + 0x0008
#define AP_PWR_SLPST                                      AP_PWR_BASE + 0x000C
#define AP_PWR_PLLCR                                     AP_PWR_BASE + 0x0010
/*Ê±ÖÓÏà¹Ø¼Ä´æÆ÷*/
#define AP_PWR_PLL0CFG_CTL0                              AP_PWR_BASE + 0x0030
#define AP_PWR_PLL1CFG_CTL                               AP_PWR_BASE + 0x0034
#define AP_PWR_PLL1MCLK_CTL                              AP_PWR_BASE + 0x0038
#define AP_PWR_PLL0_F_UPD_CTL                           AP_PWR_BASE + 0x003C
#define AP_PWR_HA7_CLK_CTL                              AP_PWR_BASE + 0x0040
#define AP_PWR_PLL0CFG_CTL1                               AP_PWR_BASE + 0x0044
#define AP_PWR_PLL1CFG_CTL1                               AP_PWR_BASE + 0x0048
#define AP_PWR_BUSMCLK0_CTL                              AP_PWR_BASE + 0x004C
#define AP_PWR_BUSMCLK1_CTL                               AP_PWR_BASE + 0x0050
#define AP_PWR_CTLPCLK_CTL                               AP_PWR_BASE + 0x0054
#define AP_PWR_DATAPCLK_CTL                               AP_PWR_BASE + 0x0058
#define AP_PWR_SECPCLK_CTL                                AP_PWR_BASE + 0x005C
#define AP_PWR_CLK_EN0                                    AP_PWR_BASE + 0x0060
#define AP_PWR_CLK_EN1                                    AP_PWR_BASE + 0x0064
#define AP_PWR_CLK_EN2                                    AP_PWR_BASE + 0x0068
#define AP_PWR_CLK_EN3                                    AP_PWR_BASE + 0x006C
#define AP_PWR_CLK_EN4                                    AP_PWR_BASE + 0x0070
#define AP_PWR_CLK_EN5                                    AP_PWR_BASE + 0x0074
#define AP_PWR_CTLPCLK_EN                                 AP_PWR_BASE + 0x0080
#define AP_PWR_DATAPCLK_EN                                AP_PWR_BASE + 0x0084
#define AP_PWR_SECPCLK_EN                                 AP_PWR_BASE + 0x0088
#define AP_PWR_SA7CLK_CTL0                                AP_PWR_BASE + 0x0090
#define AP_PWR_SA7CLK_CTL1                                AP_PWR_BASE + 0x0094
#define AP_PWR_GPU_CLK_CTL                                AP_PWR_BASE + 0x009C
#define AP_PWR_ON2CLK_CTL0                                AP_PWR_BASE + 0x00A4
#define AP_PWR_ON2CLK_CTL1                                AP_PWR_BASE + 0x00A8
#define AP_PWR_ACC2DMCLK_CTL                              AP_PWR_BASE + 0x00B0
#define AP_PWR_NFCCLK_CTL                                 AP_PWR_BASE + 0x00B4
#define AP_PWR_HPICLK_CTL                                 AP_PWR_BASE + 0x00B8
#define AP_PWR_USBCLKDIV_CTL                              AP_PWR_BASE + 0x00BC
#define AP_PWR_LCDCAXICLK_CTL                             AP_PWR_BASE + 0x00C0
#define AP_PWR_LCDC0CLK_CTL                               AP_PWR_BASE + 0x00C4
#define AP_PWR_LCDC1CLK_CTL                               AP_PWR_BASE + 0x00C8
#define AP_PWR_DISCLK_EN                                  AP_PWR_BASE + 0x00D0
#define AP_PWR_ISPCLK_CTL0                                AP_PWR_BASE + 0x00D4
#define AP_PWR_ISPCLK_CTL1                                AP_PWR_BASE + 0x00D8
#define AP_PWR_CTLAPBMCLK_EN                              AP_PWR_BASE + 0x00EC
#define AP_PWR_TIMER0CLKCTL                               AP_PWR_BASE + 0x00F0
#define AP_PWR_TIMER1CLKCTL                               AP_PWR_BASE + 0x00F4
#define AP_PWR_TIMER2CLKCTL                               AP_PWR_BASE + 0x00F8
#define AP_PWR_TIMER3CLKCTL                               AP_PWR_BASE + 0x00FC
#define AP_PWR_TIMER4CLKCTL                               AP_PWR_BASE + 0x0100
#define AP_PWR_TIMER5CLKCTL                               AP_PWR_BASE + 0x0104
#define AP_PWR_TIMER6CLKCTL                               AP_PWR_BASE + 0x0108
#define AP_PWR_TIMER7CLKCTL                               AP_PWR_BASE + 0x010C
#define AP_PWR_PWMCLKDIV_CTL                              AP_PWR_BASE + 0x0110
#define AP_PWR_I2SCLK_CTL0                                AP_PWR_BASE + 0x0114
#define AP_PWR_I2SCLK_CTL1                                AP_PWR_BASE + 0x0118
#define AP_PWR_SSICLK_CTL0                                AP_PWR_BASE + 0x0120
#define AP_PWR_SSICLKDIV_CTL1                             AP_PWR_BASE + 0x0124
#define AP_PWR_UARTCLK_CTL0                               AP_PWR_BASE + 0x0130
#define AP_PWR_UART0CLK_CTL1                              AP_PWR_BASE + 0x0134
#define AP_PWR_UART1CLK_CTL1                              AP_PWR_BASE + 0x0138
#define AP_PWR_UART2CLK_CTL1                              AP_PWR_BASE + 0x013C
#define AP_PWR_I2CCLK_CTL                                 AP_PWR_BASE + 0x0140
#define AP_PWR_SECAPBMCLK_EN                              AP_PWR_BASE + 0x0144
#define AP_PWR_SDMMCCLK_CTL0                              AP_PWR_BASE + 0x0148
#define AP_PWR_SDMMC0CLKCTL1                              AP_PWR_BASE + 0x014C
#define AP_PWR_SDMMC1CLKCTL1                              AP_PWR_BASE + 0x0150
#define AP_PWR_SDMMC2CLKCTL1                              AP_PWR_BASE + 0x0154
#define AP_PWR_32KCLK_EN                                  AP_PWR_BASE + 0x0158
#define AP_PWR_CLKOUTSEL                                  AP_PWR_BASE + 0x0160
#define AP_PWR_CLKOUT0CTL                                 AP_PWR_BASE + 0x0164
#define AP_PWR_CLKOUT1CTL                                 AP_PWR_BASE + 0x0168
#define AP_PWR_CLKOUT2CTL                                 AP_PWR_BASE + 0x016C

/*¸´Î»Ïà¹Ø¼Ä´æÆ÷*/
#define AP_PWR_SFRST_CTL                                  AP_PWR_BASE + 0x0180
#define AP_PWR_CHIPRSTN_CTL                               AP_PWR_BASE + 0x0184
#define AP_PWR_CP_RSTCTL                                  AP_PWR_BASE + 0x0188
#define AP_PWR_TOP_RSTCTL                                 AP_PWR_BASE + 0x018C
#define AP_PWR_HA7_RSTCTL0                                AP_PWR_BASE + 0x0190
#define AP_PWR_HA7_RSTCTL1                                AP_PWR_BASE + 0x0194
#define AP_PWR_SA7_RSTCTL0                                AP_PWR_BASE + 0x01A0
#define AP_PWR_SA7_RSTCTL1                                AP_PWR_BASE + 0x01A4
#define AP_PWR_MOD_RSTCTL0                                AP_PWR_BASE + 0x01B0
#define AP_PWR_MOD_RSTCTL1                                AP_PWR_BASE + 0x01B4
#define AP_PWR_MOD_RSTCTL2                                AP_PWR_BASE + 0x01B8
#define AP_PWR_MOD_RSTCTL3                                AP_PWR_BASE + 0x01BC
#define AP_PWR_GPU_RSTCTL                                 AP_PWR_BASE + 0x01C0
#define AP_PWR_VIDEO_RSTCTL                               AP_PWR_BASE + 0x01C4
#define AP_PWR_DSIISP_RSTCTL                              AP_PWR_BASE + 0x01C8
#define AP_PWR_USB_RSTCTL                                 AP_PWR_BASE + 0x01CC
#define AP_PWR_HBLK0_RSTCTL                               AP_PWR_BASE + 0x01D0
#define AP_PWR_WDT_RSTCTL                                 AP_PWR_BASE + 0x01D4
/*ÖÐ¶ÏÏà¹Ø¼Ä´æÆ÷*/
#define AP_PWR_INTR_FLAG0                                 AP_PWR_BASE + 0x01F0
#define AP_PWR_INTR_FLAG1                                 AP_PWR_BASE + 0x01F4
#define AP_PWR_INTR_FLAG2                                 AP_PWR_BASE + 0x01F8
#define AP_PWR_INT_RAW                                    AP_PWR_BASE + 0x0200
#define AP_PWR_INTEN_A7                                   AP_PWR_BASE + 0x0204
#define AP_PWR_INTST_A7                                   AP_PWR_BASE + 0x0208
/*µçÔ´¹ÜÀíÏà¹Ø¼Ä´æÆ÷*/
#define AP_PWR_PDFSM_ST0                                  AP_PWR_BASE + 0x0220
#define AP_PWR_PDFSM_ST1                                  AP_PWR_BASE + 0x0224
#define AP_PWR_PDFSM_ST2                                  AP_PWR_BASE + 0x0228
#define AP_PWR_HA7_SCU_PD_CTL                             AP_PWR_BASE + 0x0230
#define AP_PWR_HA7_C0_PD_CTL                              AP_PWR_BASE + 0x0234
#define AP_PWR_HA7_C1_PD_CTL                              AP_PWR_BASE + 0x0238
#define AP_PWR_HA7_C2_PD_CTL                              AP_PWR_BASE + 0x023C
#define AP_PWR_HA7_C3_PD_CTL                              AP_PWR_BASE + 0x0240
#define AP_PWR_SA7_SCU_PD_CTL                             AP_PWR_BASE + 0x0248
#define AP_PWR_SA7_C_PD_CTL                               AP_PWR_BASE + 0x024C
#define AP_PWR_ON2_PD_CTL                                 AP_PWR_BASE + 0x0250
#define AP_PWR_ACC2D_PD_CTL                               AP_PWR_BASE + 0x0258
#define AP_PWR_DISPLAY_PD_CTL                             AP_PWR_BASE + 0x025C
#define AP_PWR_ISP_PD_CTL                                 AP_PWR_BASE + 0x0260
#define AP_PWR_USB_PD_CTL                                 AP_PWR_BASE + 0x0264
#define AP_PWR_HSIC_PD_CTL                                AP_PWR_BASE + 0x0268
#define AP_PWR_HA7_SCU_PD_CNT1                            AP_PWR_BASE + 0x0270
#define AP_PWR_HA7_C_PD_CNT1                              AP_PWR_BASE + 0x0274
#define AP_PWR_SA7_SCU_PD_CNT1                            AP_PWR_BASE + 0x0278
#define AP_PWR_SA7_C_PD_CNT1                              AP_PWR_BASE + 0x027C
#define AP_PWR_ON2_PD_CNT1                                AP_PWR_BASE + 0x0280
#define AP_PWR_ACC2D_PD_CNT1                              AP_PWR_BASE + 0x0288
#define AP_PWR_DISPLAY_PD_CNT1                            AP_PWR_BASE + 0x028C
#define AP_PWR_ISP_PD_CNT1                                AP_PWR_BASE + 0x0290
#define AP_PWR_USB_PD_CNT1                                AP_PWR_BASE + 0x0294
#define AP_PWR_HSIC_C_PD_CNT1                             AP_PWR_BASE + 0x0298
#define AP_PWR_HA7_SCU_PD_CNT2                            AP_PWR_BASE + 0x02A0
#define AP_PWR_HA7_C_PD_CNT2                              AP_PWR_BASE + 0x02A4
#define AP_PWR_SA7_SCU_PD_CNT2                            AP_PWR_BASE + 0x02A8
#define AP_PWR_SA7_C_PD_CNT2                              AP_PWR_BASE + 0x02AC
#define AP_PWR_ON2_PD_CNT2                                AP_PWR_BASE + 0x02B0
#define AP_PWR_ACC2D_PD_CNT2                              AP_PWR_BASE + 0x02B8
#define AP_PWR_DISPLAY_PD_CNT2                            AP_PWR_BASE + 0x02BC
#define AP_PWR_ISP_PD_CNT2                                AP_PWR_BASE + 0x02C0
#define AP_PWR_USB_PD_CNT2                                AP_PWR_BASE + 0x02C4
#define AP_PWR_HSIC_C_PD_CNT2                             AP_PWR_BASE + 0x02C8
#define AP_PWR_HA7_SCU_PD_CNT3                            AP_PWR_BASE + 0x02D0
#define AP_PWR_HA7_C_PD_CNT3                              AP_PWR_BASE + 0x02D4
#define AP_PWR_SA7_SCU_PD_CNT3                            AP_PWR_BASE + 0x02D8
#define AP_PWR_SA7_C_PD_CNT3                              AP_PWR_BASE + 0x02DC
#define AP_PWR_ON2_PD_CNT3                                AP_PWR_BASE + 0x02E0
#define AP_PWR_ACC2D_PD_CNT3                              AP_PWR_BASE + 0x02E8
#define AP_PWR_DISPLAY_PD_CNT3                            AP_PWR_BASE + 0x02EC
#define AP_PWR_ISP_PD_CNT3                                AP_PWR_BASE + 0x02F0
#define AP_PWR_USB_PD_CNT3                                AP_PWR_BASE + 0x02F4
#define AP_PWR_HSIC_PD_CNT3                               AP_PWR_BASE + 0x02F8
#define AP_PWR_PD_TMCTL                                   AP_PWR_BASE + 0x0300
#define AP_PWR_PWEN_CTL                                   AP_PWR_BASE + 0x0308
#define AP_PWR_GPU_PD_CTL                                 AP_PWR_BASE + 0x030C
#define AP_PWR_GPU_PD_CNT1                                AP_PWR_BASE + 0x0310
#define AP_PWR_GPU_PD_CNT2                                AP_PWR_BASE + 0x0314
#define AP_PWR_GPU_PD_CNT3                                AP_PWR_BASE + 0x0318
/*ÆäËü¼Ä´æÆ÷*/
#define AP_PWR_BOOTCTL                                    AP_PWR_BASE + 0x0320
#define AP_PWR_RESERVED_REG                               AP_PWR_BASE + 0x0324
#define AP_PWR_DEVICE_ST                                  AP_PWR_BASE + 0x0328
#define AP_PWR_LP_CTL                                     AP_PWR_BASE + 0x0330
#define AP_PWR_APB_DFS_LIMIT                              AP_PWR_BASE + 0x0334
#define AP_PWR_TM32K_CTL                                  AP_PWR_BASE + 0x0340
#define AP_PWR_TM32K_INIT_VAL                             AP_PWR_BASE + 0x0344
#define AP_PWR_TM32K_CUR_VAL                              AP_PWR_BASE + 0x0348
#define AP_PWR_BUSLP_CTL                                  AP_PWR_BASE + 0x034C
#define AP_PWR_BUS_LP_EN0                                 AP_PWR_BASE + 0x0350
#define AP_PWR_BUS_LP_EN1                                 AP_PWR_BASE + 0x0354
#define AP_PWR_BUS_LP_EN2                                 AP_PWR_BASE + 0x0358
#define AP_PWR_TESTPIN_CTL                                AP_PWR_BASE + 0x0360
#define AP_PWR_ACCESS_ERR_CLR                             AP_PWR_BASE + 0x036C
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST0                      AP_PWR_BASE + 0x0370
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST1                      AP_PWR_BASE + 0x0374
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST2                      AP_PWR_BASE + 0x0378
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST3                      AP_PWR_BASE + 0x037C
/*ÆäËü¼Ä´æÆ÷*/
#define AP_PWR_BOOTCTL                                    AP_PWR_BASE + 0x0320
#define AP_PWR_RESERVED_REG                               AP_PWR_BASE + 0x0324
#define AP_PWR_DEVICE_ST                                  AP_PWR_BASE + 0x0328
#define AP_PWR_LP_CTL                                     AP_PWR_BASE + 0x0330
#define AP_PWR_APB_DFS_LIMIT                              AP_PWR_BASE + 0x0334
#define AP_PWR_TM32K_CTL                                  AP_PWR_BASE + 0x0340
#define AP_PWR_TM32K_INIT_VAL                             AP_PWR_BASE + 0x0344
#define AP_PWR_TM32K_CUR_VAL                              AP_PWR_BASE + 0x0348
#define AP_PWR_BUSLP_CTL                                  AP_PWR_BASE + 0x034C
#define AP_PWR_BUS_LP_EN0                                 AP_PWR_BASE + 0x0350
#define AP_PWR_BUS_LP_EN1                                 AP_PWR_BASE + 0x0354
#define AP_PWR_BUS_LP_EN2                                 AP_PWR_BASE + 0x0358
#define AP_PWR_TESTPIN_CTL                                AP_PWR_BASE + 0x0360
#define AP_PWR_ACCESS_ERR_CLR                             AP_PWR_BASE + 0x036C
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST0                      AP_PWR_BASE + 0x0370
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST1                      AP_PWR_BASE + 0x0374
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST2                      AP_PWR_BASE + 0x0378
#define AP_PWR_CP_PWR_LPCTRL_FSM_ST3                      AP_PWR_BASE + 0x037C

/*******************************************************************************
AP_WDT4 registers' address on LC1860
********************************************************************************/
#define AP_WDT4_CR                                        AP_WDT4_BASE + 0x0000
#define AP_WDT4_TORR                                      AP_WDT4_BASE + 0x0004
#define AP_WDT4_CCVR                                      AP_WDT4_BASE + 0x0008
#define AP_WDT4_CRR                                       AP_WDT4_BASE + 0x000C
#define AP_WDT4_STAT                                      AP_WDT4_BASE + 0x0010
#define AP_WDT4_ICR                                       AP_WDT4_BASE + 0x0014

/*******************************************************************************
I2C2 registers' address on LC1860
********************************************************************************/
#define I2C2_CON                                          I2C2_BASE + 0x0000
#define I2C2_TAR                                          I2C2_BASE + 0x0004
#define I2C2_HS_MADDR                                     I2C2_BASE + 0x000C
#define I2C2_DATA_CMD                                     I2C2_BASE + 0x0010
#define I2C2_SS_SCL_HCNT                                  I2C2_BASE + 0x0014
#define I2C2_SS_SCL_LCNT                                  I2C2_BASE + 0x0018
#define I2C2_FS_SCL_HCNT                                  I2C2_BASE + 0x001C
#define I2C2_FS_SCL_LCNT                                  I2C2_BASE + 0x0020
#define I2C2_HS_SCL_HCNT                                  I2C2_BASE + 0x0024
#define I2C2_HS_SCL_LCNT                                  I2C2_BASE + 0x0028
#define I2C2_INTR_STAT                                    I2C2_BASE + 0x002C
#define I2C2_INTR_EN                                      I2C2_BASE + 0x0030
#define I2C2_RAW_INTR_STAT                                I2C2_BASE + 0x0034
#define I2C2_RX_TL                                        I2C2_BASE + 0x0038
#define I2C2_TX_TL                                        I2C2_BASE + 0x003C
#define I2C2_CLR_INTR                                     I2C2_BASE + 0x0040
#define I2C2_CLR_RX_UNDER                                 I2C2_BASE + 0x0044
#define I2C2_CLR_RX_OVER                                  I2C2_BASE + 0x0048
#define I2C2_CLR_TX_OVER                                  I2C2_BASE + 0x004C
#define I2C2_CLR_TX_ABRT                                  I2C2_BASE + 0x0054
#define I2C2_CLR_ACTIVITY                                 I2C2_BASE + 0x005C
#define I2C2_CLR_STOP_DET                                 I2C2_BASE + 0x0060
#define I2C2_CLR_START_DET                                I2C2_BASE + 0x0064
#define I2C2_CLR_GEN_CALL                                 I2C2_BASE + 0x0068
#define I2C2_ENABLE                                       I2C2_BASE + 0x006C
#define I2C2_STATUS                                       I2C2_BASE + 0x0070
#define I2C2_TXFLR                                        I2C2_BASE + 0x0074
#define I2C2_RXFLR                                        I2C2_BASE + 0x0078
#define I2C2_SDA_HOLD                                     I2C2_BASE + 0x007C
#define I2C2_TX_ABRT_SOURCE                               I2C2_BASE + 0x0080
#define I2C2_FS_SPKLEN                                    I2C2_BASE + 0x00A0
#define I2C2_HS_SPKLEN                                    I2C2_BASE + 0x00A4

/*******************************************************************************
KBS registers' address on LC1860
********************************************************************************/
#define KBS_DIRKEY_CTL                                    KBS_BASE + 0x0004
#define KBS_MASK                                          KBS_BASE + 0x0008
#define KBS_DETECT_INTVAL                                 KBS_BASE + 0x000C
#define KBS_LPRS_INTVAL                                   KBS_BASE + 0x0014
#define KBS_DIRKEY_INT                                    KBS_BASE + 0x0034
#define KBS_DIRKEY_INT_RAW                                KBS_BASE + 0x003C
#define KBS_DIRKEY_INT_EN                                 KBS_BASE + 0x0044

/*******************************************************************************
SSI2 registers' address on LC1860
********************************************************************************/
#define SSI2_CTRL0                                        SSI2_BASE + 0x0000
#define SSI2_CTRL1                                        SSI2_BASE + 0x0004
#define SSI2_EN                                           SSI2_BASE + 0x0008
#define SSI2_SE                                           SSI2_BASE + 0x0010
#define SSI2_BAUD                                         SSI2_BASE + 0x0014
#define SSI2_TXFTL                                        SSI2_BASE + 0x0018
#define SSI2_RXFTL                                        SSI2_BASE + 0x001C
#define SSI2_TXFL                                         SSI2_BASE + 0x0020
#define SSI2_RXFL                                         SSI2_BASE + 0x0024
#define SSI2_STS                                          SSI2_BASE + 0x0028
#define SSI2_IE                                           SSI2_BASE + 0x002C
#define SSI2_IS                                           SSI2_BASE + 0x0030
#define SSI2_RIS                                          SSI2_BASE + 0x0034
#define SSI2_TXOIC                                        SSI2_BASE + 0x0038
#define SSI2_RXOIC                                        SSI2_BASE + 0x003C
#define SSI2_RXUIC                                        SSI2_BASE + 0x0040
#define SSI2_IC                                           SSI2_BASE + 0x0048
#define SSI2_DMAC                                         SSI2_BASE + 0x004C
#define SSI2_DMATDL                                       SSI2_BASE + 0x0050
#define SSI2_DMARDL                                       SSI2_BASE + 0x0054
#define SSI2_DATA                                         SSI2_BASE + 0x0060

/*******************************************************************************
BP147 registers' address on LC1860
********************************************************************************/
#define BP147_SR_SIZE                                     BP147_BASE + 0x0000
#define BP147_LCDC_SE_STAT                                BP147_BASE + 0x0800
#define BP147_LCDC_SE_SET                                 BP147_BASE + 0x0804
#define BP147_LCDC_SE_CLR                                 BP147_BASE + 0x0808
#define BP147_LCDC_LOCK_STAT                              BP147_BASE + 0x080C
#define BP147_LCDC_LOCK_SET                               BP147_BASE + 0x0810
#define BP147_LCDC_LOCK_CLR                               BP147_BASE + 0x0814
#define BP147_DMAG_SE_STAT                                BP147_BASE + 0x0818
#define BP147_DMAG_SE_SET                                 BP147_BASE + 0x081C
#define BP147_DMAG_SE_CLR                                 BP147_BASE + 0x0820
#define BP147_DMAG_LOCK_STAT                              BP147_BASE + 0x0824
#define BP147_DMAG_LOCK_SET                               BP147_BASE + 0x0828
#define BP147_DMAG_LOCK_CLR                               BP147_BASE + 0x082C

/*******************************************************************************
TPZCTL registers' address on LC1860
********************************************************************************/
#define TPZCTL_INTLV_RGN_CFG                              TPZCTL_BASE + 0x0000
#define TPZCTL_INTLV_RGN_UPDT                             TPZCTL_BASE + 0x0004
#define TPZCTL_INTLV_EN                                   TPZCTL_BASE + 0x0008
/*AP²àDDRÇøÓòÅäÖÃ*/
#define TPZCTL_AP_TPZ_DADDR                               TPZCTL_BASE + 0x000C
#define TPZCTL_AP_TPZ_CFG0                                TPZCTL_BASE + 0x0010
#define TPZCTL_AP_RGN0_EN                                 TPZCTL_BASE + 0x0014
#define TPZCTL_AP_RGN0_UPDT                               TPZCTL_BASE + 0x0018
#define TPZCTL_AP_TPZ_ATTR0                               TPZCTL_BASE + 0x001C
#define TPZCTL_AP_TPZ_CFG1                                TPZCTL_BASE + 0x0020
#define TPZCTL_AP_RGN1_EN                                 TPZCTL_BASE + 0x0024
#define TPZCTL_AP_RGN1_UPDT                               TPZCTL_BASE + 0x0028
#define TPZCTL_AP_TPZ_ATTR1                               TPZCTL_BASE + 0x002C
#define TPZCTL_AP_TPZ_CFG2                                TPZCTL_BASE + 0x0030
#define TPZCTL_AP_RGN2_EN                                 TPZCTL_BASE + 0x0034
#define TPZCTL_AP_RGN2_UPDT                               TPZCTL_BASE + 0x0038
#define TPZCTL_AP_TPZ_ATTR2                               TPZCTL_BASE + 0x003C
#define TPZCTL_AP_TPZ_CFG3                                TPZCTL_BASE + 0x0040
#define TPZCTL_AP_RGN3_EN                                 TPZCTL_BASE + 0x0044
#define TPZCTL_AP_RGN3_UPDT                               TPZCTL_BASE + 0x0048
#define TPZCTL_AP_TPZ_ATTR3                               TPZCTL_BASE + 0x004C
/*CP²àDDRÇøÓòÅäÖÃ*/
#define TPZCTL_CP_TPZ_CFG0                                TPZCTL_BASE + 0x0050
#define TPZCTL_CP_RGN0_EN                                 TPZCTL_BASE + 0x0054
#define TPZCTL_CP_RGN0_UPDT                               TPZCTL_BASE + 0x0058
#define TPZCTL_CP_TPZ_CFG1                                TPZCTL_BASE + 0x0060
#define TPZCTL_CP_RGN1_EN                                 TPZCTL_BASE + 0x0064
#define TPZCTL_CP_RGN1_UPDT                               TPZCTL_BASE + 0x0068
#define TPZCTL_CP_TPZ_CFG2                                TPZCTL_BASE + 0x0070
#define TPZCTL_CP_RGN2_EN                                 TPZCTL_BASE + 0x0074
#define TPZCTL_CP_RGN2_UPDT                               TPZCTL_BASE + 0x0078
#define TPZCTL_CP_TPZ_CFG3                                TPZCTL_BASE + 0x0080
#define TPZCTL_CP_RGN3_EN                                 TPZCTL_BASE + 0x0084
#define TPZCTL_CP_RGN3_UPDT                               TPZCTL_BASE + 0x0088
#define TPZCTL_CP_TPZ_DADDR                               TPZCTL_BASE + 0x008C
/*CPA7 DDRÇøÓòÅäÖÃ*/
#define TPZCTL_CPA7_TPZ_CFG0                              TPZCTL_BASE + 0x0090
#define TPZCTL_CPA7_RGN0_EN                               TPZCTL_BASE + 0x0094
#define TPZCTL_CPA7_RGN0_UPDT                             TPZCTL_BASE + 0x0098
#define TPZCTL_CPA7_TPZ_CFG1                              TPZCTL_BASE + 0x00A0
#define TPZCTL_CPA7_RGN1_EN                               TPZCTL_BASE + 0x00A4
#define TPZCTL_CPA7_RGN1_UPDT                             TPZCTL_BASE + 0x00A8
#define TPZCTL_CPA7_TPZ_CFG2                              TPZCTL_BASE + 0x00B0
#define TPZCTL_CPA7_RGN2_EN                               TPZCTL_BASE + 0x00B4
#define TPZCTL_CPA7_RGN2_UPDT                             TPZCTL_BASE + 0x00B8
#define TPZCTL_CPA7_TPZ_CFG3                              TPZCTL_BASE + 0x00C0
#define TPZCTL_CPA7_RGN3_EN                               TPZCTL_BASE + 0x00C4
#define TPZCTL_CPA7_RGN3_UPDT                             TPZCTL_BASE + 0x00C8
#define TPZCTL_CPA7_TPZ_DADDR                             TPZCTL_BASE + 0x00CC
/*TOP DDRÇøÓòÅäÖÃ*/
#define TPZCTL_TOP_TPZ_CFG0                               TPZCTL_BASE + 0x00D0
#define TPZCTL_TOP_RGN0_EN                                TPZCTL_BASE + 0x00D4
#define TPZCTL_TOP_RGN0_UPDT                              TPZCTL_BASE + 0x00D8
#define TPZCTL_TOP_TPZ_CFG1                               TPZCTL_BASE + 0x00E0
#define TPZCTL_TOP_RGN1_EN                                TPZCTL_BASE + 0x00E4
#define TPZCTL_TOP_RGN1_UPDT                              TPZCTL_BASE + 0x00E8
#define TPZCTL_TOP_TPZ_CFG2                               TPZCTL_BASE + 0x00F0
#define TPZCTL_TOP_RGN2_EN                                TPZCTL_BASE + 0x00F4
#define TPZCTL_TOP_RGN2_UPDT                              TPZCTL_BASE + 0x00F8
#define TPZCTL_TOP_TPZ_CFG3                               TPZCTL_BASE + 0x0100
#define TPZCTL_TOP_RGN3_EN                                TPZCTL_BASE + 0x0104
#define TPZCTL_TOP_RGN3_UPDT                              TPZCTL_BASE + 0x0108
#define TPZCTL_TOP_TPZ_DADDR                              TPZCTL_BASE + 0x010C
/*ÖÐ¶Ï¼Ä´æÆ÷*/
#define TPZCTL_INTRAW                                     TPZCTL_BASE + 0x0110
#define TPZCTL_INTE                                       TPZCTL_BASE + 0x0114
#define TPZCTL_INTS                                       TPZCTL_BASE + 0x0118

/*******************************************************************************
TOP_MAILBOX registers' address on LC1860
********************************************************************************/
/*CPUÖ®¼ä»¥·¢ÖÐ¶Ï¼Ä´æÆ÷*/
#define TOP_MAILBOX_CPA7_DSP_INTR_SET                     TOP_MAILBOX_BASE + 0x0000
#define TOP_MAILBOX_CPA7_INTR_EN                          TOP_MAILBOX_BASE + 0x0004
#define TOP_MAILBOX_CPA7_INTR_SRC_EN0                     TOP_MAILBOX_BASE + 0x0008
#define TOP_MAILBOX_CPA7_INTR_SRC_EN1                     TOP_MAILBOX_BASE + 0x000C
#define TOP_MAILBOX_CPA7_INTR_STA0                        TOP_MAILBOX_BASE + 0x0010
#define TOP_MAILBOX_CPA7_INTR_STA1                        TOP_MAILBOX_BASE + 0x0014
#define TOP_MAILBOX_X1643_INTR_EN                         TOP_MAILBOX_BASE + 0x0018
#define TOP_MAILBOX_X1643_INTR_SRC_EN0                    TOP_MAILBOX_BASE + 0x001C
#define TOP_MAILBOX_X1643_INTR_SRC_EN1                    TOP_MAILBOX_BASE + 0x0020
#define TOP_MAILBOX_X1643_INTR_STA0                       TOP_MAILBOX_BASE + 0x0024
#define TOP_MAILBOX_X1643_INTR_STA1                       TOP_MAILBOX_BASE + 0x0028
#define TOP_MAILBOX_CPA7_DSP_INTR_STA_RAW0                TOP_MAILBOX_BASE + 0x002C
#define TOP_MAILBOX_CPA7_DSP_INTR_STA_RAW1                TOP_MAILBOX_BASE + 0x0030
#define TOP_MAILBOX_CPA7_DSP_INTR_STA_RAW2                TOP_MAILBOX_BASE + 0x0034
#define TOP_MAILBOX_CPA7_DSP_INTR_STA_RAW3                TOP_MAILBOX_BASE + 0x0038
#define TOP_MAILBOX_APA7_INTR_SET                         TOP_MAILBOX_BASE + 0x0040
#define TOP_MAILBOX_APA7_INTR_EN                          TOP_MAILBOX_BASE + 0x0044
#define TOP_MAILBOX_APA7_INTR_SRC_EN0                     TOP_MAILBOX_BASE + 0x0048
#define TOP_MAILBOX_APA7_INTR_SRC_EN1                     TOP_MAILBOX_BASE + 0x004C
#define TOP_MAILBOX_APA7_INTR_STA0                        TOP_MAILBOX_BASE + 0x0050
#define TOP_MAILBOX_APA7_INTR_STA1                        TOP_MAILBOX_BASE + 0x0054
#define TOP_MAILBOX_APA7_INTR_STA_RAW0                    TOP_MAILBOX_BASE + 0x0058
#define TOP_MAILBOX_APA7_INTR_STA_RAW1                    TOP_MAILBOX_BASE + 0x005C
#define TOP_MAILBOX_APA7_INTR_STA_RAW2                    TOP_MAILBOX_BASE + 0x0060
#define TOP_MAILBOX_APA7_INTR_STA_RAW3                    TOP_MAILBOX_BASE + 0x0064
#define TOP_MAILBOX_TL420_INTR_SET                        TOP_MAILBOX_BASE + 0x0070
#define TOP_MAILBOX_TL420_INTR_EN                         TOP_MAILBOX_BASE + 0x0074
#define TOP_MAILBOX_TL420_INTR_STA                        TOP_MAILBOX_BASE + 0x0078
#define TOP_MAILBOX_TL420_INTR_STA_RAW                    TOP_MAILBOX_BASE + 0x007C
#define TOP_MAILBOX_XC4210_INTR_EN                        TOP_MAILBOX_BASE + 0x0200
#define TOP_MAILBOX_XC4210_INTR_SRC_EN0                   TOP_MAILBOX_BASE + 0x0204
#define TOP_MAILBOX_XC4210_INTR_SRC_EN1                   TOP_MAILBOX_BASE + 0x0208
#define TOP_MAILBOX_XC4210_INTR_STA0                      TOP_MAILBOX_BASE + 0x020C
#define TOP_MAILBOX_XC4210_INTR_STA1                      TOP_MAILBOX_BASE + 0x0210
/*×ÜÏßÓÅÏÈ¼¶¿ØÖÆ¼Ä´æÆ÷*/
#define TOP_MAILBOX_MASTER_PRIOR0                         TOP_MAILBOX_BASE + 0x0090
#define TOP_MAILBOX_MASTER_PRIOR1                         TOP_MAILBOX_BASE + 0x0094
#define TOP_MAILBOX_MASTER_PRIOR2                         TOP_MAILBOX_BASE + 0x0098
#define TOP_MAILBOX_MASTER_PRIOR3                         TOP_MAILBOX_BASE + 0x009C
#define TOP_MAILBOX_MASTER_PRIOR4                         TOP_MAILBOX_BASE + 0x00A0
#define TOP_MAILBOX_MASTER_PRIOR5                         TOP_MAILBOX_BASE + 0x00A4
#define TOP_MAILBOX_MASTER_PRIOR6                         TOP_MAILBOX_BASE + 0x00A8
#define TOP_MAILBOX_MASTER_PRIOR7                         TOP_MAILBOX_BASE + 0x00AC
#define TOP_MAILBOX_MASTER_PRIOR8                         TOP_MAILBOX_BASE + 0x00B0
#define TOP_MAILBOX_MASTER_PRIOR9                         TOP_MAILBOX_BASE + 0x00B4
#define TOP_MAILBOX_MASTER_PRIOR10                        TOP_MAILBOX_BASE + 0x00B8
#define TOP_MAILBOX_MASTER_PRIOR11                        TOP_MAILBOX_BASE + 0x00BC
#define TOP_MAILBOX_MASTER_PRIOR12                        TOP_MAILBOX_BASE + 0x0080
#define TOP_MAILBOX_MASTER_PRIOR13                        TOP_MAILBOX_BASE + 0x0084
#define TOP_MAILBOX_MASTER_PRIOR14                        TOP_MAILBOX_BASE + 0x0088
#define TOP_MAILBOX_MASTER_PRIOR15                        TOP_MAILBOX_BASE + 0x008C
#define TOP_MAILBOX_MASTER_PRIOR16                        TOP_MAILBOX_BASE + 0x00C0
#define TOP_MAILBOX_MASTER_PRIOR17                        TOP_MAILBOX_BASE + 0x00C4
/*A7 trigger¼Ä´æÆ÷*/
#define TOP_MAILBOX_CPA7_TRIG                             TOP_MAILBOX_BASE + 0x0120
#define TOP_MAILBOX_APA7_TRIG                             TOP_MAILBOX_BASE + 0x0124
/*Á÷Á¿¼à¿Ø×é¶¨Òå¼Ä´æÆ÷*/
#define TOP_MAILBOX_MASTER_GRP_DEF0                       TOP_MAILBOX_BASE + 0x00D0
#define TOP_MAILBOX_MASTER_GRP_DEF1                       TOP_MAILBOX_BASE + 0x00D4
#define TOP_MAILBOX_MASTER_GRP_DEF2                       TOP_MAILBOX_BASE + 0x00D8
#define TOP_MAILBOX_MASTER_GRP_DEF3                       TOP_MAILBOX_BASE + 0x00DC
#define TOP_MAILBOX_MASTER_GRP_DEF4                       TOP_MAILBOX_BASE + 0x00E0
#define TOP_MAILBOX_MASTER_GRP_DEF5                       TOP_MAILBOX_BASE + 0x00E4
#define TOP_MAILBOX_MASTER_GRP_DEF6                       TOP_MAILBOX_BASE + 0x00E8
#define TOP_MAILBOX_MASTER_GRP_DEF7                       TOP_MAILBOX_BASE + 0x00EC
#define TOP_MAILBOX_MASTER_GRP_DEF8                       TOP_MAILBOX_BASE + 0x00F0
#define TOP_MAILBOX_MASTER_GRP_DEF9                       TOP_MAILBOX_BASE + 0x00F4
#define TOP_MAILBOX_MASTER_GRP_DEF10                      TOP_MAILBOX_BASE + 0x00F8
#define TOP_MAILBOX_MASTER_GRP_DEF11                      TOP_MAILBOX_BASE + 0x00FC
#define TOP_MAILBOX_MASTER_GRP_DEF12                      TOP_MAILBOX_BASE + 0x0100
#define TOP_MAILBOX_MASTER_GRP_DEF13                      TOP_MAILBOX_BASE + 0x0104
#define TOP_MAILBOX_MASTER_GRP_DEF14                      TOP_MAILBOX_BASE + 0x0108
#define TOP_MAILBOX_MASTER_GRP_DEF15                      TOP_MAILBOX_BASE + 0x010C
#define TOP_MAILBOX_MASTER_GRP_DEF16                      TOP_MAILBOX_BASE + 0x0110
#define TOP_MAILBOX_MASTER_GRP_DEF17                      TOP_MAILBOX_BASE + 0x0114
/*µÍ¹¦ºÄ¿ØÖÆ¶Ï¼Ä´æÆ÷*/
#define TOP_MAILBOX_LP_MODE_CTRL                          TOP_MAILBOX_BASE + 0x0100
/*ÖÙ²ÃÆ÷¿ØÖÆ¼Ä´æÆ÷*/
#define TOP_MAILBOX_ARBITER_EN                            TOP_MAILBOX_BASE + 0x0130
#define TOP_MAILBOX_CPU0_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x0140
#define TOP_MAILBOX_CPU0_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x0144
#define TOP_MAILBOX_CPU0_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x0148
#define TOP_MAILBOX_CPU0_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x014C
#define TOP_MAILBOX_CPU1_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x0150
#define TOP_MAILBOX_CPU1_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x0154
#define TOP_MAILBOX_CPU1_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x0158
#define TOP_MAILBOX_CPU1_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x015C
#define TOP_MAILBOX_CPU2_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x0160
#define TOP_MAILBOX_CPU2_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x0164
#define TOP_MAILBOX_CPU2_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x0168
#define TOP_MAILBOX_CPU2_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x016C
#define TOP_MAILBOX_CPU3_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x0170
#define TOP_MAILBOX_CPU3_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x0174
#define TOP_MAILBOX_CPU3_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x0178
#define TOP_MAILBOX_CPU3_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x017C
#define TOP_MAILBOX_CPU4_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x0180
#define TOP_MAILBOX_CPU4_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x0184
#define TOP_MAILBOX_CPU4_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x0188
#define TOP_MAILBOX_CPU4_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x018C
#define TOP_MAILBOX_CPU5_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x0190
#define TOP_MAILBOX_CPU5_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x0194
#define TOP_MAILBOX_CPU5_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x0198
#define TOP_MAILBOX_CPU5_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x019C
#define TOP_MAILBOX_CPU6_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x01A0
#define TOP_MAILBOX_CPU6_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x01A4
#define TOP_MAILBOX_CPU6_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x01A8
#define TOP_MAILBOX_CPU6_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x01AC
#define TOP_MAILBOX_CPU7_ARBITER_REQ0                     TOP_MAILBOX_BASE + 0x01B0
#define TOP_MAILBOX_CPU7_ARBITER_REQ1                     TOP_MAILBOX_BASE + 0x01B4
#define TOP_MAILBOX_CPU7_ARBITER_REQ2                     TOP_MAILBOX_BASE + 0x01B8
#define TOP_MAILBOX_CPU7_ARBITER_REQ3                     TOP_MAILBOX_BASE + 0x01BC
/*TL420¿ØÖÆ¼Ä´æÆ÷*/
#define TOP_MAILBOX_TL420_STA                             TOP_MAILBOX_BASE + 0x01E0
#define TOP_MAILBOX_TL420_EXCEPT_INTR_EN                  TOP_MAILBOX_BASE + 0x01E4
#define TOP_MAILBOX_TL420_EXCEPT_INTR_STA                 TOP_MAILBOX_BASE + 0x01E8
#define TOP_MAILBOX_TL420_EXCEPT_INTR_RAW                 TOP_MAILBOX_BASE + 0x01EC
#define TOP_MAILBOX_TL420_EXT_BP                          TOP_MAILBOX_BASE + 0x01F0
#define TOP_MAILBOX_TL420_CTRL                            TOP_MAILBOX_BASE + 0x01F4
/*HSL FIFO½Ó¿Ú*/
#define TOP_MAILBOX_HSL_FIFO_DATA                         TOP_MAILBOX_BASE + 0x01F8
#define TOP_MAILBOX_HSL_FIFO_STA                          TOP_MAILBOX_BASE + 0x01FC
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define TOP_MAILBOX_GEN_REG0                              TOP_MAILBOX_BASE + 0x0220
#define TOP_MAILBOX_GEN_REG1                              TOP_MAILBOX_BASE + 0x0224
#define TOP_MAILBOX_GEN_REG2                              TOP_MAILBOX_BASE + 0x0228
#define TOP_MAILBOX_GEN_REG3                              TOP_MAILBOX_BASE + 0x022C
#define TOP_MAILBOX_GEN_REG4                              TOP_MAILBOX_BASE + 0x0230
#define TOP_MAILBOX_GEN_REG5                              TOP_MAILBOX_BASE + 0x0234
#define TOP_MAILBOX_GEN_REG6                              TOP_MAILBOX_BASE + 0x0238
#define TOP_MAILBOX_GEN_REG7                              TOP_MAILBOX_BASE + 0x023C
/*RAM EMA¿ØÖÆ¼Ä´æÆ÷*/
#define TOP_MAILBOX_RAM_EMA_CTRL0                         TOP_MAILBOX_BASE + 0x0240
#define TOP_MAILBOX_RAM_EMA_CTRL1                         TOP_MAILBOX_BASE + 0x0244
#define TOP_MAILBOX_BUS_CACHE_CTRL                        TOP_MAILBOX_BASE + 0x0248
/*ÎÂ¶È¼ì²â¿ØÖÆ¼Ä´æÆ÷*/
#define TOP_MAILBOX_METS_CTRL                             TOP_MAILBOX_BASE + 0x0250
#define TOP_MAILBOX_TEMP_THRES_CFG                        TOP_MAILBOX_BASE + 0x0254
#define TOP_MAILBOX_TEMP_MON_EN                           TOP_MAILBOX_BASE + 0x0258
#define TOP_MAILBOX_TEMP_MON_INTR_EN                      TOP_MAILBOX_BASE + 0x025C
#define TOP_MAILBOX_TEMP_MON_INTR                         TOP_MAILBOX_BASE + 0x0260
#define TOP_MAILBOX_TEMP_MON_INTR_RAW                     TOP_MAILBOX_BASE + 0x0264
#define TOP_MAILBOX_TEMP_VALUE                            TOP_MAILBOX_BASE + 0x0268
/*DDRÖÐ¶Ï¿ØÖÆ¼Ä´æÆ÷*/
#define TOP_MAILBOX_DDR2CPA7_INTR_EN                      TOP_MAILBOX_BASE + 0x0270
#define TOP_MAILBOX_DDR2CPA7_INTR                         TOP_MAILBOX_BASE + 0x0274
#define TOP_MAILBOX_DDR_INTR_RAW                          TOP_MAILBOX_BASE + 0x0278
#define TOP_MAILBOX_DDR2X1643_INTR_EN                     TOP_MAILBOX_BASE + 0x0280
#define TOP_MAILBOX_DDR2X1643_INTR                        TOP_MAILBOX_BASE + 0x0284
#define TOP_MAILBOX_DDR2XC4210_INTR_EN                    TOP_MAILBOX_BASE + 0x0288
#define TOP_MAILBOX_DDR2XC4210_INTR                       TOP_MAILBOX_BASE + 0x028C
#define TOP_MAILBOX_TOP2CPA7_INTR_EN                      TOP_MAILBOX_BASE + 0x0290
#define TOP_MAILBOX_TOP2CPA7_INTR                         TOP_MAILBOX_BASE + 0x0294
#define TOP_MAILBOX_COM_UART2X1643_INTR_EN                TOP_MAILBOX_BASE + 0x0298
#define TOP_MAILBOX_COM_UART2X1643_INTR                   TOP_MAILBOX_BASE + 0x029C
#define TOP_MAILBOX_COM_UART2XC4210_INTR_EN               TOP_MAILBOX_BASE + 0x02A0
#define TOP_MAILBOX_COM_UART2XC4210_INTR                  TOP_MAILBOX_BASE + 0x02A4

/*******************************************************************************
MEMCTL0 registers' address on LC1860
********************************************************************************/
#if 0
#define MEMCTL0_DENALI_CTL_00                             MEMCTL0_BASE + 0x0000
#define MEMCTL0_DENALI_CTL_04                             MEMCTL0_BASE + 0x0010
#define MEMCTL0_DENALI_CTL_05                             MEMCTL0_BASE + 0x0014
#define MEMCTL0_DENALI_CTL_06                             MEMCTL0_BASE + 0x0018
#define MEMCTL0_DENALI_CTL_07                             MEMCTL0_BASE + 0x001C
#define MEMCTL0_DENALI_CTL_08                             MEMCTL0_BASE + 0x0020
#define MEMCTL0_DENALI_CTL_09                             MEMCTL0_BASE + 0x0024
#define MEMCTL0_DENALI_CTL_10                             MEMCTL0_BASE + 0x0028
#define MEMCTL0_DENALI_CTL_11                             MEMCTL0_BASE + 0x002C
#define MEMCTL0_DENALI_CTL_12                             MEMCTL0_BASE + 0x0030
#define MEMCTL0_DENALI_CTL_13                             MEMCTL0_BASE + 0x0034
#define MEMCTL0_DENALI_CTL_14                             MEMCTL0_BASE + 0x0038
#define MEMCTL0_DENALI_CTL_15                             MEMCTL0_BASE + 0x003C
#define MEMCTL0_DENALI_CTL_16                             MEMCTL0_BASE + 0x0040
#define MEMCTL0_DENALI_CTL_17                             MEMCTL0_BASE + 0x0044
#define MEMCTL0_DENALI_CTL_18                             MEMCTL0_BASE + 0x0048
#define MEMCTL0_DENALI_CTL_19                             MEMCTL0_BASE + 0x004C
#define MEMCTL0_DENALI_CTL_20                             MEMCTL0_BASE + 0x0050
#define MEMCTL0_DENALI_CTL_21                             MEMCTL0_BASE + 0x0054
#define MEMCTL0_DENALI_CTL_22                             MEMCTL0_BASE + 0x0058
#define MEMCTL0_DENALI_CTL_23                             MEMCTL0_BASE + 0x005C
#define MEMCTL0_DENALI_CTL_24                             MEMCTL0_BASE + 0x0060
#define MEMCTL0_DENALI_CTL_25                             MEMCTL0_BASE + 0x0064
#define MEMCTL0_DENALI_CTL_26                             MEMCTL0_BASE + 0x0068
#define MEMCTL0_DENALI_CTL_27                             MEMCTL0_BASE + 0x006C
#define MEMCTL0_DENALI_CTL_28                             MEMCTL0_BASE + 0x0070
#define MEMCTL0_DENALI_CTL_29                             MEMCTL0_BASE + 0x0074
#define MEMCTL0_DENALI_CTL_30                             MEMCTL0_BASE + 0x0078
#define MEMCTL0_DENALI_CTL_31                             MEMCTL0_BASE + 0x007C
#define MEMCTL0_DENALI_CTL_32                             MEMCTL0_BASE + 0x0080
#define MEMCTL0_DENALI_CTL_33                             MEMCTL0_BASE + 0x0084
#define MEMCTL0_DENALI_CTL_34                             MEMCTL0_BASE + 0x0088
#define MEMCTL0_DENALI_CTL_35                             MEMCTL0_BASE + 0x008C
#define MEMCTL0_DENALI_CTL_36                             MEMCTL0_BASE + 0x0090
#define MEMCTL0_DENALI_CTL_37                             MEMCTL0_BASE + 0x0094
#define MEMCTL0_DENALI_CTL_38                             MEMCTL0_BASE + 0x0098
#define MEMCTL0_DENALI_CTL_39                             MEMCTL0_BASE + 0x009C
#define MEMCTL0_DENALI_CTL_40                             MEMCTL0_BASE + 0x00A0
#define MEMCTL0_DENALI_CTL_41                             MEMCTL0_BASE + 0x00A4
#define MEMCTL0_DENALI_CTL_42                             MEMCTL0_BASE + 0x00A8
#define MEMCTL0_DENALI_CTL_43                             MEMCTL0_BASE + 0x00AC
#define MEMCTL0_DENALI_CTL_44                             MEMCTL0_BASE + 0x00B0
#define MEMCTL0_DENALI_CTL_45                             MEMCTL0_BASE + 0x00B4
#define MEMCTL0_DENALI_CTL_46                             MEMCTL0_BASE + 0x00B8
#define MEMCTL0_DENALI_CTL_47                             MEMCTL0_BASE + 0x00BC
#define MEMCTL0_DENALI_CTL_48                             MEMCTL0_BASE + 0x00C0
#define MEMCTL0_DENALI_CTL_49                             MEMCTL0_BASE + 0x00C4
#define MEMCTL0_DENALI_CTL_50                             MEMCTL0_BASE + 0x00C8
#define MEMCTL0_DENALI_CTL_51                             MEMCTL0_BASE + 0x00CC
#define MEMCTL0_DENALI_CTL_52                             MEMCTL0_BASE + 0x00D0
#define MEMCTL0_DENALI_CTL_53                             MEMCTL0_BASE + 0x00D4
#define MEMCTL0_DENALI_CTL_54                             MEMCTL0_BASE + 0x00D8
#define MEMCTL0_DENALI_CTL_55                             MEMCTL0_BASE + 0x00DC
#define MEMCTL0_DENALI_CTL_56                             MEMCTL0_BASE + 0x00E0
#define MEMCTL0_DENALI_CTL_57                             MEMCTL0_BASE + 0x00E4
#define MEMCTL0_DENALI_CTL_58                             MEMCTL0_BASE + 0x00E8
#define MEMCTL0_DENALI_CTL_59                             MEMCTL0_BASE + 0x00EC
#define MEMCTL0_DENALI_CTL_60                             MEMCTL0_BASE + 0x00F0
#define MEMCTL0_DENALI_CTL_61                             MEMCTL0_BASE + 0x00F4
#define MEMCTL0_DENALI_CTL_62                             MEMCTL0_BASE + 0x00F8
#define MEMCTL0_DENALI_CTL_63                             MEMCTL0_BASE + 0x00FC
#define MEMCTL0_DENALI_CTL_64                             MEMCTL0_BASE + 0x0100
#define MEMCTL0_DENALI_CTL_65                             MEMCTL0_BASE + 0x0104
#define MEMCTL0_DENALI_CTL_66                             MEMCTL0_BASE + 0x0108
#define MEMCTL0_DENALI_CTL_67                             MEMCTL0_BASE + 0x010C
#define MEMCTL0_DENALI_CTL_68                             MEMCTL0_BASE + 0x0110
#define MEMCTL0_DENALI_CTL_69                             MEMCTL0_BASE + 0x0114
#define MEMCTL0_DENALI_CTL_70                             MEMCTL0_BASE + 0x0118
#define MEMCTL0_DENALI_CTL_71                             MEMCTL0_BASE + 0x011C
#define MEMCTL0_DENALI_CTL_72                             MEMCTL0_BASE + 0x0120
#define MEMCTL0_DENALI_CTL_73                             MEMCTL0_BASE + 0x0124
#define MEMCTL0_DENALI_CTL_74                             MEMCTL0_BASE + 0x0128
#define MEMCTL0_DENALI_CTL_75                             MEMCTL0_BASE + 0x012C
#define MEMCTL0_DENALI_CTL_76                             MEMCTL0_BASE + 0x0130
#define MEMCTL0_DENALI_CTL_77                             MEMCTL0_BASE + 0x0134
#define MEMCTL0_DENALI_CTL_78                             MEMCTL0_BASE + 0x0138
#define MEMCTL0_DENALI_CTL_79                             MEMCTL0_BASE + 0x013C
#define MEMCTL0_DENALI_CTL_80                             MEMCTL0_BASE + 0x0140
#define MEMCTL0_DENALI_CTL_81                             MEMCTL0_BASE + 0x0144
#define MEMCTL0_DENALI_CTL_82                             MEMCTL0_BASE + 0x0148
#define MEMCTL0_DENALI_CTL_83                             MEMCTL0_BASE + 0x014C
#define MEMCTL0_DENALI_CTL_84                             MEMCTL0_BASE + 0x0150
#define MEMCTL0_DENALI_CTL_85                             MEMCTL0_BASE + 0x0154
#define MEMCTL0_DENALI_CTL_86                             MEMCTL0_BASE + 0x0158
#define MEMCTL0_DENALI_CTL_87                             MEMCTL0_BASE + 0x015C
#define MEMCTL0_DENALI_CTL_88                             MEMCTL0_BASE + 0x0160
#define MEMCTL0_DENALI_CTL_89                             MEMCTL0_BASE + 0x0164
#define MEMCTL0_DENALI_CTL_91                             MEMCTL0_BASE + 0x016C
#define MEMCTL0_DENALI_CTL_92                             MEMCTL0_BASE + 0x0170
#define MEMCTL0_DENALI_CTL_93                             MEMCTL0_BASE + 0x0174
#define MEMCTL0_DENALI_CTL_94                             MEMCTL0_BASE + 0x0178
#define MEMCTL0_DENALI_CTL_95                             MEMCTL0_BASE + 0x017C
#define MEMCTL0_DENALI_CTL_96                             MEMCTL0_BASE + 0x0180
#define MEMCTL0_DENALI_CTL_97                             MEMCTL0_BASE + 0x0184
#define MEMCTL0_DENALI_CTL_98                             MEMCTL0_BASE + 0x0188
#define MEMCTL0_DENALI_CTL_99                             MEMCTL0_BASE + 0x018C
#define MEMCTL0_DENALI_CTL_100                            MEMCTL0_BASE + 0x0190
#define MEMCTL0_DENALI_CTL_101                            MEMCTL0_BASE + 0x0194
#define MEMCTL0_DENALI_CTL_102                            MEMCTL0_BASE + 0x0198
#define MEMCTL0_DENALI_CTL_103                            MEMCTL0_BASE + 0x019C
#define MEMCTL0_DENALI_CTL_104                            MEMCTL0_BASE + 0x01A0
#define MEMCTL0_DENALI_CTL_105                            MEMCTL0_BASE + 0x01A4
#define MEMCTL0_DENALI_CTL_106                            MEMCTL0_BASE + 0x01A8
#define MEMCTL0_DENALI_CTL_107                            MEMCTL0_BASE + 0x01AC
#define MEMCTL0_DENALI_CTL_108                            MEMCTL0_BASE + 0x01B0
#define MEMCTL0_DENALI_CTL_109                            MEMCTL0_BASE + 0x01B4
#define MEMCTL0_DENALI_CTL_110                            MEMCTL0_BASE + 0x01B8
#define MEMCTL0_DENALI_CTL_111                            MEMCTL0_BASE + 0x01BC
#define MEMCTL0_DENALI_CTL_112                            MEMCTL0_BASE + 0x01C0
#define MEMCTL0_DENALI_CTL_113                            MEMCTL0_BASE + 0x01C4
#define MEMCTL0_DENALI_CTL_114                            MEMCTL0_BASE + 0x01C8
#define MEMCTL0_DENALI_CTL_115                            MEMCTL0_BASE + 0x01CC
#define MEMCTL0_DENALI_CTL_116                            MEMCTL0_BASE + 0x01D0
#define MEMCTL0_DENALI_CTL_117                            MEMCTL0_BASE + 0x01D4
#define MEMCTL0_DENALI_CTL_119                            MEMCTL0_BASE + 0x01DC
#define MEMCTL0_DENALI_CTL_120                            MEMCTL0_BASE + 0x01E0
#define MEMCTL0_DENALI_CTL_121                            MEMCTL0_BASE + 0x01E4
#define MEMCTL0_DENALI_CTL_122                            MEMCTL0_BASE + 0x01E8
#define MEMCTL0_DENALI_CTL_123                            MEMCTL0_BASE + 0x01EC
#define MEMCTL0_DENALI_CTL_124                            MEMCTL0_BASE + 0x01F0
#define MEMCTL0_DENALI_CTL_125                            MEMCTL0_BASE + 0x01F4
#define MEMCTL0_DENALI_CTL_126                            MEMCTL0_BASE + 0x01F8
#define MEMCTL0_DENALI_CTL_127                            MEMCTL0_BASE + 0x01FC
#define MEMCTL0_DENALI_CTL_128                            MEMCTL0_BASE + 0x0200
#define MEMCTL0_DENALI_CTL_136                            MEMCTL0_BASE + 0x0220
#define MEMCTL0_DENALI_CTL_137                            MEMCTL0_BASE + 0x0224
#define MEMCTL0_DENALI_CTL_138                            MEMCTL0_BASE + 0x0228
#define MEMCTL0_DENALI_CTL_139                            MEMCTL0_BASE + 0x022C
#define MEMCTL0_DENALI_CTL_140                            MEMCTL0_BASE + 0x0230
#define MEMCTL0_DENALI_CTL_141                            MEMCTL0_BASE + 0x0234
#define MEMCTL0_DENALI_CTL_142                            MEMCTL0_BASE + 0x0238
#define MEMCTL0_DENALI_CTL_144                            MEMCTL0_BASE + 0x0240
#define MEMCTL0_DENALI_CTL_145                            MEMCTL0_BASE + 0x0244
#define MEMCTL0_DENALI_CTL_146                            MEMCTL0_BASE + 0x0248
#define MEMCTL0_DENALI_CTL_147                            MEMCTL0_BASE + 0x024C
#define MEMCTL0_DENALI_CTL_149                            MEMCTL0_BASE + 0x0254
#define MEMCTL0_DENALI_CTL_150                            MEMCTL0_BASE + 0x0258
#define MEMCTL0_DENALI_CTL_151                            MEMCTL0_BASE + 0x025C
#define MEMCTL0_DENALI_CTL_152                            MEMCTL0_BASE + 0x0260
#define MEMCTL0_DENALI_CTL_153                            MEMCTL0_BASE + 0x0264
#define MEMCTL0_DENALI_CTL_154                            MEMCTL0_BASE + 0x0268
#define MEMCTL0_DENALI_CTL_155                            MEMCTL0_BASE + 0x026C
#define MEMCTL0_DENALI_CTL_156                            MEMCTL0_BASE + 0x0270
#define MEMCTL0_DENALI_CTL_157                            MEMCTL0_BASE + 0x0274
#define MEMCTL0_DENALI_CTL_158                            MEMCTL0_BASE + 0x0278
#define MEMCTL0_DENALI_CTL_159                            MEMCTL0_BASE + 0x027C
#define MEMCTL0_DENALI_CTL_161                            MEMCTL0_BASE + 0x0284
#define MEMCTL0_DENALI_CTL_162                            MEMCTL0_BASE + 0x0288
#define MEMCTL0_DENALI_CTL_163                            MEMCTL0_BASE + 0x028C
#define MEMCTL0_DENALI_CTL_164                            MEMCTL0_BASE + 0x0290
#define MEMCTL0_DENALI_CTL_165                            MEMCTL0_BASE + 0x0294
#define MEMCTL0_DENALI_CTL_166                            MEMCTL0_BASE + 0x0298
#define MEMCTL0_DENALI_CTL_167                            MEMCTL0_BASE + 0x029C
#define MEMCTL0_DENALI_CTL_168                            MEMCTL0_BASE + 0x02A0
#define MEMCTL0_DENALI_CTL_169                            MEMCTL0_BASE + 0x02A4
#endif

/*******************************************************************************
MEMCTL1 registers' address on LC1860
********************************************************************************/
#if 0
#define MEMCTL1_DENALI_CTL_00                             MEMCTL1_BASE + 0x0000
#define MEMCTL1_DENALI_CTL_04                             MEMCTL1_BASE + 0x0010
#define MEMCTL1_DENALI_CTL_05                             MEMCTL1_BASE + 0x0014
#define MEMCTL1_DENALI_CTL_06                             MEMCTL1_BASE + 0x0018
#define MEMCTL1_DENALI_CTL_07                             MEMCTL1_BASE + 0x001C
#define MEMCTL1_DENALI_CTL_08                             MEMCTL1_BASE + 0x0020
#define MEMCTL1_DENALI_CTL_09                             MEMCTL1_BASE + 0x0024
#define MEMCTL1_DENALI_CTL_10                             MEMCTL1_BASE + 0x0028
#define MEMCTL1_DENALI_CTL_11                             MEMCTL1_BASE + 0x002C
#define MEMCTL1_DENALI_CTL_12                             MEMCTL1_BASE + 0x0030
#define MEMCTL1_DENALI_CTL_13                             MEMCTL1_BASE + 0x0034
#define MEMCTL1_DENALI_CTL_14                             MEMCTL1_BASE + 0x0038
#define MEMCTL1_DENALI_CTL_15                             MEMCTL1_BASE + 0x003C
#define MEMCTL1_DENALI_CTL_16                             MEMCTL1_BASE + 0x0040
#define MEMCTL1_DENALI_CTL_17                             MEMCTL1_BASE + 0x0044
#define MEMCTL1_DENALI_CTL_18                             MEMCTL1_BASE + 0x0048
#define MEMCTL1_DENALI_CTL_19                             MEMCTL1_BASE + 0x004C
#define MEMCTL1_DENALI_CTL_20                             MEMCTL1_BASE + 0x0050
#define MEMCTL1_DENALI_CTL_21                             MEMCTL1_BASE + 0x0054
#define MEMCTL1_DENALI_CTL_22                             MEMCTL1_BASE + 0x0058
#define MEMCTL1_DENALI_CTL_23                             MEMCTL1_BASE + 0x005C
#define MEMCTL1_DENALI_CTL_24                             MEMCTL1_BASE + 0x0060
#define MEMCTL1_DENALI_CTL_25                             MEMCTL1_BASE + 0x0064
#define MEMCTL1_DENALI_CTL_26                             MEMCTL1_BASE + 0x0068
#define MEMCTL1_DENALI_CTL_27                             MEMCTL1_BASE + 0x006C
#define MEMCTL1_DENALI_CTL_28                             MEMCTL1_BASE + 0x0070
#define MEMCTL1_DENALI_CTL_29                             MEMCTL1_BASE + 0x0074
#define MEMCTL1_DENALI_CTL_30                             MEMCTL1_BASE + 0x0078
#define MEMCTL1_DENALI_CTL_31                             MEMCTL1_BASE + 0x007C
#define MEMCTL1_DENALI_CTL_32                             MEMCTL1_BASE + 0x0080
#define MEMCTL1_DENALI_CTL_33                             MEMCTL1_BASE + 0x0084
#define MEMCTL1_DENALI_CTL_34                             MEMCTL1_BASE + 0x0088
#define MEMCTL1_DENALI_CTL_35                             MEMCTL1_BASE + 0x008C
#define MEMCTL1_DENALI_CTL_36                             MEMCTL1_BASE + 0x0090
#define MEMCTL1_DENALI_CTL_37                             MEMCTL1_BASE + 0x0094
#define MEMCTL1_DENALI_CTL_38                             MEMCTL1_BASE + 0x0098
#define MEMCTL1_DENALI_CTL_39                             MEMCTL1_BASE + 0x009C
#define MEMCTL1_DENALI_CTL_40                             MEMCTL1_BASE + 0x00A0
#define MEMCTL1_DENALI_CTL_41                             MEMCTL1_BASE + 0x00A4
#define MEMCTL1_DENALI_CTL_42                             MEMCTL1_BASE + 0x00A8
#define MEMCTL1_DENALI_CTL_43                             MEMCTL1_BASE + 0x00AC
#define MEMCTL1_DENALI_CTL_44                             MEMCTL1_BASE + 0x00B0
#define MEMCTL1_DENALI_CTL_45                             MEMCTL1_BASE + 0x00B4
#define MEMCTL1_DENALI_CTL_46                             MEMCTL1_BASE + 0x00B8
#define MEMCTL1_DENALI_CTL_47                             MEMCTL1_BASE + 0x00BC
#define MEMCTL1_DENALI_CTL_48                             MEMCTL1_BASE + 0x00C0
#define MEMCTL1_DENALI_CTL_49                             MEMCTL1_BASE + 0x00C4
#define MEMCTL1_DENALI_CTL_50                             MEMCTL1_BASE + 0x00C8
#define MEMCTL1_DENALI_CTL_51                             MEMCTL1_BASE + 0x00CC
#define MEMCTL1_DENALI_CTL_52                             MEMCTL1_BASE + 0x00D0
#define MEMCTL1_DENALI_CTL_53                             MEMCTL1_BASE + 0x00D4
#define MEMCTL1_DENALI_CTL_54                             MEMCTL1_BASE + 0x00D8
#define MEMCTL1_DENALI_CTL_55                             MEMCTL1_BASE + 0x00DC
#define MEMCTL1_DENALI_CTL_56                             MEMCTL1_BASE + 0x00E0
#define MEMCTL1_DENALI_CTL_57                             MEMCTL1_BASE + 0x00E4
#define MEMCTL1_DENALI_CTL_58                             MEMCTL1_BASE + 0x00E8
#define MEMCTL1_DENALI_CTL_59                             MEMCTL1_BASE + 0x00EC
#define MEMCTL1_DENALI_CTL_60                             MEMCTL1_BASE + 0x00F0
#define MEMCTL1_DENALI_CTL_61                             MEMCTL1_BASE + 0x00F4
#define MEMCTL1_DENALI_CTL_62                             MEMCTL1_BASE + 0x00F8
#define MEMCTL1_DENALI_CTL_63                             MEMCTL1_BASE + 0x00FC
#define MEMCTL1_DENALI_CTL_64                             MEMCTL1_BASE + 0x0100
#define MEMCTL1_DENALI_CTL_65                             MEMCTL1_BASE + 0x0104
#define MEMCTL1_DENALI_CTL_66                             MEMCTL1_BASE + 0x0108
#define MEMCTL1_DENALI_CTL_67                             MEMCTL1_BASE + 0x010C
#define MEMCTL1_DENALI_CTL_68                             MEMCTL1_BASE + 0x0110
#define MEMCTL1_DENALI_CTL_69                             MEMCTL1_BASE + 0x0114
#define MEMCTL1_DENALI_CTL_70                             MEMCTL1_BASE + 0x0118
#define MEMCTL1_DENALI_CTL_71                             MEMCTL1_BASE + 0x011C
#define MEMCTL1_DENALI_CTL_72                             MEMCTL1_BASE + 0x0120
#define MEMCTL1_DENALI_CTL_73                             MEMCTL1_BASE + 0x0124
#define MEMCTL1_DENALI_CTL_74                             MEMCTL1_BASE + 0x0128
#define MEMCTL1_DENALI_CTL_75                             MEMCTL1_BASE + 0x012C
#define MEMCTL1_DENALI_CTL_76                             MEMCTL1_BASE + 0x0130
#define MEMCTL1_DENALI_CTL_77                             MEMCTL1_BASE + 0x0134
#define MEMCTL1_DENALI_CTL_78                             MEMCTL1_BASE + 0x0138
#define MEMCTL1_DENALI_CTL_79                             MEMCTL1_BASE + 0x013C
#define MEMCTL1_DENALI_CTL_80                             MEMCTL1_BASE + 0x0140
#define MEMCTL1_DENALI_CTL_81                             MEMCTL1_BASE + 0x0144
#define MEMCTL1_DENALI_CTL_82                             MEMCTL1_BASE + 0x0148
#define MEMCTL1_DENALI_CTL_83                             MEMCTL1_BASE + 0x014C
#define MEMCTL1_DENALI_CTL_84                             MEMCTL1_BASE + 0x0150
#define MEMCTL1_DENALI_CTL_85                             MEMCTL1_BASE + 0x0154
#define MEMCTL1_DENALI_CTL_86                             MEMCTL1_BASE + 0x0158
#define MEMCTL1_DENALI_CTL_87                             MEMCTL1_BASE + 0x015C
#define MEMCTL1_DENALI_CTL_88                             MEMCTL1_BASE + 0x0160
#define MEMCTL1_DENALI_CTL_89                             MEMCTL1_BASE + 0x0164
#define MEMCTL1_DENALI_CTL_91                             MEMCTL1_BASE + 0x016C
#define MEMCTL1_DENALI_CTL_92                             MEMCTL1_BASE + 0x0170
#define MEMCTL1_DENALI_CTL_93                             MEMCTL1_BASE + 0x0174
#define MEMCTL1_DENALI_CTL_94                             MEMCTL1_BASE + 0x0178
#define MEMCTL1_DENALI_CTL_95                             MEMCTL1_BASE + 0x017C
#define MEMCTL1_DENALI_CTL_96                             MEMCTL1_BASE + 0x0180
#define MEMCTL1_DENALI_CTL_97                             MEMCTL1_BASE + 0x0184
#define MEMCTL1_DENALI_CTL_98                             MEMCTL1_BASE + 0x0188
#define MEMCTL1_DENALI_CTL_99                             MEMCTL1_BASE + 0x018C
#define MEMCTL1_DENALI_CTL_100                            MEMCTL1_BASE + 0x0190
#define MEMCTL1_DENALI_CTL_101                            MEMCTL1_BASE + 0x0194
#define MEMCTL1_DENALI_CTL_102                            MEMCTL1_BASE + 0x0198
#define MEMCTL1_DENALI_CTL_103                            MEMCTL1_BASE + 0x019C
#define MEMCTL1_DENALI_CTL_104                            MEMCTL1_BASE + 0x01A0
#define MEMCTL1_DENALI_CTL_105                            MEMCTL1_BASE + 0x01A4
#define MEMCTL1_DENALI_CTL_106                            MEMCTL1_BASE + 0x01A8
#define MEMCTL1_DENALI_CTL_107                            MEMCTL1_BASE + 0x01AC
#define MEMCTL1_DENALI_CTL_108                            MEMCTL1_BASE + 0x01B0
#define MEMCTL1_DENALI_CTL_109                            MEMCTL1_BASE + 0x01B4
#define MEMCTL1_DENALI_CTL_110                            MEMCTL1_BASE + 0x01B8
#define MEMCTL1_DENALI_CTL_111                            MEMCTL1_BASE + 0x01BC
#define MEMCTL1_DENALI_CTL_112                            MEMCTL1_BASE + 0x01C0
#define MEMCTL1_DENALI_CTL_113                            MEMCTL1_BASE + 0x01C4
#define MEMCTL1_DENALI_CTL_114                            MEMCTL1_BASE + 0x01C8
#define MEMCTL1_DENALI_CTL_115                            MEMCTL1_BASE + 0x01CC
#define MEMCTL1_DENALI_CTL_116                            MEMCTL1_BASE + 0x01D0
#define MEMCTL1_DENALI_CTL_117                            MEMCTL1_BASE + 0x01D4
#define MEMCTL1_DENALI_CTL_119                            MEMCTL1_BASE + 0x01DC
#define MEMCTL1_DENALI_CTL_120                            MEMCTL1_BASE + 0x01E0
#define MEMCTL1_DENALI_CTL_121                            MEMCTL1_BASE + 0x01E4
#define MEMCTL1_DENALI_CTL_122                            MEMCTL1_BASE + 0x01E8
#define MEMCTL1_DENALI_CTL_123                            MEMCTL1_BASE + 0x01EC
#define MEMCTL1_DENALI_CTL_124                            MEMCTL1_BASE + 0x01F0
#define MEMCTL1_DENALI_CTL_125                            MEMCTL1_BASE + 0x01F4
#define MEMCTL1_DENALI_CTL_126                            MEMCTL1_BASE + 0x01F8
#define MEMCTL1_DENALI_CTL_127                            MEMCTL1_BASE + 0x01FC
#define MEMCTL1_DENALI_CTL_128                            MEMCTL1_BASE + 0x0200
#define MEMCTL1_DENALI_CTL_136                            MEMCTL1_BASE + 0x0220
#define MEMCTL1_DENALI_CTL_137                            MEMCTL1_BASE + 0x0224
#define MEMCTL1_DENALI_CTL_138                            MEMCTL1_BASE + 0x0228
#define MEMCTL1_DENALI_CTL_139                            MEMCTL1_BASE + 0x022C
#define MEMCTL1_DENALI_CTL_140                            MEMCTL1_BASE + 0x0230
#define MEMCTL1_DENALI_CTL_141                            MEMCTL1_BASE + 0x0234
#define MEMCTL1_DENALI_CTL_142                            MEMCTL1_BASE + 0x0238
#define MEMCTL1_DENALI_CTL_144                            MEMCTL1_BASE + 0x0240
#define MEMCTL1_DENALI_CTL_145                            MEMCTL1_BASE + 0x0244
#define MEMCTL1_DENALI_CTL_146                            MEMCTL1_BASE + 0x0248
#define MEMCTL1_DENALI_CTL_147                            MEMCTL1_BASE + 0x024C
#define MEMCTL1_DENALI_CTL_149                            MEMCTL1_BASE + 0x0254
#define MEMCTL1_DENALI_CTL_150                            MEMCTL1_BASE + 0x0258
#define MEMCTL1_DENALI_CTL_151                            MEMCTL1_BASE + 0x025C
#define MEMCTL1_DENALI_CTL_152                            MEMCTL1_BASE + 0x0260
#define MEMCTL1_DENALI_CTL_153                            MEMCTL1_BASE + 0x0264
#define MEMCTL1_DENALI_CTL_154                            MEMCTL1_BASE + 0x0268
#define MEMCTL1_DENALI_CTL_155                            MEMCTL1_BASE + 0x026C
#define MEMCTL1_DENALI_CTL_156                            MEMCTL1_BASE + 0x0270
#define MEMCTL1_DENALI_CTL_157                            MEMCTL1_BASE + 0x0274
#define MEMCTL1_DENALI_CTL_158                            MEMCTL1_BASE + 0x0278
#define MEMCTL1_DENALI_CTL_159                            MEMCTL1_BASE + 0x027C
#define MEMCTL1_DENALI_CTL_161                            MEMCTL1_BASE + 0x0284
#define MEMCTL1_DENALI_CTL_162                            MEMCTL1_BASE + 0x0288
#define MEMCTL1_DENALI_CTL_163                            MEMCTL1_BASE + 0x028C
#define MEMCTL1_DENALI_CTL_164                            MEMCTL1_BASE + 0x0290
#define MEMCTL1_DENALI_CTL_165                            MEMCTL1_BASE + 0x0294
#define MEMCTL1_DENALI_CTL_166                            MEMCTL1_BASE + 0x0298
#define MEMCTL1_DENALI_CTL_167                            MEMCTL1_BASE + 0x029C
#define MEMCTL1_DENALI_CTL_168                            MEMCTL1_BASE + 0x02A0
#define MEMCTL1_DENALI_CTL_169                            MEMCTL1_BASE + 0x02A4
#endif

/*******************************************************************************
TOP_DMAS registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define TOP_DMAS_EN                                       TOP_DMAS_BASE + 0x0000
#define TOP_DMAS_CLR                                      TOP_DMAS_BASE + 0x0004
#define TOP_DMAS_STA                                      TOP_DMAS_BASE + 0x0008
#define TOP_DMAS_INT_RAW0                                 TOP_DMAS_BASE + 0x000C
#define TOP_DMAS_INT_EN0_CP_A7                            TOP_DMAS_BASE + 0x0010
#define TOP_DMAS_INT_EN0_AP_A7                            TOP_DMAS_BASE + 0x0014
#define TOP_DMAS_INT_EN0_TL420                            TOP_DMAS_BASE + 0x0018
#define TOP_DMAS_INT0_CP_A7                               TOP_DMAS_BASE + 0x001C
#define TOP_DMAS_INT0_AP_A7                               TOP_DMAS_BASE + 0x0020
#define TOP_DMAS_INT0_TL420                               TOP_DMAS_BASE + 0x0024
#define TOP_DMAS_INT_CLR0                                 TOP_DMAS_BASE + 0x0028
#define TOP_DMAS_INT_RAW1                                 TOP_DMAS_BASE + 0x030C
#define TOP_DMAS_INT_EN1_CP_A7                            TOP_DMAS_BASE + 0x0310
#define TOP_DMAS_INT_EN1_AP_A7                            TOP_DMAS_BASE + 0x0314
#define TOP_DMAS_INT_EN1_TL420                            TOP_DMAS_BASE + 0x0318
#define TOP_DMAS_INT1_CP_A7                               TOP_DMAS_BASE + 0x031C
#define TOP_DMAS_INT1_AP_A7                               TOP_DMAS_BASE + 0x0320
#define TOP_DMAS_INT1_TL420                               TOP_DMAS_BASE + 0x0324
#define TOP_DMAS_INT_CLR1                                 TOP_DMAS_BASE + 0x0328
#define TOP_DMAS_INTV_UNIT                                TOP_DMAS_BASE + 0x002C
#define TOP_DMAS_LP_CTL                                   TOP_DMAS_BASE + 0x03FC
/*·¢ËÍÍ¨µÀ¼Ä´æÆ÷*/
#define TOP_DMAS_CH0_SAR                                  TOP_DMAS_BASE + 0x0040
#define TOP_DMAS_CH0_DAR                                  TOP_DMAS_BASE + 0x0044
#define TOP_DMAS_CH0_CTL0                                 TOP_DMAS_BASE + 0x0048
#define TOP_DMAS_CH0_CTL1                                 TOP_DMAS_BASE + 0x004C
#define TOP_DMAS_CH0_WD                                   TOP_DMAS_BASE + 0x0240
#define TOP_DMAS_CH0_CA                                   TOP_DMAS_BASE + 0x0050
#define TOP_DMAS_CH0_INTA                                 TOP_DMAS_BASE + 0x0054
#define TOP_DMAS_CH1_SAR                                  TOP_DMAS_BASE + 0x0060
#define TOP_DMAS_CH1_DAR                                  TOP_DMAS_BASE + 0x0064
#define TOP_DMAS_CH1_CTL0                                 TOP_DMAS_BASE + 0x0068
#define TOP_DMAS_CH1_CTL1                                 TOP_DMAS_BASE + 0x006C
#define TOP_DMAS_CH1_WD                                   TOP_DMAS_BASE + 0x0244
#define TOP_DMAS_CH1_CA                                   TOP_DMAS_BASE + 0x0070
#define TOP_DMAS_CH1_INTA                                 TOP_DMAS_BASE + 0x0074
#define TOP_DMAS_CH2_SAR                                  TOP_DMAS_BASE + 0x0080
#define TOP_DMAS_CH2_DAR                                  TOP_DMAS_BASE + 0x0084
#define TOP_DMAS_CH2_CTL0                                 TOP_DMAS_BASE + 0x0088
#define TOP_DMAS_CH2_CTL1                                 TOP_DMAS_BASE + 0x008C
#define TOP_DMAS_CH2_WD                                   TOP_DMAS_BASE + 0x0248
#define TOP_DMAS_CH2_CA                                   TOP_DMAS_BASE + 0x0090
#define TOP_DMAS_CH2_INTA                                 TOP_DMAS_BASE + 0x0094
#define TOP_DMAS_CH3_SAR                                  TOP_DMAS_BASE + 0x00A0
#define TOP_DMAS_CH3_DAR                                  TOP_DMAS_BASE + 0x00A4
#define TOP_DMAS_CH3_CTL0                                 TOP_DMAS_BASE + 0x00A8
#define TOP_DMAS_CH3_CTL1                                 TOP_DMAS_BASE + 0x00AC
#define TOP_DMAS_CH3_WD                                   TOP_DMAS_BASE + 0x024C
#define TOP_DMAS_CH3_CA                                   TOP_DMAS_BASE + 0x00B0
#define TOP_DMAS_CH3_INTA                                 TOP_DMAS_BASE + 0x00B4
#define TOP_DMAS_CH4_SAR                                  TOP_DMAS_BASE + 0x00C0
#define TOP_DMAS_CH4_DAR                                  TOP_DMAS_BASE + 0x00C4
#define TOP_DMAS_CH4_CTL0                                 TOP_DMAS_BASE + 0x00C8
#define TOP_DMAS_CH4_CTL1                                 TOP_DMAS_BASE + 0x00CC
#define TOP_DMAS_CH4_WD                                   TOP_DMAS_BASE + 0x0250
#define TOP_DMAS_CH4_CA                                   TOP_DMAS_BASE + 0x00D0
#define TOP_DMAS_CH4_INTA                                 TOP_DMAS_BASE + 0x00D4
#define TOP_DMAS_CH5_SAR                                  TOP_DMAS_BASE + 0x00E0
#define TOP_DMAS_CH5_DAR                                  TOP_DMAS_BASE + 0x00E4
#define TOP_DMAS_CH5_CTL0                                 TOP_DMAS_BASE + 0x00E8
#define TOP_DMAS_CH5_CTL1                                 TOP_DMAS_BASE + 0x00EC
#define TOP_DMAS_CH5_WD                                   TOP_DMAS_BASE + 0x0254
#define TOP_DMAS_CH5_CA                                   TOP_DMAS_BASE + 0x00F0
#define TOP_DMAS_CH5_INTA                                 TOP_DMAS_BASE + 0x00F4
#define TOP_DMAS_CH6_SAR                                  TOP_DMAS_BASE + 0x0100
#define TOP_DMAS_CH6_DAR                                  TOP_DMAS_BASE + 0x0104
#define TOP_DMAS_CH6_CTL0                                 TOP_DMAS_BASE + 0x0108
#define TOP_DMAS_CH6_CTL1                                 TOP_DMAS_BASE + 0x010C
#define TOP_DMAS_CH6_WD                                   TOP_DMAS_BASE + 0x0258
#define TOP_DMAS_CH6_CA                                   TOP_DMAS_BASE + 0x0110
#define TOP_DMAS_CH6_INTA                                 TOP_DMAS_BASE + 0x0114
#define TOP_DMAS_CH7_SAR                                  TOP_DMAS_BASE + 0x0120
#define TOP_DMAS_CH7_DAR                                  TOP_DMAS_BASE + 0x0124
#define TOP_DMAS_CH7_CTL0                                 TOP_DMAS_BASE + 0x0128
#define TOP_DMAS_CH7_CTL1                                 TOP_DMAS_BASE + 0x012C
#define TOP_DMAS_CH7_WD                                   TOP_DMAS_BASE + 0x025C
#define TOP_DMAS_CH7_CA                                   TOP_DMAS_BASE + 0x0130
#define TOP_DMAS_CH7_INTA                                 TOP_DMAS_BASE + 0x0134
/*½ÓÊÕÍ¨µÀ¼Ä´æÆ÷*/
#define TOP_DMAS_CH8_SAR                                  TOP_DMAS_BASE + 0x0140
#define TOP_DMAS_CH8_DAR                                  TOP_DMAS_BASE + 0x0144
#define TOP_DMAS_CH8_CTL0                                 TOP_DMAS_BASE + 0x0148
#define TOP_DMAS_CH8_CTL1                                 TOP_DMAS_BASE + 0x014C
#define TOP_DMAS_CH8_CA                                   TOP_DMAS_BASE + 0x0150
#define TOP_DMAS_CH8_INTA                                 TOP_DMAS_BASE + 0x0154
#define TOP_DMAS_CH9_SAR                                  TOP_DMAS_BASE + 0x0160
#define TOP_DMAS_CH9_DAR                                  TOP_DMAS_BASE + 0x0164
#define TOP_DMAS_CH9_CTL0                                 TOP_DMAS_BASE + 0x0168
#define TOP_DMAS_CH9_CTL1                                 TOP_DMAS_BASE + 0x016C
#define TOP_DMAS_CH9_CA                                   TOP_DMAS_BASE + 0x0170
#define TOP_DMAS_CH9_INTA                                 TOP_DMAS_BASE + 0x0174
#define TOP_DMAS_CH10_SAR                                 TOP_DMAS_BASE + 0x0180
#define TOP_DMAS_CH10_DAR                                 TOP_DMAS_BASE + 0x0184
#define TOP_DMAS_CH10_CTL0                                TOP_DMAS_BASE + 0x0188
#define TOP_DMAS_CH10_CTL1                                TOP_DMAS_BASE + 0x018C
#define TOP_DMAS_CH10_CA                                  TOP_DMAS_BASE + 0x0190
#define TOP_DMAS_CH10_INTA                                TOP_DMAS_BASE + 0x0194
#define TOP_DMAS_CH11_SAR                                 TOP_DMAS_BASE + 0x01A0
#define TOP_DMAS_CH11_DAR                                 TOP_DMAS_BASE + 0x01A4
#define TOP_DMAS_CH11_CTL0                                TOP_DMAS_BASE + 0x01A8
#define TOP_DMAS_CH11_CTL1                                TOP_DMAS_BASE + 0x01AC
#define TOP_DMAS_CH11_CA                                  TOP_DMAS_BASE + 0x01B0
#define TOP_DMAS_CH11_INTA                                TOP_DMAS_BASE + 0x01B4
#define TOP_DMAS_CH12_SAR                                 TOP_DMAS_BASE + 0x01C0
#define TOP_DMAS_CH12_DAR                                 TOP_DMAS_BASE + 0x01C4
#define TOP_DMAS_CH12_CTL0                                TOP_DMAS_BASE + 0x01C8
#define TOP_DMAS_CH12_CTL1                                TOP_DMAS_BASE + 0x01CC
#define TOP_DMAS_CH12_CA                                  TOP_DMAS_BASE + 0x01D0
#define TOP_DMAS_CH12_INTA                                TOP_DMAS_BASE + 0x01D4
#define TOP_DMAS_CH13_SAR                                 TOP_DMAS_BASE + 0x01E0
#define TOP_DMAS_CH13_DAR                                 TOP_DMAS_BASE + 0x01E4
#define TOP_DMAS_CH13_CTL0                                TOP_DMAS_BASE + 0x01E8
#define TOP_DMAS_CH13_CTL1                                TOP_DMAS_BASE + 0x01EC
#define TOP_DMAS_CH13_CA                                  TOP_DMAS_BASE + 0x01F0
#define TOP_DMAS_CH13_INTA                                TOP_DMAS_BASE + 0x01F4
#define TOP_DMAS_CH14_SAR                                 TOP_DMAS_BASE + 0x0200
#define TOP_DMAS_CH14_DAR                                 TOP_DMAS_BASE + 0x0204
#define TOP_DMAS_CH14_CTL0                                TOP_DMAS_BASE + 0x0208
#define TOP_DMAS_CH14_CTL1                                TOP_DMAS_BASE + 0x020C
#define TOP_DMAS_CH14_CA                                  TOP_DMAS_BASE + 0x0210
#define TOP_DMAS_CH14_INTA                                TOP_DMAS_BASE + 0x0214
#define TOP_DMAS_CH15_SAR                                 TOP_DMAS_BASE + 0x0220
#define TOP_DMAS_CH15_DAR                                 TOP_DMAS_BASE + 0x0224
#define TOP_DMAS_CH15_CTL0                                TOP_DMAS_BASE + 0x0228
#define TOP_DMAS_CH15_CTL1                                TOP_DMAS_BASE + 0x022C
#define TOP_DMAS_CH15_CA                                  TOP_DMAS_BASE + 0x0230
#define TOP_DMAS_CH15_INTA                                TOP_DMAS_BASE + 0x0234

/*******************************************************************************
TOP_DMAG registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define TOP_DMAG_CH_STATUS                                TOP_DMAG_BASE + 0x0000
#define TOP_DMAG_CH_PRIOR0                                TOP_DMAG_BASE + 0x0020
#define TOP_DMAG_CH_INTR_EN0                              TOP_DMAG_BASE + 0x0040
#define TOP_DMAG_CH_INTR_MASK0                            TOP_DMAG_BASE + 0x0044
#define TOP_DMAG_CH_INTR_STATUS0                          TOP_DMAG_BASE + 0x0048
#define TOP_DMAG_CH_INTR_EN1                              TOP_DMAG_BASE + 0x0050
#define TOP_DMAG_CH_INTR_MASK1                            TOP_DMAG_BASE + 0x0054
#define TOP_DMAG_CH_INTR_STATUS1                          TOP_DMAG_BASE + 0x0058
#define TOP_DMAG_CH_INTR_EN2                              TOP_DMAG_BASE + 0x0060
#define TOP_DMAG_CH_INTR_MASK2                            TOP_DMAG_BASE + 0x0064
#define TOP_DMAG_CH_INTR_STATUS2                          TOP_DMAG_BASE + 0x0068
#define TOP_DMAG_CH_LP_EN0                                TOP_DMAG_BASE + 0x0080
#define TOP_DMAG_CH_BUS_LP_EN                             TOP_DMAG_BASE + 0x0088
/*Í¨µÀ0¼Ä´æÆ÷*/
#define TOP_DMAG_CH0_CTRL                                 TOP_DMAG_BASE + 0x0100
#define TOP_DMAG_CH0_CONFIG                               TOP_DMAG_BASE + 0x0104
#define TOP_DMAG_CH0_SRC_ADDR                             TOP_DMAG_BASE + 0x0108
#define TOP_DMAG_CH0_DST_ADDR                             TOP_DMAG_BASE + 0x0110
#define TOP_DMAG_CH0_SIZE                                 TOP_DMAG_BASE + 0x0118
#define TOP_DMAG_CH0_INTR_EN                              TOP_DMAG_BASE + 0x012C
#define TOP_DMAG_CH0_INTR_STATUS                          TOP_DMAG_BASE + 0x0130
#define TOP_DMAG_CH0_INTR_RAW                             TOP_DMAG_BASE + 0x0134
#define TOP_DMAG_CH0_MONITOR_CTRL                         TOP_DMAG_BASE + 0x0138
#define TOP_DMAG_CH0_MONITOR_OUT                          TOP_DMAG_BASE + 0x013C
/*Í¨µÀ1¼Ä´æÆ÷*/
#define TOP_DMAG_CH1_CTRL                                 TOP_DMAG_BASE + 0x0140
#define TOP_DMAG_CH1_CONFIG                               TOP_DMAG_BASE + 0x0144
#define TOP_DMAG_CH1_SRC_ADDR                             TOP_DMAG_BASE + 0x0148
#define TOP_DMAG_CH1_DST_ADDR                             TOP_DMAG_BASE + 0x0150
#define TOP_DMAG_CH1_SIZE                                 TOP_DMAG_BASE + 0x0158
#define TOP_DMAG_CH1_INTR_EN                              TOP_DMAG_BASE + 0x016C
#define TOP_DMAG_CH1_INTR_STATUS                          TOP_DMAG_BASE + 0x0170
#define TOP_DMAG_CH1_INTR_RAW                             TOP_DMAG_BASE + 0x0174
#define TOP_DMAG_CH1_MONITOR_CTRL                         TOP_DMAG_BASE + 0x0178
#define TOP_DMAG_CH1_MONITOR_OUT                          TOP_DMAG_BASE + 0x017C

/*******************************************************************************
COM_I2C registers' address on LC1860
********************************************************************************/
#define COM_I2C_CON                                       COM_I2C_BASE + 0x0000
#define COM_I2C_TAR                                       COM_I2C_BASE + 0x0004
#define COM_I2C_HS_MADDR                                  COM_I2C_BASE + 0x000C
#define COM_I2C_DATA_CMD                                  COM_I2C_BASE + 0x0010
#define COM_I2C_SS_SCL_HCNT                               COM_I2C_BASE + 0x0014
#define COM_I2C_SS_SCL_LCNT                               COM_I2C_BASE + 0x0018
#define COM_I2C_FS_SCL_HCNT                               COM_I2C_BASE + 0x001C
#define COM_I2C_FS_SCL_LCNT                               COM_I2C_BASE + 0x0020
#define COM_I2C_HS_SCL_HCNT                               COM_I2C_BASE + 0x0024
#define COM_I2C_HS_SCL_LCNT                               COM_I2C_BASE + 0x0028
#define COM_I2C_INTR_STAT                                 COM_I2C_BASE + 0x002C
#define COM_I2C_INTR_EN                                   COM_I2C_BASE + 0x0030
#define COM_I2C_RAW_INTR_STAT                             COM_I2C_BASE + 0x0034
#define COM_I2C_RX_TL                                     COM_I2C_BASE + 0x0038
#define COM_I2C_TX_TL                                     COM_I2C_BASE + 0x003C
#define COM_I2C_CLR_INTR                                  COM_I2C_BASE + 0x0040
#define COM_I2C_CLR_RX_UNDER                              COM_I2C_BASE + 0x0044
#define COM_I2C_CLR_RX_OVER                               COM_I2C_BASE + 0x0048
#define COM_I2C_CLR_TX_OVER                               COM_I2C_BASE + 0x004C
#define COM_I2C_CLR_TX_ABRT                               COM_I2C_BASE + 0x0054
#define COM_I2C_CLR_ACTIVITY                              COM_I2C_BASE + 0x005C
#define COM_I2C_CLR_STOP_DET                              COM_I2C_BASE + 0x0060
#define COM_I2C_CLR_START_DET                             COM_I2C_BASE + 0x0064
#define COM_I2C_CLR_GEN_CALL                              COM_I2C_BASE + 0x0068
#define COM_I2C_ENABLE                                    COM_I2C_BASE + 0x006C
#define COM_I2C_STATUS                                    COM_I2C_BASE + 0x0070
#define COM_I2C_TXFLR                                     COM_I2C_BASE + 0x0074
#define COM_I2C_RXFLR                                     COM_I2C_BASE + 0x0078
#define COM_I2C_SDA_HOLD                                  COM_I2C_BASE + 0x007C
#define COM_I2C_TX_ABRT_SOURCE                            COM_I2C_BASE + 0x0080
#define COM_I2C_FS_SPKLEN                                 COM_I2C_BASE + 0x00A0
#define COM_I2C_HS_SPKLEN                                 COM_I2C_BASE + 0x00A4

/*******************************************************************************
COM_PCM registers' address on LC1860
********************************************************************************/
#define COM_PCM_MODE                                      COM_PCM_BASE + 0x0000
#define COM_PCM_SCLK_CFG                                  COM_PCM_BASE + 0x0004
#define COM_PCM_SYNC_CFG                                  COM_PCM_BASE + 0x0008
#define COM_PCM_EN                                        COM_PCM_BASE + 0x000C
#define COM_PCM_FIFO_STA                                  COM_PCM_BASE + 0x0010
#define COM_PCM_REC_FIFO                                  COM_PCM_BASE + 0x0014
#define COM_PCM_TRAN_FIFO                                 COM_PCM_BASE + 0x0018
#define COM_PCM_INTR_STA_RAW                              COM_PCM_BASE + 0x0020
#define COM_PCM_INTR_STA                                  COM_PCM_BASE + 0x0024
#define COM_PCM_INTR_EN                                   COM_PCM_BASE + 0x0028

/*******************************************************************************
MUXPIN registers' address on LC1860
********************************************************************************/
//#define MUXPIN_CTRL_DIGRFEN                               MUXPIN_BASE + 0x0000
#define MUXPIN_CTRL_DIGRFEN                                MUXPIN_BASE + 0x0000
#define MUXPIN_CTRL_RFCKEN                                MUXPIN_BASE + 0x0004
#define MUXPIN_CTRL_RFGPO0                                MUXPIN_BASE + 0x0008
#define MUXPIN_CTRL_RFGPO1                                MUXPIN_BASE + 0x000C
#define MUXPIN_CTRL_RFGPO2                                MUXPIN_BASE + 0x0010
#define MUXPIN_CTRL_RFGPO3                                MUXPIN_BASE + 0x0014
#define MUXPIN_CTRL_RFGPO4                                MUXPIN_BASE + 0x0018
#define MUXPIN_CTRL_RFGPO5                                MUXPIN_BASE + 0x001C
#define MUXPIN_CTRL_RFGPO6                                MUXPIN_BASE + 0x0020
#define MUXPIN_CTRL_RFGPO7                                MUXPIN_BASE + 0x0024
#define MUXPIN_CTRL_RFGPO8                                MUXPIN_BASE + 0x0028
#define MUXPIN_CTRL_RFGPO9                                MUXPIN_BASE + 0x002C
#define MUXPIN_CTRL_RFGPO10                               MUXPIN_BASE + 0x0030
#define MUXPIN_CTRL_RFGPO11                               MUXPIN_BASE + 0x0034
#define MUXPIN_CTRL_RFGPO12                               MUXPIN_BASE + 0x0038
#define MUXPIN_CTRL_RFGPO13                               MUXPIN_BASE + 0x003C
#define MUXPIN_CTRL_RFGPO14                               MUXPIN_BASE + 0x0040
#define MUXPIN_CTRL_RFGPO15                               MUXPIN_BASE + 0x0044
#define MUXPIN_CTRL_RFGPO16                               MUXPIN_BASE + 0x0048
#define MUXPIN_CTRL_RFGPO17                               MUXPIN_BASE + 0x004C
#define MUXPIN_CTRL_RFGPO18                               MUXPIN_BASE + 0x0050
#define MUXPIN_CTRL_RFGPO19                               MUXPIN_BASE + 0x0054
#define MUXPIN_CTRL_RFGPO20                               MUXPIN_BASE + 0x0058
#define MUXPIN_CTRL_RFGPO21                               MUXPIN_BASE + 0x005C
#define MUXPIN_CTRL_RFGPO22                               MUXPIN_BASE + 0x0060
#define MUXPIN_CTRL_RFGPO23                               MUXPIN_BASE + 0x0064
#define MUXPIN_CTRL_RFGPO24                               MUXPIN_BASE + 0x0068
#define MUXPIN_CTRL_RFGPO25                               MUXPIN_BASE + 0x006C
#define MUXPIN_CTRL_RFGPO26                               MUXPIN_BASE + 0x0070
#define MUXPIN_CTRL_RFGPO27                               MUXPIN_BASE + 0x0074
#define MUXPIN_CTRL_RFGPO28                               MUXPIN_BASE + 0x0078
#define MUXPIN_CTRL_RFGPO29                               MUXPIN_BASE + 0x007C
#define MUXPIN_CTRL_RFGPO30                               MUXPIN_BASE + 0x0080
#define MUXPIN_CTRL_RFGPO31                               MUXPIN_BASE + 0x0084
#define MUXPIN_CTRL_RFGPO32                               MUXPIN_BASE + 0x0088
#define MUXPIN_CTRL_RFGPO33                               MUXPIN_BASE + 0x008C
#define MUXPIN_CTRL_RFGPO34                               MUXPIN_BASE + 0x0090
#define MUXPIN_CTRL_SPICS0                                MUXPIN_BASE + 0x0094
#define MUXPIN_CTRL_SPIDIN                                MUXPIN_BASE + 0x0098
#define MUXPIN_CTRL_SPIDO                                 MUXPIN_BASE + 0x009C
#define MUXPIN_CTRL_SPICK                                 MUXPIN_BASE + 0x00A0
#define MUXPIN_CTRL_RFFECK                                MUXPIN_BASE + 0x00A4
#define MUXPIN_CTRL_RFFED                                 MUXPIN_BASE + 0x00A8
#define MUXPIN_CTRL_HSLF0                                 MUXPIN_BASE + 0x00AC
#define MUXPIN_CTRL_HSLF1                                 MUXPIN_BASE + 0x00B0
#define MUXPIN_CTRL_HSLA                                  MUXPIN_BASE + 0x00B4
#define MUXPIN_CTRL_HSLD0                                 MUXPIN_BASE + 0x00B8
#define MUXPIN_CTRL_HSLD1                                 MUXPIN_BASE + 0x00BC
#define MUXPIN_CTRL_HSLD2                                 MUXPIN_BASE + 0x00C0
#define MUXPIN_CTRL_HSLD3                                 MUXPIN_BASE + 0x00C4
#define MUXPIN_CTRL_HSLD4                                 MUXPIN_BASE + 0x00C8
#define MUXPIN_CTRL_HSLD5                                 MUXPIN_BASE + 0x00CC
#define MUXPIN_CTRL_HSLD6                                 MUXPIN_BASE + 0x00D0
#define MUXPIN_CTRL_HSLD7                                 MUXPIN_BASE + 0x00D4
#define MUXPIN_CTRL_HSLD8                                 MUXPIN_BASE + 0x00D8
#define MUXPIN_CTRL_HSLD9                                 MUXPIN_BASE + 0x00DC
#define MUXPIN_CTRL_HSLD10                                MUXPIN_BASE + 0x00E0
#define MUXPIN_CTRL_HSLD11                                MUXPIN_BASE + 0x00E4
#define MUXPIN_CTRL_HSLD12                                MUXPIN_BASE + 0x00E8
#define MUXPIN_CTRL_HSLD13                                MUXPIN_BASE + 0x00EC
#define MUXPIN_CTRL_HSLD14                                MUXPIN_BASE + 0x00F0
#define MUXPIN_CTRL_HSLD15                                MUXPIN_BASE + 0x00F4
#define MUXPIN_CTRL_HSLPKEND                              MUXPIN_BASE + 0x00F8
#define MUXPIN_CTRL_HSLCS                                 MUXPIN_BASE + 0x00FC
#define MUXPIN_CTRL_HSLWR                                 MUXPIN_BASE + 0x0100
#define MUXPIN_CTRL_HSLCK                                 MUXPIN_BASE + 0x0104
#define MUXPIN_CTRL_SIM0CLK                               MUXPIN_BASE + 0x0108
#define MUXPIN_CTRL_SIM0IO                                MUXPIN_BASE + 0x010C
#define MUXPIN_CTRL_SIM0RST                               MUXPIN_BASE + 0x0110
#define MUXPIN_CTRL_SIM1CLK                               MUXPIN_BASE + 0x0114
#define MUXPIN_CTRL_SIM1IO                                MUXPIN_BASE + 0x0118
#define MUXPIN_CTRL_SIM1RST                               MUXPIN_BASE + 0x011C
#define MUXPIN_CTRL_GPIO72                                MUXPIN_BASE + 0x0120
#define MUXPIN_CTRL_GPIO73                                MUXPIN_BASE + 0x0124
#define MUXPIN_CTRL_GPIO74                                MUXPIN_BASE + 0x0128
#define MUXPIN_CTRL_GPIO75                                MUXPIN_BASE + 0x012C
#define MUXPIN_CTRL_U0TXD                                 MUXPIN_BASE + 0x0130
#define MUXPIN_CTRL_U0RXD                                 MUXPIN_BASE + 0x0134
#define MUXPIN_CTRL_U0CTS                                 MUXPIN_BASE + 0x0138
#define MUXPIN_CTRL_U0RTS                                 MUXPIN_BASE + 0x013C
#define MUXPIN_CTRL_CS0TXD                                MUXPIN_BASE + 0x0140
#define MUXPIN_CTRL_CS0RXD                                MUXPIN_BASE + 0x0144
#define MUXPIN_CTRL_CS0CLK                                MUXPIN_BASE + 0x0148
#define MUXPIN_CTRL_CS0SN                                 MUXPIN_BASE + 0x014C
#define MUXPIN_CTRL_IIS0DI                                MUXPIN_BASE + 0x0150
#define MUXPIN_CTRL_IIS0DO                                MUXPIN_BASE + 0x0154
#define MUXPIN_CTRL_IIS0CK                                MUXPIN_BASE + 0x0158
#define MUXPIN_CTRL_IIS0WS                                MUXPIN_BASE + 0x015C
#define MUXPIN_CTRL_CIICSCL                               MUXPIN_BASE + 0x0160
#define MUXPIN_CTRL_CIICSDA                               MUXPIN_BASE + 0x0164
#define MUXPIN_CTRL_CLKO0                                 MUXPIN_BASE + 0x0168
#define MUXPIN_CTRL_CLKO4                                 MUXPIN_BASE + 0x016C
#define MUXPIN_CTRL_OSCEN                                 MUXPIN_BASE + 0x0170
#define MUXPIN_CTRL_PWEN                                  MUXPIN_BASE + 0x0174
#define MUXPIN_CTRL_RSTN                                  MUXPIN_BASE + 0x0178
#define MUXPIN_CTRL_CPUDVFS0                              MUXPIN_BASE + 0x017C
#define MUXPIN_CTRL_CPUDVFS1                              MUXPIN_BASE + 0x0180
#define MUXPIN_CTRL_GPUDVFC0                              MUXPIN_BASE + 0x0184
#define MUXPIN_CTRL_GPUDVFC1                              MUXPIN_BASE + 0x0188
#define MUXPIN_CTRL_GPIO99                                MUXPIN_BASE + 0x018C
#define MUXPIN_CTRL_CPBOOT                                MUXPIN_BASE + 0x0190
#define MUXPIN_CTRL_USBDRV                                MUXPIN_BASE + 0x0194
#define MUXPIN_CTRL_NANDD15                               MUXPIN_BASE + 0x0198
#define MUXPIN_CTRL_MMC1D2                                MUXPIN_BASE + 0x019C
#define MUXPIN_CTRL_NANDD13                               MUXPIN_BASE + 0x01A0
#define MUXPIN_CTRL_NANDD12                               MUXPIN_BASE + 0x01A4
#define MUXPIN_CTRL_MMC1D1                                MUXPIN_BASE + 0x01A8
#define MUXPIN_CTRL_NANDD10                               MUXPIN_BASE + 0x01AC
#define MUXPIN_CTRL_NANDD9                                MUXPIN_BASE + 0x01B0
#define MUXPIN_CTRL_NANDD8                                MUXPIN_BASE + 0x01B4
#define MUXPIN_CTRL_MMC1D3                                MUXPIN_BASE + 0x01B8
#define MUXPIN_CTRL_MMC1D0                                MUXPIN_BASE + 0x01BC
#define MUXPIN_CTRL_MMC1D4                                MUXPIN_BASE + 0x01C0
#define MUXPIN_CTRL_MMC1D5                                MUXPIN_BASE + 0x01C4
#define MUXPIN_CTRL_NANDD3                                MUXPIN_BASE + 0x01C8
#define MUXPIN_CTRL_NANDD2                                MUXPIN_BASE + 0x01CC
#define MUXPIN_CTRL_NANDD1                                MUXPIN_BASE + 0x01D0
#define MUXPIN_CTRL_NANDD0                                MUXPIN_BASE + 0x01D4
#define MUXPIN_CTRL_MMC1D7                                MUXPIN_BASE + 0x01D8
#define MUXPIN_CTRL_MMC1D6                                MUXPIN_BASE + 0x01DC
#define MUXPIN_CTRL_NANDWEN                               MUXPIN_BASE + 0x01E0
#define MUXPIN_CTRL_MMC1CLK                               MUXPIN_BASE + 0x01E4
#define MUXPIN_CTRL_MMC1CMD                               MUXPIN_BASE + 0x01E8
#define MUXPIN_CTRL_NANDCS0                               MUXPIN_BASE + 0x01EC
#define MUXPIN_CTRL_NANDCS1                               MUXPIN_BASE + 0x01F0
#define MUXPIN_CTRL_LCD23                                 MUXPIN_BASE + 0x01F4
#define MUXPIN_CTRL_LCD22                                 MUXPIN_BASE + 0x01F8
#define MUXPIN_CTRL_LCD21                                 MUXPIN_BASE + 0x01FC
#define MUXPIN_CTRL_LCD20                                 MUXPIN_BASE + 0x0200
#define MUXPIN_CTRL_LCD19                                 MUXPIN_BASE + 0x0204
#define MUXPIN_CTRL_LCD18                                 MUXPIN_BASE + 0x0208
#define MUXPIN_CTRL_LCD17                                 MUXPIN_BASE + 0x020C
#define MUXPIN_CTRL_LCD16                                 MUXPIN_BASE + 0x0210
#define MUXPIN_CTRL_LCD15                                 MUXPIN_BASE + 0x0214
#define MUXPIN_CTRL_LCD14                                 MUXPIN_BASE + 0x0218
#define MUXPIN_CTRL_LCD13                                 MUXPIN_BASE + 0x021C
#define MUXPIN_CTRL_LCD12                                 MUXPIN_BASE + 0x0220
#define MUXPIN_CTRL_LCD11                                 MUXPIN_BASE + 0x0224
#define MUXPIN_CTRL_LCD10                                 MUXPIN_BASE + 0x0228
#define MUXPIN_CTRL_LCD9                                  MUXPIN_BASE + 0x022C
#define MUXPIN_CTRL_LCD8                                  MUXPIN_BASE + 0x0230
#define MUXPIN_CTRL_LCD7                                  MUXPIN_BASE + 0x0234
#define MUXPIN_CTRL_LCD6                                  MUXPIN_BASE + 0x0238
#define MUXPIN_CTRL_LCD5                                  MUXPIN_BASE + 0x023C
#define MUXPIN_CTRL_LCD4                                  MUXPIN_BASE + 0x0240
#define MUXPIN_CTRL_LCD3                                  MUXPIN_BASE + 0x0244
#define MUXPIN_CTRL_LCD2                                  MUXPIN_BASE + 0x0248
#define MUXPIN_CTRL_LCD1                                  MUXPIN_BASE + 0x024C
#define MUXPIN_CTRL_LCD0                                  MUXPIN_BASE + 0x0250
#define MUXPIN_CTRL_LCDHS                                 MUXPIN_BASE + 0x0254
#define MUXPIN_CTRL_LCDCK                                 MUXPIN_BASE + 0x0258
#define MUXPIN_CTRL_ENAB                                  MUXPIN_BASE + 0x025C
#define MUXPIN_CTRL_LCDVS                                 MUXPIN_BASE + 0x0260
#define MUXPIN_CTRL_PWM0                                  MUXPIN_BASE + 0x0264
#define MUXPIN_CTRL_GPIO154                               MUXPIN_BASE + 0x0268
#define MUXPIN_CTRL_GPIO155                               MUXPIN_BASE + 0x026C
#define MUXPIN_CTRL_GPIO156                               MUXPIN_BASE + 0x0270
#define MUXPIN_CTRL_S0TXD                                 MUXPIN_BASE + 0x0274
#define MUXPIN_CTRL_S0RXD                                 MUXPIN_BASE + 0x0278
#define MUXPIN_CTRL_S0CLK                                 MUXPIN_BASE + 0x027C
#define MUXPIN_CTRL_S0SN                                  MUXPIN_BASE + 0x0280
#define MUXPIN_CTRL_S2TXD                                 MUXPIN_BASE + 0x0284
#define MUXPIN_CTRL_S2RXD                                 MUXPIN_BASE + 0x0288
#define MUXPIN_CTRL_S2CLK                                 MUXPIN_BASE + 0x028C
#define MUXPIN_CTRL_S2SN                                  MUXPIN_BASE + 0x0290
#define MUXPIN_CTRL_GPIO165                               MUXPIN_BASE + 0x0294
#define MUXPIN_CTRL_GPIO166                               MUXPIN_BASE + 0x0298
#define MUXPIN_CTRL_ISPSCL                                MUXPIN_BASE + 0x029C
#define MUXPIN_CTRL_ISPSDA                                MUXPIN_BASE + 0x02A0
#define MUXPIN_CTRL_GPIO169                               MUXPIN_BASE + 0x02A4
#define MUXPIN_CTRL_GPIO170                               MUXPIN_BASE + 0x02A8
#define MUXPIN_CTRL_GPIO171                               MUXPIN_BASE + 0x02AC
#define MUXPIN_CTRL_GPIO172                               MUXPIN_BASE + 0x02B0
#define MUXPIN_CTRL_GPIO173                               MUXPIN_BASE + 0x02B4
#define MUXPIN_CTRL_ISPPWM                                MUXPIN_BASE + 0x02B8
#define MUXPIN_CTRL_ISPFIN0                               MUXPIN_BASE + 0x02BC
#define MUXPIN_CTRL_ISPFIN1                               MUXPIN_BASE + 0x02C0
#define MUXPIN_CTRL_ISPSTBE                               MUXPIN_BASE + 0x02C4
#define MUXPIN_CTRL_CLKO2                                 MUXPIN_BASE + 0x02C8
#define MUXPIN_CTRL_U1TXD                                 MUXPIN_BASE + 0x02CC
#define MUXPIN_CTRL_U1RXD                                 MUXPIN_BASE + 0x02D0
#define MUXPIN_CTRL_U1CTS                                 MUXPIN_BASE + 0x02D4
#define MUXPIN_CTRL_U1RTS                                 MUXPIN_BASE + 0x02D8
#define MUXPIN_CTRL_U2TXD                                 MUXPIN_BASE + 0x02DC
#define MUXPIN_CTRL_U2RXD                                 MUXPIN_BASE + 0x02E0
#define MUXPIN_CTRL_U2CTS                                 MUXPIN_BASE + 0x02E4
#define MUXPIN_CTRL_U2RTS                                 MUXPIN_BASE + 0x02E8
#define MUXPIN_CTRL_CS1TXD                                MUXPIN_BASE + 0x02EC
#define MUXPIN_CTRL_CS1RXD                                MUXPIN_BASE + 0x02F0
#define MUXPIN_CTRL_CS1CLK                                MUXPIN_BASE + 0x02F4
#define MUXPIN_CTRL_CS1SN                                 MUXPIN_BASE + 0x02F8
#define MUXPIN_CTRL_MMC2CLK                               MUXPIN_BASE + 0x02FC
#define MUXPIN_CTRL_MMC2CMD                               MUXPIN_BASE + 0x0300
#define MUXPIN_CTRL_MMC2D3                                MUXPIN_BASE + 0x0304
#define MUXPIN_CTRL_MMC2D2                                MUXPIN_BASE + 0x0308
#define MUXPIN_CTRL_MMC2D1                                MUXPIN_BASE + 0x030C
#define MUXPIN_CTRL_MMC2D0                                MUXPIN_BASE + 0x0310
#define MUXPIN_CTRL_CLKO3                                 MUXPIN_BASE + 0x0314
#define MUXPIN_CTRL_GPIO198                               MUXPIN_BASE + 0x0318
#define MUXPIN_CTRL_GPIO199                               MUXPIN_BASE + 0x031C
#define MUXPIN_CTRL_GPIO200                               MUXPIN_BASE + 0x0320
#define MUXPIN_CTRL_GPIO201                               MUXPIN_BASE + 0x0324
#define MUXPIN_CTRL_GPIO202                               MUXPIN_BASE + 0x0328
#define MUXPIN_CTRL_GPIO203                               MUXPIN_BASE + 0x032C
#define MUXPIN_CTRL_GPIO204                               MUXPIN_BASE + 0x0330
#define MUXPIN_CTRL_GPIO205                               MUXPIN_BASE + 0x0334
#define MUXPIN_CTRL_GPIO206                               MUXPIN_BASE + 0x0338
#define MUXPIN_CTRL_IIC1SCL                               MUXPIN_BASE + 0x033C
#define MUXPIN_CTRL_IIC1SDA                               MUXPIN_BASE + 0x0340
#define MUXPIN_CTRL_GPIO209                               MUXPIN_BASE + 0x0344
#define MUXPIN_CTRL_GPIO210                               MUXPIN_BASE + 0x0348
#define MUXPIN_CTRL_GPIO211                               MUXPIN_BASE + 0x034C
#define MUXPIN_CTRL_GPIO212                               MUXPIN_BASE + 0x0350
#define MUXPIN_CTRL_IIC0SCL                               MUXPIN_BASE + 0x0354
#define MUXPIN_CTRL_IIC0SDA                               MUXPIN_BASE + 0x0358
#define MUXPIN_CTRL_GPIO215                               MUXPIN_BASE + 0x035C
#define MUXPIN_CTRL_GPIO216                               MUXPIN_BASE + 0x0360
#define MUXPIN_CTRL_GPIO217                               MUXPIN_BASE + 0x0364
#define MUXPIN_CTRL_GPIO218                               MUXPIN_BASE + 0x0368
#define MUXPIN_CTRL_S1TXD                                 MUXPIN_BASE + 0x036C
#define MUXPIN_CTRL_S1RXD                                 MUXPIN_BASE + 0x0370
#define MUXPIN_CTRL_S1CLK                                 MUXPIN_BASE + 0x0374
#define MUXPIN_CTRL_S1SN                                  MUXPIN_BASE + 0x0378
#define MUXPIN_CTRL_GPIO223                               MUXPIN_BASE + 0x037C
#define MUXPIN_CTRL_GPIO224                               MUXPIN_BASE + 0x0380
#define MUXPIN_CTRL_CLKO1                                 MUXPIN_BASE + 0x0384
#define MUXPIN_CTRL_KB0                                   MUXPIN_BASE + 0x0388
#define MUXPIN_CTRL_KB1                                   MUXPIN_BASE + 0x038C
#define MUXPIN_CTRL_KB2                                   MUXPIN_BASE + 0x0390
#define MUXPIN_CTRL_KB3                                   MUXPIN_BASE + 0x0394
#define MUXPIN_CTRL_TCK0                                  MUXPIN_BASE + 0x0398
#define MUXPIN_CTRL_TDI0                                  MUXPIN_BASE + 0x039C
#define MUXPIN_CTRL_TDO0                                  MUXPIN_BASE + 0x03A0
#define MUXPIN_CTRL_TMS0                                  MUXPIN_BASE + 0x03A4
#define MUXPIN_CTRL_NTRS0                                 MUXPIN_BASE + 0x03A8
#define MUXPIN_CTRL_TCK1                                  MUXPIN_BASE + 0x03AC
#define MUXPIN_CTRL_TDI1                                  MUXPIN_BASE + 0x03B0
#define MUXPIN_CTRL_TDO1                                  MUXPIN_BASE + 0x03B4
#define MUXPIN_CTRL_TMS1                                  MUXPIN_BASE + 0x03B8
#define MUXPIN_CTRL_NTRS1                                 MUXPIN_BASE + 0x03BC
#define MUXPIN_CTRL_BOOTCTL2                              MUXPIN_BASE + 0x03C0
#define MUXPIN_CTRL_BOOTCTL1                              MUXPIN_BASE + 0x03C4
#define MUXPIN_CTRL_BOOTCTL0                              MUXPIN_BASE + 0x03C8
#define MUXPIN_CTRL_GPIO243                               MUXPIN_BASE + 0x03CC
#define MUXPIN_CTRL_U3TXD                                 MUXPIN_BASE + 0x03D0
#define MUXPIN_CTRL_U3RXD                                 MUXPIN_BASE + 0x03D4
#define MUXPIN_CTRL_MMC0CMD                               MUXPIN_BASE + 0x03D8
#define MUXPIN_CTRL_MMC0D0                                MUXPIN_BASE + 0x03DC
#define MUXPIN_CTRL_OSCEN1                                MUXPIN_BASE + 0x03E0
#define MUXPIN_CTRL_DRFD                                  MUXPIN_BASE + 0x03E4
#define MUXPIN_CTRL_DRFDEN                                MUXPIN_BASE + 0x03E8
#define MUXPIN_CTRL_DRFCTLD                               MUXPIN_BASE + 0x03EC
#define MUXPIN_CTRL_DRFCTLCLK                             MUXPIN_BASE + 0x03F0
#define MUXPIN_CTRL_DRFCTLEN                              MUXPIN_BASE + 0x03F4
#define MUXPIN_CTRL_DRFSTBE                               MUXPIN_BASE + 0x03F8
#define MUXPIN_CTRL_GSM1GPO0                              MUXPIN_BASE + 0x03FC
#define MUXPIN_CTRL_GSM1GPO1                              MUXPIN_BASE + 0x0480
#define MUXPIN_CTRL_GSM1GPO2                              MUXPIN_BASE + 0x0484
#define MUXPIN_CTRL_GSM1GPO3                              MUXPIN_BASE + 0x0488
#define MUXPIN_CTRL_GSM1GPO4                              MUXPIN_BASE + 0x048C
#define MUXPIN_CTRL_GSM1GPO5                              MUXPIN_BASE + 0x0490
#define MUXPIN_CTRL_GSM1GPO6                              MUXPIN_BASE + 0x0494
#define MUXPIN_CTRL_GSM1GPO7                              MUXPIN_BASE + 0x0498
#define MUXPIN_CTRL_GSM1GPO8                              MUXPIN_BASE + 0x049C
#define MUXPIN_CTRL_TMOD1                                 MUXPIN_BASE + 0x04A0
#define MUXPIN_CTRL_TMOD0                                 MUXPIN_BASE + 0x04A4
#define MUXPIN_CTRL_BTVDSEL                               MUXPIN_BASE + 0x04A8
#define MUXPIN_CTRL_SMOD                                  MUXPIN_BASE + 0x04AC
#define MUXPIN_CTRL_MMC0CLK                               MUXPIN_BASE + 0x04B0
#define MUXPIN_CTRL_MMC0D3                                MUXPIN_BASE + 0x04B4
#define MUXPIN_CTRL_MMC0D2                                MUXPIN_BASE + 0x04B8
#define MUXPIN_CTRL_MMC0D1                                MUXPIN_BASE + 0x04BC
#define MUXPIN_PVDD3_VOL_CTRL                             MUXPIN_BASE + 0x04C0
#define MUXPIN_PVDD4_VOL_CTRL                             MUXPIN_BASE + 0x04C4
#define MUXPIN_PVDD8_VOL_CTRL                             MUXPIN_BASE + 0x04C8
#define MUXPIN_PVDD9_VOL_CTRL                             MUXPIN_BASE + 0x04CC
#define MUXPIN_PVDD10_VOL_CTRL                            MUXPIN_BASE + 0x04D0
#define MUXPIN_TESTPIN_CTRL0                              MUXPIN_BASE + 0x04D4
#define MUXPIN_PCM_IO_SEL                                 MUXPIN_BASE + 0x04D8
#define MUXPIN_SEC_PORT                                   MUXPIN_BASE + 0x04DC

/*******************************************************************************
DDR_PWR registers' address on LC1860
********************************************************************************/
#define DDR_PWR_PLLCFG0                                   DDR_PWR_BASE + 0x0000
#define DDR_PWR_PLLCFG1                                   DDR_PWR_BASE + 0x021C
#define DDR_PWR_PLLCTL                                    DDR_PWR_BASE + 0x0004
#define DDR_PWR_PLLTIME                                   DDR_PWR_BASE + 0x0008
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_DDRCLKDIV_NOW                             DDR_PWR_BASE + 0x000C
#define DDR_PWR_COMPRECLKDIV                              DDR_PWR_BASE + 0x0010
#define DDR_PWR_I2SCLKDIV                                 DDR_PWR_BASE + 0x0014
#define DDR_PWR_CORESIGHTCLKDIV                           DDR_PWR_BASE + 0x0018
#define DDR_PWR_I2CCLKDIV                                 DDR_PWR_BASE + 0x001C
#define DDR_PWR_PCMCLKDIV                                 DDR_PWR_BASE + 0x0020
#define DDR_PWR_COMUARTCLKDIV                             DDR_PWR_BASE + 0x0024
#define DDR_PWR_CLKOUT0DIV                                DDR_PWR_BASE + 0x0028
#define DDR_PWR_CLKOUT4DIV                                DDR_PWR_BASE + 0x002C
#define DDR_PWR_BUSCLKDIV                                 DDR_PWR_BASE + 0x0030
#define DDR_PWR_COMAPBCLKDIV                              DDR_PWR_BASE + 0x0034
#define DDR_PWR_PCLKEN                                    DDR_PWR_BASE + 0x003C
#define DDR_PWR_CLKEN0                                    DDR_PWR_BASE + 0x0040
#define DDR_PWR_CLKEN1                                    DDR_PWR_BASE + 0x0044
#define DDR_PWR_UARTCLKCTL                                DDR_PWR_BASE + 0x0048
#define DDR_PWR_METERCLKCTL                               DDR_PWR_BASE + 0x004C
#define DDR_PWR_TSCLKCTL                                  DDR_PWR_BASE + 0x0050
#define DDR_PWR_LPCLKCTL                                  DDR_PWR_BASE + 0x0054
#define DDR_PWR_A7GTMCLKCTL                               DDR_PWR_BASE + 0x0058
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_CMDFIFOCTL                                DDR_PWR_BASE + 0x005C
#define DDR_PWR_CMDFIFOSTATUS                             DDR_PWR_BASE + 0x0060
#define DDR_PWR_CMDFIFOWR                                 DDR_PWR_BASE + 0x0064
#define DDR_PWR_DDRCLKDIVL                                DDR_PWR_BASE + 0x0068
#define DDR_PWR_DDRCLKDIVH                                DDR_PWR_BASE + 0x006C
#define DDR_PWR_AP_DDR_FREQ                               DDR_PWR_BASE + 0x0070
#define DDR_PWR_CP_DDR_FREQ                               DDR_PWR_BASE + 0x0074
#define DDR_PWR_MEMCTLLPEXTCTL                            DDR_PWR_BASE + 0x0078
#define DDR_PWR_MEMCTLDATAAXICTL                          DDR_PWR_BASE + 0x007C
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_TOP_RAM_PD_CTL                            DDR_PWR_BASE + 0x0094
#define DDR_PWR_TOP_RAM_PD_CNT1                           DDR_PWR_BASE + 0x0098
#define DDR_PWR_TOP_RAM_PD_CNT2                           DDR_PWR_BASE + 0x009C
#define DDR_PWR_TOP_RAM_PD_CNT3                           DDR_PWR_BASE + 0x00A0
#define DDR_PWR_TL420_PD_CTL                              DDR_PWR_BASE + 0x00A4
#define DDR_PWR_TL420_PD_CNT1                             DDR_PWR_BASE + 0x00A8
#define DDR_PWR_TL420_PD_CNT2                             DDR_PWR_BASE + 0x00AC
#define DDR_PWR_TL420_PD_CNT3                             DDR_PWR_BASE + 0x00B0
#define DDR_PWR_MEMCTL0_PD_CTL                            DDR_PWR_BASE + 0x00B4
#define DDR_PWR_MEMCTL1_PD_CTL                            DDR_PWR_BASE + 0x00B8
#define DDR_PWR_MEMCTL_PD_CNT1                            DDR_PWR_BASE + 0x00BC
#define DDR_PWR_MEMCTL_PD_CNT2                            DDR_PWR_BASE + 0x00C0
#define DDR_PWR_MEMCTL_PD_CNT3                            DDR_PWR_BASE + 0x00C4
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_FC_CTL                                    DDR_PWR_BASE + 0x00C8
#define DDR_PWR_FC_SHORT_PERIOD                           DDR_PWR_BASE + 0x00CC
#define DDR_PWR_FC_LONG_PERIOD                            DDR_PWR_BASE + 0x00D0
#define DDR_PWR_FC_AWUSER                                 DDR_PWR_BASE + 0x00D4
#define DDR_PWR_FC_ARUSR                                  DDR_PWR_BASE + 0x00D8
#define DDR_PWR_FC_LONG_TIMESCAL0                         DDR_PWR_BASE + 0x00DC
#define DDR_PWR_FC_LONG_TIMESCAL1                         DDR_PWR_BASE + 0x00E0
#define DDR_PWR_FC_LONG_TIMESCAL2                         DDR_PWR_BASE + 0x00E4
#define DDR_PWR_FC_LONG_TIMESCAL3                         DDR_PWR_BASE + 0x00E8
#define DDR_PWR_FC_LONG_TIMESCAL4                         DDR_PWR_BASE + 0x00EC
#define DDR_PWR_FC_LONG_TIMESCAL5                         DDR_PWR_BASE + 0x00F0
#define DDR_PWR_FC_LONG_TIMESCAL6                         DDR_PWR_BASE + 0x00F4
#define DDR_PWR_FC_LONG_TIMESCAL7                         DDR_PWR_BASE + 0x00F8
#define DDR_PWR_FC_LONG_BYTESCAL0                         DDR_PWR_BASE + 0x00FC
#define DDR_PWR_FC_LONG_BYTESCAL1                         DDR_PWR_BASE + 0x0100
#define DDR_PWR_FC_LONG_BYTESCAL2                         DDR_PWR_BASE + 0x0104
#define DDR_PWR_FC_LONG_BYTESCAL3                         DDR_PWR_BASE + 0x0108
#define DDR_PWR_FC_LONG_BYTESCAL4                         DDR_PWR_BASE + 0x010C
#define DDR_PWR_FC_LONG_BYTESCAL5                         DDR_PWR_BASE + 0x0110
#define DDR_PWR_FC_LONG_BYTESCAL6                         DDR_PWR_BASE + 0x0114
#define DDR_PWR_FC_LONG_BYTESCAL7                         DDR_PWR_BASE + 0x011C
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_INTR_MASK_CPA7                            DDR_PWR_BASE + 0x0120
#define DDR_PWR_INTR_MASK_APA7                            DDR_PWR_BASE + 0x0124
#define DDR_PWR_INTR_MASK_XC4210                          DDR_PWR_BASE + 0x0128
#define DDR_PWR_INTR_MASK_TL420                           DDR_PWR_BASE + 0x012C
#define DDR_PWR_INTR_MASK_X1643                           DDR_PWR_BASE + 0x0130
#define DDR_PWR_INTR_RAW                                  DDR_PWR_BASE + 0x0134
#define DDR_PWR_INTR_EN_CPA7                              DDR_PWR_BASE + 0x0138
#define DDR_PWR_INTR_CPA7                                 DDR_PWR_BASE + 0x013C
#define DDR_PWR_INTR_EN_APA7                              DDR_PWR_BASE + 0x0140
#define DDR_PWR_INTR_APA7                                 DDR_PWR_BASE + 0x0144
#define DDR_PWR_INTR_EN_XC4210                            DDR_PWR_BASE + 0x0148
#define DDR_PWR_INTR_XC4210                               DDR_PWR_BASE + 0x014C
#define DDR_PWR_INTR_EN_TL420                             DDR_PWR_BASE + 0x0150
#define DDR_PWR_INTR_TL420                                DDR_PWR_BASE + 0x0154
#define DDR_PWR_INTR_EN_X1643                             DDR_PWR_BASE + 0x0158
#define DDR_PWR_INTR_X1643                                DDR_PWR_BASE + 0x015C
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_AT_CTL                                    DDR_PWR_BASE + 0x0160
#define DDR_PWR_AT_C1                                     DDR_PWR_BASE + 0x0164
#define DDR_PWR_AT_C2                                     DDR_PWR_BASE + 0x0168
#define DDR_PWR_AT_C1_SNAP                                DDR_PWR_BASE + 0x016C
#define DDR_PWR_AT_C2_SNAP                                DDR_PWR_BASE + 0x0170
#define DDR_PWR_AT_CALIB_CTL                              DDR_PWR_BASE + 0x0174
#define DDR_PWR_AT_CALIB_32KINC                           DDR_PWR_BASE + 0x0178
#define DDR_PWR_AT_CALIB_32KNUM                           DDR_PWR_BASE + 0x017C
#define DDR_PWR_AT_CALIB_RESULT                           DDR_PWR_BASE + 0x0180
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_TOPBUSLPCTL                               DDR_PWR_BASE + 0x0184
#define DDR_PWR_TOPBUSLPCNTNUM                            DDR_PWR_BASE + 0x0188
#define DDR_PWR_TOPBUSLPCNTNUM2                           DDR_PWR_BASE + 0x018C
#define DDR_PWR_DDRBUSLPCTL                               DDR_PWR_BASE + 0x0190
#define DDR_PWR_DDRBUSLPCNTNUM                            DDR_PWR_BASE + 0x0198
#define DDR_PWR_TOPCTRLBUSLPCTL                           DDR_PWR_BASE + 0x019C
#define DDR_PWR_TOPCTRLBUSLPCNTNUM                        DDR_PWR_BASE + 0x01A4
#define DDR_PWR_MEMCTLLPCTL                               DDR_PWR_BASE + 0x01A8
#define DDR_PWR_TL420LPCTL                                DDR_PWR_BASE + 0x01AC
/*¼Ä´æÆ÷Ãû*/
#define DDR_PWR_REVLOCK                                   DDR_PWR_BASE + 0x01C0
#define DDR_PWR_TL420BOOTADDRSEL                          DDR_PWR_BASE + 0x01C4
#define DDR_PWR_TL420_BOOT_ADDR1                          DDR_PWR_BASE + 0x01C8
#define DDR_PWR_TOPFSMCTL                                 DDR_PWR_BASE + 0x01CC
//#define DDR_PWR_DDR_INITIALIZED                           DDR_PWR_BASE + 0x01D0
#define DDR_PWR_PMU_CTL                                   DDR_PWR_BASE + 0x01D4
#define DDR_PWR_RESET                                     DDR_PWR_BASE + 0x01D8
#define DDR_PWR_NPDRAMCTL                                 DDR_PWR_BASE + 0x01DC
#define DDR_PWR_MEMCTL0EN                                 DDR_PWR_BASE + 0x01E0
#define DDR_PWR_MEMCTL1EN                                 DDR_PWR_BASE + 0x01E4
#define DDR_PWR_MEMCTLIFCTL                               DDR_PWR_BASE + 0x01E8
#define DDR_PWR_CPCLK26MSEL                               DDR_PWR_BASE + 0x01EC
#define DDR_PWR_CDBGPWRUPREQMASK                          DDR_PWR_BASE + 0x01F0
#define DDR_PWR_RESET2                                    DDR_PWR_BASE + 0x01F4
#define DDR_PWR_TL420CTL                                  DDR_PWR_BASE + 0x01F8
#define DDR_PWR_APOSCCTL                                  DDR_PWR_BASE + 0x01FC
#define DDR_PWR_CPOSCCTL                                  DDR_PWR_BASE + 0x0200
#define DDR_PWR_CPDBGCTL                                  DDR_PWR_BASE + 0x0204
#define DDR_PWR_ATSPECIALTIME                             DDR_PWR_BASE + 0x0208
#define DDR_PWR_ATSPECIALTIMEMASK                         DDR_PWR_BASE + 0x020C
#define DDR_PWR_UARTCONNECTSEL                            DDR_PWR_BASE + 0x0214
#define DDR_PWR_A7GTMCTL                                  DDR_PWR_BASE + 0x0218

/*******************************************************************************
COM_UART registers' address on LC1860
********************************************************************************/
#define COM_UART_RBR                                      COM_UART_BASE + 0x0000
#define COM_UART_THR                                      COM_UART_BASE + 0x0000
#define COM_UART_DLL                                      COM_UART_BASE + 0x0000
#define COM_UART_IER                                      COM_UART_BASE + 0x0004
#define COM_UART_DLH                                      COM_UART_BASE + 0x0004
#define COM_UART_IIR                                      COM_UART_BASE + 0x0008
#define COM_UART_FCR                                      COM_UART_BASE + 0x0008
#define COM_UART_TCR                                      COM_UART_BASE + 0x000C
#define COM_UART_TSR                                      COM_UART_BASE + 0x0014
#define COM_UART_USR                                      COM_UART_BASE + 0x007C

/*******************************************************************************
GPIO registers' address on LC1860
********************************************************************************/
#define GPIO_PORT_DR0                                     GPIO_BASE + 0x0000
#define GPIO_PORT_DR1                                     GPIO_BASE + 0x0004
#define GPIO_PORT_DR2                                     GPIO_BASE + 0x0008
#define GPIO_PORT_DR3                                     GPIO_BASE + 0x000C
#define GPIO_PORT_DR4                                     GPIO_BASE + 0x0010
#define GPIO_PORT_DR5                                     GPIO_BASE + 0x0014
#define GPIO_PORT_DR6                                     GPIO_BASE + 0x0018
#define GPIO_PORT_DR7                                     GPIO_BASE + 0x001C
#define GPIO_PORT_DR8                                     GPIO_BASE + 0x0020
#define GPIO_PORT_DR9                                     GPIO_BASE + 0x0024
#define GPIO_PORT_DR10                                    GPIO_BASE + 0x0028
#define GPIO_PORT_DR11                                    GPIO_BASE + 0x002C
#define GPIO_PORT_DR12                                    GPIO_BASE + 0x0030
#define GPIO_PORT_DR13                                    GPIO_BASE + 0x0034
#define GPIO_PORT_DR14                                    GPIO_BASE + 0x0038
#define GPIO_PORT_DR15                                    GPIO_BASE + 0x003C
#define GPIO_PORT_DDR0                                    GPIO_BASE + 0x0040
#define GPIO_PORT_DDR1                                    GPIO_BASE + 0x0044
#define GPIO_PORT_DDR2                                    GPIO_BASE + 0x0048
#define GPIO_PORT_DDR3                                    GPIO_BASE + 0x004C
#define GPIO_PORT_DDR4                                    GPIO_BASE + 0x0050
#define GPIO_PORT_DDR5                                    GPIO_BASE + 0x0054
#define GPIO_PORT_DDR6                                    GPIO_BASE + 0x0058
#define GPIO_PORT_DDR7                                    GPIO_BASE + 0x005C
#define GPIO_PORT_DDR8                                    GPIO_BASE + 0x0060
#define GPIO_PORT_DDR9                                    GPIO_BASE + 0x0064
#define GPIO_PORT_DDR10                                   GPIO_BASE + 0x0068
#define GPIO_PORT_DDR11                                   GPIO_BASE + 0x006C
#define GPIO_PORT_DDR12                                   GPIO_BASE + 0x0070
#define GPIO_PORT_DDR13                                   GPIO_BASE + 0x0074
#define GPIO_PORT_DDR14                                   GPIO_BASE + 0x0078
#define GPIO_PORT_DDR15                                   GPIO_BASE + 0x007C
#define GPIO_EXT_PORT0                                    GPIO_BASE + 0x0080
#define GPIO_EXT_PORT1                                    GPIO_BASE + 0x0084
#define GPIO_EXT_PORT2                                    GPIO_BASE + 0x0088
#define GPIO_EXT_PORT3                                    GPIO_BASE + 0x008C
#define GPIO_EXT_PORT4                                    GPIO_BASE + 0x0090
#define GPIO_EXT_PORT5                                    GPIO_BASE + 0x0094
#define GPIO_EXT_PORT6                                    GPIO_BASE + 0x0098
#define GPIO_EXT_PORT7                                    GPIO_BASE + 0x009C
#define GPIO_INTR_CTRL0                                   GPIO_BASE + 0x00A0
#define GPIO_INTR_CTRL1                                   GPIO_BASE + 0x00A4
#define GPIO_INTR_CTRL2                                   GPIO_BASE + 0x00A8
#define GPIO_INTR_CTRL3                                   GPIO_BASE + 0x00AC
#define GPIO_INTR_CTRL4                                   GPIO_BASE + 0x00B0
#define GPIO_INTR_CTRL5                                   GPIO_BASE + 0x00B4
#define GPIO_INTR_CTRL6                                   GPIO_BASE + 0x00B8
#define GPIO_INTR_CTRL7                                   GPIO_BASE + 0x00BC
#define GPIO_INTR_CTRL8                                   GPIO_BASE + 0x00C0
#define GPIO_INTR_CTRL9                                   GPIO_BASE + 0x00C4
#define GPIO_INTR_CTRL10                                  GPIO_BASE + 0x00C8
#define GPIO_INTR_CTRL11                                  GPIO_BASE + 0x00CC
#define GPIO_INTR_CTRL12                                  GPIO_BASE + 0x00D0
#define GPIO_INTR_CTRL13                                  GPIO_BASE + 0x00D4
#define GPIO_INTR_CTRL14                                  GPIO_BASE + 0x00D8
#define GPIO_INTR_CTRL15                                  GPIO_BASE + 0x00DC
#define GPIO_INTR_CTRL16                                  GPIO_BASE + 0x00E0
#define GPIO_INTR_CTRL17                                  GPIO_BASE + 0x00E4
#define GPIO_INTR_CTRL18                                  GPIO_BASE + 0x00E8
#define GPIO_INTR_CTRL19                                  GPIO_BASE + 0x00EC
#define GPIO_INTR_CTRL20                                  GPIO_BASE + 0x00F0
#define GPIO_INTR_CTRL21                                  GPIO_BASE + 0x00F4
#define GPIO_INTR_CTRL22                                  GPIO_BASE + 0x00F8
#define GPIO_INTR_CTRL23                                  GPIO_BASE + 0x00FC
#define GPIO_INTR_CTRL24                                  GPIO_BASE + 0x0100
#define GPIO_INTR_CTRL25                                  GPIO_BASE + 0x0104
#define GPIO_INTR_CTRL26                                  GPIO_BASE + 0x0108
#define GPIO_INTR_CTRL27                                  GPIO_BASE + 0x010C
#define GPIO_INTR_CTRL28                                  GPIO_BASE + 0x0110
#define GPIO_INTR_CTRL29                                  GPIO_BASE + 0x0114
#define GPIO_INTR_CTRL30                                  GPIO_BASE + 0x0118
#define GPIO_INTR_CTRL31                                  GPIO_BASE + 0x011C
#define GPIO_INTR_CTRL32                                  GPIO_BASE + 0x0120
#define GPIO_INTR_CTRL33                                  GPIO_BASE + 0x0124
#define GPIO_INTR_CTRL34                                  GPIO_BASE + 0x0128
#define GPIO_INTR_CTRL35                                  GPIO_BASE + 0x012C
#define GPIO_INTR_CTRL36                                  GPIO_BASE + 0x0130
#define GPIO_INTR_CTRL37                                  GPIO_BASE + 0x0134
#define GPIO_INTR_CTRL38                                  GPIO_BASE + 0x0138
#define GPIO_INTR_CTRL39                                  GPIO_BASE + 0x013C
#define GPIO_INTR_CTRL40                                  GPIO_BASE + 0x0140
#define GPIO_INTR_CTRL41                                  GPIO_BASE + 0x0144
#define GPIO_INTR_CTRL42                                  GPIO_BASE + 0x0148
#define GPIO_INTR_CTRL43                                  GPIO_BASE + 0x014C
#define GPIO_INTR_CTRL44                                  GPIO_BASE + 0x0150
#define GPIO_INTR_CTRL45                                  GPIO_BASE + 0x0154
#define GPIO_INTR_CTRL46                                  GPIO_BASE + 0x0158
#define GPIO_INTR_CTRL47                                  GPIO_BASE + 0x015C
#define GPIO_INTR_CTRL48                                  GPIO_BASE + 0x0160
#define GPIO_INTR_CTRL49                                  GPIO_BASE + 0x0164
#define GPIO_INTR_CTRL50                                  GPIO_BASE + 0x0168
#define GPIO_INTR_CTRL51                                  GPIO_BASE + 0x016C
#define GPIO_INTR_CTRL52                                  GPIO_BASE + 0x0170
#define GPIO_INTR_CTRL53                                  GPIO_BASE + 0x0174
#define GPIO_INTR_CTRL54                                  GPIO_BASE + 0x0178
#define GPIO_INTR_CTRL55                                  GPIO_BASE + 0x017C
#define GPIO_INTR_CTRL56                                  GPIO_BASE + 0x0180
#define GPIO_INTR_CTRL57                                  GPIO_BASE + 0x0184
#define GPIO_INTR_CTRL58                                  GPIO_BASE + 0x0188
#define GPIO_INTR_CTRL59                                  GPIO_BASE + 0x018C
#define GPIO_INTR_CTRL60                                  GPIO_BASE + 0x0190
#define GPIO_INTR_CTRL61                                  GPIO_BASE + 0x0194
#define GPIO_INTR_CTRL62                                  GPIO_BASE + 0x0198
#define GPIO_INTR_CTRL63                                  GPIO_BASE + 0x019C
#define GPIO_DEBOUNCE0                                    GPIO_BASE + 0x01A0
#define GPIO_DEBOUNCE1                                    GPIO_BASE + 0x01A4
#define GPIO_DEBOUNCE2                                    GPIO_BASE + 0x01A8
#define GPIO_DEBOUNCE3                                    GPIO_BASE + 0x01AC
#define GPIO_DEBOUNCE4                                    GPIO_BASE + 0x01B0
#define GPIO_DEBOUNCE5                                    GPIO_BASE + 0x01B4
#define GPIO_DEBOUNCE6                                    GPIO_BASE + 0x01B8
#define GPIO_DEBOUNCE7                                    GPIO_BASE + 0x01BC
#define GPIO_DEBOUNCE8                                    GPIO_BASE + 0x01C0
#define GPIO_DEBOUNCE9                                    GPIO_BASE + 0x01C4
#define GPIO_DEBOUNCE10                                   GPIO_BASE + 0x01C8
#define GPIO_DEBOUNCE11                                   GPIO_BASE + 0x01CC
#define GPIO_DEBOUNCE12                                   GPIO_BASE + 0x01D0
#define GPIO_DEBOUNCE13                                   GPIO_BASE + 0x01D4
#define GPIO_DEBOUNCE14                                   GPIO_BASE + 0x01D8
#define GPIO_DEBOUNCE15                                   GPIO_BASE + 0x01DC
#define GPIO_INTR_RAW0                                    GPIO_BASE + 0x01E0
#define GPIO_INTR_RAW1                                    GPIO_BASE + 0x01E4
#define GPIO_INTR_RAW2                                    GPIO_BASE + 0x01E8
#define GPIO_INTR_RAW3                                    GPIO_BASE + 0x01EC
#define GPIO_INTR_RAW4                                    GPIO_BASE + 0x01F0
#define GPIO_INTR_RAW5                                    GPIO_BASE + 0x01F4
#define GPIO_INTR_RAW6                                    GPIO_BASE + 0x01F8
#define GPIO_INTR_RAW7                                    GPIO_BASE + 0x01FC
#define GPIO_INTR_CLR0                                    GPIO_BASE + 0x0200
#define GPIO_INTR_CLR1                                    GPIO_BASE + 0x0204
#define GPIO_INTR_CLR2                                    GPIO_BASE + 0x0208
#define GPIO_INTR_CLR3                                    GPIO_BASE + 0x020C
#define GPIO_INTR_CLR4                                    GPIO_BASE + 0x0210
#define GPIO_INTR_CLR5                                    GPIO_BASE + 0x0214
#define GPIO_INTR_CLR6                                    GPIO_BASE + 0x0218
#define GPIO_INTR_CLR7                                    GPIO_BASE + 0x021C
#define GPIO_INTR_MASK_C00                                GPIO_BASE + 0x0220
#define GPIO_INTR_MASK_C01                                GPIO_BASE + 0x0224
#define GPIO_INTR_MASK_C02                                GPIO_BASE + 0x0228
#define GPIO_INTR_MASK_C03                                GPIO_BASE + 0x022C
#define GPIO_INTR_MASK_C04                                GPIO_BASE + 0x0230
#define GPIO_INTR_MASK_C05                                GPIO_BASE + 0x0234
#define GPIO_INTR_MASK_C06                                GPIO_BASE + 0x0238
#define GPIO_INTR_MASK_C07                                GPIO_BASE + 0x023C
#define GPIO_INTR_MASK_C08                                GPIO_BASE + 0x0240
#define GPIO_INTR_MASK_C09                                GPIO_BASE + 0x0244
#define GPIO_INTR_MASK_C010                               GPIO_BASE + 0x0248
#define GPIO_INTR_MASK_C011                               GPIO_BASE + 0x024C
#define GPIO_INTR_MASK_C012                               GPIO_BASE + 0x0250
#define GPIO_INTR_MASK_C013                               GPIO_BASE + 0x0254
#define GPIO_INTR_MASK_C014                               GPIO_BASE + 0x0258
#define GPIO_INTR_MASK_C015                               GPIO_BASE + 0x025C
#define GPIO_INTR_STATUS_C00                              GPIO_BASE + 0x0260
#define GPIO_INTR_STATUS_C01                              GPIO_BASE + 0x0264
#define GPIO_INTR_STATUS_C02                              GPIO_BASE + 0x0268
#define GPIO_INTR_STATUS_C03                              GPIO_BASE + 0x026C
#define GPIO_INTR_STATUS_C04                              GPIO_BASE + 0x0270
#define GPIO_INTR_STATUS_C05                              GPIO_BASE + 0x0274
#define GPIO_INTR_STATUS_C06                              GPIO_BASE + 0x0278
#define GPIO_INTR_STATUS_C07                              GPIO_BASE + 0x027C
#define GPIO_INTR_MASK_C10                                GPIO_BASE + 0x0280
#define GPIO_INTR_MASK_C11                                GPIO_BASE + 0x0284
#define GPIO_INTR_MASK_C12                                GPIO_BASE + 0x0288
#define GPIO_INTR_MASK_C13                                GPIO_BASE + 0x028C
#define GPIO_INTR_MASK_C14                                GPIO_BASE + 0x0290
#define GPIO_INTR_MASK_C15                                GPIO_BASE + 0x0294
#define GPIO_INTR_MASK_C16                                GPIO_BASE + 0x0298
#define GPIO_INTR_MASK_C17                                GPIO_BASE + 0x029C
#define GPIO_INTR_MASK_C18                                GPIO_BASE + 0x02A0
#define GPIO_INTR_MASK_C19                                GPIO_BASE + 0x02A4
#define GPIO_INTR_MASK_C110                               GPIO_BASE + 0x02A8
#define GPIO_INTR_MASK_C111                               GPIO_BASE + 0x02AC
#define GPIO_INTR_MASK_C112                               GPIO_BASE + 0x02B0
#define GPIO_INTR_MASK_C113                               GPIO_BASE + 0x02B4
#define GPIO_INTR_MASK_C114                               GPIO_BASE + 0x02B8
#define GPIO_INTR_MASK_C115                               GPIO_BASE + 0x02BC
#define GPIO_INTR_STATUS_C10                              GPIO_BASE + 0x02C0
#define GPIO_INTR_STATUS_C11                              GPIO_BASE + 0x02C4
#define GPIO_INTR_STATUS_C12                              GPIO_BASE + 0x02C8
#define GPIO_INTR_STATUS_C13                              GPIO_BASE + 0x02CC
#define GPIO_INTR_STATUS_C14                              GPIO_BASE + 0x02D0
#define GPIO_INTR_STATUS_C15                              GPIO_BASE + 0x02D4
#define GPIO_INTR_STATUS_C16                              GPIO_BASE + 0x02D8
#define GPIO_INTR_STATUS_C17                              GPIO_BASE + 0x02DC
#define GPIO_INTR_MASK_C20                                GPIO_BASE + 0x02E0
#define GPIO_INTR_MASK_C21                                GPIO_BASE + 0x02E4
#define GPIO_INTR_MASK_C22                                GPIO_BASE + 0x02E8
#define GPIO_INTR_MASK_C23                                GPIO_BASE + 0x02EC
#define GPIO_INTR_MASK_C24                                GPIO_BASE + 0x02F0
#define GPIO_INTR_MASK_C25                                GPIO_BASE + 0x02F4
#define GPIO_INTR_MASK_C26                                GPIO_BASE + 0x02F8
#define GPIO_INTR_MASK_C27                                GPIO_BASE + 0x02FC
#define GPIO_INTR_MASK_C28                                GPIO_BASE + 0x0300
#define GPIO_INTR_MASK_C29                                GPIO_BASE + 0x0304
#define GPIO_INTR_MASK_C210                               GPIO_BASE + 0x0308
#define GPIO_INTR_MASK_C211                               GPIO_BASE + 0x030C
#define GPIO_INTR_MASK_C212                               GPIO_BASE + 0x0310
#define GPIO_INTR_MASK_C213                               GPIO_BASE + 0x0314
#define GPIO_INTR_MASK_C214                               GPIO_BASE + 0x0318
#define GPIO_INTR_MASK_C215                               GPIO_BASE + 0x031C
#define GPIO_INTR_STATUS_C20                              GPIO_BASE + 0x0320
#define GPIO_INTR_STATUS_C21                              GPIO_BASE + 0x0324
#define GPIO_INTR_STATUS_C22                              GPIO_BASE + 0x0328
#define GPIO_INTR_STATUS_C23                              GPIO_BASE + 0x032C
#define GPIO_INTR_STATUS_C24                              GPIO_BASE + 0x0330
#define GPIO_INTR_STATUS_C25                              GPIO_BASE + 0x0334
#define GPIO_INTR_STATUS_C26                              GPIO_BASE + 0x0338
#define GPIO_INTR_STATUS_C27                              GPIO_BASE + 0x033C
#define GPIO_INTR_MASK_C30                                GPIO_BASE + 0x0340
#define GPIO_INTR_MASK_C31                                GPIO_BASE + 0x0344
#define GPIO_INTR_MASK_C32                                GPIO_BASE + 0x0348
#define GPIO_INTR_MASK_C33                                GPIO_BASE + 0x034C
#define GPIO_INTR_MASK_C34                                GPIO_BASE + 0x0350
#define GPIO_INTR_MASK_C35                                GPIO_BASE + 0x0354
#define GPIO_INTR_MASK_C36                                GPIO_BASE + 0x0358
#define GPIO_INTR_MASK_C37                                GPIO_BASE + 0x035C
#define GPIO_INTR_MASK_C38                                GPIO_BASE + 0x0360
#define GPIO_INTR_MASK_C39                                GPIO_BASE + 0x0364
#define GPIO_INTR_MASK_C310                               GPIO_BASE + 0x0368
#define GPIO_INTR_MASK_C311                               GPIO_BASE + 0x036C
#define GPIO_INTR_MASK_C312                               GPIO_BASE + 0x0370
#define GPIO_INTR_MASK_C313                               GPIO_BASE + 0x0374
#define GPIO_INTR_MASK_C314                               GPIO_BASE + 0x0378
#define GPIO_INTR_MASK_C315                               GPIO_BASE + 0x037C
#define GPIO_INTR_STATUS_C30                              GPIO_BASE + 0x0380
#define GPIO_INTR_STATUS_C31                              GPIO_BASE + 0x0384
#define GPIO_INTR_STATUS_C32                              GPIO_BASE + 0x0388
#define GPIO_INTR_STATUS_C33                              GPIO_BASE + 0x038C
#define GPIO_INTR_STATUS_C34                              GPIO_BASE + 0x0390
#define GPIO_INTR_STATUS_C35                              GPIO_BASE + 0x0394
#define GPIO_INTR_STATUS_C36                              GPIO_BASE + 0x0398
#define GPIO_INTR_STATUS_C37                              GPIO_BASE + 0x039C
#define GPIO_INTR_MASK_C40                                GPIO_BASE + 0x03A0
#define GPIO_INTR_MASK_C41                                GPIO_BASE + 0x03A4
#define GPIO_INTR_MASK_C42                                GPIO_BASE + 0x03A8
#define GPIO_INTR_MASK_C43                                GPIO_BASE + 0x03AC
#define GPIO_INTR_MASK_C44                                GPIO_BASE + 0x03B0
#define GPIO_INTR_MASK_C45                                GPIO_BASE + 0x03B4
#define GPIO_INTR_MASK_C46                                GPIO_BASE + 0x03B8
#define GPIO_INTR_MASK_C47                                GPIO_BASE + 0x03BC
#define GPIO_INTR_MASK_C48                                GPIO_BASE + 0x03C0
#define GPIO_INTR_MASK_C49                                GPIO_BASE + 0x03C4
#define GPIO_INTR_MASK_C410                               GPIO_BASE + 0x03C8
#define GPIO_INTR_MASK_C411                               GPIO_BASE + 0x03CC
#define GPIO_INTR_MASK_C412                               GPIO_BASE + 0x03D0
#define GPIO_INTR_MASK_C413                               GPIO_BASE + 0x03D4
#define GPIO_INTR_MASK_C414                               GPIO_BASE + 0x03D8
#define GPIO_INTR_MASK_C415                               GPIO_BASE + 0x03DC
#define GPIO_INTR_STATUS_C40                              GPIO_BASE + 0x03E0
#define GPIO_INTR_STATUS_C41                              GPIO_BASE + 0x03E4
#define GPIO_INTR_STATUS_C42                              GPIO_BASE + 0x03E8
#define GPIO_INTR_STATUS_C43                              GPIO_BASE + 0x03EC
#define GPIO_INTR_STATUS_C44                              GPIO_BASE + 0x03F0
#define GPIO_INTR_STATUS_C45                              GPIO_BASE + 0x03F4
#define GPIO_INTR_STATUS_C46                              GPIO_BASE + 0x03F8
#define GPIO_INTR_STATUS_C47                              GPIO_BASE + 0x03FC

/*******************************************************************************
COM_I2S registers' address on LC1860
********************************************************************************/
#define COM_I2S_SCLK_CFG                                  COM_I2S_BASE + 0x0000
#define COM_I2S_FSYNC_CFG                                 COM_I2S_BASE + 0x0004
#define COM_I2S_FIFO_STA                                  COM_I2S_BASE + 0x0008
#define COM_I2S_MODE                                      COM_I2S_BASE + 0x0010
#define COM_I2S_REC_FIFO                                  COM_I2S_BASE + 0x0014
#define COM_I2S_TRAN_FIFO                                 COM_I2S_BASE + 0x0018

/*******************************************************************************
TL420_ICTL registers' address on LC1860
********************************************************************************/
#define TL420_ICTL_IRQ_INTEN                              TL420_ICTL_BASE + 0x0000
#define TL420_ICTL_IRQ_INTMASK                            TL420_ICTL_BASE + 0x0008
#define TL420_ICTL_IRQ_INTFORCE                           TL420_ICTL_BASE + 0x0010
#define TL420_ICTL_IRQ_RAWSTS                             TL420_ICTL_BASE + 0x0018
#define TL420_ICTL_IRQ_STATUS                             TL420_ICTL_BASE + 0x0020
#define TL420_ICTL_IRQ_MASKSTS                            TL420_ICTL_BASE + 0x0028
#define TL420_ICTL_IRQ_FINALSTS                           TL420_ICTL_BASE + 0x0030
#define TL420_ICTL_IRQ_VECTOR                             TL420_ICTL_BASE + 0x0038
#define TL420_ICTL_IRQ_VECTOR_0                           TL420_ICTL_BASE + 0x0040
#define TL420_ICTL_IRQ_VECTOR_1                           TL420_ICTL_BASE + 0x0048
#define TL420_ICTL_IRQ_VECTOR_2                           TL420_ICTL_BASE + 0x0050
#define TL420_ICTL_IRQ_VECTOR_3                           TL420_ICTL_BASE + 0x0058
#define TL420_ICTL_IRQ_VECTOR_4                           TL420_ICTL_BASE + 0x0060
#define TL420_ICTL_IRQ_VECTOR_5                           TL420_ICTL_BASE + 0x0068
#define TL420_ICTL_IRQ_VECTOR_6                           TL420_ICTL_BASE + 0x0070
#define TL420_ICTL_IRQ_VECTOR_7                           TL420_ICTL_BASE + 0x0078
#define TL420_ICTL_IRQ_VECTOR_8                           TL420_ICTL_BASE + 0x0080
#define TL420_ICTL_IRQ_VECTOR_9                           TL420_ICTL_BASE + 0x0088
#define TL420_ICTL_IRQ_VECTOR_10                          TL420_ICTL_BASE + 0x0090
#define TL420_ICTL_IRQ_VECTOR_11                          TL420_ICTL_BASE + 0x0098
#define TL420_ICTL_IRQ_VECTOR_12                          TL420_ICTL_BASE + 0x00A0
#define TL420_ICTL_IRQ_VECTOR_13                          TL420_ICTL_BASE + 0x00A8
#define TL420_ICTL_IRQ_VECTOR_14                          TL420_ICTL_BASE + 0x00B0
#define TL420_ICTL_IRQ_VECTOR_15                          TL420_ICTL_BASE + 0x00B8
#define TL420_ICTL_IRQ_PLEVEL                             TL420_ICTL_BASE + 0x00D8
#define TL420_ICTL_IRQ_INTERAL_PLEVEL                     TL420_ICTL_BASE + 0x00DC
#define TL420_ICTL_IRQ_PLEVEL_0                           TL420_ICTL_BASE + 0x00E8
#define TL420_ICTL_IRQ_PLEVEL_1                           TL420_ICTL_BASE + 0x00EC
#define TL420_ICTL_IRQ_PLEVEL_2                           TL420_ICTL_BASE + 0x00F0
#define TL420_ICTL_IRQ_PLEVEL_3                           TL420_ICTL_BASE + 0x00F4
#define TL420_ICTL_IRQ_PLEVEL_4                           TL420_ICTL_BASE + 0x00F8
#define TL420_ICTL_IRQ_PLEVEL_5                           TL420_ICTL_BASE + 0x00FC
#define TL420_ICTL_IRQ_PLEVEL_6                           TL420_ICTL_BASE + 0x0100
#define TL420_ICTL_IRQ_PLEVEL_7                           TL420_ICTL_BASE + 0x0104
#define TL420_ICTL_IRQ_PLEVEL_8                           TL420_ICTL_BASE + 0x0108
#define TL420_ICTL_IRQ_PLEVEL_9                           TL420_ICTL_BASE + 0x010C
#define TL420_ICTL_IRQ_PLEVEL_10                          TL420_ICTL_BASE + 0x0110
#define TL420_ICTL_IRQ_PLEVEL_11                          TL420_ICTL_BASE + 0x0114
#define TL420_ICTL_IRQ_PLEVEL_12                          TL420_ICTL_BASE + 0x0118

/*******************************************************************************
CP_MAILBOX registers' address on LC1860
********************************************************************************/
#define CP_MAILBOX_A72X1643_INTR_SET                      CP_MAILBOX_BASE + 0x0000
#define CP_MAILBOX_A72X1643_INTR_EN                       CP_MAILBOX_BASE + 0x0004
#define CP_MAILBOX_A72X1643_INTR_STA                      CP_MAILBOX_BASE + 0x0008
#define CP_MAILBOX_A72X1643_INTR_STA_RAW                  CP_MAILBOX_BASE + 0x000C
#define CP_MAILBOX_X1643_A7_INTR_SET                      CP_MAILBOX_BASE + 0x0010
#define CP_MAILBOX_X1643_A7_INTR_EN                       CP_MAILBOX_BASE + 0x0014
#define CP_MAILBOX_X1643_A7_INTR_STA                      CP_MAILBOX_BASE + 0x0018
#define CP_MAILBOX_X1643_A7_INTR_STA_RAW                  CP_MAILBOX_BASE + 0x001C
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_XC4210_A7_INTR_SET                     CP_MAILBOX_BASE + 0x0020
#define CP_MAILBOX_XC4210_A7_INTR_EN                      CP_MAILBOX_BASE + 0x0024
#define CP_MAILBOX_XC4210_A7_INTR_STA                     CP_MAILBOX_BASE + 0x0028
#define CP_MAILBOX_XC4210_A7_INTR_STA_RAW                 CP_MAILBOX_BASE + 0x002C
#define CP_MAILBOX_XC4210_X1643_INTR_SET                  CP_MAILBOX_BASE + 0x0030
#define CP_MAILBOX_XC4210_X1643_INTR_EN                   CP_MAILBOX_BASE + 0x0034
#define CP_MAILBOX_XC4210_X1643_INTR_STA                  CP_MAILBOX_BASE + 0x0038
#define CP_MAILBOX_XC4210_X1643_INTR_STA_RAW              CP_MAILBOX_BASE + 0x003C
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_A7_BOOT_CTL                            CP_MAILBOX_BASE + 0x0040
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_X1643_BOOT_ADDR                        CP_MAILBOX_BASE + 0x0050
#define CP_MAILBOX_XC4210_BOOT_ADDR                       CP_MAILBOX_BASE + 0x0054
#define CP_MAILBOX_INTR_MASK                              CP_MAILBOX_BASE + 0x0058
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_LP_MODE_CTRL                           CP_MAILBOX_BASE + 0x0070
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_X1643_STA                              CP_MAILBOX_BASE + 0x0080
#define CP_MAILBOX_X1643_EXCEPT_INTR_EN                   CP_MAILBOX_BASE + 0x0084
#define CP_MAILBOX_X1643_EXCEPT_INTR_STA                  CP_MAILBOX_BASE + 0x0088
#define CP_MAILBOX_X1643_EXCEPT_INTR_RAW                  CP_MAILBOX_BASE + 0x008C
#define CP_MAILBOX_X1643_EXT_BP                           CP_MAILBOX_BASE + 0x0090
#define CP_MAILBOX_X1643_CTRL                             CP_MAILBOX_BASE + 0x0094
#define CP_MAILBOX_X1643_EXCEPT_CTRL                      CP_MAILBOX_BASE + 0x0098
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_XC4210_STA                             CP_MAILBOX_BASE + 0x00A0
#define CP_MAILBOX_XC4210_NMI_INTR_EN                     CP_MAILBOX_BASE + 0x00A4
#define CP_MAILBOX_XC4210_NMI_INTR_STA                    CP_MAILBOX_BASE + 0x00A8
#define CP_MAILBOX_XC4210_NMI_INTR_RAW                    CP_MAILBOX_BASE + 0x00AC
#define CP_MAILBOX_XC4210_GPIN                            CP_MAILBOX_BASE + 0x00B0
#define CP_MAILBOX_XC4210_INTR_EN                         CP_MAILBOX_BASE + 0x00B4
#define CP_MAILBOX_XC4210_INTR_STA                        CP_MAILBOX_BASE + 0x00B8
#define CP_MAILBOX_XC4210_INTR_RAW                        CP_MAILBOX_BASE + 0x00BC
#define CP_MAILBOX_XC4210_EXT_BP                          CP_MAILBOX_BASE + 0x00C0
#define CP_MAILBOX_XC4210_CTRL                            CP_MAILBOX_BASE + 0x00C4
#define CP_MAILBOX_XC4210_EXCEPT_CTRL                     CP_MAILBOX_BASE + 0x00C8
/*¼Ä´æÆ÷Ãû*/
#define CP_MAILBOX_CPA7_CTRL                              CP_MAILBOX_BASE + 0x00D0
#define CP_MAILBOX_XC4210_BOOT                            CP_MAILBOX_BASE + 0x00D4
#define CP_MAILBOX_XC4210_CACHE_CTRL                      CP_MAILBOX_BASE + 0x00D8
#define CP_MAILBOX_LTE_ACC_SEL                            CP_MAILBOX_BASE + 0x00DC
#define CP_MAILBOX_X1643_BOOT                             CP_MAILBOX_BASE + 0x00E0
#define CP_MAILBOX_TIMER1_CTRL                            CP_MAILBOX_BASE + 0x00E4
#define CP_MAILBOX_TIMER2_CTRL                            CP_MAILBOX_BASE + 0x00E8
#define CP_MAILBOX_TIMER3_CTRL                            CP_MAILBOX_BASE + 0x00EC
#define CP_MAILBOX_TIMER4_CTRL                            CP_MAILBOX_BASE + 0x00F0
#define CP_MAILBOX_X1643_NMI_EN                           CP_MAILBOX_BASE + 0x00F4
#define CP_MAILBOX_XC4210_NMI_EN                          CP_MAILBOX_BASE + 0x00F8
#define CP_MAILBOX_BUS_CACHE_CTRL                         CP_MAILBOX_BASE + 0x00FC

/*******************************************************************************
XC4210_ICTL registers' address on LC1860
********************************************************************************/
#define XC4210_ICTL_IRQ_INTEN                             XC4210_ICTL_BASE + 0x0000
#define XC4210_ICTL_IRQ_INTMASK                           XC4210_ICTL_BASE + 0x0008
#define XC4210_ICTL_IRQ_INTFORCE                          XC4210_ICTL_BASE + 0x0010
#define XC4210_ICTL_IRQ_RAWSTS                            XC4210_ICTL_BASE + 0x0018
#define XC4210_ICTL_IRQ_STATUS                            XC4210_ICTL_BASE + 0x0020
#define XC4210_ICTL_IRQ_MASKSTS                           XC4210_ICTL_BASE + 0x0028
#define XC4210_ICTL_IRQ_FINALSTS                          XC4210_ICTL_BASE + 0x0030
#define XC4210_ICTL_IRQ_VECTOR                            XC4210_ICTL_BASE + 0x0038
#define XC4210_ICTL_IRQ_VECTOR_0                          XC4210_ICTL_BASE + 0x0040
#define XC4210_ICTL_IRQ_VECTOR_1                          XC4210_ICTL_BASE + 0x0048
#define XC4210_ICTL_IRQ_VECTOR_2                          XC4210_ICTL_BASE + 0x0050
#define XC4210_ICTL_IRQ_VECTOR_3                          XC4210_ICTL_BASE + 0x0058
#define XC4210_ICTL_IRQ_VECTOR_4                          XC4210_ICTL_BASE + 0x0060
#define XC4210_ICTL_IRQ_VECTOR_5                          XC4210_ICTL_BASE + 0x0068
#define XC4210_ICTL_IRQ_VECTOR_6                          XC4210_ICTL_BASE + 0x0070
#define XC4210_ICTL_IRQ_VECTOR_7                          XC4210_ICTL_BASE + 0x0078
#define XC4210_ICTL_IRQ_VECTOR_8                          XC4210_ICTL_BASE + 0x0080
#define XC4210_ICTL_IRQ_VECTOR_9                          XC4210_ICTL_BASE + 0x0088
#define XC4210_ICTL_IRQ_VECTOR_10                         XC4210_ICTL_BASE + 0x0090
#define XC4210_ICTL_IRQ_VECTOR_11                         XC4210_ICTL_BASE + 0x0098
#define XC4210_ICTL_IRQ_VECTOR_12                         XC4210_ICTL_BASE + 0x00A0
#define XC4210_ICTL_IRQ_VECTOR_13                         XC4210_ICTL_BASE + 0x00A8
#define XC4210_ICTL_IRQ_VECTOR_14                         XC4210_ICTL_BASE + 0x00B0
#define XC4210_ICTL_IRQ_VECTOR_15                         XC4210_ICTL_BASE + 0x00B8
#define XC4210_ICTL_IRQ_PLEVEL                            XC4210_ICTL_BASE + 0x00D8
#define XC4210_ICTL_IRQ_INTERNAL_PLEVEL                   XC4210_ICTL_BASE + 0x00DC
#define XC4210_ICTL_IRQ_PLEVEL_0                          XC4210_ICTL_BASE + 0x00E8
#define XC4210_ICTL_IRQ_PLEVEL_1                          XC4210_ICTL_BASE + 0x00EC
#define XC4210_ICTL_IRQ_PLEVEL_2                          XC4210_ICTL_BASE + 0x00F0
#define XC4210_ICTL_IRQ_PLEVEL_3                          XC4210_ICTL_BASE + 0x00F4
#define XC4210_ICTL_IRQ_PLEVEL_4                          XC4210_ICTL_BASE + 0x00F8
#define XC4210_ICTL_IRQ_PLEVEL_5                          XC4210_ICTL_BASE + 0x00FC
#define XC4210_ICTL_IRQ_PLEVEL_6                          XC4210_ICTL_BASE + 0x0100
#define XC4210_ICTL_IRQ_PLEVEL_7                          XC4210_ICTL_BASE + 0x0104
#define XC4210_ICTL_IRQ_PLEVEL_8                          XC4210_ICTL_BASE + 0x0108
#define XC4210_ICTL_IRQ_PLEVEL_9                          XC4210_ICTL_BASE + 0x010C
#define XC4210_ICTL_IRQ_PLEVEL_10                         XC4210_ICTL_BASE + 0x0110
#define XC4210_ICTL_IRQ_PLEVEL_11                         XC4210_ICTL_BASE + 0x0114
#define XC4210_ICTL_IRQ_PLEVEL_12                         XC4210_ICTL_BASE + 0x0118
#define XC4210_ICTL_IRQ_PLEVEL_13                         XC4210_ICTL_BASE + 0x011C
#define XC4210_ICTL_IRQ_PLEVEL_14                         XC4210_ICTL_BASE + 0x0120
#define XC4210_ICTL_IRQ_PLEVEL_15                         XC4210_ICTL_BASE + 0x0124
#define XC4210_ICTL_IRQ_PLEVEL_16                         XC4210_ICTL_BASE + 0x0128
#define XC4210_ICTL_IRQ_PLEVEL_17                         XC4210_ICTL_BASE + 0x012C
#define XC4210_ICTL_IRQ_PLEVEL_18                         XC4210_ICTL_BASE + 0x0130
#define XC4210_ICTL_IRQ_PLEVEL_19                         XC4210_ICTL_BASE + 0x0134
#define XC4210_ICTL_IRQ_PLEVEL_20                         XC4210_ICTL_BASE + 0x0138
#define XC4210_ICTL_IRQ_PLEVEL_21                         XC4210_ICTL_BASE + 0x013C
#define XC4210_ICTL_IRQ_PLEVEL_22                         XC4210_ICTL_BASE + 0x0140
#define XC4210_ICTL_IRQ_PLEVEL_23                         XC4210_ICTL_BASE + 0x0144
#define XC4210_ICTL_IRQ_PLEVEL_24                         XC4210_ICTL_BASE + 0x0148
#define XC4210_ICTL_IRQ_PLEVEL_25                         XC4210_ICTL_BASE + 0x014C
#define XC4210_ICTL_IRQ_PLEVEL_26                         XC4210_ICTL_BASE + 0x0150
#define XC4210_ICTL_IRQ_PLEVEL_27                         XC4210_ICTL_BASE + 0x0154

/*******************************************************************************
X1643_ICTL registers' address on LC1860
********************************************************************************/
#define X1643_ICTL_IRQ_INTEN                              X1643_ICTL_BASE + 0x0000
#define X1643_ICTL_IRQ_INTMASK                            X1643_ICTL_BASE + 0x0008
#define X1643_ICTL_IRQ_INTFORCE                           X1643_ICTL_BASE + 0x0010
#define X1643_ICTL_IRQ_RAWSTS                             X1643_ICTL_BASE + 0x0018
#define X1643_ICTL_IRQ_STATUS                             X1643_ICTL_BASE + 0x0020
#define X1643_ICTL_IRQ_MASKSTS                            X1643_ICTL_BASE + 0x0028
#define X1643_ICTL_IRQ_FINALSTS                           X1643_ICTL_BASE + 0x0030
#define X1643_ICTL_IRQ_VECTOR                             X1643_ICTL_BASE + 0x0038
#define X1643_ICTL_IRQ_VECTOR_0                           X1643_ICTL_BASE + 0x0040
#define X1643_ICTL_IRQ_VECTOR_1                           X1643_ICTL_BASE + 0x0048
#define X1643_ICTL_IRQ_VECTOR_2                           X1643_ICTL_BASE + 0x0050
#define X1643_ICTL_IRQ_VECTOR_3                           X1643_ICTL_BASE + 0x0058
#define X1643_ICTL_IRQ_VECTOR_4                           X1643_ICTL_BASE + 0x0060
#define X1643_ICTL_IRQ_VECTOR_5                           X1643_ICTL_BASE + 0x0068
#define X1643_ICTL_IRQ_VECTOR_6                           X1643_ICTL_BASE + 0x0070
#define X1643_ICTL_IRQ_VECTOR_7                           X1643_ICTL_BASE + 0x0078
#define X1643_ICTL_IRQ_VECTOR_8                           X1643_ICTL_BASE + 0x0080
#define X1643_ICTL_IRQ_VECTOR_9                           X1643_ICTL_BASE + 0x0088
#define X1643_ICTL_IRQ_VECTOR_10                          X1643_ICTL_BASE + 0x0090
#define X1643_ICTL_IRQ_VECTOR_11                          X1643_ICTL_BASE + 0x0098
#define X1643_ICTL_IRQ_VECTOR_12                          X1643_ICTL_BASE + 0x00A0
#define X1643_ICTL_IRQ_VECTOR_13                          X1643_ICTL_BASE + 0x00A8
#define X1643_ICTL_IRQ_VECTOR_14                          X1643_ICTL_BASE + 0x00B0
#define X1643_ICTL_IRQ_VECTOR_15                          X1643_ICTL_BASE + 0x00B8
#define X1643_ICTL_IRQ_PLEVEL                             X1643_ICTL_BASE + 0x00D8
#define X1643_ICTL_IRQ_INTERNAL_PLEVEL                    X1643_ICTL_BASE + 0x00DC
#define X1643_ICTL_IRQ_PLEVEL_0                           X1643_ICTL_BASE + 0x00E8
#define X1643_ICTL_IRQ_PLEVEL_1                           X1643_ICTL_BASE + 0x00EC
#define X1643_ICTL_IRQ_PLEVEL_2                           X1643_ICTL_BASE + 0x00F0
#define X1643_ICTL_IRQ_PLEVEL_3                           X1643_ICTL_BASE + 0x00F4
#define X1643_ICTL_IRQ_PLEVEL_4                           X1643_ICTL_BASE + 0x00F8
#define X1643_ICTL_IRQ_PLEVEL_5                           X1643_ICTL_BASE + 0x00FC
#define X1643_ICTL_IRQ_PLEVEL_6                           X1643_ICTL_BASE + 0x0100
#define X1643_ICTL_IRQ_PLEVEL_7                           X1643_ICTL_BASE + 0x0104
#define X1643_ICTL_IRQ_PLEVEL_8                           X1643_ICTL_BASE + 0x0108
#define X1643_ICTL_IRQ_PLEVEL_9                           X1643_ICTL_BASE + 0x010C
#define X1643_ICTL_IRQ_PLEVEL_10                          X1643_ICTL_BASE + 0x0110
#define X1643_ICTL_IRQ_PLEVEL_11                          X1643_ICTL_BASE + 0x0114
#define X1643_ICTL_IRQ_PLEVEL_12                          X1643_ICTL_BASE + 0x0118
#define X1643_ICTL_IRQ_PLEVEL_13                          X1643_ICTL_BASE + 0x011C
#define X1643_ICTL_IRQ_PLEVEL_14                          X1643_ICTL_BASE + 0x0120
#define X1643_ICTL_IRQ_PLEVEL_15                          X1643_ICTL_BASE + 0x0124
#define X1643_ICTL_IRQ_PLEVEL_16                          X1643_ICTL_BASE + 0x0128
#define X1643_ICTL_IRQ_PLEVEL_17                          X1643_ICTL_BASE + 0x012C

/*******************************************************************************
LTEU registers' address on LC1860
********************************************************************************/
#define LTEU_VERSION                                      LTEU_BASE + 0x0000
#define LTEU_RESET                                        LTEU_BASE + 0x0004
#define LTEU_DSP_INT_RAW_1643                             LTEU_BASE + 0x0008
#define LTEU_DSP_INT_EN_1643                              LTEU_BASE + 0x000C
#define LTEU_DSP_INT_MASK_1643                            LTEU_BASE + 0x0010
#define LTEU_UL_MODE                                      LTEU_BASE + 0x0014
#define LTEU_DSP_INT_STATUS_1643                          LTEU_BASE + 0x0018
#define LTEU_ACC_ACT                                      LTEU_BASE + 0x001C
#define LTEU_UBP_START                                    LTEU_BASE + 0x0020
#define LTEU_USP_START                                    LTEU_BASE + 0x0024
#define LTEU_UBP_AB_FLAG                                  LTEU_BASE + 0x0028
#define LTEU_USP_AB_FLAG                                  LTEU_BASE + 0x002C
#define LTEU_DSP_INT_RAW_4210                             LTEU_BASE + 0x0030
#define LTEU_DSP_INT_EN_4210                              LTEU_BASE + 0x0034
#define LTEU_DSP_INT_MASK_4210                            LTEU_BASE + 0x0038
#define LTEU_DSP_INT_STATUS_4210                          LTEU_BASE + 0x003C
/*¼Ä´æÆ÷Ãû*/
#define LTEU_CH_STATUS                                    LTEU_BASE + 0xB0000
#define LTEU_CH0_CTRL                                     LTEU_BASE + 0xB0010
#define LTEU_CH0_CFG                                      LTEU_BASE + 0xB0014
#define LTEU_CH0_SAR                                      LTEU_BASE + 0xB0018
#define LTEU_CH0_CUR_SAR                                  LTEU_BASE + 0xB001C
#define LTEU_TXFIFO_STATUS                                LTEU_BASE + 0xB0060
/*¼Ä´æÆ÷Ãû*/
#define LTEU_UL_DST_ADDR                                  LTEU_BASE + 0xB0084
#define LTEU_UL_DST_SIZE                                  LTEU_BASE + 0xB0088
#define LTEU_UL_FIFO_STATUS                               LTEU_BASE + 0xB0080
/*¼Ä´æÆ÷Ãû*/
#define LTEU_UBP_PARA0                                    LTEU_BASE + 0x1000
#define LTEU_UBP_PARA1                                    LTEU_BASE + 0x1004
#define LTEU_UBP_PARA2                                    LTEU_BASE + 0x1008
#define LTEU_UBP_PARA3                                    LTEU_BASE + 0x100C
#define LTEU_UBP_PARA4                                    LTEU_BASE + 0x1010
#define LTEU_UBP_PARA5                                    LTEU_BASE + 0x1014
#define LTEU_UBP_PARA6                                    LTEU_BASE + 0x1018
#define LTEU_UBP_PARA7                                    LTEU_BASE + 0x101C
#define LTEU_UBP_PARA8                                    LTEU_BASE + 0x1020
#define LTEU_UBP_PARA9                                    LTEU_BASE + 0x1024
#define LTEU_UBP_PARA10                                   LTEU_BASE + 0x1028
#define LTEU_UBP_PARA11                                   LTEU_BASE + 0x102C
#define LTEU_UBP_PARA12                                   LTEU_BASE + 0x1030
#define LTEU_UBP_PARA13                                   LTEU_BASE + 0x1034
/*¼Ä´æÆ÷Ãû*/
#define LTEU_USP_PAR_AB_FLAG                              LTEU_BASE + 0x2000
#define LTEU_USP_MODE_A                                   LTEU_BASE + 0x2004
#define LTEU_PUSCH_PUCCH_SCRAM_INI_A                      LTEU_BASE + 0x2008
#define LTEU_PUSCH_PRB_START_A                            LTEU_BASE + 0x200C
#define LTEU_BAND_WIDTH_A                                 LTEU_BASE + 0x2010
/*¼Ä´æÆ÷Ãû*/
#define LTEU_MODU_MODE_A                                  LTEU_BASE + 0x2014
#define LTEU_DFT_ALPHA_A                                  LTEU_BASE + 0x2018
#define LTEU_DFT_PARA1_A                                  LTEU_BASE + 0x201C
#define LTEU_DFT_PARA2_A                                  LTEU_BASE + 0x2020
#define LTEU_DFT_PARA3_A                                  LTEU_BASE + 0x2024
#define LTEU_DFT_SHIFT_A                                  LTEU_BASE + 0x2028
#define LTEU_PUSCH_DATA_BETA_A                            LTEU_BASE + 0x202C

/*******************************************************************************
LTET registers' address on LC1860
********************************************************************************/
/*¹«¹²¿ØÖÆ¼Ä´æÆ÷*/
#define LTET_VERSION                                      LTET_BASE + 0x0000
#define LTET_RESET                                        LTET_BASE + 0x0004
#define LTET_DL_MODE                                      LTET_BASE + 0x0008
#define LTET_DSP_INT_RAW_1643                             LTET_BASE + 0x000C
#define LTET_DSP_INT_EN_1643                              LTET_BASE + 0x0010
#define LTET_DSP_INT_MASK_1643                            LTET_BASE + 0x0014
#define LTET_DSP_INT_STATUS_1643                          LTET_BASE + 0x0018
#define LTET_DSP_INT_RAW_4210                             LTET_BASE + 0x001C
#define LTET_DSP_INT_EN_4210                              LTET_BASE + 0x0020
#define LTET_DSP_INT_MASK_4210                            LTET_BASE + 0x0024
#define LTET_DSP_INT_STATUS_4210                          LTET_BASE + 0x0028
#define LTET_ACC_ACT                                      LTET_BASE + 0x002C
#define LTET_TFT_START                                    LTET_BASE + 0x0030
/*¼Ä´æÆ÷Ãû*/
#define LTET_CH_STATUS_LTET                               LTET_BASE + 0x2000
#define LTET_CH1_CTRL                                     LTET_BASE + 0x2020
#define LTET_CH1_CFG                                      LTET_BASE + 0x2024
#define LTET_CH1_SAR                                      LTET_BASE + 0x2028
#define LTET_CH1_CUR_SAR                                  LTET_BASE + 0x202C
#define LTET_CH2_CTRL                                     LTET_BASE + 0x2030
#define LTET_CH2_CFG                                      LTET_BASE + 0x2034
#define LTET_CH2_SAR                                      LTET_BASE + 0x2038
#define LTET_CH2_CUR_SAR                                  LTET_BASE + 0x203C
#define LTET_CH_BEGIN                                     LTET_BASE + 0x2040
#define LTET_CH_END                                       LTET_BASE + 0x2044
#define LTET_RXFIFO1_STATUS                               LTET_BASE + 0x2068
#define LTET_RXFIFO2_STATUS                               LTET_BASE + 0x206C
/*¼Ä´æÆ÷Ãû*/
#define LTET_DMA_FIFO_STATUS                              LTET_BASE + 0x2070
/*¼Ä´æÆ÷Ãû*/
#define LTET_TFT_PAR_ABC_FLAG                             LTET_BASE + 0x1000
#define LTET_TFT_RFIF_RAM_FLAG_A                          LTET_BASE + 0x1004
#define LTET_TFT_CTR_A                                    LTET_BASE + 0x1008
#define LTET_TFT_CTR_1_A                                  LTET_BASE + 0x100C
#define LTET_TFT_CTR_2_A                                  LTET_BASE + 0x1010
#define LTET_TFT_CTR_3_A                                  LTET_BASE + 0x1014
#define LTET_TFT_CTR_4_A                                  LTET_BASE + 0x1018
#define LTET_TFT_CTR_5_A                                  LTET_BASE + 0x101C
#define LTET_TFT_CTR_6_A                                  LTET_BASE + 0x1020
#define LTET_RSSI_PARA_A                                  LTET_BASE + 0x1024
#define LTET_FFT_SIZE_A                                   LTET_BASE + 0x1028
#define LTET_TFT_BAND_WIDTH_A                             LTET_BASE + 0x102C
#define LTET_SHIFT_FFT_A                                  LTET_BASE + 0x1030
#define LTET_TF_FOE_A                                     LTET_BASE + 0x1034
#define LTET_PHASE_BASE_A                                 LTET_BASE + 0x1038
#define LTET_TFT_AGC_A_RSSI_E                             LTET_BASE + 0x103C
#define LTET_TFT_AGC_A_CTRL                               LTET_BASE + 0x1040
#define LTET_TFT_RFIF_RAM_FLAG_B                          LTET_BASE + 0x1044
#define LTET_TFT_CTR_B                                    LTET_BASE + 0x1048
#define LTET_TFT_CTR_1_B                                  LTET_BASE + 0x104C
#define LTET_TFT_CTR_2_B                                  LTET_BASE + 0x1050
#define LTET_TFT_CTR_3_B                                  LTET_BASE + 0x1054
#define LTET_TFT_CTR_4_B                                  LTET_BASE + 0x1058
#define LTET_TFT_CTR_5_B                                  LTET_BASE + 0x105C
#define LTET_TFT_CTR_6_B                                  LTET_BASE + 0x1060
#define LTET_RSSI_PARA_B                                  LTET_BASE + 0x1064
#define LTET_FFT_SIZE_B                                   LTET_BASE + 0x1068
#define LTET_TFT_BAND_WIDTH_B                             LTET_BASE + 0x106C
#define LTET_SHIFT_FFT_B                                  LTET_BASE + 0x1070
#define LTET_TF_FOE_B                                     LTET_BASE + 0x1074
#define LTET_PHASE_BASE_B                                 LTET_BASE + 0x1078
#define LTET_TFT_AGC_B_RSSI_E                             LTET_BASE + 0x107C
#define LTET_TFT_AGC_B_CTRL                               LTET_BASE + 0x1080
#define LTET_TFT_RFIF_RAM_FLAG_C                          LTET_BASE + 0x1084
#define LTET_TFT_CTR_C                                    LTET_BASE + 0x1088
#define LTET_TFT_CTR_1_C                                  LTET_BASE + 0x108C
#define LTET_TFT_CTR_2_C                                  LTET_BASE + 0x1090
#define LTET_TFT_CTR_3_C                                  LTET_BASE + 0x1094
#define LTET_TFT_CTR_4_C                                  LTET_BASE + 0x1098
#define LTET_TFT_CTR_5_C                                  LTET_BASE + 0x109C
#define LTET_TFT_CTR_6_C                                  LTET_BASE + 0x10A0
#define LTET_RSSI_PARA_C                                  LTET_BASE + 0x10A4
#define LTET_SHIFT_FFT_C                                  LTET_BASE + 0x10B0
#define LTET_TF_FOE_C                                     LTET_BASE + 0x10B4
#define LTET_PHASE_BASE_C                                 LTET_BASE + 0x10B8
#define LTET_TFT_AGC_C_RSSI_E                             LTET_BASE + 0x10BC
#define LTET_TFT_AGC_C_CTRL                               LTET_BASE + 0x10C0
#define LTET_TFT_FRAME_TRANSFER_IN_A                      LTET_BASE + 0x10C4
#define LTET_TFT_FRAME_TRANSFER_IN_B                      LTET_BASE + 0x11E4
#define LTET_TFT_FRAME_TRANSFER_IN_C                      LTET_BASE + 0x11E8
/*¼Ä´æÆ÷Ãû*/
#define LTET_TFT_END_FLAG                                 LTET_BASE + 0x10C8
#define LTET_RSSI_FRE_ANT0_A                              LTET_BASE + 0x10CC
#define LTET_RSSI_FRE_ANT1_A                              LTET_BASE + 0x10D0
#define LTET_TFT_D_AGC_A                                  LTET_BASE + 0x11D4
#define LTET_RSSI_ANT0_1                                  LTET_BASE + 0x10D8
#define LTET_RSSI_ANT0_2                                  LTET_BASE + 0x10DC
#define LTET_RSSI_ANT0_3                                  LTET_BASE + 0x10E0
#define LTET_RSSI_ANT0_4                                  LTET_BASE + 0x10E4
#define LTET_RSSI_ANT0_5                                  LTET_BASE + 0x10E8
#define LTET_RSSI_ANT0_6                                  LTET_BASE + 0x10EC
#define LTET_RSSI_ANT0_7                                  LTET_BASE + 0x10F0
#define LTET_RSSI_ANT0_8                                  LTET_BASE + 0x10F4
#define LTET_RSSI_ANT0_9                                  LTET_BASE + 0x10F8
#define LTET_RSSI_ANT0_10                                 LTET_BASE + 0x10FC
#define LTET_RSSI_ANT0_11                                 LTET_BASE + 0x1100
#define LTET_RSSI_ANT0_12                                 LTET_BASE + 0x1104
#define LTET_RSSI_ANT0_13                                 LTET_BASE + 0x1108
#define LTET_RSSI_ANT0_14                                 LTET_BASE + 0x110C
#define LTET_RSSI_ANT0_15                                 LTET_BASE + 0x1110
#define LTET_RSSI_ANT0_16                                 LTET_BASE + 0x1114
#define LTET_RSSI_ANT0_17                                 LTET_BASE + 0x1118
#define LTET_RSSI_ANT0_18                                 LTET_BASE + 0x111C
#define LTET_RSSI_ANT0_19                                 LTET_BASE + 0x1120
#define LTET_RSSI_ANT0_20                                 LTET_BASE + 0x1124
#define LTET_RSSI_ANT0_21                                 LTET_BASE + 0x1128
#define LTET_RSSI_ANT0_22                                 LTET_BASE + 0x112C
#define LTET_RSSI_ANT0_23                                 LTET_BASE + 0x1130
#define LTET_RSSI_ANT0_24                                 LTET_BASE + 0x1134
#define LTET_RSSI_ANT0_25                                 LTET_BASE + 0x1138
#define LTET_RSSI_ANT0_26                                 LTET_BASE + 0x113C
#define LTET_RSSI_ANT0_27                                 LTET_BASE + 0x1140
#define LTET_RSSI_ANT0_28                                 LTET_BASE + 0x1144
#define LTET_RSSI_ANT0_29                                 LTET_BASE + 0x1148
#define LTET_RSSI_ANT0_30                                 LTET_BASE + 0x114C
#define LTET_RSSI_ANT1_1                                  LTET_BASE + 0x1150
#define LTET_RSSI_ANT1_2                                  LTET_BASE + 0x1154
#define LTET_RSSI_ANT1_3                                  LTET_BASE + 0x1158
#define LTET_RSSI_ANT1_4                                  LTET_BASE + 0x115C
#define LTET_RSSI_ANT1_5                                  LTET_BASE + 0x1160
#define LTET_RSSI_ANT1_6                                  LTET_BASE + 0x1164
#define LTET_RSSI_ANT1_7                                  LTET_BASE + 0x1168
#define LTET_RSSI_ANT1_8                                  LTET_BASE + 0x116C
#define LTET_RSSI_ANT1_9                                  LTET_BASE + 0x1170
#define LTET_RSSI_ANT1_10                                 LTET_BASE + 0x1174
#define LTET_RSSI_ANT1_11                                 LTET_BASE + 0x1178
#define LTET_RSSI_ANT1_12                                 LTET_BASE + 0x117C
#define LTET_RSSI_ANT1_13                                 LTET_BASE + 0x1180
#define LTET_RSSI_ANT1_14                                 LTET_BASE + 0x1184
#define LTET_RSSI_ANT1_15                                 LTET_BASE + 0x1188
#define LTET_RSSI_ANT1_16                                 LTET_BASE + 0x118C
#define LTET_RSSI_ANT1_17                                 LTET_BASE + 0x1190
#define LTET_RSSI_ANT1_18                                 LTET_BASE + 0x1194
#define LTET_RSSI_ANT1_19                                 LTET_BASE + 0x1198
#define LTET_RSSI_ANT1_20                                 LTET_BASE + 0x119C
#define LTET_RSSI_ANT1_21                                 LTET_BASE + 0x11A0
#define LTET_RSSI_ANT1_22                                 LTET_BASE + 0x11A4
#define LTET_RSSI_ANT1_23                                 LTET_BASE + 0x11A8
#define LTET_RSSI_ANT1_24                                 LTET_BASE + 0x11AC
#define LTET_RSSI_ANT1_25                                 LTET_BASE + 0x11B0
#define LTET_RSSI_ANT1_26                                 LTET_BASE + 0x11B4
#define LTET_RSSI_ANT1_27                                 LTET_BASE + 0x11B8
#define LTET_RSSI_ANT1_28                                 LTET_BASE + 0x11BC
#define LTET_RSSI_ANT1_29                                 LTET_BASE + 0x11C0
#define LTET_RSSI_ANT1_30                                 LTET_BASE + 0x11C4
#define LTET_RSSI_FRE_ANT0_B                              LTET_BASE + 0x11C8
#define LTET_RSSI_FRE_ANT1_B                              LTET_BASE + 0x11CC
#define LTET_TFT_D_AGC_B                                  LTET_BASE + 0x11D0
#define LTET_RSSI_FRE_ANT0_C                              LTET_BASE + 0x11D4
#define LTET_RSSI_FRE_ANT1_C                              LTET_BASE + 0x11D8
#define LTET_TFT_D_AGC_C                                  LTET_BASE + 0x11DC
#define LTET_TFT_FRAME_TRANSFER_OUT_A                     LTET_BASE + 0x11E0
#define LTET_RESERVED                                     LTET_BASE + 0x11E4
#define LTET_RESERVED2                                    LTET_BASE + 0x11E8
#define LTET_TFT_FRAME_TRANSFER_OUT_B                     LTET_BASE + 0x11EC
#define LTET_TFT_FRAME_TRANSFER_OUT_C                     LTET_BASE + 0x11F0

/*******************************************************************************
RFIF registers' address on LC1860
********************************************************************************/
/*¶¨Ê±Æ÷0Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TIMER0_STATUS                                RFIF_BASE + 0x0000
#define RFIF_TIMER0_CUR_CNT                               RFIF_BASE + 0x0004
#define RFIF_TIMER0_CUR_SF                                RFIF_BASE + 0x0008
#define RFIF_TIMER0_CYC                                   RFIF_BASE + 0x000C
#define RFIF_TIMER0_SF_MAX                                RFIF_BASE + 0x0010
#define RFIF_TIMER0_ADJ                                   RFIF_BASE + 0x0014
#define RFIF_TIMER0_FRAME_SET                             RFIF_BASE + 0x0018
#define RFIF_TIMER0_SF_SET                                RFIF_BASE + 0x001C
#define RFIF_TIMER0_EN                                    RFIF_BASE + 0x0020
#define RFIF_TIMER0_CTRL                                  RFIF_BASE + 0x0024
#define RFIF_TIMER0_SELF_CTRL_OP                          RFIF_BASE + 0x0028
#define RFIF_TIMER0_SELF_CTRL                             RFIF_BASE + 0x002C
#define RFIF_TIMER0_OUT_CTRL_OP                           RFIF_BASE + 0x0030
#define RFIF_TIMER0_OUT_CTRL                              RFIF_BASE + 0x0034
#define RFIF_TIMER0_CALIB_AUTO                            RFIF_BASE + 0x0038
#define RFIF_TIMER0_CALIB_EN                              RFIF_BASE + 0x003C
#define RFIF_TIMER0_CALIB_CTRL                            RFIF_BASE + 0x0040
#define RFIF_TIMER0_CALIB_DATA                            RFIF_BASE + 0x0044
#define RFIF_TIMER0_STM_INT                               RFIF_BASE + 0x0048
#define RFIF_TIMER0_STM_INT_SF                            RFIF_BASE + 0x004C
#define RFIF_TIMER0_STM_SIG                               RFIF_BASE + 0x0050
#define RFIF_TIMER0_STM_SIG_SF                            RFIF_BASE + 0x0054
#define RFIF_TIMER0_STM_SW                                RFIF_BASE + 0x0058
#define RFIF_TIMER0_STM_SW_SF                             RFIF_BASE + 0x005C
#define RFIF_TIMER0_STM_CUR                               RFIF_BASE + 0x0060
#define RFIF_TIMER0_STM_CUR_SF                            RFIF_BASE + 0x0064
#define RFIF_TIMER0_STM_INC                               RFIF_BASE + 0x0068
/*¶¨Ê±Æ÷1Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TIMER1_STATUS                                RFIF_BASE + 0x0080
#define RFIF_TIMER1_CUR_CNT                               RFIF_BASE + 0x0084
#define RFIF_TIMER1_CUR_SF                                RFIF_BASE + 0x0088
#define RFIF_TIMER1_CYC                                   RFIF_BASE + 0x008C
#define RFIF_TIMER1_SF_MAX                                RFIF_BASE + 0x0090
#define RFIF_TIMER1_ADJ                                   RFIF_BASE + 0x0094
#define RFIF_TIMER1_FRAME_SET                             RFIF_BASE + 0x0098
#define RFIF_TIMER1_SF_SET                                RFIF_BASE + 0x009C
#define RFIF_TIMER1_EN                                    RFIF_BASE + 0x00A0
#define RFIF_TIMER1_CTRL                                  RFIF_BASE + 0x00A4
#define RFIF_TIMER1_SELF_CTRL_OP                          RFIF_BASE + 0x00A8
#define RFIF_TIMER1_SELF_CTRL                             RFIF_BASE + 0x00AC
#define RFIF_TIMER1_OUT_CTRL_OP                           RFIF_BASE + 0x00B0
#define RFIF_TIMER1_OUT_CTRL                              RFIF_BASE + 0x00B4
#define RFIF_TIMER1_CALIB_AUTO                            RFIF_BASE + 0x00B8
#define RFIF_TIMER1_CALIB_EN                              RFIF_BASE + 0x00BC
#define RFIF_TIMER1_CALIB_CTRL                            RFIF_BASE + 0x00C0
#define RFIF_TIMER1_CALIB_DATA                            RFIF_BASE + 0x00C4
#define RFIF_TIMER1_STM_INT                               RFIF_BASE + 0x00C8
#define RFIF_TIMER1_STM_INT_SF                            RFIF_BASE + 0x00CC
#define RFIF_TIMER1_STM_SIG                               RFIF_BASE + 0x00D0
#define RFIF_TIMER1_STM_SIG_SF                            RFIF_BASE + 0x00D4
#define RFIF_TIMER1_STM_SW                                RFIF_BASE + 0x00D8
#define RFIF_TIMER1_STM_SW_SF                             RFIF_BASE + 0x00DC
#define RFIF_TIMER1_STM_CUR                               RFIF_BASE + 0x00E0
#define RFIF_TIMER1_STM_CUR_SF                            RFIF_BASE + 0x00E4
#define RFIF_TIMER1_STM_INC                               RFIF_BASE + 0x00E8
/*¶¨Ê±Æ÷2Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TIMER2_STATUS                                RFIF_BASE + 0x0100
#define RFIF_TIMER2_CUR_CNT                               RFIF_BASE + 0x0104
#define RFIF_TIMER2_CUR_SF                                RFIF_BASE + 0x0108
#define RFIF_TIMER2_CYC                                   RFIF_BASE + 0x010C
#define RFIF_TIMER2_SF_MAX                                RFIF_BASE + 0x0110
#define RFIF_TIMER2_ADJ                                   RFIF_BASE + 0x0114
#define RFIF_TIMER2_FRAME_SET                             RFIF_BASE + 0x0118
#define RFIF_TIMER2_SF_SET                                RFIF_BASE + 0x011C
#define RFIF_TIMER2_EN                                    RFIF_BASE + 0x0120
#define RFIF_TIMER2_CTRL                                  RFIF_BASE + 0x0124
#define RFIF_TIMER2_SELF_CTRL_OP                          RFIF_BASE + 0x0128
#define RFIF_TIMER2_SELF_CTRL                             RFIF_BASE + 0x012C
#define RFIF_TIMER2_OUT_CTRL_OP                           RFIF_BASE + 0x0130
#define RFIF_TIMER2_OUT_CTRL                              RFIF_BASE + 0x0134
#define RFIF_TIMER2_CALIB_AUTO                            RFIF_BASE + 0x0138
#define RFIF_TIMER2_CALIB_EN                              RFIF_BASE + 0x013C
#define RFIF_TIMER2_CALIB_CTRL                            RFIF_BASE + 0x0140
#define RFIF_TIMER2_CALIB_DATA                            RFIF_BASE + 0x0144
#define RFIF_TIMER2_STM_INT                               RFIF_BASE + 0x0148
#define RFIF_TIMER2_STM_INT_SF                            RFIF_BASE + 0x014C
#define RFIF_TIMER2_STM_SIG                               RFIF_BASE + 0x0150
#define RFIF_TIMER2_STM_SIG_SF                            RFIF_BASE + 0x0154
#define RFIF_TIMER2_STM_SW                                RFIF_BASE + 0x0158
#define RFIF_TIMER2_STM_SW_SF                             RFIF_BASE + 0x015C
#define RFIF_TIMER2_STM_CUR                               RFIF_BASE + 0x0160
#define RFIF_TIMER2_STM_CUR_SF                            RFIF_BASE + 0x0164
#define RFIF_TIMER2_STM_INC                               RFIF_BASE + 0x0168
/*¶¨Ê±Æ÷3Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TIMER3_STATUS                                RFIF_BASE + 0x0180
#define RFIF_TIMER3_CUR_CNT                               RFIF_BASE + 0x0184
#define RFIF_TIMER3_CUR_SF                                RFIF_BASE + 0x0188
#define RFIF_TIMER3_CYC                                   RFIF_BASE + 0x018C
#define RFIF_TIMER3_SF_MAX                                RFIF_BASE + 0x0190
#define RFIF_TIMER3_ADJ                                   RFIF_BASE + 0x0194
#define RFIF_TIMER3_FRAME_SET                             RFIF_BASE + 0x0198
#define RFIF_TIMER3_SF_SET                                RFIF_BASE + 0x019C
#define RFIF_TIMER3_EN                                    RFIF_BASE + 0x01A0
#define RFIF_TIMER3_CTRL                                  RFIF_BASE + 0x01A4
#define RFIF_TIMER3_SELF_CTRL_OP                          RFIF_BASE + 0x01A8
#define RFIF_TIMER3_SELF_CTRL                             RFIF_BASE + 0x01AC
#define RFIF_TIMER3_OUT_CTRL_OP                           RFIF_BASE + 0x01B0
#define RFIF_TIMER3_OUT_CTRL                              RFIF_BASE + 0x01B4
#define RFIF_TIMER3_CALIB_AUTO                            RFIF_BASE + 0x01B8
#define RFIF_TIMER3_CALIB_EN                              RFIF_BASE + 0x01BC
#define RFIF_TIMER3_CALIB_CTRL                            RFIF_BASE + 0x01C0
#define RFIF_TIMER3_CALIB_DATA                            RFIF_BASE + 0x01C4
#define RFIF_TIMER3_STM_INT                               RFIF_BASE + 0x01C8
#define RFIF_TIMER3_STM_INT_SF                            RFIF_BASE + 0x01CC
#define RFIF_TIMER3_STM_SIG                               RFIF_BASE + 0x01D0
#define RFIF_TIMER3_STM_SIG_SF                            RFIF_BASE + 0x01D4
#define RFIF_TIMER3_STM_SW                                RFIF_BASE + 0x01D8
#define RFIF_TIMER3_STM_SW_SF                             RFIF_BASE + 0x01DC
#define RFIF_TIMER3_STM_CUR                               RFIF_BASE + 0x01E0
#define RFIF_TIMER3_STM_CUR_SF                            RFIF_BASE + 0x01E4
#define RFIF_TIMER3_STM_INC                               RFIF_BASE + 0x01E8
/*¶¨Ê±Æ÷4Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TIMER4_STATUS                                RFIF_BASE + 0x0200
#define RFIF_TIMER4_CUR_CNT                               RFIF_BASE + 0x0204
#define RFIF_TIMER4_CUR_SF                                RFIF_BASE + 0x0208
#define RFIF_TIMER4_CYC                                   RFIF_BASE + 0x020C
#define RFIF_TIMER4_SF_MAX                                RFIF_BASE + 0x0210
#define RFIF_TIMER4_ADJ                                   RFIF_BASE + 0x0214
#define RFIF_TIMER4_FRAME_SET                             RFIF_BASE + 0x0218
#define RFIF_TIMER4_SF_SET                                RFIF_BASE + 0x021C
#define RFIF_TIMER4_EN                                    RFIF_BASE + 0x0220
#define RFIF_TIMER4_CTRL                                  RFIF_BASE + 0x0224
#define RFIF_TIMER4_SELF_CTRL_OP                          RFIF_BASE + 0x0228
#define RFIF_TIMER4_SELF_CTRL                             RFIF_BASE + 0x022C
#define RFIF_TIMER4_OUT_CTRL_OP                           RFIF_BASE + 0x0230
#define RFIF_TIMER4_OUT_CTRL                              RFIF_BASE + 0x0234
#define RFIF_TIMER4_CALIB_AUTO                            RFIF_BASE + 0x0238
#define RFIF_TIMER4_CALIB_EN                              RFIF_BASE + 0x023C
#define RFIF_TIMER4_CALIB_CTRL                            RFIF_BASE + 0x0240
#define RFIF_TIMER4_CALIB_DATA                            RFIF_BASE + 0x0244
#define RFIF_TIMER4_STM_INT                               RFIF_BASE + 0x0248
#define RFIF_TIMER4_STM_INT_SF                            RFIF_BASE + 0x024C
#define RFIF_TIMER4_STM_SIG                               RFIF_BASE + 0x0250
#define RFIF_TIMER4_STM_SIG_SF                            RFIF_BASE + 0x0254
#define RFIF_TIMER4_STM_SW                                RFIF_BASE + 0x0258
#define RFIF_TIMER4_STM_SW_SF                             RFIF_BASE + 0x025C
#define RFIF_TIMER4_STM_CUR                               RFIF_BASE + 0x0260
#define RFIF_TIMER4_STM_CUR_SF                            RFIF_BASE + 0x0264
#define RFIF_TIMER4_STM_INC                               RFIF_BASE + 0x0268
/*¶¨Ê±Æ÷5Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TIMER5_STATUS                                RFIF_BASE + 0x0280
#define RFIF_TIMER5_CUR_CNT                               RFIF_BASE + 0x0284
#define RFIF_TIMER5_CUR_SF                                RFIF_BASE + 0x0288
#define RFIF_TIMER5_CYC                                   RFIF_BASE + 0x028C
#define RFIF_TIMER5_SF_MAX                                RFIF_BASE + 0x0290
#define RFIF_TIMER5_ADJ                                   RFIF_BASE + 0x0294
#define RFIF_TIMER5_FRAME_SET                             RFIF_BASE + 0x0298
#define RFIF_TIMER5_SF_SET                                RFIF_BASE + 0x029C
#define RFIF_TIMER5_EN                                    RFIF_BASE + 0x02A0
#define RFIF_TIMER5_CTRL                                  RFIF_BASE + 0x02A4
#define RFIF_TIMER5_SELF_CTRL_OP                          RFIF_BASE + 0x02A8
#define RFIF_TIMER5_SELF_CTRL                             RFIF_BASE + 0x02AC
#define RFIF_TIMER5_OUT_CTRL_OP                           RFIF_BASE + 0x02B0
#define RFIF_TIMER5_OUT_CTRL                              RFIF_BASE + 0x02B4
#define RFIF_TIMER5_CALIB_AUTO                            RFIF_BASE + 0x02B8
#define RFIF_TIMER5_CALIB_EN                              RFIF_BASE + 0x02BC
#define RFIF_TIMER5_CALIB_CTRL                            RFIF_BASE + 0x02C0
#define RFIF_TIMER5_CALIB_DATA                            RFIF_BASE + 0x02C4
#define RFIF_TIMER5_STM_INT                               RFIF_BASE + 0x02C8
#define RFIF_TIMER5_STM_INT_SF                            RFIF_BASE + 0x02CC
#define RFIF_TIMER5_STM_SIG                               RFIF_BASE + 0x02D0
#define RFIF_TIMER5_STM_SIG_SF                            RFIF_BASE + 0x02D4
#define RFIF_TIMER5_STM_SW                                RFIF_BASE + 0x02D8
#define RFIF_TIMER5_STM_SW_SF                             RFIF_BASE + 0x02DC
#define RFIF_TIMER5_STM_CUR                               RFIF_BASE + 0x02E0
#define RFIF_TIMER5_STM_CUR_SF                            RFIF_BASE + 0x02E4
#define RFIF_TIMER5_STM_INC                               RFIF_BASE + 0x02E8
/*¶¨Ê±Æ÷¹«ÓÃ¼Ä´æÆ÷*/
#define RFIF_TIMER_RD_SYNC                                RFIF_BASE + 0x0300
#define RFIF_TIMER_SYN_STATUS                             RFIF_BASE + 0x0304
#define RFIF_TIMER0_SYN_CUR                               RFIF_BASE + 0x0308
#define RFIF_TIMER1_SYN_CUR                               RFIF_BASE + 0x030C
#define RFIF_TIMER2_SYN_CUR                               RFIF_BASE + 0x0310
#define RFIF_TIMER3_SYN_CUR                               RFIF_BASE + 0x0314
#define RFIF_TIMER4_SYN_CUR                               RFIF_BASE + 0x0318
#define RFIF_TIMER5_SYN_CUR                               RFIF_BASE + 0x031C
/*¶¨Ê±Æ÷ÖÐ¶Ï*/
#define RFIF_TIMER_IXT_EX                                 RFIF_BASE + 0x0320
#define RFIF_TIMER_IXT_STATUS                             RFIF_BASE + 0x0324
#define RFIF_TIMER_INT_RAW                                RFIF_BASE + 0x0328
/*RFIF¿ØÖÆ¼Ä´æÆ÷*/
#define RFIF_CTRL                                         RFIF_BASE + 0x0330
/*RFIFÖÜÆÚºÍ·ÇÖÜÆÚÖÐ¶ÏÉèÖÃ¼Ä´æÆ÷*/
#define RFIF_CYC_CTRL                                     RFIF_BASE + 0x0340
#define RFIF_CYC0_VALID                                   RFIF_BASE + 0x0350
#define RFIF_CYC0                                         RFIF_BASE + 0x0354
#define RFIF_CYC1_VALID                                   RFIF_BASE + 0x0358
#define RFIF_CYC1                                         RFIF_BASE + 0x035C
#define RFIF_CYC2_VALID                                   RFIF_BASE + 0x0360
#define RFIF_CYC2                                         RFIF_BASE + 0x0364
#define RFIF_CYC3_VALID                                   RFIF_BASE + 0x0368
#define RFIF_CYC3                                         RFIF_BASE + 0x036C
#define RFIF_CYC4_VALID                                   RFIF_BASE + 0x0370
#define RFIF_CYC4                                         RFIF_BASE + 0x0374
#define RFIF_CYC5_VALID                                   RFIF_BASE + 0x0378
#define RFIF_CYC5                                         RFIF_BASE + 0x037C
#define RFIF_CYC6_VALID                                   RFIF_BASE + 0x0380
#define RFIF_CYC6                                         RFIF_BASE + 0x0384
#define RFIF_CYC7_VALID                                   RFIF_BASE + 0x0388
#define RFIF_CYC7                                         RFIF_BASE + 0x038C
#define RFIF_NCYC_CTRL                                    RFIF_BASE + 0x0390
#define RFIF_NCYC0                                        RFIF_BASE + 0x03A0
#define RFIF_NCYC1                                        RFIF_BASE + 0x03A4
#define RFIF_NCYC2                                        RFIF_BASE + 0x03A8
#define RFIF_NCYC3                                        RFIF_BASE + 0x03AC
#define RFIF_NCYC4                                        RFIF_BASE + 0x03B0
#define RFIF_NCYC5                                        RFIF_BASE + 0x03B4
#define RFIF_NCYC6                                        RFIF_BASE + 0x03B8
#define RFIF_NCYC7                                        RFIF_BASE + 0x03BC
/*RFIFÖÐ¶Ï¼Ä´æÆ÷*/
#define RFIF_INT_MASK_ARM                                 RFIF_BASE + 0x0400
#define RFIF_INT_STATUS_FINAL_ARM                         RFIF_BASE + 0x0404
#define RFIF_INT_EN_ARM                                   RFIF_BASE + 0x0408
#define RFIF_INT_STATUS_ARM                               RFIF_BASE + 0x040C
#define RFIF_INT_MASK_XC4210                              RFIF_BASE + 0x0410
#define RFIF_INT_STATUS_FINAL_XC4210                      RFIF_BASE + 0x0414
#define RFIF_INT_EN_XC4210                                RFIF_BASE + 0x0418
#define RFIF_INT_STATUS_XC4210                            RFIF_BASE + 0x041C
#define RFIF_INT_MASK_X1643                               RFIF_BASE + 0x0420
#define RFIF_INT_STATUS_FINAL_X1643                       RFIF_BASE + 0x0424
#define RFIF_INT_EN_X1643                                 RFIF_BASE + 0x0428
#define RFIF_INT_STATUS_X1643                             RFIF_BASE + 0x042C
#define RFIF_INT_RAW                                      RFIF_BASE + 0x0430
/*Ê±Ðò´¦ÀíÆ÷¹«ÓÃ¼Ä´æÆ÷*/
#define RFIF_TP_EN                                        RFIF_BASE + 0x0480
#define RFIF_TP_STATUS                                    RFIF_BASE + 0x0484
#define RFIF_TP_CTRL                                      RFIF_BASE + 0x0488
/*Ê±Ðò´¦ÀíÆ÷0Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TP0_STATUS                                   RFIF_BASE + 0x04A0
#define RFIF_TP0_STR                                      RFIF_BASE + 0x04A4
#define RFIF_TP0_STR_CTRL                                 RFIF_BASE + 0x04A8
#define RFIF_TP0_DELTA_CNT_MAX                            RFIF_BASE + 0x04AC
#define RFIF_TP0_DELTA_CNT_CUR                            RFIF_BASE + 0x04B0
#define RFIF_TP0_JUMP_CTRL                                RFIF_BASE + 0x04B4
/*Ê±Ðò´¦ÀíÆ÷1Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TP1_STATUS                                   RFIF_BASE + 0x04C0
#define RFIF_TP1_STR                                      RFIF_BASE + 0x04C4
#define RFIF_TP1_STR_CTRL                                 RFIF_BASE + 0x04C8
#define RFIF_TP1_DELTA_CNT_MAX                            RFIF_BASE + 0x04CC
#define RFIF_TP1_DELTA_CNT_CUR                            RFIF_BASE + 0x04D0
#define RFIF_TP1_JUMP_CTRL                                RFIF_BASE + 0x04D4
/*Ê±Ðò´¦ÀíÆ÷2Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TP2_STATUS                                   RFIF_BASE + 0x04E0
#define RFIF_TP2_STR                                      RFIF_BASE + 0x04E4
#define RFIF_TP2_STR_CTRL                                 RFIF_BASE + 0x04E8
#define RFIF_TP2_DELTA_CNT_MAX                            RFIF_BASE + 0x04EC
#define RFIF_TP2_DELTA_CNT_CUR                            RFIF_BASE + 0x04F0
#define RFIF_TP2_JUMP_CTRL                                RFIF_BASE + 0x04F4
/*Ê±Ðò´¦ÀíÆ÷3Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TP3_STATUS                                   RFIF_BASE + 0x0500
#define RFIF_TP3_STR                                      RFIF_BASE + 0x0504
#define RFIF_TP3_STR_CTRL                                 RFIF_BASE + 0x0508
#define RFIF_TP3_DELTA_CNT_MAX                            RFIF_BASE + 0x050C
#define RFIF_TP3_DELTA_CNT_CUR                            RFIF_BASE + 0x0510
#define RFIF_TP3_JUMP_CTRL                                RFIF_BASE + 0x0514
/*Ê±Ðò´¦ÀíÆ÷4Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TP4_STATUS                                   RFIF_BASE + 0x0520
#define RFIF_TP4_STR                                      RFIF_BASE + 0x0524
#define RFIF_TP4_STR_CTRL                                 RFIF_BASE + 0x0528
#define RFIF_TP4_DELTA_CNT_MAX                            RFIF_BASE + 0x052C
#define RFIF_TP4_DELTA_CNT_CUR                            RFIF_BASE + 0x0530
#define RFIF_TP4_JUMP_CTRL                                RFIF_BASE + 0x0534
/*Ê±Ðò´¦ÀíÆ÷5Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_TP5_STATUS                                   RFIF_BASE + 0x0540
#define RFIF_TP5_STR                                      RFIF_BASE + 0x0544
#define RFIF_TP5_STR_CTRL                                 RFIF_BASE + 0x0548
#define RFIF_TP5_DELTA_CNT_MAX                            RFIF_BASE + 0x054C
#define RFIF_TP5_DELTA_CNT_CUR                            RFIF_BASE + 0x0550
#define RFIF_TP5_JUMP_CTRL                                RFIF_BASE + 0x0554
/*Ê±Ðò´¦ÀíÆ÷6*/
#define RFIF_TP6_STATUS                                   RFIF_BASE + 0x0560
#define RFIF_TP6_STR                                      RFIF_BASE + 0x0564
#define RFIF_TP6_STR_CTRL                                 RFIF_BASE + 0x0568
#define RFIF_TP6_DELTA_CNT_MAX                            RFIF_BASE + 0x056C
#define RFIF_TP6_DELTA_CNT_CUR                            RFIF_BASE + 0x0570
#define RFIF_TP6_JUMP_CTRL                                RFIF_BASE + 0x0574
/*Ê±Ðò´¦ÀíÆ÷Í¨ÓÃÊä³öºÍ´¥·¢¼Ä´æÆ÷*/
#define RFIF_TP_GPO0                                      RFIF_BASE + 0x05A0
#define RFIF_TP_GPO1                                      RFIF_BASE + 0x05A4
#define RFIF_TP_GPO2                                      RFIF_BASE + 0x05A8
#define RFIF_TP_GPO3                                      RFIF_BASE + 0x05AC
#define RFIF_TP_GPO4                                      RFIF_BASE + 0x05B0
#define RFIF_TP_GPO_MASK0                                 RFIF_BASE + 0x05C0
#define RFIF_TP_GPO_MASK1                                 RFIF_BASE + 0x05C4
#define RFIF_TP_GPO_MASK2                                 RFIF_BASE + 0x05C8
#define RFIF_TP_GPO_MASK3                                 RFIF_BASE + 0x05CC
#define RFIF_TP_GPO_MASK4                                 RFIF_BASE + 0x05D0
#define RFIF_TP_TRG0                                      RFIF_BASE + 0x05E0
#define RFIF_TP_TRG1                                      RFIF_BASE + 0x05E4
#define RFIF_TP_INT_EN_ARM                                RFIF_BASE + 0x05F0
#define RFIF_TP_INT_STATUS_ARM                            RFIF_BASE + 0x05F4
#define RFIF_TP_INT_EN_XC4210                             RFIF_BASE + 0x05F8
#define RFIF_TP_INT_STATUS_XC4210                         RFIF_BASE + 0x05FC
#define RFIF_TP_INT_EN_X1643                              RFIF_BASE + 0x0600
#define RFIF_TP_INT_STATUS_X1643                          RFIF_BASE + 0x0604
#define RFIF_TP_INT_RAW                                   RFIF_BASE + 0x0608
/*Êý¾ÝÍ¨Â·¼Ä´æÆ÷*/
#define RFIF_RX_DP_CTRL                                   RFIF_BASE + 0x0610
#define RFIF_TX_DP_CTRL                                   RFIF_BASE + 0x0614
#define RFIF_TX_FIR_PARA0                                 RFIF_BASE + 0x0630
#define RFIF_TX_FIR_PARA1                                 RFIF_BASE + 0x0634
#define RFIF_TX_FIR_PARA2                                 RFIF_BASE + 0x0638
#define RFIF_TX_FIR_PARA3                                 RFIF_BASE + 0x063C
#define RFIF_TX_FIR_PARA4                                 RFIF_BASE + 0x0640
#define RFIF_TX_FIR_PARA5                                 RFIF_BASE + 0x0644
#define RFIF_TX_FIR_PARA6                                 RFIF_BASE + 0x0648
#define RFIF_TX_FIR_PARA7                                 RFIF_BASE + 0x064C
#define RFIF_TX_FIR_PARA8                                 RFIF_BASE + 0x0650
#define RFIF_TX_FIR_PARA9                                 RFIF_BASE + 0x0654
#define RFIF_TX_FIR_PARA10                                RFIF_BASE + 0x0658
#define RFIF_TX_FIR_PARA11                                RFIF_BASE + 0x065C
#define RFIF_TX_FIR_PARA12                                RFIF_BASE + 0x0660
#define RFIF_RX_TX_TIMER_EN                               RFIF_BASE + 0x0670
#define RFIF_RX_TIMER_VALID                               RFIF_BASE + 0x0674
#define RFIF_TX_TIMER_VALID                               RFIF_BASE + 0x0678
#define RFIF_RX_TFT_TIMER0_ST                             RFIF_BASE + 0x0680
#define RFIF_RX_TFT_TIMER0_END                            RFIF_BASE + 0x0684
#define RFIF_RX_TFT_TIMER1_ST                             RFIF_BASE + 0x0688
#define RFIF_RX_TFT_TIMER1_END                            RFIF_BASE + 0x068C
#define RFIF_RX_HSL_TIMER0_ST                             RFIF_BASE + 0x0690
#define RFIF_RX_HSL_TIMER0_END                            RFIF_BASE + 0x0694
#define RFIF_RX_HSL_TIMER1_ST                             RFIF_BASE + 0x0698
#define RFIF_RX_HSL_TIMER1_END                            RFIF_BASE + 0x069C
#define RFIF_RX_DMADRX01_TIMER0_ST                        RFIF_BASE + 0x06A0
#define RFIF_RX_DMADRX01_TIMER0_END                       RFIF_BASE + 0x06A4
#define RFIF_RX_DMADRX01_TIMER1_ST                        RFIF_BASE + 0x06A8
#define RFIF_RX_DMADRX01_TIMER1_END                       RFIF_BASE + 0x06AC
#define RFIF_RX_DMADRX23_TIMER0_ST                        RFIF_BASE + 0x06B0
#define RFIF_RX_DMADRX23_TIMER0_END                       RFIF_BASE + 0x06B4
#define RFIF_RX_DMADRX23_TIMER1_ST                        RFIF_BASE + 0x06B8
#define RFIF_RX_DMADRX23_TIMER1_END                       RFIF_BASE + 0x06BC
#define RFIF_TX_HSL_TIMER0_ST                             RFIF_BASE + 0x06C0
#define RFIF_TX_HSL_TIMER0_END                            RFIF_BASE + 0x06C4
#define RFIF_TX_HSL_TIMER1_ST                             RFIF_BASE + 0x06C8
#define RFIF_TX_HSL_TIMER1_END                            RFIF_BASE + 0x06CC
#define RFIF_RX0_TIMER_CNT                                RFIF_BASE + 0x06D0
#define RFIF_RX1_TIMER_CNT                                RFIF_BASE + 0x06D4
#define RFIF_TX_TIMER_CNT                                 RFIF_BASE + 0x06D8
#define RFIF_HSL_CUT_CTRL                                 RFIF_BASE + 0x06DC
#define RFIF_DIGRFV4_MPHY_CTRL                            RFIF_BASE + 0x06E0
#define RFIF_DIGRFV4_MPHY_STATUS                          RFIF_BASE + 0x06E4
#define RFIF_DIGRFV4_INT_STATUS                           RFIF_BASE + 0x0720
#define RFIF_DP_FIFO_INT_EN                               RFIF_BASE + 0x0724
#define RFIF_DP_FIFO_INT_STATUS                           RFIF_BASE + 0x0728
#define RFIF_DP_FIFO_INT_RAW                              RFIF_BASE + 0x072C
#define RFIF_DP_TIMER_INT_EN                              RFIF_BASE + 0x0730
#define RFIF_DP_TIMER_INT_STATUS                          RFIF_BASE + 0x0734
#define RFIF_DP_TIMER_INT_RAW                             RFIF_BASE + 0x0738
/*SPIÏà¹Ø¼Ä´æÆ÷*/
#define RFIF_SPI_EN                                       RFIF_BASE + 0x0760
#define RFIF_SPI_CTRL                                     RFIF_BASE + 0x0764
#define RFIF_SPI_RX_CTRL                                  RFIF_BASE + 0x0768
#define RFIF_SPI_RST                                      RFIF_BASE + 0x076C
#define RFIF_SPI_WAIT                                     RFIF_BASE + 0x0770
#define RFIF_SPI0_PARA                                    RFIF_BASE + 0x0774
#define RFIF_SPI1_PARA                                    RFIF_BASE + 0x0778
#define RFIF_SPI2_PARA                                    RFIF_BASE + 0x077C
#define RFIF_RFFE_EN                                      RFIF_BASE + 0x0784
#define RFIF_RFFE_CTRL                                    RFIF_BASE + 0x0788
#define RFIF_RFFE_RX_CTRL                                 RFIF_BASE + 0x078C
#define RFIF_RFFE_RST                                     RFIF_BASE + 0x0790
/*µ÷ÊÔ¼Ä´æÆ÷*/
#define RFIF_MONITOR_SIG_SEL0                             RFIF_BASE + 0x07B0
#define RFIF_MONITOR_SIG_SEL1                             RFIF_BASE + 0x07B4
#define RFIF_MONITOR_SIG_SEL2                             RFIF_BASE + 0x07B8
#define RFIF_MONITOR_SIG_SEL3                             RFIF_BASE + 0x07BC
#define RFIF_RECORD_SIG_SEL                               RFIF_BASE + 0x07D0
#define RFIF_RECORD_SIG_CTRL                              RFIF_BASE + 0x07D4
#define RFIF_COXFLICT_IXT_EX                              RFIF_BASE + 0x07F0
#define RFIF_COXFLICT_IXT_STATUS                          RFIF_BASE + 0x07F4
#define RFIF_COXFLICT_IXT_RAW                             RFIF_BASE + 0x07F8
/*µÍ¹¦ºÄ¿ØÖÆ*/
#define RFIF_LP_CTRL                                      RFIF_BASE + 0x07FC
/*SPIÊý¾Ý½Ó¿Ú*/
#define RFIF_SPI0_RAM                                     RFIF_BASE + 0x0800
#define RFIF_SPI1_RAM                                     RFIF_BASE + 0x0C00
#define RFIF_SPI2_RAM                                     RFIF_BASE + 0x1000
#define RFIF_RFFE_RAM                                     RFIF_BASE + 0x1400
#define RFIF_SPI_RFFE_STATUS_RAM                          RFIF_BASE + 0x1C00
/*Ê±Ðò´¦ÀíÆ÷Ö¸Áî´æ´¢Æ÷*/
#define RFIF_TP0_INSTR_RAM                                RFIF_BASE + 0x2000
#define RFIF_TP1_INSTR_RAM                                RFIF_BASE + 0x2200
#define RFIF_TP2_INSTR_RAM                                RFIF_BASE + 0x2400
#define RFIF_TP3_INSTR_RAM                                RFIF_BASE + 0x2600
#define RFIF_TP4_INSTR_RAM                                RFIF_BASE + 0x2800
#define RFIF_TP5_INSTR_RAM                                RFIF_BASE + 0x2A00
#define RFIF_TP6_INSTR_RAM                                RFIF_BASE + 0x2C00
/*Ê±Ðò´¦ÀíÆ÷Êý¾Ý´æ´¢Æ÷*/
#define RFIF_TP_DATA_RAM                                  RFIF_BASE + 0x3000
/*GSM1Ïà¹Ø¼Ä´æÆ÷*/
/*GSM1Ê±Ðò´¦ÀíÆ÷¹«ÓÃ¼Ä´æÆ÷*/
#define RFIF_GSM1_TP_EN                                   RFIF_BASE + 0x4000
#define RFIF_GSM1_TP_STATUS                               RFIF_BASE + 0x4004
/*GSM1×¨ÓÃÊ±Ðò´¦ÀíÆ÷0Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_GSM1_TP0_STATUS                              RFIF_BASE + 0x4020
#define RFIF_GSM1_TP0_STR                                 RFIF_BASE + 0x4024
#define RFIF_GSM1_TP0_STR_CTRL                            RFIF_BASE + 0x4028
#define RFIF_GSM1_TP0_DELTA_CNT_MAX                       RFIF_BASE + 0x402C
#define RFIF_GSM1_TP0_DELTA_CNT_CUR                       RFIF_BASE + 0x4030
#define RFIF_GSM1_TP0_JUMP_CTRL                           RFIF_BASE + 0x4034
/*GSM1×¨ÓÃÊ±Ðò´¦ÀíÆ÷1Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_GSM1_TP1_STATUS                              RFIF_BASE + 0x4040
#define RFIF_GSM1_TP1_STR                                 RFIF_BASE + 0x4044
#define RFIF_GSM1_TP1_STR_CTRL                            RFIF_BASE + 0x4048
#define RFIF_GSM1_TP1_DELTA_CNT_MAX                       RFIF_BASE + 0x404C
#define RFIF_GSM1_TP1_DELTA_CNT_CUR                       RFIF_BASE + 0x4050
#define RFIF_GSM1_TP1_JUMP_CTRL                           RFIF_BASE + 0x4054
/*GSM1×¨ÓÃÊ±Ðò´¦ÀíÆ÷2Ïà¹Ø¼Ä´æÆ÷*/
#define RFIF_GSM1_TP2_STATUS                              RFIF_BASE + 0x4060
#define RFIF_GSM1_TP2_STR                                 RFIF_BASE + 0x4064
#define RFIF_GSM1_TP2_STR_CTRL                            RFIF_BASE + 0x4068
#define RFIF_GSM1_TP2_DELTA_CNT_MAX                       RFIF_BASE + 0x406C
#define RFIF_GSM1_TP2_DELTA_CNT_CUR                       RFIF_BASE + 0x4070
#define RFIF_GSM1_TP2_JUMP_CTRL                           RFIF_BASE + 0x4074
/*GSM1Í¨ÓÃ´¥·¢¼Ä´æÆ÷*/
#define RFIF_GSM1_TP_TRG                                  RFIF_BASE + 0x4100
/*GSM1 Ê±Ðò´¦ÀíÆ÷ÖÐ¶Ï*/
#define RFIF_GSM1_TP_INT_EN_ARM                           RFIF_BASE + 0x4110
#define RFIF_GSM1_TP_INT_STATUS_ARM                       RFIF_BASE + 0x4114
#define RFIF_GSM1_TP_INT_EN_XC4210                        RFIF_BASE + 0x4118
#define RFIF_GSM1_TP_INT_STATUS_XC4210                    RFIF_BASE + 0x411C
#define RFIF_GSM1_TP_INT_EN_X1643                         RFIF_BASE + 0x4120
#define RFIF_GSM1_TP_INT_STATUS_X1643                     RFIF_BASE + 0x4124
#define RFIF_GSM1_TP_INT_RAW                              RFIF_BASE + 0x4128
/*GSM1Êý¾ÝÍ¨Â·¼Ä´æÆ÷*/
#define RFIF_GSM1_CTRL                                    RFIF_BASE + 0x4140
#define RFIF_GSM1_TX_DATA_NUM                             RFIF_BASE + 0x4144
#define RFIF_GSM1_PREAMBLE_CTL                            RFIF_BASE + 0x4148
#define RFIF_GSM1_PREAMBLE_DATA                           RFIF_BASE + 0x414C
#define RFIF_GSM1_POSTAMBLE_CTL                           RFIF_BASE + 0x4150
#define RFIF_GSM1_POSTAMBLE_DATA                          RFIF_BASE + 0x4154
#define RFIF_GSM1_DP_INT_EN                               RFIF_BASE + 0x4170
#define RFIF_GSM1_DP_INT_STATUS                           RFIF_BASE + 0x4174
#define RFIF_GSM1_DP_INT_RAW                              RFIF_BASE + 0x4178
/*GSM1 SPI*/
#define RFIF_GSM1_SPI_EN                                  RFIF_BASE + 0x4190
#define RFIF_GSM1_SPI_CTRL                                RFIF_BASE + 0x4194
#define RFIF_GSM1_SPI_RX_CTRL                             RFIF_BASE + 0x4198
#define RFIF_GSM1_SPI_RST                                 RFIF_BASE + 0x419C
#define RFIF_GSM1_SPI_PARA                                RFIF_BASE + 0x41A4
/*GSM1³åÍ»ÖÐ¶Ï*/
#define RFIF_GSM1_CONFLICT_INT_EN                         RFIF_BASE + 0x4200
#define RFIF_GSM1_CONFLICT_INT_STATUS                     RFIF_BASE + 0x4204
#define RFIF_GSM1_CONFLICT_INT_RAW                        RFIF_BASE + 0x4208
/*µÍ¹¦ºÄ¿ØÖÆ*/
#define RFIF_GSM1_LP_CTRL                                 RFIF_BASE + 0x43FC
/*GSM1 SPIÊý¾Ý´æ´¢Æ÷*/
#define RFIF_GSM1_SPI_RAM                                 RFIF_BASE + 0x4400
/*GSM1 Ê±Ðò´¦ÀíÆ÷Ö¸Áî´æ´¢Æ÷*/
#define RFIF_GSM1_TP0_INSTR_RAM                           RFIF_BASE + 0x4800
#define RFIF_GSM1_TP1_INSTR_RAM                           RFIF_BASE + 0x4A00
#define RFIF_GSM1_TP2_INSTR_RAM                           RFIF_BASE + 0x4C00

/*******************************************************************************
DIGRFV4 registers' address on LC1860
********************************************************************************/
#define DIGRFV4_BLOCK_SEL                                 DIGRFV4_BASE + 0x03FC
#define DIGRFV4_MPHY_INLNCFG                              DIGRFV4_BASE + 0x03F0
#define DIGRFV4_MPHY_CFG_UPDT                             DIGRFV4_BASE + 0x03F8
#define DIGRFV4_MPHY_DA                                   DIGRFV4_BASE + 0x03F4
#define DIGRFV4_TASLC_PL0                                 DIGRFV4_BASE + 0x0000
#define DIGRFV4_TASLC_PL1                                 DIGRFV4_BASE + 0x0004
#define DIGRFV4_TASLC_PL2                                 DIGRFV4_BASE + 0x0008
#define DIGRFV4_TASLC_PL3                                 DIGRFV4_BASE + 0x000C
#define DIGRFV4_TASLC_CNT0                                DIGRFV4_BASE + 0x0010
#define DIGRFV4_TASLC_CNT1                                DIGRFV4_BASE + 0x0014
#define DIGRFV4_TASLC_CNT2                                DIGRFV4_BASE + 0x0018
#define DIGRFV4_TASLC_CNT3                                DIGRFV4_BASE + 0x001C
#define DIGRFV4_TXDLCB1C                                  DIGRFV4_BASE + 0x0040
#define DIGRFV4_TXDLCB2C                                  DIGRFV4_BASE + 0x0044
#define DIGRFV4_TXDLCB3C                                  DIGRFV4_BASE + 0x0048
#define DIGRFV4_TXDLCB4C                                  DIGRFV4_BASE + 0x004C
#define DIGRFV4_TXDLCB1CS                                 DIGRFV4_BASE + 0x0050
#define DIGRFV4_TXDLCB2CS                                 DIGRFV4_BASE + 0x0054
#define DIGRFV4_TXDLCB3CS                                 DIGRFV4_BASE + 0x0058
#define DIGRFV4_TXDLCB4CS                                 DIGRFV4_BASE + 0x005C
#define DIGRFV4_TXDLCINT0F                                DIGRFV4_BASE + 0x0060
#define DIGRFV4_TXDLCINT1F                                DIGRFV4_BASE + 0x0064
#define DIGRFV4_TXDLCINT0C                                DIGRFV4_BASE + 0x0068
#define DIGRFV4_TXDLCINT1C                                DIGRFV4_BASE + 0x006C
#define DIGRFV4_TXDLCINT1S                                DIGRFV4_BASE + 0x0070
#define DIGRFV4_RXDLCB1C                                  DIGRFV4_BASE + 0x0080
#define DIGRFV4_RXDLCB2C                                  DIGRFV4_BASE + 0x0084
#define DIGRFV4_RXDLCB3C                                  DIGRFV4_BASE + 0x0088
#define DIGRFV4_RXDLCB4C                                  DIGRFV4_BASE + 0x008C
#define DIGRFV4_RXDLCB1CS                                 DIGRFV4_BASE + 0x0090
#define DIGRFV4_RXDLCB2CS                                 DIGRFV4_BASE + 0x0094
#define DIGRFV4_RXDLCB3CS                                 DIGRFV4_BASE + 0x0098
#define DIGRFV4_RXDLCB4CS                                 DIGRFV4_BASE + 0x009C
#define DIGRFV4_RXDLCINT0F                                DIGRFV4_BASE + 0x00A0
#define DIGRFV4_RXDLCINT1F                                DIGRFV4_BASE + 0x00A4
#define DIGRFV4_RXDLCINT0C                                DIGRFV4_BASE + 0x00A8
#define DIGRFV4_RXDLCINT1C                                DIGRFV4_BASE + 0x00AC
#define DIGRFV4_TXCLC_CFG                                 DIGRFV4_BASE + 0x00C0
#define DIGRFV4_TXCLC_PL                                  DIGRFV4_BASE + 0x00C4
#define DIGRFV4_TXCLC_STS                                 DIGRFV4_BASE + 0x00C8
#define DIGRFV4_TXCLC_INT0F                               DIGRFV4_BASE + 0x00D0
#define DIGRFV4_TXCLC_INT1F                               DIGRFV4_BASE + 0x00D4
#define DIGRFV4_TXCLC_INT0C                               DIGRFV4_BASE + 0x00D8
#define DIGRFV4_TXCLC_INT1C                               DIGRFV4_BASE + 0x00DC
#define DIGRFV4_TXCLC_INT1S                               DIGRFV4_BASE + 0x00E0
#define DIGRFV4_TACLC_CNT                                 DIGRFV4_BASE + 0x00E4
#define DIGRFV4_RICLC_INFO                                DIGRFV4_BASE + 0x0100
#define DIGRFV4_RICLC_PL                                  DIGRFV4_BASE + 0x0104
#define DIGRFV4_RASLC_INFO                                DIGRFV4_BASE + 0x0108
#define DIGRFV4_RASLC_PL                                  DIGRFV4_BASE + 0x010C
#define DIGRFV4_RNCLC_INFO                                DIGRFV4_BASE + 0x0110
#define DIGRFV4_RNCLC_PL                                  DIGRFV4_BASE + 0x0114
#define DIGRFV4_RBCLC_INFO                                DIGRFV4_BASE + 0x0118
#define DIGRFV4_RBCLC_PL                                  DIGRFV4_BASE + 0x011C
#define DIGRFV4_RACLC_INFO                                DIGRFV4_BASE + 0x0120
#define DIGRFV4_RACLC_PL                                  DIGRFV4_BASE + 0x0124
#define DIGRFV4_RHLCLC_INFO                               DIGRFV4_BASE + 0x0128
#define DIGRFV4_RHLCLC_PL                                 DIGRFV4_BASE + 0x012C
#define DIGRFV4_RSCLC_INFO                                DIGRFV4_BASE + 0x0130
#define DIGRFV4_RSCLC_PL                                  DIGRFV4_BASE + 0x0134
#define DIGRFV4_RXCLC_INFO                                DIGRFV4_BASE + 0x0138
#define DIGRFV4_RXCLC_PL                                  DIGRFV4_BASE + 0x013C
#define DIGRFV4_RACLC_CNT                                 DIGRFV4_BASE + 0x0140
#define DIGRFV4_DEV_CTRL                                  DIGRFV4_BASE + 0x0180
#define DIGRFV4_FIFO_CLR                                  DIGRFV4_BASE + 0x0184
#define DIGRFV4_HS_SYNC_PAT                               DIGRFV4_BASE + 0x0188
#define DIGRFV4_REDUCED_LANE                              DIGRFV4_BASE + 0x018C
#define DIGRFV4_CFG_CHG_TIME                              DIGRFV4_BASE + 0x0198
#define DIGRFV4_APB_ERRF                                  DIGRFV4_BASE + 0x019C
#define DIGRFV4_APB_ERRC                                  DIGRFV4_BASE + 0x01A0
#define DIGRFV4_BIST_STS                                  DIGRFV4_BASE + 0x01A4
#define DIGRFV4_CLK_TEST_STS                              DIGRFV4_BASE + 0x01A8
#define DIGRFV4_LOGIC_LP_STS                              DIGRFV4_BASE + 0x01AC
#define DIGRFV4_ASSERT_DELAY_DIGRFEN                      DIGRFV4_BASE + 0x01B0
#define DIGRFV4_DEASSERT_DELAY_DIGRFEN                    DIGRFV4_BASE + 0x01B4
#define DIGRFV4_TEST_REG                                  DIGRFV4_BASE + 0x01B8
#define DIGRFV4_TEST_MODE                                 DIGRFV4_BASE + 0x01BC
#define DIGRFV4_MTX1_ST                                   DIGRFV4_BASE + 0x01C0
#define DIGRFV4_MRX1_ST                                   DIGRFV4_BASE + 0x01D0
#define DIGRFV4_MRX2_ST                                   DIGRFV4_BASE + 0x01D4
#define DIGRFV4_LANE_STATUS                               DIGRFV4_BASE + 0x01E0
#define DIGRFV4_TM_RST_HIBERN8_CTRL                       DIGRFV4_BASE + 0x01E8
#define DIGRFV4_TM_RST_HIBERN8                            DIGRFV4_BASE + 0x01EC
#define DIGRFV4_DEBUG_TP                                  DIGRFV4_BASE + 0x03E8

/*******************************************************************************
RF_DMAD registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define RF_DMAD_CH_STATUS                                 RF_DMAD_BASE + 0x0000
#define RF_DMAD_CH_PRIOR0                                 RF_DMAD_BASE + 0x0020
#define RF_DMAD_CH_PRIOR4                                 RF_DMAD_BASE + 0x0030
#define RF_DMAD_CH_PRIOR5                                 RF_DMAD_BASE + 0x0034
#define RF_DMAD_CH_INTR_EN0                               RF_DMAD_BASE + 0x0040
#define RF_DMAD_CH_INTR_MASK0                             RF_DMAD_BASE + 0x0044
#define RF_DMAD_CH_INTR_STATUS0                           RF_DMAD_BASE + 0x0048
#define RF_DMAD_CH_INTR_EN1                               RF_DMAD_BASE + 0x0050
#define RF_DMAD_CH_INTR_MASK1                             RF_DMAD_BASE + 0x0054
#define RF_DMAD_CH_INTR_STATUS1                           RF_DMAD_BASE + 0x0058
#define RF_DMAD_CH_LP_EN0                                 RF_DMAD_BASE + 0x0080
#define RF_DMAD_CH_LP_EN1                                 RF_DMAD_BASE + 0x0084
#define RF_DMAD_CH_BUS_LP_EN                              RF_DMAD_BASE + 0x0088
#define RF_DMAD_RAM_ADDR_BOUNDARY0_SRC                    RF_DMAD_BASE + 0x00A0
#define RF_DMAD_RAM_ADDR_BOUNDARY1_SRC                    RF_DMAD_BASE + 0x00A4
#define RF_DMAD_RAM_ADDR_BOUNDARY2_SRC                    RF_DMAD_BASE + 0x00B0
#define RF_DMAD_RAM_ADDR_BOUNDARY16_SRC                   RF_DMAD_BASE + 0x00E0
#define RF_DMAD_RAM_ADDR_BOUNDARY17_SRC                   RF_DMAD_BASE + 0x00E4
#define RF_DMAD_RAM_ADDR_BOUNDARY18_SRC                   RF_DMAD_BASE + 0x00E8
#define RF_DMAD_RAM_ADDR_BOUNDARY19_SRC                   RF_DMAD_BASE + 0x00EC
#define RF_DMAD_RAM_ADDR_BOUNDARY20_SRC                   RF_DMAD_BASE + 0x00F0
#define RF_DMAD_RAM_ADDR_BOUNDARY21_SRC                   RF_DMAD_BASE + 0x0018
#define RF_DMAD_RAM_ADDR_BOUNDARY0_DST                    RF_DMAD_BASE + 0x00A8
#define RF_DMAD_RAM_ADDR_BOUNDARY1_DST                    RF_DMAD_BASE + 0x00AC
#define RF_DMAD_RAM_ADDR_BOUNDARY2_DST                    RF_DMAD_BASE + 0x00B4
#define RF_DMAD_RAM_ADDR_BOUNDARY16_DST                   RF_DMAD_BASE + 0x00F4
#define RF_DMAD_RAM_ADDR_BOUNDARY17_DST                   RF_DMAD_BASE + 0x00F8
#define RF_DMAD_RAM_ADDR_BOUNDARY18_DST                   RF_DMAD_BASE + 0x00FC
#define RF_DMAD_RAM_ADDR_BOUNDARY19_DST                   RF_DMAD_BASE + 0x0010
#define RF_DMAD_RAM_ADDR_BOUNDARY20_DST                   RF_DMAD_BASE + 0x0014
#define RF_DMAD_RAM_ADDR_BOUNDARY21_DST                   RF_DMAD_BASE + 0x001C
/*Í¨µÀ0¼Ä´æÆ÷*/
#define RF_DMAD_CH0_CTRL                                  RF_DMAD_BASE + 0x0100
#define RF_DMAD_CH0_CONFIG                                RF_DMAD_BASE + 0x0104
#define RF_DMAD_CH0_SRC_ADDR                              RF_DMAD_BASE + 0x0108
#define RF_DMAD_CH0_DST_ADDR                              RF_DMAD_BASE + 0x0110
#define RF_DMAD_CH0_SIZE                                  RF_DMAD_BASE + 0x0118
#define RF_DMAD_CH0_LINK_ADDR                             RF_DMAD_BASE + 0x0124
#define RF_DMAD_CH0_LINK_NUM                              RF_DMAD_BASE + 0x0128
#define RF_DMAD_CH0_INTR_EN                               RF_DMAD_BASE + 0x012C
#define RF_DMAD_CH0_INTR_STATUS                           RF_DMAD_BASE + 0x0130
#define RF_DMAD_CH0_INTR_RAW                              RF_DMAD_BASE + 0x0134
#define RF_DMAD_CH0_MONITOR_CTRL                          RF_DMAD_BASE + 0x0138
#define RF_DMAD_CH0_MONITOR_OUT                           RF_DMAD_BASE + 0x013C
/*Í¨µÀ1¼Ä´æÆ÷*/
#define RF_DMAD_CH1_CTRL                                  RF_DMAD_BASE + 0x0140
#define RF_DMAD_CH1_CONFIG                                RF_DMAD_BASE + 0x0144
#define RF_DMAD_CH1_SRC_ADDR                              RF_DMAD_BASE + 0x0148
#define RF_DMAD_CH1_DST_ADDR                              RF_DMAD_BASE + 0x0150
#define RF_DMAD_CH1_SIZE                                  RF_DMAD_BASE + 0x0158
#define RF_DMAD_CH1_LINK_ADDR                             RF_DMAD_BASE + 0x0164
#define RF_DMAD_CH1_LINK_NUM                              RF_DMAD_BASE + 0x0168
#define RF_DMAD_CH1_INTR_EN                               RF_DMAD_BASE + 0x016C
#define RF_DMAD_CH1_INTR_STATUS                           RF_DMAD_BASE + 0x0170
#define RF_DMAD_CH1_INTR_RAW                              RF_DMAD_BASE + 0x0174
#define RF_DMAD_CH1_MONITOR_CTRL                          RF_DMAD_BASE + 0x0178
#define RF_DMAD_CH1_MONITOR_OUT                           RF_DMAD_BASE + 0x017C
/*Í¨µÀ2¼Ä´æÆ÷*/
#define RF_DMAD_CH2_CTRL                                  RF_DMAD_BASE + 0x0180
#define RF_DMAD_CH2_CONFIG                                RF_DMAD_BASE + 0x0184
#define RF_DMAD_CH2_SRC_ADDR                              RF_DMAD_BASE + 0x0188
#define RF_DMAD_CH2_DST_ADDR                              RF_DMAD_BASE + 0x0190
#define RF_DMAD_CH2_SIZE                                  RF_DMAD_BASE + 0x0198
#define RF_DMAD_CH2_LINK_ADDR                             RF_DMAD_BASE + 0x01A4
#define RF_DMAD_CH2_LINK_NUM                              RF_DMAD_BASE + 0x01A8
#define RF_DMAD_CH2_INTR_EN                               RF_DMAD_BASE + 0x01AC
#define RF_DMAD_CH2_INTR_STATUS                           RF_DMAD_BASE + 0x01B0
#define RF_DMAD_CH2_INTR_RAW                              RF_DMAD_BASE + 0x01B4
#define RF_DMAD_CH2_MONITOR_CTRL                          RF_DMAD_BASE + 0x01B8
#define RF_DMAD_CH2_MONITOR_OUT                           RF_DMAD_BASE + 0x01BC
/*Í¨µÀ16¼Ä´æÆ÷*/
#define RF_DMAD_CH16_CTRL                                 RF_DMAD_BASE + 0x0500
#define RF_DMAD_CH16_CONFIG                               RF_DMAD_BASE + 0x0504
#define RF_DMAD_CH16_SRC_ADDR                             RF_DMAD_BASE + 0x0508
#define RF_DMAD_CH16_DST_ADDR                             RF_DMAD_BASE + 0x0510
#define RF_DMAD_CH16_SIZE                                 RF_DMAD_BASE + 0x0518
#define RF_DMAD_CH16_LINK_ADDR                            RF_DMAD_BASE + 0x0524
#define RF_DMAD_CH16_LINK_NUM                             RF_DMAD_BASE + 0x0528
#define RF_DMAD_CH16_INTR_EN                              RF_DMAD_BASE + 0x052C
#define RF_DMAD_CH16_INTR_STATUS                          RF_DMAD_BASE + 0x0530
#define RF_DMAD_CH16_INTR_RAW                             RF_DMAD_BASE + 0x0534
#define RF_DMAD_CH16_MONITOR_CTRL                         RF_DMAD_BASE + 0x0538
#define RF_DMAD_CH16_MONITOR_OUT                          RF_DMAD_BASE + 0x053C
/*Í¨µÀ17¼Ä´æÆ÷*/
#define RF_DMAD_CH17_CTRL                                 RF_DMAD_BASE + 0x0540
#define RF_DMAD_CH17_CONFIG                               RF_DMAD_BASE + 0x0544
#define RF_DMAD_CH17_SRC_ADDR                             RF_DMAD_BASE + 0x0548
#define RF_DMAD_CH17_DST_ADDR                             RF_DMAD_BASE + 0x0550
#define RF_DMAD_CH17_SIZE                                 RF_DMAD_BASE + 0x0558
#define RF_DMAD_CH17_LINK_ADDR                            RF_DMAD_BASE + 0x0564
#define RF_DMAD_CH17_LINK_NUM                             RF_DMAD_BASE + 0x0568
#define RF_DMAD_CH17_INTR_EN                              RF_DMAD_BASE + 0x056C
#define RF_DMAD_CH17_INTR_STATUS                          RF_DMAD_BASE + 0x0570
#define RF_DMAD_CH17_INTR_RAW                             RF_DMAD_BASE + 0x0574
#define RF_DMAD_CH17_MONITOR_CTRL                         RF_DMAD_BASE + 0x0578
#define RF_DMAD_CH17_MONITOR_OUT                          RF_DMAD_BASE + 0x057C
/*Í¨µÀ18¼Ä´æÆ÷*/
#define RF_DMAD_CH18_CTRL                                 RF_DMAD_BASE + 0x0580
#define RF_DMAD_CH18_CONFIG                               RF_DMAD_BASE + 0x0584
#define RF_DMAD_CH18_SRC_ADDR                             RF_DMAD_BASE + 0x0588
#define RF_DMAD_CH18_DST_ADDR                             RF_DMAD_BASE + 0x0590
#define RF_DMAD_CH18_SIZE                                 RF_DMAD_BASE + 0x0598
#define RF_DMAD_CH18_LINK_ADDR                            RF_DMAD_BASE + 0x05A4
#define RF_DMAD_CH18_LINK_NUM                             RF_DMAD_BASE + 0x05A8
#define RF_DMAD_CH18_INTR_EN                              RF_DMAD_BASE + 0x05AC
#define RF_DMAD_CH18_INTR_STATUS                          RF_DMAD_BASE + 0x05B0
#define RF_DMAD_CH18_INTR_RAW                             RF_DMAD_BASE + 0x05B4
#define RF_DMAD_CH18_MONITOR_CTRL                         RF_DMAD_BASE + 0x05B8
#define RF_DMAD_CH18_MONITOR_OUT                          RF_DMAD_BASE + 0x05BC
/*Í¨µÀ19¼Ä´æÆ÷*/
#define RF_DMAD_CH19_CTRL                                 RF_DMAD_BASE + 0x05C0
#define RF_DMAD_CH19_CONFIG                               RF_DMAD_BASE + 0x05C4
#define RF_DMAD_CH19_SRC_ADDR                             RF_DMAD_BASE + 0x05C8
#define RF_DMAD_CH19_DST_ADDR                             RF_DMAD_BASE + 0x05D0
#define RF_DMAD_CH19_SIZE                                 RF_DMAD_BASE + 0x05D8
#define RF_DMAD_CH19_LINK_ADDR                            RF_DMAD_BASE + 0x05E4
#define RF_DMAD_CH19_LINK_NUM                             RF_DMAD_BASE + 0x05E8
#define RF_DMAD_CH19_INTR_EN                              RF_DMAD_BASE + 0x05EC
#define RF_DMAD_CH19_INTR_STATUS                          RF_DMAD_BASE + 0x05F0
#define RF_DMAD_CH19_INTR_RAW                             RF_DMAD_BASE + 0x05F4
#define RF_DMAD_CH19_MONITOR_CTRL                         RF_DMAD_BASE + 0x05F8
#define RF_DMAD_CH19_MONITOR_OUT                          RF_DMAD_BASE + 0x05FC
/*Í¨µÀ20¼Ä´æÆ÷*/
#define RF_DMAD_CH20_CTRL                                 RF_DMAD_BASE + 0x0600
#define RF_DMAD_CH20_CONFIG                               RF_DMAD_BASE + 0x0604
#define RF_DMAD_CH20_SRC_ADDR                             RF_DMAD_BASE + 0x0608
#define RF_DMAD_CH20_DST_ADDR                             RF_DMAD_BASE + 0x0610
#define RF_DMAD_CH20_SIZE                                 RF_DMAD_BASE + 0x0618
#define RF_DMAD_CH20_LINK_ADDR                            RF_DMAD_BASE + 0x0624
#define RF_DMAD_CH20_LINK_NUM                             RF_DMAD_BASE + 0x0628
#define RF_DMAD_CH20_INTR_EN                              RF_DMAD_BASE + 0x062C
#define RF_DMAD_CH20_INTR_STATUS                          RF_DMAD_BASE + 0x0630
#define RF_DMAD_CH20_INTR_RAW                             RF_DMAD_BASE + 0x0634
#define RF_DMAD_CH20_MONITOR_CTRL                         RF_DMAD_BASE + 0x0638
#define RF_DMAD_CH20_MONITOR_OUT                          RF_DMAD_BASE + 0x063C
/*Í¨µÀ21¼Ä´æÆ÷*/
#define RF_DMAD_CH21_CTRL                                 RF_DMAD_BASE + 0x0640
#define RF_DMAD_CH21_CONFIG                               RF_DMAD_BASE + 0x0644
#define RF_DMAD_CH21_SRC_ADDR                             RF_DMAD_BASE + 0x0648
#define RF_DMAD_CH21_DST_ADDR                             RF_DMAD_BASE + 0x0650
#define RF_DMAD_CH21_SIZE                                 RF_DMAD_BASE + 0x0658
#define RF_DMAD_CH21_LINK_ADDR                            RF_DMAD_BASE + 0x0664
#define RF_DMAD_CH21_LINK_NUM                             RF_DMAD_BASE + 0x0668
#define RF_DMAD_CH21_INTR_EN                              RF_DMAD_BASE + 0x066C
#define RF_DMAD_CH21_INTR_STATUS                          RF_DMAD_BASE + 0x0670
#define RF_DMAD_CH21_INTR_RAW                             RF_DMAD_BASE + 0x0674
#define RF_DMAD_CH21_MONITOR_CTRL                         RF_DMAD_BASE + 0x0678
#define RF_DMAD_CH21_MONITOR_OUT                          RF_DMAD_BASE + 0x067C
/*ÄÚ²¿RAM½Ó¿Ú*/
#define RF_DMAD_RAM_DATA                                  RF_DMAD_BASE + 0x2000

/*******************************************************************************
THU registers' address on LC1860
********************************************************************************/
#define THU_VERSION                                       THU_BASE + 0x0000
#define THU_RESET                                         THU_BASE + 0x0004
#define THU_RESERVED                                      THU_BASE + 0x0008
#define THU_RESERVED2                                     THU_BASE + 0x000C
#define THU_INT_STATUS_X1643                              THU_BASE + 0x0010
#define THU_INT_EN_X1643                                  THU_BASE + 0x0014
#define THU_INT_MASK_X1643                                THU_BASE + 0x0018
#define THU_ERROR_STATUS_X1643                            THU_BASE + 0x001C
#define THU_ERROR_EN_X1643                                THU_BASE + 0x0020
#define THU_ERROR_MASK_X1643                              THU_BASE + 0x0024
#define THU_INT_STATUS_RAW_X1643                          THU_BASE + 0x0028
#define THU_ERROR_SATUS_RAW_X1643                         THU_BASE + 0x002C
#define THU_INT_STATUS_RAW_XC4210                         THU_BASE + 0x0030
#define THU_INT_STATUS_XC4210                             THU_BASE + 0x0034
#define THU_INT_EN_XC4210                                 THU_BASE + 0x0038
#define THU_INT_MASK_XC4210                               THU_BASE + 0x003C
#define THU_ERROR_STATUS_XC4210                           THU_BASE + 0x0040
#define THU_ERROR_EN_XC4210                               THU_BASE + 0x0044
#define THU_ERROR_MASK_XC4210                             THU_BASE + 0x0048
#define THU_ERROR_SATUS_RAW_XC4210                        THU_BASE + 0x004C
/*¼Ä´æÆ÷Ãû³Æ*/
#define THU_UDP_ACT                                       THU_BASE + 0x1000
#define THU_UDP_CC_START                                  THU_BASE + 0x1010
#define THU_UDP_BC_START                                  THU_BASE + 0x1014
#define THU_UDP_BC_FLAG                                   THU_BASE + 0x101C
#define THU_UDP_BYPASS                                    THU_BASE + 0x1020
#define THU_UDP_FRAME_FLAG                                THU_BASE + 0x1024
/*¼Ä´æÆ÷Ãû³Æ*/
#define THU_UDP_CCF0_PAR1                                 THU_BASE + 0x2000
#define THU_UDP_CCF0_PAR2                                 THU_BASE + 0x2004
#define THU_UDP_CCF0_PAR3                                 THU_BASE + 0x2008
#define THU_UDP_CCF1_PAR1                                 THU_BASE + 0x200C
#define THU_UDP_CCF1_PAR2                                 THU_BASE + 0x2010
#define THU_UDP_CCF1_PAR3                                 THU_BASE + 0x2014
#define THU_UDP_CCF2_PAR1                                 THU_BASE + 0x2018
#define THU_UDP_CCF2_PAR2                                 THU_BASE + 0x201C
#define THU_UDP_CCF2_PAR3                                 THU_BASE + 0x2020
#define THU_UDP_CCF3_PAR1                                 THU_BASE + 0x2024
#define THU_UDP_CCF3_PAR2                                 THU_BASE + 0x2028
#define THU_UDP_CCF3_PAR3                                 THU_BASE + 0x202C
#define THU_UDP_CCF4_PAR1                                 THU_BASE + 0x2030
#define THU_UDP_CCF4_PAR2                                 THU_BASE + 0x2034
#define THU_UDP_CCF4_PAR3                                 THU_BASE + 0x2038
#define THU_UDP_CCF5_PAR1                                 THU_BASE + 0x203C
#define THU_UDP_CCF5_PAR2                                 THU_BASE + 0x2040
#define THU_UDP_CCF5_PAR3                                 THU_BASE + 0x2044
#define THU_UDP_CCF6_PAR1                                 THU_BASE + 0x2048
#define THU_UDP_CCF6_PAR2                                 THU_BASE + 0x204C
#define THU_UDP_CCF6_PAR3                                 THU_BASE + 0x2050
#define THU_UDP_CCF7_PAR1                                 THU_BASE + 0x2054
#define THU_UDP_CCF7_PAR2                                 THU_BASE + 0x2058
#define THU_UDP_CCF7_PAR3                                 THU_BASE + 0x205C
#define THU_UPA_EDF_PAR1                                  THU_BASE + 0x2060
#define THU_UPA_EDF_PAR2                                  THU_BASE + 0x2064
#define THU_UPA_EDF_PAR3                                  THU_BASE + 0x2068
#define THU_UDP_CCB_RM0_PAR0                              THU_BASE + 0x206C
#define THU_UDP_CCB_RM0_PAR1                              THU_BASE + 0x2070
#define THU_UDP_CCB_RM0_PAR2                              THU_BASE + 0x2074
#define THU_UDP_CCB_RM0_PAR3                              THU_BASE + 0x2078
#define THU_UDP_CCB_RM0_PAR4                              THU_BASE + 0x207C
#define THU_UDP_CCB_RM1_PAR0                              THU_BASE + 0x2080
#define THU_UDP_CCB_RM1_PAR1                              THU_BASE + 0x2084
#define THU_UDP_CCB_RM1_PAR2                              THU_BASE + 0x2088
#define THU_UDP_CCB_RM1_PAR3                              THU_BASE + 0x208C
#define THU_UDP_CCB_RM1_PAR4                              THU_BASE + 0x2090
#define THU_UDP_CCB_RM2_PAR0                              THU_BASE + 0x2094
#define THU_UDP_CCB_RM2_PAR1                              THU_BASE + 0x2098
#define THU_UDP_CCB_RM2_PAR2                              THU_BASE + 0x209C
#define THU_UDP_CCB_RM2_PAR3                              THU_BASE + 0x20A0
#define THU_UDP_CCB_RM2_PAR4                              THU_BASE + 0x20A4
#define THU_UDP_CCB_RM3_PAR0                              THU_BASE + 0x20A8
#define THU_UDP_CCB_RM3_PAR1                              THU_BASE + 0x20AC
#define THU_UDP_CCB_RM3_PAR2                              THU_BASE + 0x20B0
#define THU_UDP_CCB_RM3_PAR3                              THU_BASE + 0x20B4
#define THU_UDP_CCB_RM3_PAR4                              THU_BASE + 0x20B8
#define THU_UDP_CCB_RM4_PAR0                              THU_BASE + 0x20BC
#define THU_UDP_CCB_RM4_PAR1                              THU_BASE + 0x20C0
#define THU_UDP_CCB_RM4_PAR2                              THU_BASE + 0x20C4
#define THU_UDP_CCB_RM4_PAR3                              THU_BASE + 0x20C8
#define THU_UDP_CCB_RM4_PAR4                              THU_BASE + 0x20CC
#define THU_UDP_CCB_RM5_PAR0                              THU_BASE + 0x20D0
#define THU_UDP_CCB_RM5_PAR1                              THU_BASE + 0x20D4
#define THU_UDP_CCB_RM5_PAR2                              THU_BASE + 0x20D8
#define THU_UDP_CCB_RM5_PAR3                              THU_BASE + 0x20DC
#define THU_UDP_CCB_RM5_PAR4                              THU_BASE + 0x20E0
#define THU_UDP_CCB_RM6_PAR0                              THU_BASE + 0x20E4
#define THU_UDP_CCB_RM6_PAR1                              THU_BASE + 0x20E8
#define THU_UDP_CCB_RM6_PAR2                              THU_BASE + 0x20EC
#define THU_UDP_CCB_RM6_PAR3                              THU_BASE + 0x20F0
#define THU_UDP_CCB_RM6_PAR4                              THU_BASE + 0x20F4
#define THU_UDP_CCB_RM7_PAR0                              THU_BASE + 0x20F8
#define THU_UDP_CCB_RM7_PAR1                              THU_BASE + 0x20FC
#define THU_UDP_CCB_RM7_PAR2                              THU_BASE + 0x2100
#define THU_UDP_CCB_RM7_PAR3                              THU_BASE + 0x2104
#define THU_UDP_CCB_RM7_PAR4                              THU_BASE + 0x2108
#define THU_UDP_CCB_INT_PAR0                              THU_BASE + 0x210C
#define THU_UDP_CCB_INT_PAR1                              THU_BASE + 0x2110
#define THU_UDP_CCB_INT_PAR2                              THU_BASE + 0x2114
#define THU_UDP_CCB_INT_PAR3                              THU_BASE + 0x2118
#define THU_UDP_CCB_INT_PAR4                              THU_BASE + 0x211C
#define THU_UPA_EDB_RM_PAR0                               THU_BASE + 0x2120
#define THU_UPA_EDB_RM_PAR1                               THU_BASE + 0x2124
#define THU_UPA_EDB_RM_PAR2                               THU_BASE + 0x2128
#define THU_UPA_EDB_RM_PAR3                               THU_BASE + 0x212C
#define THU_UPA_EDB_RM_PAR4                               THU_BASE + 0x2130
#define THU_UPA_EDB_RM_PAR5                               THU_BASE + 0x2134
#define THU_UPA_EDB_RM_PAR6                               THU_BASE + 0x2138
#define THU_UPA_EDB_HARQ_PAR                              THU_BASE + 0x213C
#define THU_UDP_CCB_FRA_ADD                               THU_BASE + 0x2140
/*¼Ä´æÆ÷Ãû³Æ*/
#define THU_BC_PHY1_CTR1                                  THU_BASE + 0x2200
#define THU_BC_PHY1_CTR2                                  THU_BASE + 0x2204
#define THU_BC_PHY1_CTR3                                  THU_BASE + 0x2208
#define THU_BC_PHY1_CTR4                                  THU_BASE + 0x220C
#define THU_BC_PHY1_CTR5                                  THU_BASE + 0x2210
#define THU_BC_PHY1_CTR6                                  THU_BASE + 0x2214
#define THU_BC_PHY1_CTR7                                  THU_BASE + 0x2218
#define THU_BC_PHY1_CTR8                                  THU_BASE + 0x221C
#define THU_BC_PHY1_CTR9                                  THU_BASE + 0x2220
#define THU_BC_PHY2_CTR1                                  THU_BASE + 0x2224
#define THU_BC_PHY2_CTR2                                  THU_BASE + 0x2228
#define THU_BC_PHY2_CTR3                                  THU_BASE + 0x222C
#define THU_BC_PHY2_CTR4                                  THU_BASE + 0x2230
#define THU_BC_PHY2_CTR5                                  THU_BASE + 0x2234
#define THU_BC_PHY2_CTR6                                  THU_BASE + 0x2238
#define THU_BC_PHY2_CTR7                                  THU_BASE + 0x223C
#define THU_BC_PHY2_CTR8                                  THU_BASE + 0x2240
#define THU_BC_PHY2_CTR9                                  THU_BASE + 0x2244
#define THU_BC_PHY3_CTR1                                  THU_BASE + 0x2248
#define THU_BC_PHY3_CTR2                                  THU_BASE + 0x224C
#define THU_BC_PHY3_CTR3                                  THU_BASE + 0x2250
#define THU_BC_PHY3_CTR4                                  THU_BASE + 0x2254
#define THU_BC_PHY3_CTR5                                  THU_BASE + 0x2258
#define THU_BC_PHY3_CTR6                                  THU_BASE + 0x225C
#define THU_BC_PHY3_CTR7                                  THU_BASE + 0x2260
#define THU_BC_PHY3_CTR8                                  THU_BASE + 0x2264
#define THU_BC_PHY3_CTR9                                  THU_BASE + 0x2268
#define THU_BC_PHY4_CTR1                                  THU_BASE + 0x226C
#define THU_BC_PHY4_CTR2                                  THU_BASE + 0x2270
#define THU_BC_PHY4_CTR3                                  THU_BASE + 0x2274
#define THU_BC_PHY4_CTR4                                  THU_BASE + 0x2278
#define THU_BC_PHY4_CTR5                                  THU_BASE + 0x227C
#define THU_BC_PHY4_CTR6                                  THU_BASE + 0x2280
#define THU_BC_PHY4_CTR7                                  THU_BASE + 0x2284
#define THU_BC_PHY4_CTR8                                  THU_BASE + 0x2288
#define THU_BC_PHY4_CTR9                                  THU_BASE + 0x228C
#define THU_BC_MID1                                       THU_BASE + 0x2290
#define THU_BC_MID2                                       THU_BASE + 0x2294
#define THU_BC_MID3                                       THU_BASE + 0x2298
#define THU_BC_MID4                                       THU_BASE + 0x229C
#define THU_BC_PARA1                                      THU_BASE + 0x22A0
#define THU_BC_PARA2                                      THU_BASE + 0x22A4
#define THU_BC_PARA3                                      THU_BASE + 0x22A8
#define THU_BC_PARA4                                      THU_BASE + 0x22AC
#define THU_BC_PARA5                                      THU_BASE + 0x22B0
#define THU_BC_PARA6                                      THU_BASE + 0x22B4
#define THU_BC_PARA7                                      THU_BASE + 0x22B8
#define THU_BC_PARA8                                      THU_BASE + 0x22BC
#define THU_BC_PARA9                                      THU_BASE + 0x22C0
#define THU_BC_PARA10                                     THU_BASE + 0x22C4
#define THU_BC_PARA11                                     THU_BASE + 0x22C8
#define THU_BC_PARA12                                     THU_BASE + 0x22CC
#define THU_BC_PARA13                                     THU_BASE + 0x22D0
#define THU_BC_PARA14                                     THU_BASE + 0x22D4
#define THU_BC_PARA15                                     THU_BASE + 0x22D8
#define THU_BC_PARA16                                     THU_BASE + 0x22DC
#define THU_BC_PARA17                                     THU_BASE + 0x22E0

/*******************************************************************************
CP_DMAD registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define CP_DMAD_CH_STATUS                                 CP_DMAD_BASE + 0x0000
#define CP_DMAD_CH_PRIOR0                                 CP_DMAD_BASE + 0x0020
#define CP_DMAD_CH_PRIOR1                                 CP_DMAD_BASE + 0x0024
#define CP_DMAD_CH_PRIOR2                                 CP_DMAD_BASE + 0x0028
#define CP_DMAD_CH_INTR_EN0                               CP_DMAD_BASE + 0x0040
#define CP_DMAD_CH_INTR_MASK0                             CP_DMAD_BASE + 0x0044
#define CP_DMAD_CH_INTR_STATUS0                           CP_DMAD_BASE + 0x0048
#define CP_DMAD_CH_INTR_EN1                               CP_DMAD_BASE + 0x0050
#define CP_DMAD_CH_INTR_MASK1                             CP_DMAD_BASE + 0x0054
#define CP_DMAD_CH_INTR_STATUS1                           CP_DMAD_BASE + 0x0058
#define CP_DMAD_CH_INTR_EN2                               CP_DMAD_BASE + 0x0060
#define CP_DMAD_CH_INTR_MASK2                             CP_DMAD_BASE + 0x0064
#define CP_DMAD_CH_INTR_STATUS2                           CP_DMAD_BASE + 0x0068
#define CP_DMAD_CH_LP_EN0                                 CP_DMAD_BASE + 0x0080
#define CP_DMAD_CH_BUS_LP_EN                              CP_DMAD_BASE + 0x0088
#define CP_DMAD_RAM_ADDR_BOUNDARY0_SRC                    CP_DMAD_BASE + 0x00A0
#define CP_DMAD_RAM_ADDR_BOUNDARY1_SRC                    CP_DMAD_BASE + 0x00A4
#define CP_DMAD_RAM_ADDR_BOUNDARY2_SRC                    CP_DMAD_BASE + 0x00A8
#define CP_DMAD_RAM_ADDR_BOUNDARY4_SRC                    CP_DMAD_BASE + 0x00B0
#define CP_DMAD_RAM_ADDR_BOUNDARY5_SRC                    CP_DMAD_BASE + 0x00B4
#define CP_DMAD_RAM_ADDR_BOUNDARY6_SRC                    CP_DMAD_BASE + 0x00B8
#define CP_DMAD_RAM_ADDR_BOUNDARY7_SRC                    CP_DMAD_BASE + 0x00BC
#define CP_DMAD_RAM_ADDR_BOUNDARY8_SRC                    CP_DMAD_BASE + 0x00C0
#define CP_DMAD_RAM_ADDR_BOUNDARY9_SRC                    CP_DMAD_BASE + 0x00C4
#define CP_DMAD_RAM_ADDR_BOUNDARY0_DST                    CP_DMAD_BASE + 0x00E0
#define CP_DMAD_RAM_ADDR_BOUNDARY1_DST                    CP_DMAD_BASE + 0x00E4
#define CP_DMAD_RAM_ADDR_BOUNDARY2_DST                    CP_DMAD_BASE + 0x00E8
#define CP_DMAD_RAM_ADDR_BOUNDARY4_DST                    CP_DMAD_BASE + 0x00F0
#define CP_DMAD_RAM_ADDR_BOUNDARY5_DST                    CP_DMAD_BASE + 0x00F4
#define CP_DMAD_RAM_ADDR_BOUNDARY6_DST                    CP_DMAD_BASE + 0x00F8
#define CP_DMAD_RAM_ADDR_BOUNDARY7_DST                    CP_DMAD_BASE + 0x00FC
#define CP_DMAD_RAM_ADDR_BOUNDARY8_DST                    CP_DMAD_BASE + 0x0010
#define CP_DMAD_RAM_ADDR_BOUNDARY9_DST                    CP_DMAD_BASE + 0x0014
/*Í¨µÀ0¼Ä´æÆ÷*/
#define CP_DMAD_CH0_CTRL                                  CP_DMAD_BASE + 0x0100
#define CP_DMAD_CH0_CONFIG                                CP_DMAD_BASE + 0x0104
#define CP_DMAD_CH0_SRC_ADDR                              CP_DMAD_BASE + 0x0108
#define CP_DMAD_CH0_DST_ADDR                              CP_DMAD_BASE + 0x0110
#define CP_DMAD_CH0_SIZE                                  CP_DMAD_BASE + 0x0118
#define CP_DMAD_CH0_LINK_ADDR                             CP_DMAD_BASE + 0x0124
#define CP_DMAD_CH0_LINK_NUM                              CP_DMAD_BASE + 0x0128
#define CP_DMAD_CH0_INTR_EN                               CP_DMAD_BASE + 0x012C
#define CP_DMAD_CH0_INTR_STATUS                           CP_DMAD_BASE + 0x0130
#define CP_DMAD_CH0_INTR_RAW                              CP_DMAD_BASE + 0x0134
#define CP_DMAD_CH0_MONITOR_CTRL                          CP_DMAD_BASE + 0x0138
#define CP_DMAD_CH0_MONITOR_OUT                           CP_DMAD_BASE + 0x013C
/*Í¨µÀ1¼Ä´æÆ÷*/
#define CP_DMAD_CH1_CTRL                                  CP_DMAD_BASE + 0x0140
#define CP_DMAD_CH1_CONFIG                                CP_DMAD_BASE + 0x0144
#define CP_DMAD_CH1_SRC_ADDR                              CP_DMAD_BASE + 0x0148
#define CP_DMAD_CH1_DST_ADDR                              CP_DMAD_BASE + 0x0150
#define CP_DMAD_CH1_SIZE                                  CP_DMAD_BASE + 0x0158
#define CP_DMAD_CH1_LINK_ADDR                             CP_DMAD_BASE + 0x0164
#define CP_DMAD_CH1_LINK_NUM                              CP_DMAD_BASE + 0x0168
#define CP_DMAD_CH1_INTR_EN                               CP_DMAD_BASE + 0x016C
#define CP_DMAD_CH1_INTR_STATUS                           CP_DMAD_BASE + 0x0170
#define CP_DMAD_CH1_INTR_RAW                              CP_DMAD_BASE + 0x0174
#define CP_DMAD_CH1_MONITOR_CTRL                          CP_DMAD_BASE + 0x0178
#define CP_DMAD_CH1_MONITOR_OUT                           CP_DMAD_BASE + 0x017C
/*Í¨µÀ2¼Ä´æÆ÷*/
#define CP_DMAD_CH2_CTRL                                  CP_DMAD_BASE + 0x0180
#define CP_DMAD_CH2_CONFIG                                CP_DMAD_BASE + 0x0184
#define CP_DMAD_CH2_SRC_ADDR                              CP_DMAD_BASE + 0x0188
#define CP_DMAD_CH2_DST_ADDR                              CP_DMAD_BASE + 0x0190
#define CP_DMAD_CH2_SIZE                                  CP_DMAD_BASE + 0x0198
#define CP_DMAD_CH2_LINK_ADDR                             CP_DMAD_BASE + 0x01A4
#define CP_DMAD_CH2_LINK_NUM                              CP_DMAD_BASE + 0x01A8
#define CP_DMAD_CH2_INTR_EN                               CP_DMAD_BASE + 0x01AC
#define CP_DMAD_CH2_INTR_STATUS                           CP_DMAD_BASE + 0x01B0
#define CP_DMAD_CH2_INTR_RAW                              CP_DMAD_BASE + 0x01B4
#define CP_DMAD_CH2_MONITOR_CTRL                          CP_DMAD_BASE + 0x01B8
#define CP_DMAD_CH2_MONITOR_OUT                           CP_DMAD_BASE + 0x01BC
/*Í¨µÀ4¼Ä´æÆ÷*/
#define CP_DMAD_CH4_CTRL                                  CP_DMAD_BASE + 0x0200
#define CP_DMAD_CH4_CONFIG                                CP_DMAD_BASE + 0x0204
#define CP_DMAD_CH4_SRC_ADDR                              CP_DMAD_BASE + 0x0208
#define CP_DMAD_CH4_DST_ADDR                              CP_DMAD_BASE + 0x0210
#define CP_DMAD_CH4_SIZE                                  CP_DMAD_BASE + 0x0218
#define CP_DMAD_CH4_LINK_ADDR                             CP_DMAD_BASE + 0x0224
#define CP_DMAD_CH4_LINK_NUM                              CP_DMAD_BASE + 0x0228
#define CP_DMAD_CH4_INTR_EN                               CP_DMAD_BASE + 0x022C
#define CP_DMAD_CH4_INTR_STATUS                           CP_DMAD_BASE + 0x0230
#define CP_DMAD_CH4_INTR_RAW                              CP_DMAD_BASE + 0x0234
#define CP_DMAD_CH4_MONITOR_CTRL                          CP_DMAD_BASE + 0x0238
#define CP_DMAD_CH4_MONITOR_OUT                           CP_DMAD_BASE + 0x023C
/*Í¨µÀ5¼Ä´æÆ÷*/
#define CP_DMAD_CH5_CTRL                                  CP_DMAD_BASE + 0x0240
#define CP_DMAD_CH5_CONFIG                                CP_DMAD_BASE + 0x0244
#define CP_DMAD_CH5_SRC_ADDR                              CP_DMAD_BASE + 0x0248
#define CP_DMAD_CH5_DST_ADDR                              CP_DMAD_BASE + 0x0250
#define CP_DMAD_CH5_SIZE                                  CP_DMAD_BASE + 0x0258
#define CP_DMAD_CH5_LINK_ADDR                             CP_DMAD_BASE + 0x0264
#define CP_DMAD_CH5_LINK_NUM                              CP_DMAD_BASE + 0x0268
#define CP_DMAD_CH5_INTR_EN                               CP_DMAD_BASE + 0x026C
#define CP_DMAD_CH5_INTR_STATUS                           CP_DMAD_BASE + 0x0270
#define CP_DMAD_CH5_INTR_RAW                              CP_DMAD_BASE + 0x0274
#define CP_DMAD_CH5_MONITOR_CTRL                          CP_DMAD_BASE + 0x0278
#define CP_DMAD_CH5_MONITOR_OUT                           CP_DMAD_BASE + 0x027C
/*Í¨µÀ6¼Ä´æÆ÷*/
#define CP_DMAD_CH6_CTRL                                  CP_DMAD_BASE + 0x0280
#define CP_DMAD_CH6_CONFIG                                CP_DMAD_BASE + 0x0284
#define CP_DMAD_CH6_SRC_ADDR                              CP_DMAD_BASE + 0x0288
#define CP_DMAD_CH6_DST_ADDR                              CP_DMAD_BASE + 0x0290
#define CP_DMAD_CH6_SIZE                                  CP_DMAD_BASE + 0x0298
#define CP_DMAD_CH6_LINK_ADDR                             CP_DMAD_BASE + 0x02A4
#define CP_DMAD_CH6_LINK_NUM                              CP_DMAD_BASE + 0x02A8
#define CP_DMAD_CH6_INTR_EN                               CP_DMAD_BASE + 0x02AC
#define CP_DMAD_CH6_INTR_STATUS                           CP_DMAD_BASE + 0x02B0
#define CP_DMAD_CH6_INTR_RAW                              CP_DMAD_BASE + 0x02B4
#define CP_DMAD_CH6_MONITOR_CTRL                          CP_DMAD_BASE + 0x02B8
#define CP_DMAD_CH6_MONITOR_OUT                           CP_DMAD_BASE + 0x02BC
/*Í¨µÀ7¼Ä´æÆ÷*/
#define CP_DMAD_CH7_CTRL                                  CP_DMAD_BASE + 0x02C0
#define CP_DMAD_CH7_CONFIG                                CP_DMAD_BASE + 0x02C4
#define CP_DMAD_CH7_SRC_ADDR                              CP_DMAD_BASE + 0x02C8
#define CP_DMAD_CH7_DST_ADDR                              CP_DMAD_BASE + 0x02D0
#define CP_DMAD_CH7_SIZE                                  CP_DMAD_BASE + 0x02D8
#define CP_DMAD_CH7_LINK_ADDR                             CP_DMAD_BASE + 0x02E4
#define CP_DMAD_CH7_LINK_NUM                              CP_DMAD_BASE + 0x02E8
#define CP_DMAD_CH7_INTR_EN                               CP_DMAD_BASE + 0x02EC
#define CP_DMAD_CH7_INTR_STATUS                           CP_DMAD_BASE + 0x02F0
#define CP_DMAD_CH7_INTR_RAW                              CP_DMAD_BASE + 0x02F4
#define CP_DMAD_CH7_MONITOR_CTRL                          CP_DMAD_BASE + 0x02F8
#define CP_DMAD_CH7_MONITOR_OUT                           CP_DMAD_BASE + 0x02FC
/*Í¨µÀ8¼Ä´æÆ÷*/
#define CP_DMAD_CH8_CTRL                                  CP_DMAD_BASE + 0x0300
#define CP_DMAD_CH8_CONFIG                                CP_DMAD_BASE + 0x0304
#define CP_DMAD_CH8_SRC_ADDR                              CP_DMAD_BASE + 0x0308
#define CP_DMAD_CH8_DST_ADDR                              CP_DMAD_BASE + 0x0310
#define CP_DMAD_CH8_SIZE                                  CP_DMAD_BASE + 0x0318
#define CP_DMAD_CH8_LINK_ADDR                             CP_DMAD_BASE + 0x0324
#define CP_DMAD_CH8_LINK_NUM                              CP_DMAD_BASE + 0x0328
#define CP_DMAD_CH8_INTR_EN                               CP_DMAD_BASE + 0x032C
#define CP_DMAD_CH8_INTR_STATUS                           CP_DMAD_BASE + 0x0330
#define CP_DMAD_CH8_INTR_RAW                              CP_DMAD_BASE + 0x0334
#define CP_DMAD_CH8_MONITOR_CTRL                          CP_DMAD_BASE + 0x0338
#define CP_DMAD_CH8_MONITOR_OUT                           CP_DMAD_BASE + 0x033C
/*Í¨µÀ9¼Ä´æÆ÷*/
#define CP_DMAD_CH9_CTRL                                  CP_DMAD_BASE + 0x0340
#define CP_DMAD_CH9_CONFIG                                CP_DMAD_BASE + 0x0344
#define CP_DMAD_CH9_SRC_ADDR                              CP_DMAD_BASE + 0x0348
#define CP_DMAD_CH9_DST_ADDR                              CP_DMAD_BASE + 0x0350
#define CP_DMAD_CH9_SIZE                                  CP_DMAD_BASE + 0x0358
#define CP_DMAD_CH9_LINK_ADDR                             CP_DMAD_BASE + 0x0364
#define CP_DMAD_CH9_LINK_NUM                              CP_DMAD_BASE + 0x0368
#define CP_DMAD_CH9_INTR_EN                               CP_DMAD_BASE + 0x036C
#define CP_DMAD_CH9_INTR_STATUS                           CP_DMAD_BASE + 0x0370
#define CP_DMAD_CH9_INTR_RAW                              CP_DMAD_BASE + 0x0374
#define CP_DMAD_CH9_MONITOR_CTRL                          CP_DMAD_BASE + 0x0378
#define CP_DMAD_CH9_MONITOR_OUT                           CP_DMAD_BASE + 0x037C
/*ÄÚ²¿RAM½Ó¿Ú*/
#define CP_DMAD_RAM_DATA                                  CP_DMAD_BASE + 0x2000

/*******************************************************************************
CP_DMAG registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define CP_DMAG_CH_STATUS                                 CP_DMAG_BASE + 0x0000
#define CP_DMAG_CH_PRIOR0                                 CP_DMAG_BASE + 0x0020
#define CP_DMAG_CH_PRIOR1                                 CP_DMAG_BASE + 0x0024
#define CP_DMAG_CH_PRIOR2                                 CP_DMAG_BASE + 0x0028
#define CP_DMAG_CH_PRIOR3                                 CP_DMAG_BASE + 0x002C
#define CP_DMAG_CH_INTR_EN0                               CP_DMAG_BASE + 0x0040
#define CP_DMAG_CH_INTR_MASK0                             CP_DMAG_BASE + 0x0044
#define CP_DMAG_CH_INTR_STATUS0                           CP_DMAG_BASE + 0x0048
#define CP_DMAG_CH_INTR_EN1                               CP_DMAG_BASE + 0x0050
#define CP_DMAG_CH_INTR_MASK1                             CP_DMAG_BASE + 0x0054
#define CP_DMAG_CH_INTR_STATUS1                           CP_DMAG_BASE + 0x0058
#define CP_DMAG_CH_INTR_EN2                               CP_DMAG_BASE + 0x0060
#define CP_DMAG_CH_INTR_MASK2                             CP_DMAG_BASE + 0x0064
#define CP_DMAG_CH_INTR_STATUS2                           CP_DMAG_BASE + 0x0068
#define CP_DMAG_CH_LP_EN0                                 CP_DMAG_BASE + 0x0080
#define CP_DMAG_CH_BUS_LP_EN                              CP_DMAG_BASE + 0x0088
#define CP_DMAG_RAM_ADDR_BOUNDARY0                        CP_DMAG_BASE + 0x00A0
#define CP_DMAG_RAM_ADDR_BOUNDARY1                        CP_DMAG_BASE + 0x00A4
#define CP_DMAG_RAM_ADDR_BOUNDARY2                        CP_DMAG_BASE + 0x00A8
#define CP_DMAG_RAM_ADDR_BOUNDARY3                        CP_DMAG_BASE + 0x00AC
#define CP_DMAG_RAM_ADDR_BOUNDARY4                        CP_DMAG_BASE + 0x00B0
#define CP_DMAG_RAM_ADDR_BOUNDARY5                        CP_DMAG_BASE + 0x00B4
#define CP_DMAG_RAM_ADDR_BOUNDARY6                        CP_DMAG_BASE + 0x00B8
#define CP_DMAG_RAM_ADDR_BOUNDARY7                        CP_DMAG_BASE + 0x00BC
#define CP_DMAG_RAM_ADDR_BOUNDARY8                        CP_DMAG_BASE + 0x00C0
#define CP_DMAG_RAM_ADDR_BOUNDARY9                        CP_DMAG_BASE + 0x00C4
#define CP_DMAG_RAM_ADDR_BOUNDARY10                       CP_DMAG_BASE + 0x00C8
#define CP_DMAG_RAM_ADDR_BOUNDARY11                       CP_DMAG_BASE + 0x00CC
#define CP_DMAG_RAM_ADDR_BOUNDARY12                       CP_DMAG_BASE + 0x00D0
#define CP_DMAG_RAM_ADDR_BOUNDARY13                       CP_DMAG_BASE + 0x00D4
#define CP_DMAG_RAM_ADDR_BOUNDARY14                       CP_DMAG_BASE + 0x00D8
#define CP_DMAG_RAM_ADDR_BOUNDARY15                       CP_DMAG_BASE + 0x00DC
/*Í¨µÀ0¼Ä´æÆ÷*/
#define CP_DMAG_CH0_CTRL                                  CP_DMAG_BASE + 0x0100
#define CP_DMAG_CH0_CONFIG                                CP_DMAG_BASE + 0x0104
#define CP_DMAG_CH0_SRC_ADDR                              CP_DMAG_BASE + 0x0108
#define CP_DMAG_CH0_DST_ADDR                              CP_DMAG_BASE + 0x0110
#define CP_DMAG_CH0_SIZE                                  CP_DMAG_BASE + 0x0118
#define CP_DMAG_CH0_LINK_ADDR                             CP_DMAG_BASE + 0x0124
#define CP_DMAG_CH0_LINK_NUM                              CP_DMAG_BASE + 0x0128
#define CP_DMAG_CH0_INTR_EN                               CP_DMAG_BASE + 0x012C
#define CP_DMAG_CH0_INTR_STATUS                           CP_DMAG_BASE + 0x0130
#define CP_DMAG_CH0_INTR_RAW                              CP_DMAG_BASE + 0x0134
#define CP_DMAG_CH0_MONITOR_CTRL                          CP_DMAG_BASE + 0x0138
#define CP_DMAG_CH0_MONITOR_OUT                           CP_DMAG_BASE + 0x013C
/*Í¨µÀ1¼Ä´æÆ÷*/
#define CP_DMAG_CH1_CTRL                                  CP_DMAG_BASE + 0x0140
#define CP_DMAG_CH1_CONFIG                                CP_DMAG_BASE + 0x0144
#define CP_DMAG_CH1_SRC_ADDR                              CP_DMAG_BASE + 0x0148
#define CP_DMAG_CH1_DST_ADDR                              CP_DMAG_BASE + 0x0150
#define CP_DMAG_CH1_SIZE                                  CP_DMAG_BASE + 0x0158
#define CP_DMAG_CH1_LINK_ADDR                             CP_DMAG_BASE + 0x0164
#define CP_DMAG_CH1_LINK_NUM                              CP_DMAG_BASE + 0x0168
#define CP_DMAG_CH1_INTR_EN                               CP_DMAG_BASE + 0x016C
#define CP_DMAG_CH1_INTR_STATUS                           CP_DMAG_BASE + 0x0170
#define CP_DMAG_CH1_INTR_RAW                              CP_DMAG_BASE + 0x0174
#define CP_DMAG_CH1_MONITOR_CTRL                          CP_DMAG_BASE + 0x0178
#define CP_DMAG_CH1_MONITOR_OUT                           CP_DMAG_BASE + 0x017C
/*Í¨µÀ2¼Ä´æÆ÷*/
#define CP_DMAG_CH2_CTRL                                  CP_DMAG_BASE + 0x0180
#define CP_DMAG_CH2_CONFIG                                CP_DMAG_BASE + 0x0184
#define CP_DMAG_CH2_SRC_ADDR                              CP_DMAG_BASE + 0x0188
#define CP_DMAG_CH2_DST_ADDR                              CP_DMAG_BASE + 0x0190
#define CP_DMAG_CH2_SIZE                                  CP_DMAG_BASE + 0x0198
#define CP_DMAG_CH2_LINK_ADDR                             CP_DMAG_BASE + 0x01A4
#define CP_DMAG_CH2_LINK_NUM                              CP_DMAG_BASE + 0x01A8
#define CP_DMAG_CH2_INTR_EN                               CP_DMAG_BASE + 0x01AC
#define CP_DMAG_CH2_INTR_STATUS                           CP_DMAG_BASE + 0x01B0
#define CP_DMAG_CH2_INTR_RAW                              CP_DMAG_BASE + 0x01B4
#define CP_DMAG_CH2_MONITOR_CTRL                          CP_DMAG_BASE + 0x01B8
#define CP_DMAG_CH2_MONITOR_OUT                           CP_DMAG_BASE + 0x01BC
/*Í¨µÀ3¼Ä´æÆ÷*/
#define CP_DMAG_CH3_CTRL                                  CP_DMAG_BASE + 0x01C0
#define CP_DMAG_CH3_CONFIG                                CP_DMAG_BASE + 0x01C4
#define CP_DMAG_CH3_SRC_ADDR                              CP_DMAG_BASE + 0x01C8
#define CP_DMAG_CH3_DST_ADDR                              CP_DMAG_BASE + 0x01D0
#define CP_DMAG_CH3_SIZE                                  CP_DMAG_BASE + 0x01D8
#define CP_DMAG_CH3_LINK_ADDR                             CP_DMAG_BASE + 0x01E4
#define CP_DMAG_CH3_LINK_NUM                              CP_DMAG_BASE + 0x01E8
#define CP_DMAG_CH3_INTR_EN                               CP_DMAG_BASE + 0x01EC
#define CP_DMAG_CH3_INTR_STATUS                           CP_DMAG_BASE + 0x01F0
#define CP_DMAG_CH3_INTR_RAW                              CP_DMAG_BASE + 0x01F4
#define CP_DMAG_CH3_MONITOR_CTRL                          CP_DMAG_BASE + 0x01F8
#define CP_DMAG_CH3_MONITOR_OUT                           CP_DMAG_BASE + 0x01FC
/*Í¨µÀ4¼Ä´æÆ÷*/
#define CP_DMAG_CH4_CTRL                                  CP_DMAG_BASE + 0x0200
#define CP_DMAG_CH4_CONFIG                                CP_DMAG_BASE + 0x0204
#define CP_DMAG_CH4_SRC_ADDR                              CP_DMAG_BASE + 0x0208
#define CP_DMAG_CH4_DST_ADDR                              CP_DMAG_BASE + 0x0210
#define CP_DMAG_CH4_SIZE                                  CP_DMAG_BASE + 0x0218
#define CP_DMAG_CH4_LINK_ADDR                             CP_DMAG_BASE + 0x0224
#define CP_DMAG_CH4_LINK_NUM                              CP_DMAG_BASE + 0x0228
#define CP_DMAG_CH4_INTR_EN                               CP_DMAG_BASE + 0x022C
#define CP_DMAG_CH4_INTR_STATUS                           CP_DMAG_BASE + 0x0230
#define CP_DMAG_CH4_INTR_RAW                              CP_DMAG_BASE + 0x0234
#define CP_DMAG_CH4_MONITOR_CTRL                          CP_DMAG_BASE + 0x0238
#define CP_DMAG_CH4_MONITOR_OUT                           CP_DMAG_BASE + 0x023C
/*Í¨µÀ5¼Ä´æÆ÷*/
#define CP_DMAG_CH5_CTRL                                  CP_DMAG_BASE + 0x0240
#define CP_DMAG_CH5_CONFIG                                CP_DMAG_BASE + 0x0244
#define CP_DMAG_CH5_SRC_ADDR                              CP_DMAG_BASE + 0x0248
#define CP_DMAG_CH5_DST_ADDR                              CP_DMAG_BASE + 0x0250
#define CP_DMAG_CH5_SIZE                                  CP_DMAG_BASE + 0x0258
#define CP_DMAG_CH5_LINK_ADDR                             CP_DMAG_BASE + 0x0264
#define CP_DMAG_CH5_LINK_NUM                              CP_DMAG_BASE + 0x0268
#define CP_DMAG_CH5_INTR_EN                               CP_DMAG_BASE + 0x026C
#define CP_DMAG_CH5_INTR_STATUS                           CP_DMAG_BASE + 0x0270
#define CP_DMAG_CH5_INTR_RAW                              CP_DMAG_BASE + 0x0274
#define CP_DMAG_CH5_MONITOR_CTRL                          CP_DMAG_BASE + 0x0278
#define CP_DMAG_CH5_MONITOR_OUT                           CP_DMAG_BASE + 0x027C
/*Í¨µÀ6¼Ä´æÆ÷*/
#define CP_DMAG_CH6_CTRL                                  CP_DMAG_BASE + 0x0280
#define CP_DMAG_CH6_CONFIG                                CP_DMAG_BASE + 0x0284
#define CP_DMAG_CH6_SRC_ADDR                              CP_DMAG_BASE + 0x0288
#define CP_DMAG_CH6_DST_ADDR                              CP_DMAG_BASE + 0x0290
#define CP_DMAG_CH6_SIZE                                  CP_DMAG_BASE + 0x0298
#define CP_DMAG_CH6_LINK_ADDR                             CP_DMAG_BASE + 0x02A4
#define CP_DMAG_CH6_LINK_NUM                              CP_DMAG_BASE + 0x02A8
#define CP_DMAG_CH6_INTR_EN                               CP_DMAG_BASE + 0x02AC
#define CP_DMAG_CH6_INTR_STATUS                           CP_DMAG_BASE + 0x02B0
#define CP_DMAG_CH6_INTR_RAW                              CP_DMAG_BASE + 0x02B4
#define CP_DMAG_CH6_MONITOR_CTRL                          CP_DMAG_BASE + 0x02B8
#define CP_DMAG_CH6_MONITOR_OUT                           CP_DMAG_BASE + 0x02BC
/*Í¨µÀ7¼Ä´æÆ÷*/
#define CP_DMAG_CH7_CTRL                                  CP_DMAG_BASE + 0x02C0
#define CP_DMAG_CH7_CONFIG                                CP_DMAG_BASE + 0x02C4
#define CP_DMAG_CH7_SRC_ADDR                              CP_DMAG_BASE + 0x02C8
#define CP_DMAG_CH7_DST_ADDR                              CP_DMAG_BASE + 0x02D0
#define CP_DMAG_CH7_SIZE                                  CP_DMAG_BASE + 0x02D8
#define CP_DMAG_CH7_LINK_ADDR                             CP_DMAG_BASE + 0x02E4
#define CP_DMAG_CH7_LINK_NUM                              CP_DMAG_BASE + 0x02E8
#define CP_DMAG_CH7_INTR_EN                               CP_DMAG_BASE + 0x02EC
#define CP_DMAG_CH7_INTR_STATUS                           CP_DMAG_BASE + 0x02F0
#define CP_DMAG_CH7_INTR_RAW                              CP_DMAG_BASE + 0x02F4
#define CP_DMAG_CH7_MONITOR_CTRL                          CP_DMAG_BASE + 0x02F8
#define CP_DMAG_CH7_MONITOR_OUT                           CP_DMAG_BASE + 0x02FC
/*Í¨µÀ8¼Ä´æÆ÷*/
#define CP_DMAG_CH8_CTRL                                  CP_DMAG_BASE + 0x0300
#define CP_DMAG_CH8_CONFIG                                CP_DMAG_BASE + 0x0304
#define CP_DMAG_CH8_SRC_ADDR                              CP_DMAG_BASE + 0x0308
#define CP_DMAG_CH8_DST_ADDR                              CP_DMAG_BASE + 0x0310
#define CP_DMAG_CH8_SIZE                                  CP_DMAG_BASE + 0x0318
#define CP_DMAG_CH8_LINK_ADDR                             CP_DMAG_BASE + 0x0324
#define CP_DMAG_CH8_LINK_NUM                              CP_DMAG_BASE + 0x0328
#define CP_DMAG_CH8_INTR_EN                               CP_DMAG_BASE + 0x032C
#define CP_DMAG_CH8_INTR_STATUS                           CP_DMAG_BASE + 0x0330
#define CP_DMAG_CH8_INTR_RAW                              CP_DMAG_BASE + 0x0334
#define CP_DMAG_CH8_MONITOR_CTRL                          CP_DMAG_BASE + 0x0338
#define CP_DMAG_CH8_MONITOR_OUT                           CP_DMAG_BASE + 0x033C
/*Í¨µÀ9¼Ä´æÆ÷*/
#define CP_DMAG_CH9_CTRL                                  CP_DMAG_BASE + 0x0340
#define CP_DMAG_CH9_CONFIG                                CP_DMAG_BASE + 0x0344
#define CP_DMAG_CH9_SRC_ADDR                              CP_DMAG_BASE + 0x0348
#define CP_DMAG_CH9_DST_ADDR                              CP_DMAG_BASE + 0x0350
#define CP_DMAG_CH9_SIZE                                  CP_DMAG_BASE + 0x0358
#define CP_DMAG_CH9_LINK_ADDR                             CP_DMAG_BASE + 0x0364
#define CP_DMAG_CH9_LINK_NUM                              CP_DMAG_BASE + 0x0368
#define CP_DMAG_CH9_INTR_EN                               CP_DMAG_BASE + 0x036C
#define CP_DMAG_CH9_INTR_STATUS                           CP_DMAG_BASE + 0x0370
#define CP_DMAG_CH9_INTR_RAW                              CP_DMAG_BASE + 0x0374
#define CP_DMAG_CH9_MONITOR_CTRL                          CP_DMAG_BASE + 0x0378
#define CP_DMAG_CH9_MONITOR_OUT                           CP_DMAG_BASE + 0x037C
/*Í¨µÀ10¼Ä´æÆ÷*/
#define CP_DMAG_CH10_CTRL                                 CP_DMAG_BASE + 0x0380
#define CP_DMAG_CH10_CONFIG                               CP_DMAG_BASE + 0x0384
#define CP_DMAG_CH10_SRC_ADDR                             CP_DMAG_BASE + 0x0388
#define CP_DMAG_CH10_DST_ADDR                             CP_DMAG_BASE + 0x0390
#define CP_DMAG_CH10_SIZE                                 CP_DMAG_BASE + 0x0398
#define CP_DMAG_CH10_LINK_ADDR                            CP_DMAG_BASE + 0x03A4
#define CP_DMAG_CH10_LINK_NUM                             CP_DMAG_BASE + 0x03A8
#define CP_DMAG_CH10_INTR_EN                              CP_DMAG_BASE + 0x03AC
#define CP_DMAG_CH10_INTR_STATUS                          CP_DMAG_BASE + 0x03B0
#define CP_DMAG_CH10_INTR_RAW                             CP_DMAG_BASE + 0x03B4
#define CP_DMAG_CH10_MONITOR_CTRL                         CP_DMAG_BASE + 0x03B8
#define CP_DMAG_CH10_MONITOR_OUT                          CP_DMAG_BASE + 0x03BC
/*Í¨µÀ11¼Ä´æÆ÷*/
#define CP_DMAG_CH11_CTRL                                 CP_DMAG_BASE + 0x03C0
#define CP_DMAG_CH11_CONFIG                               CP_DMAG_BASE + 0x03C4
#define CP_DMAG_CH11_SRC_ADDR                             CP_DMAG_BASE + 0x03C8
#define CP_DMAG_CH11_DST_ADDR                             CP_DMAG_BASE + 0x03D0
#define CP_DMAG_CH11_SIZE                                 CP_DMAG_BASE + 0x03D8
#define CP_DMAG_CH11_LINK_ADDR                            CP_DMAG_BASE + 0x03E4
#define CP_DMAG_CH11_LINK_NUM                             CP_DMAG_BASE + 0x03E8
#define CP_DMAG_CH11_INTR_EN                              CP_DMAG_BASE + 0x03EC
#define CP_DMAG_CH11_INTR_STATUS                          CP_DMAG_BASE + 0x03F0
#define CP_DMAG_CH11_INTR_RAW                             CP_DMAG_BASE + 0x03F4
#define CP_DMAG_CH11_MONITOR_CTRL                         CP_DMAG_BASE + 0x03F8
#define CP_DMAG_CH11_MONITOR_OUT                          CP_DMAG_BASE + 0x03FC
/*Í¨µÀ12¼Ä´æÆ÷*/
#define CP_DMAG_CH12_CTRL                                 CP_DMAG_BASE + 0x0400
#define CP_DMAG_CH12_CONFIG                               CP_DMAG_BASE + 0x0404
#define CP_DMAG_CH12_SRC_ADDR                             CP_DMAG_BASE + 0x0408
#define CP_DMAG_CH12_DST_ADDR                             CP_DMAG_BASE + 0x0410
#define CP_DMAG_CH12_SIZE                                 CP_DMAG_BASE + 0x0418
#define CP_DMAG_CH12_LINK_ADDR                            CP_DMAG_BASE + 0x0424
#define CP_DMAG_CH12_LINK_NUM                             CP_DMAG_BASE + 0x0428
#define CP_DMAG_CH12_INTR_EN                              CP_DMAG_BASE + 0x042C
#define CP_DMAG_CH12_INTR_STATUS                          CP_DMAG_BASE + 0x0430
#define CP_DMAG_CH12_INTR_RAW                             CP_DMAG_BASE + 0x0434
#define CP_DMAG_CH12_MONITOR_CTRL                         CP_DMAG_BASE + 0x0438
#define CP_DMAG_CH12_MONITOR_OUT                          CP_DMAG_BASE + 0x043C
/*Í¨µÀ13¼Ä´æÆ÷*/
#define CP_DMAG_CH13_CTRL                                 CP_DMAG_BASE + 0x0440
#define CP_DMAG_CH13_CONFIG                               CP_DMAG_BASE + 0x0444
#define CP_DMAG_CH13_SRC_ADDR                             CP_DMAG_BASE + 0x0448
#define CP_DMAG_CH13_DST_ADDR                             CP_DMAG_BASE + 0x0450
#define CP_DMAG_CH13_SIZE                                 CP_DMAG_BASE + 0x0458
#define CP_DMAG_CH13_LINK_ADDR                            CP_DMAG_BASE + 0x0464
#define CP_DMAG_CH13_LINK_NUM                             CP_DMAG_BASE + 0x0468
#define CP_DMAG_CH13_INTR_EN                              CP_DMAG_BASE + 0x046C
#define CP_DMAG_CH13_INTR_STATUS                          CP_DMAG_BASE + 0x0470
#define CP_DMAG_CH13_INTR_RAW                             CP_DMAG_BASE + 0x0474
#define CP_DMAG_CH13_MONITOR_CTRL                         CP_DMAG_BASE + 0x0478
#define CP_DMAG_CH13_MONITOR_OUT                          CP_DMAG_BASE + 0x047C
/*Í¨µÀ14¼Ä´æÆ÷*/
#define CP_DMAG_CH14_CTRL                                 CP_DMAG_BASE + 0x0480
#define CP_DMAG_CH14_CONFIG                               CP_DMAG_BASE + 0x0484
#define CP_DMAG_CH14_SRC_ADDR                             CP_DMAG_BASE + 0x0488
#define CP_DMAG_CH14_DST_ADDR                             CP_DMAG_BASE + 0x0490
#define CP_DMAG_CH14_SIZE                                 CP_DMAG_BASE + 0x0498
#define CP_DMAG_CH14_LINK_ADDR                            CP_DMAG_BASE + 0x04A4
#define CP_DMAG_CH14_LINK_NUM                             CP_DMAG_BASE + 0x04A8
#define CP_DMAG_CH14_INTR_EN                              CP_DMAG_BASE + 0x04AC
#define CP_DMAG_CH14_INTR_STATUS                          CP_DMAG_BASE + 0x04B0
#define CP_DMAG_CH14_INTR_RAW                             CP_DMAG_BASE + 0x04B4
#define CP_DMAG_CH14_MONITOR_CTRL                         CP_DMAG_BASE + 0x04B8
#define CP_DMAG_CH14_MONITOR_OUT                          CP_DMAG_BASE + 0x04BC
/*Í¨µÀ15¼Ä´æÆ÷*/
#define CP_DMAG_CH15_CTRL                                 CP_DMAG_BASE + 0x04C0
#define CP_DMAG_CH15_CONFIG                               CP_DMAG_BASE + 0x04C4
#define CP_DMAG_CH15_SRC_ADDR                             CP_DMAG_BASE + 0x04C8
#define CP_DMAG_CH15_DST_ADDR                             CP_DMAG_BASE + 0x04D0
#define CP_DMAG_CH15_SIZE                                 CP_DMAG_BASE + 0x04D8
#define CP_DMAG_CH15_LINK_ADDR                            CP_DMAG_BASE + 0x04E4
#define CP_DMAG_CH15_LINK_NUM                             CP_DMAG_BASE + 0x04E8
#define CP_DMAG_CH15_INTR_EN                              CP_DMAG_BASE + 0x04EC
#define CP_DMAG_CH15_INTR_STATUS                          CP_DMAG_BASE + 0x04F0
#define CP_DMAG_CH15_INTR_RAW                             CP_DMAG_BASE + 0x04F4
#define CP_DMAG_CH15_MONITOR_CTRL                         CP_DMAG_BASE + 0x04F8
#define CP_DMAG_CH15_MONITOR_OUT                          CP_DMAG_BASE + 0x04FC
/*ÄÚ²¿RAM½Ó¿Ú*/
#define CP_DMAG_RAM_DATA                                  CP_DMAG_BASE + 0x8000

/*******************************************************************************
IPHWA registers' address on LC1860
********************************************************************************/
#define IPHWA_PPPACC_UP_CTRL                              IPHWA_BASE + 0x0000
#define IPHWA_PPPACC_UP_ACCM                              IPHWA_BASE + 0x0004
#define IPHWA_PPPACC_UP_FCS                               IPHWA_BASE + 0x0008
#define IPHWA_PPPACC_UP_STATUS                            IPHWA_BASE + 0x000C
#define IPHWA_PPPACC_UP_TRIPLE                            IPHWA_BASE + 0x0010
#define IPHWA_PPPACC_UP_PARA_OUT                          IPHWA_BASE + 0x0020
#define IPHWA_PPPACC_DOWN_CTRL                            IPHWA_BASE + 0x0400
#define IPHWA_PPPACC_DOWN_ACCM                            IPHWA_BASE + 0x0404
#define IPHWA_PPPACC_DOWN_PARA_OUT                        IPHWA_BASE + 0x0460
#define IPHWA_PPPACC_INTR_EN                              IPHWA_BASE + 0x0500
#define IPHWA_PPPACC_INTR_STATUS                          IPHWA_BASE + 0x0504
#define IPHWA_PPPACC_INTR_RAW                             IPHWA_BASE + 0x0508
#define IPHWA_PPPACC_LP_EN                                IPHWA_BASE + 0x0510
/*¼Ä´æÆ÷Ãû£º0x600*/
#define IPHWA_INTR_EN                                     IPHWA_BASE + 0x0600
#define IPHWA_INTR_MASK                                   IPHWA_BASE + 0x0604
#define IPHWA_INTR_STATUS                                 IPHWA_BASE + 0x0608
/*¼Ä´æÆ÷Ãû:0x4000*/
/*Í¨ÓÃ¼Ä´æÆ÷*/
#define IPHWA_DCH_STATUS                                  IPHWA_BASE + 0x4000
#define IPHWA_DCH_PRIOR0                                  IPHWA_BASE + 0x4020
#define IPHWA_DCH_PRIOR4                                  IPHWA_BASE + 0x4030
#define IPHWA_DCH_INTR_EN0                                IPHWA_BASE + 0x4040
#define IPHWA_DCH_INTR_MASK0                              IPHWA_BASE + 0x4044
#define IPHWA_DCH_INTR_STATUS0                            IPHWA_BASE + 0x4048
#define IPHWA_DCH_INTR_EN1                                IPHWA_BASE + 0x4050
#define IPHWA_DCH_INTR_MASK1                              IPHWA_BASE + 0x4054
#define IPHWA_DCH_INTR_STATUS1                            IPHWA_BASE + 0x4058
#define IPHWA_DCH_LP_EN0                                  IPHWA_BASE + 0x4080
#define IPHWA_DCH_LP_EN1                                  IPHWA_BASE + 0x4084
#define IPHWA_DCH_BUS_LP_EN                               IPHWA_BASE + 0x4088
#define IPHWA_DCH_RAM_ADDR_BOUNDARY1                      IPHWA_BASE + 0x40A4
#define IPHWA_DCH_RAM_ADDR_BOUNDARY2                      IPHWA_BASE + 0x40A8
#define IPHWA_DCH_RAM_ADDR_BOUNDARY3                      IPHWA_BASE + 0x40AC
#define IPHWA_DCH_RAM_ADDR_BOUNDARY17                     IPHWA_BASE + 0x40E4
/*Í¨µÀ0¼Ä´æÆ÷*/
#define IPHWA_DCH0_CTRL                                   IPHWA_BASE + 0x4100
#define IPHWA_DCH0_CONFIG                                 IPHWA_BASE + 0x4104
#define IPHWA_DCH0_SRC_ADDR                               IPHWA_BASE + 0x4108
#define IPHWA_DCH0_DST_ADDR                               IPHWA_BASE + 0x4110
#define IPHWA_DCH0_SIZE                                   IPHWA_BASE + 0x4118
#define IPHWA_DCH0_INTR_EN                                IPHWA_BASE + 0x412C
#define IPHWA_DCH0_INTR_STATUS                            IPHWA_BASE + 0x4130
#define IPHWA_DCH0_INTR_RAW                               IPHWA_BASE + 0x4134
#define IPHWA_DCH0_MONITOR_CTRL                           IPHWA_BASE + 0x4138
#define IPHWA_DCH0_MONITOR_OUT                            IPHWA_BASE + 0x413C
/*Í¨µÀ1¼Ä´æÆ÷*/
#define IPHWA_DCH1_CTRL                                   IPHWA_BASE + 0x4140
#define IPHWA_DCH1_CONFIG                                 IPHWA_BASE + 0x4144
#define IPHWA_DCH1_SRC_ADDR                               IPHWA_BASE + 0x4148
#define IPHWA_DCH1_DST_ADDR                               IPHWA_BASE + 0x4110
#define IPHWA_DCH1_SIZE                                   IPHWA_BASE + 0x4158
#define IPHWA_DCH1_LINK_ADDR                              IPHWA_BASE + 0x4164
#define IPHWA_DCH1_LINK_NUM                               IPHWA_BASE + 0x4168
#define IPHWA_DCH1_INTR_EN                                IPHWA_BASE + 0x416C
#define IPHWA_DCH1_INTR_STATUS                            IPHWA_BASE + 0x4170
#define IPHWA_DCH1_INTR_RAW                               IPHWA_BASE + 0x4174
#define IPHWA_DCH1_MONITOR_CTRL                           IPHWA_BASE + 0x4178
#define IPHWA_DCH1_MONITOR_OUT                            IPHWA_BASE + 0x417C
/*Í¨µÀ2¼Ä´æÆ÷*/
#define IPHWA_DCH2_CTRL                                   IPHWA_BASE + 0x4180
#define IPHWA_DCH2_CONFIG                                 IPHWA_BASE + 0x4184
#define IPHWA_DCH2_SRC_ADDR                               IPHWA_BASE + 0x4188
#define IPHWA_DCH2_SRC_OFFSET                             IPHWA_BASE + 0x418C
#define IPHWA_DCH2_DST_ADDR                               IPHWA_BASE + 0x4190
#define IPHWA_DCH2_DST_OFFSET                             IPHWA_BASE + 0x4194
#define IPHWA_DCH2_SIZE                                   IPHWA_BASE + 0x4198
#define IPHWA_DCH2_Y_SIZE                                 IPHWA_BASE + 0x419C
#define IPHWA_DCH2_SPACE                                  IPHWA_BASE + 0x41A0
#define IPHWA_DCH2_LINK_ADDR                              IPHWA_BASE + 0x41A4
#define IPHWA_DCH2_LINK_NUM                               IPHWA_BASE + 0x41A8
#define IPHWA_DCH2_INTR_EN                                IPHWA_BASE + 0x41AC
#define IPHWA_DCH2_INTR_STATUS                            IPHWA_BASE + 0x41B0
#define IPHWA_DCH2_INTR_RAW                               IPHWA_BASE + 0x41B4
#define IPHWA_DCH2_MONITOR_CTRL                           IPHWA_BASE + 0x41B8
#define IPHWA_DCH2_MONITOR_OUT                            IPHWA_BASE + 0x41BC
/*Í¨µÀ3¼Ä´æÆ÷*/
#define IPHWA_DCH3_CTRL                                   IPHWA_BASE + 0x41C0
#define IPHWA_DCH3_CONFIG                                 IPHWA_BASE + 0x41C4
#define IPHWA_DCH3_SRC_ADDR                               IPHWA_BASE + 0x41C8
#define IPHWA_DCH3_SRC_OFFSET                             IPHWA_BASE + 0x41CC
#define IPHWA_DCH3_DST_ADDR                               IPHWA_BASE + 0x41D0
#define IPHWA_DCH3_DST_OFFSET                             IPHWA_BASE + 0x41D4
#define IPHWA_DCH3_SIZE                                   IPHWA_BASE + 0x41D8
#define IPHWA_DCH3_LINK_ADDR                              IPHWA_BASE + 0x41E4
#define IPHWA_DCH3_LINK_NUM                               IPHWA_BASE + 0x41E8
#define IPHWA_DCH3_INTR_EN                                IPHWA_BASE + 0x41EC
#define IPHWA_DCH3_INTR_STATUS                            IPHWA_BASE + 0x41F0
#define IPHWA_DCH3_INTR_RAW                               IPHWA_BASE + 0x41F4
#define IPHWA_DCH3_MONITOR_CTRL                           IPHWA_BASE + 0x41F8
#define IPHWA_DCH3_MONITOR_OUT                            IPHWA_BASE + 0x41FC
/*Í¨µÀ16¼Ä´æÆ÷*/
#define IPHWA_DCH16_CTRL                                  IPHWA_BASE + 0x4500
#define IPHWA_DCH16_CONFIG                                IPHWA_BASE + 0x4504
#define IPHWA_DCH16_SRC_ADDR                              IPHWA_BASE + 0x4508
#define IPHWA_DCH16_DST_ADDR                              IPHWA_BASE + 0x4510
#define IPHWA_DCH16_INTR_EN                               IPHWA_BASE + 0x452C
#define IPHWA_DCH16_INTR_STATUS                           IPHWA_BASE + 0x4530
#define IPHWA_DCH16_INTR_RAW                              IPHWA_BASE + 0x4534
#define IPHWA_DCH16_MONITOR_CTRL                          IPHWA_BASE + 0x4538
#define IPHWA_DCH16_MONITOR_OUT                           IPHWA_BASE + 0x453C
/*Í¨µÀ17¼Ä´æÆ÷*/
#define IPHWA_DCH17_CTRL                                  IPHWA_BASE + 0x4540
#define IPHWA_DCH17_CONFIG                                IPHWA_BASE + 0x4544
#define IPHWA_DCH17_SRC_ADDR                              IPHWA_BASE + 0x4548
#define IPHWA_DCH17_DST_ADDR                              IPHWA_BASE + 0x4550
#define IPHWA_DCH17_SIZE                                  IPHWA_BASE + 0x4558
#define IPHWA_DCH17_LINK_ADDR                             IPHWA_BASE + 0x4564
#define IPHWA_DCH17_LINK_NUM                              IPHWA_BASE + 0x4568
#define IPHWA_DCH17_INTR_EN                               IPHWA_BASE + 0x456C
#define IPHWA_DCH17_INTR_STATUS                           IPHWA_BASE + 0x4570
#define IPHWA_DCH17_INTR_RAW                              IPHWA_BASE + 0x4574
#define IPHWA_DCH17_MONITOR_CTRL                          IPHWA_BASE + 0x4578
#define IPHWA_DCH17_MONITOR_OUT                           IPHWA_BASE + 0x457C
/*ÄÚ²¿RAM½Ó¿Ú*/
#define IPHWA_DCH_RAM_DATA                                IPHWA_BASE + 0x5000

/*******************************************************************************
HSL registers' address on LC1860
********************************************************************************/
#define HSL_EN                                            HSL_BASE + 0x0000
#define HSL_FIFO_EN                                       HSL_BASE + 0x0004
#define HSL_CH0_FIFO                                      HSL_BASE + 0x0008
#define HSL_CH1_FIFO                                      HSL_BASE + 0x000C
#define HSL_CH2_FIFO                                      HSL_BASE + 0x0010
#define HSL_CH3_FIFO                                      HSL_BASE + 0x0014
#define HSL_CH4_FIFO                                      HSL_BASE + 0x0018
#define HSL_CH5_FIFO                                      HSL_BASE + 0x001C
#define HSL_CH6_FIFO                                      HSL_BASE + 0x0020
#define HSL_CH7_FIFO                                      HSL_BASE + 0x0024
#define HSL_CH8_FIFO                                      HSL_BASE + 0x0028
#define HSL_FIFO_TIMER_EN                                 HSL_BASE + 0x002C
#define HSL_FIFO0_TIMER                                   HSL_BASE + 0x0030
#define HSL_FIFO1_TIMER                                   HSL_BASE + 0x0034
#define HSL_FIFO2_TIMER                                   HSL_BASE + 0x0038
#define HSL_FIFO3_TIMER                                   HSL_BASE + 0x003C
#define HSL_FIFO4_TIMER                                   HSL_BASE + 0x0040
#define HSL_FIFO5_TIMER                                   HSL_BASE + 0x0044
#define HSL_FIFO6_TIMER                                   HSL_BASE + 0x0048
#define HSL_FIFO_COUNTER_EN                               HSL_BASE + 0x004C
#define HSL_FIFO0_COUNTER                                 HSL_BASE + 0x0050
#define HSL_FIFO1_COUNTER                                 HSL_BASE + 0x0054
#define HSL_FIFO2_COUNTER                                 HSL_BASE + 0x0058
#define HSL_FIFO3_COUNTER                                 HSL_BASE + 0x005C
#define HSL_FIFO4_COUNTER                                 HSL_BASE + 0x0060
#define HSL_FIFO5_COUNTER                                 HSL_BASE + 0x0064
#define HSL_FIFO6_COUNTER                                 HSL_BASE + 0x0068
#define HSL_FIFO0_RMARK                                   HSL_BASE + 0x006C
#define HSL_FIFO1_RMARK                                   HSL_BASE + 0x0070
#define HSL_FIFO2_RMARK                                   HSL_BASE + 0x0074
#define HSL_FIFO3_RMARK                                   HSL_BASE + 0x0078
#define HSL_FIFO4_RMARK                                   HSL_BASE + 0x007C
#define HSL_FIFO5_RMARK                                   HSL_BASE + 0x0080
#define HSL_FIFO6_RMARK                                   HSL_BASE + 0x0084
#define HSL_FIFO0_PRIOR                                   HSL_BASE + 0x0088
#define HSL_FIFO1_PRIOR                                   HSL_BASE + 0x008C
#define HSL_FIFO2_PRIOR                                   HSL_BASE + 0x0090
#define HSL_FIFO3_PRIOR                                   HSL_BASE + 0x0094
#define HSL_FIFO4_PRIOR                                   HSL_BASE + 0x0098
#define HSL_FIFO5_PRIOR                                   HSL_BASE + 0x009C
#define HSL_FIFO6_PRIOR                                   HSL_BASE + 0x00A0
#define HSL_INTR_RAW                                      HSL_BASE + 0x00A4
#define HSL_INTR_EN                                       HSL_BASE + 0x00A8
#define HSL_INTR_STA                                      HSL_BASE + 0x00AC
#define HSL_EXT_CH                                        HSL_BASE + 0x00B0
#define HSL_STATUS                                        HSL_BASE + 0x00B4
#define HSL_GPIF_CLK_CTRL                                 HSL_BASE + 0x00B8
#define HSL_FIFOX_EMPTY                                   HSL_BASE + 0x00BC
#define HSL_SYN_WORD                                      HSL_BASE + 0x00C0
#define HSL_FIFO0_LENGTH                                  HSL_BASE + 0x00C4
#define HSL_FIFO1_LENGTH                                  HSL_BASE + 0x00C8
#define HSL_FIFO2_LENGTH                                  HSL_BASE + 0x00CC
#define HSL_FIFO3_LENGTH                                  HSL_BASE + 0x00D0
#define HSL_FIFO4_LENGTH                                  HSL_BASE + 0x00D4
#define HSL_FIFO5_LENGTH                                  HSL_BASE + 0x00D8
#define HSL_FIFO6_LENGTH                                  HSL_BASE + 0x00DC
#define HSL_A7_ACCESS                                     HSL_BASE + 0x0110

/*******************************************************************************
A5 registers' address on LC1860
********************************************************************************/
#define A5_CFG                                            A5_BASE + 0x0000
#define A5_LN                                             A5_BASE + 0x0004
#define A5_FN0                                            A5_BASE + 0x0008
#define A5_FN1                                            A5_BASE + 0x000C
#define A5_UPDATE                                         A5_BASE + 0x0010
#define A5_KEY0                                           A5_BASE + 0x0014
#define A5_KEY1                                           A5_BASE + 0x0018
#define A5_KEY2                                           A5_BASE + 0x001C
#define A5_KEY3                                           A5_BASE + 0x0020
#define A5_INTR_EN                                        A5_BASE + 0x0024
#define A5_INTR_STATUS                                    A5_BASE + 0x0028
#define A5_INTR_RAW                                       A5_BASE + 0x002C
#define A5_DMA_TRANS_EN                                   A5_BASE + 0x0030
#define A5_DMA_ADDR                                       A5_BASE + 0x0034
#define A5_DOUT                                           A5_BASE + 0x0038
#define A5_HADDR_M                                        A5_BASE + 0x003C
#define A5_LP_EN                                          A5_BASE + 0x0040

/*******************************************************************************
GEA registers' address on LC1860
********************************************************************************/
#define GEA_CFG                                           GEA_BASE + 0x0000
#define GEA_LN                                            GEA_BASE + 0x0004
#define GEA_INITVEC_HIGH                                  GEA_BASE + 0x0008
#define GEA_INITVEC_LOW                                   GEA_BASE + 0x000C
#define GEA_STATUS                                        GEA_BASE + 0x0010
#define GEA_UPDATE                                        GEA_BASE + 0x0014
#define GEA_STATUS_UP                                     GEA_BASE + 0x0018
#define GEA_KASUMI_CFG                                    GEA_BASE + 0x001C
#define GEA_KASUMI_START                                  GEA_BASE + 0x0020
#define GEA_KASUMI_IN_HIGH                                GEA_BASE + 0x0024
#define GEA_KASUMI_IN_LOW                                 GEA_BASE + 0x0028
#define GEA_KASUMI_OUT_HIGH                               GEA_BASE + 0x002C
#define GEA_KASUMI_OUT_LOW                                GEA_BASE + 0x0030
#define GEA_KEY0                                          GEA_BASE + 0x0034
#define GEA_KEY1                                          GEA_BASE + 0x0038
#define GEA_KEY2                                          GEA_BASE + 0x003C
#define GEA_KEY3                                          GEA_BASE + 0x0040
#define GEA_INTR_EN                                       GEA_BASE + 0x0044
#define GEA_INTR_STATUS                                   GEA_BASE + 0x0048
#define GEA_INTR_RAW                                      GEA_BASE + 0x004C
#define GEA_DMA_TRANS_EN                                  GEA_BASE + 0x0050
#define GEA_DMA_ADDR                                      GEA_BASE + 0x0054
#define GEA_DOUT                                          GEA_BASE + 0x0058
#define GEA_HADDR_M                                       GEA_BASE + 0x005C
#define GEA_LP_EN                                         GEA_BASE + 0x0060

/*******************************************************************************
CIPHERHWA registers' address on LC1860
********************************************************************************/
/*KDF:0*/
#define CIPHERHWA_KDF_1ST_CTRL                            CIPHERHWA_BASE + 0x0000
#define CIPHERHWA_KDF_1ST_KEY0                            CIPHERHWA_BASE + 0x0008
#define CIPHERHWA_KDF_1ST_KEY1                            CIPHERHWA_BASE + 0x000C
#define CIPHERHWA_KDF_1ST_KEY2                            CIPHERHWA_BASE + 0x0010
#define CIPHERHWA_KDF_1ST_KEY3                            CIPHERHWA_BASE + 0x0014
#define CIPHERHWA_KDF_1ST_KEY4                            CIPHERHWA_BASE + 0x0018
#define CIPHERHWA_KDF_1ST_KEY5                            CIPHERHWA_BASE + 0x001C
#define CIPHERHWA_KDF_1ST_KEY6                            CIPHERHWA_BASE + 0x0020
#define CIPHERHWA_KDF_1ST_KEY7                            CIPHERHWA_BASE + 0x0024
#define CIPHERHWA_KEY_1ST_LENGTH                          CIPHERHWA_BASE + 0x0028
#define CIPHERHWA_KDF_1ST_TXT0                            CIPHERHWA_BASE + 0x002C
#define CIPHERHWA_KDF_1ST_TXT1                            CIPHERHWA_BASE + 0x0030
#define CIPHERHWA_KDF_1ST_TXT2                            CIPHERHWA_BASE + 0x0034
#define CIPHERHWA_KDF_1ST_TXT3                            CIPHERHWA_BASE + 0x0038
#define CIPHERHWA_KDF_1ST_TXT4                            CIPHERHWA_BASE + 0x003C
#define CIPHERHWA_KDF_1ST_TXT5                            CIPHERHWA_BASE + 0x0040
#define CIPHERHWA_KDF_1ST_TXT6                            CIPHERHWA_BASE + 0x0044
#define CIPHERHWA_KDF_1ST_TXT7                            CIPHERHWA_BASE + 0x0048
#define CIPHERHWA_KDF_1ST_TXT8                            CIPHERHWA_BASE + 0x004C
#define CIPHERHWA_KDF_1ST_TXT9                            CIPHERHWA_BASE + 0x0050
#define CIPHERHWA_KDF_1ST_TXT10                           CIPHERHWA_BASE + 0x0054
#define CIPHERHWA_KDF_1ST_TXT11                           CIPHERHWA_BASE + 0x0058
#define CIPHERHWA_KDF_1ST_TXT12                           CIPHERHWA_BASE + 0x005C
#define CIPHERHWA_KDF_1ST_TXT13                           CIPHERHWA_BASE + 0x0060
#define CIPHERHWA_KDF_1ST_TXT14                           CIPHERHWA_BASE + 0x0064
#define CIPHERHWA_KDF_1ST_TXT15                           CIPHERHWA_BASE + 0x0068
#define CIPHERHWA_KDF_2ND_CTRL                            CIPHERHWA_BASE + 0x006C
#define CIPHERHWA_KDF_2ND_KEY0                            CIPHERHWA_BASE + 0x0074
#define CIPHERHWA_KDF_2ND_KEY1                            CIPHERHWA_BASE + 0x0078
#define CIPHERHWA_KDF_2ND_KEY2                            CIPHERHWA_BASE + 0x007C
#define CIPHERHWA_KDF_2ND_KEY3                            CIPHERHWA_BASE + 0x0080
#define CIPHERHWA_KDF_2ND_KEY4                            CIPHERHWA_BASE + 0x0084
#define CIPHERHWA_KDF_2ND_KEY5                            CIPHERHWA_BASE + 0x0088
#define CIPHERHWA_KDF_2ND_KEY6                            CIPHERHWA_BASE + 0x008C
#define CIPHERHWA_KDF_2ND_KEY7                            CIPHERHWA_BASE + 0x0090
#define CIPHERHWA_KEY_2ND_LENGTH                          CIPHERHWA_BASE + 0x0094
#define CIPHERHWA_KDF_2ND_TXT0                            CIPHERHWA_BASE + 0x0098
#define CIPHERHWA_KDF_2ND_TXT1                            CIPHERHWA_BASE + 0x009C
#define CIPHERHWA_KDF_2ND_TXT2                            CIPHERHWA_BASE + 0x00A0
#define CIPHERHWA_KDF_2ND_TXT3                            CIPHERHWA_BASE + 0x00A4
#define CIPHERHWA_KDF_2ND_TXT4                            CIPHERHWA_BASE + 0x00A8
#define CIPHERHWA_KDF_2ND_TXT5                            CIPHERHWA_BASE + 0x00AC
#define CIPHERHWA_KDF_2ND_TXT6                            CIPHERHWA_BASE + 0x00B0
#define CIPHERHWA_KDF_2ND_TXT7                            CIPHERHWA_BASE + 0x00B4
#define CIPHERHWA_KDF_2ND_TXT8                            CIPHERHWA_BASE + 0x00B8
#define CIPHERHWA_KDF_2ND_TXT9                            CIPHERHWA_BASE + 0x00BC
#define CIPHERHWA_KDF_2ND_TXT10                           CIPHERHWA_BASE + 0x00C0
#define CIPHERHWA_KDF_2ND_TXT11                           CIPHERHWA_BASE + 0x00C4
#define CIPHERHWA_KDF_2ND_TXT12                           CIPHERHWA_BASE + 0x00C8
#define CIPHERHWA_KDF_2ND_TXT13                           CIPHERHWA_BASE + 0x00CC
#define CIPHERHWA_KDF_2ND_TXT14                           CIPHERHWA_BASE + 0x00D0
#define CIPHERHWA_KDF_2ND_TXT15                           CIPHERHWA_BASE + 0x00D4
#define CIPHERHWA_KDF_1ST_KD0                             CIPHERHWA_BASE + 0x00D8
#define CIPHERHWA_KDF_1ST_KD1                             CIPHERHWA_BASE + 0x00DC
#define CIPHERHWA_KDF_1ST_KD2                             CIPHERHWA_BASE + 0x00E0
#define CIPHERHWA_KDF_1ST_KD3                             CIPHERHWA_BASE + 0x00E4
#define CIPHERHWA_KDF_1ST_KD4                             CIPHERHWA_BASE + 0x00E8
#define CIPHERHWA_KDF_1ST_KD5                             CIPHERHWA_BASE + 0x00EC
#define CIPHERHWA_KDF_1ST_KD6                             CIPHERHWA_BASE + 0x00F0
#define CIPHERHWA_KDF_1ST_KD7                             CIPHERHWA_BASE + 0x00F4
#define CIPHERHWA_KDF_2ND_KD0                             CIPHERHWA_BASE + 0x00F8
#define CIPHERHWA_KDF_2ND_KD1                             CIPHERHWA_BASE + 0x00FC
#define CIPHERHWA_KDF_2ND_KD2                             CIPHERHWA_BASE + 0x0100
#define CIPHERHWA_KDF_2ND_KD3                             CIPHERHWA_BASE + 0x0104
#define CIPHERHWA_KDF_2ND_KD4                             CIPHERHWA_BASE + 0x0108
#define CIPHERHWA_KDF_2ND_KD5                             CIPHERHWA_BASE + 0x010C
#define CIPHERHWA_KDF_2ND_KD6                             CIPHERHWA_BASE + 0x0110
#define CIPHERHWA_KDF_2ND_KD7                             CIPHERHWA_BASE + 0x0114
#define CIPHERHWA_KDF_LP_EN                               CIPHERHWA_BASE + 0x0118
#define CIPHERHWA_KDF_INTR_EN                             CIPHERHWA_BASE + 0x011C
#define CIPHERHWA_KDF_INTR_STATUS                         CIPHERHWA_BASE + 0x0120
#define CIPHERHWA_KDF_INTR_RAW                            CIPHERHWA_BASE + 0x0124
/*SCRT0:0x400*/
#define CIPHERHWA_SCRT0_CTRL                              CIPHERHWA_BASE + 0x0400
#define CIPHERHWA_SCRT0_MODE_SEL                          CIPHERHWA_BASE + 0x0404
#define CIPHERHWA_SCRT0_COUNT                             CIPHERHWA_BASE + 0x0408
#define CIPHERHWA_SCRT0_BEARER                            CIPHERHWA_BASE + 0x040C
#define CIPHERHWA_SCRT0_DIRECTION                         CIPHERHWA_BASE + 0x0410
#define CIPHERHWA_SCRT0_FRESH                             CIPHERHWA_BASE + 0x0414
#define CIPHERHWA_SCRT0_LENGTH                            CIPHERHWA_BASE + 0x0418
#define CIPHERHWA_SCRT0_KUPDT                             CIPHERHWA_BASE + 0x041C
#define CIPHERHWA_SCRT0_KEY0                              CIPHERHWA_BASE + 0x0420
#define CIPHERHWA_SCRT0_KEY1                              CIPHERHWA_BASE + 0x0424
#define CIPHERHWA_SCRT0_KEY2                              CIPHERHWA_BASE + 0x0428
#define CIPHERHWA_SCRT0_KEY3                              CIPHERHWA_BASE + 0x042C
#define CIPHERHWA_SCRT0_MAC                               CIPHERHWA_BASE + 0x0430
#define CIPHERHWA_SCRT0_LP_EN                             CIPHERHWA_BASE + 0x0434
#define CIPHERHWA_SCRT0_INTR_EN                           CIPHERHWA_BASE + 0x0440
#define CIPHERHWA_SCRT0_INTR_STATUS                       CIPHERHWA_BASE + 0x0444
#define CIPHERHWA_SCRT0_INTR_RAW                          CIPHERHWA_BASE + 0x0448
/*SCRT1:0x800*/
#define CIPHERHWA_SCRT1_CTRL                              CIPHERHWA_BASE + 0x0800
#define CIPHERHWA_SCRT1_MODE_SEL                          CIPHERHWA_BASE + 0x0804
#define CIPHERHWA_SCRT1_COUNT                             CIPHERHWA_BASE + 0x0808
#define CIPHERHWA_SCRT1_BEARER                            CIPHERHWA_BASE + 0x080C
#define CIPHERHWA_SCRT1_DIRECTION                         CIPHERHWA_BASE + 0x0810
#define CIPHERHWA_SCRT1_FRESH                             CIPHERHWA_BASE + 0x0814
#define CIPHERHWA_SCRT1_LENGTH                            CIPHERHWA_BASE + 0x0818
#define CIPHERHWA_SCRT1_KUPDT                             CIPHERHWA_BASE + 0x081C
#define CIPHERHWA_SCRT1_KEY0                              CIPHERHWA_BASE + 0x0820
#define CIPHERHWA_SCRT1_KEY1                              CIPHERHWA_BASE + 0x0824
#define CIPHERHWA_SCRT1_KEY2                              CIPHERHWA_BASE + 0x0828
#define CIPHERHWA_SCRT1_KEY3                              CIPHERHWA_BASE + 0x082C
#define CIPHERHWA_SCRT1_MAC                               CIPHERHWA_BASE + 0x0830
#define CIPHERHWA_SCRT1_LP_EN                             CIPHERHWA_BASE + 0x0834
#define CIPHERHWA_SCRT1_INTR_EN                           CIPHERHWA_BASE + 0x0840
#define CIPHERHWA_SCRT1_INTR_STATUS                       CIPHERHWA_BASE + 0x0844
#define CIPHERHWA_SCRT1_INTR_RAW                          CIPHERHWA_BASE + 0x0848
/*DCH:0x4000*/
#define CIPHERHWA_CH_STATUS                               CIPHERHWA_BASE + 0x4000
#define CIPHERHWA_CH_PRIOR0                               CIPHERHWA_BASE + 0x4020
#define CIPHERHWA_CH_PRIOR3                               CIPHERHWA_BASE + 0x4030
#define CIPHERHWA_CH_INTR_EN0                             CIPHERHWA_BASE + 0x4040
#define CIPHERHWA_CH_INTR_MASK0                           CIPHERHWA_BASE + 0x4044
#define CIPHERHWA_CH_INTR_STATUS0                         CIPHERHWA_BASE + 0x4048
#define CIPHERHWA_CH_LP_EN0                               CIPHERHWA_BASE + 0x4080
#define CIPHERHWA_CH_LP_EN1                               CIPHERHWA_BASE + 0x4084
#define CIPHERHWA_CH_BUS_LP_EN                            CIPHERHWA_BASE + 0x4088
#define CIPHERHWA_RAM_ADDR_BOUNDARY0                      CIPHERHWA_BASE + 0x40A0
#define CIPHERHWA_RAM_ADDR_BOUNDARY1                      CIPHERHWA_BASE + 0x40A4
#define CIPHERHWA_RAM_ADDR_BOUNDARY16                     CIPHERHWA_BASE + 0x40E0
#define CIPHERHWA_RAM_ADDR_BOUNDARY17                     CIPHERHWA_BASE + 0x40E4
#define CIPHERHWA_CH0_CTRL                                CIPHERHWA_BASE + 0x4100
#define CIPHERHWA_CH0_CONFIG                              CIPHERHWA_BASE + 0x4104
#define CIPHERHWA_CH0_SRC_ADDR                            CIPHERHWA_BASE + 0x4108
#define CIPHERHWA_CH0_DST_ADDR                            CIPHERHWA_BASE + 0x4110
#define CIPHERHWA_CH0_SIZE                                CIPHERHWA_BASE + 0x4118
#define CIPHERHWA_CH0_LINK_ADDR                           CIPHERHWA_BASE + 0x4124
#define CIPHERHWA_CH0_LINK_NUM                            CIPHERHWA_BASE + 0x4128
#define CIPHERHWA_CH0_INTR_EN                             CIPHERHWA_BASE + 0x412C
#define CIPHERHWA_CH0_INTR_STATUS                         CIPHERHWA_BASE + 0x4130
#define CIPHERHWA_CH0_INTR_RAW                            CIPHERHWA_BASE + 0x4134
#define CIPHERHWA_CH0_MONITOR_CTRL                        CIPHERHWA_BASE + 0x4138
#define CIPHERHWA_CH0_MONITOR_OUT                         CIPHERHWA_BASE + 0x413C
#define CIPHERHWA_CH1_CTRL                                CIPHERHWA_BASE + 0x4140
#define CIPHERHWA_CH1_CONFIG                              CIPHERHWA_BASE + 0x4144
#define CIPHERHWA_CH1_SRC_ADDR                            CIPHERHWA_BASE + 0x4148
#define CIPHERHWA_CH1_DST_ADDR                            CIPHERHWA_BASE + 0x4150
#define CIPHERHWA_CH1_SIZE                                CIPHERHWA_BASE + 0x4158
#define CIPHERHWA_CH1_LINK_ADDR                           CIPHERHWA_BASE + 0x4164
#define CIPHERHWA_CH1_LINK_NUM                            CIPHERHWA_BASE + 0x4168
#define CIPHERHWA_CH1_INTR_EN                             CIPHERHWA_BASE + 0x416C
#define CIPHERHWA_CH1_INTR_STATUS                         CIPHERHWA_BASE + 0x4170
#define CIPHERHWA_CH1_INTR_RAW                            CIPHERHWA_BASE + 0x4174
#define CIPHERHWA_CH1_MONITOR_CTRL                        CIPHERHWA_BASE + 0x4178
#define CIPHERHWA_CH1_MONITOR_OUT                         CIPHERHWA_BASE + 0x417C
#define CIPHERHWA_CH16_CTRL                               CIPHERHWA_BASE + 0x4500
#define CIPHERHWA_CH16_CONFIG                             CIPHERHWA_BASE + 0x4504
#define CIPHERHWA_CH16_SRC_ADDR                           CIPHERHWA_BASE + 0x4508
#define CIPHERHWA_CH16_DST_ADDR                           CIPHERHWA_BASE + 0x4510
#define CIPHERHWA_CH16_SIZE                               CIPHERHWA_BASE + 0x4518
#define CIPHERHWA_CH16_LINK_ADDR                          CIPHERHWA_BASE + 0x4524
#define CIPHERHWA_CH16_LINK_NUM                           CIPHERHWA_BASE + 0x4528
#define CIPHERHWA_CH16_INTR_EN                            CIPHERHWA_BASE + 0x452C
#define CIPHERHWA_CH16_INTR_STATUS                        CIPHERHWA_BASE + 0x4530
#define CIPHERHWA_CH16_INTR_RAW                           CIPHERHWA_BASE + 0x4534
#define CIPHERHWA_CH16_MONITOR_CTRL                       CIPHERHWA_BASE + 0x4538
#define CIPHERHWA_CH16_MONITOR_OUT                        CIPHERHWA_BASE + 0x453C
#define CIPHERHWA_CH17_CTRL                               CIPHERHWA_BASE + 0x4540
#define CIPHERHWA_CH17_CONFIG                             CIPHERHWA_BASE + 0x4544
#define CIPHERHWA_CH17_SRC_ADDR                           CIPHERHWA_BASE + 0x4548
#define CIPHERHWA_CH17_DST_ADDR                           CIPHERHWA_BASE + 0x4550
#define CIPHERHWA_CH17_SIZE                               CIPHERHWA_BASE + 0x4558
#define CIPHERHWA_CH17_LINK_ADDR                          CIPHERHWA_BASE + 0x4564
#define CIPHERHWA_CH17_LINK_NUM                           CIPHERHWA_BASE + 0x4568
#define CIPHERHWA_CH17_INTR_EN                            CIPHERHWA_BASE + 0x456C
#define CIPHERHWA_CH17_INTR_STATUS                        CIPHERHWA_BASE + 0x4570
#define CIPHERHWA_CH17_INTR_RAW                           CIPHERHWA_BASE + 0x4574
#define CIPHERHWA_CH17_MONITOR_CTRL                       CIPHERHWA_BASE + 0x4578
#define CIPHERHWA_CH17_MONITOR_OUT                        CIPHERHWA_BASE + 0x457C
#define CIPHERHWA_DCH_RAM                                 CIPHERHWA_BASE + 0x6000
/*SELF_CONFIG:0x8000*/
#define CIPHERHWA_SC0_CTRL                                CIPHERHWA_BASE + 0x8000
#define CIPHERHWA_SC0_LINK_ADDR0                          CIPHERHWA_BASE + 0x8004
#define CIPHERHWA_SC0_SIZE                                CIPHERHWA_BASE + 0x800C
#define CIPHERHWA_SC0_CUR_ADDR0                           CIPHERHWA_BASE + 0x8010
#define CIPHERHWA_SC0_RAM_BOUNDARY                        CIPHERHWA_BASE + 0x8018
#define CIPHERHWA_SC1_CTRL                                CIPHERHWA_BASE + 0x8020
#define CIPHERHWA_SC1_LINK_ADDR0                          CIPHERHWA_BASE + 0x8024
#define CIPHERHWA_SC1_SIZE                                CIPHERHWA_BASE + 0x802C
#define CIPHERHWA_SC1_CUR_ADDR0                           CIPHERHWA_BASE + 0x8030
#define CIPHERHWA_SC1_RAM_BOUNDARY                        CIPHERHWA_BASE + 0x8038
#define CIPHERHWA_SELF_CONFIG_INTR_EN                     CIPHERHWA_BASE + 0x8040
#define CIPHERHWA_SELF_CONFIG_INTR_STATUS                 CIPHERHWA_BASE + 0x8044
#define CIPHERHWA_SELF_CONFIG_INTR_RAW                    CIPHERHWA_BASE + 0x8048
#define CIPHERHWA_SELF_CONFIG_LP_EN                       CIPHERHWA_BASE + 0x8050
#define CIPHERHWA_SELF_CONFIG_RAM                         CIPHERHWA_BASE + 0xC000
/*ÖÐ¶Ï:0x8100*/
#define CIPHERHWA_INTR_EN                                 CIPHERHWA_BASE + 0x8100
#define CIPHERHWA_INTR_MASK                               CIPHERHWA_BASE + 0x8104
#define CIPHERHWA_INTR_STATUS                             CIPHERHWA_BASE + 0x8108

/*******************************************************************************
XCDMA registers' address on LC1860
********************************************************************************/
/*Í¨ÓÃÅäÖÃ×´Ì¬¼Ä´æÆ÷*/
#define XCDMA_GCS_STS                                     XCDMA_BASE + 0x0000
#define XCDMA_GCS_HWCFG                                   XCDMA_BASE + 0x0004
#define XCDMA_GCS_SBSTS                                   XCDMA_BASE + 0x0008
#define XCDMA_GCS_CFG                                     XCDMA_BASE + 0x000C
#define XCDMA_GCS_BISTS                                   XCDMA_BASE + 0x0010
#define XCDMA_GCS_SAMBP                                   XCDMA_BASE + 0x0014
#define XCDMA_GCS_DAMBP                                   XCDMA_BASE + 0x0018
#define XCDMA_GCS_ACBA                                    XCDMA_BASE + 0x001C
#define XCDMA_GCS_ACC                                     XCDMA_BASE + 0x0020
#define XCDMA_GCS_ACC_NEW                                 XCDMA_BASE + 0x0024
#define XCDMA_GCS_ACCADD                                  XCDMA_BASE + 0x0028
/*Êý¾Ý´«ÊäÅäÖÃ¼Ä´æÆ÷*/
#define XCDMA_DTC_CTRL                                    XCDMA_BASE + 0x0030
#define XCDMA_DTC_SSA                                     XCDMA_BASE + 0x0034
#define XCDMA_DTC_DSA                                     XCDMA_BASE + 0x0038
#define XCDMA_DTC_BEN                                     XCDMA_BASE + 0x003C
#define XCDMA_DTC_FCN                                     XCDMA_BASE + 0x0040
#define XCDMA_DTC_EPM                                     XCDMA_BASE + 0x0044
#define XCDMA_DTC_BPM                                     XCDMA_BASE + 0x0048
#define XCDMA_DTC_CPM                                     XCDMA_BASE + 0x004C
#define XCDMA_DTC_SCA                                     XCDMA_BASE + 0x0050
#define XCDMA_DTC_DCA                                     XCDMA_BASE + 0x0054
#define XCDMA_DTC_CBC                                     XCDMA_BASE + 0x0058
#define XCDMA_DTC_CEC                                     XCDMA_BASE + 0x005C

/*******************************************************************************
SIM0 registers' address on LC1860
********************************************************************************/
#define SIM0_PORT                                         SIM0_BASE + 0x0000
#define SIM0_CNTL                                         SIM0_BASE + 0x0004
#define SIM0_NA                                           SIM0_BASE + 0x0008
#define SIM0_ENABLE                                       SIM0_BASE + 0x000C
#define SIM0_INT_STAT                                     SIM0_BASE + 0x0010
#define SIM0_INT_RAW_STAT                                 SIM0_BASE + 0x0014
#define SIM0_INT_ENABLE                                   SIM0_BASE + 0x0018
#define SIM0_XMT_BUF                                      SIM0_BASE + 0x001C
#define SIM0_RCV_BUF                                      SIM0_BASE + 0x0020
#define SIM0_XMT_TH                                       SIM0_BASE + 0x0024
#define SIM0_GUARD                                        SIM0_BASE + 0x0028
#define SIM0_SRES                                         SIM0_BASE + 0x002C
#define SIM0_CH_WAIT                                      SIM0_BASE + 0x0030
#define SIM0_GPCNT                                        SIM0_BASE + 0x0034
#define SIM0_DIVISOR                                      SIM0_BASE + 0x0038
#define SIM0_LOWIMEN                                      SIM0_BASE + 0x0040
#define SIM0_HICLKNUM                                     SIM0_BASE + 0x0044
#define SIM0_HICNT0_SET                                   SIM0_BASE + 0x0048
#define SIM0_HICNT1_SET                                   SIM0_BASE + 0x004C
#define SIM0_HICNT2_SET                                   SIM0_BASE + 0x0050

/*******************************************************************************
SIM1 registers' address on LC1860
********************************************************************************/
#define SIM1_PORT                                         SIM1_BASE + 0x0000
#define SIM1_CNTL                                         SIM1_BASE + 0x0004
#define SIM1_NA                                           SIM1_BASE + 0x0008
#define SIM1_ENABLE                                       SIM1_BASE + 0x000C
#define SIM1_INT_STAT                                     SIM1_BASE + 0x0010
#define SIM1_INT_RAW_STAT                                 SIM1_BASE + 0x0014
#define SIM1_INT_ENABLE                                   SIM1_BASE + 0x0018
#define SIM1_XMT_BUF                                      SIM1_BASE + 0x001C
#define SIM1_RCV_BUF                                      SIM1_BASE + 0x0020
#define SIM1_XMT_TH                                       SIM1_BASE + 0x0024
#define SIM1_GUARD                                        SIM1_BASE + 0x0028
#define SIM1_SRES                                         SIM1_BASE + 0x002C
#define SIM1_CH_WAIT                                      SIM1_BASE + 0x0030
#define SIM1_GPCNT                                        SIM1_BASE + 0x0034
#define SIM1_DIVISOR                                      SIM1_BASE + 0x0038
#define SIM1_LOWIMEN                                      SIM1_BASE + 0x0040
#define SIM1_HICLKNUM                                     SIM1_BASE + 0x0044
#define SIM1_HICNT0_SET                                   SIM1_BASE + 0x0048
#define SIM1_HICNT1_SET                                   SIM1_BASE + 0x004C
#define SIM1_HICNT2_SET                                   SIM1_BASE + 0x0050


/*define controller bit for SIM_PORT register*/
#define SIM_PORT_SVEN 								    5
#define SIM_PORT_SVEN_0			            		    0
#define SIM_PORT_SVEN_1			            		    1

#define SIM_PORT_SCSP 									4
#define SIM_PORT_SCSP_SIMCLK0			            	0
#define SIM_PORT_SCSP_SIMCLK1			            	1

#define SIM_PORT_SCEN			                    	3
#define SIM_PORT_SCEN_CONSTANT	                    	0
#define SIM_PORT_SCEN_VARIABLE		                	1

#define SIM_PORT_SRST			                    	2
#define SIM_PORT_SRST_0			                    	0
#define SIM_PORT_SRST_1			                    	1

#define SIM_PORT_SAPD			                    	0
#define SIM_PORT_SAPD_AUTOLOST_EN                   	1
#define SIM_PORT_SAPD_AUTOLAST_DIS		            	0

/*define controller bit for SIM_CNTL register*/
#define SIM_CNTL_NACK_EN								9
#define SIM_CNTL_GP_EN					            	8
#define SIM_CNTL_CWTEN					            	7

#define SIM_CNTL_BAUD_SEL				            	4
#define SIM_CNTL_BAUD_SEL_372			            	0
#define SIM_CNTL_BAUD_SEL_256		                	1
#define SIM_CNTL_BAUD_SEL_128		                	2
#define SIM_CNTL_BAUD_SEL_64		                	3
#define SIM_CNTL_BAUD_SEL_32		                	4
#define SIM_CNTL_BAUD_SEL_16		                	6
#define SIM_CNTL_BAUD_SEL_DIVISOR                   	7

#define SIM_CNTL_ONACK					            	3
#define SIM_CNTL_ONACK_EN				            	1
#define SIM_CNTL_ONACK_DIS				            	0

#define SIM_CNTL_ANACK					            	2
#define SIM_CNTL_ANACK_EN				            	1
#define SIM_CNTL_ANACK_DIS				            	0

#define SIM_CNTL_ICM					            	1
#define SIM_CNTL_ICM_EN					            	1
#define SIM_CNTL_ICM_DIS				            	0

#define SIM_CNTL_IC						            	0
#define SIM_CNTL_IC_EN					            	1
#define SIM_CNTL_IC_DIS					            	0

/*define controller bit for SIM_ENABLE register*/
#define SIM_ENABLE_SIMEN								2
#define SIM_ENABLE_XMT_EN				            	1
#define SIM_ENABLE_RCV_EN				            	0

/*define all interrupt type for sim module*/
#define SIM_GPCNT_INTR									11
#define SIM_TC_INTR				            			10
#define SIM_TDNF_INTR				                	9
#define SIM_TDHF_INTR				                	8
#define SIM_TDAE_INTR				                	7
#define SIM_TDO_INTR				                	6
#define SIM_XTE_INTR				                	5
#define SIM_CWT_INTR	                            	4
#define SIM_RDAF_INTR	                            	3
#define SIM_RDHF_INTR	                            	2
#define SIM_RFNE_INTR	                            	1
#define SIM_RDOE_INTR	                            	0

/*define controller bit for SIM_RCV_BUF register*/
#define SIM_RCV_BUF_FE									9
#define SIM_RCV_BUF_PE			    	            	8
#define SIM_RCV_BUF_RCV			    	            	0

/*define controller bit for SIM_GUARD register*/
#define SIM_GUARD_RCVR11								8
#define SIM_GUARD_RCVR11_ETU12_STOP2	            	0
#define SIM_GUARD_RCVR11_ETU11_STOP1	            	1

#define SIM_GUARD_GETU			    	            	0
#define SIM_GUARD_GETU_0ETU	    	                    0


/*define controller bit for SIM_SRES register*/
#define SIM_SRES_SOFT_RES								2
#define SIM_SRES_FLUSH_XMT		    	            	1
#define SIM_SRES_FLUSH_RCV		    	            	0

/*******************************************************************************
CP_TIMER registers' address on LC1860
********************************************************************************/
/*CP_TIMER0µÄ¼Ä´æÆ÷*/
#define CP_TIMER0_TLC                                     CP_TIMER_BASE + 0x0000
#define CP_TIMER0_TCV                                     CP_TIMER_BASE + 0x0004
#define CP_TIMER0_TCR                                     CP_TIMER_BASE + 0x0008
#define CP_TIMER0_TIC                                     CP_TIMER_BASE + 0x000C
#define CP_TIMER0_TIS                                     CP_TIMER_BASE + 0x0010
/*CP_TIMER1µÄ¼Ä´æÆ÷*/
#define CP_TIMER1_TLC                                     CP_TIMER_BASE + 0x0014
#define CP_TIMER1_TCV                                     CP_TIMER_BASE + 0x0018
#define CP_TIMER1_TCR                                     CP_TIMER_BASE + 0x001C
#define CP_TIMER1_TIC                                     CP_TIMER_BASE + 0x0020
#define CP_TIMER1_TIS                                     CP_TIMER_BASE + 0x0024
/*CP_TIMER2µÄ¼Ä´æÆ÷*/
#define CP_TIMER2_TLC                                     CP_TIMER_BASE + 0x0028
#define CP_TIMER2_TCV                                     CP_TIMER_BASE + 0x002C
#define CP_TIMER2_TCR                                     CP_TIMER_BASE + 0x0030
#define CP_TIMER2_TIC                                     CP_TIMER_BASE + 0x0034
#define CP_TIMER2_TIS                                     CP_TIMER_BASE + 0x0038
/*CP_TIMER3µÄ¼Ä´æÆ÷*/
#define CP_TIMER3_TLC                                     CP_TIMER_BASE + 0x003C
#define CP_TIMER3_TCV                                     CP_TIMER_BASE + 0x0040
#define CP_TIMER3_TCR                                     CP_TIMER_BASE + 0x0044
#define CP_TIMER3_TIC                                     CP_TIMER_BASE + 0x0048
#define CP_TIMER3_TIS                                     CP_TIMER_BASE + 0x004C
/*CP_TIMERµÄÈ«¾Ö¼Ä´æÆ÷*/
#define CP_TIMER_TIS                                      CP_TIMER_BASE + 0x00A0
#define CP_TIMER_TIC                                      CP_TIMER_BASE + 0x00A4
#define CP_TIMER_RTIS                                     CP_TIMER_BASE + 0x00A8

/*******************************************************************************
CP_WDT registers' address on LC1860
********************************************************************************/
#define CP_WDT_CR                                         CP_WDT_BASE + 0x0000
#define CP_WDT_TORR                                       CP_WDT_BASE + 0x0004
#define CP_WDT_CCVR                                       CP_WDT_BASE + 0x0008
#define CP_WDT_CRR                                        CP_WDT_BASE + 0x000C
#define CP_WDT_STAT                                       CP_WDT_BASE + 0x0010
#define CP_WDT_ICR                                        CP_WDT_BASE + 0x0014


/*******************************************************************************
CP_PWR registers' address on LC1860
********************************************************************************/
/*Ë¯ÃßÏà¹Ø¼Ä´æÆ÷*/
#define CP_PWR_SLPCTL0                                    CP_PWR_BASE + 0x0000
#define CP_PWR_SLPCTL1                                    CP_PWR_BASE + 0x0004
#define CP_PWR_SLPCTL2                                    CP_PWR_BASE + 0x0008
#define CP_PWR_SLPCNT_LIMIT                               CP_PWR_BASE + 0x0010
#define CP_PWR_SLPST                                      CP_PWR_BASE + 0x0014
#define CP_PWR_PLLCR                                      CP_PWR_BASE + 0x0018
/*Ê±ÖÓÏà¹Ø¼Ä´æÆ÷*/
#define CP_PWR_PLL0CFG_CTL0                               CP_PWR_BASE + 0x0030
#define CP_PWR_PLL1CFG_CTL0                               CP_PWR_BASE + 0x0034
#define CP_PWR_PLL2CFG_CTL0                               CP_PWR_BASE + 0x0038
#define CP_PWR_PLL0CFG_CTL1                               CP_PWR_BASE + 0x0040
#define CP_PWR_PLL1CFG_CTL1                               CP_PWR_BASE + 0x0048
#define CP_PWR_PLL2CFG_CTL1                               CP_PWR_BASE + 0x004C
#define CP_PWR_A7_CLK_CTL                                 CP_PWR_BASE + 0x0044
#define CP_PWR_BUSMCLK0_CTL                               CP_PWR_BASE + 0x0050
#define CP_PWR_BUSMCLK1_CTL                               CP_PWR_BASE + 0x0054
#define CP_PWR_CTLPCLK_CTL                                CP_PWR_BASE + 0x0058
#define CP_PWR_CLK_EN0                                    CP_PWR_BASE + 0x0060
#define CP_PWR_CLK_EN1                                    CP_PWR_BASE + 0x0064
#define CP_PWR_CLK_EN2                                    CP_PWR_BASE + 0x0068
#define CP_PWR_CLK_EN3                                    CP_PWR_BASE + 0x006C
#define CP_PWR_CLK_EN4                                    CP_PWR_BASE + 0x0070
#define CP_PWR_CLK_EN5                                    CP_PWR_BASE + 0x0074
#define CP_PWR_PLL2CLKIN_CTL                              CP_PWR_BASE + 0x0088
#define CP_PWR_PLL1MCLK_CTL                               CP_PWR_BASE + 0x008C
#define CP_PWR_GPIFCLK_CTL                                CP_PWR_BASE + 0x0090
#define CP_PWR_D4GPCLK_CTL                                CP_PWR_BASE + 0x0094
#define CP_PWR_32KCLK_EN                                  CP_PWR_BASE + 0x00A0
#define CP_PWR_GSMSPCLK_CTL                               CP_PWR_BASE + 0x00A4
#define CP_PWR_GSMCALCLK_CTL                              CP_PWR_BASE + 0x00A8
#define CP_PWR_SIMCLK_CTL                                 CP_PWR_BASE + 0x00AC
#define CP_PWR_GSM1CLK_CTL0                               CP_PWR_BASE + 0x00B0
#define CP_PWR_GSM1CLK_CTL1                               CP_PWR_BASE + 0x00B4
#define CP_PWR_A5MCLK_CTL                                 CP_PWR_BASE + 0x00B8
#define CP_PWR_GEAMCLK_CTL                                CP_PWR_BASE + 0x00BC
#define CP_PWR_SPICLK_CTL                                 CP_PWR_BASE + 0x00C0
#define CP_PWR_RFFECLK_CTL                                CP_PWR_BASE + 0x00C4
#define CP_PWR_TDSPCLK_CTL                                CP_PWR_BASE + 0x00D0
#define CP_PWR_TDCALCLK_CTL                               CP_PWR_BASE + 0x00D4
#define CP_PWR_WSPCLK_CTL                                 CP_PWR_BASE + 0x00D8
#define CP_PWR_WCALCLK_CTL                                CP_PWR_BASE + 0x00DC
#define CP_PWR_MATCHCLK_CTL                               CP_PWR_BASE + 0x00E0
#define CP_PWR_D4GRXCLK_CTL                               CP_PWR_BASE + 0x00E4
#define CP_PWR_C2KSPCLK_CTL                               CP_PWR_BASE + 0x00E8
#define CP_PWR_C2KCALCLK_CTL                              CP_PWR_BASE + 0x00EC
#define CP_PWR_TDDSPCLK_CTL                               CP_PWR_BASE + 0x00F0
#define CP_PWR_TDDCALCLK_CTL                              CP_PWR_BASE + 0x00F4
#define CP_PWR_FDDSPCLK_CTL                               CP_PWR_BASE + 0x00F8
#define CP_PWR_FDDCALCLK_CTL                              CP_PWR_BASE + 0x00FC
#define CP_PWR_FIRCLK_CTL                                 CP_PWR_BASE + 0x0100
#define CP_PWR_PLL2MCLK_CTL                               CP_PWR_BASE + 0x0104
#define CP_PWR_TIMER0CLKCTL                               CP_PWR_BASE + 0x0120
#define CP_PWR_TIMER1CLKCTL                               CP_PWR_BASE + 0x0124
#define CP_PWR_TIMER2CLKCTL                               CP_PWR_BASE + 0x0128
#define CP_PWR_TIMER3CLKCTL                               CP_PWR_BASE + 0x012C
#define CP_PWR_CLKOUTSEL                                  CP_PWR_BASE + 0x0140
#define CP_PWR_CLKOUT0CLKCTL                              CP_PWR_BASE + 0x0144
/*¸´Î»Ïà¹Ø¼Ä´æÆ÷*/
#define CP_PWR_A7_RSTCTL0                                 CP_PWR_BASE + 0x0180
#define CP_PWR_A7_RSTCTL1                                 CP_PWR_BASE + 0x0184
#define CP_PWR_CEVA_RSTCTL                                CP_PWR_BASE + 0x0188
#define CP_PWR_RSTCTL0                                    CP_PWR_BASE + 0x0190
#define CP_PWR_RSTCTL1                                    CP_PWR_BASE + 0x0194
#define CP_PWR_RSTCTL2                                    CP_PWR_BASE + 0x0198
#define CP_PWR_WDTRST_CTL                                 CP_PWR_BASE + 0x01B0
/*ÖÐ¶ÏÏà¹Ø¼Ä´æÆ÷*/
#define CP_PWR_A7INTIN_MK                                 CP_PWR_BASE + 0x01E0
#define CP_PWR_INT_RAW                                    CP_PWR_BASE + 0x01F0
#define CP_PWR_INTEN_A7                                   CP_PWR_BASE + 0x01F4
#define CP_PWR_INTEN_1643                                 CP_PWR_BASE + 0x01F8
#define CP_PWR_INTEN_4210                                 CP_PWR_BASE + 0x01FC
#define CP_PWR_INTST_A7                                   CP_PWR_BASE + 0x0200
#define CP_PWR_INTST_1643                                 CP_PWR_BASE + 0x0204
#define CP_PWR_INTST_4210                                 CP_PWR_BASE + 0x0208
#define CP_PWR_INTR_FLAG0                                 CP_PWR_BASE + 0x0210
#define CP_PWR_INTR_FLAG1                                 CP_PWR_BASE + 0x0214
/*µçÔ´¹ÜÀíÏà¹Ø¼Ä´æÆ÷*/
#define CP_PWR_A7_SCU_PD_CTL                              CP_PWR_BASE + 0x0220
#define CP_PWR_1643_PD_CTL                                CP_PWR_BASE + 0x0224
#define CP_PWR_4210_PD_CTL                                CP_PWR_BASE + 0x0228
#define CP_PWR_A7_CORE_PD_CTL                             CP_PWR_BASE + 0x022C
#define CP_PWR_1643_PD_CTL1                               CP_PWR_BASE + 0x0234
#define CP_PWR_4210_PD_CTL1                               CP_PWR_BASE + 0x0238
#define CP_PWR_L12ACC_PD_CTL                              CP_PWR_BASE + 0x0240
#define CP_PWR_RF_PD_CTL                                  CP_PWR_BASE + 0x0244
#define CP_PWR_SHRAM0_PD_CTL                              CP_PWR_BASE + 0x0248
#define CP_PWR_A7_SCU_PD_CNT1                             CP_PWR_BASE + 0x0260
#define CP_PWR_1643_PD_CNT1                               CP_PWR_BASE + 0x0264
#define CP_PWR_4210_PD_CNT1                               CP_PWR_BASE + 0x0268
#define CP_PWR_L12ACC_PD_CNT1                             CP_PWR_BASE + 0x026C
#define CP_PWR_RF_PD_CNT1                                 CP_PWR_BASE + 0x0270
#define CP_PWR_SHRAM0_PD_CNT1                             CP_PWR_BASE + 0x0274
#define CP_PWR_A7_CORE_PD_CNT1                            CP_PWR_BASE + 0x027C
#define CP_PWR_A7_SCU_PD_CNT2                             CP_PWR_BASE + 0x0280
#define CP_PWR_1643_PD_CNT2                               CP_PWR_BASE + 0x0284
#define CP_PWR_4210_PD_CNT2                               CP_PWR_BASE + 0x0288
#define CP_PWR_L12ACC_PD_CNT2                             CP_PWR_BASE + 0x028C
#define CP_PWR_RF_PD_CNT2                                 CP_PWR_BASE + 0x0290
#define CP_PWR_SHRAM0_PD_CNT2                             CP_PWR_BASE + 0x0294
#define CP_PWR_A7_CORE_PD_CNT2                            CP_PWR_BASE + 0x029C
#define CP_PWR_A7_PD_CNT3                                 CP_PWR_BASE + 0x02A0
#define CP_PWR_1643_PD_CNT3                               CP_PWR_BASE + 0x02A4
#define CP_PWR_4210_PD_CNT3                               CP_PWR_BASE + 0x02A8
#define CP_PWR_L12ACC_PD_CNT3                             CP_PWR_BASE + 0x02AC
#define CP_PWR_RF_PD_CNT3                                 CP_PWR_BASE + 0x02B0
#define CP_PWR_SHRAM0_PD_CNT3                             CP_PWR_BASE + 0x02B4
#define CP_PWR_A7_CORE_PD_CNT3                            CP_PWR_BASE + 0x02BC
#define CP_PWR_PDFSM_ST0                                  CP_PWR_BASE + 0x02C0
#define CP_PWR_PDFSM_ST1                                  CP_PWR_BASE + 0x02C4
/*ÆäËü¼Ä´æÆ÷*/
#define CP_PWR_RES_REG                                    CP_PWR_BASE + 0x0320
#define CP_PWR_DEVICE_ST                                  CP_PWR_BASE + 0x0330
#define CP_PWR_LP_CTL                                     CP_PWR_BASE + 0x0334
#define CP_PWR_DFS_LIMIT                                  CP_PWR_BASE + 0x0338
#define CP_PWR_TM32K_CTL                                  CP_PWR_BASE + 0x0344
#define CP_PWR_TM32K_INIT_VAL                             CP_PWR_BASE + 0x0348
#define CP_PWR_TM_CUR_VAL                                 CP_PWR_BASE + 0x034C
#define CP_PWR_BUSLP_CTL                                  CP_PWR_BASE + 0x0350
#define CP_PWR_BUSLP_EN0                                  CP_PWR_BASE + 0x0360
#define CP_PWR_BUSLP_EN1                                  CP_PWR_BASE + 0x0364
#define CP_PWR_BUSLP_EN2                                  CP_PWR_BASE + 0x0368
#define CP_PWR_CEVA_CTL                                   CP_PWR_BASE + 0x0370
#define CP_PWR_TSTPIN_CTL                                 CP_PWR_BASE + 0x0374
#define CP_PWR_ACCESS_ERR_CLR                             CP_PWR_BASE + 0x0380
#define CP_PWR_LPCTRL_FSM_ST0                             CP_PWR_BASE + 0x038C
#define CP_PWR_LPCTRL_FSM_ST1                             CP_PWR_BASE + 0x0390
#define CP_PWR_LPCTRL_FSM_ST2                             CP_PWR_BASE + 0x0394
#define CP_PWR_LPCTRL_FSM_ST3                             CP_PWR_BASE + 0x0398

/*******************************************************************************
RTC registers' address on LC1860
********************************************************************************/
#define RTC_CCVR                         RTC_BASE + 0x0000
#define RTC_CLR                          RTC_BASE + 0x0004
#define RTC_CMR_ONE                      RTC_BASE + 0x0008
#define RTC_CMR_TWO                      RTC_BASE + 0x000C
#define RTC_CMR_THREE                    RTC_BASE + 0x0010
#define RTC_ICR                          RTC_BASE + 0x0014
#define RTC_ISR                          RTC_BASE + 0x0018
#define RTC_EOI                          RTC_BASE + 0x0018
#define RTC_WVR                          RTC_BASE + 0x001C
#define RTC_WLR                          RTC_BASE + 0x0020
#define RTC_RAW_LIMIT                    RTC_BASE + 0x0024
#define RTC_SECOND_LIMIT                 RTC_BASE + 0x0028
#define RTC_MINUTE_LIMIT                 RTC_BASE + 0x002C
#define RTC_HOUR_LIMIT                   RTC_BASE + 0x0030
#define RTC_ISR_RAW                      RTC_BASE + 0x0034
#define RTC_RVR                          RTC_BASE + 0x0038

/*******************************************************************************
USB_OTG register map
********************************************************************************/
//GLOBAL register
#define USB_OTG_GOTGCTL                  USB_OTG_BASE + 0x000
#define USB_OTG_GOTGINT                  USB_OTG_BASE + 0x004
#define USB_OTG_GAHBCFG                  USB_OTG_BASE + 0x008
#define USB_OTG_GUSBCFG                  USB_OTG_BASE + 0x00C
#define USB_OTG_GRSTCTL                  USB_OTG_BASE + 0x010
#define USB_OTG_GINTSTS                  USB_OTG_BASE + 0x014
#define USB_OTG_GINTMSK                  USB_OTG_BASE + 0x018
#define USB_OTG_GRXSTSR                  USB_OTG_BASE + 0x01C
#define USB_OTG_GRXSTSP                  USB_OTG_BASE + 0x020
#define USB_OTG_GRXFSIZ                  USB_OTG_BASE + 0x024
#define USB_OTG_GNPTXFSIZ                USB_OTG_BASE + 0x028
#define USB_OTG_GNPTXSTS                 USB_OTG_BASE + 0x02C
#define USB_OTG_GI2CCTL                  USB_OTG_BASE + 0x030
#define USB_OTG_GPVNDCTL                 USB_OTG_BASE + 0x034
#define USB_OTG_GGPIO                    USB_OTG_BASE + 0x038
#define USB_OTG_GUID                     USB_OTG_BASE + 0x03C
#define USB_OTG_GSNPSID                  USB_OTG_BASE + 0x040
#define USB_OTG_GHWCFG1                  USB_OTG_BASE + 0x044
#define USB_OTG_GHWCFG2                  USB_OTG_BASE + 0x048
#define USB_OTG_GHWCFG3                  USB_OTG_BASE + 0x04C
#define USB_OTG_GHWCFG4                  USB_OTG_BASE + 0x050

#define USB_OTG_HPTXFSIZ             	 USB_OTG_BASE + 0x100
#define USB_OTG_DPTXFSIZ1                USB_OTG_BASE + 0x104
#define USB_OTG_DPTXFSIZ2                USB_OTG_BASE + 0x108
#define USB_OTG_DPTXFSIZ3                USB_OTG_BASE + 0x10c
#define USB_OTG_DPTXFSIZ4                USB_OTG_BASE + 0x110
#define USB_OTG_DPTXFSIZ5                USB_OTG_BASE + 0x114
#define USB_OTG_DPTXFSIZ6                USB_OTG_BASE + 0x118
#define USB_OTG_DPTXFSIZ7                USB_OTG_BASE + 0x11c
#define USB_OTG_DPTXFSIZ8                USB_OTG_BASE + 0x120
#define USB_OTG_DPTXFSIZ9                USB_OTG_BASE + 0x124
#define USB_OTG_DPTXFSIZ10               USB_OTG_BASE + 0x128
#define USB_OTG_DPTXFSIZ11               USB_OTG_BASE + 0x12c
#define USB_OTG_DPTXFSIZ12               USB_OTG_BASE + 0x130
#define USB_OTG_DPTXFSIZ13               USB_OTG_BASE + 0x134
#define USB_OTG_DPTXFSIZ14               USB_OTG_BASE + 0x138
#define USB_OTG_DPTXFSIZ15               USB_OTG_BASE + 0x13c
//HOST
#define USB_OTG_HCFG                     USB_OTG_BASE + 0x400
#define USB_OTG_HFIR                     USB_OTG_BASE + 0x404
#define USB_OTG_HFNUM                    USB_OTG_BASE + 0x408
#define USB_OTG_HPTXSTS                  USB_OTG_BASE + 0x410
#define USB_OTG_HAINT                    USB_OTG_BASE + 0x414
#define USB_OTG_HAINTMSK                 USB_OTG_BASE + 0x418
#define USB_OTG_HPRT0                    USB_OTG_BASE + 0x440
#define USB_OTG_HCCHAR0                  USB_OTG_BASE + 0x500
#define USB_OTG_HCSPLT0                  USB_OTG_BASE + 0x504
#define USB_OTG_HCINT0                   USB_OTG_BASE + 0x508
#define USB_OTG_HCINTMSK0                USB_OTG_BASE + 0x50C
#define USB_OTG_HCTSIZ0                  USB_OTG_BASE + 0x510
#define USB_OTG_HCDMA0                   USB_OTG_BASE + 0x514
#define USB_OTG_HCCHAR1                  USB_OTG_BASE + 0x520
#define USB_OTG_HCSPLT1                  USB_OTG_BASE + 0x524
#define USB_OTG_HCINT1                   USB_OTG_BASE + 0x528
#define USB_OTG_HCINTMSK1                USB_OTG_BASE + 0x52C
#define USB_OTG_HCTSIZ1                  USB_OTG_BASE + 0x530
#define USB_OTG_HCDMA1                   USB_OTG_BASE + 0x534
#define USB_OTG_HCCHAR2                  USB_OTG_BASE + 0x540
#define USB_OTG_HCSPLT2                  USB_OTG_BASE + 0x544
#define USB_OTG_HCINT2                   USB_OTG_BASE + 0x548
#define USB_OTG_HCINTMSK2                USB_OTG_BASE + 0x54C
#define USB_OTG_HCTSIZ2                  USB_OTG_BASE + 0x550
#define USB_OTG_HCDMA2                   USB_OTG_BASE + 0x554
#define USB_OTG_HCCHAR3                  USB_OTG_BASE + 0x560
#define USB_OTG_HCSPLT3                  USB_OTG_BASE + 0x564
#define USB_OTG_HCINT3                   USB_OTG_BASE + 0x568
#define USB_OTG_HCINTMSK3                USB_OTG_BASE + 0x56C
#define USB_OTG_HCTSIZ3                  USB_OTG_BASE + 0x570
#define USB_OTG_HCDMA3                   USB_OTG_BASE + 0x574
#define USB_OTG_HCCHAR4                  USB_OTG_BASE + 0x580
#define USB_OTG_HCSPLT4                  USB_OTG_BASE + 0x584
#define USB_OTG_HCINT4                   USB_OTG_BASE + 0x588
#define USB_OTG_HCINTMSK4                USB_OTG_BASE + 0x58C
#define USB_OTG_HCTSIZ4                  USB_OTG_BASE + 0x590
#define USB_OTG_HCDMA4                   USB_OTG_BASE + 0x594
#define USB_OTG_HCCHAR5                  USB_OTG_BASE + 0x5A0
#define USB_OTG_HCSPLT5                  USB_OTG_BASE + 0x5A4
#define USB_OTG_HCINT5                   USB_OTG_BASE + 0x5A8
#define USB_OTG_HCINTMSK5                USB_OTG_BASE + 0x5AC
#define USB_OTG_HCTSIZ5                  USB_OTG_BASE + 0x5B0
#define USB_OTG_HCDMA5                   USB_OTG_BASE + 0x5B4
#define USB_OTG_HCCHAR6                  USB_OTG_BASE + 0x5C0
#define USB_OTG_HCSPLT6                  USB_OTG_BASE + 0x5C4
#define USB_OTG_HCINT6                   USB_OTG_BASE + 0x5C8
#define USB_OTG_HCINTMSK6                USB_OTG_BASE + 0x5CC
#define USB_OTG_HCTSIZ6                  USB_OTG_BASE + 0x5D0
#define USB_OTG_HCDMA6                   USB_OTG_BASE + 0x5D4
#define USB_OTG_HCCHAR7                  USB_OTG_BASE + 0x5E0
#define USB_OTG_HCSPLT7                  USB_OTG_BASE + 0x5E4
#define USB_OTG_HCINT7                   USB_OTG_BASE + 0x5E8
#define USB_OTG_HCINTMSK7                USB_OTG_BASE + 0x5EC
#define USB_OTG_HCTSIZ7                  USB_OTG_BASE + 0x5F0
#define USB_OTG_HCDMA7                   USB_OTG_BASE + 0x5F4
#define USB_OTG_HCCHAR8                  USB_OTG_BASE + 0x600
#define USB_OTG_HCSPLT8                  USB_OTG_BASE + 0x604
#define USB_OTG_HCINT8                   USB_OTG_BASE + 0x608
#define USB_OTG_HCINTMSK8                USB_OTG_BASE + 0x60C
#define USB_OTG_HCTSIZ8                  USB_OTG_BASE + 0x610
#define USB_OTG_HCDMA8                   USB_OTG_BASE + 0x614
#define USB_OTG_HCCHAR9                  USB_OTG_BASE + 0x620
#define USB_OTG_HCSPLT9                  USB_OTG_BASE + 0x624
#define USB_OTG_HCINT9                   USB_OTG_BASE + 0x628
#define USB_OTG_HCINTMSK9                USB_OTG_BASE + 0x62C
#define USB_OTG_HCTSIZ9                  USB_OTG_BASE + 0x630
#define USB_OTG_HCDMA9                   USB_OTG_BASE + 0x634
#define USB_OTG_HCCHAR10                 USB_OTG_BASE + 0x640
#define USB_OTG_HCSPLT10                 USB_OTG_BASE + 0x644
#define USB_OTG_HCINT10                  USB_OTG_BASE + 0x648
#define USB_OTG_HCINTMSK10               USB_OTG_BASE + 0x64C
#define USB_OTG_HCTSIZ10                 USB_OTG_BASE + 0x650
#define USB_OTG_HCDMA10                  USB_OTG_BASE + 0x654
#define USB_OTG_HCCHAR11                 USB_OTG_BASE + 0x660
#define USB_OTG_HCSPLT11                 USB_OTG_BASE + 0x664
#define USB_OTG_HCINT11                  USB_OTG_BASE + 0x668
#define USB_OTG_HCINTMSK11               USB_OTG_BASE + 0x66C
#define USB_OTG_HCTSIZ11                 USB_OTG_BASE + 0x670
#define USB_OTG_HCDMA11                  USB_OTG_BASE + 0x674
#define USB_OTG_HCCHAR12                 USB_OTG_BASE + 0x680
#define USB_OTG_HCSPLT12                 USB_OTG_BASE + 0x684
#define USB_OTG_HCINT12                  USB_OTG_BASE + 0x688
#define USB_OTG_HCINTMSK12               USB_OTG_BASE + 0x68C
#define USB_OTG_HCTSIZ12                 USB_OTG_BASE + 0x690
#define USB_OTG_HCDMA12                  USB_OTG_BASE + 0x694
#define USB_OTG_HCCHAR13                 USB_OTG_BASE + 0x6A0
#define USB_OTG_HCSPLT13                 USB_OTG_BASE + 0x6A4
#define USB_OTG_HCINT13                  USB_OTG_BASE + 0x6A8
#define USB_OTG_HCINTMSK13               USB_OTG_BASE + 0x6AC
#define USB_OTG_HCTSIZ13                 USB_OTG_BASE + 0x6B0
#define USB_OTG_HCDMA13                  USB_OTG_BASE + 0x6B4
#define USB_OTG_HCCHAR14                 USB_OTG_BASE + 0x6C0
#define USB_OTG_HCSPLT14                 USB_OTG_BASE + 0x6C4
#define USB_OTG_HCINT14                  USB_OTG_BASE + 0x6C8
#define USB_OTG_HCINTMSK14               USB_OTG_BASE + 0x6CC
#define USB_OTG_HCTSIZ14                 USB_OTG_BASE + 0x6D0
#define USB_OTG_HCDMA14                  USB_OTG_BASE + 0x6D4
#define USB_OTG_HCCHAR15                 USB_OTG_BASE + 0x6E0
#define USB_OTG_HCSPLT15                 USB_OTG_BASE + 0x6E4
#define USB_OTG_HCINT15                  USB_OTG_BASE + 0x6E8
#define USB_OTG_HCINTMSK15               USB_OTG_BASE + 0x6EC
#define USB_OTG_HCTSIZ15                 USB_OTG_BASE + 0x6F0
#define USB_OTG_HCDMA15                  USB_OTG_BASE + 0x6F4

#define USB_OTG_DCFG                      USB_OTG_BASE + 0x800
#define USB_OTG_DCTL                      USB_OTG_BASE + 0x804
#define USB_OTG_DSTS                      USB_OTG_BASE + 0x808
#define USB_OTG_DIEPMSK                   USB_OTG_BASE + 0x810
#define USB_OTG_DOEPMSK                   USB_OTG_BASE + 0x814
#define USB_OTG_DAINT                     USB_OTG_BASE + 0x818
#define USB_OTG_DAINTMSK                  USB_OTG_BASE + 0x81C
#define USB_OTG_DBBUSDIS                  USB_OTG_BASE + 0x828
#define USB_OTG_DBBUSPULSE                USB_OTG_BASE + 0x82C
#define USB_OTG_DTHRCTL                   USB_OTG_BASE + 0x830
#define USB_OTG_DIEPEMPMSK                USB_OTG_BASE + 0x834
#define USB_OTG_DIEPCTL0                  USB_OTG_BASE + 0x900  
#define USB_OTG_DIEPINT0                  USB_OTG_BASE + 0x908  
#define USB_OTG_DIEPTSIZ0                 USB_OTG_BASE + 0x910  
#define USB_OTG_DIEPDMA0                  USB_OTG_BASE + 0x914  
#define USB_OTG_DTXFSTS0                  USB_OTG_BASE + 0x918  
#define USB_OTG_DIEPDMAB0                 USB_OTG_BASE + 0x91c  
#define USB_OTG_DIEPCTL1                  USB_OTG_BASE + 0x920  
#define USB_OTG_DIEPINT1                  USB_OTG_BASE + 0x928  
#define USB_OTG_DIEPTSIZ1                 USB_OTG_BASE + 0x930  
#define USB_OTG_DIEPDMA1                  USB_OTG_BASE + 0x934  
#define USB_OTG_DTXFSTS1                  USB_OTG_BASE + 0x938  
#define USB_OTG_DIEPDMAB1                 USB_OTG_BASE + 0x93c  
#define USB_OTG_DIEPCTL2                  USB_OTG_BASE + 0x940  
#define USB_OTG_DIEPINT2                  USB_OTG_BASE + 0x948  
#define USB_OTG_DIEPTSIZ2                 USB_OTG_BASE + 0x950  
#define USB_OTG_DIEPDMA2                  USB_OTG_BASE + 0x954  
#define USB_OTG_DTXFSTS2                  USB_OTG_BASE + 0x958  
#define USB_OTG_DIEPDMAB2                 USB_OTG_BASE + 0x95c  
#define USB_OTG_DIEPCTL3                  USB_OTG_BASE + 0x960  
#define USB_OTG_DIEPINT3                  USB_OTG_BASE + 0x968  
#define USB_OTG_DIEPTSIZ3                 USB_OTG_BASE + 0x970  
#define USB_OTG_DIEPDMA3                  USB_OTG_BASE + 0x974  
#define USB_OTG_DTXFSTS3                  USB_OTG_BASE + 0x978  
#define USB_OTG_DIEPDMAB3                 USB_OTG_BASE + 0x97C  
#define USB_OTG_DIEPCTL4                  USB_OTG_BASE + 0x980  
#define USB_OTG_DIEPINT4                  USB_OTG_BASE + 0x988  
#define USB_OTG_DIEPTSIZ4                 USB_OTG_BASE + 0x990  
#define USB_OTG_DIEPDMA4                  USB_OTG_BASE + 0x994  
#define USB_OTG_DTXFSTS4                  USB_OTG_BASE + 0x998  
#define USB_OTG_DIEPDMAB4                 USB_OTG_BASE + 0x99C  
#define USB_OTG_DIEPCTL5                  USB_OTG_BASE + 0x9A0  
#define USB_OTG_DIEPINT5                  USB_OTG_BASE + 0x9A8  
#define USB_OTG_DIEPTSIZ5                 USB_OTG_BASE + 0x9B0  
#define USB_OTG_DIEPDMA5                  USB_OTG_BASE + 0x9B4  
#define USB_OTG_DTXFSTS5                  USB_OTG_BASE + 0x9B8  
#define USB_OTG_DIEPDMAB5                 USB_OTG_BASE + 0x9BC  
#define USB_OTG_DIEPCTL6                  USB_OTG_BASE + 0x9C0  
#define USB_OTG_DIEPINT6                  USB_OTG_BASE + 0x9C8  
#define USB_OTG_DIEPTSIZ6                 USB_OTG_BASE + 0x9D0  
#define USB_OTG_DIEPDMA6                  USB_OTG_BASE + 0x9D4  
#define USB_OTG_DTXFSTS6                  USB_OTG_BASE + 0x9D8  
#define USB_OTG_DIEPDMAB6                 USB_OTG_BASE + 0x9DC  
#define USB_OTG_DIEPCTL7                  USB_OTG_BASE + 0x9E0  
#define USB_OTG_DIEPINT7                  USB_OTG_BASE + 0x9E8  
#define USB_OTG_DIEPTSIZ7                 USB_OTG_BASE + 0x9F0  
#define USB_OTG_DIEPDMA7                  USB_OTG_BASE + 0x9F4  
#define USB_OTG_DTXFSTS7                  USB_OTG_BASE + 0x9F8  
#define USB_OTG_DIEPDMAB7                 USB_OTG_BASE + 0x9Fc  
#define USB_OTG_DOEPCTL0                  USB_OTG_BASE + 0xB00 
#define USB_OTG_DOEPFN0                   USB_OTG_BASE + 0xB04 
#define USB_OTG_DOEPINT0                  USB_OTG_BASE + 0xB08 
#define USB_OTG_DOEPTSIZ0                 USB_OTG_BASE + 0xB10 
#define USB_OTG_DOEPDMA0                  USB_OTG_BASE + 0xB14 
#define USB_OTG_DOEPDMAB0                 USB_OTG_BASE + 0xB1C 
#define USB_OTG_DOEPCTL8                  USB_OTG_BASE + 0xC00 
#define USB_OTG_DOEPFN8                   USB_OTG_BASE + 0xC04 
#define USB_OTG_DOEPINT8                  USB_OTG_BASE + 0xC08 
#define USB_OTG_DOEPTSIZ8                 USB_OTG_BASE + 0xC10 
#define USB_OTG_DOEPDMA8                  USB_OTG_BASE + 0xC14 
#define USB_OTG_DOEPDMAB8                 USB_OTG_BASE + 0xC1C 
#define USB_OTG_DOEPCTL9                  USB_OTG_BASE + 0xC20 
#define USB_OTG_DOEPFN9                   USB_OTG_BASE + 0xC24 
#define USB_OTG_DOEPINT9                  USB_OTG_BASE + 0xC28 
#define USB_OTG_DOEPTSIZ9                 USB_OTG_BASE + 0xC30 
#define USB_OTG_DOEPDMA9                  USB_OTG_BASE + 0xC34 
#define USB_OTG_DOEPDMAB9                 USB_OTG_BASE + 0xC3C 
#define USB_OTG_DOEPCTL10                 USB_OTG_BASE + 0xC40 
#define USB_OTG_DOEPFN10                  USB_OTG_BASE + 0xC44 
#define USB_OTG_DOEPINT10                 USB_OTG_BASE + 0xC48 
#define USB_OTG_DOEPTSIZ10                USB_OTG_BASE + 0xC50 
#define USB_OTG_DOEPDMA10                 USB_OTG_BASE + 0xC54 
#define USB_OTG_DOEPDMAB10                USB_OTG_BASE + 0xC5C 
#define USB_OTG_DOEPCTL11                 USB_OTG_BASE + 0xC60 
#define USB_OTG_DOEPFN11                  USB_OTG_BASE + 0xC64 
#define USB_OTG_DOEPINT11                 USB_OTG_BASE + 0xC68 
#define USB_OTG_DOEPTSIZ11                USB_OTG_BASE + 0xC70 
#define USB_OTG_DOEPDMA11                 USB_OTG_BASE + 0xC74 
#define USB_OTG_DOEPDMAB11                USB_OTG_BASE + 0xC7C 
#define USB_OTG_DOEPCTL12                 USB_OTG_BASE + 0xC80 
#define USB_OTG_DOEPFN12                  USB_OTG_BASE + 0xC84 
#define USB_OTG_DOEPINT12                 USB_OTG_BASE + 0xC88 
#define USB_OTG_DOEPTSIZ12                USB_OTG_BASE + 0xC90 
#define USB_OTG_DOEPDMA12                 USB_OTG_BASE + 0xC94 
#define USB_OTG_DOEPDMAB12                USB_OTG_BASE + 0xC9C 
#define USB_OTG_DOEPCTL13                 USB_OTG_BASE + 0xCA0 
#define USB_OTG_DOEPFN13                  USB_OTG_BASE + 0xCA4 
#define USB_OTG_DOEPINT13                 USB_OTG_BASE + 0xCA8 
#define USB_OTG_DOEPTSIZ13                USB_OTG_BASE + 0xCB0 
#define USB_OTG_DOEPDMA13                 USB_OTG_BASE + 0xCB4 
#define USB_OTG_DOEPDMAB13                USB_OTG_BASE + 0xCBC 
#define USB_OTG_DOEPCTL14                 USB_OTG_BASE + 0xCC0 
#define USB_OTG_DOEPFN14                  USB_OTG_BASE + 0xCC4 
#define USB_OTG_DOEPINT14                 USB_OTG_BASE + 0xCC8 
#define USB_OTG_DOEPTSIZ14                USB_OTG_BASE + 0xCD0 
#define USB_OTG_DOEPDMA14                 USB_OTG_BASE + 0xCD4 
#define USB_OTG_DOEPDMAB14                USB_OTG_BASE + 0xCDC 

#define		__OFFSET__		   4
#define     USB_OTG_DFIFO0                  ((REGISTER*)(USB_OTG_BASE + 0x0400*__OFFSET__))
#define     USB_OTG_DFIFO1                  ((REGISTER*)(USB_OTG_BASE + 0x0800*__OFFSET__))
#define     USB_OTG_DFIFO2                  ((REGISTER*)(USB_OTG_BASE + 0x0c00*__OFFSET__))
#define     USB_OTG_DFIFO3                  ((REGISTER*)(USB_OTG_BASE + 0x1000*__OFFSET__))
#define     USB_OTG_DFIFO4                  ((REGISTER*)(USB_OTG_BASE + 0x1400*__OFFSET__))
#define     USB_OTG_DFIFO5                  ((REGISTER*)(USB_OTG_BASE + 0x1800*__OFFSET__))
#define     USB_OTG_DFIFO6                  ((REGISTER*)(USB_OTG_BASE + 0x1c00*__OFFSET__))
#define     USB_OTG_DFIFO7                  ((REGISTER*)(USB_OTG_BASE + 0x2000*__OFFSET__))
#define     USB_OTG_DFIFO8                  ((REGISTER*)(USB_OTG_BASE + 0x2400*__OFFSET__))
#define     USB_OTG_DFIFO9                  ((REGISTER*)(USB_OTG_BASE + 0x2800*__OFFSET__))
#define     USB_OTG_DFIFO10                 ((REGISTER*)(USB_OTG_BASE + 0x2c00*__OFFSET__))
#define     USB_OTG_DFIFO11                 ((REGISTER*)(USB_OTG_BASE + 0x3000*__OFFSET__))
#define     USB_OTG_DFIFO12                 ((REGISTER*)(USB_OTG_BASE + 0x3400*__OFFSET__))
#define     USB_OTG_DFIFO13                 ((REGISTER*)(USB_OTG_BASE + 0x3800*__OFFSET__))
#define     USB_OTG_DFIFO14                 ((REGISTER*)(USB_OTG_BASE + 0x3c00*__OFFSET__))
#define     USB_OTG_DFIFO15                 ((REGISTER*)(USB_OTG_BASE + 0x4000*__OFFSET__))

/********************************************************************************
USB_HSIC register map
********************************************************************************/
#define USB_HSIC_GOTGCTL		USB_HSIC_BASE + 0x0000 
#define USB_HSIC_GOTGINT		USB_HSIC_BASE + 0x0004 
#define USB_HSIC_GAHBCFG		USB_HSIC_BASE + 0x0008 
#define USB_HSIC_GUSBCFG		USB_HSIC_BASE + 0x000C 
#define USB_HSIC_GRSTCTL		USB_HSIC_BASE + 0x0010 
#define USB_HSIC_GINTSTS		USB_HSIC_BASE + 0x0014 
#define USB_HSIC_GINTMSK		USB_HSIC_BASE + 0x0018 
#define USB_HSIC_GRXSTSR		USB_HSIC_BASE + 0x001C 
#define USB_HSIC_GRXSTSP		USB_HSIC_BASE + 0x0020 
#define USB_HSIC_GRXFSIZ		USB_HSIC_BASE + 0x0024 
#define USB_HSIC_GNPTXFSIZ		USB_HSIC_BASE + 0x0028 
#define USB_HSIC_GNPTXSTS		USB_HSIC_BASE + 0x002C 
#define USB_HSIC_GI2CCTL		USB_HSIC_BASE + 0x0030 
#define USB_HSIC_GPVNDCTL		USB_HSIC_BASE + 0x0034 
#define USB_HSIC_GGPIO			USB_HSIC_BASE + 0x0038 
#define USB_HSIC_GUID			USB_HSIC_BASE + 0x003C 
#define USB_HSIC_GSNPSID		USB_HSIC_BASE + 0x0040 
#define USB_HSIC_GHWCFG1		USB_HSIC_BASE + 0x0044 
#define USB_HSIC_GHWCFG2		USB_HSIC_BASE + 0x0048 
#define USB_HSIC_GHWCFG3		USB_HSIC_BASE + 0x004C 
#define USB_HSIC_GHWCFG4		USB_HSIC_BASE + 0x0050 
#define USB_HSIC_GLPMCFG		USB_HSIC_BASE + 0x0054 
#define USB_HSIC_HPTXFSIZ		USB_HSIC_BASE + 0x0100 
#define USB_HSIC_HCFG		    USB_HSIC_BASE + 0x0400 
#define USB_HSIC_HFIR		    USB_HSIC_BASE + 0x0404 
#define USB_HSIC_HFNUM		    USB_HSIC_BASE + 0x0408 
#define USB_HSIC_HPTXSTS		USB_HSIC_BASE + 0x0410 
#define USB_HSIC_HAINT		    USB_HSIC_BASE + 0x0414 
#define USB_HSIC_HAINTMSK		USB_HSIC_BASE + 0x0418 
#define USB_HSIC_HPRT0		    USB_HSIC_BASE + 0x0440 
#define USB_HSIC_HCCHAR0		USB_HSIC_BASE + 0x0500 
#define USB_HSIC_HCSPLT0		USB_HSIC_BASE + 0x0504 
#define USB_HSIC_HCINT0		    USB_HSIC_BASE + 0x0508 
#define USB_HSIC_HCINTMSK0 		USB_HSIC_BASE + 0x050C 
#define USB_HSIC_HCTSIZ0		USB_HSIC_BASE + 0x0510 
#define USB_HSIC_HCDMA0		    USB_HSIC_BASE + 0x0514 
#define USB_HSIC_HCDMAB0	    USB_HSIC_BASE + 0x051C 
#define USB_HSIC_HCCHAR1		USB_HSIC_BASE + 0x0520 
#define USB_HSIC_HCSPLT1		USB_HSIC_BASE + 0x0524 
#define USB_HSIC_HCINT1		    USB_HSIC_BASE + 0x0528 
#define USB_HSIC_HCINTMSK1		USB_HSIC_BASE + 0x052C 
#define USB_HSIC_HCTSIZ1		USB_HSIC_BASE + 0x0530 
#define USB_HSIC_HCDMA1		    USB_HSIC_BASE + 0x0534 
#define USB_HSIC_HCDMAB1	    USB_HSIC_BASE + 0x053C 
#define USB_HSIC_HCCHAR2		USB_HSIC_BASE + 0x0540 
#define USB_HSIC_HCSPLT2		USB_HSIC_BASE + 0x0544 
#define USB_HSIC_HCINT2		    USB_HSIC_BASE + 0x0548 
#define USB_HSIC_HCINTMSK2		USB_HSIC_BASE + 0x054C 
#define USB_HSIC_HCTSIZ2		USB_HSIC_BASE + 0x0550 
#define USB_HSIC_HCDMA2		    USB_HSIC_BASE + 0x0554 
#define USB_HSIC_HCDMAB2	    USB_HSIC_BASE + 0x055C 
#define USB_HSIC_HCCHAR3		USB_HSIC_BASE + 0x0560 
#define USB_HSIC_HCSPLT3		USB_HSIC_BASE + 0x0564 
#define USB_HSIC_HCINT3			USB_HSIC_BASE + 0x0568 
#define USB_HSIC_HCINTMSK3		USB_HSIC_BASE + 0x056C 
#define USB_HSIC_HCTSIZ3		USB_HSIC_BASE + 0x0570 
#define USB_HSIC_HCDMA3			USB_HSIC_BASE + 0x0574 
#define USB_HSIC_HCDMAB3	    USB_HSIC_BASE + 0x057C 
#define USB_HSIC_HCCHAR4		USB_HSIC_BASE + 0x0580 
#define USB_HSIC_HCSPLT4		USB_HSIC_BASE + 0x0584 
#define USB_HSIC_HCINT4			USB_HSIC_BASE + 0x0588 
#define USB_HSIC_HCINTMSK4		USB_HSIC_BASE + 0x058C 
#define USB_HSIC_HCTSIZ4	    USB_HSIC_BASE + 0x0590 
#define USB_HSIC_HCDMA4			USB_HSIC_BASE + 0x0594 
#define USB_HSIC_HCDMAB4	    USB_HSIC_BASE + 0x059C 
#define USB_HSIC_HCCHAR5		USB_HSIC_BASE + 0x05A0 
#define USB_HSIC_HCSPLT5		USB_HSIC_BASE + 0x05A4 
#define USB_HSIC_HCINT5			USB_HSIC_BASE + 0x05A8 
#define USB_HSIC_HCINTMSK5		USB_HSIC_BASE + 0x05AC 
#define USB_HSIC_HCTSIZ5		USB_HSIC_BASE + 0x05B0 
#define USB_HSIC_HCDMA5		    USB_HSIC_BASE + 0x05B4 
#define USB_HSIC_HCDMAB5	    USB_HSIC_BASE + 0x05BC 
#define USB_HSIC_HCCHAR6		USB_HSIC_BASE + 0x05C0 
#define USB_HSIC_HCSPLT6		USB_HSIC_BASE + 0x05C4 
#define USB_HSIC_HCINT6		    USB_HSIC_BASE + 0x05C8 
#define USB_HSIC_HCINTMSK6 		USB_HSIC_BASE + 0x05CC 
#define USB_HSIC_HCTSIZ6		USB_HSIC_BASE + 0x05D0 
#define USB_HSIC_HCDMA6		    USB_HSIC_BASE + 0x05D4 
#define USB_HSIC_HCDMAB6	    USB_HSIC_BASE + 0x05DC 
#define USB_HSIC_HCCHAR7		USB_HSIC_BASE + 0x05E0 
#define USB_HSIC_HCSPLT7		USB_HSIC_BASE + 0x05E4 
#define USB_HSIC_HCINT7		    USB_HSIC_BASE + 0x05E8 
#define USB_HSIC_HCINTMSK7		USB_HSIC_BASE + 0x05EC 
#define USB_HSIC_HCTSIZ7		USB_HSIC_BASE + 0x05F0 
#define USB_HSIC_HCDMA7		    USB_HSIC_BASE + 0x05F4 
#define USB_HSIC_HCDMAB7	    USB_HSIC_BASE + 0x05FC 
#define USB_HSIC_HCCHAR8		USB_HSIC_BASE + 0x0600 
#define USB_HSIC_HCSPLT8		USB_HSIC_BASE + 0x0604 
#define USB_HSIC_HCINT8		    USB_HSIC_BASE + 0x0608 
#define USB_HSIC_HCINTMSK8		USB_HSIC_BASE + 0x060C 
#define USB_HSIC_HCTSIZ8		USB_HSIC_BASE + 0x0610 
#define USB_HSIC_HCDMA8		    USB_HSIC_BASE + 0x0614 
#define USB_HSIC_HCDMAB8	    USB_HSIC_BASE + 0x061C 
#define USB_HSIC_HCCHAR9		USB_HSIC_BASE + 0x0620 
#define USB_HSIC_HCSPLT9		USB_HSIC_BASE + 0x0624 
#define USB_HSIC_HCINT9		    USB_HSIC_BASE + 0x0628 
#define USB_HSIC_HCINTMSK9		USB_HSIC_BASE + 0x062C 
#define USB_HSIC_HCTSIZ9		USB_HSIC_BASE + 0x0630 
#define USB_HSIC_HCDMA9		    USB_HSIC_BASE + 0x0634 
#define USB_HSIC_HCDMAB9	    USB_HSIC_BASE + 0x063C 
#define USB_HSIC_HCCHAR10		USB_HSIC_BASE + 0x0640 
#define USB_HSIC_HCSPLT10		USB_HSIC_BASE + 0x0644 
#define USB_HSIC_HCINT10	    USB_HSIC_BASE + 0x0648 
#define USB_HSIC_HCINTMSK10		USB_HSIC_BASE + 0x064C 
#define USB_HSIC_HCTSIZ10		USB_HSIC_BASE + 0x0650 
#define USB_HSIC_HCDMA10	    USB_HSIC_BASE + 0x0654 
#define USB_HSIC_HCDMAB10	    USB_HSIC_BASE + 0x065C 
#define USB_HSIC_HCCHAR11		USB_HSIC_BASE + 0x0660 
#define USB_HSIC_HCSPLT11		USB_HSIC_BASE + 0x0664 
#define USB_HSIC_HCINT11	    USB_HSIC_BASE + 0x0668 
#define USB_HSIC_HCINTMSK11		USB_HSIC_BASE + 0x066C 
#define USB_HSIC_HCTSIZ11		USB_HSIC_BASE + 0x0670 
#define USB_HSIC_HCDMA11	    USB_HSIC_BASE + 0x0674 
#define USB_HSIC_HCDMAB11	    USB_HSIC_BASE + 0x067C 
#define USB_HSIC_HCCHAR12		USB_HSIC_BASE + 0x0680 
#define USB_HSIC_HCSPLT12		USB_HSIC_BASE + 0x0684 
#define USB_HSIC_HCINT12	    USB_HSIC_BASE + 0x0688 
#define USB_HSIC_HCINTMSK12		USB_HSIC_BASE + 0x068C 
#define USB_HSIC_HCTSIZ12		USB_HSIC_BASE + 0x0690 
#define USB_HSIC_HCDMA12	    USB_HSIC_BASE + 0x0694 
#define USB_HSIC_HCDMAB12	    USB_HSIC_BASE + 0x069C 
#define USB_HSIC_HCCHAR13		USB_HSIC_BASE + 0x06A0 
#define USB_HSIC_HCSPLT13		USB_HSIC_BASE + 0x06A4 
#define USB_HSIC_HCINT13	    USB_HSIC_BASE + 0x06A8 
#define USB_HSIC_HCINTMSK13		USB_HSIC_BASE + 0x06AC 
#define USB_HSIC_HCTSIZ13		USB_HSIC_BASE + 0x06B0 
#define USB_HSIC_HCDMA13	    USB_HSIC_BASE + 0x06B4 
#define USB_HSIC_HCDMAB13	    USB_HSIC_BASE + 0x06BC 
#define USB_HSIC_HCCHAR14		USB_HSIC_BASE + 0x06C0 
#define USB_HSIC_HCSPLT14		USB_HSIC_BASE + 0x06C4 
#define USB_HSIC_HCINT14	    USB_HSIC_BASE + 0x06C8 
#define USB_HSIC_HCINTMSK14		USB_HSIC_BASE + 0x06CC 
#define USB_HSIC_HCTSIZ14		USB_HSIC_BASE + 0x06D0 
#define USB_HSIC_HCDMA14	    USB_HSIC_BASE + 0x06D4 
#define USB_HSIC_HCDMAB14	    USB_HSIC_BASE + 0x06DC 
#define USB_HSIC_HCCHAR15		USB_HSIC_BASE + 0x06E0 
#define USB_HSIC_HCSPLT15		USB_HSIC_BASE + 0x06E4 
#define USB_HSIC_HCINT15	    USB_HSIC_BASE + 0x06E8 
#define USB_HSIC_HCINTMSK15		USB_HSIC_BASE + 0x06EC 
#define USB_HSIC_HCTSIZ15		USB_HSIC_BASE + 0x06F0 
#define USB_HSIC_HCDMA15	    USB_HSIC_BASE + 0x06F4 
#define USB_HSIC_HCDMAB15	    USB_HSIC_BASE + 0x06FC 
#define USB_HSIC_PCGCCTL        USB_HSIC_BASE + 0x0E00 
//internal data fifo
#define USB_HSIC_RXFIFO_CH0        	 USB_HSIC_BASE + 0x1000 
#define USB_HSIC_PTXFIFO_CH0         USB_HSIC_BASE + 0x1000 
#define USB_HSIC_NPTXFIFO_CH0        USB_HSIC_BASE + 0x1000 
#define USB_HSIC_RXFIFO_CH1        	 USB_HSIC_BASE + 0x2000 
#define USB_HSIC_PTXFIFO_CH1         USB_HSIC_BASE + 0x2000 
#define USB_HSIC_NPTXFIFO_CH1        USB_HSIC_BASE + 0x2000 
#define USB_HSIC_RXFIFO_CH2        	 USB_HSIC_BASE + 0x3000 
#define USB_HSIC_PTXFIFO_CH2         USB_HSIC_BASE + 0x3000 
#define USB_HSIC_NPTXFIFO_CH2        USB_HSIC_BASE + 0x3000 
#define USB_HSIC_RXFIFO_CH3        	 USB_HSIC_BASE + 0x4000 
#define USB_HSIC_PTXFIFO_CH3         USB_HSIC_BASE + 0x4000 
#define USB_HSIC_NPTXFIFO_CH3        USB_HSIC_BASE + 0x4000 
#define USB_HSIC_RXFIFO_CH4        	 USB_HSIC_BASE + 0x5000 
#define USB_HSIC_PTXFIFO_CH4         USB_HSIC_BASE + 0x5000 
#define USB_HSIC_NPTXFIFO_CH4        USB_HSIC_BASE + 0x5000 
#define USB_HSIC_RXFIFO_CH5        	 USB_HSIC_BASE + 0x6000 
#define USB_HSIC_PTXFIFO_CH5         USB_HSIC_BASE + 0x6000 
#define USB_HSIC_NPTXFIFO_CH5        USB_HSIC_BASE + 0x6000 
#define USB_HSIC_RXFIFO_CH6        	 USB_HSIC_BASE + 0x7000 
#define USB_HSIC_PTXFIFO_CH6         USB_HSIC_BASE + 0x7000 
#define USB_HSIC_NPTXFIFO_CH6        USB_HSIC_BASE + 0x7000 
#define USB_HSIC_RXFIFO_CH7        	 USB_HSIC_BASE + 0x8000 
#define USB_HSIC_PTXFIFO_CH7         USB_HSIC_BASE + 0x8000 
#define USB_HSIC_NPTXFIFO_CH7        USB_HSIC_BASE + 0x8000 
#define USB_HSIC_RXFIFO_CH8        	 USB_HSIC_BASE + 0x9000 
#define USB_HSIC_PTXFIFO_CH8         USB_HSIC_BASE + 0x9000 
#define USB_HSIC_NPTXFIFO_CH8        USB_HSIC_BASE + 0x9000 
#define USB_HSIC_RXFIFO_CH9        	 USB_HSIC_BASE + 0xa000 
#define USB_HSIC_PTXFIFO_CH9         USB_HSIC_BASE + 0xa000 
#define USB_HSIC_NPTXFIFO_CH9        USB_HSIC_BASE + 0xa000 
#define USB_HSIC_RXFIFO_CH10         USB_HSIC_BASE + 0xb000 
#define USB_HSIC_PTXFIFO_CH10        USB_HSIC_BASE + 0xb000 
#define USB_HSIC_NPTXFIFO_CH10       USB_HSIC_BASE + 0xb000 
#define USB_HSIC_RXFIFO_CH11         USB_HSIC_BASE + 0xc000 
#define USB_HSIC_PTXFIFO_CH11        USB_HSIC_BASE + 0xc000 
#define USB_HSIC_NPTXFIFO_CH11       USB_HSIC_BASE + 0xc000 
#define USB_HSIC_RXFIFO_CH12         USB_HSIC_BASE + 0xd000 
#define USB_HSIC_PTXFIFO_CH12        USB_HSIC_BASE + 0xd000 
#define USB_HSIC_NPTXFIFO_CH12       USB_HSIC_BASE + 0xd000 
#define USB_HSIC_RXFIFO_CH13         USB_HSIC_BASE + 0xe000 
#define USB_HSIC_PTXFIFO_CH13        USB_HSIC_BASE + 0xe000 
#define USB_HSIC_NPTXFIFO_CH13       USB_HSIC_BASE + 0xe000 
#define USB_HSIC_RXFIFO_CH14         USB_HSIC_BASE + 0xf000 
#define USB_HSIC_PTXFIFO_CH14        USB_HSIC_BASE + 0xf000 
#define USB_HSIC_NPTXFIFO_CH14       USB_HSIC_BASE + 0xf000 
#define USB_HSIC_RXFIFO_CH15         USB_HSIC_BASE + 0x10000
#define USB_HSIC_PTXFIFO_CH15        USB_HSIC_BASE + 0x10000
#define USB_HSIC_NPTXFIFO_CH15       USB_HSIC_BASE + 0x10000


#ifndef CP_HEADER
#define		A7_PRIVATE_CPUCORE		AP_A7_GIC_BASE
#else
#define		A7_PRIVATE_CPUCORE		CP_A7_GIC_BASE
#endif

#define		ICDDCR_BASEADDR			(A7_PRIVATE_CPUCORE + 0x1000)
#define		ICCICR_BASEADDR			(A7_PRIVATE_CPUCORE + 0x2000)
#define		PRITIMERWDT_BASEADDR	(A7_PRIVATE_CPUCORE + 0x600)

#define		ICDDCR					ICDDCR_BASEADDR + 0x00 
#define		ICDICTR					ICDDCR_BASEADDR + 0x04 
#define		ICDIIDR					ICDDCR_BASEADDR + 0x08 
#define		ICDISR					ICDDCR_BASEADDR + 0x80 
#define		ICDISR0					ICDDCR_BASEADDR + 0x80 
#define		ICDISR1					ICDDCR_BASEADDR + 0x84 
#define		ICDISR2					ICDDCR_BASEADDR + 0x88 
#define		ICDISER					ICDDCR_BASEADDR + 0x100 
#define		ICDISER0				ICDDCR_BASEADDR + 0x100 
#define		ICDISER1				ICDDCR_BASEADDR + 0x104 
#define		ICDISER2				ICDDCR_BASEADDR + 0x108 
#define		ICDISER3				ICDDCR_BASEADDR + 0x10C 
#define		ICDICER					ICDDCR_BASEADDR + 0x180 
#define		ICDISPR					ICDDCR_BASEADDR + 0x200 
#define		ICDICPR					ICDDCR_BASEADDR + 0x280 
#define		ICDABR					ICDDCR_BASEADDR + 0x300 
#define		ICDIPR					ICDDCR_BASEADDR + 0x400 
#define		ICDIPR0					ICDDCR_BASEADDR + 0x400 
#define		ICDIPR1					ICDDCR_BASEADDR + 0x404 
#define		ICDIPR2					ICDDCR_BASEADDR + 0x408 
#define		ICDIPR3					ICDDCR_BASEADDR + 0x40C 
#define		ICDIPR4					ICDDCR_BASEADDR + 0x410 
#define		ICDIPR5					ICDDCR_BASEADDR + 0x414 
#define		ICDIPR6					ICDDCR_BASEADDR + 0x418 
#define		ICDIPR7					ICDDCR_BASEADDR + 0x41C 
#define		ICDIPR8					ICDDCR_BASEADDR + 0x420 
#define		ICDIPR9					ICDDCR_BASEADDR + 0x424 
#define		ICDIPR10				ICDDCR_BASEADDR + 0x428 
#define		ICDIPR11				ICDDCR_BASEADDR + 0x42C 
#define		ICDIPR12				ICDDCR_BASEADDR + 0x430 
#define		ICDIPR13				ICDDCR_BASEADDR + 0x434 
#define		ICDIPR14				ICDDCR_BASEADDR + 0x438 
#define		ICDIPR15				ICDDCR_BASEADDR + 0x43C 
#define		ICDIPR16				ICDDCR_BASEADDR + 0x440 
#define		ICDIPR17				ICDDCR_BASEADDR + 0x444 
#define		ICDIPR18				ICDDCR_BASEADDR + 0x448 
#define		ICDIPR19				ICDDCR_BASEADDR + 0x44C 
#define		ICDIPR20				ICDDCR_BASEADDR + 0x450 
#define		ICDIPR21				ICDDCR_BASEADDR + 0x454 
#define		ICDIPR22				ICDDCR_BASEADDR + 0x458 
#define		ICDIPR23				ICDDCR_BASEADDR + 0x45C 

#define		ICDIPTR					ICDDCR_BASEADDR + 0x800 
#define		ICDIPTR0				ICDDCR_BASEADDR + 0x800 
#define		ICDIPTR1				ICDDCR_BASEADDR + 0x804 
#define		ICDIPTR2				ICDDCR_BASEADDR + 0x808 
#define		ICDIPTR3				ICDDCR_BASEADDR + 0x80c 
#define		ICDIPTR4				ICDDCR_BASEADDR + 0x810 
#define		ICDIPTR5				ICDDCR_BASEADDR + 0x814 
#define		ICDIPTR6				ICDDCR_BASEADDR + 0x818 
#define		ICDIPTR7				ICDDCR_BASEADDR + 0x81c 
#define		ICDIPTR8				ICDDCR_BASEADDR + 0x820 
#define		ICDIPTR9				ICDDCR_BASEADDR + 0x824 
#define		ICDIPTR10				ICDDCR_BASEADDR + 0x828 
#define		ICDIPTR11				ICDDCR_BASEADDR + 0x82C 
#define		ICDIPTR12				ICDDCR_BASEADDR + 0x830 
#define		ICDIPTR13				ICDDCR_BASEADDR + 0x834 
#define		ICDIPTR14				ICDDCR_BASEADDR + 0x838 
#define		ICDIPTR15				ICDDCR_BASEADDR + 0x83C 
#define		ICDIPTR16				ICDDCR_BASEADDR + 0x840 
#define		ICDIPTR17				ICDDCR_BASEADDR + 0x844 
#define		ICDIPTR18				ICDDCR_BASEADDR + 0x848 
#define		ICDIPTR19				ICDDCR_BASEADDR + 0x84C 
#define		ICDIPTR20				ICDDCR_BASEADDR + 0x850 
#define		ICDIPTR21				ICDDCR_BASEADDR + 0x854 
#define		ICDIPTR22				ICDDCR_BASEADDR + 0x858 

#define		ICDICFR					ICDDCR_BASEADDR + 0xc00 
#define		ICDICFR0				ICDDCR_BASEADDR + 0xc00 
#define		ICDICFR1				ICDDCR_BASEADDR + 0xc04 
#define		ICDICFR2				ICDDCR_BASEADDR + 0xc08 
#define		ICDICFR3				ICDDCR_BASEADDR + 0xc0c 
#define		ICDICFR4				ICDDCR_BASEADDR + 0xc10 
#define		ICDICFR5				ICDDCR_BASEADDR + 0xc14 
#define		ICDICFR6				ICDDCR_BASEADDR + 0xc18 
#define		ICDICFR7				ICDDCR_BASEADDR + 0xc1C 

#define		ICDSGIR					ICDDCR_BASEADDR + 0xf00 

#define		ICCICR					ICCICR_BASEADDR + 0x00 
#define		ICCPMR					ICCICR_BASEADDR + 0x04 
#define		ICCBPR					ICCICR_BASEADDR + 0x08 
#define		ICCIAR					ICCICR_BASEADDR + 0x0c 
#define		ICCEOIR					ICCICR_BASEADDR + 0x10 
#define		ICCRPR					ICCICR_BASEADDR + 0x14 
#define		ICCHPIR					ICCICR_BASEADDR + 0x18 
#define		ICCABPR					ICCICR_BASEADDR + 0x1c 
#define		ICCIIDR					ICCICR_BASEADDR + 0xfc 


#define		PRITIMERLOAD			PRITIMERWDT_BASEADDR + 0x00 
#define		PRITIMERCNT				PRITIMERWDT_BASEADDR + 0x04 
#define		PRITIMERCTL				PRITIMERWDT_BASEADDR + 0x08 
#define		PRITIMERINTSTATUS		PRITIMERWDT_BASEADDR + 0x0c 
#define		PRIWDTLOAD				PRITIMERWDT_BASEADDR + 0x20 
#define		PRIWDTCNT				PRITIMERWDT_BASEADDR + 0x24 
#define		PRIWDTCTL				PRITIMERWDT_BASEADDR + 0x28 
#define		PRIWDTINTSTATUS			PRITIMERWDT_BASEADDR + 0x2c 
#define		PRIWDTRSTSTATUS			PRITIMERWDT_BASEADDR + 0x30 
#define		PRIWDTDISABLE			PRITIMERWDT_BASEADDR + 0x34 



#define   ap_ispreg                 ISP_BASE + 0x63023
#define   REG_ISP_SOFT_STANDBY      ISP_BASE + 0x60100

#endif

