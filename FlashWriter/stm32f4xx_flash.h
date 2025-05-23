/**
  ******************************************************************************
  * @file    stm32f4xx_flash.h
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file contains all the functions prototypes for the FLASH 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_FLASH_H
#define __STM32F4xx_FLASH_H
 
#ifdef __cplusplus
 extern "C" {
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
 
/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */
 
/** @addtogroup FLASH
  * @{
  */ 
 
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief FLASH Status  
  */ 
typedef enum
{ 
  FLASH_BUSY2 = 1,
  FLASH_ERROR_RD2,
  FLASH_ERROR_PGS2,
  FLASH_ERROR_PGP2,
  FLASH_ERROR_PGA2,
  FLASH_ERROR_WRP2,
  FLASH_ERROR_PROGRAM2,
  FLASH_ERROR_OPERATION2,
  FLASH_COMPLETE2
}FLASH_Status;
 
/* Exported constants --------------------------------------------------------*/
 
/** @defgroup FLASH_Exported_Constants
  * @{
  */  
 
/** @defgroup Flash_Latency 
  * @{
  */ 
#define FLASH_Latency_0                ((uint8_t)0x0000)  /*!< FLASH Zero Latency cycle      */
#define FLASH_Latency_1                ((uint8_t)0x0001)  /*!< FLASH One Latency cycle       */
#define FLASH_Latency_2                ((uint8_t)0x0002)  /*!< FLASH Two Latency cycles      */
#define FLASH_Latency_3                ((uint8_t)0x0003)  /*!< FLASH Three Latency cycles    */
#define FLASH_Latency_4                ((uint8_t)0x0004)  /*!< FLASH Four Latency cycles     */
#define FLASH_Latency_5                ((uint8_t)0x0005)  /*!< FLASH Five Latency cycles     */
#define FLASH_Latency_6                ((uint8_t)0x0006)  /*!< FLASH Six Latency cycles      */
#define FLASH_Latency_7                ((uint8_t)0x0007)  /*!< FLASH Seven Latency cycles    */
#define FLASH_Latency_8                ((uint8_t)0x0008)  /*!< FLASH Eight Latency cycles    */
#define FLASH_Latency_9                ((uint8_t)0x0009)  /*!< FLASH Nine Latency cycles     */
#define FLASH_Latency_10               ((uint8_t)0x000A)  /*!< FLASH Ten Latency cycles      */
#define FLASH_Latency_11               ((uint8_t)0x000B)  /*!< FLASH Eleven Latency cycles   */
#define FLASH_Latency_12               ((uint8_t)0x000C)  /*!< FLASH Twelve Latency cycles   */
#define FLASH_Latency_13               ((uint8_t)0x000D)  /*!< FLASH Thirteen Latency cycles */
#define FLASH_Latency_14               ((uint8_t)0x000E)  /*!< FLASH Fourteen Latency cycles */
#define FLASH_Latency_15               ((uint8_t)0x000F)  /*!< FLASH Fifteen Latency cycles  */
 
/**
  * @}
  */ 
 
/** @defgroup FLASH_Voltage_Range 
  * @{
  */ 
#define VoltageRange_1        ((uint8_t)0x00)  /*!< Device operating range: 1.8V to 2.1V */
#define VoltageRange_2        ((uint8_t)0x01)  /*!<Device operating range: 2.1V to 2.7V */
#define VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */
#define VoltageRange_4        ((uint8_t)0x03)  /*!<Device operating range: 2.7V to 3.6V + External Vpp */
 
/*
#define IS_VOLTAGERANGE(RANGE)(((RANGE) == VoltageRange_1) || \
                               ((RANGE) == VoltageRange_2) || \
                               ((RANGE) == VoltageRange_3) || \
                               ((RANGE) == VoltageRange_4))
*/

/**
  * @}
  */ 
 
/** @defgroup FLASH_Sectors
  * @{
  */
#define FLASH_Sector_0     ((uint16_t)0x0000) /*!< Sector Number 0   */
#define FLASH_Sector_1     ((uint16_t)0x0008) /*!< Sector Number 1   */
#define FLASH_Sector_2     ((uint16_t)0x0010) /*!< Sector Number 2   */
#define FLASH_Sector_3     ((uint16_t)0x0018) /*!< Sector Number 3   */
#define FLASH_Sector_4     ((uint16_t)0x0020) /*!< Sector Number 4   */
#define FLASH_Sector_5     ((uint16_t)0x0028) /*!< Sector Number 5   */
#define FLASH_Sector_6     ((uint16_t)0x0030) /*!< Sector Number 6   */
#define FLASH_Sector_7     ((uint16_t)0x0038) /*!< Sector Number 7   */
#define FLASH_Sector_8     ((uint16_t)0x0040) /*!< Sector Number 8   */
#define FLASH_Sector_9     ((uint16_t)0x0048) /*!< Sector Number 9   */
#define FLASH_Sector_10    ((uint16_t)0x0050) /*!< Sector Number 10  */
#define FLASH_Sector_11    ((uint16_t)0x0058) /*!< Sector Number 11  */
#define FLASH_Sector_12    ((uint16_t)0x0080) /*!< Sector Number 12  */
#define FLASH_Sector_13    ((uint16_t)0x0088) /*!< Sector Number 13  */
#define FLASH_Sector_14    ((uint16_t)0x0090) /*!< Sector Number 14  */
#define FLASH_Sector_15    ((uint16_t)0x0098) /*!< Sector Number 15  */
#define FLASH_Sector_16    ((uint16_t)0x00A0) /*!< Sector Number 16  */
#define FLASH_Sector_17    ((uint16_t)0x00A8) /*!< Sector Number 17  */
#define FLASH_Sector_18    ((uint16_t)0x00B0) /*!< Sector Number 18  */
#define FLASH_Sector_19    ((uint16_t)0x00B8) /*!< Sector Number 19  */
#define FLASH_Sector_20    ((uint16_t)0x00C0) /*!< Sector Number 20  */
#define FLASH_Sector_21    ((uint16_t)0x00C8) /*!< Sector Number 21  */
#define FLASH_Sector_22    ((uint16_t)0x00D0) /*!< Sector Number 22  */
#define FLASH_Sector_23    ((uint16_t)0x00D8) /*!< Sector Number 23  */
 
#if defined (STM32F427_437xx) || defined (STM32F429_439xx) || defined (STM32F469_479xx)
#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= 0x08000000) && ((ADDRESS) <= 0x081FFFFF)) ||\
                                   (((ADDRESS) >= 0x1FFF7800) && ((ADDRESS) <= 0x1FFF7A0F)))  
#endif /* STM32F427_437xx || STM32F429_439xx || STM32F469_479xx */
 
#if defined (STM32F40_41xxx) || defined(STM32F412xG)
#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= 0x08000000) && ((ADDRESS) <= 0x080FFFFF)) ||\
                                   (((ADDRESS) >= 0x1FFF7800) && ((ADDRESS) <= 0x1FFF7A0F))) 
#endif /* STM32F40_41xxx || STM32F412xG */
 
#if defined (STM32F401xx)
#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= 0x08000000) && ((ADDRESS) <= 0x0803FFFF)) ||\
                                   (((ADDRESS) >= 0x1FFF7800) && ((ADDRESS) <= 0x1FFF7A0F)))
#endif /* STM32F401xx */
 
#if defined (STM32F411xE) || defined (STM32F446xx)
//#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= 0x08000000) && ((ADDRESS) <= 0x0807FFFF)) ||\
//                                   (((ADDRESS) >= 0x1FFF7800) && ((ADDRESS) <= 0x1FFF7A0F)))
#endif /* STM32F411xE || STM32F446xx */
 
#if defined (STM32F410xx)
#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= 0x08000000) && ((ADDRESS) <= 0x0801FFFF)) ||\
                                   (((ADDRESS) >= 0x1FFF7800) && ((ADDRESS) <= 0x1FFF7A0F)))
#endif /* STM32F410xx */
 
/**
  * @}
  */ 
 
/** @defgroup Option_Bytes_Write_Protection 
  * @{
  */ 
#define OB_WRP_Sector_0       ((uint32_t)0x00000001) /*!< Write protection of Sector0     */
#define OB_WRP_Sector_1       ((uint32_t)0x00000002) /*!< Write protection of Sector1     */
#define OB_WRP_Sector_2       ((uint32_t)0x00000004) /*!< Write protection of Sector2     */
#define OB_WRP_Sector_3       ((uint32_t)0x00000008) /*!< Write protection of Sector3     */
#define OB_WRP_Sector_4       ((uint32_t)0x00000010) /*!< Write protection of Sector4     */
#define OB_WRP_Sector_5       ((uint32_t)0x00000020) /*!< Write protection of Sector5     */
#define OB_WRP_Sector_6       ((uint32_t)0x00000040) /*!< Write protection of Sector6     */
#define OB_WRP_Sector_7       ((uint32_t)0x00000080) /*!< Write protection of Sector7     */
#define OB_WRP_Sector_8       ((uint32_t)0x00000100) /*!< Write protection of Sector8     */
#define OB_WRP_Sector_9       ((uint32_t)0x00000200) /*!< Write protection of Sector9     */
#define OB_WRP_Sector_10      ((uint32_t)0x00000400) /*!< Write protection of Sector10    */
#define OB_WRP_Sector_11      ((uint32_t)0x00000800) /*!< Write protection of Sector11    */
#define OB_WRP_Sector_12      ((uint32_t)0x00000001) /*!< Write protection of Sector12    */
#define OB_WRP_Sector_13      ((uint32_t)0x00000002) /*!< Write protection of Sector13    */
#define OB_WRP_Sector_14      ((uint32_t)0x00000004) /*!< Write protection of Sector14    */
#define OB_WRP_Sector_15      ((uint32_t)0x00000008) /*!< Write protection of Sector15    */
#define OB_WRP_Sector_16      ((uint32_t)0x00000010) /*!< Write protection of Sector16    */
#define OB_WRP_Sector_17      ((uint32_t)0x00000020) /*!< Write protection of Sector17    */
#define OB_WRP_Sector_18      ((uint32_t)0x00000040) /*!< Write protection of Sector18    */
#define OB_WRP_Sector_19      ((uint32_t)0x00000080) /*!< Write protection of Sector19    */
#define OB_WRP_Sector_20      ((uint32_t)0x00000100) /*!< Write protection of Sector20    */
#define OB_WRP_Sector_21      ((uint32_t)0x00000200) /*!< Write protection of Sector21    */
#define OB_WRP_Sector_22      ((uint32_t)0x00000400) /*!< Write protection of Sector22    */
#define OB_WRP_Sector_23      ((uint32_t)0x00000800) /*!< Write protection of Sector23    */
#define OB_WRP_Sector_All     ((uint32_t)0x00000FFF) /*!< Write protection of all Sectors */
 
#define IS_OB_WRP(SECTOR)((((SECTOR) & (uint32_t)0xFFFFF000) == 0x00000000) && ((SECTOR) != 0x00000000))
/**
  * @}
  */
 
/** @defgroup  Selection_Protection_Mode
  * @{
  */
#define OB_PcROP_Disable   ((uint8_t)0x00) /*!< Disabled PcROP, nWPRi bits used for Write Protection on sector i */
#define OB_PcROP_Enable    ((uint8_t)0x80) /*!< Enable PcROP, nWPRi bits used for PCRoP Protection on sector i   */

/**
  * @}
  */
 
/** @defgroup Option_Bytes_PC_ReadWrite_Protection 
  * @{
  */ 
#define OB_PCROP_Sector_0        ((uint32_t)0x00000001) /*!< PC Read/Write protection of Sector0      */
#define OB_PCROP_Sector_1        ((uint32_t)0x00000002) /*!< PC Read/Write protection of Sector1      */
#define OB_PCROP_Sector_2        ((uint32_t)0x00000004) /*!< PC Read/Write protection of Sector2      */
#define OB_PCROP_Sector_3        ((uint32_t)0x00000008) /*!< PC Read/Write protection of Sector3      */
#define OB_PCROP_Sector_4        ((uint32_t)0x00000010) /*!< PC Read/Write protection of Sector4      */
#define OB_PCROP_Sector_5        ((uint32_t)0x00000020) /*!< PC Read/Write protection of Sector5      */
#define OB_PCROP_Sector_6        ((uint32_t)0x00000040) /*!< PC Read/Write protection of Sector6      */
#define OB_PCROP_Sector_7        ((uint32_t)0x00000080) /*!< PC Read/Write protection of Sector7      */
#define OB_PCROP_Sector_8        ((uint32_t)0x00000100) /*!< PC Read/Write protection of Sector8      */
#define OB_PCROP_Sector_9        ((uint32_t)0x00000200) /*!< PC Read/Write protection of Sector9      */
#define OB_PCROP_Sector_10       ((uint32_t)0x00000400) /*!< PC Read/Write protection of Sector10     */
#define OB_PCROP_Sector_11       ((uint32_t)0x00000800) /*!< PC Read/Write protection of Sector11     */
#define OB_PCROP_Sector_12       ((uint32_t)0x00000001) /*!< PC Read/Write protection of Sector12     */
#define OB_PCROP_Sector_13       ((uint32_t)0x00000002) /*!< PC Read/Write protection of Sector13     */
#define OB_PCROP_Sector_14       ((uint32_t)0x00000004) /*!< PC Read/Write protection of Sector14     */
#define OB_PCROP_Sector_15       ((uint32_t)0x00000008) /*!< PC Read/Write protection of Sector15     */
#define OB_PCROP_Sector_16       ((uint32_t)0x00000010) /*!< PC Read/Write protection of Sector16     */
#define OB_PCROP_Sector_17       ((uint32_t)0x00000020) /*!< PC Read/Write protection of Sector17     */
#define OB_PCROP_Sector_18       ((uint32_t)0x00000040) /*!< PC Read/Write protection of Sector18     */
#define OB_PCROP_Sector_19       ((uint32_t)0x00000080) /*!< PC Read/Write protection of Sector19     */
#define OB_PCROP_Sector_20       ((uint32_t)0x00000100) /*!< PC Read/Write protection of Sector20     */
#define OB_PCROP_Sector_21       ((uint32_t)0x00000200) /*!< PC Read/Write protection of Sector21     */
#define OB_PCROP_Sector_22       ((uint32_t)0x00000400) /*!< PC Read/Write protection of Sector22     */
#define OB_PCROP_Sector_23       ((uint32_t)0x00000800) /*!< PC Read/Write protection of Sector23     */
#define OB_PCROP_Sector_All      ((uint32_t)0x00000FFF) /*!< PC Read/Write protection of all Sectors  */
 
/**
  * @}
  */
 
/** @defgroup FLASH_Option_Bytes_Read_Protection 
  * @{
  */
#define OB_RDP_Level_0   ((uint8_t)0xAA)
#define OB_RDP_Level_1   ((uint8_t)0x55)
/*#define OB_RDP_Level_2   ((uint8_t)0xCC)*/ /*!< Warning: When enabling read protection level 2 
                                                  it's no more possible to go back to level 1 or 0 */
#define IS_OB_RDP(LEVEL) (((LEVEL) == OB_RDP_Level_0)||\
                          ((LEVEL) == OB_RDP_Level_1))/*||\
                          ((LEVEL) == OB_RDP_Level_2))*/
/**
  * @}
  */ 
 
/** @defgroup FLASH_Option_Bytes_IWatchdog 
  * @{
  */ 

#define IS_OB_IWDG_SOURCE(SOURCE) (((SOURCE) == OB_IWDG_SW) || ((SOURCE) == OB_IWDG_HW))
/**
  * @}
  */ 
 
/** @defgroup FLASH_Option_Bytes_nRST_STOP 
  * @{
  */ 
#define OB_STOP_NoRST                  ((uint8_t)0x40) /*!< No reset generated when entering in STOP */

/**
  * @}
  */ 
 
 
/** @defgroup FLASH_Option_Bytes_nRST_STDBY 
  * @{
  */ 
#define OB_STDBY_NoRST                 ((uint8_t)0x80) /*!< No reset generated when entering in STANDBY */
//#define OB_STDBY_RST                   ((uint8_t)0x00) /*!< Reset generated when entering in STANDBY */
//#define IS_OB_STDBY_SOURCE(SOURCE) (((SOURCE) == OB_STDBY_NoRST) || ((SOURCE) == OB_STDBY_RST))
/**
  * @}
  */
  
/** @defgroup FLASH_BOR_Reset_Level 
  * @{
  */  

#define IS_OB_BOR(LEVEL) (((LEVEL) == OB_BOR_LEVEL1) || ((LEVEL) == OB_BOR_LEVEL2) ||\
                          ((LEVEL) == OB_BOR_LEVEL3) || ((LEVEL) == OB_BOR_OFF))
/**
  * @}
  */
  
/** @defgroup FLASH_Dual_Boot
  * @{
  */
#define OB_Dual_BootEnabled   ((uint8_t)0x10) /*!< Dual Bank Boot Enable                             */
#define OB_Dual_BootDisabled  ((uint8_t)0x00) /*!< Dual Bank Boot Disable, always boot on User Flash */
#define IS_OB_BOOT(BOOT) (((BOOT) == OB_Dual_BootEnabled) || ((BOOT) == OB_Dual_BootDisabled))
/**
  * @}
  */
 
/** @defgroup FLASH_Interrupts 
  * @{
  */ 

#define IS_FLASH_IT(IT) ((((IT) & (uint32_t)0xFCFFFFFF) == 0x00000000) && ((IT) != 0x00000000))
/**
  * @}
  */ 
 
/** @defgroup FLASH_Flags 
  * @{
  */ 

#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFE0C) == 0x00000000) && ((FLAG) != 0x00000000))
#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_EOP)    || ((FLAG) == FLASH_FLAG_OPERR)  || \
                                  ((FLAG) == FLASH_FLAG_WRPERR) || ((FLAG) == FLASH_FLAG_PGAERR) || \
                                  ((FLAG) == FLASH_FLAG_PGPERR) || ((FLAG) == FLASH_FLAG_PGSERR) || \
                                  ((FLAG) == FLASH_FLAG_BSY)    || ((FLAG) == FLASH_FLAG_RDERR))
 
/** 
  * @brief   ACR register byte 0 (Bits[7:0]) base address  
  */ 
//#define ACR_BYTE0_ADDRESS           ((uint32_t)0x40023C00) 
/** 
  * @brief   OPTCR register byte 0 (Bits[7:0]) base address  
  */ 
//#define OPTCR_BYTE0_ADDRESS         ((uint32_t)0x40023C14)
/** 
  * @brief   OPTCR register byte 1 (Bits[15:8]) base address  
  */ 
//#define OPTCR_BYTE1_ADDRESS         ((uint32_t)0x40023C15)
/** 
  * @brief   OPTCR register byte 2 (Bits[23:16]) base address  
  */ 
//#define OPTCR_BYTE2_ADDRESS         ((uint32_t)0x40023C16)
/** 
  * @brief   OPTCR register byte 3 (Bits[31:24]) base address  
  */ 
//#define OPTCR_BYTE3_ADDRESS         ((uint32_t)0x40023C17)
 
/** 
  * @brief   OPTCR1 register byte 0 (Bits[7:0]) base address  
  */ 
#define OPTCR1_BYTE2_ADDRESS         ((uint32_t)0x40023C1A)
 
/**
  * @}
  */ 
 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
 
/* FLASH Interface configuration functions ************************************/
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
void FLASH_InstructionCacheCmd(FunctionalState NewState);
void FLASH_DataCacheCmd(FunctionalState NewState);
void FLASH_InstructionCacheReset(void);
void FLASH_DataCacheReset(void);
 
/* FLASH Memory Programming functions *****************************************/   
void         FLASH_Unlock(void);
void         FLASH_Lock(void);
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllBank1Sectors(uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllBank2Sectors(uint8_t VoltageRange);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data);
 
/* Option Bytes Programming functions *****************************************/ 
void         FLASH_OB_Unlock(void);
void         FLASH_OB_Lock(void);
void         FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
void         FLASH_OB_WRP1Config(uint32_t OB_WRP, FunctionalState NewState);
void         FLASH_OB_PCROPSelectionConfig(uint8_t OB_PcROP);
void         FLASH_OB_PCROPConfig(uint32_t OB_PCROP, FunctionalState NewState);
void         FLASH_OB_PCROP1Config(uint32_t OB_PCROP, FunctionalState NewState);
void         FLASH_OB_RDPConfig(uint8_t OB_RDP);
void         FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
void         FLASH_OB_BORConfig(uint8_t OB_BOR);
void         FLASH_OB_BootConfig(uint8_t OB_BOOT);
FLASH_Status FLASH_OB_Launch(void);
uint8_t      FLASH_OB_GetUser(void);
uint16_t     FLASH_OB_GetWRP(void);
uint16_t     FLASH_OB_GetWRP1(void);
uint16_t     FLASH_OB_GetPCROP(void);
uint16_t     FLASH_OB_GetPCROP1(void);
FlagStatus   FLASH_OB_GetRDP(void);
uint8_t      FLASH_OB_GetBOR(void);
 
/* Interrupts and flags management functions **********************************/
void         FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus   FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void         FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation2(void);
 
#ifdef __cplusplus
}
#endif
 
#endif /* __STM32F4xx_FLASH_H */
 
/**
  * @}
  */ 
 
/**
  * @}
  */ 
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


