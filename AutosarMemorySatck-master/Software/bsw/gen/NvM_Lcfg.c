/*
 * NvM_Lcfg.c
 *
 *
 *      Author: Sahar
 */


#include "NvM_Types.h"
#include "NvM_Cfg.h"
#include "Std_Types.h"
#include "Fee_Cfg.h"
#include "AppCfg.h"

extern  uint8 Block3_RAMBLOCK[NVM_NVRAM_BLOCK_3_LENGTH];
extern  uint8 Block5_PRAMBLOCK[NVM_NVRAM_BLOCK_5_LENGTH];
extern  uint8 Block5_ROMBLOCK[NVM_NVRAM_BLOCK_5_LENGTH];
NvMBlockDescriptorType NvMBlockDescriptor[NUMBER_OF_NVM_BLOCKS] =
 {
         /*Block 0 Saved*/
         {},
         /*Block 1 Saved*/
         {},
         /*Block 2 Configurations*/
         {
             .NvMBlockCrcType               =   NVM_CRC32           ,
             .NvMBlockManagement            =   NVM_BLOCK_NATIVE    ,
             .NvMBlockUseCrc                =   STD_ON              ,
             .NvMBlockUseSetRamBlockStatus  =   STD_ON              ,
             .NvMCalcRamBlockCrc            =   STD_OFF              ,
             .NvMMaxNumOfWriteRetries       =   1                   ,
             .NvMInitBlockCallback          =   NULL                ,
             .NvMNvBlockBaseNumber          =   NVM_NVRAM_BLOCK_2_BASENUMBER    ,
             .NvMNvBlockLength              =   NVM_NVRAM_BLOCK_2_LENGTH        ,
             .NvMNvBlockNum                 =   1                               ,
             .NvMNvramDeviceId              =   1                               ,
             .NvMRamBlockDataAddress        =   Block2_PRAMBLOCK                ,
             .NvMRomBlockDataAddress        =   Block2_ROMBLOCK                 ,
             .NvMRomBlockNum                =   1                               ,
             .NvMSelectBlockForReadAll      =   STD_ON                          ,
             .NvMSelectBlockForWriteAll     =   STD_ON                          ,
             .NvMSingleBlockCallback        =   Block_2_NvMSingleBlockCallback  ,
         },
         /*Block 3 Configurations*/
         {
             .NvMBlockCrcType               =   NVM_CRC32           ,
             .NvMBlockManagement            =   NVM_BLOCK_NATIVE   ,
             .NvMBlockUseCrc                =   STD_ON              ,
             .NvMBlockUseSetRamBlockStatus  =   STD_ON              ,
             .NvMCalcRamBlockCrc            =   STD_ON              ,
             .NvMMaxNumOfWriteRetries       =   1                   ,
             .NvMInitBlockCallback          =   NULL                ,
             .NvMNvBlockBaseNumber          =   NVM_NVRAM_BLOCK_3_BASENUMBER    ,
             .NvMNvBlockLength              =   NVM_NVRAM_BLOCK_3_LENGTH        ,
             .NvMNvBlockNum                 =   2                               ,
             .NvMNvramDeviceId              =   1                               ,
             .NvMRamBlockDataAddress        =   Block3_RAMBLOCK                        ,
             .NvMRomBlockDataAddress        =   Block3_ROMBLOCK                 ,
             .NvMRomBlockNum                =   1                               ,
             .NvMSelectBlockForReadAll      =   STD_ON                         ,
             .NvMSelectBlockForWriteAll     =   STD_ON                          ,
             .NvMSingleBlockCallback        =   Block_2_NvMSingleBlockCallback  ,
         }
         ,
         {
             .NvMBlockCrcType               =   NVM_CRC32           ,
             .NvMBlockManagement            =   NVM_BLOCK_NATIVE    ,
             .NvMBlockUseCrc                =   STD_ON            ,
             .NvMBlockUseSetRamBlockStatus  =   STD_ON              ,
             .NvMCalcRamBlockCrc            =   STD_ON              ,
             .NvMMaxNumOfWriteRetries       =   1                   ,
             .NvMInitBlockCallback          =   NULL                ,
             .NvMNvBlockBaseNumber          =   NVM_NVRAM_BLOCK_4_BASENUMBER    ,
             .NvMNvBlockLength              =   NVM_NVRAM_BLOCK_4_LENGTH        ,
             .NvMNvBlockNum                 =   3                               ,
             .NvMNvramDeviceId              =   1                               ,
             .NvMRamBlockDataAddress        =   Block4_PRAMBLOCK                ,
             .NvMRomBlockDataAddress        =   Block4_ROMBLOCK                 ,
             .NvMRomBlockNum                =   1                               ,
             .NvMSelectBlockForReadAll      =   STD_ON                          ,
             .NvMSelectBlockForWriteAll     =   STD_ON                          ,
             .NvMSingleBlockCallback        =   Block_2_NvMSingleBlockCallback  ,
         }

         ,
         {
             .NvMBlockCrcType               =   NVM_CRC32           ,
             .NvMBlockManagement            =   NVM_BLOCK_NATIVE    ,
             .NvMBlockUseCrc                =   STD_ON              ,
             .NvMBlockUseSetRamBlockStatus  =   STD_ON              ,
             .NvMCalcRamBlockCrc            =   STD_ON              ,
             .NvMMaxNumOfWriteRetries       =   1                   ,
             .NvMInitBlockCallback          =   NULL                ,
             .NvMNvBlockBaseNumber          =   NVM_NVRAM_BLOCK_5_BASENUMBER    ,
             .NvMNvBlockLength              =   NVM_NVRAM_BLOCK_5_LENGTH        ,
             .NvMNvBlockNum                 =   4                               ,
             .NvMNvramDeviceId              =   0                               ,
             .NvMRamBlockDataAddress        =   Block5_PRAMBLOCK                ,
             .NvMRomBlockDataAddress        =   Block5_ROMBLOCK                 ,
             .NvMRomBlockNum                =   1                               ,
             .NvMSelectBlockForReadAll      =   STD_ON                          ,
             .NvMSelectBlockForWriteAll     =   STD_ON                          ,
             .NvMSingleBlockCallback        =   Block_2_NvMSingleBlockCallback  ,
         }
 };

