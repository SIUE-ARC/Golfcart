Project:      output/PSoC_Steer_Final
Build Number: 0
Date:         Wed Sep 21 19:38:14 2016
Compiler:     ICCM8C
Version:      7.05.00


Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   01A0   0268 =    616. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       021F  LoadConfigTBL_dac_prac_Bank0
       02B8  LoadConfigTBL_dac_prac_Bank1
       0307  __ctype
       0408  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        func_lit   0408   0002 =      2. bytes (rel,con,rom,proclab)

       Addr  Global Symbol
      -----  --------------------------------
       0408  __func_lit_start
       040A  __func_lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     psoc_config   040A   0170 =    368. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       040A  LoadConfigTBL_dac_prac_Ordered
       050F  LoadConfigInit
       050F  _LoadConfigInit
       051F  _LoadConfig_dac_prac
       051F  LoadConfig_dac_prac
       057A  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   057A   073E =   1854. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       057A  _UART_TX_ISR
       057B  _UART_RX_ISR
       05EF  UART_EnableInt
       05EF  _UART_EnableInt
       05F6  _UART_DisableInt
       05F6  UART_DisableInt
       05FD  _UART_SetTxIntMode
       05FD  UART_SetTxIntMode
       060F  _UART_Start
       060F  UART_Start
       0618  _UART_Stop
       0618  UART_Stop
       061F  _UART_SendData
       061F  UART_SendData
       0622  UART_bReadTxStatus
       0622  bUART_ReadTxStatus
       0622  _bUART_ReadTxStatus
       0622  _UART_bReadTxStatus
       0625  UART_bReadRxData
       0625  bUART_ReadRxData
       0625  _bUART_ReadRxData
       0625  _UART_bReadRxData
       0628  _UART_bReadRxStatus
       0628  _bUART_ReadRxStatus
       0628  bUART_ReadRxStatus
       0628  UART_bReadRxStatus
       062B  _UART_TxIntMode
       062B  UART_TxIntMode
       063E  UART_IntCntl
       063E  _UART_IntCntl
       0669  _UART_PutSHexByte
       0669  UART_PutSHexByte
       067C  _UART_PutSHexInt
       067C  UART_PutSHexInt
       0683  _UART_PutChar
       0683  UART_PutChar
       068B  UART_cGetChar
       068B  _UART_cGetChar
       0693  _UART_cReadChar
       0693  UART_cReadChar
       06AA  _UART_iReadChar
       06AA  UART_iReadChar
       06C0  _UART_PutString
       06C0  UART_PutString
       06D2  UART_Write
       06D2  _UART_Write
       06FA  UART_CWrite
       06FA  _UART_CWrite
       0726  UART_CPutString
       0726  _UART_CPutString
       0738  UART_PutCRLF
       0738  _UART_PutCRLF
       0741  _UART_CmdReset
       0741  UART_CmdReset
       0757  UART_bCmdCheck
       0757  _UART_bCmdCheck
       075F  UART_bErrCheck
       075F  _UART_bErrCheck
       076A  UART_bCmdLength
       076A  _UART_bCmdLength
       0770  _UART_szGetParam
       0770  UART_szGetParam
       07CE  _UART_szGetRestOfParams
       07CE  UART_szGetRestOfParams
       07D9  _TX8_ISR
       07DA  TX8_EnableInt
       07DA  _TX8_EnableInt
       07DE  TX8_DisableInt
       07DE  _TX8_DisableInt
       07E2  _TX8_SetTxIntMode
       07E2  TX8_SetTxIntMode
       07F4  _TX8_Start
       07F4  TX8_Start
       07F9  _TX8_Stop
       07F9  TX8_Stop
       07FD  TX8_SendData
       07FD  _TX8_SendData
       0800  bTX8_ReadTxStatus
       0800  _TX8_bReadTxStatus
       0800  _bTX8_ReadTxStatus
       0800  TX8_bReadTxStatus
       0813  TX8_PutSHexByte
       0813  _TX8_PutSHexByte
       0826  _TX8_PutSHexInt
       0826  TX8_PutSHexInt
       082D  TX8_PutChar
       082D  _TX8_PutChar
       0835  TX8_PutString
       0835  _TX8_PutString
       0847  _TX8_Write
       0847  TX8_Write
       086F  TX8_CWrite
       086F  _TX8_CWrite
       0897  _TX8_CPutString
       0897  TX8_CPutString
       08A9  TX8_PutCRLF
       08A9  _TX8_PutCRLF
       08B2  _Steer_Pot_Start
       08B2  Steer_Pot_SetPower
       08B2  Steer_Pot_Start
       08B2  _Steer_Pot_SetPower
       08C4  Steer_Pot_SetGain
       08C4  _Steer_Pot_SetGain
       08EE  _Steer_Pot_Stop
       08EE  Steer_Pot_Stop
       08F2  _LCD_PrCString
       08F2  LCD_PrCString
       0904  LCD_HEX_STR
       0914  LCD_PrHexByte
       0914  _LCD_PrHexByte
       0927  LCD_PrHexInt
       0927  _LCD_PrHexInt
       092E  LCD_PrString
       092E  _LCD_PrString
       0940  _LCD_Write_Data
       0940  _LCD_WriteData
       0940  LCD_Write_Data
       0940  LCD_WriteData
       0954  _LCD_Control
       0954  LCD_Control
       0A04  LCD_Init
       0A04  LCD_Start
       0A04  _LCD_Init
       0A04  _LCD_Start
       0A73  LCD_ROW_OFFSET
       0A77  LCD_Position
       0A77  _LCD_Position
       0A87  _LCD_Delay50uTimes
       0A87  LCD_Delay50uTimes
       0A8D  LCD_Delay50u_Table
       0A95  _LCD_Delay50u
       0A95  LCD_Delay50u
       0AB7  _DUALADC_CNT1_ISR
       0ABA  _DUALADC_CNT2_ISR
       0ABD  _DUALADC_PWM16_ISR
       0B7B  _DUALADC_Start
       0B7B  _DUALADC_SetPower
       0B7B  DUALADC_Start
       0B7B  DUALADC_SetPower
       0BA5  DUALADC_Stop
       0BA5  _DUALADC_Stop
       0BAC  DUALADC_GetSamples
       0BAC  _DUALADC_GetSamples
       0BEF  _DUALADC_StopAD
       0BEF  DUALADC_StopAD
       0C10  DUALADC_fIsData
       0C10  _DUALADC_fIsDataAvailable
       0C10  DUALADC_fIsDataAvailable
       0C10  _DUALADC_fIsData
       0C18  _DUALADC_iGetData1ClearFlag
       0C18  DUALADC_iGetData1ClearFlag
       0C23  _DUALADC_iGetData1
       0C23  DUALADC_iGetData1
       0C2B  DUALADC_iGetData2ClearFlag
       0C2B  _DUALADC_iGetData2ClearFlag
       0C36  _DUALADC_iGetData2
       0C36  DUALADC_iGetData2
       0C3E  DUALADC_ClearFlag
       0C3E  _DUALADC_ClearFlag
       0C45  _DUALADC_SetResolution
       0C45  DUALADC_SetResolution
       0C78  _Actuator_Pot_SetPower
       0C78  Actuator_Pot_Start
       0C78  _Actuator_Pot_Start
       0C78  Actuator_Pot_SetPower
       0C8A  _Actuator_Pot_SetGain
       0C8A  Actuator_Pot_SetGain
       0CB4  _Actuator_Pot_Stop
       0CB4  Actuator_Pot_Stop
       0CB8  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0CB8   095D =   2397. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0CB8  _min
       0CB8  __text_start
       0CF2  _max
       0D2C  _main
       0E5F  _command_lookup
       106B  _setControllerSpeed
       113A  _stop
       114F  _updateBrakeCtl
       11D4  _updateTurnCtl
       123C  _getBrakePosition
       127D  _getSteerPotPosition
       12BE  _turn
       1301  _brake
       1344  _calibrateSteering
       13D1  _PSoC_GPIO_ISR_C
       145B  _atoi
       159C  _isdigit
       15BD  _isspace
       15DE  _abs
       1615  __xidata_start
       1615  __text_end
       161B  __xidata_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0000   0146 =    326. bytes (abs,ovr,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0068  __Start
       0143  _bGetPowerSetting
       0143  bGetPowerSetting

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   0000   000D =     13. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  _UART_fStatus
       0000  UART_fStatus
       0001  _UART_bRxCnt
       0001  UART_bRxCnt
       0002  NO_SHADOW
       0002  _NO_SHADOW
       0002  _Port_2_Data_SHADE
       0002  Port_2_Data_SHADE
       0003  _Port_2_DriveMode_0_SHADE
       0003  Port_2_DriveMode_0_SHADE
       0004  _Port_2_DriveMode_1_SHADE
       0004  Port_2_DriveMode_1_SHADE
       0005  DUALADC_cCounter1U
       0006  DUALADC_cCounter2U
       0007  _DUALADC_iResult1
       0007  DUALADC_iResult1
       0009  _DUALADC_iResult2
       0009  DUALADC_iResult2
       000B  _DUALADC_bfStatus
       000B  DUALADC_bfStatus
       000C  DUALADC_bSampC

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        UART_RAM   0000   0021 =     33. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       000D  _UART_aRxBuffer
       000D  UART_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   002D   0009 =      9. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       002D  __rZ
       002E  __rY
       002F  __rX
       0030  __r5
       0031  __r4
       0032  __r3
       0033  __r2
       0034  __r1
       0035  __r0

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0000   0011 =     17. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __data_start
       0036  _steerCount
       0040  _steerSetpoint
       0042  _brakeSetpoint
       0044  _cacheValid
       0045  _curPrt
       0046  _prevPrt

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
obj/main.o	[ main.c ]
<library>	[ uartint.asm, uart.asm, tx8int.asm, tx8.asm, steer_pot.asm, psocconfigtbl.asm, psocconfig.asm ]
[ lcd.asm, dualadcint.asm, dualadc.asm, actuator_pot.asm, rZ.asm, rY.asm, rX.asm, r5.asm, r4.asm, r3.asm ]
[ r2.asm, r1.asm, r0.asm, divmodu_16x16_16_util.asm, i_div16_block_util.asm, atoi.s, isdigit.s ]
[ isspace.s, ctype.s, abs.s ]

Area Definitions

-blit:0x1a0.0x7fff
-bdata:0.0x6ff
-BInterruptRAM:0	allocated at: 0
-Bvirtual_registers	allocated at: 0x2d
-Bdac_prac_RAM
-rwh:SSCParmBlk:0xf7.0xff

