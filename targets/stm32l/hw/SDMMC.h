/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * stm32l/hw/SDMMC.h
 */

#pragma once

#include <base/base.h>

#include <hw/ConfigRegister.h>
#include <hw/GPIO.h>
#include <hw/DMA.h>

struct SDMMC : SDMMC_TypeDef
{
    #pragma region Configuration registers

    using RegPOWER = ConfigRegister<&SDMMC_TypeDef::POWER>;
    using RegCLKCR = ConfigRegister<&SDMMC_TypeDef::CLKCR>;
    using RegDCTRL = ConfigRegister<&SDMMC_TypeDef::DCTRL>;
    using RegMASK = ConfigRegister<&SDMMC_TypeDef::MASK>;

    #pragma region POWER configuration register

    static constexpr auto Power(bool enable = true) { return RegPOWER(enable, SDMMC_POWER_PWRCTRL); }

    #pragma endregion

    #pragma region CLKCR configuration register

    static constexpr auto CardClockDivisor(unsigned div) { ASSERT(div > 0); return RegCLKCR(div == 1 ? SDMMC_CLKCR_BYPASS : div - 2, SDMMC_CLKCR_BYPASS | SDMMC_CLKCR_CLKDIV); }
    static constexpr auto CardClockEnable(bool enable = true) { return RegCLKCR(enable, SDMMC_CLKCR_CLKEN); }
    static constexpr auto PowerSaving(bool enable = true) { return RegCLKCR(enable, SDMMC_CLKCR_PWRSAV); }

    enum BusWidth
    {
        BusWidth1,
        BusWidth4 = SDMMC_CLKCR_WIDBUS_0,
        BusWidth8 = SDMMC_CLKCR_WIDBUS_1,
    };

    static constexpr auto BusWidth(BusWidth width) { return RegCLKCR(uint32_t(width), SDMMC_CLKCR_WIDBUS); }
    static constexpr auto CardClockDephase(bool enable = true) { return RegCLKCR(enable, SDMMC_CLKCR_NEGEDGE); }
    static constexpr auto FlowControl(bool enable = true) { return RegCLKCR(enable, SDMMC_CLKCR_HWFC_EN); }

    #pragma endregion

    #pragma region DCTRL configuration register

    static constexpr auto DataTransferStart(bool enable = true) { return RegDCTRL(enable, SDMMC_DCTRL_DTEN); }
    static constexpr auto DataTransferDirection(bool read) { return RegDCTRL(read, SDMMC_DCTRL_DTDIR); }
    static constexpr auto DataTransferStream(bool enable = true) { return RegDCTRL(enable, SDMMC_DCTRL_DTMODE); }
    static constexpr auto DataTransferDma(bool enable = true) { return RegDCTRL(enable, SDMMC_DCTRL_DMAEN); }
    static constexpr auto DataTransferBlockSize(unsigned blockSize) { return RegDCTRL(uint32_t(__builtin_ctz(blockSize) << SDMMC_DCTRL_DBLOCKSIZE_Pos), SDMMC_DCTRL_DBLOCKSIZE); }

    enum ReadWait
    {
        ReadWaitNone = 0,
        ReadWaitStart = SDMMC_DCTRL_RWSTART,
        ReadWaitStartStop = SDMMC_DCTRL_RWSTART | SDMMC_DCTRL_RWSTOP,
    };

    static constexpr auto DataTransferReadWait(ReadWait readWait) { return RegDCTRL(uint32_t(readWait), SDMMC_DCTRL_RWSTART | SDMMC_DCTRL_RWSTOP); }
    static constexpr auto DataTransferReadWaitOnClock(bool enable = true) { return RegDCTRL(enable, SDMMC_DCTRL_RWMOD); }
    static constexpr auto DataTransferSdio(bool enable = true) { return RegDCTRL(enable, SDMMC_DCTRL_SDIOEN); }

    #pragma endregion

    #pragma region MASK configuration register

    static constexpr auto InterruptCommandCrcError(bool enable = true) { return RegMASK(enable, SDMMC_MASK_CCRCFAILIE); }
    static constexpr auto InterruptDataCrcError(bool enable = true) { return RegMASK(enable, SDMMC_MASK_DCRCFAILIE); }
    static constexpr auto InterruptCommandTimeout(bool enable = true) { return RegMASK(enable, SDMMC_MASK_CTIMEOUTIE); }
    static constexpr auto InterruptDataTimeout(bool enable = true) { return RegMASK(enable, SDMMC_MASK_DTIMEOUTIE); }
    static constexpr auto InterruptTxUnderrun(bool enable = true) { return RegMASK(enable, SDMMC_MASK_TXUNDERRIE); }
    static constexpr auto InterruptRxOverrun(bool enable = true) { return RegMASK(enable, SDMMC_MASK_RXOVERRIE); }
    static constexpr auto InterruptCommandResponse(bool enable = true) { return RegMASK(enable, SDMMC_MASK_CMDRENDIE); }
    static constexpr auto InterruptCommandSent(bool enable = true) { return RegMASK(enable, SDMMC_MASK_CMDSENTIE); }
    static constexpr auto InterruptDataEnd(bool enable = true) { return RegMASK(enable, SDMMC_MASK_DATAENDIE); }
    static constexpr auto InterruptDataBlockEnd(bool enable = true) { return RegMASK(enable, SDMMC_MASK_DBCKENDIE); }
    static constexpr auto InterruptCommandActing(bool enable = true) { return RegMASK(enable, SDMMC_MASK_CMDACTIE); }
    static constexpr auto InterruptDataTxActing(bool enable = true) { return RegMASK(enable, SDMMC_MASK_TXACTIE); }
    static constexpr auto InterruptDataRxActing(bool enable = true) { return RegMASK(enable, SDMMC_MASK_RXACTIE); }
    static constexpr auto InterruptTxFifoHalf(bool enable = true) { return RegMASK(enable, SDMMC_MASK_TXFIFOHEIE); }
    static constexpr auto InterruptRxFifoHalf(bool enable = true) { return RegMASK(enable, SDMMC_MASK_RXFIFOHFIE); }
    static constexpr auto InterruptTxFifoFull(bool enable = true) { return RegMASK(enable, SDMMC_MASK_TXFIFOFIE); }
    static constexpr auto InterruptRxFifoFull(bool enable = true) { return RegMASK(enable, SDMMC_MASK_RXFIFOFIE); }
    static constexpr auto InterruptTxFifoEmpty(bool enable = true) { return RegMASK(enable, SDMMC_MASK_TXFIFOEIE); }
    static constexpr auto InterruptRxFifoEmpty(bool enable = true) { return RegMASK(enable, SDMMC_MASK_RXFIFOEIE); }
    static constexpr auto InterruptTxData(bool enable = true) { return RegMASK(enable, SDMMC_MASK_TXDAVLIE); }
    static constexpr auto InterruptRxData(bool enable = true) { return RegMASK(enable, SDMMC_MASK_RXDAVLIE); }
    static constexpr auto InterruptSdio(bool enable = true) { return RegMASK(enable, SDMMC_MASK_SDIOITIE); }

    #pragma endregion

    DEFINE_CONFIGURE_METHOD(SDMMC_TypeDef);

    #pragma endregion

    #pragma region Commands

    struct CommandResult
    {
    public:
        constexpr CommandResult(uint32_t sta = 0) : res(sta) {}
        constexpr operator bool() const { return Success(); }

        constexpr bool Success() const { return res & (SDMMC_STA_CMDSENT | SDMMC_STA_CMDREND); }
        constexpr bool Timeout() const { return res & SDMMC_STA_CTIMEOUT; }
        constexpr bool CRCError() const { return res & SDMMC_STA_CCRCFAIL; }

    private:
        uint32_t res;
    };

    struct DataResult
    {
    public:
        constexpr DataResult(uint32_t sta) : res(sta) {}
        constexpr operator bool() const { return Success(); }

        constexpr bool Success() const { return res & SDMMC_STA_DBCKEND; }
        constexpr bool Timeout() const { return res & SDMMC_STA_DTIMEOUT; }
        constexpr bool CRCError() const { return res & SDMMC_STA_DCRCFAIL; }

    private:
        uint32_t res;
    };

    enum struct HostVoltage
    {
        Voltage2p7_3p6 = 1,
    };

    CommandResult Command_GoIdle() { return Command(CmdGoIdleState, 0); }
    CommandResult Command_SendOpCond() { return Command(CmdSendOpCond, 0); }
    CommandResult Command_SendIfCond(HostVoltage hv = HostVoltage::Voltage2p7_3p6) { return Command(CmdSendIfCond, TestPattern | (uint8_t(hv) << 8)); }
    CommandResult Command_AllSendCid() { return Command(CmdAllSendCid, 0); }
    CommandResult Command_SendRelativeAddr() { return Command(CmdSendRelativeAddr, 0); }
    CommandResult Command_SelectCard(uint16_t rca) { return Command(CmdSelectCard, rca << 16); }
    CommandResult Command_SendCsd(uint16_t rca) { return Command(CmdSendCsd, rca << 16); }
    CommandResult Command_SendStatus(uint16_t rca) { return Command(CmdSendStatus, rca << 16); }
    CommandResult Command_ReadSingleBlock(uint32_t address) { return Command(CmdReadSingleBlock, address); }
    CommandResult Command_WriteBlock(uint32_t address) { return Command(CmdWriteBlock, address); }
    //! Opcodes existed in the enum below from the start, just never had a wrapper - SDMMCDriver's
    //! Read()/Write() looped single-block commands one sector at a time regardless of how many
    //! sectors were actually requested, paying a full command-response round trip (plus its own
    //! DMA setup/teardown) per 512 bytes. See SDMMCDriver.cpp for where these are actually used.
    CommandResult Command_ReadMultipleBlock(uint32_t address) { return Command(CmdReadMultipleBlock, address); }
    CommandResult Command_WriteMultipleBlock(uint32_t address) { return Command(CmdWriteMultipleBlock, address); }
    //! Terminates a CMD18/CMD25 multi-block transfer - required by the SD spec after one (never
    //! needed, and never sent, after a single-block CMD17/CMD24, which completes on its own).
    CommandResult Command_StopTransmission() { return Command(CmdStopTransmission, 0); }

    CommandResult AppCommand_Test() { return Command(CmdApp, 0); }
    CommandResult AppCommand_SendOpCond(uint32_t arg) { return Command(ACmdSendOpCond, arg); }

    PACKED_UNALIGNED_STRUCT CID
    {
        uint8_t mfg;
        char appId[2];
        char prod[5];
        uint8_t bcdRev;
        uint32_t psn;
        uint8_t yearh;
        uint8_t month : 4;
        uint8_t yearl : 4;
        uint8_t stop : 1;
        uint8_t crc : 7;

        constexpr int Year() const { return 2000 + (yearh << 4 | yearl); }
    };

    CID ResultCid()
    {
        union { uint32_t raw[4]; CID cid; } u = { {  FROM_BE32(RESP1), FROM_BE32(RESP2), FROM_BE32(RESP3), FROM_BE32(RESP4) } };
        return u.cid;
    }

    PACKED_UNALIGNED_STRUCT CSDv1
    {
        // byte 0 [120:127]
        uint8_t : 6;
        uint8_t csd : 2;

        // bytes 1-4 [96:119]
        uint8_t taac, nsac, tranSpeed, cccH;

        // byte 5 [80:87]
        uint8_t rdLen : 4;
        uint8_t cccL : 4;

        // byte 6 [72:79]
        uint8_t csizeH : 2;
        uint8_t : 2;
        bool dsrImp : 1;
        bool rdMis : 1;
        bool wrMis : 1;
        bool rdPart : 1;

        // byte 7 [64:71]
        uint8_t csizeM;

        // byte 8 [56:63]
        uint8_t iRdVMax : 3;
        uint8_t iRdVMin : 3;
        uint8_t csizeL : 2;

        // byte 9 [48:55]
        uint8_t csizeMulH : 2;
        uint8_t iWrVMax : 3;
        uint8_t iWrVMin : 3;

        // byte 10 [40:47]
        uint8_t secSizeH : 6;
        bool eraseBlk : 1;
        uint8_t csizeMulL : 1;

        // byte 11 [32:39]
        uint8_t wpGrpSize : 7;
        uint8_t secSizeL : 1;

        // byte 12 [24:31]
        uint8_t wrLenH : 2;
        uint8_t rwFactor : 3;
        uint8_t : 2;
        bool wpGrpEnable : 1;

        // byte 13 [16:23]
        uint8_t : 5;
        bool wrPart : 1;
        uint8_t wrLenL : 2;

        // byte 14 [8:15]
        uint8_t : 1;
        bool wpUpc : 1;
        uint8_t format : 2;
        bool wpTmp : 1;
        bool wpPerm : 1;
        bool copy : 1;
        bool ffGroup : 1;

        // byte 15 [0:7]
        uint8_t stop : 1;
        uint8_t crc : 7;

        constexpr unsigned CSizeMult() const { return csizeMulH << 1 | csizeMulL; }
        constexpr unsigned CSize() const { return csizeH << 10 | csizeM << 2 | csizeL; }
        constexpr unsigned BlockCount() const { return (CSize() + 1) << (CSizeMult() + 2); }
        constexpr unsigned BlockSize() const { return 1 << rdLen; }
        constexpr unsigned CapacityBytes() const { return BlockCount() << rdLen; }
        constexpr unsigned CapacityMB() const { return CapacityBytes() >> 20; }
    };

    PACKED_UNALIGNED_STRUCT CSDv2
    {
        // byte 0 [120:127]
        uint8_t : 6;
        uint8_t csd : 2;

        // bytes 1-4 [96:119]
        uint8_t taac, nsac, tranSpeed, cccH;

        // byte 5 [80:87]
        uint8_t rdLen : 4;
        uint8_t cccL : 4;

        // byte 6 [72:79]
        uint8_t : 4;
        bool dsrImp : 1;
        bool rdMis : 1;
        bool wrMis : 1;
        bool rdPart : 1;

        // byte 7 [64:71]
        uint8_t csizeH : 6;
        uint8_t : 2;

        // byte 8-9 [48:63]
        uint16_t csizeL;

        // byte 10 [40:47]
        uint8_t secSizeH : 6;
        bool eraseBlk : 1;
        uint8_t : 1;

        // byte 11 [32:39]
        uint8_t wpGrpSize : 7;
        uint8_t secSizeL : 1;

        // byte 12 [24:31]
        uint8_t wrLenH : 2;
        uint8_t rwFactor : 3;
        uint8_t : 2;
        bool wpGrpEnable : 1;

        // byte 13 [16:23]
        uint8_t : 5;
        bool wrPart : 1;
        uint8_t wrLenL : 2;

        // byte 14 [8:15]
        uint8_t : 1;
        bool wpUpc : 1;
        uint8_t format : 2;
        bool wpTmp : 1;
        bool wpPerm : 1;
        bool copy : 1;
        bool ffGroup : 1;

        // byte 15 [0:7]
        uint8_t stop : 1;
        uint8_t crc : 7;

        constexpr size_t CSize() const { return csizeH << 16 | FROM_BE16(csizeL); }
        constexpr unsigned BlockSize() const { return 512; }
        constexpr unsigned BlockCount() const { return (CSize() + 1) << 10; }
        constexpr size_t CapacityMB() const { return (CSize() + 1) >> 1; }
    };

    union CSD
    {
        CSDv1 v1;
        CSDv2 v2;

        constexpr bool IsV2() const { return v2.csd; }
        constexpr unsigned BlockSize() const { return IsV2() ? v2.BlockSize() : v1.BlockSize(); }
        constexpr unsigned BlockCount() const { return IsV2() ? v2.BlockCount() : v1.BlockCount(); }
        constexpr unsigned CapacityMB() const { return IsV2() ? v2.CapacityMB() : v1.CapacityMB(); }
    };

    CSD ResultCsd() {
        union { uint32_t raw[4]; CSD csd; } u = { {  FROM_BE32(RESP1), FROM_BE32(RESP2), FROM_BE32(RESP3), FROM_BE32(RESP4) } };
        return u.csd;
    }

    enum struct CardStatus
    {
        Idle, Ready, Ident, Stby,
        Tran, Data, Rcv, Prg, Dis,
    };

    PACKED_UNALIGNED_STRUCT Rca
    {
        uint16_t : 9;
        CardStatus status : 4;
        bool error : 1;
        bool illegalCmd : 1;
        bool crcError : 1;
        uint16_t rca;
    };

    Rca ResultRca()
    {
        union { uint32_t u; Rca r; } u = { RESP1 };
        return u.r;
    }

    #pragma endregion

    #pragma region Configuration helpers

    //! @p blockSize must be the SD block size (512) regardless of how large @p buf is - DLEN
    //! (derived from buf.Length()) is the *total* transfer length, DTBLOCKSIZE is the fixed unit
    //! the card raises DBCKEND on; a single-block transfer has them equal, but a multi-block one
    //! (see Command_ReadMultipleBlock/WriteMultipleBlock) does not - defaulted to buf.Length()
    //! only so the (now historical) single-sector-only callers before multi-block existed keep
    //! compiling unchanged.
    void ConfigureDmaRead(DMAChannel& dma, Buffer buf, size_t blockSize = 0);
    void ConfigureDmaWrite(DMAChannel& dma, Span buf, size_t blockSize = 0);
    DataResult AbortDmaTransfer(DMAChannel& dma);
    DataResult CompleteDmaTransfer(DMAChannel& dma);

    #pragma endregion

    #pragma region High-level interface

    struct CardInfo
    {
        CID cid;
        CSD csd;
        Rca rca;
        bool blockAddressing;
    };

    async(Initialize);
    async(IdentifyCard, CardInfo& ci);
    async_once(WaitNotBusy, OPT_TIMEOUT_ARG) { return async_forward(WaitMask, STA, SDMMC_STA_TXACT | SDMMC_STA_RXACT, 0, timeout); }

    #pragma endregion

private:
    enum
    {
        CmdSendMask = BIT(12) - 1,

        RespShort = SDMMC_CMD_WAITRESP_0,
        RespLong = SDMMC_CMD_WAITRESP_0 | SDMMC_CMD_WAITRESP_1,
        RespNoCmd = BIT(12),
        RespNoCRC = BIT(13),

        TestPattern = 0xAA,

        CmdGoIdleState = 0,
        CmdSendOpCond = 1 | RespShort,
        CmdAllSendCid = 2 | RespLong,
        CmdSendRelativeAddr = 3 | RespShort,
        CmdSwitchFunc = 6 | RespShort,
        CmdSelectCard = 7 | RespShort,
        CmdSendIfCond = 8 | RespShort,
        CmdSendCsd = 9 | RespLong,
        CmdSendCid = 10 | RespLong,
        CmdStopTransmission = 12 | RespShort,
        CmdSendStatus = 13 | RespShort,

        CmdSetBlockLen = 16 | RespShort,

        CmdReadSingleBlock = 17 | RespShort,
        CmdReadMultipleBlock = 18 | RespShort,

        CmdWriteBlock = 24 | RespShort,
        CmdWriteMultipleBlock = 25 | RespShort,

        CmdProgramCsd = 27 | RespShort,

        CmdEraseStartAddress = 32 | RespShort,
        CmdEraseEndAddress = 33 | RespShort,
        CmdErase = 38 | RespShort,

        CmdApp = 55 | RespShort,
        CmdGen = 56 | RespShort,
        CmdReadOcr = 58 | RespShort,
        CmdCrcOnOff = 59 | RespShort,

        ACmdSdStatus = ~(13 | RespLong),
        ACmdSendNumWrBlocks = ~(22 | RespShort),
        ACmdSetWrBlkEraseCount = ~(23 | RespShort),
        ACmdSendOpCond = ~(41 | RespShort | RespNoCmd | RespNoCRC),
        ACmdSetClrCardDetect = ~(42 | RespShort),
        ACmdSendScr = ~(52 | RespShort),

        SD_OpCondInitDone = BIT(31),
        SD_OpCondHS = BIT(30),
        SD_OpCondVoltages = ::MASK(9, 15),

        SDMMC_STA_CMASK = SDMMC_STA_CMDSENT | SDMMC_STA_CMDREND | SDMMC_STA_CCRCFAIL | SDMMC_STA_CTIMEOUT,
        SDMMC_STA_DMASK = SDMMC_STA_DATAEND | SDMMC_STA_DBCKEND | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT,
    };

    CommandResult Command(uint32_t cmd, uint32_t arg);
    ALWAYS_INLINE CommandResult AppCommand(uint32_t cmd, uint32_t arg) { return Command(~cmd, arg); }
};

template<unsigned n>
struct _SDMMC : SDMMC
{
    static const GPIOPinTable_t afClk, afCmd, afD0, afD1, afD2, afD3, afD4, afD5, afD6, afD7;

    //! Gets the zero-based index of the peripheral
    constexpr unsigned Index() const { return n - 1; }

    //! Enables peripheral clock
    void EnableClock() const;

    void ConfigureClk(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afClk, mode); }
    void ConfigureCmd(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afCmd, mode); }
    void ConfigureD0(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD0, mode); }
    void ConfigureD1(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD1, mode); }
    void ConfigureD2(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD2, mode); }
    void ConfigureD3(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD3, mode); }
    void ConfigureD4(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD4, mode); }
    void ConfigureD5(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD5, mode); }
    void ConfigureD6(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD6, mode); }
    void ConfigureD7(GPIOPin pin, GPIOPin::Mode mode = GPIOPin::SpeedVeryHigh)
        { pin.ConfigureAlternate(afD7, mode); }

    DMAChannel* Dma() const;
};

#undef SDMMC1
#define SDMMC1  CM_PERIPHERAL(_SDMMC<1>, SDMMC1_BASE)

template<> inline void _SDMMC<1>::EnableClock() const { RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN; __DSB(); }
template<> inline DMAChannel* _SDMMC<1>::Dma() const { return DMA::ClaimChannel({ 1, 4, 7 }, { 1, 5, 7 }); }
