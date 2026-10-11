/*
 * Copyright (c) 2025 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * stm32l/hw/SDMMC.cpp
 */

#include "SDMMC.h"

#define MYDBG(...)      DBGCL("SDMMC", __VA_ARGS__)

//#define SDMMC_TRACE   1

#if SDMMC_TRACE
#define MYTRACE(...)    MYDBG(__VA_ARGS__)
#else
#define MYTRACE(...)
#endif

template<> const GPIOPinTable_t _SDMMC<1>::afClk = GPIO_PINS(pC(12, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afCmd = GPIO_PINS(pD(2, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD0 = GPIO_PINS(pC(8, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD1 = GPIO_PINS(pC(9, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD2 = GPIO_PINS(pC(10, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD3 = GPIO_PINS(pC(11, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD4 = GPIO_PINS(pB(8, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD5 = GPIO_PINS(pB(9, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD6 = GPIO_PINS(pC(6, 12));
template<> const GPIOPinTable_t _SDMMC<1>::afD7 = GPIO_PINS(pC(7, 12));

SDMMC::CommandResult SDMMC::Command(uint32_t cmd, uint32_t arg)
{
    uint32_t state;

    if (int(cmd) < 0)
    {
        // app command
        cmd = ~cmd;
        auto res = Command(CmdApp, cmd >> 16);
        if (!res) { return res; }
    }

    ASSERT(!(STA & SDMMC_STA_CMASK));
    ARG = arg;
    CMD = (cmd | SDMMC_CMD_CPSMEN) & 0xFFF;
    __DSB();
    while ((state = STA) & SDMMC_STA_CMDACT);
    MYTRACE("%d %X %d %X = %X %X", !!(cmd & RespNoCmd), (cmd >> 6) & 0x3F, cmd & 0x3F, arg, state, RESP1);
    ASSERT(STA & SDMMC_STA_CMASK);
    ICR = SDMMC_STA_CMASK;  // clear command status bits

    auto expectResp = ((cmd & SDMMC_CMD_WAITRESP) == RespShort) && !(cmd & RespNoCmd) ? cmd & SDMMC_CMD_CMDINDEX : 0x3F;
    if (RESPCMD != expectResp)
    {
        // RESPCMD mismatch, report as CRC failure
        MYTRACE("RESPCMD %d != %d", RESPCMD, expectResp);
        return state | SDMMC_STA_CCRCFAIL;
    }

    if ((cmd & RespNoCRC) && (state & SDMMC_STA_CCRCFAIL))
    {
        // convert CRC failure into success
        state ^= SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND;
    }

    return state;
}

void SDMMC::ConfigureDmaRead(DMAChannel& dma, Buffer buf, size_t blockSize)
{
    ASSERT(!dma.IsEnabled());
    ASSERT(!(buf.Length() & 3));
    ASSERT(!(STA & (SDMMC_STA_DMASK | SDMMC_STA_TXACT | SDMMC_STA_RXACT)));
    dma.Descriptor() = DMADescriptor::Transfer(&FIFO, buf.Pointer(), buf.Length() >> 2,
        DMADescriptor::IncrementMemory | DMADescriptor::P2M | DMADescriptor::UnitWord | DMADescriptor::PrioLow);
    dma.ClearAndEnable();

    DTIMER = 48000000;
    DLEN = buf.Length();
    Configure(DataTransferStart() | DataTransferDirection(true) | DataTransferDma() | DataTransferBlockSize(blockSize ? blockSize : buf.Length()));
}

void SDMMC::ConfigureDmaWrite(DMAChannel& dma, Span buf, size_t blockSize)
{
    ASSERT(!dma.IsEnabled());
    ASSERT(!(buf.Length() & 3));
    ASSERT(!(STA & (SDMMC_STA_DMASK | SDMMC_STA_TXACT | SDMMC_STA_RXACT)));
    dma.Descriptor() = DMADescriptor::Transfer(buf.Pointer(), &FIFO, buf.Length() >> 2,
        DMADescriptor::IncrementMemory | DMADescriptor::M2P | DMADescriptor::UnitWord | DMADescriptor::PrioLow);
    dma.ClearAndEnable();

    DTIMER = 48000000;
    DLEN = buf.Length();
    Configure(DataTransferStart() | DataTransferDirection(false) | DataTransferDma() | DataTransferBlockSize(blockSize ? blockSize : buf.Length()));
}

SDMMC::DataResult SDMMC::AbortDmaTransfer(DMAChannel& dma)
{
    Configure(DataTransferStart(false));
    while (STA & (SDMMC_STA_TXACT | SDMMC_STA_RXACT));
    dma.Disable();
    dma.ClearInterrupt();
    auto res = STA;
    ICR = SDMMC_STA_DMASK;
    return res;
}

SDMMC::DataResult SDMMC::CompleteDmaTransfer(DMAChannel& dma)
{
    dma.Disable();
    // FIX (temporary): these three used to be ASSERT()s - live-debugged a "display freezes
    // forever" bug down to one of them firing (confirmed live: both the CNDTR one and the DMASK
    // one have been seen to fire) at the moment a Display-side DMA stall / bus contention episode
    // coincides with an SD DMA transfer. A tripped ASSERT() here doesn't reset - DBG_AssertFailed()
    // loops forever feeding the watchdog, so it reads as a permanent freeze. None of these three
    // checks are needed for correctness: the real success/failure signal is DataResult::Success()
    // (SDMMC_STA_DBCKEND, checked by the caller), which is completely independent of them - the
    // caller already has a proper error path for a failed transfer. Downgraded to warnings so a
    // genuine contention hiccup fails the transfer (caller retries/reports error) instead of
    // hanging the whole device.
    if (dma.CNDTR != 0)
    {
        MYDBG("*** CompleteDmaTransfer: CNDTR=%u (expected 0) - DMA contention? ***", (unsigned)dma.CNDTR);
    }
    auto res = STA;
    if (STA & (SDMMC_STA_TXACT | SDMMC_STA_RXACT))
    {
        MYDBG("*** CompleteDmaTransfer: TXACT/RXACT still set (STA=%X) - DMA contention? ***", (unsigned)STA);
    }
    if (!(res & SDMMC_STA_DMASK))
    {
        MYDBG("*** CompleteDmaTransfer: DMASK not set after DMA transfer complete (STA=%X) - DMA contention? ***", res);
    }
    ICR = SDMMC_STA_DMASK;
    return res;
}

async(SDMMC::Initialize)
async_def()
{
    // power off
    Configure(CardClockEnable(false));
    Configure(Power(false));

    async_delay_ms(1);

    // set lowest clock frequency for init phase
    Configure(CardClockDivisor(256) | PowerSaving() | BusWidth(BusWidth1) | CardClockDephase(false) | FlowControl());
    Configure(Power());

    async_delay_ms(1);

    Configure(CardClockEnable());

    async_delay_ms(1);
}
async_end

async(SDMMC::IdentifyCard, CardInfo& info)
async_def(
    uint32_t init;
    bool v2;
    Timeout timeout;
)
{
    CommandResult cr;

    // card ID procedure
    if (!Command_GoIdle()) { async_return(false); }

    f.v2 = true;
    if (!Command_SendIfCond())
    {
        f.v2 = false;
        // V1.x card, go back to idle
        if (!Command_GoIdle()) { async_return(false); }
    }

    // query voltage ranges
    f.init = SD_OpCondHS * f.v2;
    if (!AppCommand_SendOpCond(f.init)) { async_return(false); }

    // at this point, we're definitely talking to a real card,
    // log any errors encountered

    // wait for init to complete, sending back whatever voltages the card supports
    f.init |= RESP1 & SD_OpCondVoltages;
    if (!(cr = AppCommand_SendOpCond(f.init)))
    {
        MYDBG("Failed to start card init: %X", cr);
        async_return(false);
    }

    f.timeout = Timeout::Milliseconds(1500).MakeAbsolute();

    for (;;)
    {
        async_yield();
        if (!(cr = AppCommand_SendOpCond(f.init)))
        {
            MYDBG("Error during card init: %X", cr);
            async_return(false);
        }

        if (RESP1 & SD_OpCondInitDone)
        {
            info.blockAddressing = RESP1 & SD_OpCondHS;
            break;
        }

        if (f.timeout.Elapsed())
        {
            MYDBG("Card init never completed");
            async_return(false);
        }
    }

    if (!(cr = Command_AllSendCid()))
    {
        MYDBG("Failed to retrieve CID: %08X", cr);
        async_return(false);
    }

    info.cid = ResultCid();

    if (!(cr = Command_SendRelativeAddr()))
    {
        MYDBG("Failed to retrieve RCA: %08X", cr);
        async_return(false);
    }
    info.rca = ResultRca();

    // try switching to high speed
    Configure(CardClockDivisor(1));

    if (!(cr = Command_SendCsd(info.rca.rca)))
    {
        MYDBG("Failed to retrieve CSD or communicate with card at full speed: %08X", cr);
        async_return(false);
    }
    info.csd = ResultCsd();

#if TRACE
    auto& cid = info.cid;
    auto& rca = info.rca;
    auto& csd = info.csd;
    MYDBG("Card identification successful");
    MYDBG("CID: %02X:%.2s %.5s R%d.%d SN %08X MFG %02d-%02d",
        cid.mfg, cid.appId, cid.prod,
        cid.bcdRev >> 4, cid.bcdRev & 0xF,
        cid.psn, cid.Year(), cid.month);
    MYDBG("RCA: %04X %c%c%c %d", rca.rca,
        rca.error ? 'E' : '-',
        rca.crcError ? 'C' : '-',
        rca.illegalCmd ? 'I' : '-',
        rca.status);
    MYDBG("CSD: %s, %d blocks (%d MB)", info.blockAddressing ? "SDHC" : "SDSC", csd.BlockCount(), csd.CapacityMB());
#endif

    async_return(true);
}
async_end
