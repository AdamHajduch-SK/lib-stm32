/*
 * Copyright (c) 2025 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * stm32l/fatfs/SDMMCDriver.cpp
 */

#include "SDMMCDriver.h"

#define MYDBG(...)  DBGCL("SDMMC", __VA_ARGS__)

//#define SDMMC_TRACE   1

//! tries per sector read before the card is declared lost - see Read
#define SD_READ_ATTEMPTS    3

#if SDMMC_TRACE
#define MYTRACE(...)  MYDBG(__VA_ARGS__)
#else
#define MYTRACE(...)
#endif

namespace fatfs
{

async(SDMMCDriver::Init)
async_def(
    bool v2;
    uint32_t init;
    Timeout timeout;
)
{
    if (Status() & STA_NOINIT)
    {
        // enable SDMMC clock, reset it
        RCC->APB2RSTR |= RCC_APB2RSTR_SDMMC1RST;
        RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SDMMC1RST;

        await(sd.Initialize);

        Status(STA_NODISK);
    }

    if (Status() & STA_NODISK)
    {
        if (await(sd.IdentifyCard, ci))
        {
            addrMul = ci.blockAddressing ? 1 : FF_MAX_SS;
            if (sd.Command_SelectCard(ci.rca.rca))
            {
                Status(0);
                async_return(RES_OK);
            }
        }

        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    async_return(RES_OK);
}
async_end

async(SDMMCDriver::Test)
async_def()
{
    if (!await(sd.WaitNotBusy, Timeout::Seconds(1)))
    {
        MYDBG("Timeout waiting for card to become available");
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    if (!sd.Command_SendStatus(ci.rca.rca))
    {
        MYDBG("Failed to get card status");
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    async_return(RES_OK);
}
async_end

async(SDMMCDriver::Read, void* buf, LBA_t sectorStart, size_t sectorCount)
async_def(
    LBA_t sec;
    void* buf;
    size_t count;
    int attempt;
)
{
    SDMMC::CommandResult cr;

    f.sec = sectorStart;
    f.buf = buf;
    f.count = sectorCount;

    if (!await(sd.WaitNotBusy, Timeout::Seconds(1)))
    {
        MYDBG("Timeout waiting for card to become available");
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    // One CMD18 (READ_MULTIPLE_BLOCK) + one DMA burst covering the whole request instead of a
    // fresh CMD17 (READ_SINGLE_BLOCK) - with its own DMA setup/teardown and command-response
    // round trip - for every single 512-byte sector. Confirmed live as the actual bottleneck
    // behind slow USB file transfers: every sector paid that overhead on top of the real data
    // time, regardless of how many consecutive sectors the caller actually asked for in one go.
    // CMD17 is kept for the (now uncommon, but still correct) exact single-sector case - it
    // completes on its own with no STOP_TRANSMISSION needed, unlike CMD18.
    // A failed read is retried a few times before the card is declared lost. Confirmed live: a
    // single data CRC error (DCRCFAIL, with the DMA still 5 words short) on the first read after
    // a mount, typically while the rest of the system is busy (boot, screen redraws), used to
    // drop the whole card - "Card lost", remount, and quite often the same error again, in a
    // loop. A plain retry of the same read does not work either: the card can be left in the
    // middle of sending the block, so it ignores the next read command (CTIMEOUT, "202005").
    // Hence the recovery before each retry: stop the transmission (CMD12, harmless when the card
    // is not sending), drain whatever is left in the FIFO and clear the static flags.
    for (f.attempt = 0;; f.attempt++)
    {
        if (f.attempt)
        {
            sd.Command_StopTransmission();
            while (sd.STA & SDMMC_STA_RXDAVL) { (void)sd.FIFO; }
            sd.ICR = 0x004007FF;    // all static flags
            if (!await(WaitCardReady, Timeout::Milliseconds(500)))
            {
                MYDBG("Card not ready for a read retry");
            }
        }

        sd.ConfigureDmaRead(dma, Buffer(f.buf, f.count * FF_MAX_SS), FF_MAX_SS);
        if (!(cr = f.count > 1 ? sd.Command_ReadMultipleBlock(f.sec * addrMul) : sd.Command_ReadSingleBlock(f.sec * addrMul)))
        {
            sd.AbortDmaTransfer(dma);
            MYDBG("Failed to start reading %u sector(s) at %X: %X (buf %p, attempt %d)", f.count, f.sec, cr, f.buf, f.attempt + 1);
            if (f.attempt + 1 < SD_READ_ATTEMPTS) { continue; }
            Status(STA_NODISK | STA_NOINIT);
            async_return(RES_ERROR);
        }

        MYTRACE("Reading %u sector(s) at %X, RS: %X", f.count, f.sec, sd.RESP1);
        // One timeout covering the whole (possibly multi-sector) transfer - scaled by how much data
        // is actually in flight rather than the fixed 1s that was plenty for a single sector but
        // would false-positive on a large one.
        if (!await(sd.WaitNotBusy, Timeout::Milliseconds(1000 + f.count * 20)))
        {
            sd.AbortDmaTransfer(dma);
            if (f.count > 1) { sd.Command_StopTransmission(); }
            MYDBG("Timeout while reading %u sector(s) at %X", f.count, f.sec);
            Status(STA_NODISK | STA_NOINIT);
            async_return(RES_ERROR);
        }

        auto dr = sd.CompleteDmaTransfer(dma);

        if (f.count > 1)
        {
            // CMD12 - the card does not stop sending on its own after a multi-block read
            sd.Command_StopTransmission();
        }

        if (!dr)
        {
            MYDBG("Error while reading %u sector(s) at %X: %X (buf %p, attempt %d)", f.count, f.sec, dr, f.buf, f.attempt + 1);
            if (f.attempt + 1 < SD_READ_ATTEMPTS) { continue; }
            Status(STA_NODISK | STA_NOINIT);
            async_return(RES_ERROR);
        }

        if (f.attempt)
        {
            MYDBG("Read %u sector(s) at %X succeeded on attempt %d", f.count, f.sec, f.attempt + 1);
        }
        MYTRACE("Read %u sector(s) at %X", f.count, f.sec);

        async_return(RES_OK);
    }
}
async_end

//! CMD13/SEND_STATUS, polled until the card's own READY_FOR_DATA bit (bit 8 of the R1 card
//! status, per the SD spec) comes back set. SDMMC::WaitNotBusy only reflects the SDMMC
//! peripheral's own TXACT/RXACT flags, which clear as soon as the *bus* phase of a transfer is
//! done - not when the card has actually finished committing a previous write internally.
//! Confirmed live: a WRITE_BLOCK/WRITE_MULTIPLE_BLOCK issued right after WaitNotBusy alone still
//! timed out (CTIMEOUT) completely, repeatedly, for ~30 seconds straight under sustained USB
//! write load - the peripheral itself was idle, but the card's own DAT0 line evidently was not.
//! This asks the card directly instead of inferring its state from the local peripheral.
async(SDMMCDriver::WaitCardReady, Timeout timeout)
async_def(
    Timeout timeout;
)
{
    f.timeout = timeout.MakeAbsolute();
    for (;;)
    {
        if (sd.Command_SendStatus(ci.rca.rca) && (sd.RESP1 & BIT(8)))
        {
            async_return(true);
        }
        if (f.timeout.Elapsed())
        {
            async_return(false);
        }
        async_yield();
    }
}
async_end

async(SDMMCDriver::Write, const void* buf, LBA_t sectorStart, size_t sectorCount)
async_def(
    LBA_t sec;
    const void* buf;
    size_t count;
)
{
    f.sec = sectorStart;
    f.buf = buf;
    f.count = sectorCount;

    if (!await(sd.WaitNotBusy, Timeout::Seconds(1)))
    {
        MYDBG("Timeout waiting for card to become available");
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    // See WaitCardReady's own comment - WaitNotBusy above only proves the SDMMC peripheral is
    // idle, not that the card itself is done with whatever it was doing (typically committing a
    // previous write to flash). Only needed here, not in Read() - this is specifically the
    // asymmetry confirmed live (writes timing out, reads not) that WaitNotBusy alone can't catch.
    if (!await(WaitCardReady, Timeout::Milliseconds(1000 + f.count * 20)))
    {
        MYDBG("Card status never reported ready before writing %u sector(s) at %X", f.count, f.sec);
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    ASSERT(!dma.IsEnabled());

    SDMMC::CommandResult cr;

    // One CMD25 (WRITE_MULTIPLE_BLOCK) + one DMA burst covering the whole request instead of a
    // fresh CMD24 (WRITE_BLOCK) - with its own DMA setup/teardown and command-response round trip
    // - for every single 512-byte sector. See the matching comment on Read() above; this is the
    // one that actually matters for USB file copies, since writing (not reading) is what a pilot
    // does when copying AGL files onto the card. CMD24 is kept for the single-sector case - it
    // completes on its own with no STOP_TRANSMISSION needed, unlike CMD25.
    if (!(cr = f.count > 1 ? sd.Command_WriteMultipleBlock(f.sec * addrMul) : sd.Command_WriteBlock(f.sec * addrMul)))
    {
        // Log cr itself, not a live sd.STA read - Command() already clears the command status
        // bits (ICR = SDMMC_STA_CMASK) before returning, so reading STA here always came back
        // near 0 regardless of what actually failed (CRC? timeout? RESPCMD mismatch?) - this was
        // a pre-existing blind spot in exactly the one message needed to diagnose a live write
        // failure (confirmed: every "Failed to start writing" line captured this way printed ": 0"
        // with no way to tell the real cause apart). Read()'s matching message already got this
        // right by capturing cr into a local - this just brings Write() in line with it.
        MYDBG("Failed to start writing %u sector(s) at %X: %X", f.count, f.sec, cr);
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    MYTRACE("Writing %u sector(s) from %p at %X, RS: %X", f.count, f.buf, f.sec, sd.RESP1);
    sd.ConfigureDmaWrite(dma, Span(f.buf, f.count * FF_MAX_SS), FF_MAX_SS);
    // One timeout covering the whole (possibly multi-sector) transfer - see the matching comment
    // on Read() above.
    if (!await(sd.WaitNotBusy, Timeout::Milliseconds(1000 + f.count * 20)))
    {
        sd.AbortDmaTransfer(dma);
        if (f.count > 1) { sd.Command_StopTransmission(); }
        MYDBG("Timeout while writing %u sector(s) at %X", f.count, f.sec);
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }

    auto dr = sd.CompleteDmaTransfer(dma);

    if (f.count > 1)
    {
        // CMD12 - the card does not stop accepting data on its own after a multi-block write
        sd.Command_StopTransmission();

        // CompleteDmaTransfer only confirms the *bus* phase ended (every byte clocked out) - the
        // card itself keeps DAT0 busy afterward while it actually commits all those blocks to
        // flash, which (unlike a read) takes real time and grows with how much was just written.
        // The single-block path never needed this: one sector's worth of programming finishes
        // fast enough that it was already done by the time anything asked again. Batching many
        // sectors per command (see MSC_IO_SECTORS) made that race actually losable - confirmed
        // live: writes reliably failed (dropped connection / device reset) once batched, reads
        // did not, which is exactly the asymmetry a missing post-write busy-wait would produce.
        // Re-using WaitNotBusy rather than inventing a new check - same primitive every other
        // card-ready wait in this file already uses.
        if (!await(sd.WaitNotBusy, Timeout::Milliseconds(1000 + f.count * 20)))
        {
            MYDBG("Card still busy after STOP_TRANSMISSION following %u-sector write at %X", f.count, f.sec);
            Status(STA_NODISK | STA_NOINIT);
            async_return(RES_ERROR);
        }
    }

    if (!dr)
    {
        MYDBG("Error while writing %u sector(s) at %X: %X", f.count, f.sec, sd.STA);
        Status(STA_NODISK | STA_NOINIT);
        async_return(RES_ERROR);
    }
    MYTRACE("Written %u sector(s) at %X: %X", f.count, f.sec, sd.STA);

    async_return(RES_OK);
}
async_end

//! What FatFs asks the disk itself. Mounting and file access need none of it; f_mkfs (formatting
//! the card in the vario) needs the size and the erase block - left empty, it formatted with
//! whatever happened to be in its variables.
async(SDMMCDriver::IoCtl, uint8_t cmd, void* buff)
async_def_sync()
{
    switch (cmd)
    {
        case GET_SECTOR_COUNT:
        {
            // CSD v1 (SDSC) counts its own read blocks of 512..2048 B, FatFs 512 B sectors
            auto& csd = ci.csd;
            *(LBA_t*)buff = csd.IsV2() ? LBA_t(csd.v2.BlockCount()) : LBA_t(csd.v1.BlockCount()) << (csd.v1.rdLen - 9);
            break;
        }

        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            break;

        case GET_BLOCK_SIZE:
            // in sectors: 4 MB, the allocation unit of SDHC/SDXC cards - the filesystem areas
            // aligned to it are erased and written by the card as whole units
            *(DWORD*)buff = 8192;
            break;

        default:
            // CTRL_SYNC: every write is complete when Write returns, nothing is cached here
            break;
    }
    async_return(RES_OK);
}
async_end

}
