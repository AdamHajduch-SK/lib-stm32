/*
 * Copyright (c) 2025 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * stm32l/fatfs/SDMMCDriver.h
 */

#pragma once

#include <fatfs/fatfs.h>

#include <hw/SDMMC.h>

namespace fatfs
{

class SDMMCDriver : public DiskDriver
{
public:
    SDMMCDriver(SDMMC* sd, DMAChannel* dma)
        : sd(*sd), dma(*dma) {}

    static void Register(SDMMC* sd);

    virtual async(Init) final override;
    async(Test);

    const SDMMC::CardInfo& CardInfo() const { return ci; }

    //! Makes the next Init() start from scratch (controller reset + card identification) - for
    //! when the card's state is unknown, e.g. after its power was cut or another host used it
    void ForceReinit() { Status(STA_NODISK | STA_NOINIT); }

protected:
    virtual async(Read, void* buf, LBA_t sectorStart, size_t sectorCount) final override;
    virtual async(Write, const void* buf, LBA_t sectorStart, size_t sectorCount) final override;
    virtual async(IoCtl, uint8_t cmd, void* buff) final override;

private:
    SDMMC& sd;
    DMAChannel& dma;
    SDMMC::CardInfo ci;
    unsigned addrMul;

    //! Polls the card's own status (CMD13/SEND_STATUS) until its READY_FOR_DATA bit is set, or
    //! @p timeout elapses - see the long comment on its own definition in SDMMCDriver.cpp for why
    //! this exists alongside SDMMC::WaitNotBusy.
    async(WaitCardReady, Timeout timeout);
};

}
