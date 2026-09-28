/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * stm32l/io/DMATransmitter.h
 *
 * A transmitter strategy using DMA buffer linking.
 *
 * Recommended for most scenarios.
 */

#pragma once

#include <io/Transmitter.h>

#include <hw/DMA.h>

namespace io
{

class DMATransmitter : public Transmitter
{
public:
    DMATransmitter(DMAChannel& dma, volatile void* destination, DMADescriptor::Flags flags = {})
        : dma(dma)
    {
        dma.CPAR = uint32_t(destination);
        dma.CCR = flags | DMADescriptor::M2P | DMADescriptor::IncrementMemory | DMADescriptor::UnitByte;
    }

public:
    // Root cause of a bug caught live: TryLinkBuffer() chains the next buffer onto whatever
    // transfer is currently running, to be picked up by that transfer's completion IRQ
    // (LinkHandler). If the underlying peripheral is force-disabled while a buffer is chained
    // that way (e.g. mid bus-recovery reset - this channel's own EN bit is untouched by that, so
    // IsEnabled() still reports true, but the peripheral has stopped feeding it and the transfer
    // that was supposed to complete and trigger LinkHandler() never will), the chained link is
    // left dangling forever: every later TryLinkBuffer() sees a stale, non-matching link and
    // silently refuses to accept any new buffer (returns 0) - permanently, with no error anywhere.
    // Call this right after force-disabling the peripheral (before re-enabling it) so the next
    // TryLinkBuffer() starts clean instead of inheriting that stale state.
    virtual void Reset() override { dma.Disable(); dma.Unlink(); }

protected:
    virtual size_t TryAddBlock(Span block);
    virtual const char* GetReadPointer();
    virtual async_once(Wait, const char* current, Timeout timeout = Timeout::Infinite);

private:
    DMAChannel& dma;
};

}