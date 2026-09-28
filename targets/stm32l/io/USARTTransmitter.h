/*
 * Copyright (c) 2024 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * stm32l/io/USARTTransmitter.h
 */

#pragma once

#include <io/io.h>

#include <hw/USART.h>

namespace io
{

class USARTTransmitter
{
public:
    USARTTransmitter(USART* usart, Transmitter* transmitter)
        : usart(usart), transmitter(transmitter) {}

    USARTTransmitter(USART* usart)
        : usart(usart), transmitter(new USARTInterruptTransmitter(USARTInterrupt::Get(usart))) {}
    template<unsigned n> USARTTransmitter(_USART<n>* usart)
        : usart(usart), transmitter(new DMATransmitter(*usart->DmaTx(), &usart->TDR, DMADescriptor::PrioHigh)) {}

    ~USARTTransmitter() { delete transmitter; }

    operator PipeWriter() { return pipe; }

    void Start(size_t blockHint = 1)
    {
        transmitter->StartTransmitFromPipe(pipe);
    }

    //! Forwards to the underlying strategy's Reset() - see Transmitter::Reset / DMATransmitter::Reset.
    //! Call after force-disabling the USART peripheral (e.g. a bus-recovery reset), before
    //! re-enabling it, so a DMA-backed transmitter doesn't inherit a stale chained buffer.
    void Reset() { transmitter->Reset(); }

private:
    Pipe pipe;
    USART* usart;
    Transmitter* transmitter;
};

}
