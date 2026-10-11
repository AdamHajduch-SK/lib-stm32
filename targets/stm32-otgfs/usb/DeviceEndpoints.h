/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * efm32-usb/usb/DeviceEndpoints.h
 */

#pragma once

#include <kernel/kernel.h>
#include <io/io.h>

#include <usb/Descriptors.h>

#include <hw/USB.h>

namespace usb
{

class DeviceInEndpoint
{
    friend class Device;

    class Device* owner;
    USBInEndpoint* ep;
    const uint32_t* txPtr;
    size_t txWords;
    bool transferComplete, endpointDisabled;

    void Reset();
    async(Configure, const EndpointDescriptor* config);
    void HandleInterrupts(uint32_t status);

public:
    // low-level packet interface
    async(SendPacket, Span data, Timeout timeout = Timeout::Infinite);
};

class DeviceOutEndpoint
{
    friend class Device;

    class Device* owner;
    USBOutEndpoint* ep;
    Buffer packetBuffer;
    //! bytes received so far into packetBuffer by a multi-packet ReceiveTransfer
    size_t rxDone;
    //! the current receive spans several packets (ReceiveTransfer), not just one (ReceivePacket)
    bool multi;
    bool transferComplete;
    bool endpointDisabled;

    void Reset();
    async(Configure, const EndpointDescriptor* config);
    void PacketReceived(volatile uint32_t& fifo, size_t size);
    void HandleInterrupts(uint32_t status)
    {
        if (status & USB_OTG_DOEPINT_XFRC) { transferComplete = true; }
        if (status & USB_OTG_DOEPINT_EPDISD) { endpointDisabled = true; }
    }

public:
    bool IsActive() const { return ep->IsActive(); }
    // low-level packet interface
    async(ReceivePacket, Buffer buffer, Timeout timeout = Timeout::Infinite);
    //! Receives up to buffer.Length() bytes as one hardware transfer (many packets, no software
    //! round trip between them) - ends early on a short packet. The length should be exactly what
    //! the host is going to send, or a following unrelated packet ends up in the buffer too.
    async(ReceiveTransfer, Buffer buffer, Timeout timeout = Timeout::Infinite);
};

}
