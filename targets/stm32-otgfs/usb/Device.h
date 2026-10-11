/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * efm32-usb/usb/Device.h
 */

#pragma once

#include <kernel/kernel.h>

#include <hw/USB.h>

#include <usb/Packets.h>
#include <usb/Descriptors.h>
#include <usb/DeviceEndpoints.h>

namespace usb
{

class DeviceCallbacks
{
    friend class Device;

protected:
    virtual async(HandleControl, SetupPacket pkt, Span packet) async_def_return(false);
    virtual async(GetDescriptor, SetupPacket pkt, Buffer buf) async_def_return(0);
    virtual async(OnConfigured, const ConfigDescriptorHeader* config) async_def_return(false);

    virtual void OnReset() {}
};

class Device
{
public:
    template<typename TStringTable, typename... TConfig> Device(const DeviceDescriptor& deviceDescriptor,
        const TStringTable& strings,
        const TConfig&... configs) :
        deviceDescriptor(deviceDescriptor),
        stringDescriptors(strings),
        configDescriptorCount(sizeof...(TConfig))
    {
        static const ConfigDescriptorHeader* configArray[] = { &configs... };
        configDescriptors = configArray;
    }

    //! @p connect false starts the device fully soft-disconnected (no D+ pull-up) instead of the
    //! default connect-on-boot - for a device class that wants to control exactly when the host
    //! first sees it, like MSC's own "stay off the bus until the pilot opens USB Storage" policy.
    //! Trying to achieve that by calling Disconnect() right after Start() instead does NOT work:
    //! Task() runs as its own background coroutine (Start() only schedules it, same as every
    //! kernel::Task::Run - see MSC.cpp's own long-form comment on that exact point elsewhere),
    //! and does not reach its own unconditional DeviceConnect() until several awaited steps
    //! later (core enable, core reset, a bounded mode-switch wait) - by which point any Disconnect()
    //! already issued from the caller's own constructor has long since been overwritten. Confirmed
    //! live: MSC's "default inhibited" never actually took effect, the device kept enumerating on
    //! its own in the background even with the USB Storage screen never opened.
    void Start(DeviceCallbacks& callbacks, bool connect = true)
    {
        this->callbacks = &callbacks;
        startConnected = connect;
        kernel::Task::Run(this, &Device::Task);
    }

    //! Soft-disconnect (D+ pull-up removed) - the host sees exactly what it would see if the
    //! cable were physically unplugged; VBUS/charging is unaffected.
    void Disconnect() { usb->DeviceDisconnect(); }
    //! Re-enables the soft-connect pull-up after Disconnect().
    void Connect() { usb->DeviceConnect(); }

    void ControlSuccess(Span span = Span());

    DeviceInEndpoint& In(unsigned n) { ASSERT(n > 0 && n < USB_IN_ENDPOINTS); return in[n - 1]; }
    DeviceOutEndpoint& Out(unsigned n) { ASSERT(n > 0 && n < USB_OUT_ENDPOINTS); return out[n - 1]; }

private:
    enum struct State : uint8_t
    {
        None, Default, Addressed, Configured,
    };

    enum struct ControlState : uint8_t
    {
        Idle,
        DataRx,
        Processing,
        DataTx,
        StatusRx,
        StatusTx,
        Stall,
    };

    enum struct Tasks : uint8_t
    {
        None = 0,
        Control = 1,
    };

    enum struct PktSts : uint32_t
    {
        Nak = 1,
        Rx = 2,
        RxComplete = 3,
        SetupComplete = 4,
        SetupRx = 6,
    };

    struct STSPKT
    {
        uint32_t epnum: 4;
        uint32_t bcnt: 11;
        uint32_t dpid: 2;
        PktSts pktsts: 4;
        uint32_t frmnum: 4;
        uint32_t: 2;
        uint32_t stsphst: 1;
        uint32_t: 4;
    };

    DECLARE_FLAG_ENUM(Tasks);

    enum
    {
        USB_OTG_GINTMSK_RESET = USB_OTG_GINTMSK_OTGINT | USB_OTG_GINTSTS_MMIS,
        USB_OTG_GINTMSK_INIT = USB_OTG_GINTMSK_RESET | USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_ESUSPM | USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_RXFLVLM,
        USB_OTG_GINTMSK_ALL = USB_OTG_GINTMSK_INIT | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_WUIM,
    };

    _USB* usb = USB;
    //! See Start()'s own comment - whether Task() connects at the end of its init sequence.
    bool startConnected = true;
    const ConfigDescriptorHeader* config = NULL;
    State state = State::None;
    Tasks tasks = Tasks::None;
    DeviceCallbacks* callbacks = NULL;

    struct
    {
        Span txRemain;
        SetupPacket setup;
        union
        {
            SetupPacket setupBuffer[3];
            uint8_t data[64];
            uint16_t data16[32];
            uint32_t data32[16];
        };
        ControlState state = ControlState::Idle;
        bool hasResult;
    } ctrl;
    bool suspended = false;
    bool remoteWakeupEnabled = false;
    bool fullSpeed;
    const DeviceDescriptor& deviceDescriptor;
    const ConfigDescriptorHeader** configDescriptors;
    const StringDescriptor* stringDescriptors;
    size_t configDescriptorCount;
    // control structures excluding EP0
    DeviceInEndpoint in[USB_IN_ENDPOINTS - 1];
    DeviceOutEndpoint out[USB_OUT_ENDPOINTS - 1];

    async(Task);
    void IRQHandler();

    void EnableResetInterrupts() { usb->GINTMSK = USB_OTG_GINTMSK_RESET; }
    void EnableInitInterrupts() { usb->GINTMSK = USB_OTG_GINTMSK_INIT; }
    void EnableAllInterrupts() { usb->GINTMSK = USB_OTG_GINTMSK_ALL; }
    void ControlSetup() { usb->Out(0).ConfigureSetup(3); ControlState(ControlState::Idle); }
    void ControlStall() { usb->Out(0).Stall(); usb->In(0).Stall(); ControlSetup(); ControlState(ControlState::Stall);  }
    void ControlState(ControlState state);

    void AllocateFifo();
    void HandleRx();
    void HandleOut();
    void HandleOutControl();
    void HandleIn();
    void HandleInControl();

    async(HandleControl);
    void HandleControlStandard(SetupPacket setup);
    void HandleControlGetStatus(SetupPacket setup);
    void HandleControlFeature(SetupPacket setup, bool set);
    void HandleControlSetAddress(SetupPacket setup);
    async(HandleControlGetDescriptor, SetupPacket setup);
    void HandleControlGetConfiguration(SetupPacket setup);
    async(HandleControlSetConfiguration, SetupPacket setup);

    const ConfigDescriptorHeader* FindConfiguration(uint8_t value);

    async(ConfigureEndpoints);
    async(ConfigureEndpoint, DeviceInEndpoint& ep, const EndpointDescriptor* cfg);
    async(ConfigureEndpoint, DeviceOutEndpoint& ep, const EndpointDescriptor* cfg);
    void ReconfigureInterface(int interface, int altConfig);
};

DEFINE_FLAG_ENUM(Device::Tasks);

}
