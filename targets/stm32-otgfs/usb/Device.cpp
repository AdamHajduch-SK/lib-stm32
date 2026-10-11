/*
 * Copyright (c) 2019 triaxis s.r.o.
 * Licensed under the MIT license. See LICENSE.txt file in the repository root
 * for full license information.
 *
 * efm32-usb/usb/Device.cpp
 */

#include <usb/Device.h>

#include <hw/USB.h>

namespace usb
{

async(Device::Task)
async_def()
{
    // initialize the endpoint structures
    for (unsigned i = 1; i < USB_IN_ENDPOINTS; i++)
    {
        auto& ep = In(i);
        ep.owner = this;
        ep.ep = &usb->In(i);
    }

    for (unsigned i = 1; i < USB_OUT_ENDPOINTS; i++)
    {
        auto& ep = Out(i);
        ep.owner = this;
        ep.ep = &usb->Out(i);
    }

    auto irq = usb->IRQ();
    irq.Disable();
    irq.SetHandler(this, &Device::IRQHandler);
    irq.Priority(CORTEX_MAXIMUM_PRIO);

    await(usb->CoreEnable);
    PLATFORM_DEEP_SLEEP_DISABLE();  // no more sleeping while USB is active
    await(usb->CoreReset);

    EnableResetInterrupts();
    usb->EnablePHY();
    MODMASK(usb->GUSBCFG, USB_OTG_GUSBCFG_TRDT, 6 << USB_OTG_GUSBCFG_TRDT_Pos);

    USBDIAG("Setting device mode");
    usb->ForceDevice();
    await_mask_ms(usb->GINTSTS, USB_OTG_GINTSTS_CMOD, 0, 200);
    USBDIAG("Device mode set");

    // reset TX fifo pointers
    for (auto& txf: usb->DIEPTXF) { txf = 0; }
    usb->GCCFG |= USB_OTG_GCCFG_VBDEN;
    // restart PHY clock
    usb->PCGCCTL = 0;

    // configure device (Full speed, 80% periodic frame interval)
    usb->DeviceSetup(_USB::SpeedFull | _USB::PeriodicFrame80 | _USB::NonZeroLengthStatusOutHandshake);
    usb->TxFifoFlushAll();
    usb->RxFifoFlush();

    // clear all pending Device Interrupts
    usb->DEV.DIEPEMPMSK = 0U;
    usb->DEV.DOEPMSK = 0U;
    usb->DEV.DAINTMSK = 0U;

    for (auto& iep: usb->IEP)
    {
        if (iep.IsEnabled())
        {
            iep.DIEPCTL = ((&iep != &usb->IEP[0]) * USB_OTG_DIEPCTL_EPDIS) | USB_OTG_DIEPCTL_SNAK;
        }
        else
        {
            iep.DIEPCTL = 0;
        }
        iep.DIEPTSIZ = 0U;
        iep.DIEPINT = 0xFB7FU;
    }

    for (auto& oep: usb->OEP)
    {
        if (oep.IsEnabled())
        {
            oep.DOEPCTL = ((&oep != &usb->OEP[0]) * USB_OTG_DOEPCTL_EPDIS) | USB_OTG_DOEPCTL_SNAK;
        }
        else
        {
            oep.DOEPCTL = 0;
        }

        oep.DOEPTSIZ = 0U;
        oep.DOEPINT  = 0xFB7FU;
    }

    usb->DEV.DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

    /* Disable all interrupts. */
    usb->GINTMSK = 0U;

    /* Clear any pending interrupts */
    usb->GINTSTS = 0xBFFFFFFFU;

    /* Enable the common interrupts */
    usb->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

    /* Enable interrupts matching to the Device mode ONLY */
    usb->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                    USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                    USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
                    USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;

    usb->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);

    // DeviceDisconnect() first unconditionally, forcing a clean soft-disconnected starting point
    // regardless of whatever state the pull-up happened to be in - only then does startConnected
    // (see Start()'s own comment) decide whether to immediately connect or leave it off until the
    // caller explicitly calls Connect() later.
    usb->DeviceDisconnect();
    if (startConnected) { usb->DeviceConnect(); }
    USBDEBUG(startConnected ? "Device ready for connection" : "Device started, staying disconnected");

//    EnableInitInterrupts();
    irq.Enable();

    // handle interrupts
    for (;;)
    {
        await_signal(tasks);
        auto todo = kernel::AtomicExchange(tasks, Tasks::None);

        if (!!(todo & Tasks::Control))
            await(HandleControl);
    }
}
async_end

void Device::AllocateFifo()
{
    // the STM32 fifo is 1280 bytes (320 words)
    // the default allocation is two TX packets per 6 IN endpoint (32 words per endpoint) = 192 words
    // this leaves 128 words for RX

    constexpr unsigned rxWords = 64, txWords = 32;
    usb->RxFifoSetup(rxWords);
    usb->TxFifo0Setup(rxWords, txWords);

    for (unsigned i = 1, ptr = rxWords + txWords; i < USB_IN_ENDPOINTS; i++, ptr += txWords)
    {
        usb->TxFifoSetup(i, ptr, txWords);
    }
}

void Device::IRQHandler()
{
    auto core = usb->GINTSTS & usb->GINTMSK;
    uint32_t handled = 0;

    // start by receiving one packet, which can alter the remaining interrupt flags
    if (core & USB_OTG_GINTSTS_RXFLVL)
    {
        handled |= USB_OTG_GINTSTS_RXFLVL;
        HandleRx();
        // read core status again
        core = usb->GINTSTS & usb->GINTMSK;
    }

    // clear all interrupts that will be handled
    usb->GINTSTS = core;

    if (core & (USB_OTG_GINTSTS_USBRST | USB_OTG_GINTSTS_WKUINT | USB_OTG_GINTSTS_USBSUSP | USB_OTG_GINTSTS_IEPINT | USB_OTG_GINTSTS_OEPINT))
    {
        handled |= USB_OTG_GINTSTS_WKUINT | USB_OTG_GINTSTS_USBSUSP;

        if (suspended != !!(core & USB_OTG_GINTSTS_USBSUSP))
        {
            suspended = !suspended;
            USBDEBUG(suspended ? "SUSPEND" : "WAKEUP");
        }
    }

    if (core & USB_OTG_GINTSTS_USBRST)
    {
        handled |= USB_OTG_GINTSTS_USBRST;
        state = State::Default;
        ControlState(ControlState::Idle);
        USBDEBUG("RESET");
        callbacks->OnReset();

        usb->DeviceAddress(0);

        //  reset all endpoints
        usb->In(0).Reinitialize();
        usb->Out(0).Reinitialize();
        for (unsigned i = 1; i < USB_IN_ENDPOINTS; i++)
            In(i).Reset();
        for (unsigned i = 1; i < USB_OUT_ENDPOINTS; i++)
            Out(i).Reset();

        //  configure endpoint interrupts for SETUP
        usb->DEV.DAINTMSK |= BIT(USB_OTG_DAINTMSK_IEPM_Pos) | BIT(USB_OTG_DAINTMSK_OEPM_Pos);
        usb->DEV.DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM;
        usb->DEV.DIEPMSK |= USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_EPDM;

        //  configure FIFO
        AllocateFifo();

        //  configure OUT 0 for SETUP
        ControlSetup();
    }

    if (core & USB_OTG_GINTSTS_ENUMDNE)
    {
        // enumeration done
        handled |= USB_OTG_GINTSTS_ENUMDNE;
        fullSpeed = usb->DeviceFullSpeed();
        state = State::Default;
        USBDEBUG("ENUMDONE: %s Speed", fullSpeed ? "Full" : "Low");

        EnableAllInterrupts();
    }

    if (core & USB_OTG_GINTSTS_IEPINT)
    {
        handled |= USB_OTG_GINTSTS_IEPINT;
        HandleIn();
    }

    if (core & USB_OTG_GINTSTS_OEPINT)
    {
        handled |= USB_OTG_GINTSTS_OEPINT;
        HandleOut();
    }

    if (core & USB_OTG_GINTSTS_SRQINT)
    {
        handled |= USB_OTG_GINTSTS_SRQINT;
        USBDEBUG("SESSION REQUEST");
    }

    if (core & USB_OTG_GINTSTS_ESUSP)
    {
        handled |= USB_OTG_GINTSTS_ESUSP;
        USBDEBUG("EARLY SUSPEND");
    }

    if (core & USB_OTG_GINTSTS_OTGINT)
    {
        handled |= USB_OTG_GINTSTS_OTGINT;
        auto otg = usb->GOTGINT;
        usb->GOTGINT = otg;
        USBDEBUG("OTG INTERRUPT: %08X", otg);
    }

    if (core &= ~handled)
    {
        USBDEBUG("UNHANDLED INTERRUPT: %08X", core);
    }
}

inline void Device::HandleRx()
{
    auto grxstsp = usb->GRXSTSP;
    if (!grxstsp) { return; }

    STSPKT pkt = unsafe_cast<STSPKT>(grxstsp);

    switch (pkt.pktsts)
    {
        case PktSts::Nak:
            USBDEBUG("!! RX NAK");
            break;

        case PktSts::RxComplete:
        case PktSts::SetupComplete:
            // handled in EP interrupt
            break;

        case PktSts::SetupRx:
            if (ctrl.state != ControlState::Idle)
            {
                USBDEBUG("!! RX SETUP while not expecting data: %X %d", pkt, ctrl.state);
                ctrl.state = ControlState::Idle;
            }
            usb->RxFifoRead(0, ctrl.data, pkt.bcnt);
            USBDIAG("SETUP << %H", Span(ctrl.data, pkt.bcnt));
            break;

        case PktSts::Rx:
            if (!pkt.bcnt)
            {
                USBDIAG("RX << ACK");
                break;
            }

            USBDIAG("RX: EP%d %db %s %d", pkt.epnum, pkt.bcnt,
                STRINGS("DATA0", "?", "DATA1", "?")[pkt.dpid],
                pkt.frmnum);

            Out(pkt.epnum).PacketReceived(usb->DFIFO[pkt.epnum].u32, pkt.bcnt);
            break;

        default:
            USBDEBUG("!! INVALID RX PKTSTS: %d", pkt.pktsts);
            break;
    }
}

inline void Device::HandleOut()
{
    auto mask = usb->DeviceEndpointInterrupts();

    if (GETBIT(mask, USB_OTG_DAINT_OEPINT_Pos))
    {
        // endpoint 0 is handled separately
        HandleOutControl();
    }

    for (unsigned i = 1; i < USB_OUT_ENDPOINTS; i++)
    {
        if (!GETBIT(mask, USB_OTG_DAINT_OEPINT_Pos + i))
            continue;

        auto& ep = Out(i);
        auto status = ep.ep->InterruptClear();

        USBDIAG("EP OUT(%d) INT %08X", i, status);

        ep.HandleInterrupts(status);
    }
}

inline void Device::HandleIn()
{
    auto mask = usb->DeviceEndpointInterrupts();

    if (GETBIT(mask, USB_OTG_DAINT_IEPINT_Pos))
    {
        HandleInControl();
    }

    for (unsigned i = 1; i < USB_IN_ENDPOINTS; i++)
    {
        if (!GETBIT(mask, USB_OTG_DAINT_IEPINT_Pos + i))
            continue;

        auto& ep = In(i);
        auto status = ep.ep->InterruptClear();

        USBDIAG("EP IN(%d) INT %08X", i, status);

        ep.HandleInterrupts(status);
    }
}

inline void Device::HandleOutControl()
{
    auto& ep = usb->Out(0);
    auto status = ep.InterruptClear();

    USBDIAG("EP OUT(0) INT %08X", status);

    if (status & USB_OTG_DOEPINT_STUP)
    {
        SetupPacket pkt;

        if (status & USB_OTG_DOEPINT_B2BSTUP)
        {
            // TODO
            pkt = {};
        }
        else
        {
            // setup calculated from counter
            auto n = ep.SetupCount();
            pkt = ctrl.setupBuffer[n >= countof(ctrl.setupBuffer) ? 0 : countof(ctrl.setupBuffer) - 1 - n];
        }

        ctrl.setup = pkt;
        if (pkt.direction == pkt.DirOut && pkt.wLength)
        {
            // more data will follow, however
            // we can accept at most one full packet
            if (pkt.wLength > sizeof(ctrl.data))
            {
                // data too long, not supported
                USBDEBUG("!!! CONTROL request too long: %d > %d", pkt.wLength, sizeof(ctrl.data));
                ControlStall();
            }
            else
            {
                ep.ReceivePacket(64);
                ControlState(ControlState::DataRx);
            }
        }
        else
        {
            ControlState(ControlState::Processing);
            tasks = tasks | Tasks::Control;
        }
    }
    else if (status & USB_OTG_DOEPINT_XFRC)
    {
        switch (ctrl.state)
        {
        case ControlState::DataRx:
        {
            USBDIAG("EP0 data received: %H", Span(ctrl.data, ep.PacketSize() - ep.ReceivedLength()));
            if (ep.PacketSize() - ep.ReceivedLength() != ctrl.setup.wLength)
            {
                USBDEBUG("!!! Invalid SETUP OUT data length %d != %d", ep.PacketSize() - ep.ReceivedLength(), ctrl.setup.wLength);
                ControlStall();
            }
            else
            {
                ControlState(ControlState::Processing);
                tasks = tasks | Tasks::Control;
            }

            ControlState(ControlState::Processing);
            tasks = tasks | Tasks::Control;
            break;
        }

        case ControlState::StatusRx:
            USBDIAG("EP0 data confirmed");
            if (ep.ReceivedLength() != ep.PacketSize())
            {
                USBDEBUG("!!! Data received in STATUS: %H", Span(ctrl.data, ep.PacketSize() - ep.ReceivedLength()));
            }
            ControlSetup();
            break;

        default:
            break;
        }
    }
}

inline void Device::HandleInControl()
{
    // handle endpoint 0 separately
    auto& ep = usb->In(0);
    auto status = ep.InterruptClear();

    USBDIAG("EP IN(0) CTL %08X INT %08X", ep.DIEPCTL, status);

    if (status & USB_OTG_DIEPINT_XFRC)
    {
        switch (ctrl.state)
        {
            case ControlState::StatusTx:
                // prepare for next SETUP packet
                USBDIAG("Control ACK sent, waiting for next SETUP");
                ControlSetup();
                break;

            case ControlState::DataTx:
                if (ctrl.txRemain.Length())
                {
                    // continue transmitting
                    auto pkt = ctrl.txRemain.ConsumeLeft(ep.PacketSize());
                    ep.TransmitPacket(pkt.Length());
                    usb->TxFifoWrite(0, pkt.Pointer(), pkt.Length());
                }
                else
                {
                    USBDIAG("Control data transmitted, waiting for STATUS");
                    usb->Out(0).ReceivePacket(64);
                    ControlState(ControlState::StatusRx);
                }
                break;

            default:
                break;
        }
    }
}

void Device::ControlState(enum ControlState state)
{
    // DBGL("%c>%c", "IDPdSs!"[int(ctrl.state)], "IDPdSs!"[int(state)]);
    ctrl.state = state;
}

void Device::ControlSuccess(Span data)
{
    if (ctrl.hasResult)
    {
        USBDEBUG("!!! ControlResult() called multiple times");
        return;
    }

    if (ctrl.state != ControlState::Processing)
    {
        USBDEBUG("!!! ControlResult() called for interrupted setup");
        return;
    }

    ctrl.hasResult = true;

    if (ctrl.setup.direction == SetupPacket::DirOut)
    {
        if (data.Length())
        {
            USBDEBUG("!!! ControlResult() data providen in OUT (H2D) direction: %H", data);
            data = Span();
        }
        ControlState(ControlState::StatusTx);
    }
    else
    {
        // cut the data to the requested length
        data = data.Left(ctrl.setup.wLength);

        // data fitting in one packet is copied in the control data buffer
        // it is up to the application to provide a peristent location for data
        // longer than that
        if (data.Length() <= sizeof(ctrl.data))
        {
            data = data.CopyTo(ctrl.data);
        }

        auto send = data.ConsumeLeft(usb->In(0).PacketSize());
        ctrl.txRemain = data;
        ControlState(ControlState::DataTx);
        data = send;
    }

    if (data.Length())
        USBDIAG("CONTROL %08X >> %H", usb->In(0).DIEPCTL, data);
    else
        USBDIAG("CONTROL %08X ACK", usb->In(0).DIEPCTL);

    usb->In(0).TransmitPacket(data.Length());
    usb->TxFifoWrite(0, data.Pointer(), data.Length());
}

async(Device::HandleControl)
async_def()
{
    auto setup = ctrl.setup;
    ctrl.hasResult = false;

    switch (setup.type)
    {
        case SetupPacket::TypeStandard:
            switch (setup.bRequest)
            {
                case SetupPacket::StdGetStatus: HandleControlGetStatus(setup); break;
                case SetupPacket::StdClearFeature: HandleControlFeature(setup, false); break;
                case SetupPacket::StdSetFeature: HandleControlFeature(setup, true); break;
                case SetupPacket::StdSetAddress: HandleControlSetAddress(setup); break;
                case SetupPacket::StdGetDescriptor: await(HandleControlGetDescriptor, setup); break;
                case SetupPacket::StdGetConfiguration: HandleControlGetConfiguration(setup); break;
                case SetupPacket::StdSetConfiguration: await(HandleControlSetConfiguration, setup);
                default: break;
            }
            break;

        default:
            if (ctrl.setup.direction == SetupPacket::DirOut)
                await(callbacks->HandleControl, ctrl.setup, Span(ctrl.data, ctrl.setup.wLength));
            else
                await(callbacks->HandleControl, ctrl.setup, Span());
            break;
    }

    if (!ctrl.hasResult)
    {
        USBDEBUG("!!! CONTROL %H %H unsupported", Span(setup), Span(ctrl.data, setup.wLength));
        ControlStall();
    }
}
async_end

void Device::HandleControlGetStatus(SetupPacket setup)
{
    if (setup.direction == SetupPacket::DirIn && !setup.wValue && setup.wLength == 2)
    {
        switch (setup.recipient)
        {
            case SetupPacket::RecipientDevice:
                if (setup.wIndex == 0)
                {
                    if (config)
                    {
                        ctrl.data16[0] = !!(config->bmAttributes & ConfigAttributes::SelfPowered) * BIT(0) |
                            !!(config->bmAttributes & ConfigAttributes::RemoteWakeup) * BIT(1);
                    }
                    else
                    {
                        ctrl.data16[0] = 0;
                    }
                    ControlSuccess(ctrl.data16[0]);
                    return;
                }
                break;

            default:
                break;
        }
    }

    USBDEBUG("!!! GET_STATUS packet corrupted");
}

void Device::HandleControlFeature(SetupPacket setup, bool set)
{
    if (setup.wLength == 0)
    {
        switch (setup.recipient)
        {
            case SetupPacket::RecipientDevice:
                if (setup.wIndex != 0)
                    break;

                if (!(config && state == State::Configured))
                    return; // not supported right now

                if (setup.wValue == SetupPacket::FeatureDeviceRemoteWakeup)
                {
                    if (!!(config->bmAttributes & ConfigAttributes::RemoteWakeup))
                    {
                        USBDIAG("Feature REMOTE_WAKEUP %d", set);
                        remoteWakeupEnabled = set;
                        ControlSuccess();
                    }
                }
                return;

            case SetupPacket::RecipientEndpoint:
            {
                unsigned num = setup.wIndex & 0x7F;
                bool in = setup.wIndex & 0x80;
                if (num == 0)
                    return; // no features on EP0

                if (num > (in ? USB_IN_ENDPOINTS : USB_OUT_ENDPOINTS))
                    return; // endpoint number out of range

                if (setup.wValue == SetupPacket::FeatureEndpointHalt)
                {
                    USBDIAG("%s(%d) STALL: %d", in ? "IN" : "OUT", num, set);

                    if (in)
                        usb->In(num).Stall(set);
                    else
                        usb->Out(num).Stall(set);
                    ControlSuccess();
                }
                return;
            }

            default:
                // unsupported recipient
                return;
        }
    }

    USBDEBUG("!!! %s_FEATURE packet corrupted", set ? "SET" : "CLEAR");
}

void Device::HandleControlSetAddress(SetupPacket setup)
{
    if (setup.recipient != SetupPacket::RecipientDevice ||
        setup.direction != SetupPacket::DirOut ||
        setup.wIndex || setup.wLength || setup.wValue > 127)
    {
        USBDEBUG("!!! SET_ADDRESS packet corrupted");
        return;
    }

    USBDIAG("SET_ADDRESS %04X", setup.wValue);
    usb->DeviceAddress(setup.wValue);
    if (state == State::Default && setup.wValue)
        state = State::Addressed;
    else if (state == State::Addressed && !setup.wValue)
        state = State::Default;

    ControlSuccess();
}

async(Device::HandleControlGetDescriptor, SetupPacket setup)
async_def(
    size_t len;
)
{
    USBDIAG("GET_DESCRIPTOR %04X ID %d len %d", setup.wValue, setup.wIndex, setup.wLength);
    if (setup.recipient != SetupPacket::RecipientDevice ||
        setup.direction != SetupPacket::DirIn)
    {
        USBDEBUG("!!! GET_DESCRIPTOR packet corrupted");
        async_return(false);
    }

    // give the callback a chance to handle the request
    f.len = await(callbacks->GetDescriptor, setup, Buffer(ctrl.data).RemoveLeft(2));

    Span data;

    if (f.len)
    {
        ctrl.data[0] = f.len + 2;
        ctrl.data[1] = uint8_t(setup.descriptorType);
        data = Span(ctrl.data, f.len + 2);
    }
    else switch (setup.descriptorType)
    {
        case DescriptorType::Device:
            if (setup.descriptorIndex != 0)
            {
                USBDEBUG("!!! GET_DEVICE_DESCRIPTOR %d > 0", setup.descriptorIndex);
                async_return(false);
            }

            data = deviceDescriptor;
            break;

        case DescriptorType::Config:
        {
            if (setup.descriptorIndex >= configDescriptorCount)
            {
                USBDEBUG("!!! GET_CONFIG_DESCRIPTOR %d >= %d", setup.descriptorIndex, configDescriptorCount);
                async_return(false);
            }

            auto desc = configDescriptors[setup.descriptorIndex];
            data = Span(desc, desc->wTotalLength);
            break;
        }

        case DescriptorType::String:
        {
            auto desc = stringDescriptors;
            int nValid = 0;

            while (desc && nValid < setup.descriptorIndex)
            {
                desc = desc->Next();
                if (desc->len)
                    nValid++;
                else
                    desc = NULL;
            }

            if (!desc)
            {
                USBDEBUG("!!! GET_STRING_DESCRIPTOR %d >= %d", setup.descriptorIndex, nValid);
                async_return(false);
            }

            data = Span(desc, desc->len);
            break;
        }

        default:
            break;
    }

    ControlSuccess(data);
    async_return(true);
}
async_end

void Device::HandleControlGetConfiguration(SetupPacket setup)
{
    USBDIAG("GET_CONFIGURATION");
    if (setup.recipient != SetupPacket::RecipientDevice ||
        setup.direction != SetupPacket::DirIn ||
        setup.wIndex || setup.wValue || setup.wLength != 1)
    {
        USBDEBUG("!!! GET_CONFIGURATION packet corrupted");
        return;
    }

    ctrl.data[0] = config ? config->bConfigurationValue : 0;
    ControlSuccess(ctrl.data[0]);
}

async(Device::HandleControlSetConfiguration, SetupPacket setup)
async_def()
{
    USBDIAG("SET_CONFIGURATION %d", setup.wValue);
    if (setup.recipient != SetupPacket::RecipientDevice ||
        setup.wIndex || setup.wLength || setup.wValue > 255)
    {
        USBDEBUG("!!! SET_CONFIGURATION packet corrupted");
        async_return(false);
    }

    if (setup.wValue && (state == State::Addressed || state == State::Configured))
    {
        if (auto cfg = FindConfiguration(setup.wValue))
        {
            config = cfg;
            remoteWakeupEnabled = !!(config->bmAttributes & ConfigAttributes::RemoteWakeup);
        }
        else
        {
            USBDEBUG("!!! SET_CONFIGURATION called with unknown configuration value %d", setup.wValue);
            async_return(false);
        }
        state = State::Configured;
    }
    else if (state == State::Configured)
    {
        config = NULL;
        state = State::Addressed;
    }

    await(ConfigureEndpoints);
    await(callbacks->OnConfigured, config);
    ControlSuccess();
}
async_end

const ConfigDescriptorHeader* Device::FindConfiguration(uint8_t value)
{
    for (unsigned i = 0; i < configDescriptorCount; i++)
    {
        if (configDescriptors[i]->bConfigurationValue == value)
            return configDescriptors[i];
    }

    return NULL;
}

async(Device::ConfigureEndpoints)
async_def(
    unsigned i;
    const ConfigDescriptorHeader* config;
    const EndpointDescriptor* epConfig;
)
{
    f.config = config;

    if (f.config == NULL)
    {
        // deactivate all endpoints
        for (f.i = 1; f.i < USB_IN_ENDPOINTS; f.i++)
            await(In(f.i).Configure, NULL);
        for (f.i = 1; f.i < USB_OUT_ENDPOINTS; f.i++)
            await(Out(f.i).Configure, NULL);
    }
    else
    {
        // configure or reconfigure all endpoints
        for (f.i = 1; f.i < USB_IN_ENDPOINTS; f.i++)
        {
            f.epConfig = f.config->FindEndpoint(0x80 | f.i);
            await(In(f.i).Configure, f.epConfig);
        }

        for (f.i = 1; f.i < USB_OUT_ENDPOINTS; f.i++)
        {
            f.epConfig = f.config->FindEndpoint(f.i);
            await(Out(f.i).Configure, f.epConfig);
        }
    }
}
async_end

}
