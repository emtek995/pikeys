#![no_std]
#![no_main]

mod keyboard;

use panic_halt as _;
use rtic::app;

#[app(device = rp_pico::hal::pac, peripherals = true)]
mod pikeys {
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico::*;
    use usb_device::{class_prelude::*, prelude::*};

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    // "free" codes for testing
    const VID: u16 = 0xFC32;
    const PID: u16 = 0x1287;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut reset = c.device.RESETS;
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut reset,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let timer_mono = Rp2040Monotonic::new(c.device.TIMER);

        let sio = hal::Sio::new(c.device.SIO);

        let _pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut reset,
        );

        // Setup the usb bus
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut reset,
        ));

        let usb_bus = unsafe {
            USB_BUS = Some(usb_bus);
            USB_BUS.as_ref().unwrap()
        };

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer("derp")
            .product("Derpboard")
            .serial_number(core::env!("CARGO_PKG_VERSION"))
            .build();

        (
            Shared { usb_device },
            Local {},
            init::Monotonics(timer_mono),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USBCTRL_IRQ, priority = 2, shared = [usb_device])]
    fn usbctrl(c: usbctrl::Context) {
        let mut usb_device = c.shared.usb_device;
        usb_device.lock(|usb_device| {
            usb_device.poll(&mut []);
        });
    }
}
