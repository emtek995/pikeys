#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;

#[app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [DMA_IRQ_0])]
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
    struct Shared {}

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

        let _usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer("derp")
            .product("Derpboard")
            .serial_number(core::env!("CARGO_PKG_VERSION"))
            .build();

        (Shared {}, Local {}, init::Monotonics(timer_mono))
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
