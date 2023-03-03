#![no_std]
#![no_main]

mod layout;

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod pikeys {
    use cortex_m::prelude::{
        _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable,
    };
    use fugit::ExtU32;
    use hal::gpio::DynPin;
    use keyberon::{
        debounce::Debouncer,
        key_code::KbHidReport,
        layout::{Event, Layout},
        matrix::Matrix,
    };
    use rp_pico::hal::{self, timer::Alarm, usb::UsbBus};
    use usb_device::{class_prelude::*, prelude::UsbDeviceState};

    // "free" codes for testing
    const VID: u16 = 0xFC32;
    const PID: u16 = 0x1287;

    const SCAN_TIME_US: u32 = 1000;
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            rp2040_hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        layout: Layout<10, 4, 1, ()>,
    }

    #[local]
    struct Local {
        debouncer: Debouncer<[[bool; 10]; 4]>,
        matrix: Matrix<DynPin, DynPin, 10, 4>,
        watchdog: hal::watchdog::Watchdog,
        alarm: hal::timer::Alarm0,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Safety: Just reset. Have to reset spinlocks cause they don't
        // always from a warm boot
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut reset = c.device.RESETS;
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
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

        let sio = hal::Sio::new(c.device.SIO);

        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut reset,
        );

        let matrix: Matrix<DynPin, DynPin, 10, 4> = Matrix::new(
            [
                pins.gpio2.into_pull_up_input().into(),
                pins.gpio3.into_pull_up_input().into(),
                pins.gpio4.into_pull_up_input().into(),
                pins.gpio5.into_pull_up_input().into(),
                pins.gpio6.into_pull_up_input().into(),
                pins.gpio7.into_pull_up_input().into(),
                pins.gpio8.into_pull_up_input().into(),
                pins.gpio9.into_pull_up_input().into(),
                pins.gpio10.into_pull_up_input().into(),
                pins.gpio11.into_pull_up_input().into(),
            ],
            [
                pins.gpio18.into_push_pull_output().into(),
                pins.gpio19.into_push_pull_output().into(),
                pins.gpio20.into_push_pull_output().into(),
                pins.gpio21.into_push_pull_output().into(),
            ],
        )
        .unwrap();

        // delay to make sure hardware is good to go
        for _ in 1..1000 {
            cortex_m::asm::nop();
        }

        let mut timer = hal::Timer::new(c.device.TIMER, &mut reset);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.micros());
        alarm.enable_interrupt();

        // Setup the usb bus
        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut reset,
        ));
        // Safety: Bus isn't being used yet
        unsafe {
            USB_BUS = Some(usb_bus);
        }

        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });
        let layout = Layout::new(&crate::layout::LAYERS);
        let debouncer = Debouncer::new([[false; 10]; 4], [[false; 10]; 4], 5);

        watchdog.start(10_000.micros());

        (
            Shared {
                usb_dev,
                usb_class,
                layout,
            },
            Local {
                debouncer,
                matrix,
                watchdog,
                alarm,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        (c.shared.usb_dev, c.shared.usb_class).lock(|usb_dev, usb_class| {
            if usb_dev.poll(&mut [usb_class]) {
                usb_class.poll();
            }
        })
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<Event>) {
        match event {
            None => match c.shared.layout.lock(|l| l.tick()) {
                _ => (),
            },
            Some(e) => {
                c.shared.layout.lock(|l| l.event(e));
                return;
            }
        };
        let report: KbHidReport = c.shared.layout.lock(|l| l.keycodes().collect());
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(binds = TIMER_IRQ_0, priority = 1, local = [matrix, debouncer, watchdog, alarm])]
    fn timer_irq(c: timer_irq::Context) {
        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US.micros());
        c.local.watchdog.feed();

        let keys_pressed = c.local.matrix.get().unwrap();
        let events = c.local.debouncer.events(keys_pressed);

        for event in events {
            handle_event::spawn(Some(event)).unwrap();
        }
        handle_event::spawn(None).unwrap();
    }
}
