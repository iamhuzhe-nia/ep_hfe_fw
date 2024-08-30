#![no_std]
#![no_main]
#![allow(non_snake_case)]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::pwm::SetDutyCycle;

use hal::gpio::*;
use hal::multicore::{Multicore, Stack};
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    watchdog::Watchdog,
    Sio,
};
use rp2040_hal as hal;
//use rp2040_hal::pwm::B;
//use hal::{pwm::{InputHighRunning, Slices}};

// // USB Device support
// use usb_device::{class_prelude::*, prelude::*};

// // USB Communications Class Device support
// use usbd_serial::SerialPort;

// use corncobs::*;

use rp2040_boot2;
#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// const FIRMWARE_VERSION: [u8; 3] = [1, 0, 0];
/// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;

/// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 25000;

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    let channel_b = &mut pwm.channel_b;
    channel_b.output_to(pins.gpio9);

    // monitor battery charing pin
    let mut charging_pin: Pin<bank0::Gpio2, FunctionSio<SioInput>, PullNone> =
        pins.gpio2.into_floating_input();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    let mut one_shot: bool = true;
    loop {
        // toggle LED when charging not done
        if charging_pin.is_low().unwrap() {
            one_shot = true;
            // Ramp brightness up
            for i in LOW..=HIGH {
                delay.delay_us(8);
                let _ = channel_b.set_duty_cycle(i);
            }

            // Ramp brightness down
            for i in (LOW..=HIGH).rev() {
                delay.delay_us(8);
                let _ = channel_b.set_duty_cycle(i);
            }
        } else {
            if one_shot {
                one_shot = false;
                channel_b.set_duty_cycle(60000).unwrap();
                delay.delay_ms(5_000);
            } else {
                channel_b.set_duty_cycle_fully_off().unwrap();
            }
        }
        // if let Some(word) =  sio.fifo.read(){
        //     channel_a.set_duty_cycle(word as u16).unwrap();
        // }

        delay.delay_ms(800); // 0.125Hz freq.
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let sys_freq = clocks.system_clock.freq().to_Hz();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq)
    });

    let mut pwm_slices2 = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_g = pins.gpio1.into_push_pull_output();
    led_g.set_high().unwrap(); // turn on green LED when power-on

    //let mut motor = pins.gpio0.into_push_pull_output();
    //motor.set_high().unwrap(); // turn on motor for testing

    // use PWM to drive motor

    let pwm_motor = &mut pwm_slices2.pwm0;
    pwm_motor.set_ph_correct();
    pwm_motor.enable();

    let channel_a = &mut pwm_motor.channel_a;
    channel_a.output_to(pins.gpio0);
    channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off

    // channel_a.set_duty_cycle(5).unwrap();
    loop {
        //notification 1
        // on 200ms
        channel_a.set_duty_cycle(10).unwrap();
        delay.delay_ms(200);
        // off 100ms
        channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off
        delay.delay_ms(100);
        // on 500ms
        channel_a.set_duty_cycle(10).unwrap();
        delay.delay_ms(500);

        channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off

        delay.delay_ms(60_000);

        //notification 2
        for _ in 0..3 {
            // on 200ms
            channel_a.set_duty_cycle(20).unwrap();
            delay.delay_ms(200);
            // off 100ms
            channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off
            delay.delay_ms(100);
        }

        delay.delay_ms(60_000);

        //notification 3
        for _ in 0..2 {
            // on 100ms
            channel_a.set_duty_cycle(20).unwrap();
            delay.delay_ms(100);
            // off 100ms
            channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off
            delay.delay_ms(100);
        }

        delay.delay_ms(300);

        for _ in 0..2 {
            // on 100ms
            channel_a.set_duty_cycle(20).unwrap();
            delay.delay_ms(100);
            // off 100ms
            channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off
            delay.delay_ms(100);
        }

        delay.delay_ms(60_000);

        //notification 4
        // on 500ms
        channel_a.set_duty_cycle(10).unwrap();
        delay.delay_ms(500);
        // off 100ms
        channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off
        delay.delay_ms(100);
        // on 200ms
        channel_a.set_duty_cycle(10).unwrap();
        delay.delay_ms(200);
        channel_a.set_duty_cycle_fully_off().unwrap(); // turn motor off

        delay.delay_ms(60_000);
    }

    // // Set up the USB driver
    // let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
    //     pac.USBCTRL_REGS,
    //     pac.USBCTRL_DPRAM,
    //     clocks.usb_clock,
    //     true,
    //     &mut pac.RESETS,
    // ));

    // // Set up the USB Communications Class Device driver
    // let mut serial = SerialPort::new(&usb_bus);

    // // Create a USB device with a fake VID and PID
    // let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
    // .strings(&[StringDescriptors::default()
    //     .manufacturer("Nia Therapeutics")
    //     .product("serial port")
    //     .serial_number("NIA-0001")]).unwrap()
    //     .device_class(2) // from: https://www.usb.org/defined-class-codes
    //     .build();

    // // incoming COBS from serial
    // enum Fsm {
    //     IDLE,
    //     COBS,
    // }
    // let mut state = Fsm::IDLE;
    // let mut serial_buffer_index = 0usize;
    // let mut serial_buffer = [0u8; 10];
    // let mut decoded_data = [0u8; 8];
    // let mut encoded = [0u8; 40];

    // loop {
    //     match state {
    //         Fsm::IDLE => {
    //             if usb_dev.poll(&mut [&mut serial]) {
    //                 let mut buf = [0u8; 64];
    //                 match serial.read(&mut buf) {
    //                     Err(_e) => {
    //                         // Do nothing
    //                     }
    //                     Ok(0) => {
    //                         // Do nothing
    //                     }
    //                     Ok(count) => {

    //                         serial_buffer_index = 0;

    //                         for x in &buf[..count] {
    //                             serial_buffer[serial_buffer_index] = *x;
    //                             serial_buffer_index += 1;
    //                             if *x==0u8 {
    //                                 state = Fsm::COBS;  //end of packet; throw away the rest
    //                                 break;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }

    //         Fsm::COBS => { //decode and exec
    //             state = Fsm::IDLE;
    //             let decoded_data_length = decode_buf(&serial_buffer[..serial_buffer_index], &mut decoded_data).unwrap();

    //             if decoded_data_length == 1 {
    //                 match decoded_data[0] {
    //                     21 => {
    //                         let n = encode_buf(&FIRMWARE_VERSION, &mut encoded);

    //                          // Send back to the host
    //                         let mut wr_ptr = &encoded[..n];
    //                         while !wr_ptr.is_empty() {
    //                             match serial.write(wr_ptr) {
    //                                 Ok(len) => wr_ptr = &wr_ptr[len..],
    //                                 // On error, just drop unwritten data.
    //                                 // One possible error is Err(WouldBlock), meaning the USB
    //                                 // write buffer is full.
    //                                 Err(_) => break,
    //                             }
    //                         }
    //                     }
    //                     _ =>{}
    //                 }
    //             } else if decoded_data_length == 2 {
    //                 sio.fifo.write((((decoded_data[0] as u16)<<8)+(decoded_data[1] as u16)) as u32);

    //             }

    //         }

    //     }

    // }
}

// End of file
