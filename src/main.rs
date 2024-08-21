#![no_std]
#![no_main]
#![allow(non_snake_case)]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

use embedded_hal::{ digital::v2::ToggleableOutputPin}; //blocking::i2c::Write, serial::Read, adc::OneShot};

use embedded_hal::spi::MODE_0;
use embedded_hal::blocking::spi::Write;
use fugit::RateExtU32;

use rp2040_hal as hal;
use hal::gpio::*;
use hal::{
    clocks::init_clocks_and_plls,        
    watchdog::Watchdog,
    pac,
    Sio
};
use hal::pio::PIOExt;
use rp2040_hal::clocks::Clock;
use hal::multicore::{Multicore, Stack};
use embedded_hal::digital::StatefulOutputPin;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;


use corncobs::*;


use rp2040_boot2;
#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// 9/13/2023: CLASSE 2.0.0
// 3/25/2024: CLASSE 2.1.0
// 6/6/2024   v 2.2.0   for classE driver v1

// 8/4/2024 v2.2.3 for tim box v3

const FIRMWARE_VERSION: [u8; 3] = [2, 2, 3]; 


/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut charging_pin: Pin<bank0::Gpio2, FunctionSio<SioInput>, PullNone> = pins.gpio2.into_floating_input();
    let mut led_w = pins.gpio9.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    loop {
        if charging_pin.is_low().unwrap() {
            led_w.toggle().unwrap();
        } else {
            led_w.set_high().unwrap();
        }
        delay.delay_ms(1000);
     //   let input = sio.fifo.read();
        // if let Some(word) = input {
            //delay.delay_ms(word);
            //led_pin.toggle().unwrap();
            //sio.fifo.write_blocking(CORE1_TASK_COMPLETE);
        // };
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


    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());


    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut motor = pins.gpio0.into_push_pull_output();
    let mut led_g = pins.gpio1.into_push_pull_output();
    
    //let mut charging_pin: Pin<bank0::Gpio2, FunctionSio<SioInput>, PullNone> = pins.gpio2.into_floating_input();


    led_g.set_high().unwrap(); // turn on green LED
    //led_w.set_high().unwrap(); // turn on white LED
    motor.set_high().unwrap(); // turn on white LED

    // loop{
    //     if charging_pin.is_low().unwrap() {
    //         led_w.toggle().unwrap();
    //     } else {
    //         led_w.set_high().unwrap();
    //     }
    //     delay.delay_ms(1000);


    // }

    
    


    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
    .strings(&[StringDescriptors::default()
        .manufacturer("Nia Therapeutics")
        .product("serial port")
        .serial_number("NIA-0001")]).unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();




    // incoming COBS from serial
    enum Fsm {
        IDLE,
        COBS,        
    }
    let mut state = Fsm::IDLE;
    let mut serial_buffer_index = 0usize;
    let mut serial_buffer = [0u8; 10];
    let mut decoded_data = [0u8; 8];
    let mut encoded = [0u8; 40];


    loop {
        match state {
            Fsm::IDLE => {
                if usb_dev.poll(&mut [&mut serial]) {
                    let mut buf = [0u8; 64];
                    match serial.read(&mut buf) {
                        Err(_e) => {
                            // Do nothing
                        }
                        Ok(0) => {
                            // Do nothing
                        }
                        Ok(count) => {
                            
                            serial_buffer_index = 0;

                            for x in &buf[..count] {                                
                                serial_buffer[serial_buffer_index] = *x;
                                serial_buffer_index += 1;
                                if *x==0u8 {
                                    state = Fsm::COBS;  //end of packet; throw away the rest
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            Fsm::COBS => { //decode and exec
                state = Fsm::IDLE;
                let decoded_data_length = decode_buf(&serial_buffer[..serial_buffer_index], &mut decoded_data).unwrap();

                if decoded_data_length == 3 {
                    
                    
                } else if decoded_data_length == 1 {
                    match decoded_data[0] {
                        21 => {
                            let n = encode_buf(&FIRMWARE_VERSION, &mut encoded);

                             // Send back to the host
                            let mut wr_ptr = &encoded[..n];
                            while !wr_ptr.is_empty() {
                                match serial.write(wr_ptr) {
                                    Ok(len) => wr_ptr = &wr_ptr[len..],
                                    // On error, just drop unwritten data.
                                    // One possible error is Err(WouldBlock), meaning the USB
                                    // write buffer is full.
                                    Err(_) => break,
                                }
                            }
                        }
                        _ =>{}                            
                    }    
                }                   

                // } else if decoded_data_length == 2 { // write DAC value: 8-bit
                //     // spi_cs.set_low().unwrap();
                //     // spi.write(&[(decoded_data[0] as u16) <<4]).unwrap();
                //     // spi_cs.set_high().unwrap();
                //     tx.write(((decoded_data[0] as u32)<<8) + decoded_data[1] as u32);
                // }
            }


        }

    }       

             
}

// End of file
