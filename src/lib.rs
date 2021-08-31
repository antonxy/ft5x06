#![no_std]

//! Driver for I2C communication with the FocalTech FT5x06 touch controllers.
//! [I2C register documentation](https://www.newhavendisplay.com/app_notes/FT5x06_registers.pdf)

extern crate embedded_hal as hal;

use hal::blocking::i2c::{WriteRead, SevenBitAddress};

use modular_bitfield_msb::prelude::*;

const ADDR: u8 = 0b0111000;

const GESTURE_REGISTER: u8 = 0x02;
const NUM_TOUCHES_REGISTER: u8 = 0x02;
const TOUCHES_REGISTER_START: u8 = 0x03;
const TOUCHES_REGISTER_STEP: u8 = 0x06;

#[derive(BitfieldSpecifier, Debug)]
#[bits = 8]
pub enum Gesture {
    MoveUp = 0x10,
    MoveLeft = 0x14,
    MoveDown = 0x18,
    MoveRight = 0x1C,
    ZoomIn = 0x48,
    ZoomOut = 0x49,
}

#[derive(BitfieldSpecifier, Debug)]
#[bits = 2]
/// Range of output in mA
pub enum Event {
    Down = 0,
    Up = 1,
    Contact = 2,
}

#[bitfield(bits = 32)]
#[derive(Clone, Copy, Debug)]
pub struct Touch {
    pub event: Event,
    #[skip] __: B2,
    pub x: B12,
    pub id: B4,
    pub y: B12,
}

pub struct FT5x06<I2C>
{
    i2c: I2C
}

impl<E, I2C> FT5x06<I2C>
where
    I2C: WriteRead<SevenBitAddress, Error = E>
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c
        }
    }

    pub fn read_num_touches(&mut self) -> Result<u8, E> {
        let mut num_touches_bytes = [0];
        self.i2c.write_read(ADDR, &[NUM_TOUCHES_REGISTER], &mut num_touches_bytes)?;

        // Only the lowest 4 bits are valid
        Ok(num_touches_bytes[0] & 0b00001111)
    }

    pub fn read_touch(&mut self, touch_id: u8) -> Result<Touch, E> {
        assert!(touch_id < 10);

        let mut touch_bytes = [0;4];
        self.i2c.write_read(ADDR, &[TOUCHES_REGISTER_START + touch_id * TOUCHES_REGISTER_STEP], &mut touch_bytes)?;

        Ok(Touch::from_bytes(touch_bytes))
    }

    pub fn read_gesture(&mut self) -> Result<Option<Gesture>, E> {
        let mut gesture_bytes = [0];
        self.i2c.write_read(ADDR, &[GESTURE_REGISTER], &mut gesture_bytes)?;

        if gesture_bytes[0] == 0x00 {
            Ok(None)
        } else {
            Ok(Some(Gesture::from_bytes(gesture_bytes[0]).unwrap()))
        }
    }

}
