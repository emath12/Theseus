#![no_std]
extern crate prusti_contracts;
use prusti_contracts::*;

extern crate bit_field;
extern crate volatile;

use bit_field::BitField;
use volatile::{Volatile, ReadOnly};

#[extern_spec]
impl<T: Copy> Volatile<T> {

    #[pure]
    pub const fn new(value: T) -> Volatile<T>;

    #[pure]
    pub fn read(&self) -> T;

    pub fn write(&mut self, value: T);
}


