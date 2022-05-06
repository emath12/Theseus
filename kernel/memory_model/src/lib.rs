#![no_std]
extern crate prusti_contracts;
use prusti_contracts::*;

extern crate bit_field;
use bit_field::BitField;

#[derive(Copy, Clone)]
pub struct PhysicalAddress {
    pub(crate) number: usize,
}

impl PhysicalAddress {
    pub fn value(&self) -> usize {
        self.number
    }

    pub fn new(phys_addr: usize) -> Option<PhysicalAddress> {
        if Self::is_canonical_physical_address(phys_addr) {
            Some(PhysicalAddress{number: phys_addr})
        } else {
            None
        }
    }

    #[inline]
    pub fn is_canonical_physical_address(phys_addr: usize) -> bool {
        match phys_addr.get_bits(52..64) {
            0 => true,
            _ => false,
        }
    }
}

pub fn usize_to_u64(number: usize) -> u64 {
    let bytes = number.to_be_bytes();
    u64::from_be_bytes(bytes)
}

pub fn u64_to_usize(number: u64) -> usize {
    let bytes = number.to_be_bytes();
    usize::from_be_bytes(bytes)
}

pub struct EntryFlags(u64);

impl EntryFlags {
    pub const fn default() -> EntryFlags {
        EntryFlags(0)
    }
}

pub struct MappedPages {
    addr: PhysicalAddress
}

impl MappedPages {
    pub fn empty() -> MappedPages {
        MappedPages{
            addr: PhysicalAddress{number: 0}
        }
    }
}

// #[trusted]
pub fn create_contiguous_mapping(size_in_bytes: usize, flags: EntryFlags) -> (MappedPages, PhysicalAddress) {
    (MappedPages::empty(), PhysicalAddress{number: 0})
}