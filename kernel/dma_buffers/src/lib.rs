//! Defines buffers that are used for DMA.

#![no_std]

extern crate alloc;
#[macro_use] extern crate log;
extern crate memory;
extern crate mpmc;

use core::ops::{Deref, DerefMut};
use alloc::vec::Vec;
use memory::{PhysicalAddress, MappedPages, EntryFlags, create_contiguous_mapping};


/// A buffer that is guaranteed to be contiguous in physical memory. 
/// Auto-dereferences into a `MappedPages` object that represents its underlying memory. 
pub struct SWOwnedBuffer {
    pub mp: MappedPages,
    pub phys_addr: PhysicalAddress,
    pub length: u16,
}
impl SWOwnedBuffer {
    /// Creates a new TransmitBuffer with the specified size in bytes.
    /// The size is a `u16` because that is the maximum size of an NIC transmit buffer. 
    pub fn new(size_in_bytes: u16) -> Result<SWOwnedBuffer, &'static str> {
        let (mp, starting_phys_addr) = create_contiguous_mapping(
            size_in_bytes as usize,
            EntryFlags::WRITABLE | EntryFlags::NO_CACHE | EntryFlags::NO_EXECUTE,
        )?;
        Ok(SWOwnedBuffer {
            mp: mp,
            phys_addr: starting_phys_addr,
            length: size_in_bytes,
        })
    }

    pub fn transfer_ownership(self, desc_id: usize) -> HWOwnedBuffer {
        HWOwnedBuffer{
            mp: self.mp,
            phys_addr: self.phys_addr,
            length: self.length,
            desc_id
        }
    }
}

impl Deref for SWOwnedBuffer {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}
impl DerefMut for SWOwnedBuffer {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}


/// A buffer that is guaranteed to be contiguous in physical memory. 
/// Auto-dereferences into a `MappedPages` object that represents its underlying memory. 
pub struct HWOwnedBuffer {
    mp: MappedPages,
    phys_addr: PhysicalAddress,
    length: u16,
    desc_id: usize
}
impl HWOwnedBuffer {
    pub fn transfer_ownership(self) -> SWOwnedBuffer {
        SWOwnedBuffer{
            mp: self.mp,
            phys_addr: self.phys_addr,
            length: self.length,

        }
    }
}

impl Deref for HWOwnedBuffer {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}
impl DerefMut for HWOwnedBuffer {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}

