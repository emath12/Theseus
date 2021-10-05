//! Defines buffers that are used for DMA.

#![no_std]
#![feature(const_generics)]

extern crate alloc;
#[macro_use] extern crate log;
extern crate memory;
extern crate mpmc;

use core::ops::{Deref, DerefMut};
use alloc::{
    vec::Vec,
    sync::Arc
};
use memory::{PhysicalAddress, MappedPages, EntryFlags, create_contiguous_mapping};

#[derive(PartialEq, Eq)]
pub enum State {
    SWOwned,
    HWOwned
}

/// A buffer that is guaranteed to be contiguous in physical memory. 
/// It can be in one of the given `State`s. The state dictates if SW has access to the underlying memory.
pub struct Buffer<const S: State> {
    mp: MappedPages,
    pub phys_addr: PhysicalAddress,
    pub length: usize,
}

impl Buffer<{State::SWOwned}> {
    /// Creates a new buffer with the specified size in bytes.
    /// Buffers are always created in the SW owned state
    pub fn new(size_in_bytes: usize) -> Result<Buffer<{State::SWOwned}>, &'static str> {
        let (mp, starting_phys_addr) = create_contiguous_mapping(
            size_in_bytes as usize,
            EntryFlags::WRITABLE | EntryFlags::NO_CACHE | EntryFlags::NO_EXECUTE,
        )?;
        Ok(Buffer {
            mp: mp,
            phys_addr: starting_phys_addr,
            length: size_in_bytes,
        })
    }
    /// Transfers a buffer into a HW owned state, indicating that it can not longer be manipulated by SW.
    pub fn transfer_ownership(self) -> Buffer<{State::HWOwned}> {
        Buffer{
            mp: self.mp,
            phys_addr: self.phys_addr,
            length: self.length,
        }
    }
}

impl Deref for Buffer<{State::SWOwned}> {
    type Target = MappedPages;
    fn deref(&self) -> &MappedPages {
        &self.mp
    }
}
impl DerefMut for Buffer<{State::SWOwned}> {
    fn deref_mut(&mut self) -> &mut MappedPages {
        &mut self.mp
    }
}

impl Buffer<{State::HWOwned}> {
    /// Transfers a buffer into a SW owned state, indicating that it can now be modififed SW.
    pub fn transfer_ownership(self) -> Buffer<{State::SWOwned}> {
        Buffer{
            mp: self.mp,
            phys_addr: self.phys_addr,
            length: self.length,
        }
    }
}
