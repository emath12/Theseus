//! Structs which provide access to the ixgbe device queue registers and store their backing memory pages.
//! 
//! They implement the `RxQueueRegisters` and `TxQueueRegisters` traits which allows 
//! the registers to be accessed through virtual NICs

use crate::mapped_pages_fragments::{MappedPagesFragments, Fragment};
use super::regs::{RegistersRx, RegistersTx};
use alloc::{
    sync::Arc,
    boxed::Box
};
use core::ops::{Deref, DerefMut};
use core::mem::ManuallyDrop;
use memory::MappedPages;


/// Struct that stores a pointer to registers for one ixgbe receive queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub struct IxgbeRxQueueRegisters {
    /// We prevent the drop handler from dropping the `regs` because the backing memory is not in the heap,
    /// but in the stored mapped pages. The memory will be deallocated when the `backing_pages` are dropped.
    pub regs: ManuallyDrop<Box<RegistersRx>>,
    pub backing_pages: Arc<MappedPages>
}

impl Deref for IxgbeRxQueueRegisters {
    type Target = Box<RegistersRx>;
    fn deref(&self) -> &Box<RegistersRx> {
        &self.regs
    }
}
impl DerefMut for IxgbeRxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Box<RegistersRx> {
        &mut self.regs
    }
}

/// Struct that stores a pointer to registers for one ixgbe transmit queue
/// as well as a shared reference to the backing `MappedPages` where these registers are located.
pub(crate) struct IxgbeTxQueueRegisters {
    /// the ID of the tx queue that these registers control
    id: usize,
    /// We prevent the drop handler from dropping the `regs` because the backing memory is not in the heap,
    /// but in the stored mapped pages. The memory will be deallocated when the `backing_pages` are dropped.
    regs: Fragment<RegistersTx>
}

impl IxgbeTxQueueRegisters {
    pub fn new(queue_id: usize, mp: &mut MappedPagesFragments) -> Result<IxgbeTxQueueRegisters, &'static str> {
        let fragment = mp.fragment(queue_id * core::mem::size_of::<RegistersTx>())?;

        Ok(IxgbeTxQueueRegisters {
            id: queue_id,
            regs: fragment
        })
    }

    pub fn id(&self) -> usize {
        self.id
    }
}

impl Deref for IxgbeTxQueueRegisters {
    type Target = Fragment<RegistersTx>;
    fn deref(&self) -> &Fragment<RegistersTx> {
        &self.regs
    }
}

impl DerefMut for IxgbeTxQueueRegisters {
    fn deref_mut(&mut self) -> &mut Fragment<RegistersTx> {
        &mut self.regs
    }
}

