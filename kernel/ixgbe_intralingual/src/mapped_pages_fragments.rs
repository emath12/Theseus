
use memory::{VirtualAddress, MappedPages};
use alloc::{
    vec::Vec,
    sync::Arc,
    boxed::Box
};
use zerocopy::FromBytes;
use core::{cmp::{max, min}, ops::{DerefMut, Deref}};
use core::mem::ManuallyDrop;

type AllocatedFragment = (VirtualAddress, usize);

pub struct MappedPagesFragments {
    mp: Arc<MappedPages>,
    allocated: Vec<AllocatedFragment>
}

impl MappedPagesFragments {

    pub fn new(mp: MappedPages) -> MappedPagesFragments {
        MappedPagesFragments { mp: Arc::new(mp), allocated: Vec::new() }
    }
    
    pub fn fragment<T: FromBytes>(&mut self, offset: usize) -> Result<Fragment<T>, &'static str> {
        let size = core::mem::size_of::<T>();
        let end = offset + size;

        // check that size of the type T fits within the size of the mapping
        if end > self.mp.size_in_bytes() {
            error!("the requested type doesn't fit in the MappedPages bound at the given offset");
            return Err("the requested type doesn't fit in the MappedPages bound at the given offset");
        }

        // check that the type T at offset does not overlap with an existing allocate fragment
        let start_addr = (self.mp.start_address().value() + offset) as isize;
        let end_addr = (self.mp.start_address().value() + offset + size) as isize;
        // debug!("required start addr: {:#X}, required end_addr: {:#X}", start_addr, end_addr);

        for fragment in &self.allocated {
            let frag_start_addr = fragment.0.value() as isize;
            let frag_end_addr = frag_start_addr + fragment.1 as isize - 1;
            // debug!("frag start addr: {:#X}, frag end_addr: {:#X}", frag_start_addr, frag_end_addr);

            let overlapping = max(0, min(end_addr, frag_end_addr) - max(start_addr, frag_start_addr) + 1);

            if overlapping != 0 {
                error!("the requested type at start: {:#X} till {:#X} overlaps with an allocated fragment at {:#X} till {:#X}", 
                    start_addr, end_addr, frag_start_addr, frag_end_addr);
                return Err("the requested type at offset overlaps with an allocated fragment");
            }
        }

        self.allocated.push((self.mp.start_address() + offset, size));

        let pointer_to_type = unsafe{ Box::from_raw((self.mp.start_address() + offset).value() as *mut T) };
        Ok(
            Fragment {
                ptr: ManuallyDrop::new(pointer_to_type),
                backing_mp: self.mp.clone()
            }
        )
    }
}

pub struct Fragment<T: FromBytes> {
    ptr: ManuallyDrop<Box<T>>,
    backing_mp: Arc<MappedPages>
}

impl<T: FromBytes> Deref for Fragment<T> {
    type Target = T;
    fn deref(&self) -> &T{
        &self.ptr
    }
}

impl<T: FromBytes> DerefMut for Fragment<T> {
    fn deref_mut(&mut self) -> &mut T{
        &mut self.ptr
    }
}