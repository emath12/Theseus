use alloc::alloc::GlobalAlloc;

pub trait HeapAccounting: GlobalAlloc + Send + Sync {
    fn allocated(&self, id: usize) -> usize;
    fn used(&self, id: usize) -> usize;
    fn allocated_and_used(&self, id: usize) -> (usize, usize);
    fn max_allocated_and_used(&self) -> (usize, usize);
}