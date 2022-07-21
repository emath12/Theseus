#![no_std]

// #[macro_use] extern crate log;
extern crate prusti_contracts;

use core::{
    ops::{Add, AddAssign, Sub, SubAssign, Deref},
    cmp::Ordering
};
use prusti_contracts::*;

/// The lower 12 bits of a virtual address correspond to the P1 page frame offset. 
pub const PAGE_SHIFT: usize = 12;
/// Page size is 4096 bytes, 4KiB pages.
pub const PAGE_SIZE: usize = 1 << PAGE_SHIFT;
pub const MAX_VIRTUAL_ADDRESS: usize = usize::MAX;
pub const MAX_PAGE_NUMBER: usize = MAX_VIRTUAL_ADDRESS / PAGE_SIZE;

// #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
// pub struct Page {
//     number: usize
// }

// impl Add<usize> for Page {
//     type Output = Page;
//     fn add(self, rhs: usize) -> Page {
//         // cannot exceed max page number (which is also max frame number)
//         Page {
//             number: core::cmp::min(MAX_PAGE_NUMBER, self.number.saturating_add(rhs)),
//         }
//     }
// }
// impl AddAssign<usize> for Page {
//     fn add_assign(&mut self, rhs: usize) {
//         *self = Page {
//             number: core::cmp::min(MAX_PAGE_NUMBER, self.number.saturating_add(rhs)),
//         };
//     }
// }
// impl Sub<usize> for Page {
//     type Output = Page;
//     fn sub(self, rhs: usize) -> Page {
//         Page {
//             number: self.number.saturating_sub(rhs),
//         }
//     }
// }
// impl SubAssign<usize> for Page {
//     fn sub_assign(&mut self, rhs: usize) {
//         *self = Page {
//             number: self.number.saturating_sub(rhs),
//         };
//     }
// }

// impl Deref for Page {
//     type Target = usize;
//     // #[pure]
//     fn deref(&self) -> &usize {
//         &self.number
//     }
// }

// #[extern_spec]
// impl Ord for Page {
//     #[pure]
//     fn cmp(&self, other: &Self) -> Ordering; 
// }

// #[extern_spec]
// impl PartialOrd for Page {
//     #[pure]
//     fn partial_cmp(&self, other: &Self) -> Option<Ordering>; 
// }

#[derive(Clone, Copy)]
pub struct RangeInclusive {
    start: usize,
    end: usize
}

impl RangeInclusive {
    #[requires(start <= end)]
    #[ensures(result.start == start)]
    #[ensures(result.end == end)]
    pub const fn new(start: usize, end: usize) -> Self {
        Self{start, end}
    }
}

// #[extern_spec]
// impl RangeInclusive<usize> {
//     #[pure]
//     pub const fn start(&self) -> &usize;

//     #[pure]
//     pub const fn end(&self) -> &usize;
// }

#[pure]
fn split_chunk_has_no_overlapping_ranges(chunk1: &Option<Chunk>, chunk2: &Chunk, chunk3: &Option<Chunk>) -> bool {
    let mut no_overlap = true;

    if let Some(c1) = chunk1 {
        no_overlap &= Chunk::no_overlap(c1, chunk2);
        if let Some(c3) = chunk3 {
            no_overlap &= Chunk::no_overlap(c1, c3);
            no_overlap &= Chunk::no_overlap(chunk2, c3);
        }
    } else {
        if let Some(c3) = chunk3 {
            no_overlap &= Chunk::no_overlap(chunk2, c3);
        }
    }

    no_overlap
}


#[pure]
fn page_is_less_than_max(chunk: &Option<Chunk>) -> bool {
    if let Some(c) = chunk {
        if c.pages.end <= MAX_PAGE_NUMBER {
            return true;
        } else {
            return false;
        }
    } else {
        return true;
    }

}

#[pure]
fn page_is_less_than_max2(chunk: &Chunk) -> bool {
    if chunk.pages.end <= MAX_PAGE_NUMBER {
        return true;
    } else {
        return false;
    }
}

#[pure]
#[requires(page_is_less_than_max(chunk1))]
#[requires(page_is_less_than_max2(chunk2))]
#[requires(page_is_less_than_max(chunk3))]
fn split_chunk_is_contiguous(chunk1: &Option<Chunk>, chunk2: &Chunk, chunk3: &Option<Chunk>) -> bool {
    let mut contiguous = true;
    if let Some(c1) = chunk1 {
        contiguous &= c1.pages.end + 1 == chunk2.pages.start
    } 
    if let Some(c3) = chunk3 {
        contiguous &= chunk2.pages.end + 1 == c3.pages.start
    }
    contiguous
}

#[pure]
fn start_end_are_equal(orig_chunk: &Chunk, split_chunk: &(Option<Chunk>, Chunk, Option<Chunk>)) -> bool {
    let (chunk1,chunk2,chunk3) = split_chunk;
    let min_page;
    let max_page;

    if let Some(c1) = chunk1 {
        min_page = c1.pages.start;    
    } else {
        min_page = chunk2.pages.start;
    }

    if let Some(c3) = chunk3 {
        max_page = c3.pages.end;    
    } else {
        max_page = chunk2.pages.end;
    }

    min_page == orig_chunk.pages.start && max_page == orig_chunk.pages.end
}


struct Chunk {
    pages: RangeInclusive
}

impl Chunk {
    // #[pure]
    // pub fn start(&self) -> usize {
    //     self.pages.start
    // }

    // #[pure]
    // pub fn end(&self) -> usize {
    //     self.pages.end
    // }

    #[requires(pages.start <= pages.end)]
    #[ensures(result.pages.start == pages.start)]
    #[ensures(result.pages.end == pages.end)]
    pub fn new(pages: RangeInclusive) -> Chunk {
        Chunk { pages }
    }

    #[pure]
    fn no_overlap(chunk1: &Self, chunk2: &Self) -> bool {
        (chunk1.pages.end < chunk2.pages.start) | (chunk2.pages.end < chunk1.pages.start)
    }

    #[requires(start_page >= self.pages.start)]
    #[requires(start_page + num_pages - 1 <= self.pages.end)]
    #[requires(num_pages > 0)]
    #[requires(self.pages.end <= MAX_PAGE_NUMBER)]
    #[ensures(split_chunk_has_no_overlapping_ranges(&result.0, &result.1, &result.2))]
    #[ensures(split_chunk_is_contiguous(&result.0, &result.1, &result.2))]
    #[ensures(start_end_are_equal(&self, &result))]
    pub fn split(self, start_page: usize, num_pages: usize) -> (Option<Chunk>, Chunk, Option<Chunk>) {

        let first_chunk = if start_page == self.pages.start  {
            None
        } else {
            Some(Chunk::new(RangeInclusive::new(self.pages.start, start_page - 1)))
        };

        let second_chunk = Chunk::new(RangeInclusive::new(start_page, start_page + num_pages - 1));

        let third_chunk = if start_page + num_pages - 1 == self.pages.end {
            None
        } else {
            Some(Chunk::new(RangeInclusive::new(start_page + num_pages, self.pages.end)))
        };

        (first_chunk, second_chunk, third_chunk)
    }
}

impl Deref for Chunk {
    type Target = RangeInclusive;
    fn deref(&self) -> &RangeInclusive {
        &self.pages
    }
}