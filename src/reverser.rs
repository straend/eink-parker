use embedded_hal as hal;

use core::cell::RefCell;
pub struct Reverser<T>
{
    /// object implementing emedded hal
    s: RefCell<T>,
    arr: [u8; 33],
}
/// Chain existing embedded_hal trait implementation to
/// embedded_hal_spy
pub fn new<T>(s: T) -> Reverser<T>
{
    let arrr = [0; 33];
    Reverser {
        s: RefCell::new(s),
        arr: arrr,
    }
}

use hal::spi::FullDuplex;
extern crate nb;

impl<T> FullDuplex<u8> for Reverser<T>
where
T: FullDuplex<u8>,
{
    type Error = T::Error;
    fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
        let mut s = self.s.borrow_mut();
        let ans = s.read()?;
        Ok(ans.reverse_bits())
    }
    fn send(&mut self, w: u8) -> Result<(), nb::Error<Self::Error>> {
        let mut s = self.s.borrow_mut();
        s.send(w.reverse_bits())
    }
}

impl<T> hal::blocking::spi::Transfer<u8> for Reverser<T>
where
T: hal::blocking::spi::Transfer<u8>,
{
    type Error = T::Error;
    /// Sends `Word` to the slave. Returns the `Word` received from the slave
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        
        for (w,x) in words.iter().zip(self.arr.iter_mut()) {
            *x = w.reverse_bits();
        }
        
        let ans = self.s.borrow_mut().transfer(&mut self.arr)?;
        
        for (w, x) in words.iter_mut().zip(ans) {
            *w = x.reverse_bits();
        }
        Ok(words)
    }
}

/// Blocking write
impl<T> hal::blocking::spi::Write<u8> for Reverser<T>
where
T: hal::blocking::spi::Write<u8>,
{
    type Error = T::Error;
    /// Sends `words` to the slave, ignoring all the incoming words
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut x = [0_u8; 33];
        for (w,x) in words.iter().zip(x.iter_mut()) {
            *x = w.reverse_bits();
        }

        (self.s.borrow_mut()).write(&x)
    }
}
