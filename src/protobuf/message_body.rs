use alloc::vec::Vec;

use embedded_hal::serial::Read;
use prost::Message;

pub fn recive_message_body<S, E>(
    stream: &mut S,
    size: usize,
) -> nb::Result<Result<super::messages::Request, prost::DecodeError>, E>
where
    S: Read<u8, Error = E>,
{
    static mut BODY: Option<Vec<u8>> = None;

    loop {
        match unsafe { &mut *core::ptr::addr_of_mut!(BODY) } {
            Some(body) => {
                if body.len() == size {
                    let res = super::messages::Request::decode(body.as_slice());
                    unsafe { BODY = None };
                    return Ok(res);
                } else {
                    body.push(stream.read()?);
                }
            }
            None => unsafe { BODY = Some(Vec::with_capacity(size)) },
        }
    }
}
