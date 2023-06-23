use alloc::vec::Vec;

use prost::{bytes::BufMut, Message};

pub fn encode_md_message(resp: super::messages::Response) -> Vec<u8> {
    let size = resp.encoded_len();

    let mut result = Vec::with_capacity(size + 1 + core::mem::size_of::<u64>());
    result.put_u8(super::messages::Info::Magick as u8);

    resp.encode_length_delimited(&mut result).unwrap();

    result
}
