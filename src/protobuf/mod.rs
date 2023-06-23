mod md;
mod message_body;
mod process_requiest;
mod encode_md_message;

pub mod messages;

pub use md::recive_md_header;
pub use message_body::recive_message_body;
pub use process_requiest::process_requiest;
pub use encode_md_message::encode_md_message;

pub fn default_response(id: u32) -> messages::Response {
    use messages::{Info, Status};
    messages::Response {
        id,
        device_id: Info::LaserSetupId as u32,
        protocol_version: Info::ProtocolVersion as u32,
        global_status: Status::Ok as i32,
        timestamp: 0,
    }
}
