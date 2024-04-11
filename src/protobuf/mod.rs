mod md;
mod message_body;
mod process_requiest;
mod encode_md_message;
mod process_control;

pub mod messages;

pub use md::recive_md_header;
pub use message_body::recive_message_body;
pub use process_requiest::process_requiest;
pub use encode_md_message::encode_md_message;
pub use process_control::Control;

pub fn default_response(id: u32, now: u64) -> messages::Response {
    use messages::{Info, Status};
    messages::Response {
        id,
        device_id: Info::LaserSetupId as u32,
        protocol_version: Info::ProtocolVersion as u32,
        global_status: Status::Ok as i32,
        timestamp: now,

        ..Default::default()
    }
}
