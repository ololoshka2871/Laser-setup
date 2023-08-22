pub fn process_requiest(
    req: &super::messages::Request,
    resp: &mut super::messages::Response,
    current_control_state: &mut super::Control,
) {
    if !(req.device_id == super::messages::Info::LaserSetupId as u32
        || req.device_id == super::messages::Info::IdDiscover as u32)
    {
        defmt::error!("Protobuf: unknown target device id: 0x{:X}", req.device_id);

        resp.global_status = super::messages::Status::ProtocolError as i32;
        return;
    }

    if req.protocol_version != super::messages::Info::ProtocolVersion as u32
        && req.device_id != super::messages::Info::IdDiscover as u32
    {
        defmt::warn!(
            "Protobuf: unsupported protocol version {}",
            req.protocol_version
        );
        resp.global_status = super::messages::Status::ProtocolError as i32;
        return;
    }

    if let Some(control) = &req.control {
        super::process_control::process_control(&control, current_control_state);
        resp.control = Some(current_control_state.into_response());
    }
}
