use std::path::PathBuf;

static PROTOBUF_FILE: &str = "ProtobufDevice_0000E008.proto";
static PROTOBUF_DIR: &str = "src/protobuf";

fn gen_protobuf() {
    let mut protofile = PathBuf::from(PROTOBUF_DIR);
    protofile.push(PROTOBUF_FILE);

    prost_build::compile_protos(&[protofile], &[PROTOBUF_DIR]).unwrap();
}

fn main() {
    gen_protobuf();
}
