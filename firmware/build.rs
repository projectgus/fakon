use std::fs::File;
use std::io::BufWriter;
use dbc_codegen::{Config, FeatureConfig};

fn main() {
    let dbc_path = "./dbc/pcan.dbc";
    let dbc_file = std::fs::read(dbc_path).unwrap();
    println!("cargo:rerun-if-changed={}", dbc_path);

    let config = Config::builder()
        .dbc_name("pcan.dbc")
        .dbc_content(&dbc_file)
        .allow_dead_code(true) // Don't emit warnings if not all generated code is used
        //.impl_arbitrary(FeatureConfig::Gated("arbitrary")) // Optional impls.
        .impl_debug(FeatureConfig::Always)                 // See rustdoc for more,
        //.check_ranges(FeatureConfig::Never)                // or look below for an example.
        .build();

    // Note: I would prefer to code generate to OUT_DIR, but the current
    // dbc_codegen output seems to not work via include! as it has top-level
    // inner attributes, not include-able as per
    // https://github.com/rust-lang/rust/issues/117464
    //
    // Similarly, module #path attribute doesn't allow to dereference OUT_DIR -
    // i.e. https://internals.rust-lang.org/t/pre-rfc-macros-in-attributes/5767
    //
    // So clutching my pearls and code generating into the source directory,
    // although the generated files are ignored in git...
    let mut out = BufWriter::new(File::create("src/dbc/pcan.rs").unwrap());
    dbc_codegen::codegen(config, &mut out).expect("dbc-codegen failed");
}
