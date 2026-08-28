fn main() {
    println!("cargo:rustc-link-arg=-Tlinkall.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    // WiFi credentials are baked into the firmware at build time. They are
    // optional: empty means the firmware runs USB-only (no WiFi bring-up).
    for var in ["PICOPROG_WIFI_SSID", "PICOPROG_WIFI_PASSWORD"] {
        let value = std::env::var(var).unwrap_or_default();
        println!("cargo:rustc-env={}={}", var, value);
        println!("cargo:rerun-if-env-changed={}", var);
    }
}
