idf_component_register(
    SRCS "src/st7735_gfx.cpp"
    INCLUDE_DIRS "include"
    # NOTE: "adafruit_gfx" is the expected registered component name for the
    # fasani/adafruit_gfx registry package. If idf.py reports this component
    # not found, run `idf.py reconfigure` and check the "Component list" in
    # the build log (or managed_components/fasani__adafruit_gfx) for the
    # exact registered name, then fix this line to match.
    REQUIRES adafruit_gfx driver esp_driver_spi
    PRIV_REQUIRES esp_driver_gpio
)
