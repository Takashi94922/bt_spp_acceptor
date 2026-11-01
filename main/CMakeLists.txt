idf_component_register(SRCS "flight_log.cpp" "ble_gatt_comm.cpp" "main.cpp" "ESP32_i2c_LSM9DS1.cpp" "MadgwickAHRS.cpp" "motor.cpp" "Motion_control.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES bt 
                    PRIV_REQUIRES bt driver nvs_flash)
#target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=uninitialized)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=uninitialized -Wno-maybe-uninitialized -Wno-clobbered)