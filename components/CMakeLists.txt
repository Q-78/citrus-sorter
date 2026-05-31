set(src_dirs
            CAMERA
            FRUIT_DETECT
            LED)

set(include_dirs
            CAMERA
            FRUIT_DETECT
            LED)

set(requires
            esp32-camera
            esp_jpeg
            driver
            esp_http_server
            esp_wifi
            esp_netif
            esp_event
            nvs_flash)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)
