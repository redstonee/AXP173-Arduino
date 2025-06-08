if(NOT ESP_PLATFORM)
    project(axp173 LANGUAGES C CXX)
    target_include_directories(axp173 PUBLIC .)
    add_library(axp173 STATIC AXP173.cpp)

else()
    idf_component_register(SRCS "AXP173.cpp"
        INCLUDE_DIRS "."
        PRIV_REQUIRES arduino)
endif()
