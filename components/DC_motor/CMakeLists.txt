# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS DC_motor_control.c       # list the source files of this component
    INCLUDE_DIRS "include"       # optional, add here public include directories
    REQUIRES esp_driver_mcpwm esp_driver_pcnt esp_timer	
)
