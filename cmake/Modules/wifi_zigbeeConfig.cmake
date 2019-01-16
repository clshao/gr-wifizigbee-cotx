INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_WIFI_ZIGBEE wifi_zigbee)

FIND_PATH(
    WIFI_ZIGBEE_INCLUDE_DIRS
    NAMES wifi_zigbee/api.h
    HINTS $ENV{WIFI_ZIGBEE_DIR}/include
        ${PC_WIFI_ZIGBEE_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    WIFI_ZIGBEE_LIBRARIES
    NAMES gnuradio-wifi_zigbee
    HINTS $ENV{WIFI_ZIGBEE_DIR}/lib
        ${PC_WIFI_ZIGBEE_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(WIFI_ZIGBEE DEFAULT_MSG WIFI_ZIGBEE_LIBRARIES WIFI_ZIGBEE_INCLUDE_DIRS)
MARK_AS_ADVANCED(WIFI_ZIGBEE_LIBRARIES WIFI_ZIGBEE_INCLUDE_DIRS)

