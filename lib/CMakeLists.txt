# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND wifi_zigbee_sources
    preamble_sync_impl.cc
    hdr_clearance_probe_impl.cc
    zigbee_reeler_short_impl.cc
    zigbee_reeler_medium_impl.cc
    zigbee_reeler_long_impl.cc
    zero_padding_add_impl.cc
    eop_clearance_probe_impl.cc )

set(wifi_zigbee_sources "${wifi_zigbee_sources}" PARENT_SCOPE)
if(NOT wifi_zigbee_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT wifi_zigbee_sources)

add_library(gnuradio-wifi_zigbee SHARED ${wifi_zigbee_sources})
target_link_libraries(gnuradio-wifi_zigbee ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES})
set_target_properties(gnuradio-wifi_zigbee PROPERTIES DEFINE_SYMBOL "gnuradio_wifi_zigbee_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-wifi_zigbee PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-wifi_zigbee RUNTIME_COMPONENT "wifi_zigbee_runtime" DEVEL_COMPONENT "wifi_zigbee_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_wifi_zigbee_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_wifi_zigbee.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_wifi_zigbee.cc
)

add_executable(test-wifi_zigbee ${test_wifi_zigbee_sources})

target_link_libraries(
  test-wifi_zigbee
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-wifi_zigbee
)

GR_ADD_TEST(test_wifi_zigbee test-wifi_zigbee)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

