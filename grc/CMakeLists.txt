# Copyright 2011 Free Software Foundation, Inc.
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
install(FILES
    wifi_zigbee_preamble_sync.xml
    wifi_zigbee_hdr_clearance_probe.xml
    wifi_zigbee_zigbee_reeler_short.xml
    wifi_zigbee_zigbee_reeler_medium.xml
    wifi_zigbee_zigbee_reeler_long.xml
    wifi_zigbee_zero_padding_add.xml
    wifi_zigbee_eop_clearance_probe.xml DESTINATION share/gnuradio/grc/blocks
)
