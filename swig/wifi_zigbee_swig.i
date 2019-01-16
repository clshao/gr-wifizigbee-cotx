/* -*- c++ -*- */

#define WIFI_ZIGBEE_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "wifi_zigbee_swig_doc.i"

%{
#include "wifi_zigbee/preamble_sync.h"
#include "wifi_zigbee/hdr_clearance_probe.h"
#include "wifi_zigbee/zigbee_reeler_short.h"
#include "wifi_zigbee/zigbee_reeler_medium.h"
#include "wifi_zigbee/zigbee_reeler_long.h"
#include "wifi_zigbee/zero_padding_add.h"
#include "wifi_zigbee/eop_clearance_probe.h"
%}


%include "wifi_zigbee/preamble_sync.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, preamble_sync);


%include "wifi_zigbee/hdr_clearance_probe.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, hdr_clearance_probe);
%include "wifi_zigbee/zigbee_reeler_short.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, zigbee_reeler_short);
%include "wifi_zigbee/zigbee_reeler_medium.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, zigbee_reeler_medium);
%include "wifi_zigbee/zigbee_reeler_long.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, zigbee_reeler_long);

%include "wifi_zigbee/zero_padding_add.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, zero_padding_add);
%include "wifi_zigbee/eop_clearance_probe.h"
GR_SWIG_BLOCK_MAGIC2(wifi_zigbee, eop_clearance_probe);
