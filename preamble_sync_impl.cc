/* -*- c++ -*- */
/* 
* Copyright 2017 <+YOU OR YOUR COMPANY+>.
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING.  If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "preamble_sync_impl.h"

#define dout d_debug && std::cout

using namespace gr::wifi_zigbee;
using namespace std;

static const int PREAMBLE_PEAK_CNT = 2;
static const int EOP_DETECTION_THRESHOLD = 1;
static const int ZERO_CNT_LIMIT = 65;
//static const int MAX_SAMPLES = 540 * 80; //According to maximum WiFi payload length
//static const int MIN_GAP = 200 * 20; //According to ZigBee preamble and Length field

preamble_sync::sptr
preamble_sync::make(bool debug, unsigned int threshold)
{
  return gnuradio::get_initial_sptr
	(new preamble_sync_impl(debug, threshold));
}

preamble_sync_impl::preamble_sync_impl(bool debug, unsigned int threshold)
  : gr::block("preamble_sync",
		  gr::io_signature::make(4, 4, sizeof(float)),
		  gr::io_signature::make(2, 2, sizeof(float))),
		  d_debug(debug),
		  d_threshold(threshold),
		  d_preamble_peak_cnt(0),
		  d_correlation(0),
		  d_pkt_index(0),
		  d_cnt(0),
		  d_offset(0),
		  d_copy(0),
		  d_state(SEARCH),
		  d_eop_state(EOP_NOT_FOUND)
{
	set_tag_propagation_policy(block::TPP_DONT);
}


preamble_sync_impl::~preamble_sync_impl()
{
}

void
preamble_sync_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  	if (d_state == SEARCH || d_state == FIND) {
  		ninput_items_required[0] = noutput_items;
  		ninput_items_required[1] = noutput_items;
  		ninput_items_required[2] = noutput_items;
  		ninput_items_required[3] = noutput_items;
  	}
  	else {
  		ninput_items_required[0] = 0;
  		ninput_items_required[1] = 0;
  		ninput_items_required[2] = 0;
  		ninput_items_required[3] = 0;
  	}
}

int
preamble_sync_impl::general_work (int noutput_items,
				   gr_vector_int &ninput_items,
				   gr_vector_const_void_star &input_items,
				   gr_vector_void_star &output_items)
{
  const float *in_real = (const float *) input_items[0];
  const float *in_img = (const float *) input_items[1];
  const float *in_delay_real = (const float *) input_items[2];
  const float *in_delay_img = (const float *) input_items[3];
  float *out_real = (float *) output_items[0];
  float *out_img = (float *) output_items[1];

  int ninput = std::min(std::min(ninput_items[0], ninput_items[1]),
		  std::min(ninput_items[2], ninput_items[3]));
  //Since consume_each(i) appears at the end of program, the consumed
  //number of items should be less than the minimum number of available items
  //among all the input buffers.
  int noutput = noutput_items;

  int i = 0;

  switch(d_state) {
  case SEARCH:

	  //dout << "Enter SEARCH." << std::endl;

	  while (i < ninput) {
		  d_correlation += in_real[i] * HALF_SIN[d_cnt % 20];
		  i++;
		  d_cnt++;
		  if (d_cnt % 20 == 0) {
			  //dout << "correlation result: " << d_correlation << std::endl;
			  if (abs(d_correlation) >= d_threshold) {
				  if (d_preamble_peak_cnt < PREAMBLE_PEAK_CNT) {
					  d_preamble_peak_cnt++;
					  d_correlation = 0;
				  } else {
					  d_pkt_index++;
					  d_cnt = 0;
					  d_state = FIND;
					  d_preamble_peak_cnt = 0;
					  d_correlation = 0;
					  insert_tag(0, nitems_written(0), d_pkt_index, "signal start");
					  insert_tag(1, nitems_written(1), d_pkt_index, "signal start");
					  dout << "Signal found!" << i << std::endl;
					  break;
				  }
			  } else {
				  d_preamble_peak_cnt = 0;
				  d_correlation = 0;
				  d_cnt = 0;
			  }
		  }
	  }

	  consume_each (i);
	  return 0;

  case FIND:
	  //dout << "Enter FIND." << std::endl;

	  int zero_padding_offset;

	  while (i < ninput) {
		  d_register_real[d_offset % 80] = in_delay_real[i];
		  d_register_img[d_offset % 80] = in_delay_img[i];
		  i++;
		  if (d_offset % 80 == 79) {
			  zero_padding_offset = find_zero_padding(d_register_real, d_register_img);
			  //dout << "zerp padding offset: " << zero_padding_offset << std::endl;
			  d_state = COPY;
			  if (zero_padding_offset != 80) {
				  d_eop_state = EOP_FOUND;
				  /*for (int m = 0; m < 75; m++) {
					  dout << d_register_real[m % 75] << " " << std::endl;
				  }*/
				  break;
			  }
			  else {
				  d_eop_state = EOP_NOT_FOUND;
				  dout << d_offset << std::endl;
				  d_offset++;
				  break;
			  }
		  }
		  d_offset++;
	  }

	  //dout << "i: " << i << std::endl;

	  consume_each (i);
	  return 0;

  case COPY:

	  //dout << "Enter COPY." << std::endl;

	  int o = 0;

	  switch (d_eop_state) {
	  case EOP_FOUND:

		  dout << "EOP_FOUND!" << std::endl;

		  insert_tag(0, nitems_written(0), d_pkt_index, "signal end");
		  insert_tag(1, nitems_written(1), d_pkt_index, "signal end");
		  d_state = SEARCH;
		  d_offset = 0;

		  consume_each(0);
		  return 0;

	  case EOP_NOT_FOUND:
		  while (o < noutput && d_copy < 80) {
			  out_real[o] = d_register_real[d_copy];
			  out_img[o] = d_register_img[d_copy];
			  o++;
			  d_copy++;
		  }

		  if (d_copy == 80) {
			  d_state = FIND;
			  d_copy = 0;
		  }

		  consume_each(0);
		  return o;
	  }

  }

  throw std::runtime_error("preamble sync: unknown state");
  return 0;
}

void preamble_sync_impl::insert_tag(unsigned int out_index, uint64_t item, long pkt_index, string str) {
	const pmt::pmt_t key = pmt::string_to_symbol(str);
	const pmt::pmt_t value = pmt::from_long(pkt_index);
	const pmt::pmt_t srcid = pmt::string_to_symbol(name());
	add_item_tag(out_index, item, key, value, srcid);
}

int preamble_sync_impl::find_zero_padding(float *buf_real, float *buf_img) {
	int index;
	int zero_cnt = 0;

	for (index = 0; index < 80; index++) {
		if ((abs(buf_real[index]) < EOP_DETECTION_THRESHOLD) && (abs(buf_img[index]) < EOP_DETECTION_THRESHOLD)) {
			zero_cnt++;
		}
	}

	if (zero_cnt >= ZERO_CNT_LIMIT) return 0;
	else return 80;

}

const std::vector<float> preamble_sync_impl::HALF_SIN = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),

};

