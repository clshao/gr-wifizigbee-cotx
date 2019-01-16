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
#include "eop_clearance_probe_impl.h"
#include <math.h>

#define dout d_debug && std::cout

using namespace gr::wifi_zigbee;
using namespace std;

eop_clearance_probe::sptr
eop_clearance_probe::make(bool debug, unsigned int threshold)
{
  return gnuradio::get_initial_sptr
	(new eop_clearance_probe_impl(debug, threshold));
}

eop_clearance_probe_impl::eop_clearance_probe_impl(bool debug, unsigned int threshold)
      : gr::block("eop_detection",
              gr::io_signature::make(2, 2, sizeof(float)),
              gr::io_signature::make(4, 4, sizeof(float))),
			  d_debug(debug),
			  d_cnt(0),
			  d_copy(0),
			  d_threshold(threshold),
			  d_mix_pattern(0),
			  d_tag_state(EOP_NOT_FOUND),
			  d_state(SEARCH)
    {
    	set_tag_propagation_policy(block::TPP_DONT);
    }

eop_clearance_probe_impl::~eop_clearance_probe_impl()
{
}


void
eop_clearance_probe_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
	if (d_state == SEARCH) {
		ninput_items_required[0] = noutput_items;
		ninput_items_required[1] = noutput_items;
	}
	else {
		ninput_items_required[0] = 0;
		ninput_items_required[1] = 0;
	}

}


int
eop_clearance_probe_impl::general_work (int noutput_items,
				   gr_vector_int &ninput_items,
				   gr_vector_const_void_star &input_items,
				   gr_vector_void_star &output_items)
{
	  const float *in_real = (const float*)input_items[0];
	  const float *in_img = (const float*)input_items[1];
	  float *out_sm_real = (float *)output_items[0];
	  float *out_sm_img = (float *)output_items[1];
	  float *out_lg_real = (float*)output_items[2];
	  float *out_lg_img = (float*)output_items[3];

	  int ninput = min(ninput_items[0], ninput_items[1]);
      int noutput = noutput_items;

      const uint64_t nread = nitems_read(0);
      //dout << "nread" << nread << std::endl;
      const pmt::pmt_t key = pmt::string_to_symbol("signal end");
      get_tags_in_range(d_tags, 0, nread, nread + ninput + 1, key);
      if (d_tags.size()) {
    	  d_tag_state = EOP_FOUND;
    	  std::sort(d_tags.begin(), d_tags.end(), gr::tag_t::offset_compare);

    	  const uint64_t offset = d_tags.front().offset;
    	  //dout << "offset: " << offset << std::endl;
		  if (offset > nread) ninput = offset - nread;
      } else {
    	  d_tag_state = EOP_NOT_FOUND;
      }

      int i = 0;

      switch (d_state) {
      case SEARCH:

    	  //dout << "SEARCH." << std::endl;

    	  while (i < ninput) {
    		  d_buf_real[d_cnt] = in_real[i];
    		  d_buf_img[d_cnt] = in_img[i];
    		  d_cnt++;
    		  i++;
    	  }

    	  //dout << "d_cnt" << d_cnt << std::endl;
    	  //dout << "ninput: " << ninput << std::endl;
    	  //dout << "i" << i << std::endl;

    	  if (d_tag_state == EOP_FOUND) {
    		  d_state = DETERMINE;
    	  }

    	  consume_each(i);
    	  return 0;

	  case DETERMINE:

		  //dout << "DETERMINE." << std::endl;

		  d_mix_pattern = find_mix_pattern(d_buf_real, d_buf_img, d_cnt);
		  d_state = COPY;
		  if (!d_mix_pattern) {
			  insert_tag(0, nitems_written(0), d_cnt, "signal start");
			  insert_tag(1, nitems_written(1), d_cnt, "signal start");
		  }
		  else {
			  insert_tag(2, nitems_written(2), d_cnt, "signal start");
			  insert_tag(3, nitems_written(3), d_cnt, "signal start");
		  }

		  consume_each(0);
		  return 0;

      case COPY:
    	  int o = 0;

    	  switch (d_mix_pattern) {
    	  case 0:

    		  while (o < noutput) {
    			  out_sm_real[o] = d_buf_real[d_copy];
    			  out_sm_img[o] = d_buf_img[d_copy];
    			  out_lg_real[o] = 0;
				  out_lg_img[o] = 0;
				  o++;
				  d_copy++;
				  if (d_copy == d_cnt) {
					  d_state = SEARCH;
					  //dout << "Copied: " << d_copy << std::endl;
					  d_copy = 0;
					  d_cnt = 0;
					  dout << "Copy step-1 ends (WiFi short/medium)." << std::endl;
					  break;
				  }
    		  }

    		  break;

    	  case 1:

    		  while (o < noutput) {
    			  out_sm_real[o] = 0;
				  out_sm_img[o] = 0;
    			  out_lg_real[o] = d_buf_real[d_copy];
    			  out_lg_img[o] = d_buf_img[d_copy];
				  o++;
				  d_copy++;
				  if (d_copy == d_cnt) {
					  d_state = SEARCH;
					  d_copy = 0;
					  d_cnt = 0;
					  dout << "Copy step-1 ends (WiFi long)." << std::endl;
					  break;
				  }
    		  }

    		  break;
    	  }

    	  consume_each(0);
    	  return o;
      }

      throw std::runtime_error("eop clearance probe: unknown state");
      return 0;

}

short eop_clearance_probe_impl::find_mix_pattern(float *buf_real, float *buf_img, unsigned long length) {
	short mix_pattern;
	int correlation_real = 0;
	int correlation_img = 0;
	int m = 0;
	//enum {WIFI_SHORT_MEDIUM, WIFI_LONG} mix_pattern;

	for (int index = length - 20; index < length; index++) {
		correlation_real += buf_real[index] * HALF_SIN[m];
		correlation_img += buf_img[index - 10] * HALF_SIN[m];
		m++;
	}

	if (abs(correlation_real) > d_threshold && abs(correlation_img) > d_threshold) {
		mix_pattern = 0;
	}
	else {
		mix_pattern = 1;
	}

	return mix_pattern;
}

void eop_clearance_probe_impl::insert_tag(unsigned int out_index, uint64_t item, long spl_cnt, string str) {
	const pmt::pmt_t key = pmt::string_to_symbol(str);
	const pmt::pmt_t value = pmt::from_long(spl_cnt);
	const pmt::pmt_t srcid = pmt::string_to_symbol(name());
	add_item_tag(out_index, item, key, value, srcid);
}

const std::vector<float> eop_clearance_probe_impl::HALF_SIN = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),

};
