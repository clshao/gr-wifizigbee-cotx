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
#include "hdr_clearance_probe_impl.h"
#include <math.h>

#define dout d_debug && std::cout

using namespace gr::wifi_zigbee;
using namespace std;

static const int ZIGBEE_HDR_SAMPLE = 40 * 4 * 20; //preamble + SFD

hdr_clearance_probe::sptr
hdr_clearance_probe::make(bool debug, unsigned int pwr_threshold, unsigned int sc_threshold)
{
  return gnuradio::get_initial_sptr
	(new hdr_clearance_probe_impl(debug, pwr_threshold, sc_threshold));
}

hdr_clearance_probe_impl::hdr_clearance_probe_impl(bool debug, unsigned int pwr_threshold, unsigned int sc_threshold)
  : gr::block("hdr_clearance_probe",
		  gr::io_signature::make(1, 1, sizeof(gr_complex)),
		  gr::io_signature::make(6, 6, sizeof(float))),
		  d_debug(debug),
		  d_pwr_threshold(pwr_threshold),
		  d_sc_threshold(sc_threshold),
		  d_cnt(0),
		  d_length(0),
		  d_copy(0),
		  d_state(SEARCH),
		  d_mix_pattern(0)
{
	set_tag_propagation_policy(block::TPP_DONT);
}


hdr_clearance_probe_impl::~hdr_clearance_probe_impl()
{
}

void
hdr_clearance_probe_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
	if (d_state == SEARCH)	ninput_items_required[0] = noutput_items;
	else ninput_items_required[0] = 0;
}

int
hdr_clearance_probe_impl::general_work (int noutput_items,
				   gr_vector_int &ninput_items,
				   gr_vector_const_void_star &input_items,
				   gr_vector_void_star &output_items)
{
	const gr_complex *in = (const gr_complex *) input_items[0];
	float *out_short_real = (float *) output_items[0];
	float *out_short_img = (float *) output_items[1];
	float *out_medium_real = (float *) output_items[2];
	float *out_medium_img = (float *) output_items[3];
	float *out_long_real = (float *) output_items[4];
	float *out_long_img = (float *) output_items[5];

	int ninput = ninput_items[0];
	int noutput = noutput_items;

	const uint64_t nread = nitems_read(0);
	const pmt::pmt_t key = pmt::string_to_symbol("signal start");
	get_tags_in_range(d_tags, 0, nread, nread + ninput, key);
	if (d_tags.size()) {
		std::sort(d_tags.begin(), d_tags.end(), gr::tag_t::offset_compare);

		const uint64_t offset = d_tags.front().offset;

		if (offset > nread) {
			ninput = offset - nread;
		} else {
			d_length = pmt::to_long(d_tags.front().value);
		}
	}

	int i = 0;

	switch (d_state) {
	case SEARCH:

		while (i < ninput) {
			d_buf[d_cnt] = in[i];
			i++;
			d_cnt++;
		}

		if (d_cnt == d_length) {
			d_state = DETERMINE;
		}

		consume_each(i);
		return 0;

	case DETERMINE:

		d_mix_pattern = find_mix_pattern(d_buf, d_cnt);
		//dout << "mix_pattern: " << d_mix_pattern << std::endl;
		d_state = COPY;
		if (d_mix_pattern == 0) {
			insert_tag(0, nitems_written(0), d_cnt, "signal start");
			insert_tag(1, nitems_written(1), d_cnt, "signal start");
		}
		else if (d_mix_pattern == 1) {
			insert_tag(2, nitems_written(2), d_cnt, "signal start");
			insert_tag(3, nitems_written(3), d_cnt, "signal start");
		}
		else {
			insert_tag(4, nitems_written(4), d_cnt, "signal start");
			insert_tag(5, nitems_written(5), d_cnt, "signal start");
		}

		consume_each(0);
		return 0;

	case COPY:
		int o = 0;

		switch (d_mix_pattern) {
		case 0:

		  while (o < noutput) {
			  out_short_real[o] = real(d_buf[d_copy]);
			  out_short_img[o] = imag(d_buf[d_copy]);
			  out_medium_real[o] = 0;
			  out_medium_img[o] = 0;
			  out_long_real[o] = 0;
			  out_long_img[o] = 0;
			  o++;
			  d_copy++;
			  if (d_copy == d_cnt) {
				  d_state = SEARCH;
				  d_copy = 0;
				  d_cnt = 0;
				  dout << "Copy step-2 ends (WiFi short)." << std::endl;
				  break;
			  }
		  }

		  break;

		case 1:

		  while (o < noutput) {
			  out_short_real[o] = 0;
			  out_short_img[o] = 0;
			  out_medium_real[o] = real(d_buf[d_copy]);
			  out_medium_img[o] = imag(d_buf[d_copy]);
  			  out_long_real[o] = 0;
  			  out_long_img[o] = 0;
			  o++;
			  d_copy++;
			  if (d_copy == d_cnt) {
				  d_state = SEARCH;
				  d_copy = 0;
				  d_cnt = 0;
				  dout << "Copy step-2 ends (WiFi medium)." << std::endl;
				  break;
			  }
		  }

		  break;

		case 2:

			while (o < noutput) {
				out_short_real[o] = 0;
				out_short_img[o] = 0;
				out_medium_real[o] = 0;
				out_medium_img[o] = 0;
				out_long_real[o] = real(d_buf[d_copy]);
				out_long_img[o] = imag(d_buf[d_copy]);
				o++;
				d_copy++;
				if (d_copy == d_cnt) {
					d_state = SEARCH;
					d_copy = 0;
					d_cnt = 0;
					dout << "Copy step-2 ends (WiFi long)." << std::endl;
					break;
				}
			}

			break;
		}

		consume_each(0);
		return o;
	}

	throw std::runtime_error("hdr_clearance_probe: unknown state");
	return 0;
}

int hdr_clearance_probe_impl::fft_80(gr_complex *time) {
	gr_complex freq[80];
	int num = 0;

	for (int index = 0; index < 80; index++) {
		for (int element = 0; element < 80; element ++) {
			freq[index] += time[element] * gr_complex(cos(2*M_PI*element*index/80), -sin(2*M_PI*element*index/80));
		}

		if ((abs(freq[index] / gr_complex(80, 0)) * 20) >= d_pwr_threshold * 0.5) { // (* 1.5) for 25 cm distance; (* 0.5) for 1 m distance
			num++;
		}

		//dout << abs(freq[index] / gr_complex(80, 0)) << " " << std::endl;
	}

	//dout << "The number of over-threshold subchannels is: " << num << std::endl;

	return num;
}

short hdr_clearance_probe_impl::find_mix_pattern(gr_complex *buf, unsigned long length) {
	gr_complex temp[80];
	short mix_pattern;
	//enum {WIFI_SHORT, WIFI_MEDIUM, WIFI_LONG} mix_pattern;
	std::memcpy(temp, buf + ZIGBEE_HDR_SAMPLE - 80, 80 * sizeof(gr_complex));
	if (fft_80(temp) >= d_sc_threshold) {
		std::memcpy(temp, buf + ZIGBEE_HDR_SAMPLE, 80 * sizeof(gr_complex));
		if (fft_80(temp) >= d_sc_threshold) {
			std::memcpy(temp, buf + length - 80, 80 * sizeof(gr_complex));
			if (fft_80(temp) >= d_sc_threshold) {
				mix_pattern = 2;
			} else {
				mix_pattern = 1;
			}
		} else{
			mix_pattern = 0;
		}
	}
	else {
		mix_pattern = 0;
	}

	return mix_pattern;
}

void hdr_clearance_probe_impl::insert_tag(unsigned int out_index, uint64_t item, long spl_cnt, string str) {
	const pmt::pmt_t key = pmt::string_to_symbol(str);
	const pmt::pmt_t value = pmt::from_long(spl_cnt);
	const pmt::pmt_t srcid = pmt::string_to_symbol(name());
	add_item_tag(out_index, item, key, value, srcid);
}

