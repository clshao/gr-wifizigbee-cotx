/* -*- c++ -*- */
/* 
* Copyright 2017 <Chenglong Shao>.
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
#include "zigbee_reeler_long_impl.h"

#define dout d_debug && std::cout

using namespace gr::wifi_zigbee;
using namespace std;

static const int WIFI_PADDING = 404;
static const int COR_THRESHOLD = 16;
//While the correlation results for some zigbee pulses may be less
//than 16 (due to the addition of 20 ofdm samples), the decoding
//performance of zigbee rx is not affected.

zigbee_reeler_long::sptr
zigbee_reeler_long::make(bool debug)
{
  return gnuradio::get_initial_sptr
	(new zigbee_reeler_long_impl(debug));
}

zigbee_reeler_long_impl::zigbee_reeler_long_impl(bool debug)
  : gr::block("zigbee_reeler_long",
		  gr::io_signature::make(2, 2, sizeof(float)),
		  gr::io_signature::make(2, 2, sizeof(gr_complex))),
		  d_debug(debug),
		  d_cnt(0),
		  d_length(0),
		  d_pkt_spl_num(0),
		  d_copied(0),
		  d_amp_i(0),
		  d_amp_q(0),
		  d_state(COPY)
{}


zigbee_reeler_long_impl::~zigbee_reeler_long_impl()
{
}

void
zigbee_reeler_long_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
	if (d_state == COPY) {
		ninput_items_required[0] = noutput_items;
		ninput_items_required[1] = noutput_items;
	}
	else {
		ninput_items_required[0] = 0;
		ninput_items_required[1] = 0;
	}
}

int
zigbee_reeler_long_impl::general_work (int noutput_items,
				   gr_vector_int &ninput_items,
				   gr_vector_const_void_star &input_items,
				   gr_vector_void_star &output_items)
{
	const float *in_real = (const float *) input_items[0];
	const float *in_img = (const float *) input_items[1];
	gr_complex *out_zigbee = (gr_complex *) output_items[0];
	gr_complex *out_wifi = (gr_complex *) output_items[1];

	int ninput = std::min(ninput_items[0], ninput_items[1]);
	int noutput = noutput_items;

	const uint64_t nread = nitems_read(0);
	get_tags_in_range(d_tags, 0, nread, nread + ninput);
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
	int o = 0;

	switch (d_state) {
	case COPY:

		while (i < ninput) {
			d_buf_real[d_cnt] = in_real[i];
			d_buf_img[d_cnt] = in_img[i];
			i++;
			d_cnt++;
		}

		if (d_cnt == d_length) {
			d_state = REEL;
		}

		consume_each(i);
		return 0;

	case REEL:
		//Estimating the frequency offset between ZigBee samples is TBD.
		float amp_i_measure_input[128];
		float amp_q_measure_input[128];

		for (int index = 0; index < 128; index++) {
			amp_i_measure_input[index] = d_buf_real[index + 192];
			amp_q_measure_input[index] = d_buf_img[index + 32];
		}

		//The sum of benchmark pattern should be sufficiently larger than the mean of noise.
		d_amp_i = amp_i_measure(amp_i_measure_input);
		d_amp_q = amp_q_measure(amp_q_measure_input);

		//dout << "d_amp_i: " << d_amp_i << std::endl;
		//dout << "d_amp_q: " << d_amp_q << std::endl;

		make_zigbee_header(d_zigbee_hdr, d_amp_i, d_amp_q);
		d_pkt_spl_num = make_packet(d_zigbee, d_wifi, d_cnt, d_amp_i, d_amp_q);
		d_amp_i = 0;
		d_amp_q = 0;
		d_state = PADDING_OUT;

		consume_each(0);
		return 0;


	case PADDING_OUT:

		while (o < noutput) {
			out_zigbee[o] = *(d_zigbee + d_copied);
			out_wifi[o] = *(d_wifi + d_copied);
			o++;
			d_copied++;

			if (d_copied == d_pkt_spl_num) {
				dout << d_copied << " samples sent." << std::endl;
				d_copied = 0;
				d_state = COPY;
				break;
			}
		}

		consume_each(0);
		return o;

	}

	throw std::runtime_error("ZigBee reeler (long): unknown state");
	return 0;
}

double zigbee_reeler_long_impl::amp_i_measure(float *buf) {
	double amp;
	float sum_buf = 0;
	double sum_pattern = 0;
	std::vector<double> pattern = {

	/*The 2nd-4th parts out of 10 repeated sample sets in the STF.
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),*/

	//-1, -1, 1, -1, 1, 1, 1 (two repeated parts of the LTF)
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	};

	for (int i = 0; i < 64; i++) {
		sum_buf += buf[i] - buf[i + 64];
		sum_pattern += pattern[i] - pattern[i + 64];
		//dout << buf[i] - buf[i + 64] << std::endl;
		//dout << pattern[i] - pattern[i + 64] << std::endl;
	}

	amp = (sum_buf / 64) / (sum_pattern / 64);

	//dout << "sum_buf_i: " << sum_buf << std::endl;
	//dout << "sum_pattern_i: " << sum_pattern << std::endl;
	dout << "amp_i: " << amp << std::endl;

	return amp;
}

double zigbee_reeler_long_impl::amp_q_measure(float *buf) {
	double amp;
	float sum_buf = 0;
	double sum_pattern = 0;
	std::vector<double> pattern = {

	//1, -1, 1, 1, -1, -1, 1 (the last 8 repeated parts in the STF)
	sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20), sin(5*M_PI/20),
	sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
	sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20),
	sin(14*M_PI/20), sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20),
	sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20),
	};

	/*1, -1, -1, -1, -1, 1, -1 (two repeated parts of the LTF)
	sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20), -sin(10*M_PI/20), -sin(11*M_PI/20),
	-sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20), -sin(15*M_PI/20),
	-sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
	sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20),
	sin(4*M_PI/20), sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20),
	sin(8*M_PI/20), sin(9*M_PI/20), sin(10*M_PI/20), sin(11*M_PI/20),
	sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20), sin(15*M_PI/20),
	sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
	-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20),
	-sin(4*M_PI/20), -sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20),
	-sin(8*M_PI/20), -sin(9*M_PI/20),
	};*/

	for (int i = 0; i < 64; i++) {
		sum_buf += buf[i] - buf[i + 64];
		sum_pattern += pattern[i] - pattern[i + 64];
		//dout << buf[i] - buf[i + 64] << std::endl;
		//dout << pattern[i] - pattern[i + 64] << std::endl;

	}

	amp = (sum_buf / 64) / (sum_pattern / 64);

	//dout << "sum_buf_q: " << sum_buf << std::endl;
	//dout << "sum_pattern_q: " << sum_pattern << std::endl;
	dout << "amp_q: " << amp << std::endl;

	return amp;
}

int zigbee_reeler_long_impl::high_low(float *buf) {
	float correlation = 0;

	for (int i = 0; i < 20; i++) {
		correlation += buf[i] * HALF_SIN[i];
	}

	//dout << "correlation: " << correlation << std::endl;

	if (correlation >= COR_THRESHOLD) {
		return 1;
	} else if (correlation <= -COR_THRESHOLD) {
		return -1;
	} else {
		return 0;
	}
}

const std::vector<float> zigbee_reeler_long_impl::HALF_SIN = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),

};

const std::vector<float> zigbee_reeler_long_impl::PREAMBLE_HIGH_LOW_REAL = {

1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, //0000
1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, //1110 (0x7 in little-endian)
-1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, //0101 (0xA in little-endian)

};

const std::vector<float> zigbee_reeler_long_impl::PREAMBLE_HIGH_LOW_IMG = {

1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, //0000
-1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, //1110 (0x7 in little-endian)
1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1, 1, 1, //0101 (0xA in little-endian)

};

void zigbee_reeler_long_impl::make_zigbee_header(gr_complex *buf, double amp_i, double amp_q) {
	int count = 0;
	float buf_real[ZIGBEE_HDR_SAMPLE];
	float buf_img[ZIGBEE_HDR_SAMPLE];

	for (int i = 0; i < 160; i++) {
		for (int o = 0; o < 20; o++) {
			buf_real[count] = amp_i * PREAMBLE_HIGH_LOW_REAL[i] * HALF_SIN[o];
			buf_img[count] = amp_q * PREAMBLE_HIGH_LOW_IMG[i] * HALF_SIN[o];
			count++;
		}
	}

	for (int index = 0; index < ZIGBEE_HDR_SAMPLE; index++) {
		if (index < 10) {
			buf[index] = gr_complex(buf_real[index], 0);
		}
		else {
			buf[index] = gr_complex(buf_real[index], buf_img[index - 10]);
		}
	}
}

int zigbee_reeler_long_impl::make_packet(gr_complex *zigbee, gr_complex *wifi, unsigned long length,
		double amp_i, double amp_q) {
	int i;
	int m = 0;
	int index;
	int cor_real;
	int cor_img;
	int cor_real_history = 1;
	int cor_img_history = 1;
	float buf_real[length - ZIGBEE_HDR_SAMPLE];
	float buf_img[length - ZIGBEE_HDR_SAMPLE];
	float buf_img_extra[10];

	//int cor_real_sum = 0;
	//int cor_img_sum = 0;

	for (int extra = 0; extra < 10; extra++) {
		buf_img_extra[extra] = amp_q * PREAMBLE_HIGH_LOW_IMG[159] * HALF_SIN[extra + 10];
	}

	std::memcpy(buf_img, buf_img_extra, 10 * sizeof(float));

	for (index = 0; index < (length - ZIGBEE_HDR_SAMPLE) / 20; index++) {
		if (index == (length - ZIGBEE_HDR_SAMPLE) / 20 - 1) {
			cor_real = high_low(d_buf_real + ZIGBEE_HDR_SAMPLE + index * 20);
			std::memset(d_buf_img + length, 0, 10 * sizeof(float));
			cor_img = high_low(d_buf_img + ZIGBEE_HDR_SAMPLE + 10 + index * 20);
			//cor_real_sum += cor_real;
			//cor_img_sum += cor_img;
		} else {
			cor_real = high_low(d_buf_real + ZIGBEE_HDR_SAMPLE + index * 20);
			cor_img = high_low(d_buf_img + ZIGBEE_HDR_SAMPLE + 10 + index * 20);
			//cor_real_sum += cor_real;
			//cor_img_sum += cor_img;
		}

		//If either cor_real or cor_img is 0, the corresponding elements in buf_real and
		//buf_img are set to 0. However, this case also happens at the end of a zigbee packet.
		//So, we also take into consideration the values of cor_real and cor_img for the previous 20 i-phase and
		//q-phase samples. If either of the previous correlation results is 0, we then set the current buf_real
		//and buf_img to be 0. Otherwise, we set buf_real and buf_img to be non-zero and zero, respectively.
		while (m < length - ZIGBEE_HDR_SAMPLE) {
			if ((cor_real == 0 || cor_img == 0) && (cor_real_history == 0 || cor_img_history == 0) ) {
				buf_real[m] = 0;
				buf_img[m] = 0;
			}
			else {
				buf_real[m] = amp_i * cor_real * HALF_SIN[m % 20];
				buf_img[m + 10] = amp_q * cor_img * HALF_SIN[m % 20];
				//While the last 10 q-phase samples are set to 0, the decoding performance
				//of legacy zigbee rx is not affected since several bit errors are allowed
				//in the midst of chips-to-symbol matching operation.
			}
			m++;

			if (m % 20 == 0) break;
		}

		cor_real_history = cor_real;
		cor_img_history = cor_img;
	}

	//dout << "cor_real_sum: " << cor_real_sum << std::endl;
	//dout << "cor_img_sum: " << cor_img_sum << std::endl;

	for (i = 0; i < length; i++) {
		if (i < ZIGBEE_HDR_SAMPLE) {
			*(zigbee + i) = *(d_zigbee_hdr + i);
			*(wifi + i) = gr_complex(d_buf_real[i], d_buf_img[i]) - d_zigbee_hdr[i];
		}
		else {
			*(zigbee + i) = gr_complex(buf_real[i - ZIGBEE_HDR_SAMPLE], buf_img[i - ZIGBEE_HDR_SAMPLE]);
			*(wifi + i) = gr_complex(d_buf_real[i], d_buf_img[i]) - *(zigbee + i);
		}
	}

	while (i < length + WIFI_PADDING) {
		*(zigbee + i) = 0;
		*(wifi + i) = 0;
		i++;
	}

	return i;
}

