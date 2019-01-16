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
#include "zigbee_reeler_short_impl.h"

#define dout d_debug && std::cout

using namespace gr::wifi_zigbee;
using namespace std;

//static const int WIFI_PADDING = 400;
static const int ZIGBEE_PADDING = 4 + 5 * 50;

zigbee_reeler_short::sptr
zigbee_reeler_short::make(bool debug)
{
  return gnuradio::get_initial_sptr
	(new zigbee_reeler_short_impl(debug));
}

zigbee_reeler_short_impl::zigbee_reeler_short_impl(bool debug)
  : gr::block("zigbee_reeler_short",
		  gr::io_signature::make(2, 2, sizeof(float)),
		  gr::io_signature::make(2, 2, sizeof(gr_complex))),
		  d_debug(debug),
		  d_cnt(0),
		  d_pkt_spl_num(0),
		  d_copied(0),
		  d_length(0),
		  d_amp_i(0),
		  d_amp_q(0),
		  d_state(COPY)
{
	set_tag_propagation_policy(block::TPP_DONT);
}


zigbee_reeler_short_impl::~zigbee_reeler_short_impl()
{
}

void
zigbee_reeler_short_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
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
zigbee_reeler_short_impl::general_work (int noutput_items,
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
		//dout << "tag offset: " << offset << std::endl;

		if (offset > nread) {
			ninput = offset - nread;
		} else {
			d_length = pmt::to_long(d_tags.front().value);
			dout << "ZigBee packet is received." << std::endl;
			dout << "Wi-Fi packet is received." << std::endl;
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

		//dout << "REEL" << std::endl;

		for (int index = 0; index < 10; index++) {
			d_amp_i += abs_max(d_buf_real + ZIGBEE_HDR_SAMPLE + 20 * index);
			d_amp_q += abs_max(d_buf_img + ZIGBEE_HDR_SAMPLE + 20 * index);
		}

		//dout << "amp_i: " << d_amp_i / 10 << std::endl;
		//dout << "amp_q: " << d_amp_q / 10 << std::endl;

		make_zigbee_header(d_zigbee_hdr, d_amp_i / 10, d_amp_q / 10);
		d_pkt_spl_num = make_packet(d_zigbee, d_wifi, d_cnt);
		//dout << "d_pkt_spl_num: " << d_pkt_spl_num << std::endl;
		d_amp_i = 0;
		d_amp_q = 0;
		d_state = PADDING_OUT;

		consume_each(0);
		return 0;

	case PADDING_OUT:

		//dout << "PADDING_OUT" << std::endl;

		while (o < noutput) {
			out_zigbee[o] = *(d_zigbee + d_copied);
			out_wifi[o] = *(d_wifi + d_copied);
			o++;
			d_copied++;

			if (d_copied == d_pkt_spl_num) {
				//dout << d_copied << " samples sent." << std::endl;
				d_copied = 0;
				d_pkt_spl_num = 0;
				d_state = COPY;
				break;
			}
		}

		//dout << "o: " << o << std::endl;

		consume_each(0);
		return o;
	}

	throw std::runtime_error("ZigBee reeler (short): unknown state");
	return 0;
}

double zigbee_reeler_short_impl::abs_max(float *buf) {
	double temp = abs(*buf);

	for (int i = 1; i < 20; i++) {
		if (temp < abs(*(buf + i))) {
			temp = abs(*(buf + i));
		}
	}

	return temp;
}

const std::vector<float> zigbee_reeler_short_impl::HALF_SIN = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),

};

const std::vector<float> zigbee_reeler_short_impl::PREAMBLE_HIGH_LOW_REAL = {

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

const std::vector<float> zigbee_reeler_short_impl::PREAMBLE_HIGH_LOW_IMG = {

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

void zigbee_reeler_short_impl::make_zigbee_header(gr_complex *buf, double amp_i, double amp_q) {
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

	//dout << "zigbee header done." << std::endl;
}

int zigbee_reeler_short_impl::make_packet(gr_complex *zigbee, gr_complex *wifi, unsigned long length) {
	int i;

	for (i = 0; i < length; i++) {
		if (i < ZIGBEE_HDR_SAMPLE) {
			*(zigbee + i) = *(d_zigbee_hdr + i);
			*(wifi + i) = gr_complex(d_buf_real[i], d_buf_img[i]) - d_zigbee_hdr[i];
		}
		else {
			*(zigbee + i) = gr_complex(d_buf_real[i], d_buf_img[i]);
			*(wifi + i) = gr_complex(0, 0);
		}
	}

	while (i < length + ZIGBEE_PADDING) {
		*(zigbee + i) = gr_complex(0, 0);
		*(wifi + i) = gr_complex(0, 0);
		i++;
	}

	//i - ZIGBEE_HDR_SAMPLE must be larger than WIFI_PADDING (400).

	//dout << "packet done." << std::endl;

	return i;
}

