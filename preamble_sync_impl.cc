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
#include <cassert>
#include <utility>
#include <algorithm>

#define dout d_debug && std::cout

using namespace gr::wifi_zigbee;
using namespace std;

static const float EOP_DETECTION_THRESHOLD = 0.1;
static const int ZERO_CNT_LIMIT = 60;
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
		  d_fir_real(gr::filter::kernel::fir_filter_fff(1, FINE_REAL_CORRELATION)),
		  d_fir_img(gr::filter::kernel::fir_filter_fff(1, FINE_IMG_CORRELATION)),
		  d_debug(debug),
		  d_preamble_spl_cnt(0),
		  d_threshold(threshold),
		  d_preamble_peak_cnt(0),
		  d_pkt_index(0),
		  //d_head_cnt(0),
		  //d_residual_padding(0),
		  d_offset(0),
		  d_spl_index_real(0),
		  d_spl_index_img(0),
		  d_frame_start(0),
		  d_reset_cnt(0),
		  d_copy(0),
		  d_state(SEARCH),
		  d_eop_state(EOP_NOT_FOUND)
{
	set_tag_propagation_policy(block::TPP_DONT);
	d_fir_real_result = gr::fft::malloc_float(8192);
	d_fir_img_result = gr::fft::malloc_float(8192);
}


preamble_sync_impl::~preamble_sync_impl()
{
	gr::fft::free(d_fir_real_result);
	gr::fft::free(d_fir_img_result);
}

void
preamble_sync_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  	if (d_state == SEARCH || d_state == SYNC || d_state == RELEASE || d_state == FIND) {
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
  int o = 0;

  switch(d_state) {
  case SEARCH:

	  //dout << "Enter SEARCH." << std::endl;

	  while (i < ninput) {
		  if (d_preamble_spl_cnt < 80) {
			  d_preamble_buf_real[d_preamble_spl_cnt] = in_real[i];
			  d_preamble_buf_img[d_preamble_spl_cnt] = in_img[i];
			  d_preamble_spl_cnt++;
			  i++;

			  if (d_preamble_spl_cnt == 80) {
				  if (make_correlation(d_preamble_buf_real, d_preamble_buf_img)) {
					  //dout << "in_real: " << d_preamble_buf_real[0] << std::endl;
					  //d_cor_head_buf[d_head_cnt] = d_preamble_buf_real[0];
					  //d_head_cnt++;
					  d_preamble_peak_cnt++;
				  }
			  }
		  }
		  else if (d_preamble_spl_cnt == 80) {
			  float preamble_buf_real[79];
			  float preamble_buf_img[79];

			  std::memcpy(preamble_buf_real, d_preamble_buf_real + 1, 79 * sizeof(float));
			  std::memcpy(d_preamble_buf_real, preamble_buf_real, 79 * sizeof(float));
			  d_preamble_buf_real[79] = in_real[i];
			  std::memcpy(preamble_buf_img, d_preamble_buf_img + 1, 79 * sizeof(float));
			  std::memcpy(d_preamble_buf_img, preamble_buf_img, 79 * sizeof(float));
			  d_preamble_buf_img[79] = in_img[i];
			  i++;

			  if (make_correlation(d_preamble_buf_real, d_preamble_buf_img)) {
				  //dout << "in_real: " << d_preamble_buf_real[0] << std::endl;
				  //d_cor_head_buf[d_head_cnt] = d_preamble_buf_real[0];
				  //d_head_cnt++;
				  d_preamble_peak_cnt++;
				  if (d_preamble_peak_cnt == PREAMBLE_PEAK_CNT) {
					  d_pkt_index++;
					  d_state = SYNC;
					  //d_residual_padding = residual_padding_cnt(d_cor_head_buf);
					  //dout << "residual padding: " << d_residual_padding << std::endl;
					  //The number of samples to be consumed before copying.
					  //d_head_cnt = 0;
					  d_preamble_peak_cnt = 0;
					  d_preamble_spl_cnt = 0;
					  dout << "Signal found!" << std::endl;
					  break;
				  }
			  }
		  }
	  }

	  consume_each (i);
	  return 0;

  case SYNC:

	  d_fir_real.filterN(d_fir_real_result, in_delay_real, std::min(SYNC_LENGTH, std::max(ninput - 119, 0)));
	  d_fir_img.filterN(d_fir_img_result, in_delay_img, std::min(SYNC_LENGTH, std::max(ninput - 119, 0)));

	  while (i + 119 < ninput) {

		  d_fine_sync_real[d_spl_index_real] = in_delay_real[i];
		  d_fine_sync_img[d_spl_index_img] = in_delay_img[i];
		  d_cor_real.push_back(pair<float, int>(d_fir_real_result[i], d_spl_index_real));
		  d_cor_img.push_back(pair<float, int>(d_fir_img_result[i], d_spl_index_img));

		  i++;
		  d_spl_index_real++;
		  d_spl_index_img++;

		  if (d_spl_index_real == SYNC_LENGTH) {
			  search_frame_start();
			  d_reset_cnt = (SYNC_LENGTH - d_frame_start - 1) % 80;
			  d_spl_index_real = 0;
			  d_spl_index_img = 0;
			  d_state = RELEASE;
			  insert_tag(0, nitems_written(0), d_pkt_index, "signal start");
			  insert_tag(1, nitems_written(1), d_pkt_index, "signal start");
			  dout << "Signal synchronized!" << std::endl;
			  dout << "d_frame_start: " << d_frame_start << std::endl;
			  /*for (int m = 0; m <= d_frame_start; m++) {
				  dout << d_fine_sync_real[m] << std::endl;
			  }*/
			  break;
		  }
	  }

	  consume_each (i);
	  return 0;

  case RELEASE:

	  while (o < noutput && d_frame_start < SYNC_LENGTH) {
		  out_real[o] = d_fine_sync_real[d_frame_start];
		  //dout << "out_real: " << d_fine_sync_real[d_frame_start] << std::endl;
		  out_img[o] = d_fine_sync_img[d_frame_start];
		  //dout << "out_img: " << d_fine_sync_img[d_frame_start] << std::endl;
		  o++;
		  d_frame_start++;
	  }

	  if (d_frame_start == SYNC_LENGTH) {

		  while (o < noutput && i < ninput && d_reset_cnt <= 80) {
			 out_real[o] = in_delay_real[i];
			 out_img[o] = in_delay_img[i];
			 i++;
			 o++;
			 d_reset_cnt++;
		 }

		  if (d_reset_cnt == 81) {
			  d_state = FIND;
			  d_frame_start = 0;
			  d_reset_cnt = 0;
			  dout << "REALESE complete." << std::endl;
		  }
	  }

	  consume_each (i);
	  return o;

  case FIND:
	  //dout << "Enter FIND." << std::endl;

	  int zero_padding_offset;

	  while (i < ninput) {
/*		  if (d_residual_padding > 0) {
			  d_residual_padding--;
			  dout << "noise on delay real input: " << in_delay_real[i] << std::endl;
			  i++;
		  }
*/		  //else {
		  d_register_real[d_offset % 80] = in_delay_real[i];
		  d_register_img[d_offset % 80] = in_delay_img[i];
		  i++;
		  if (d_offset % 80 == 79) {
/*			  if (d_offset == 79) {
				  for (int temp = 0; temp < 80; temp++) {
					  dout << "in_delay_real[" << temp << "]: " << d_register_real[temp] << std::endl;
				  }
			  }
*/			  zero_padding_offset = find_zero_padding(d_register_real, d_register_img);
			  //dout << "zerp padding offset: " << zero_padding_offset << std::endl;
			  d_state = COPY;
			  if (zero_padding_offset != 80) {
				  d_eop_state = EOP_FOUND;
				  //dout << "d_offset - 80: " << d_offset - 80 << std::endl;
				  /*for (int m = 0; m < 75; m++) {
					  dout << d_register_real[m % 75] << " " << std::endl;
				  }*/
				  break;
			  }
			  else {
				  d_eop_state = EOP_NOT_FOUND;
				  //dout << d_offset << std::endl;
				  d_offset++;
				  break;
			  }
		  }
		  d_offset++;
		  //}
	  }

	  //dout << "i: " << i << std::endl;

	  consume_each (i);
	  return 0;

  case COPY:

	  //dout << "Enter COPY." << std::endl;

	  switch (d_eop_state) {
	  case EOP_FOUND:

		  dout << "EOP_FOUND!" << std::endl;
		  dout << "d_offset: " << d_offset << std::endl;

		  out_real[o] = 0;
		  out_img[o] = 0;
		  insert_tag(0, nitems_written(0), d_pkt_index, "signal end");
		  insert_tag(1, nitems_written(1), d_pkt_index, "signal end");
		  dout << "Tag written." << nitems_written(0) << std::endl;
		  d_state = SEARCH;
		  d_offset = 0;

		  consume_each(0);
		  return 1;

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

int preamble_sync_impl::make_correlation(float *buf_real, float *buf_img) {
	float sum_real = 0;
	float sum_img = 0;

	for (int temp = 0; temp < 80; temp++) {
		sum_real += buf_real[temp] * COARSE_REAL_CORRELATION[temp];
		sum_img += buf_img[temp] * COARSE_IMG_CORRELATION[temp];
	}

	//dout << "sum_real: " << sum_real << std::endl;
	//dout << "sum_img: " << sum_img << std::endl;

	if ((sum_real * 5 > d_threshold) && (sum_img * 5 > d_threshold)) {
		//dout << "sum_real: " << sum_real << std::endl;
		//dout << "sum_img: " << sum_img << std::endl;
		return 1;
	} else return 0;
}

bool compare_float(const std::pair<float, int>& first, const std::pair<float, int>& second) {
	return	std::get<0>(first) > std::get<0>(second);
}

void preamble_sync_impl::search_frame_start() {

	assert(d_cor_real.size() == SYNC_LENGTH);
	assert(d_cor_img.size() == SYNC_LENGTH);
	d_cor_real.sort(compare_float);
	d_cor_img.sort(compare_float);


	vector<pair<float, int>> vec_real(d_cor_real.begin(), d_cor_real.end());
	d_cor_real.clear();
	vector<pair<float, int>> vec_img(d_cor_img.begin(), d_cor_img.end());
	d_cor_img.clear();

	vector<int> frame_start;
	frame_start.reserve(4);

	frame_start[0] = std::get<1>(vec_real[0]);
	frame_start[1] = std::get<1>(vec_real[1]);
	frame_start[2] = std::get<1>(vec_real[2]);
	frame_start[3] = std::get<1>(vec_real[3]);

	dout << "vec_real[0]: " << std::get<1>(vec_real[0]) << std::endl;
	dout << "vec_real[1]: " << std::get<1>(vec_real[1]) << std::endl;
	dout << "vec_real[2]: " << std::get<1>(vec_real[2]) << std::endl;
	dout << "vec_real[3]: " << std::get<1>(vec_real[3]) << std::endl;
	dout << "vec_real[4]: " << std::get<1>(vec_real[4]) << std::endl;
	dout << "vec_real[5]: " << std::get<1>(vec_real[5]) << std::endl;
	dout << "vec_real[6]: " << std::get<1>(vec_real[6]) << std::endl;
	dout << "vec_real[7]: " << std::get<1>(vec_real[7]) << std::endl;
	dout << "vec_real[8]: " << std::get<1>(vec_real[8]) << std::endl;
	dout << "vec_real[9]: " << std::get<1>(vec_real[9]) << std::endl;
	//dout << "vec_img[0]: " << std::get<1>(vec_img[0]) << std::endl;
	//dout << "vec_img[1]: " << std::get<1>(vec_img[1]) << std::endl;
	//dout << "vec_img[2]: " << std::get<1>(vec_img[2]) << std::endl;
	//dout << "vec_img[3]: " << std::get<1>(vec_img[3]) << std::endl;
	//dout << "vec_img[4]: " << std::get<1>(vec_img[4]) << std::endl;
	//dout << "vec_img[5]: " << std::get<1>(vec_img[5]) << std::endl;
	//dout << "vec_img[6]: " << std::get<1>(vec_img[6]) << std::endl;
	//dout << "vec_img[7]: " << std::get<1>(vec_img[7]) << std::endl;



	std::sort(frame_start.begin(), frame_start.end());

	d_frame_start = SYNC_LENGTH;

	for (int i = 0; i < 3; i++) {
		if (d_fine_sync_real[frame_start[i]] > EOP_DETECTION_THRESHOLD) {
			d_frame_start = frame_start[i];
		}
	}

/*	if (frame_start[0] == (frame_start[1] - 10)) {
		d_frame_start = frame_start[0];
	}
	else if (frame_start[0] == (frame_start[2] - 40)) {
		d_frame_start = frame_start[0];
	}
	else if ((frame_start[1] - 10) == (frame_start[2] - 40)) {
		d_frame_start = frame_start[1] - 10;
	}
	else {
		frame_start[1] = frame_start[1] - 10;
		frame_start[2] = frame_start[2] - 40;
		std::sort(frame_start.begin(), frame_start.end());
		d_frame_start = frame_start[1];
	}*/
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

	if (zero_cnt >= ZERO_CNT_LIMIT) {
/*		for (int i = 0; i < 80; i++) {
			dout << buf_real[i] << std::endl;
		}
*/		return 0;
	}
	else return 80;

}

const std::vector<float> preamble_sync_impl::COARSE_REAL_CORRELATION = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),

};

const std::vector<float> preamble_sync_impl::COARSE_IMG_CORRELATION = {

0, 0, 0, 0, 0,
0, 0, 0, 0, 0,
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),

};

const std::vector<float> preamble_sync_impl::FINE_REAL_CORRELATION = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),

};

const std::vector<float> preamble_sync_impl::FINE_IMG_CORRELATION = {

sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
sin(0*M_PI/20), sin(1*M_PI/20), sin(2*M_PI/20), sin(3*M_PI/20), sin(4*M_PI/20),
sin(5*M_PI/20), sin(6*M_PI/20), sin(7*M_PI/20), sin(8*M_PI/20), sin(9*M_PI/20),
sin(10*M_PI/20), sin(11*M_PI/20), sin(12*M_PI/20), sin(13*M_PI/20), sin(14*M_PI/20),
sin(15*M_PI/20), sin(16*M_PI/20), sin(17*M_PI/20), sin(18*M_PI/20), sin(19*M_PI/20),
-sin(0*M_PI/20), -sin(1*M_PI/20), -sin(2*M_PI/20), -sin(3*M_PI/20), -sin(4*M_PI/20),
-sin(5*M_PI/20), -sin(6*M_PI/20), -sin(7*M_PI/20), -sin(8*M_PI/20), -sin(9*M_PI/20),
-sin(10*M_PI/20), -sin(11*M_PI/20), -sin(12*M_PI/20), -sin(13*M_PI/20), -sin(14*M_PI/20),
-sin(15*M_PI/20), -sin(16*M_PI/20), -sin(17*M_PI/20), -sin(18*M_PI/20), -sin(19*M_PI/20),

};



