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

#ifndef INCLUDED_WIFI_ZIGBEE_ZIGBEE_REELER_MEDIUM_IMPL_H
#define INCLUDED_WIFI_ZIGBEE_ZIGBEE_REELER_MEDIUM_IMPL_H

#include <wifi_zigbee/zigbee_reeler_medium.h>

namespace gr {
  namespace wifi_zigbee {

  static const int MAX_SAMPLE_PER_PKT = 133 * 8 * 80;
  static const int ZIGBEE_HDR_SAMPLE = 40 * 4 * 20;

    class zigbee_reeler_medium_impl : public zigbee_reeler_medium
    {
     private:
    	bool d_debug;
    	double d_amp_i;
		double d_amp_q;
    	unsigned long d_length;
		unsigned long d_cnt;
    	unsigned long d_pkt_spl_num;
    	unsigned long d_copied;
		float d_buf_real[MAX_SAMPLE_PER_PKT];
		float d_buf_img[MAX_SAMPLE_PER_PKT + 10]; //10 more elements are for the correlation-based pattern recognition.
		gr_complex d_zigbee_hdr[ZIGBEE_HDR_SAMPLE];
		gr_complex d_zigbee[MAX_SAMPLE_PER_PKT];
		gr_complex d_wifi[MAX_SAMPLE_PER_PKT];
		std::vector<gr::tag_t> d_tags;
		enum {COPY, REEL, PADDING_OUT} d_state;

		static const std::vector<float> HALF_SIN;
		static const std::vector<float> PREAMBLE_HIGH_LOW_REAL;
		static const std::vector<float> PREAMBLE_HIGH_LOW_IMG;


     public:
      zigbee_reeler_medium_impl(bool debug);
      ~zigbee_reeler_medium_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      double abs_max(float *buf);
      int high_low(float *buf);
      void make_zigbee_header(gr_complex *buf, double amp_i, double amp_q);
      int make_packet(gr_complex *zigbee, gr_complex *wifi, unsigned long length, double amp_i, double amp_q);


      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace wifi_zigbee
} // namespace gr

#endif /* INCLUDED_WIFI_ZIGBEE_ZIGBEE_REELER_MEDIUM_IMPL_H */

