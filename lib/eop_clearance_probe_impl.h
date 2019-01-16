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

#ifndef INCLUDED_WIFI_ZIGBEE_EOP_CLEARANCE_PROBE_IMPL_H
#define INCLUDED_WIFI_ZIGBEE_EOP_CLEARANCE_PROBE_IMPL_H

#include <wifi_zigbee/eop_clearance_probe.h>
#include <string>
//#include <gnuradio/filter/fir_filter.h>
//#include <gnuradio/fft/fft.h>

using namespace std;

namespace gr {
  namespace wifi_zigbee {

  static const int MAX_SAMPLE_PER_PKT = 133 * 8 * 80;
  //ZigBee pkt size limit: 133 Bytes
  //1 bit corresponds to 4 half sine pulses formed by 80 samples

    class eop_clearance_probe_impl : public eop_clearance_probe
    {
     private:
	  	std::vector<gr::tag_t> d_tags;
    	const bool d_debug;
		unsigned long d_cnt;
		unsigned int d_threshold;
		unsigned long d_copy;
		float d_buf_real[MAX_SAMPLE_PER_PKT];
		float d_buf_img[MAX_SAMPLE_PER_PKT];
		//unsigned int d_peak_num;
		//unsigned int d_preamble_peak_cnt;
		enum {EOP_FOUND, EOP_NOT_FOUND} d_tag_state;
		enum {SEARCH, DETERMINE, COPY} d_state;
		short d_mix_pattern;
		//gr::filter::kernel::fir_filter_fff d_fir;

		static const std::vector<float> HALF_SIN;

     public:
      eop_clearance_probe_impl(bool debug, unsigned int threshold);
      ~eop_clearance_probe_impl();


      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      short find_mix_pattern(float *buf_real, float *buf_img, unsigned long length);
      void insert_tag(unsigned int out_index, uint64_t item, long spl_cnt, string str);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace wifi_zigbee
} // namespace gr

#endif /* INCLUDED_WIFI_ZIGBEE_EOP_CLEARANCE_PROBE_IMPL_H */

