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

#ifndef INCLUDED_WIFI_ZIGBEE_HDR_CLEARANCE_PROBE_IMPL_H
#define INCLUDED_WIFI_ZIGBEE_HDR_CLEARANCE_PROBE_IMPL_H

#include <wifi_zigbee/hdr_clearance_probe.h>
#include <string>

using namespace std;

namespace gr {
  namespace wifi_zigbee {

  static const int MAX_SAMPLE_PER_PKT = 133 * 8 * 80;

    class hdr_clearance_probe_impl : public hdr_clearance_probe
    {
     private:
    	bool d_debug;
		unsigned int d_pwr_threshold;
		unsigned int d_sc_threshold;
		unsigned long d_cnt;
		unsigned long d_length;
		unsigned long d_copy;
		gr_complex d_buf[MAX_SAMPLE_PER_PKT];
		std::vector<gr::tag_t> d_tags;
		enum {SEARCH, DETERMINE, COPY} d_state;
		short d_mix_pattern;

     public:
      hdr_clearance_probe_impl(bool debug, unsigned int pwr_threshold, unsigned int sc_threshold);
      ~hdr_clearance_probe_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      int fft_80(gr_complex *time);
      short find_mix_pattern(gr_complex *buf, unsigned long length);
      void insert_tag(unsigned int out_index, uint64_t item, long spl_cnt, string str);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace wifi_zigbee
} // namespace gr

#endif /* INCLUDED_WIFI_ZIGBEE_HDR_CLEARANCE_PROBE_IMPL_H */

