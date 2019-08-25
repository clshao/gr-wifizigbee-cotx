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


#ifndef INCLUDED_WIFI_ZIGBEE_ZERO_PADDING_ADD_H
#define INCLUDED_WIFI_ZIGBEE_ZERO_PADDING_ADD_H

#include <wifi_zigbee/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace wifi_zigbee {

    /*!
     * \brief <+description of block+>
     * \ingroup wifi_zigbee
     *
     */
    class WIFI_ZIGBEE_API zero_padding_add : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<zero_padding_add> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of wifi_zigbee::zero_padding_add.
       *
       * To avoid accidental use of raw pointers, wifi_zigbee::zero_padding_add's
       * constructor is in a private implementation
       * class. wifi_zigbee::zero_padding_add::make is the public interface for
       * creating new instances.
       */
      static sptr make(int vlen);
    };

  } // namespace wifi_zigbee
} // namespace gr

#endif /* INCLUDED_WIFI_ZIGBEE_ZERO_PADDING_ADD_H */

