
// The MIT License (MIT)

// Copyright (c) 2018 gustavoengdm "Gustavo Teodoro Laureano"

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.


#ifndef _VCAP_DEVICE_HPP_
#define _VCAP_DEVICE_HPP_

#include <linux/videodev2.h>
#include <libv4lconvert.h>

namespace vcap {

	class device {
		public:
			const int fd;	/** File descriptor of device */
		private:
			struct v4l2_capability capability;
			struct v4lconvert_data * v4lconv;
			struct v4l2_requestbuffers reqbuf;
			struct v4l2_format raw_pixel_format, out_pixel_format;
			bool needs_convert;

			struct buffer {
					void * start;
					long int length;
			};
			struct buffer_holder {
					int qtd;
					struct buffer * buffs;
			};
			struct buffer_holder buffers;
			
		public:
			device();
			~device();
		
		public:
			bool open_device( const char * devname );
			bool get_image_dimension( int * width, int * height );
			bool start( void );
			bool grabRGB( unsigned char * data );
			bool stop( void );
			bool close_device( void );

			bool print_info( void );

			int get_fd(void) const;
			bool is_opened(void) const;
			bool set_frame_size( int width, int height );
			bool set_pixel_format( int pixelformat );
	};

} // namespace vcap

#endif