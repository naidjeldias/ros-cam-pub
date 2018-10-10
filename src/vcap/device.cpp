
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

#include "vcap.hpp"
#include "device.hpp"

#include <stdlib.h>

#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>


#include <libv4l2.h>
#include <linux/videodev2.h>

#include "../utils/msg.h"
#include "../utils/mem.hpp"

#define DEBUG 1

namespace vcap {	

	device::device() : fd(-1) {
	}

	device::~device() {}

	bool device::open_device( const char * devname ) {

		util::mem::rw(fd) = open( devname, O_RDWR | O_NONBLOCK );

		if (fd < 0) {
			__msg_perror("Can not open %s device!\n" , devname);
			return false;
		}

		if( -1 == query_capability(this->fd, &this->capability) ) {
			__msg_perror("Can not get capabilities of %s device!\n, ", devname );
			v4l2_close(fd);
			return false;
		}

		if( !(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) ) {
			__msg_warning("%s is not a V4L2_CAP_VIDEO_CAPTURE device!\n", devname );
			v4l2_close(fd);
			return false;
		}

		this->v4lconv = v4lconvert_create(fd);
		if (!v4lconv) {
			__msg_warning( "Unable to create v4lconvert data from this device\n" );
			this->close_device();
			return false;
		}

		return true;
	}

	bool device::get_image_dimension( int * width, int * height ) {
		struct v4l2_format fmt;
		if( -1 == get_format( this->fd, &fmt, V4L2_BUF_TYPE_VIDEO_CAPTURE) ) return false;
		*width = fmt.fmt.pix.width;
		*height = fmt.fmt.pix.height;
		return true;
	}

	bool device::start( void ) {

		// Init MMAP buffers and get the number of buffers
		// --------------------------------------------------------------
		if( -1 == request_buffers( this->fd, &this->reqbuf, 2, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP ) ) {
			return false;
		}


		// if number of buffers is less than 2, then clear the memory driver.
		if( this->reqbuf.count < 2 ) {
			__msg_warning("Insufficient buffer memory.\n");

			// Request 0 buffers to clean memory driver
			// --------------------------------------------------------------
			request_buffers( this->fd, &this->reqbuf, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP );
			return false;
		}

		// Alloc application buffer holders
		// --------------------------------------------------------------
		this->buffers.qtd = reqbuf.count;
		this->buffers.buffs = (struct buffer *) calloc( this->reqbuf.count, sizeof(struct buffer) );
		if ( buffers.buffs == NULL ) {
			__msg_warning("Can't alloc buffer's memory.\n");
			return false;
		}

		// Memory map (MMAP): Request MMAP buffer information.
		// --------------------------------------------------------------
		for (unsigned int i = 0; i < this->reqbuf.count; i++) {

			struct v4l2_buffer buffer = { 0 };

			if( -1 == query_buffer( this->fd, &buffer, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, i) ) {
				__msg_perror("Can't request MMAP buffer information\n");
				return false;
			}
			
			buffers.buffs[i].length = buffer.length; /* remember for munmap() */
			buffers.buffs[i].start = mmap(NULL, //
					buffer.length, //
					PROT_READ | PROT_WRITE, /* recommended */
					MAP_SHARED, /* recommended */
					fd, //
					buffer.m.offset
			);
			
			if (MAP_FAILED == buffers.buffs[i].start) {
				/* If you do not exit here you should unmap() and free() the buffers mapped so far. */
				__msg_perror("Can't do MMAP\n");
				return false;
			}
		}

		// Enqueue all buffers
		// --------------------------------------------------------------
		for (unsigned int i = 0; i < reqbuf.count; ++i) {
			
			struct v4l2_buffer buffer = { 0 };

			memset( &buffer, 0, sizeof(struct v4l2_buffer) );
			buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buffer.memory = V4L2_MEMORY_MMAP;
			buffer.index = i;

			if( -1 ==  xioctl(fd, VIDIOC_QBUF, &buffer) ) {
				__msg_perror("Buffer %u not enqueued... \n", i);
			}
		}

		// Define pixel format and converter
		// --------------------------------------------------------------
		this->raw_pixel_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if( -1 == xioctl(fd, VIDIOC_G_FMT, &this->raw_pixel_format) ) {
			__msg_perror("Can not get Pixel Format... \n");
			return false;
		}

		this->out_pixel_format = this->raw_pixel_format;
		this->out_pixel_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

		if (this->raw_pixel_format.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
			this->needs_convert = true;
		} else {
			this->needs_convert = false;
		}

		if (this->needs_convert) {
			struct v4l2_format copy = this->raw_pixel_format;
			//int k = 
			v4lconvert_try_format( v4lconv, &this->out_pixel_format, &this->raw_pixel_format);
			//printf("k: %d\n\n", k);
			// v4lconvert_try_format sometimes modifies the source format if it thinks
			// that there is a better format available. Restore our selected source
			// format since we do not want that happening.
			this->raw_pixel_format = copy;
		}

		// Stream on
		//------------------------------------------------------------
		if( vcap::streamon( this->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE ) < 0 ) return false;

		return true;
	}

	bool device::grabRGB( unsigned char * rgb ) {	

		fd_set fds;
		struct timeval tv;
		int r;

		struct v4l2_buffer vbuf;
		memset(&vbuf, 0, sizeof(struct v4l2_buffer));

		do {
			FD_ZERO(&fds);
			FD_SET(this->fd, &fds);

			/* Timeout. */
			tv.tv_sec = 1;
			tv.tv_usec = 0;

			r = select(fd + 1, &fds, NULL, NULL, &tv);
		} while ((r == -1 && (errno = EINTR)));

		if (r == -1) {
			perror("select");
			return false;
		}

		/* dequeue buffer */
		// =========================================================================
		bool again = false;
		if (!dequeue_buff(this->fd, &vbuf, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, &again)) {
			__msg_perror("Can't dequeue buffer %u!\n", vbuf.index);
			return false;
		}

		// =========================================================================
		unsigned char * data = (unsigned char *) buffers.buffs[vbuf.index].start;

		if (needs_convert) {
			int err = v4lconvert_convert(v4lconv,
							&this->raw_pixel_format, &this->out_pixel_format,
							(unsigned char *) data, vbuf.bytesused, rgb,
							this->out_pixel_format.fmt.pix.sizeimage);
			if (err < 0) {
				__msg_perror("v4lconvert...\n");
				return false;
			}
		}

		// =========================================================================
		if (!enqueue_buff(this->fd, &vbuf, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, vbuf.index)) {
			__msg_perror("Can't enqueue buffer %u!\n", vbuf.index);
			return false;
		}

		return true;
	}

	bool device::stop( void ) {
		unsigned int i;

		if( vcap::streamoff( this->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE ) < 0 ) {
			__msg_error("Can't streamoff!\n");
		}

		for ( i = 0; i < this->reqbuf.count; i++) {
			if (-1 == munmap(buffers.buffs[i].start, buffers.buffs[i].length)) {
				__msg_perror("Can't unmap buffer %u!\n", i);
			}
		}

		// = free all requested buffers ================================================
		if( vcap::request_buffers( this->fd, &this->reqbuf, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP) < 0 ) {
			__msg_perror( "Can't request 0 mmap buffers to clean MMAP.\n" );
			return false;
		}

		free(this->buffers.buffs);
		this->buffers.buffs = 0;
		this->buffers.qtd = 0;

		
		
		return true;
	}

	bool device::close_device( void ) {
		
		v4lconvert_destroy( this->v4lconv );
		v4l2_close(this->fd);

		return false;
	}

	bool device::print_info( void ) {
		int w, h;
		struct v4l2_capability cap;
		query_capability( this->fd, &cap);
		get_image_dimension( &w, &h);

		printf("-- Device info --\n");
		printf("Card:    %s\n", cap.card );
		printf("Driver:  %s\n", cap.driver );
		printf("Bus:     %s\n", cap.bus_info);
		printf("Size:    %dx%d\n", w, h);

		return true;
	}


	int device::get_fd( void ) const {
		return this->fd;
	}

	bool device::is_opened(void) const {
		return this->fd > 0;
	}

	bool device::set_frame_size( int width, int height ) {
		struct v4l2_format fmt = {};
		if ( vcap::get_format( this->fd, &fmt, V4L2_BUF_TYPE_VIDEO_CAPTURE ) < 0 ) return false;

		fmt.fmt.pix.width = width;
		fmt.fmt.pix.height = height;

		return xioctl(fd, VIDIOC_S_FMT, &fmt) >= 0;
		return true;
	}

	bool device::set_pixel_format( int pixelformat ) {
		return vcap::set_pixel_format( this->fd, pixelformat ) >= 0;
	}

}