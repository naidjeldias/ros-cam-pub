
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


/*
 * v4l_device.hpp
 *
 *  Created on: Jan 27, 2014
 *      Author: gustavo
 */

#ifndef _VCAP_VCAP_HPP_
#define _VCAP_VCAP_HPP_

#include <libv4l2.h>
#include <linux/videodev2.h>

namespace vcap {
	
	bool test_device(int dn);

	bool enum_device_name(char device[16], int ndev_max, bool init = false);

	int xioctl(int fd, int request, void * arg);

	int query_capability(int fd, struct v4l2_capability * cap);

	

	int request_buffers( int fd, struct v4l2_requestbuffers * reqbuf, int nbuffers, enum v4l2_buf_type type, enum v4l2_memory memory );

	bool enqueue_buff(int fd, struct v4l2_buffer * vbuf, enum v4l2_buf_type buf_type, enum v4l2_memory memory, int index);

	bool dequeue_buff( int fd, struct v4l2_buffer * vbuf, enum v4l2_buf_type buf_type, enum v4l2_memory memory, bool * again );

	int streamon( int fd, enum v4l2_buf_type type );

	int streamoff( int fd, enum v4l2_buf_type type );

	int query_buffer(int fd, struct v4l2_buffer * buffer, enum v4l2_buf_type type, enum v4l2_memory memory, unsigned int index);

	//+=========================
	//+= Getters
	//+=========================

	int get_format(int fd, struct v4l2_format * fmt, enum v4l2_buf_type type);
	int get_stream_parameter(int fd, struct v4l2_streamparm * streamparm, unsigned int type);
	int get_control(int fd, struct v4l2_control * ctrl, unsigned int ctrl_id);
	int get_input_index(int fd, unsigned int * input);
	int get_input(int fd, struct v4l2_input * input);
	int get_standard( int fd, struct v4l2_standard * std);

	//+=========================
	//+= Setters
	//+=========================

	int set_format(int fd, struct v4l2_format * fmt, enum v4l2_buf_type type);
	int set_input(int fd, int index);
	int set_standard(int fd, v4l2_std_id std_id);
	int set_pixel_format(int fd, unsigned int pixel_format);
	int set_frame_interval(int fd, struct v4l2_fract & fract);
	int set_control(int fd, int ctrl_id, int value);
	
	//+ =============================
	//+ Enums
	//+ =============================

	bool enum_video_input(int fd, struct v4l2_input * input, int index, bool init);
	bool enum_video_output(int fd, struct v4l2_output * output, int index, bool init);
	bool enum_video_standard(int fd, struct v4l2_standard * std, int index, bool init);
	bool enum_control_default(int fd, struct v4l2_queryctrl * qctrl, bool init);
	bool enum_control_user_base(int fd, struct v4l2_queryctrl * qctrl, bool init);
	bool enum_control_private_base(int fd, struct v4l2_queryctrl * qctrl, bool init);
	bool enum_control_menu(int fd, struct v4l2_querymenu * qmenu, struct v4l2_queryctrl & ctrl, int index, bool init);
	bool enum_pixel_format(int fd, struct v4l2_fmtdesc * fmtdesc, int index, bool init);
	bool enum_frame_size(int fd, struct v4l2_frmsizeenum * frmsize, int pixel_format, int index, bool init);
	bool enum_frame_interval(int fd, struct v4l2_frmivalenum * frmi, int pixel_format, int width, int height, int index, bool init);


}

#endif /* _VCAP__VCAP_VCAP_HPP_ */
