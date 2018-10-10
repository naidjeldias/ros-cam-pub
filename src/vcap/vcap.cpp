
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

#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <libv4l2.h>
#include <linux/videodev2.h>

#include "../utils/msg.h"

#include "vcap.hpp"

namespace vcap
{

	bool test_device(int dn)
	{
		char device[16];
		struct stat st;
		if (dn < 0)
			return false;
		snprintf(device, 15, "/dev/video%d", dn);
		if (stat(device, &st) != 0)
			return false;
		return true;
	}

	bool enum_device_name(char device[16], int ndev_max, bool init)
	{
		static int __dev_count = 0;
		bool not_found = true;
		struct stat st;
		if (init)
			__dev_count = 0;
		while (not_found && __dev_count < ndev_max)
		{
			snprintf(device, 15, "/dev/video%d", __dev_count);
			++__dev_count;
			if (stat(device, &st) != 0)
				continue;
			not_found = false;
		}
		return !not_found;
	}

	int xioctl(int fd, int request, void *arg)
	{
		int r;
		r = ioctl(fd, request, arg);
		return r;
	}

	int query_capability(int fd, struct v4l2_capability *cap)
	{
		int r = xioctl(fd, VIDIOC_QUERYCAP, cap);
		return r;
	}

	int request_buffers(int fd, struct v4l2_requestbuffers *reqbuf, int nbuffers, enum v4l2_buf_type type, enum v4l2_memory memory)
	{
		int r;
		memset(reqbuf, 0, sizeof(struct v4l2_requestbuffers));

		reqbuf->count = nbuffers;
		reqbuf->type = type;
		reqbuf->memory = memory;

		r = xioctl(fd, VIDIOC_REQBUFS, reqbuf);
		if (r >= 0)
			return true;
		__msg_perror("VIDIOC_REQBUFS\n");
		return r;
	}

	bool enqueue_buff(int fd, struct v4l2_buffer *vbuf, enum v4l2_buf_type buf_type, enum v4l2_memory memory, int index)
	{
		memset(vbuf, 0, sizeof(*vbuf));

		vbuf->type = buf_type;
		vbuf->memory = memory;
		vbuf->index = index;

		if (-1 == xioctl(fd, VIDIOC_QBUF, vbuf))
			return false;

		return true;
	}

	bool dequeue_buff(int fd, struct v4l2_buffer *vbuf, enum v4l2_buf_type buf_type, enum v4l2_memory memory, bool *again)
	{
		int r;
		memset(vbuf, 0, sizeof(struct v4l2_buffer));

		vbuf->type = buf_type;
		vbuf->memory = memory;

		r = xioctl(fd, VIDIOC_DQBUF, vbuf);

		// if the buffer is not avaliable, try again.
		*again = (r < 0) && (errno == EAGAIN);
		return (r >= 0) || (*again);
	}

	int streamon(int fd, enum v4l2_buf_type type)
	{
		int r = xioctl(fd, VIDIOC_STREAMON, &type);
		if (-1 == r)
		{
			__msg_perror("Can not start streaming...(VIDIOC_STREAMON)\n");
		}
		return r;
	}

	int streamoff(int fd, enum v4l2_buf_type type)
	{
		int r = xioctl(fd, VIDIOC_STREAMOFF, &type);
		if (-1 == r)
		{
			__msg_perror("Can not start streaming... (VIDIOC_STREAMOFF)\n");
		}
		return r;
	}

	int query_buffer(int fd, struct v4l2_buffer *buffer, enum v4l2_buf_type type, enum v4l2_memory memory, unsigned int index)
	{

		int r;

		memset(buffer, 0, sizeof(struct v4l2_buffer));
		buffer->type = type;
		buffer->memory = memory;
		buffer->index = index;

		r = xioctl(fd, VIDIOC_QUERYBUF, buffer);
		if (r < 0)
		{
			__msg_perror("VIDIOC_QUERYBUF\n");
		}
		return r;
	}

	int query_control(int fd, struct v4l2_queryctrl * qctrl, int id) {
		qctrl->id = id;
		return xioctl(fd, VIDIOC_QUERYCTRL, qctrl);
	}
	int query_control_menu(int fd, struct v4l2_querymenu * qmenu) {
		return xioctl(fd, VIDIOC_QUERYMENU, qmenu);
	}

	//+ =============================
	//+ = Getters
	//+ =============================

	int get_format(int fd, struct v4l2_format *fmt, enum v4l2_buf_type type)
	{
		fmt->type = type;
		return xioctl(fd, VIDIOC_G_FMT, fmt);
	}

	int get_input_index(int fd, unsigned int *input)
	{
		return xioctl(fd, VIDIOC_G_INPUT, input);
	}
	int get_input(int fd, struct v4l2_input *input)
	{
		unsigned int index;
		if (get_input_index(fd, &index) < 0)
			return -1;
		input->index = index;
		return xioctl(fd, VIDIOC_ENUMINPUT, input);
	}

	int get_stream_parameter(int fd, struct v4l2_streamparm *streamparm, unsigned int type)
	{
		streamparm->type = type;
		return xioctl(fd, VIDIOC_G_PARM, streamparm);
	}

	int get_control(int fd, struct v4l2_control *ctrl, unsigned int ctrl_id)
	{
		ctrl->id = ctrl_id;
		return xioctl(fd, VIDIOC_G_CTRL, ctrl);
	}

	int get_standard_id(int fd, v4l2_std_id * std_id) {
		return xioctl(fd, VIDIOC_G_STD, std_id);
	}
	int get_standard(int fd, struct v4l2_standard * standard) {
		v4l2_std_id std_id;

		if ( xioctl(fd, VIDIOC_G_STD, &std_id) < 0 ) return false;

		if (enum_video_standard(fd, standard, 0, true)) {
			do {
				if (standard->id == std_id) return true;
			} while (enum_video_standard(fd, standard, 0, false) );
		}
		return false;
	}

	//+ =============================
	//+ = Setters
	//+ =============================

	int set_format(int fd, struct v4l2_format *fmt, enum v4l2_buf_type type)
	{
		fmt->type = type;
		return xioctl(fd, VIDIOC_S_FMT, fmt);
	}

	int set_input(int fd, int index)
	{
		return xioctl(fd, VIDIOC_S_INPUT, &index);
	}
	int set_standard(int fd, v4l2_std_id std_id)
	{
		return xioctl(fd, VIDIOC_S_STD, &std_id);
	}

	int set_pixel_format(int fd, unsigned int pixel_format)
	{

		struct v4l2_format fmt;
		int r;

		r = get_format(fd, &fmt, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		if (r < 0)
		{
			__msg_perror("Can't get pixel format\n");
			return r;
		}
		fmt.fmt.pix.pixelformat = pixel_format;

		return xioctl(fd, VIDIOC_S_FMT, &fmt);
	}

	int set_frame_interval(int fd, struct v4l2_fract &fract)
	{
		int r;
		v4l2_streamparm parm;

		r = get_stream_parameter(fd, &parm, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		if (r < 0)
		{
			__msg_perror("Can't get stream parameter\n");
			return r;
		}

		if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
		{
			__msg_error("Device doesn't support \"timeperframe\" capability.\n");
			return -1;
		}

		parm.parm.capture.timeperframe = fract;

		return xioctl(fd, VIDIOC_S_PARM, &parm);
	}

	int set_control(int fd, int ctrl_id, int value)
	{
		struct v4l2_control ctrl;
		ctrl.id = ctrl_id;
		ctrl.value = value;
		return xioctl(fd, VIDIOC_S_CTRL, &ctrl);
	}

	
	//+ =============================
	//+ Enums
	//+ =============================

	bool enum_video_input(int fd, struct v4l2_input * input, int index, bool init) {
		if (init) input->index = index;
		else input->index++;
		return xioctl(fd, VIDIOC_ENUMINPUT, input) >= 0;
	}
	bool enum_video_standard(int fd, struct v4l2_standard * std, int index, bool init) {
		if (init) std->index = index;
		else std->index++;

		return xioctl(fd, VIDIOC_ENUMSTD, std) >= 0;
	}


	bool enum_video_output(int fd, struct v4l2_output * output, int index, bool init) {
		if (init) output->index = index;
		else output->index++;

		return xioctl(fd, VIDIOC_ENUMOUTPUT, output) >= 0;
	}

	// TODO: needs testing
	bool enum_control_default(int fd, struct v4l2_queryctrl * qctrl, bool init) {
		if (init) qctrl->id = V4L2_CTRL_FLAG_NEXT_CTRL;
		else qctrl->id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		return query_control(fd, qctrl, qctrl->id ) >= 0;
	}

	// TODO: needs testing
	bool enum_control_user_base(int fd, struct v4l2_queryctrl * qctrl, bool init) {
		if (init) qctrl->id = V4L2_CID_BASE;
		else {
			qctrl->id ++;
			if (qctrl->id >= V4L2_CID_LASTP1) return false;
		}

		while ( query_control(fd, qctrl, qctrl->id ) != -1 ) {
			qctrl->id++;
			if (qctrl->id > V4L2_CID_LASTP1) return false;
		}
		return true;
	}

	// TODO: needs testing
	bool enum_control_private_base(int fd, struct v4l2_queryctrl * qctrl, bool init) {
		if (init) qctrl->id = V4L2_CID_PRIVATE_BASE;
		else qctrl->id++;
		if ( query_control(fd, qctrl, qctrl->id ) < 0 ) return false;
		return true;
	}

	// TODO: needs testing
	bool enum_control_menu(int fd, struct v4l2_querymenu * qmenu, struct v4l2_queryctrl & ctrl, int index, bool init) {

		if (init) {
			memset(qmenu, 0, sizeof(struct v4l2_querymenu));
			qmenu->id = ctrl.id;
			qmenu->index = ctrl.minimum;
		} else qmenu->index++;

		if ((int) qmenu->index > ctrl.maximum) return false;

		while ( query_control_menu(fd, qmenu) == -1 ) {
			qmenu->index++;
		}

		return true;
	}

	bool enum_pixel_format(int fd, struct v4l2_fmtdesc * fmtdesc, int index, bool init) {
		if (init) {
			memset(fmtdesc, 0, sizeof(struct v4l2_fmtdesc));
			fmtdesc->index = index;
		} else fmtdesc->index++;

		fmtdesc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		return xioctl(fd, VIDIOC_ENUM_FMT, fmtdesc) >= 0;
	}

	bool enum_frame_size(int fd, struct v4l2_frmsizeenum * frmsize, int pixel_format, int index, bool init) {

		if (init) frmsize->index = index;
		else ++frmsize->index;

		frmsize->pixel_format = pixel_format;
		return xioctl(fd, VIDIOC_ENUM_FRAMESIZES, frmsize) >= 0;
	}

	bool enum_frame_interval(int fd, struct v4l2_frmivalenum * frmi, int pixel_format, int width, int height, int index, bool init) {

		if (init) frmi->index = 0;
		else frmi->index++;

		frmi->pixel_format = pixel_format;
		frmi->width = width;
		frmi->height = height;
		return xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, frmi) >= 0;
	}


} // namespace vcap
