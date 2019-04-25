/*
 * v4lgui.cpp
 *
 *  Created on: Feb 1, 2014
 *      Author: gustavo
 */

#include "V4LInterface.hpp"

#include <vcap.hpp>
#include <msg.h>

#include <iostream>

#define DEFAULT_STR " - "

namespace gui {

	// signals

	void V4LInterface::__event_bt_start_clicked() {

		if (!vdev.is_opened()) return;

		Glib::ustring label = bt_start.get_label();

		if (0 == label.compare("start")) {

			bool r;
			// = Pre-configure device ======================================

			// set frame size
			r = vdev.set_frame_size(sp_width.get_value_as_int(), sp_height.get_value_as_int() );
			if (!r) std::cout << "Can't set frame size!" << std::endl;

			// = Init memory map buffers ===================================

			if( !vdev.start() ) {
				__msg_error("Can't start device!\n" );
				return;
			}

			// = Actualize the displayed frame size ========================
			struct v4l2_format format;
			if( vcap::get_format( vdev.fd, &format, V4L2_BUF_TYPE_VIDEO_CAPTURE ) < 0 )
			{
				__msg_error("Can't get pixel format!\n");

			}

			sp_width.set_value(format.fmt.pix.width);
			sp_height.set_value(format.fmt.pix.height);

			bt_start.set_label("stop");
			cb_device.set_sensitive(false);
			cb_input.set_sensitive(false);
			cb_standard.set_sensitive(false);
			cb_frame_size.set_sensitive(false);
			cb_format_desc.set_sensitive(false);
			sp_width.set_sensitive(false);
			sp_height.set_sensitive(false);
			cb_frame_interval.set_sensitive(false);
			m_signal_start.emit(true);

		} else {

			if (!vdev.stop()) {
				std::cout << "Can't stop device!" << std::endl;
				return;
			}

			// if (!vdev.uninit_mmap()) {
			// 	std::cout << "Can't unmmap device memory!" << std::endl;
			// 	return;
			// }

			bt_start.set_label("start");
			cb_device.set_sensitive(true);
			cb_input.set_sensitive(true);
			cb_standard.set_sensitive(true);
			cb_frame_size.set_sensitive(true);
			cb_format_desc.set_sensitive(true);
			sp_width.set_sensitive(true);
			sp_height.set_sensitive(true);
			cb_frame_interval.set_sensitive(true);
			m_signal_start.emit(false);
		}

		return;

	}

	void V4LInterface::__event_cb_device_changed() {
		if (vdev.is_opened()) {
			vdev.close_device();
		}

		Glib::ustring dev = cb_device.get_active_text();

		if (dev.size() < 1) return;

		if (vdev.open_device(dev.data() )) {

			struct v4l2_capability cap;
			if( vcap::query_capability( vdev.fd, &cap ) < 0 ) {
				std::cout << dev.data() << " is not a V4L2 device!" << std::endl;
				vdev.close_device();
				return;
			}

			lb_device_name.set_text(dev.data());
			lb_device_card.set_text((const char *) cap.card);
			lb_device_driver.set_text((const char *) cap.driver);
			lb_device_bus.set_text((const char *) cap.bus_info);

		} else {
			std::cout << "Ooops!" << std::endl;
			return;
		}

		__update_all();

		// free memory =============================================================
		Gtk::Widget * page;

		page = notebook.get_nth_page(1);

		while (page) {
			notebook.remove_page(1);
			delete page;
			page = notebook.get_nth_page(1);
		}

		__make_control_list_default();
		//__make_control_list_user();
		//__make_control_list_private();

		__make_control_table(ctrl_list_default, "Default");
		//__make_control_table(ctrl_list_user, "User");
		//__make_control_table(ctrl_list_private, "Private");

		__update_control_widgets(ctrl_list_default);
		//__update_control_widgets(ctrl_list_user);
		//__update_control_widgets(ctrl_list_private);

	}

	void V4LInterface::__event_cb_input_changed() {

		if (cb_input.get_active_row_number() == -1) return;

		Gtk::TreeModel::iterator it = cb_input.get_active();

		Gtk::TreeModel::Row row = *it;
		struct v4l2_input input = row[model_input.m_col_data];

		int r = vcap::set_input( this->vdev.fd, input.index);
		if (r < 0) {
			__msg_error( "Can't set input!\n");
		}

		__update_all();
	}

	void V4LInterface::__event_cb_standard_changed() {

		if (cb_standard.get_active_row_number() == -1) return;

		Gtk::TreeModel::iterator it = cb_standard.get_active();

		Gtk::TreeModel::Row row = *it;
		struct v4l2_standard standard = row[model_standard.m_col_data];

		int r = vcap::set_standard( this->vdev.fd, standard.id);

		if ( r < 0 ) std::cout << "Can't set standard!" << std::endl;

		__update_all();

	}

	void V4LInterface::__event_cb_format_desc_changed() {

		if (cb_format_desc.get_active_row_number() == -1) return;

		Gtk::TreeModel::iterator it = cb_format_desc.get_active();

		Gtk::TreeModel::Row row = *it;
		struct v4l2_fmtdesc fmtdesc = row[model_format_desc.m_col_data];

		//int r = vdev.set_pixel_format(fmtdesc.pixelformat, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		int r = vcap::set_pixel_format( this->vdev.fd, fmtdesc.pixelformat);
		if (r < 0) 
		{
			__msg_error("Can't set format!\n");
		}

		__update_all();
	}

	void V4LInterface::__event_cb_frame_size_changed() {

		if (cb_frame_size.get_active_row_number() == -1) return;

		Gtk::TreeModel::iterator it = cb_frame_size.get_active();

		Gtk::TreeModel::Row row = *it;
		struct v4l2_frmsizeenum frmsize = row[model_frame_size.m_col_data];

		int r = vdev.set_frame_size(frmsize.discrete.width, frmsize.discrete.height);
		if (r < 0) {
			__msg_error("Can't set frame size!\n");
		}

		__update_all();

	}

	void V4LInterface::__event_cb_frame_interval_changed() {

		if (cb_frame_interval.get_active_row_number() == -1) return;

		Gtk::TreeModel::iterator it = cb_frame_interval.get_active();

		Gtk::TreeModel::Row row = *it;
		struct v4l2_frmivalenum frame_interval = row[model_frame_interval.m_col_data];

		int r = vcap::set_frame_interval( this->vdev.fd, frame_interval.discrete);
		if (r < 0) {
			__msg_error("Can't set frame interval!\n");
		}

		__update_all();

	}

	bool V4LInterface::__set_control_hscale(int type, double val, std::list<ControlHolder> * list, Gtk::Widget * wctrl) {
		std::list<ControlHolder>::iterator iter;

		for (iter = list->begin(); iter != list->end(); ++iter) {
			if ((*iter).widget == wctrl) break;
		}

		int value = static_cast<Gtk::HScale *>(wctrl)->get_value();
		struct v4l2_queryctrl qctrl = (*iter).qctrl;

		if (vcap::set_control(this->vdev.fd, qctrl.id, value) < 0) {
			std::cout << "Can not update control [" << qctrl.name << "] with value " << value << std::endl;
			return false;
		}

		struct v4l2_control ctrl;
		if (vcap::get_control(this->vdev.fd, &ctrl, qctrl.id) < 0) return false;

		__update_control_widgets(ctrl_list_default);
		__update_control_widgets(ctrl_list_user);
		__update_control_widgets(ctrl_list_private);

		return true;

	}

	void V4LInterface::__set_control(std::list<ControlHolder> * list, Gtk::Widget * wctrl) {

		std::list<ControlHolder>::iterator iter;
		for (iter = list->begin(); iter != list->end(); ++iter) {
			if ((*iter).widget == wctrl) break;
		}

		int value;
		struct v4l2_queryctrl qctrl = (*iter).qctrl;

		switch (qctrl.type) {
			case V4L2_CTRL_TYPE_INTEGER:
			case V4L2_CTRL_TYPE_INTEGER64:
			case V4L2_CTRL_TYPE_CTRL_CLASS:
			case V4L2_CTRL_TYPE_BITMASK:
			default:
				break;

//				value = static_cast<Gtk::HScale *>(wctrl)->get_value();
//				if (!vd.set_control(qctrl.id, value)) {
//					std::cout << "Can not update control [" << qctrl.name << "] with value " << value << std::endl;
//				}
//				break;

			case V4L2_CTRL_TYPE_BOOLEAN:
				value = static_cast<Gtk::CheckButton *>(wctrl)->get_active();
				if (vcap::set_control( this->vdev.fd, qctrl.id, value) < 0) {
					std::cout << "Can not update control [" << qctrl.name << "] with value " << value << std::endl;
				}
				break;

			case V4L2_CTRL_TYPE_BUTTON:
				if (vcap::set_control( this->vdev.fd, qctrl.id, 1) < 0) {
					std::cout << "Can not update control [" << qctrl.name << "] with value " << 1 << std::endl;
				}
				break;

			case V4L2_CTRL_TYPE_STRING:
				break;

			case V4L2_CTRL_TYPE_MENU:
			case V4L2_CTRL_TYPE_INTEGER_MENU:

				Gtk::TreeModel::Children::iterator iter = static_cast<Gtk::ComboBox *>(wctrl)->get_active();
				Gtk::TreeModel::Row row = *iter;
				struct v4l2_querymenu qmenu;
				qmenu = row[model_control_menu.m_col_data];
				if (vcap::set_control( this->vdev.fd, qctrl.id, qmenu.index) < 0) {
					std::cout << "Can not update control [" << qctrl.name << "] with value " << qmenu.name << std::endl;
				}
				break;

		}

		__update_control_widgets(ctrl_list_default);
		__update_control_widgets(ctrl_list_user);
		__update_control_widgets(ctrl_list_private);

	}

}
