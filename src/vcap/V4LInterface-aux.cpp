/*
 * v4lgui.cpp
 *
 *  Created on: Feb 1, 2014
 *      Author: gustavo
 */

#include "V4LInterface.hpp"

#include <string>
#include <iostream>

#include <vcap.hpp>
#include <msg.h>

#define DEFAULT_STR "unknown"

namespace gui {

	void V4LInterface::__init_combo_boxes() {

		ls_input = Gtk::ListStore::create(model_input);
		cb_input.set_model(ls_input);
		cb_input.pack_start(model_input.m_col_name);

		ls_format_desc = Gtk::ListStore::create(model_format_desc);
		cb_format_desc.set_model(ls_format_desc);
		cb_format_desc.pack_start(model_format_desc.m_col_name);

		ls_standard = Gtk::ListStore::create(model_standard);
		cb_standard.set_model(ls_standard);
		cb_standard.pack_start(model_standard.m_col_name);

		ls_frame_size = Gtk::ListStore::create(model_frame_size);
		cb_frame_size.set_model(ls_frame_size);
		cb_frame_size.pack_start(model_frame_size.m_col_name);

		ls_frame_interval = Gtk::ListStore::create(model_frame_interval);
		cb_frame_interval.set_model(ls_frame_interval);
		cb_frame_interval.pack_start(model_frame_interval.m_col_name);
	}

	void V4LInterface::__create_frm_device_info() {
		Gtk::HBox * hbox;
		Gtk::VBox * vbox;
		Gtk::Label * label;
		Gtk::Table * table;

		frm_device_info.set_border_width(5);
		frm_device_info.set_label("  Device Information  ");

		vbox = new Gtk::VBox();
		frm_device_info.add(*vbox);

		hbox = new Gtk::HBox();
		vbox->pack_start(*hbox, false, true, 0);

		label = new Gtk::Label("Device: ");
		label->set_alignment(1.0, .5);
		hbox->pack_start(*label, false, true, 0);

		hbox->pack_start(cb_device, true, true, 0);

		bt_start.set_label("start");
		hbox->pack_start(bt_start, false, true, 0);

		hbox = new Gtk::HBox();
		vbox->pack_start(*hbox, false, true, 0);

		table = new Gtk::Table(2, 4, false);

		lb_device_name.set_text(DEFAULT_STR);
		lb_device_driver.set_text(DEFAULT_STR);
		lb_device_card.set_text(DEFAULT_STR);
		lb_device_bus.set_text(DEFAULT_STR);

		label = new Gtk::Label("Device: ");
		label->set_alignment(1.0, .5);
		table->attach(*label, 0, 1, 0, 1);
		lb_device_name.set_alignment(0, .5);
		table->attach(lb_device_name, 1, 2, 0, 1);

		label = new Gtk::Label("Card: ");
		label->set_alignment(1.0, .5);
		table->attach(*label, 0, 1, 1, 2);
		lb_device_card.set_alignment(0, .5);
		table->attach(lb_device_card, 1, 2, 1, 2);

		label = new Gtk::Label("Driver: ");
		label->set_alignment(1.0, .5);
		table->attach(*label, 2, 3, 0, 1);
		lb_device_driver.set_alignment(0, .5);
		table->attach(lb_device_driver, 3, 4, 0, 1);

		label = new Gtk::Label("Bus: ");
		label->set_alignment(1.0, .5);
		table->attach(*label, 2, 3, 1, 2);
		lb_device_bus.set_alignment(0, .5);
		table->attach(lb_device_bus, 3, 4, 1, 2);

		hbox->pack_start(*table, true, true, 0);
	}

	void V4LInterface::__create_frm_device_properties() {

		Gtk::Label * label;
		Gtk::Table * table;

		frm_device_prop.set_border_width(5);
		frm_device_prop.set_label("  Device Properties  ");

		table = new Gtk::Table(4, 4, false);

		label = new Gtk::Label("Input: ");
		label->set_alignment(1, .5);
		table->attach(*label, 0, 1, 0, 1, Gtk::FILL, Gtk::EXPAND, 0, 0);
		table->attach(cb_input, 1, 2, 0, 1, Gtk::FILL, Gtk::EXPAND, 0, 0);
		cb_input.set_border_width(0);
		cb_input.set_size_request(100, -1);

		label = new Gtk::Label("Standard: ");
		label->set_alignment(1, .5);
		table->attach(*label, 3, 4, 0, 1, Gtk::FILL, Gtk::EXPAND, 0, 0);
		table->attach(cb_standard, 5, 6, 0, 1, Gtk::FILL, Gtk::EXPAND, 0, 0);
		cb_standard.set_size_request(100, -1);

		label = new Gtk::Label("Format: ");
		label->set_alignment(1, .5);
		table->attach(*label, 0, 1, 1, 2, Gtk::FILL, Gtk::EXPAND, 0, 0);
		table->attach(cb_format_desc, 1, 2, 1, 2, Gtk::FILL, Gtk::EXPAND, 0, 0);

		label = new Gtk::Label("Frame size: ");
		label->set_alignment(1, .5);
		table->attach(*label, 3, 4, 1, 2, Gtk::FILL, Gtk::EXPAND, 0, 0);
		table->attach(cb_frame_size, 5, 6, 1, 2, Gtk::FILL, Gtk::EXPAND, 0, 0);

		label = new Gtk::Label("Intervals: ");
		label->set_alignment(1, .5);
		table->attach(*label, 0, 1, 3, 4, Gtk::FILL, Gtk::EXPAND, 0, 0);
		table->attach(cb_frame_interval, 1, 2, 3, 4, Gtk::FILL, Gtk::EXPAND, 0, 0);

		table->attach(sp_width, 3, 4, 3, 4, Gtk::FILL, Gtk::EXPAND, 0, 0);
		table->attach(sp_height, 5, 6, 3, 4, Gtk::FILL, Gtk::EXPAND, 0, 0);

		notebook.append_page(*table, "Properties");
		notebook.set_border_width(5);
		notebook.set_hexpand(true);

		//notebook.set_scrollable(true);
		frm_device_prop.add(notebook);

		//frm_device_prop.set_label("Device Prop");

	}

	void V4LInterface::__update_cb_device() {

		char device[16];

		if (vcap::enum_device_name(device, 20, true)) {
			do {
				cb_device.append(device);
			} while (vcap::enum_device_name(device, 20, false));
		}
	}

	void V4LInterface::__update_cb_input() {

		cb_input_signal.block(true);
		ls_input->clear();

		struct v4l2_input input;
		unsigned int index;

		vcap::get_input(this->vdev.get_fd(), &input);
		index = input.index;

		if (vcap::enum_video_input(this->vdev.get_fd(), &input, 0, true)) {
			do {
				Gtk::TreeModel::Row row = *(ls_input->append());
				Glib::ustring name = (const char *) input.name;
				row[model_input.m_col_name] = name;
				row[model_input.m_col_data] = input;

				if (input.index == index) {
					cb_input.set_active(row);
				}

			} while (vcap::enum_video_input(this->vdev.get_fd(), &input, 0, false) );
		}

		cb_input_signal.block(false);

	}

	void V4LInterface::__update_cb_standard() {

		cb_standard_signal.block(true);

		ls_standard->clear();

		struct v4l2_standard standard;

		if( vcap::get_standard(this->vdev.get_fd(), &standard) ) {
			unsigned long index = standard.id;

			if (vcap::enum_video_standard(this->vdev.get_fd(), &standard, 0, true)) {
				do {
					Gtk::TreeModel::Row row = *(ls_standard->append());
					Glib::ustring name = (const char *) standard.name;
					row[model_standard.m_col_name] = name;
					row[model_standard.m_col_data] = standard;

					if (standard.id == index) {
						cb_standard.set_active(row);
					}

				} while (vcap::enum_video_standard( this->vdev.get_fd(), &standard, 0, false) );
			}
		}

		cb_standard_signal.block(false);
	}

	void V4LInterface::__update_cb_format_desc() {

		cb_format_desc_signal.block(true);

		ls_format_desc->clear();

		struct v4l2_format fmt;
		struct v4l2_fmtdesc fmt_desc;
		vcap::get_format(this->vdev.get_fd(), &fmt, V4L2_BUF_TYPE_VIDEO_CAPTURE);

		unsigned int index = fmt.fmt.pix.pixelformat;

		if ( vcap::enum_pixel_format(this->vdev.get_fd(), &fmt_desc, 0, true)) {
			do {
				Gtk::TreeModel::Row row = *(ls_format_desc->append());
				//Glib::ustring name = (const char *) fmt_desc.description;
				std::string name = (const char *) fmt_desc.description;

				if (fmt_desc.flags & V4L2_FMT_FLAG_EMULATED) name = name + "(Emulated)";


				row[model_format_desc.m_col_name] = name;
				row[model_format_desc.m_col_data] = fmt_desc;

				if (fmt_desc.pixelformat == index) {
					cb_format_desc.set_active(row);
				}

			} while (vcap::enum_pixel_format( this->vdev.get_fd(), &fmt_desc, 0, false));
		}

		cb_format_desc_signal.block(false);

	}

	void V4LInterface::__update_cb_frame_size() {

		cb_frame_size_signal.block(true);

		ls_frame_size->clear();

		struct v4l2_format fmt;
		vcap::get_format( this->vdev.fd, &fmt, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		sp_width.set_value((double) fmt.fmt.pix.width);
		sp_height.set_value((double) fmt.fmt.pix.height);

		struct v4l2_frmsizeenum frmsize;

		if (vcap::enum_frame_size( this->vdev.fd, &frmsize, fmt.fmt.pix.pixelformat, 0, true)) {
			do {
				Gtk::TreeModel::Row row = *(ls_frame_size->append());
				Glib::ustring name = Glib::ustring::compose("%1x%2", frmsize.discrete.width, frmsize.discrete.height);
				row[model_frame_size.m_col_name] = name;
				row[model_frame_size.m_col_data] = frmsize;

				if (frmsize.discrete.width == fmt.fmt.pix.width && frmsize.discrete.height == fmt.fmt.pix.height) {
					cb_frame_size.set_active(row);
				}

			} while (vcap::enum_frame_size(this->vdev.fd, &frmsize, fmt.fmt.pix.pixelformat, 0, false));
		}
		cb_frame_size_signal.block(false);
	}

	void V4LInterface::__update_cb_frame_interval() {

		cb_frame_interval_signal.block(true);

		ls_frame_interval->clear();

		struct v4l2_format fmt;
		unsigned int & width = fmt.fmt.pix.width;
		unsigned int & height = fmt.fmt.pix.height;

		vcap::get_format(this->vdev.fd, &fmt, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		sp_width.set_value((double) width);
		sp_height.set_value((double) height);

		struct v4l2_frmivalenum frminterval;

		struct v4l2_streamparm streamparm;
		vcap::get_stream_parameter(this->vdev.fd, &streamparm, V4L2_BUF_TYPE_VIDEO_CAPTURE);

		if (vcap::enum_frame_interval(this->vdev.fd, &frminterval, fmt.fmt.pix.pixelformat, width, height, 0, true)) {
			do {
				Gtk::TreeModel::Row row = *(ls_frame_interval->append());
				float fps = (float) frminterval.discrete.denominator / (float) frminterval.discrete.numerator;
				Glib::ustring name = Glib::ustring::compose("%1/%2 = %3 fps", frminterval.discrete.numerator, frminterval.discrete.denominator, fps);
				row[model_frame_interval.m_col_name] = name;
				row[model_frame_interval.m_col_data] = frminterval;

				if (streamparm.parm.capture.timeperframe.numerator == frminterval.discrete.numerator
				        && streamparm.parm.capture.timeperframe.denominator == frminterval.discrete.denominator) {
					cb_frame_interval.set_active(row);
				}

			} while (vcap::enum_frame_interval(this->vdev.fd, &frminterval, fmt.fmt.pix.pixelformat, width, height, 0, false));
		}
		cb_frame_interval_signal.block(false);
	}

	void V4LInterface::__update_all() {
		__update_cb_input();
		__update_cb_standard();
		__update_cb_format_desc();
		__update_cb_frame_size();
		__update_cb_frame_interval();
	}

	void V4LInterface::__make_control_list_default() {

		ctrl_list_default.clear();
		struct v4l2_queryctrl qctrl;
		struct v4l2_querymenu qmenu;

		if (vcap::enum_control_default(this->vdev.fd, &qctrl, true)) {
			do {

				Gtk::Widget * wctrl = 0;
				ControlHolder hold;
				Glib::RefPtr<Gtk::ListStore> lstore;

				switch (qctrl.type) {
					case V4L2_CTRL_TYPE_INTEGER:
					case V4L2_CTRL_TYPE_INTEGER64:

						wctrl = new Gtk::HScale();
						static_cast<Gtk::HScale *>(wctrl)->set_value_pos(Gtk::POS_RIGHT);
						static_cast<Gtk::HScale *>(wctrl)->set_range(qctrl.minimum, qctrl.maximum);
						static_cast<Gtk::HScale *>(wctrl)->set_increments(qctrl.step, 10 * qctrl.step);
						hold.con = static_cast<Gtk::HScale *>(wctrl)->signal_change_value().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control_hscale),
						                &ctrl_list_default, wctrl));
						break;

					case V4L2_CTRL_TYPE_BOOLEAN:

						wctrl = new Gtk::CheckButton((const char *) qctrl.name);
						hold.con = static_cast<Gtk::CheckButton *>(wctrl)->signal_clicked().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_default,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_BUTTON:

						wctrl = new Gtk::Button((const char *) qctrl.name);
						hold.con = static_cast<Gtk::Button *>(wctrl)->signal_clicked().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_default,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_STRING:

						wctrl = new Gtk::Label((const char *) qctrl.name);
						break;

					case V4L2_CTRL_TYPE_MENU:
					case V4L2_CTRL_TYPE_INTEGER_MENU:

						wctrl = new Gtk::ComboBox();
						lstore = Gtk::ListStore::create(model_control_menu);
						static_cast<Gtk::ComboBox *>(wctrl)->set_model(lstore);
						static_cast<Gtk::ComboBox *>(wctrl)->pack_start(model_control_menu.m_col_name);

						if (vcap::enum_control_menu(this->vdev.fd, &qmenu, qctrl, 0, true)) {
							do {
								Gtk::TreeModel::Row row = *(lstore->append());
								Glib::ustring name = (const char *) qmenu.name;
								row[model_control_menu.m_col_name] = name;
								row[model_control_menu.m_col_data] = qmenu;
							} while (vcap::enum_control_menu(this->vdev.fd, &qmenu, qctrl, 0, false));
						}

						hold.con = static_cast<Gtk::ComboBox *>(wctrl)->signal_changed().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_default,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_CTRL_CLASS:
					case V4L2_CTRL_TYPE_BITMASK:
					default:
						wctrl = 0;
						break;

				}

				if (wctrl) {
					hold.qctrl = qctrl;
					hold.widget = wctrl;
					ctrl_list_default.push_back(hold);
				}

			} while (vcap::enum_control_default(this->vdev.fd, &qctrl, false));
		}

	}

	void V4LInterface::__make_control_list_user() {

		ctrl_list_user.clear();
		struct v4l2_queryctrl qctrl;
		struct v4l2_querymenu qmenu;

		if (vcap::enum_control_default(this->vdev.fd, &qctrl, true)) {
			do {

				Gtk::Widget * wctrl = 0;
				ControlHolder hold;
				Glib::RefPtr<Gtk::ListStore> lstore;

				switch (qctrl.type) {
					case V4L2_CTRL_TYPE_INTEGER:
					case V4L2_CTRL_TYPE_INTEGER64:

						wctrl = new Gtk::HScale();
						static_cast<Gtk::HScale *>(wctrl)->set_value_pos(Gtk::POS_RIGHT);
						static_cast<Gtk::HScale *>(wctrl)->set_range(qctrl.minimum, qctrl.maximum);
						static_cast<Gtk::HScale *>(wctrl)->set_increments(qctrl.step, 10 * qctrl.step);
						hold.con = static_cast<Gtk::HScale *>(wctrl)->signal_change_value().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control_hscale),
						                &ctrl_list_user, wctrl));
						break;

					case V4L2_CTRL_TYPE_BOOLEAN:

						wctrl = new Gtk::CheckButton((const char *) qctrl.name);
						hold.con = static_cast<Gtk::CheckButton *>(wctrl)->signal_clicked().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_user,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_BUTTON:

						wctrl = new Gtk::Button((const char *) qctrl.name);
						hold.con = static_cast<Gtk::Button *>(wctrl)->signal_clicked().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_user,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_STRING:

						wctrl = new Gtk::Label((const char *) qctrl.name);
						break;

					case V4L2_CTRL_TYPE_MENU:
					case V4L2_CTRL_TYPE_INTEGER_MENU:

						wctrl = new Gtk::ComboBox();
						lstore = Gtk::ListStore::create(model_control_menu);
						static_cast<Gtk::ComboBox *>(wctrl)->set_model(lstore);
						static_cast<Gtk::ComboBox *>(wctrl)->pack_start(model_control_menu.m_col_name);

						if (vcap::enum_control_menu(this->vdev.fd, &qmenu, qctrl, 0, true)) {
							do {
								Gtk::TreeModel::Row row = *(lstore->append());
								Glib::ustring name = (const char *) qmenu.name;
								row[model_control_menu.m_col_name] = name;
								row[model_control_menu.m_col_data] = qmenu;
							} while (vcap::enum_control_menu(this->vdev.fd, &qmenu, qctrl, 0, false));
						}

						hold.con = static_cast<Gtk::ComboBox *>(wctrl)->signal_changed().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_user,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_CTRL_CLASS:
					case V4L2_CTRL_TYPE_BITMASK:
					default:
						wctrl = 0;
						break;

				}

				if (wctrl) {
					hold.qctrl = qctrl;
					hold.widget = wctrl;
					ctrl_list_user.push_back(hold);
				}

			} while (vcap::enum_control_default(this->vdev.fd, &qctrl, false));
		}

	}

	void V4LInterface::__make_control_list_private() {

		ctrl_list_private.clear();
		struct v4l2_queryctrl qctrl;
		struct v4l2_querymenu qmenu;

		if (vcap::enum_control_default(this->vdev.fd, &qctrl, true)) {
			do {

				Gtk::Widget * wctrl = 0;
				ControlHolder hold;
				Glib::RefPtr<Gtk::ListStore> lstore;

				switch (qctrl.type) {
					case V4L2_CTRL_TYPE_INTEGER:
					case V4L2_CTRL_TYPE_INTEGER64:

						wctrl = new Gtk::HScale();
						static_cast<Gtk::HScale *>(wctrl)->set_value_pos(Gtk::POS_RIGHT);
						static_cast<Gtk::HScale *>(wctrl)->set_range(qctrl.minimum, qctrl.maximum);
						static_cast<Gtk::HScale *>(wctrl)->set_increments(qctrl.step, 10 * qctrl.step);
						hold.con = static_cast<Gtk::HScale *>(wctrl)->signal_change_value().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control_hscale),
						                &ctrl_list_private, wctrl));
						break;

					case V4L2_CTRL_TYPE_BOOLEAN:

						wctrl = new Gtk::CheckButton((const char *) qctrl.name);
						hold.con = static_cast<Gtk::CheckButton *>(wctrl)->signal_clicked().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_private,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_BUTTON:

						wctrl = new Gtk::Button((const char *) qctrl.name);
						hold.con = static_cast<Gtk::Button *>(wctrl)->signal_clicked().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_private,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_STRING:

						wctrl = new Gtk::Label((const char *) qctrl.name);
						break;

					case V4L2_CTRL_TYPE_MENU:
					case V4L2_CTRL_TYPE_INTEGER_MENU:

						wctrl = new Gtk::ComboBox();
						lstore = Gtk::ListStore::create(model_control_menu);
						static_cast<Gtk::ComboBox *>(wctrl)->set_model(lstore);
						static_cast<Gtk::ComboBox *>(wctrl)->pack_start(model_control_menu.m_col_name);

						if (vcap::enum_control_menu(this->vdev.fd, &qmenu, qctrl, 0, true)) {
							do {
								Gtk::TreeModel::Row row = *(lstore->append());
								Glib::ustring name = (const char *) qmenu.name;
								row[model_control_menu.m_col_name] = name;
								row[model_control_menu.m_col_data] = qmenu;
							} while (vcap::enum_control_menu(this->vdev.fd, &qmenu, qctrl, 0, false));
						}

						hold.con = static_cast<Gtk::ComboBox *>(wctrl)->signal_changed().connect(
						        sigc::bind<std::list<ControlHolder> *, Gtk::Widget *>(sigc::mem_fun(*this, &V4LInterface::__set_control), &ctrl_list_private,
						                wctrl));
						break;

					case V4L2_CTRL_TYPE_CTRL_CLASS:
					case V4L2_CTRL_TYPE_BITMASK:
					default:
						wctrl = 0;
						break;

				}

				if (wctrl) {
					hold.qctrl = qctrl;
					hold.widget = wctrl;
					ctrl_list_private.push_back(hold);
				}

			} while (vcap::enum_control_default(this->vdev.fd, &qctrl, false));
		}

	}

	void V4LInterface::__make_control_table(std::list<ControlHolder>& list, const char * title) {

		// create new controls =====================================================
		Gtk::Label * label = NULL;
		Gtk::Table * table = NULL;
		Gtk::Widget * wctrl = NULL;

		int ROWS = 4;
		int COLS = 2;
		int r, c;
		int tab_count = 0;
		int count = 0;

		struct v4l2_queryctrl qctrl;
		std::list<ControlHolder>::iterator iter;

		for (iter = list.begin(); iter != list.end(); ++iter) {
			qctrl = (*iter).qctrl;
			wctrl = (*iter).widget;

			// create a new page of controls
			if (count % (ROWS * COLS) == 0) {
				r = 0;
				c = 0;
				count = 0;
				table = new Gtk::Table(ROWS, COLS * 2, false);
				notebook.append_page(*table, Glib::ustring::compose("%1 %2", title, tab_count++), true);
			}

			r = count / COLS;
			c = count % COLS;

			c = c * 2;

			wctrl->set_size_request(120, -1);

			if (qctrl.type == V4L2_CTRL_TYPE_BOOLEAN || qctrl.type == V4L2_CTRL_TYPE_BUTTON) {

				table->attach(*wctrl, c, c + 2, r, r + 1, Gtk::FILL, Gtk::FILL, 0, 0);

			} else {

				label = new Gtk::Label((const char *) qctrl.name);

				table->attach(*label, c, c + 1, r, r + 1, Gtk::FILL, Gtk::FILL, 0, 0);
				table->attach(*wctrl, c + 1, c + 2, r, r + 1, Gtk::FILL, Gtk::FILL, 0, 0);

				label->set_size_request(-1, -1);
				label->set_max_width_chars(10);
				label->set_alignment(1.0, 0.5);

			}

			count++;

		}

		notebook.show_all();

	}

	void V4LInterface::__block_control_signals(std::list<ControlHolder>& list, bool block) {

		std::list<ControlHolder>::iterator iter;

		for (iter = list.begin(); iter != list.end(); ++iter) {
			(*iter).con.block(block);
		}
	}

	void V4LInterface::__update_control_widgets(std::list<ControlHolder>& ctrl_list) {

		Gtk::Widget * wctrl = NULL;
		struct v4l2_queryctrl qctrl;
		struct v4l2_control control;

		__block_control_signals(ctrl_list, true);

		std::list<ControlHolder>::iterator iter;

		for (iter = ctrl_list.begin(); iter != ctrl_list.end(); ++iter) {
			qctrl = (*iter).qctrl;
			wctrl = (*iter).widget;

			if( vcap::get_control(this->vdev.fd, &control, qctrl.id) == -1 ) {
				__msg_error("Get control...\n");
				return;
			}

			switch (qctrl.type) {
				case V4L2_CTRL_TYPE_INTEGER:
				case V4L2_CTRL_TYPE_INTEGER64:

					static_cast<Gtk::HScale *>(wctrl)->set_value(control.value);
					break;

				case V4L2_CTRL_TYPE_BOOLEAN:

					static_cast<Gtk::CheckButton *>(wctrl)->set_active(control.value == 1);
					break;

				case V4L2_CTRL_TYPE_BUTTON:

					break;

				case V4L2_CTRL_TYPE_STRING:

					break;

				case V4L2_CTRL_TYPE_MENU:
				case V4L2_CTRL_TYPE_INTEGER_MENU:

					Glib::RefPtr<Gtk::TreeModel> model = static_cast<Gtk::ComboBox *>(wctrl)->get_model();

					Gtk::TreeModel::Children child = model->children();

					int i = 0;
					for (Gtk::TreeModel::Children::iterator iter = child.begin(); iter != child.end(); ++iter, ++i) {

						Gtk::TreeModel::Row row = *iter;
						struct v4l2_querymenu qm;
						qm = row[model_control_menu.m_col_data];

						if (control.value < 0) return;

						if (control.value == (int) qm.index) {
							static_cast<Gtk::ComboBox *>(wctrl)->set_active(iter);
							break;
						}

					}

					break;

			}
		}
		__block_control_signals(ctrl_list, false);

	}

	// Constructor

	V4LInterface::V4LInterface() :
			Gtk::VBox(false, 0) {

		notebook.set_scrollable(true);

		sp_width.set_range(0, 2000);
		sp_width.set_increments(1, 10);
		sp_height.set_range(0, 2000);
		sp_height.set_increments(1, 10);
		sp_width.set_tooltip_text("Width");
		sp_height.set_tooltip_text("Height");

		__init_combo_boxes();

		pack_start(frm_device_info, false, false, 5);
		__create_frm_device_info();

		pack_start(frm_device_prop, false, false, 0);
		__create_frm_device_properties();

		__update_cb_device();

		bt_start.signal_clicked().connect(sigc::mem_fun(*this, &V4LInterface::__event_bt_start_clicked));

		cb_input_signal = cb_device.signal_changed().connect(sigc::mem_fun(*this, &V4LInterface::__event_cb_device_changed));
		cb_input_signal = cb_input.signal_changed().connect(sigc::mem_fun(*this, &V4LInterface::__event_cb_input_changed));
		cb_standard_signal = cb_standard.signal_changed().connect(sigc::mem_fun(*this, &V4LInterface::__event_cb_standard_changed));
		cb_format_desc_signal = cb_format_desc.signal_changed().connect(sigc::mem_fun(*this, &V4LInterface::__event_cb_format_desc_changed));
		cb_frame_size_signal = cb_frame_size.signal_changed().connect(sigc::mem_fun(*this, &V4LInterface::__event_cb_frame_size_changed));
		cb_frame_interval_signal = cb_frame_interval.signal_changed().connect(sigc::mem_fun(*this, &V4LInterface::__event_cb_frame_interval_changed));

	}
}
