/*
 * v4lgui.hpp
 *
 *  Created on: Feb 1, 2014
 *      Author: gustavo
 */

#ifndef V4LINTERFACE_HPP_
#define V4LINTERFACE_HPP_

#include <iostream>
#include <gtkmm.h>

#include <linux/videodev2.h>

#include <device.hpp>

namespace gui {

	class V4LInterface: public Gtk::VBox {

		public:
			vcap::device vdev;
			V4LInterface();

			// void grab_rgb(unsigned char * rgb) {
			// 	std::cout << "Grabbing\n";
			// 	vcap.grabRGB(rgb);
			// }

		private:
			void __init_combo_boxes();
			void __create_frm_device_info();
			void __create_frm_device_properties();

		private:
			// Combo properties updates
			void __update_cb_device();
			void __update_cb_input();
			void __update_cb_standard();
			void __update_cb_format_desc();
			void __update_cb_frame_size();
			void __update_cb_frame_interval();
			void __update_all();

		private:
			// Signals
			sigc::connection cb_input_signal;
			sigc::connection cb_standard_signal;
			sigc::connection cb_format_desc_signal;
			sigc::connection cb_frame_size_signal;
			sigc::connection cb_frame_interval_signal;

			void __event_bt_start_clicked();
			void __event_cb_device_changed();
			void __event_cb_input_changed();
			void __event_cb_standard_changed();
			void __event_cb_format_desc_changed();
			void __event_cb_frame_size_changed();
			void __event_cb_frame_interval_changed();

		public:
			/* Signals */
			typedef sigc::signal<bool, bool> SignalStart;

			SignalStart signal_start() {
				return m_signal_start;
			}
		protected:
			SignalStart m_signal_start;

		private:

			//==================================================================
			Gtk::Frame frm_device_info;
			Gtk::ComboBoxText cb_device;
			Gtk::Button bt_start;
			//-------------------------------
			Gtk::Label lb_device_name;
			Gtk::Label lb_device_card;
			Gtk::Label lb_device_driver;
			Gtk::Label lb_device_bus;

			//==================================================================
			Gtk::Frame frm_device_prop;
			Gtk::SpinButton sp_width;
			Gtk::SpinButton sp_height;

			//==================================================================
			Gtk::ComboBox cb_input;
			Gtk::ComboBox cb_standard;
			Gtk::ComboBox cb_format_desc;
			Gtk::ComboBox cb_frame_size;
			Gtk::ComboBox cb_frame_interval;
			//----------------------------------
			Glib::RefPtr<Gtk::ListStore> ls_input;
			Glib::RefPtr<Gtk::ListStore> ls_standard;
			Glib::RefPtr<Gtk::ListStore> ls_format_desc;
			Glib::RefPtr<Gtk::ListStore> ls_frame_size;
			Glib::RefPtr<Gtk::ListStore> ls_frame_interval;
			//----------------------------------

			template<class T> class ModelColumn: public Gtk::TreeModel::ColumnRecord {
				public:

					ModelColumn() {
						add(m_col_name);
						add(m_col_data);
					}

					Gtk::TreeModelColumn<Glib::ustring> m_col_name;
					Gtk::TreeModelColumn<T> m_col_data;
			};
			typedef ModelColumn<struct v4l2_input> ModelInput;
			typedef ModelColumn<struct v4l2_standard> ModelStandard;
			typedef ModelColumn<struct v4l2_fmtdesc> ModelFormatDesc;
			typedef ModelColumn<struct v4l2_frmsizeenum> ModelFrameSize;
			typedef ModelColumn<struct v4l2_querymenu> ModelControlMenu;
			typedef ModelColumn<struct v4l2_frmivalenum> ModelFrameInterval;

			ModelInput model_input;
			ModelStandard model_standard;
			ModelFormatDesc model_format_desc;
			ModelFrameSize model_frame_size;
			ModelFrameInterval model_frame_interval;

			ModelControlMenu model_control_menu;

			//==================================================================
			Gtk::Notebook notebook;
			//==================================================================

			typedef struct __ctrl_holder {
					struct v4l2_queryctrl qctrl;
					Gtk::Widget * widget;
					sigc::connection con;
			} ControlHolder;
			std::list<ControlHolder> ctrl_list_default;
			std::list<ControlHolder> ctrl_list_user;
			std::list<ControlHolder> ctrl_list_private;

			void __make_controls();
			void __make_control_list_default();
			void __make_control_list_user();
			void __make_control_list_private();
			void __make_control_table(std::list<ControlHolder>& list, const char * title);

			void __update_control_widgets(std::list<ControlHolder>& list);
			void __block_control_signals(std::list<ControlHolder>& list, bool block);

			bool __set_control_hscale(int type, double val, std::list<ControlHolder> * list, Gtk::Widget * wctrl);
			void __set_control(std::list<ControlHolder> * list, Gtk::Widget * wctrl);

	};

}

#endif /* V4LINTERFACE_HPP_ */
