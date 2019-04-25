/*
 * ImageViewer.hpp
 *
 *  Created on: 18 Oct 2017
 *      Author: Gustavo Teodoro Laureano, gustavo
 */

#include <gtkmm.h>
#include <iostream>

#ifndef GUI_IMAGEVIEWER_HPP_
#define GUI_IMAGEVIEWER_HPP_

#include <gdkmm.h>

#include "matrix.hpp"

namespace gui {

	class ImageViewer: public Gtk::DrawingArea {
		private:
			mcore::matrix<unsigned char> * gm;
			Glib::RefPtr<Gdk::Pixbuf> pb;
			int rows, cols;

		public:
			ImageViewer() :
					gm(NULL),  pb(NULL), rows(0), cols(0) {
			}

			void refresh(void) {
				this->queue_draw();
			}

			void unlink_data(void) {
				gm = NULL;
			}

			void show_data(mcore::matrix<unsigned char> & m) {
				if (!(gm == &m)) {
					gm = &m;
					set_size_request(m.ncols/3, m.nrows);
					pb = Gdk::Pixbuf::create_from_data(gm->data, Gdk::COLORSPACE_RGB, false, 8, gm->ncols/3, gm->nrows, gm->stride);
				}
				this->queue_draw();
			}

			virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) {

				if( gm == NULL ) return false;
				Gdk::Cairo::set_source_pixbuf(cr, pb, 0, 0);
				cr->paint();

				return true;
			}

	};

}  // namespace gui

#endif /* GUI_IMAGEVIEWER_HPP_ */
