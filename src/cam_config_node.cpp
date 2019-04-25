#include <gtkmm.h>
#include <ImageViewer.hpp>
#include <V4LInterface.hpp>
#include <ros/ros.h>
#include <matrix.hpp>

class App: public Gtk::Window {

	public:
		Gtk::Window w1;
		gui::ImageViewer ga;

		mcore::matrix<unsigned char> m;

		gui::V4LInterface v4li;
		sigc::connection con;

		bool capture_and_show() {

			if (m.data == NULL) return false;

			// Grab rgb image. Returns if frame is not ready.
			if (!v4li.vdev.grabRGB(m.data)) return true;

			ga.show_data(m);
			return true;
		}

		bool start_signal(bool b) {
            int width, height;
			if (b) {
				std::cout << "Start Button Clicked!" << std::endl;

				if (m.data) {
					ga.unlink_data();
				}

                v4li.vdev.get_image_dimension( &width, &height );
				// width = v4li.vcap.format_dest.fmt.pix.width;
				// height = v4li.vcap.format_dest.fmt.pix.height;

				// Init buffers
				m.set_size(height, width * 3, 1);

				ga.set_size_request(width, height);

				con = Glib::signal_idle().connect(sigc::mem_fun(*this, &App::capture_and_show));

			} else {
				std::cout << "Stop Button Clicked!" << std::endl;
				//ga.unlink_data();
				con.disconnect();
			}
			w1.set_resizable(false);
			w1.show();

			return true;
		}

		App() {
			Gtk::VBox * vbox = new Gtk::VBox();

			this->add(*vbox);
			vbox->pack_start(v4li, true, true, 10);
			w1.add(ga);
			w1.show_all();

			v4li.signal_start().connect(sigc::mem_fun(*this, &App::start_signal));

		}

		~App() {
			con.disconnect();
			ga.unlink_data();
		}
};


int main( int argc, char ** argv ) {

     //Initialize new ROS node named "cam_node"
    ros::init(argc, argv, "config_node");
    std::cout << "ENtrou" << std::endl;
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
	App w;
	w.show_all();
	app->run(w);
    ros::spin();
    return 0;
}