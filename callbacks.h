// (08-05)
#ifndef CALLB
#define CALLB

#include <fcntl.h>      // for open()
#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>
#include <gdk/gdkkeysyms.h>

//#include <s826_subroutine.h>
//#include <s826api.h>

//#include <signal.h>
//#include <GL/glut.h>
//#include <SDL/SDL.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <string>

//#include "magnetLowLevel.h"
//#include "LXRTclass.h"
//#include "coilDisplay.h"
//#include "magnetControl.h"
//#include "NetController.h"
//#include "callbacksTS.h"
#include "vision.h"
//#include "FWcamera.h"
#include "s826_subroutine.h"

#include "coilFieldControl.h"
#include "AutoFabrication.h"
#include "twistField.h"

//#define PI 3.14159265

//extern Mat img_m_color;

//using namespace std;

void* controlThread(void*);
void* drawThread(void*);

extern "C" {  //use to fix C++ name mangling problem, when compiling with g++ instead of gcc. see http://cboard.cprogramming.com/cplusplus-programming/146982-gtkplus-cplusplus-compiler-signal-handlers.html

    void on_field_drawingArea_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data);
	void on_window_destroy (GtkWidget *widget, gpointer data);
	void on_tButton_draw_field_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_current_clicked(GtkWidget *widget, gpointer data);
	void on_videoButton_toggled (GtkToggleButton *togglebutton, gpointer data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnet Detection -- Edited by Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void on_magnet_detection_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_show_box_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_draw_annotation_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_show_process_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_draw_roi_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_draw_points_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_close_diameter_changed (GtkEditable *editable, gpointer user_data);
    void on_needle_thick_changed (GtkEditable *editable, gpointer user_data);
    void on_magnet_threshol_changed (GtkEditable *editable, gpointer user_data);
    void on_reverse_magnet_clicked (GtkWidget *widget, gpointer data);
    void on_sample_magnet_clicked (GtkWidget *widget, gpointer data);
    //discrete bending
    void on_discrete_bending_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_kp_changed (GtkEditable *editable, gpointer user_data);
    void on_ki_changed (GtkEditable *editable, gpointer user_data);
    void on_destination_angle_changed (GtkEditable *editable, gpointer user_data);
    void on_show_destination_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_show_field_direction_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_discrete_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //continuous bending
    void on_continuous_bending_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_continuous_radius_changed (GtkEditable *editable, gpointer user_data);
    void on_continuous_central_changed (GtkEditable *editable, gpointer user_data);
    void on_continuous_start_changed (GtkEditable *editable, gpointer user_data);
    void on_continuous_time_changed (GtkEditable *editable, gpointer user_data);
    void on_continuous_feedback_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_continuous_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //temperature control
    void on_temp_control_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_destination_temp_changed (GtkEditable *editable, gpointer user_data);
    void on_temp_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    // sinusoidal field (2d) thread functions
    void on_sinusoidal_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_heading_s_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_amp_s_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_fre_s_changed (GtkEditable *editable, gpointer user_data);
    void on_sinusoidalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //rotational field (vertical)
    void on_rotational_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_heading_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_amp_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_fre_changed (GtkEditable *editable, gpointer user_data);
    void on_rotationalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    // rotational field (horizontal) thread functions
    void on_rotational_field_h_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_heading_h_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_rotationalfield_go_h_toggled (GtkToggleButton *togglebutton, gpointer data);
    // sawtooth mode thread functions
    void on_sawtooth_mode_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_sawtooth_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_peak_angle_st_changed (GtkEditable *editable, gpointer user_data);
    //automatic feeding
    void on_auto_feeding_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_feeding_distance_changed (GtkEditable *editable, gpointer user_data);
    void on_feeding_speed_changed (GtkEditable *editable, gpointer user_data);
    void on_feeding_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //manual feeding
    void on_feeding_increments_changed (GtkEditable *editable, gpointer user_data);
    void on_manual_override_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_feeder_extend_button_clicked (GtkWidget *widget, gpointer data);
    void on_feeder_retract_button_clicked (GtkWidget *widget, gpointer data);
    //automatic fabrication
    void on_left_constraint_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_shape_fab_toggled (GtkToggleButton *togglebutton, gpointer data);
    // device cutting-off callbacks
    void on_neutral_cut_changed (GtkEditable *editable, gpointer user_data);
    void on_mag_cut_changed (GtkEditable *editable, gpointer user_data);
    void on_fre_cut_changed (GtkEditable *editable, gpointer user_data);
    void on_cut_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    // field control callbacks
    void on_cb_coil_selection_changed(GtkComboBox *combo_box, gpointer user_data);
    void on_field1_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field2_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field3_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field_mag_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_xy_3d_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_xz_3d_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_reset_field_button_clicked (GtkWidget *widget, gpointer data);

    // twisting field callbacks
    void on_twisting_walking_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_twisted_walking_enabler_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_theta_heading_changed (GtkEditable *editable, gpointer user_data);
    void on_beta_tilt_changed (GtkEditable *editable, gpointer user_data);
    void on_ang_freq_changed (GtkEditable *editable, gpointer user_data);
    void on_ang_span_changed (GtkEditable *editable, gpointer user_data);
    void on_tfield_mag_changed (GtkEditable *editable, gpointer user_data);
    // manual field callbacks
    void on_manual_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_manual_field_enabler_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_bx_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_by_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_bz_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_dbx_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_dby_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_dbz_mag_changed (GtkEditable *editable, gpointer user_data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Y Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void on_edgemap_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_binary_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_cannyLow_changed (GtkEditable *editable, gpointer user_data);
	void on_cannyHigh_changed (GtkEditable *editable, gpointer user_data);
	void on_gain_changed (GtkEditable *editable, gpointer user_data);
	void on_shutter_changed (GtkEditable *editable, gpointer user_data);
	void on_dilate_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam1_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam2_changed (GtkEditable *editable, gpointer user_data);
	void on_detect_toggled (GtkToggleButton *togglebutton, gpointer data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void on_edgemap_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_binary_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_cannyLow_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_cannyHigh_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_gain_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_shutter_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_dilate_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam1_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam2_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_detect_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_topcam_on_toggled (GtkToggleButton *togglebutton, gpointer data);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Catch Mouse Click Event
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	gboolean on_videoWindow_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data);
	gboolean on_videoWindow2_button_press_event(GtkWidget *widget, GdkEventButton *event, gpointer data);
}

#endif


