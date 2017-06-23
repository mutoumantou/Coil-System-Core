// (08-05) jiachen
#include "callbacks.h"

bool visionStarted = false;  //is the vision running?

static uint GUI_refresh_flag_vidWin1 = 1;        // 1: refresh the vidWin1 @ a certain rate
static uint GUI_refresh_flag_vidWin2 = 0;        // 1: refresh the vidWin2 @ a certain rate
static uint GUI_refresh_running = 0;             // 1: GUI refresh thread is running

static uint flag_draw_field = 0;

//micro_fab related variables -- Zhe
extern float field_mag_fab;
extern bool flag_temp_control;
extern float magnet_area, trust_area;
extern bool flag_magnet_sampled;
extern float factor_x, factor_y, factor_z;

////////////////////////////////////////////////  /////////////////////// ////////////////////////////////////////////
// GUI Variable
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GtkWidget *field_drawingArea;
GtkImage  *videoWindow, *videoWindow2;
GtkLabel  *label_centerPx_mm, *label_centerPy_mm;

GtkLabel  *label_info_type, *label_info_plane, *label_info_angle;   // Label near camera image, showing info. about the image
GtkLabel  *label_current_temp; // label showing current temperature
GtkLabel  *label_sampled_area, *label_current_area; // label showing magnet areas

float GUI_field_angle = 0;   // Storing the angle of current field for GUI display


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Refresh Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static gboolean update_GUI_time (gpointer userdata)
{
    time_t rawtime;
    struct tm *info;
    char buffer[80];

    time( &rawtime );

    info = localtime( &rawtime );
    //printf("Current local time and date: %s", asctime(info));

    double l_time_ms;
	struct timeval l_time_start;
	gettimeofday(&l_time_start, NULL);
    l_time_ms = (double) l_time_start.tv_usec*1e-3 ; // Current time

    //printf("ms is %d.\n", (int)l_time_ms);
    sprintf(buffer, "%sms: %d", asctime(info), (int)l_time_ms);

    gtk_label_set_text (label_info_angle, buffer);
    return G_SOURCE_REMOVE;
}

static gboolean update_vidImage1 (gpointer userdata)
{
    //printf("Inside update_vidImage1().\n");
    //if ( !flag_image_pro )
		Mat img_m_color = getImage();
	//else
	//	Mat img_m_color = get_processed_image();														// (07-19) in image_processing_subroutine.c

    gtk_image_set_from_pixbuf(videoWindow, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8,
                              img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
    //printf("Leaving update_vidImage1().\n");
    return G_SOURCE_REMOVE;
}

static gboolean update_vidImage2 (gpointer userdata)
{
    //printf("Inside update_vidImage1().\n");
    Mat img_m_color2 = getImage2();
    gtk_image_set_from_pixbuf(videoWindow2, gdk_pixbuf_new_from_data(img_m_color2.data, GDK_COLORSPACE_RGB, false, 8,
                              img_m_color2.cols, img_m_color2.rows, img_m_color2.step, NULL, NULL));
    //printf("Leaving update_vidImage1().\n");
    return G_SOURCE_REMOVE;
}

static gboolean update_magnet_areas (gpointer userdata)
{
    char temp[8];
    float d = trust_area;
    sprintf(temp, "%6.1f", d);
    gtk_label_set_text (label_sampled_area, temp);
    d = magnet_area;
    sprintf(temp, "%6.1f", d);
    gtk_label_set_text (label_current_area, temp);
    return G_SOURCE_REMOVE;
}

static gboolean update_cameraImageInfoLabel (gpointer userdata)
{
    char temp[6];
    sprintf(temp, "%.1f", GUI_field_angle);
    gtk_label_set_text (label_info_angle, temp);
    return G_SOURCE_REMOVE;
}

static gboolean update_current_temp (gpointer userdata)
{
    char temp[8];
    float d = get_current_temp();
    sprintf(temp, "%6.1f", d);
    gtk_label_set_text (label_current_temp, temp);
    return G_SOURCE_REMOVE;
}

static gboolean draw_field (gpointer userdata)
{
    //printf("Inside draw_field.\n");
    gtk_widget_queue_draw (field_drawingArea); //request a redraw
    return G_SOURCE_REMOVE;
}

void* GUI_refresh_thread(void*threadid)
{
	printf("@ the Beginning of GUI_refresh_thread().\n");
	usleep(1e6);
	Mat img_m_color,img_m_color2;
	// gdk_threads_enter and gdk_threads_leave has been deprecated since version 3.6. We use GTK2.0.

	//float refresh_rate = 30;                 //Frames to refresh per second. May not actually achieve this! Cannot be lower than 1.
	float refresh_rate = 60.0;
	float refresh_period = 1.0/refresh_rate; //seconds per refresh frame
	double time_current, time_elapsed, time_init;
	struct timeval start;
	gettimeofday(&start, NULL);
	time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time

    while(GUI_refresh_running)
    {
        if (flag_draw_field)
            g_main_context_invoke (NULL, draw_field, NULL);   // Draw field control voltage value

        if (GUI_refresh_flag_vidWin2)
        {
            img_m_color2 = getImage2();

            //gdk_threads_enter();	//display video image in program window

            if (img_m_color2.data != NULL)
            {
                g_main_context_invoke (NULL, update_vidImage2, NULL);
            }
            if (GUI_refresh_flag_vidWin1 == 1)   // If the side camera is on
            {
                img_m_color = getImage();

                if (img_m_color.data != NULL)
                {
                    g_main_context_invoke (NULL, update_vidImage1, NULL);
                }

            }
        }
        g_main_context_invoke (NULL, update_GUI_time, NULL);
        if (flag_temp_control)
            g_main_context_invoke (NULL, update_current_temp, NULL);
        if (flag_magnet_sampled)
            g_main_context_invoke (NULL, update_magnet_areas, NULL);

        //g_main_context_invoke (NULL, update_GUI_time_ms, NULL);
        gettimeofday(&start, NULL);
        time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Current time
        time_elapsed = time_current - time_init;                    // Time elapsed since last refresh
        if(time_elapsed < refresh_period)
        {
		//printf("Wait time is %.5f.\n", (refresh_period - time_elapsed) * 1e6);
            usleep( (refresh_period - time_elapsed) * 1e6 );        // Wait until refresh period has been reached
        }
        gettimeofday(&start, NULL);
        time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Current time
        time_init = time_current  ; //reset last time

        // If no GUI_refresh job is needed, stop this thread
        if ( (!flag_draw_field) && (!GUI_refresh_flag_vidWin2) && (!flag_temp_control))
            GUI_refresh_running = 0;
	}
	//fclose(fp);   // Close file
	printf("@ the End of GUI_refresh_thread().\n");

}

/* Toggle Button: Draw field control voltage value */
void on_tButton_draw_field_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if (d==1)//if button is toggled up
	{
        flag_draw_field = 1;
		//GUI_refresh_running = 1; //turn on control loops
		if (!GUI_refresh_running)  // If the GUI_refresh thread is NOT running...
		{
            GUI_refresh_running = 1;
            pthread_t   GUI_refresh;
            pthread_create(&GUI_refresh, NULL, GUI_refresh_thread, NULL);  //start control loop thread
		}
	}else
	{
		flag_draw_field = 0; //turn off control loops
	}
}

// Draw field control voltage value
void on_field_drawingArea_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data)
{
    cairo_t *cr;
    cr = gdk_cairo_create(widget->window);

	char str[10];//string for printing text. reused.
   	/* Set color for background */
   	cairo_set_source_rgb(cr, 1, 1, 1);
   	/* fill in the background color*/
   	cairo_paint(cr);

   	/* x field */
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 30, 150);
   	cairo_line_to(cr, 0, 150);
   	cairo_stroke(cr);

   	cairo_set_source_rgb(cr, 255,0,0); //set color to Red
   	cairo_set_line_width(cr, 30);
   	float coil_current_x = get_coil_current(0);

   	//printf("current x is %.1f.\n",coil_current_x);

   	cairo_move_to(cr, 15, 150);
   	cairo_line_to(cr, 15, 150 - coil_current_x * 30);
   	cairo_stroke(cr);

   	/* y field */
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 80, 150);
   	cairo_line_to(cr, 50, 150);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0,255,0); //set color to Green
   	cairo_set_line_width(cr, 30);
   	float coil_current_y = get_coil_current(1);

   	cairo_move_to(cr, 65, 150);
   	cairo_line_to(cr, 65, 150 - coil_current_y * 30);
   	cairo_stroke(cr);

   	/* z field */
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	cairo_move_to(cr, 130, 150);
   	cairo_line_to(cr, 100, 150);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0,0,255); //set color to Blue
    cairo_set_line_width(cr, 30);
   	float coil_current_z = get_coil_current(2);

   	cairo_move_to(cr, 115, 150);
   	cairo_line_to(cr, 115, 150 - coil_current_z * 30);
   	cairo_stroke(cr);

   	cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_move_to (cr, 0, 275);
	sprintf(str, "%.1f", coil_current_x * factor_x);
	cairo_show_text (cr, str);
	cairo_move_to (cr, 50, 275);
	sprintf(str, "%.1f", coil_current_y * factor_y);
	cairo_show_text (cr, str);
    cairo_move_to (cr, 100, 275);
	sprintf(str, "%.1f", coil_current_z * factor_z);
    cairo_show_text (cr, str);

    /// Orientation:
    cairo_set_source_rgb(cr, 0,0,0); //set color to black
   	cairo_set_line_width(cr, 1);
   	cairo_move_to(cr, 30, 350);
   	cairo_line_to(cr, 110, 350);
   	cairo_move_to(cr, 70, 310);
   	cairo_line_to(cr, 70, 390);

   	cairo_move_to(cr,  30, 470);
   	cairo_line_to(cr, 110, 470);
   	cairo_move_to(cr,  70, 430);
   	cairo_line_to(cr,  70, 510);
   	cairo_stroke(cr);

    /// y-z circle
    cairo_move_to(cr,  30, 590);
   	cairo_line_to(cr, 110, 590);
   	cairo_move_to(cr,  70, 550);
   	cairo_line_to(cr,  70, 630);
   	cairo_stroke(cr);

    cairo_set_source_rgb(cr, 0.17, 0.63, 0.12);
   	cairo_set_line_width(cr,1);
   	cairo_arc(cr, 70, 350, 40, 0, 2*G_PI);
   	cairo_move_to(cr, 110, 470);
   	cairo_arc(cr, 70, 470, 40, 0, 2*G_PI);
   	cairo_stroke(cr);
   	/// y-z circle
   	cairo_arc(cr, 70, 590, 40, 0, 2*G_PI);
   	cairo_stroke(cr);
    cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 200, 0, 0);
    cairo_move_to (cr, 120, 350);
	sprintf(str, "x");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 70, 300);
	sprintf(str, "y");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 120, 470);
	sprintf(str, "x");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 70, 420);
	sprintf(str, "z");
	cairo_show_text (cr, str);
	/// y-z circle
	cairo_move_to (cr, 120, 590);
	sprintf(str, "y");
	cairo_show_text (cr, str);
	cairo_move_to (cr, 70, 540);
	sprintf(str, "z");
	cairo_show_text (cr, str);

    float orientation_xy = atan2(coil_current_y, coil_current_x);
    float orientation_xz = atan2(coil_current_z, coil_current_x);
    float orientation_yz = atan2(coil_current_z, coil_current_y);

    cairo_set_font_size (cr, 14);
    cairo_set_source_rgb (cr, 0, 0, 200);
    if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_y > 0.01) || (coil_current_y < -0.01) )
    {
        cairo_move_to (cr, 100, 390);
        sprintf(str, "%.1f", orientation_xy * 180 / M_PI);
        cairo_show_text (cr, str);
    }
    if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to (cr, 100, 510);
        sprintf(str, "%.1f", orientation_xz * 180 / M_PI);
        cairo_show_text (cr, str);
    }
    /// y-z circle orientation value
    if ( (coil_current_y > 0.01) || (coil_current_y < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to (cr, 100, 630);
        sprintf(str, "%.1f", orientation_yz * 180 / M_PI);
        cairo_show_text (cr, str);
    }

	cairo_set_source_rgb(cr, 255,0,0); //set color to black
   	cairo_set_line_width(cr, 3);
   	if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_y > 0.01) || (coil_current_y < -0.01) )
    {
        cairo_move_to(cr, 70, 350);        // Center of x-y circle
        cairo_line_to(cr, 70+40 * cos(orientation_xy), 350-40 * sin(orientation_xy));
    }
    if ( (coil_current_x > 0.01) || (coil_current_x < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to(cr, 70, 470);        // Center of x-y circle
        cairo_line_to(cr, 70+40 * cos(orientation_xz), 470-40 * sin(orientation_xz));
    }
    /// y-z circle orientation line
    if ( (coil_current_y > 0.01) || (coil_current_y < -0.01) || (coil_current_z > 0.01) || (coil_current_z < -0.01) )
    {
        cairo_move_to(cr, 70, 590);        // Center of y-z circle
        cairo_line_to(cr, 70+40 * cos(orientation_yz), 590-40 * sin(orientation_yz));
    }

    cairo_stroke(cr);

	cairo_destroy(cr);
}

void on_window_destroy (GtkWidget *widget, gpointer data)
{
	stopVision(); //turn of visionloop
    GUI_refresh_running = 0;

    coilCurrentClear();                // Reset coil current to 0
    usleep(5e4);

    stop_auto_feeding();
    stop_temp_control();
    s826_close();                      // Close s826 board

	usleep(2e5); //wait for all loops to detect shutdown and end themselves

	printf("Exiting program.\n");
    gtk_main_quit();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Camera Enable: Video toggle button
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_videoButton_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		if(visionStarted) return;
		g_print("Starting vision...\n");
		visionStarted = true;
		initVision(); //in vision.c

        GUI_refresh_flag_vidWin2 = 1;   // Begin to refresh vidWin
        if (!GUI_refresh_running)       // If the GUI_refresh_thread is not running ...
        {
            pthread_t GUI_refresh_thread_instance;
            GUI_refresh_running = 1;
            pthread_create(&GUI_refresh_thread_instance, NULL, GUI_refresh_thread, NULL);  //start control loop thread
        }
	}
	else
	{
		g_print("Stopping vision...\n");
		stopVision(); //sets killThread to 1
		visionStarted = false;
        //GUI_refresh_running = 0;
		GUI_refresh_flag_vidWin2 = 0;   // Stop refreshing GUI
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Twisting Field Walking Callbacks -- Omid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// twisting field related functions -- Omid
void on_twisting_walking_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_twist_field(d);
  }else
  {
    stop_twist_field(d);
  }
}

void on_twisted_walking_enabler_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_twisted_walking(d);
  }else
  {
    stop_twisted_walking(d);
  }
}


void on_manual_field_toggled (GtkToggleButton *togglebutton, gpointer data)
{
        int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_manual_field(d);
  }else
  {
    stop_manual_field(d);
  }
}
void on_manual_field_enabler_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

  if(d==1) //if button is toggled up
  {
    init_manual_field_go(d);
  }else
  {
    stop_manual_field_go(d);
  }
}




void on_theta_heading_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_theta_heading(d);
}

void on_beta_tilt_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_beta_tild(d);
}

void on_ang_freq_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_ang_freq(d);
}

void on_ang_span_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_ang_span(d);
}

void on_tfield_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_tfield_mag(d);
}

void on_bx_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_bx_mag(d);
}

void on_by_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_by_mag(d);
}

void on_bz_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_bz_mag(d);
}

void on_dbx_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_dbx_mag(d);
}

void on_dby_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_dby_mag(d);
}

void on_dbz_mag_changed (GtkEditable *editable, gpointer user_data)
{
    float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_dbz_mag(d);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnet Detection Callbacks -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// magnet vision detection functions
void on_magnet_detection_toggled (GtkToggleButton *togglebutton, gpointer data) //magnet_detection toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_magnetdetection(d); //set magnetdetection variable in vision.c
}

void on_show_box_toggled (GtkToggleButton *togglebutton, gpointer data) //show_box toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_showbox(d); //set showbox variable in vision.c
}

void on_draw_annotation_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_draw_annotation(d);
}

void on_show_process_toggled (GtkToggleButton *togglebutton, gpointer data) //show_process toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_showprocess(d); //set showprocess variable in vision.c
}

void on_draw_roi_toggled (GtkToggleButton *togglebutton, gpointer data) //draw_roi toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_draw_roi(d); //set draw_roi variable in vision.c
}

void on_draw_points_toggled (GtkToggleButton *togglebutton, gpointer data) //draw_points toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_draw_points(d); //set draw_points variable in vision.c
}

void on_close_diameter_changed (GtkEditable *editable, gpointer user_data)
{
	int d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("close diameter changed to %.1f units\n", d );
	set_closediameter(d);
}

void on_needle_thick_changed (GtkEditable *editable, gpointer user_data)
{
	int d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("needle thickness changed to %.1f units\n", d );
	set_needle_thick(d);
}

void on_magnet_threshol_changed (GtkEditable *editable, gpointer user_data)
{
	double d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("threshold changed to %.1f units\n", d );
	set_magnet_threshold(d);
}

void on_reverse_magnet_clicked (GtkWidget *widget, gpointer data)
{
    reverse_magent(); // @vision.c, reverse magnet direction
}

void on_sample_magnet_clicked (GtkWidget *widget, gpointer data)
{
    set_magnet_trust();
}

// discrete bending thread functions -- Zhe
void on_discrete_bending_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_discrete_bending();   // coilFieldControl.c
	}else
	{
		stop_discrete_bending();
	}
}

void on_kp_changed (GtkEditable *editable, gpointer user_data) //change Kp in controller
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("kp changed to %.1f units\n", d );
	set_kp(d);
}

void on_ki_changed (GtkEditable *editable, gpointer user_data) //change Ki in controller
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("ki changed to %.1f units\n", d );
	set_ki(d);
}

void on_destination_angle_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("destination angle set to %.1f units\n", d );
	set_destination_angle(d);
}

void on_show_destination_toggled (GtkToggleButton *togglebutton, gpointer data) //show_destination toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("show_destination toggled to %i\n", d );
	set_showdestination(d); //set showprocess variable in vision.c
}

void on_show_field_direction_toggled (GtkToggleButton *togglebutton, gpointer data) //show_field_direction toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("show_field_direction toggled to %i\n", d );
	set_showfielddirection(d); //set showfielddirection variable in vision.c
}

void on_discrete_go_toggled (GtkToggleButton *togglebutton, gpointer data) //discrete_go toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("discrete_go toggled to %i\n", d );
	set_discrete_go(d); //set discrete_go variable in coilFieldControl.c
}

// continuous bending thread functions -- Zhe
void on_continuous_bending_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_continuous_bending();   // coilFieldControl.c
	}else
	{
		stop_continuous_bending();
	}
}

void on_continuous_radius_changed (GtkEditable *editable, gpointer user_data) // change radius of continuous bending
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("radius of continuous bending set to %.1f units\n", d );
	set_continuous_radius(d);
}

void on_continuous_central_changed (GtkEditable *editable, gpointer user_data) // change central angle of continuous bending
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("central angle of continuous bending set to %.1f units\n", d );
	set_continuous_central(d);
}

void on_continuous_start_changed (GtkEditable *editable, gpointer user_data) // change starting of continuous bending
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("starting angle of continuous bending set to %.1f units\n", d );
	set_continuous_start(d);
}

void on_continuous_time_changed (GtkEditable *editable, gpointer user_data) // change time of continuous bending
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("time of continuous bending set to %.1f units\n", d );
	set_continuous_time(d);
}

void on_continuous_feedback_toggled (GtkToggleButton *togglebutton, gpointer data) //continuous_feedback check button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_continuous_feedback(d); //set continuous_feedback variable in coilFieldControl.c
}

void on_continuous_go_toggled (GtkToggleButton *togglebutton, gpointer data) //continuous_go toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("continuous_go toggled to %i\n", d );
	set_continuous_go(d); //set continuous_go variable in coilFieldControl.c
}

// temperature control thread callbacks
void on_temp_control_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform temperature control
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_temp_control();   // TemperatureFeeding.c
        if (!GUI_refresh_running)  // If the GUI_refresh thread is NOT running...
		{
            GUI_refresh_running = 1;
            pthread_t   GUI_refresh;
            pthread_create(&GUI_refresh, NULL, GUI_refresh_thread, NULL);  //start control loop thread
		}
	}
	else
	{
		stop_temp_control();
	}
}

void on_destination_temp_changed (GtkEditable *editable, gpointer user_data) // change destination temperature
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("destination temperature set to %.1f units\n", d );
	set_destination_temp(d);
}

void on_temp_go_toggled (GtkToggleButton *togglebutton, gpointer data) // temperature go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("discrete_go toggled to %i\n", d );
	set_temp_go(d);
}

// sinusoidal field (2d) thread functions
void on_sinusoidal_field_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to generate a sinusoidal field
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_sinusoidal_field();   // AutoFabrication.c
	}
	else
	{
		stop_sinusoidal_field();
	}
}

void on_fab_heading_s_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("amplitude set to %.1f units\n", d );
	set_fab_heading_s(d);
}

void on_fab_amp_s_changed (GtkEditable *editable, gpointer user_data) // change micro-fabrication amplitude
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("amplitude set to %.1f units\n", d );
	set_fab_amp_s(d);
}

void on_fab_fre_s_changed (GtkEditable *editable, gpointer user_data) // change micro-fabrication frequency
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("frequency set to %.1f units\n", d );
	set_fab_fre_s(d);
}

void on_sinusoidalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data) // sinusoidal field go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("rotational field_go toggled to %i\n", d );
	set_sinusoidalfield_go(d);
}

// rotational field (vertical) thread functions
void on_rotational_field_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to generate a rotational field
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_rotational_field();   // coilFieldControl.c
	}
	else
	{
		stop_rotational_field();
	}
}

void on_fab_heading_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("amplitude set to %.1f units\n", d );
	set_fab_heading(d);
}

void on_fab_amp_changed (GtkEditable *editable, gpointer user_data) // change micro-fabrication amplitude
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("amplitude set to %.1f units\n", d );
	set_fab_amp(d);
}

void on_fab_fre_changed (GtkEditable *editable, gpointer user_data) // change micro-fabrication frequency
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("frequency set to %.1f units\n", d );
	set_fab_fre(d);
}

void on_rotationalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data) // rotational field go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("rotational field_go toggled to %i\n", d );
	set_rotationalfield_go(d); //set rotational field_go variable in AutoFabrication.c
}

// rotational field (horizontal) thread functions
void on_rotational_field_h_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to generate a rotational field
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_rotational_field_h();   // AutoFabrication.c
	}
	else
	{
		stop_rotational_field_h();
	}
}

void on_fab_heading_h_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_fab_heading_h(d);
}

void on_rotationalfield_go_h_toggled (GtkToggleButton *togglebutton, gpointer data) // rotational field go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("rotational field_go toggled to %i\n", d );
	set_rotationalfield_go_h(d); //set rotational field_go variable in coilFieldControl.c
}

// sawtooth mode thread functions
void on_sawtooth_mode_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform sawtooth mode
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d==1) //if button is toggled up
	{
		init_sawtooth_mode();   // AutoFabrication.c
	}
	else
	{
		stop_sawtooth_mode();
	}
}

void on_sawtooth_go_toggled (GtkToggleButton *togglebutton, gpointer data) // sawtooth field go/stop toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("sawtooth_go toggled to %i\n", d );
	set_sawtooth_go(d); //set sawtooth_go variable in AutoFabrication.c
}

void on_peak_angle_st_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("peak_angle_st set to %.1f units\n", d );
	set_peak_angle_st(d);
}

// automatic feeding thread callbacks
void on_auto_feeding_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform automatic feeding
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d) //if button is toggled up
	{
		init_auto_feeding();   // TemperatureFeeding.c
	}
	else
	{
		stop_auto_feeding();
	}
}

void on_feeding_distance_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("feeding distance set to %.1f units\n", d );
	set_feeding_distance(d);
}

void on_feeding_speed_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("feeding speed set to %.1f units\n", d );
	set_feeding_speed(d);
}

void on_feeding_go_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("feeding go toggle button set to %.1f units\n", d );
	set_feeding_go(d);
}

void on_feeding_increments_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("feeding increments is set to %.1f units\n", d );
	set_feeding_increments(d);
}

void on_manual_override_toggled (GtkToggleButton *togglebutton, gpointer data) // whether to perform manual
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d) //if button is toggled up
	{
		init_manual_feeding();   // TemperatureFeeding.c
		printf("manual override set.\n");
	}
	else
	{
		stop_manual_feeding();
		printf("manual override stopped.\n");
	}
}

void on_feeder_extend_button_clicked (GtkWidget *widget, gpointer data)
{
    feederIncrement();
    //printf("increment clicked\n");
}

void on_feeder_retract_button_clicked (GtkWidget *widget, gpointer data)
{
    feederDecrement();
    //printf("decrement clicked\n");
}

//automatic fabrication callbacks
void on_left_constraint_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);
    set_left_constraint(d);
}

void on_shape_fab_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d)
	{
		init_auto_fab(); // AutoFabrication.c
	}
	else
	{
		stop_auto_fab(); // AutoFabrication.c
	}
}

// device cutting-off callbacks
void on_neutral_cut_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_neutral_cut(d);
}

void on_mag_cut_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_mag_cut(d);
}

void on_fre_cut_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_fre_cut(d);
}

void on_cut_go_toggled (GtkToggleButton *togglebutton, gpointer data)
{
    int d = gtk_toggle_button_get_active(togglebutton);

	if(d)
	{
		init_cut_off(); // AutoFabrication.c
	}
	else
	{
		stop_cut_off(); // AutoFabrication.c
	}
}

// field control callbacks
void on_cb_coil_selection_changed(GtkComboBox *combo_box, gpointer user_data)
{
    int d = gtk_combo_box_get_active (combo_box);
    set_factor(d); // d=1, 3D coil system; d=0 2D coil system
}

void on_field1_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_xyz (0, d);
}

void on_field2_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_xyz (1, d);
}

void on_field3_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_xyz (2, d);
}

void on_field_mag_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	set_field_mag_fab (d);
}

void on_xy_3d_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_field_xyz ( 0, field_mag_fab*cosd(d) );
    set_field_xyz ( 1, field_mag_fab*sind(d) );
}

void on_xz_3d_fab_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
    set_field_xyz ( 0, field_mag_fab*cosd(d) );
    set_field_xyz ( 2, field_mag_fab*sind(d) );
}

void on_reset_field_button_clicked (GtkWidget *widget, gpointer data)
{
   	set_field_xyz (0, 0);
   	set_field_xyz (1, 0);
   	set_field_xyz (2, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Y Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_edgemap_toggled (GtkToggleButton *togglebutton, gpointer data) //edgemap toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_edgemap(d); //set edgemap variable in vision.c
}
void on_binary_toggled (GtkToggleButton *togglebutton, gpointer data) //edgemap toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_binary(d); //set edgemap variable in vision.c
}
void on_cannyLow_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyLow changed to %.1f units\n", d );
	setcannyHigh_vision((int)d);
	//cannyLow = d; //change frequency
	//img_test(); //test opencv. remove this later?
}
void on_cannyHigh_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyHigh changed to %.1f units\n", d );
	setcannyLow_vision((int)d);
	//cannyHigh = d; //change frequency
	//img_test(); //test opencv. remove this later?
}
void on_gain_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("gain changed to %.1f units\n", d );
	setGain_vision(d);
}
void on_shutter_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("shutter changed to %.1f units\n", d );
	setShutter_vision(d);
}
void on_dilate_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("dilate changed to %.1f units\n", d );
	setDilate_vision(d);
}
void on_visionParam1_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam1_vision(d);
}
void on_visionParam2_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam2_vision(d);
}
void on_detect_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	setdetect_vision(d);
}

void on_topcam_on_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	setTopCam_vision(d);

	if(d==1) //if button is toggled up
	{
		GUI_refresh_flag_vidWin1 = 1;
	} else
	{
		GUI_refresh_flag_vidWin1 = 0;
	}

	if(visionStarted) //if the vision is already started, restart it with new topcam choice
	{
		usleep(1e5);
		stopVision();
		visionStarted = FALSE;
		usleep(20e5);
		g_print("Restarting vision thread.\n");
		initVision();
		visionStarted = TRUE;
		usleep(20e5);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// edgemap toggle button
void on_edgemap_xz_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	//printf("edgemap toggled to %i\n", d );
	set_edgemap_xz(d); //set edgemap variable in vision.c
}
void on_binary_xz_toggled (GtkToggleButton *togglebutton, gpointer data) //edgemap toggle button
{
	int d = gtk_toggle_button_get_active(togglebutton);
	set_binary_xz(d); //set edgemap variable in vision.c
}
void on_cannyLow_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyLow changed to %.1f units\n", d );
	setcannyHigh_xz_vision((int)d);
}
void on_cannyHigh_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("cannyHigh changed to %.1f units\n", d );
	setcannyLow_xz_vision((int)d);
}
void on_gain_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("gain changed to %.1f units\n", d );
	setGain_xz_vision(d);
}
void on_shutter_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("shutter changed to %.1f units\n", d );
	setShutter_xz_vision(d);
}
void on_dilate_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	//printf("dilate changed to %.1f units\n", d );
	setDilate_xz_vision(d);
}
void on_visionParam1_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam1_xz_vision(d);
}
void on_visionParam2_xz_changed (GtkEditable *editable, gpointer user_data)
{
	float d = gtk_spin_button_get_value(GTK_SPIN_BUTTON(editable));
	setvisionParam2_xz_vision(d);
}
void on_detect_xz_toggled (GtkToggleButton *togglebutton, gpointer data)
{
	int d = gtk_toggle_button_get_active(togglebutton);
	setdetect_xz_vision(d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Catch Mouse Click Event
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gboolean on_videoWindow_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data)
{
	int click[2];
	click[0] = (int)event->x; //x position from top left in pixels
	click[1] = (int)event->y; //y position from top left in pixels
	int button_click = event->button; //which mouse button was clicked
	//g_print("Top video window %d click at location [%d %d].\n", button_click, click[0], click[1]);
	setMouse(0, button_click, click );      //void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
}

gboolean on_videoWindow2_button_press_event(GtkWidget *widget, GdkEventButton *event, gpointer data)
{
	int click[2];
	click[0] = (int)event->x; //x position from top left in pixels
	click[1] = (int)event->y; //y position from top left in pixels
	int button_click = event->button; //which mouse button was clicked
	//g_print("Side video window %d click at location [%d %d].\n", button_click, click[0], click[1]);
	setMouse(1, button_click, click );      //void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
}
