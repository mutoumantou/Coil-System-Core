// (08-15)
#include "vision.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int flag_newFrame = 0;													// (07-15) indicate whether or not a new frame has arrived

FWcamera cam, cam_xz; //see FWcamera.cpp

int width = 640;   //image width, pixels
int height = 480;  //image height, pixels
int depth = 1;     //depth of image
//unsigned char * image = NULL;
int killVisionThread = 1; //this stops our vision thread

GtkLabel *labelFPSreceive, *labelFPSreceive_xz;

static Point mouse, mouseC, mouseR;
static Point mouse_xz, mouseC_xz, mouseR_xz;

//ImageProcessor RobotTracker(width, height, 1);

int cannyLow=100, cannyHigh=150; //thresholds for image processing filter
static int dilater = 1;
static int edgemap = 0, binary = 0; //are we performing edgemap calculations?
int visionParam1 = 65; //for processing. Used in threshold() and houghCircle().
int visionParam2 = 35; //for processing
static int detect = 1; //are we detecting object?
static float fpsReceive; //frames per second of video

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int cannyLow_xz=100, cannyHigh_xz=150; //thresholds for image processing filter
static int dilater_xz = 1;
static int edgemap_xz = 0, binary_xz = 0; //are we performing edgemap calculations?
int visionParam1_xz = 65; //for processing
int visionParam2_xz = 35; //for processing
static int detect_xz = 1; //are we detecting object?
static int topcam_on = 1; //is the sidecam capturing? Default: YES
static float fpsReceive_xz; //frames per second of video

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// magnet detection variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool magnetdetection = false;
bool showbox = true;
bool draw_annotation = true;
bool showprocess = false;
bool draw_roi = true;
bool draw_points = true;
int closediameter = 6;
int needle_thick = 30;
double magnet_threshold = 48;
float needle_x, needle_y;
float m_x, m_y, m_a = 0.0, m_x_history[6] = {0,0,0,0,0,0}, m_y_history[6] = {0,0,0,0,0,0}, m_a_history[6] = {0,0,0,0,0,0}; // historical value of magnet centre (m_x, m_y) and angle m_a
float m_x_temp, m_y_temp, m_a_temp;
float magnet_area = 0;
float trust_area = 0;
bool flag_magnet_sampled = false;
float Mwidth, Mlength;
bool drawMagetization = 0, showdestination = 1, showfielddirection = 1;
extern float fangle, destination_angle, v1, v2;
extern float current_temp;
extern char fab_status[];
extern char fab_time[];
extern float field_x, field_y, field_z, field_mag, field_angle;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This thread runs the image processing functions:
// initialize camera, receive and display frames, process images

Mat img_m_color_for_display, img_m_color_for_display2;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mat getImage(void)
{
	return img_m_color_for_display;
}

Mat getImage2(void)
{
	return img_m_color_for_display2;
}

void initVision(void)
{
	pthread_t vthread, fthread, vthread_xz, fthread_xz;

	if(!cam_xz.initialize_xz()) //cam is instance of FWCamera, found in FWcamera.cpp
	{
		g_print("FW camera xz could not be found in initVision!!!\n");
		return;
	}
	usleep(1e5);
	if(!cam_xz.startGrabbingVideo_xz())
	{
		g_print("FW cam xz could not grab in initVision!!!\n");
		return;
	}

	if(killVisionThread == 0)
		g_print("Vision already running in initVision!!!\n");
	usleep(1e5);

	if(killVisionThread == 1)
  	{
  		killVisionThread = 0;
		//pthread_create(&fthread, NULL, FPSprint    , NULL);  //start frame print thread. Functionality??? comment this if you do not want to show frame per second
		pthread_create(&vthread_xz, NULL, visionThread_xz, NULL);  //start vision thread
	}

	// X-Z Camera
	if(topcam_on == 1) //if we are also using the top cam
	{
		usleep(2e5);
		printf("Before cam_xy.initialize_xz().\n");
		if(!cam.initialize()) //cam is instance of FWCamera, found in FWcamera.cpp
		{
			g_print("FW camera xy could not be found in initVision!!!\n");
			return;
		}
		usleep(1e5);

		if(!cam.startGrabbingVideo())
		{
			g_print("FW cam xy could not grab in initVision!!!\n");
			return;
		}

		usleep(1e5);
		//pthread_create(&fthread_xz, NULL, FPSprint_xz, NULL);  //start frame print thread
		pthread_create(&vthread, NULL, visionThread, NULL);  //start vision thread
	}
	return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Frame Print Thread
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This thread is not used? UI still displays images after this is commented out.
void* FPSprint(void*)
{
	char strReceive[50];
	char strReceive_xz[50];

	while(!killVisionThread)   //while the image processing is running //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
		//  int sprintf(char *str, const char *format, ...) sends formatted output to a string pointed to by str.
		sprintf(strReceive, "%.1f", fpsReceive); //writes into strRecieve the frames per second
		sprintf(strReceive_xz, "%.1f", fpsReceive_xz);
		gdk_threads_enter();
		gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive); //draw on the gui
		gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
		gdk_threads_leave();
		usleep((int)1e6); //sets frame rate display frequency
	}
	//printf("In the test zone!\n");
	sprintf(strReceive, "N/A"); //when we turn off the vision, write N/A into the gui
	sprintf(strReceive_xz, "N/A");
	gdk_threads_enter();
	gtk_label_set_text(GTK_LABEL(labelFPSreceive), strReceive);
	gtk_label_set_text(GTK_LABEL(labelFPSreceive_xz), strReceive_xz);
	gdk_threads_leave();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* visionThread(void*)
{
	printf("@ the Beginning of visionThread().\n");

	int index = 0;
	unsigned char *inImage;// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m, img_m_color, img_m_gray;

	Mat threshold_output;    // for threshold and rectangle detection

	int i;
	timeval tStart, tEnd;
	float time;
	double current_time;
	float fpsVec[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex = 0;

	double time_current, time_elapsed, time_init;
	struct timeval start;
	gettimeofday(&start, NULL);
	time_init = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time

	while(!killVisionThread) //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
        //printf("Marker 1.\n");
		//g_print("Got frame %d.   ", frame++);

		gettimeofday(&tStart, NULL);
		//usleep(6e4); //slow down vision thread

        // this function watis for a new frame, it takes a long time, so we can do some image processing in this thread
		inImage = cam.grabAframe(); //unsigned char *inImage;
		if(inImage == NULL)
		{
			g_print("Error in firewire stream! Reattempting...\n");
			usleep((int)1e3); // I don't know what the wait delay should be
		}

		img_m = Mat(height, width, CV_8UC1, inImage); //convert to Mat format

        flag_newFrame = 1;																// (07-15) indicate new frame

		//gettimeofday(&start, NULL);
		//time_current = (double) start.tv_sec + start.tv_usec*1e-6 ; // Start time
		//time_elapsed = time_current - time_init;
		//time_current = time_init;
		//printf("in vision thread, time is %.5f.\n", time_elapsed);
		//

		if(edgemap==1)
		{
			Canny(img_m, img_m, cannyLow, cannyHigh, 3 ); //edge detect

			if(dilater>0) //if dilater = 0, just use original edgemap
			{
				dilate( img_m, img_m, Mat(), Point(-1, -1), dilater, 1, 1);
				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
				erode( img_m, img_m, Mat(), Point(-1, -1), dilater, 1, 1);
			}
		//	 	circle( img_m, MM, 10, Scalar(20,100,255) , -1, 8, 0 );          // Test Hough circle detection mode
		}

        //printf("Marker 2.\n");

		if(detect == 1) //for threshold and bounding box detection
		{
			blur( img_m, threshold_output, Size(4,4) ); //blur image to remove small blips etc
			threshold( threshold_output, threshold_output, visionParam1, 255, THRESH_BINARY_INV );
			//adaptiveThreshold(img_m, threshold_output, 255,	ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,91,0);
			if(dilater>0) //if dilater = 0, just use original edgemap
			{
				dilate( threshold_output, threshold_output, Mat(), Point(-1, -1), dilater, 1, 1);
				erode( threshold_output, threshold_output, Mat(), Point(-1, -1), 2*dilater, 1, 1);
				dilate( threshold_output, threshold_output, Mat(), Point(-1, -1), dilater, 1, 1);
			}
			if(binary==1) //show binary image, don't do any more processing
			{
				cvtColor(threshold_output, img_m_color, CV_GRAY2BGR); //convert to color
				gdk_threads_enter();	//display video image in program window
				gtk_image_set_from_pixbuf(videoWindow, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8, img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
				gdk_threads_leave();
				continue; //don't do any more processing
			}
            cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color
		}
		else
			cvtColor(img_m, img_m_color, CV_GRAY2BGR); //convert to color anyways

		//draw mouse clicks:
		if(mouse.x>0)
			circle( img_m_color, mouse, 4, Scalar(  200, 50, 0), 2, 8, 0 );
		if(mouseR.x>0)
			circle( img_m_color, mouseR,4, Scalar( 20, 250, 300 ), 2, 8, 0 );
		if(mouseC.x>0)
			circle( img_m_color, mouseC, 4, Scalar( 220, 130, 100 ), 2, 8, 0 );

		//imshow("this is a test2",img_m);//display video image in separate cv window
		// compute 10 average fps

		img_m_color_for_display = img_m_color;   // Transfer the image data to the buffer for display

		//  Needed for Frame rate calculation of Top camera (xy)
		gettimeofday(&tEnd, NULL);
		current_time = ((double)tEnd.tv_sec + (double)tEnd.tv_usec*1e-6) ;
		time = (int)round( (((double)tEnd.tv_sec + (double)tEnd.tv_usec*1e-6) - ((double)tStart.tv_sec + (double)tStart.tv_usec*1e-6) )*1000.0);
		fpsVec[fpsIndex++] = 1000.0/time;
		if(fpsIndex > 9) fpsIndex = 0;
		fpsReceive = 0;
		for(int i = 0; i < 10; i++)
			fpsReceive += fpsVec[i];
		fpsReceive /= 10.0;
	} //end vision loop due to killVisionthread==1

	cam.stopGrabbingVideo();
	usleep ((int)1e5); //make sure that ImageProc_xz has closed
	cam.deinitialize();

	printf("@ the End of visionThread().\n");
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* visionThread_xz(void*)
{
	printf("In vision thread x-z.\n");

	unsigned char *inImage_xz;// = (unsigned char*)malloc(sizeof(unsigned int)*width*height*depth);
	Mat img_m_xz, img_m_color_xz;

	Mat threshold_output_xz; //for threshold and rectangle detection

	int i;
	timeval tStart_xz, tEnd_xz;
	float time_xz;
	double current_time_xz;
	float fpsVec_xz[10] = {10,10,10,10,10,10,10,10,10,10};
	int fpsIndex_xz = 0;

	// local variables - Zhe
	Mat img_m_xz_ori; // storing the original image of xz
	Mat img_m_xz_bi; // storing the binary image of xz
	Mat img_m_xz_temp;
	float magnet_maxarea, pre_area = 100.0;
	int ind_maxarea;
	bool contour_number = false; // whether the number of contours is greater than 2 ?

    double time_current_xz, time_elapsed_xz, time_init_xz;
	struct timeval start_xz;
	gettimeofday(&start_xz, NULL);
	time_init_xz = (double) start_xz.tv_sec + start_xz.tv_usec*1e-6 ; // Start time

	while(!killVisionThread) //repeat vision loop until we set killVisionthread=1 using stopVision()
	{
		//g_print("Got frame %d.   ", frame++);

		gettimeofday(&tStart_xz, NULL);
		//usleep(6e4); //slow down vision thread

		inImage_xz = cam_xz.grabAframe_xz(); //unsigned char *inImage;

		if(inImage_xz == NULL)
		{
			g_print("Error in firewire stream xz! Reattempting...\n");
			usleep((int)1e3); // I don't know what the wait delay should be
		}

		img_m_xz = Mat(height, width, CV_8UC1, inImage_xz); //convert to Mat format

		if (magnetdetection)
        {
            //detect_xz = 0;
            img_m_xz_ori = img_m_xz.clone(); // store the original image
            //blur(img_m_xz, img_m_xz, Size(3,3)); // blur

            if (mouse_xz.x>0) // designate ROI by clicking: top-left corner
            {
                img_m_xz(Range(0, mouse_xz.y),Range(0,640)) = Scalar::all(255);
                img_m_xz(Range(0,480),Range(0, mouse_xz.x)) = Scalar::all(255);
            }
            if (mouseR_xz.x>0) // designate ROI by clicking: bottom-right corner
            {
                img_m_xz(Range(mouseR_xz.y, 480),Range(0,640)) = Scalar::all(255);
                img_m_xz(Range(0,480),Range(mouseR_xz.x, 640)) = Scalar::all(255);
            }
            if (mouseC_xz.x>0) // designate ROI by clicking: needle area
            {
                needle_x = mouseC_xz.x; needle_y = mouseC_xz.y;
                img_m_xz(Range(0, mouseC_xz.y),Range(mouseC_xz.x-needle_thick/2, mouseC_xz.x+needle_thick/2)) = Scalar::all(255);
            }

            threshold( img_m_xz, img_m_xz, magnet_threshold, 255, THRESH_BINARY); // binarize

            img_m_xz_bi = img_m_xz.clone(); // store the binary image

            Mat strel = getStructuringElement( MORPH_ELLIPSE, Size(closediameter, closediameter) ); // generate a disk structure element
            dilate( img_m_xz, img_m_xz, strel); // do the close operation using disk kernel
            erode ( img_m_xz, img_m_xz, strel);

            vector<vector<Point> > magnetcontours;
            vector<Vec4i> hierarchy;
            img_m_xz_temp =  img_m_xz.clone();
            findContours( img_m_xz, magnetcontours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // find magnet contours

            RotatedRect minRect;
            if (magnetcontours.size()>=2) //contour.0 is the image
            {
                double magnetcontours_area[magnetcontours.size()];
                magnet_maxarea = 0.0;
                for ( int i=1; i<magnetcontours.size(); i++ ) // find the largest contour as the magnet contour candidate
                {
                    magnetcontours_area[i] =  contourArea(magnetcontours[i]);
                    if ( magnetcontours_area[i] > magnet_maxarea )
                    {
                        magnet_maxarea = magnetcontours_area[i];
                        ind_maxarea = i;
                    }
                }
                magnet_area = magnet_maxarea;
                contour_number = true;
            }
            else
            {
                continue; // skip this loop
            }

            if ( contour_number & (magnet_area > 0.25*trust_area) & (magnet_area < 4*trust_area) ) // draw box and calculate only if the candidate magnet area is in a certain range
            {
                minRect = minAreaRect( Mat(magnetcontours[ind_maxarea]) ); // fit using a rectangle

                img_m_xz = img_m_xz_temp.clone();

                if (!showprocess) // are we showing the already processed image?
                {
                    img_m_xz = img_m_xz_ori;
                }

                if (showbox) // are we displaying the boundary box?
                {
                    Point2f rect_points[4]; minRect.points( rect_points ); //draw the magnet contour
                    for( int j = 0; j < 4; j++ )
                    {
                        line( img_m_xz, rect_points[j], rect_points[(j+1)%4], 72, 1, 8 );
                    }
                    orientation_display(rect_points[0], rect_points[1], rect_points[2], rect_points[3]); // calculating box parameters
                    drawMagetization=1;
                }
                contour_number = false;
            }
            else
            {
                img_m_xz =  img_m_xz_temp.clone();
                if (!showprocess) // are we showing the already processed image?
                {
                    img_m_xz = img_m_xz_ori;
                }
            }
        }

		if(edgemap_xz==1)
		{
			Canny(img_m_xz, img_m_xz, cannyLow_xz, cannyHigh_xz, 3 ); //edge detect

			if(dilater_xz>0) //if dilater = 0, just use original edgemap
			{
				dilate( img_m_xz, img_m_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
				erode( img_m_xz, img_m_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
			}
		}

		if(detect_xz == 1) //for threshold and bounding box detection
		{
			blur( img_m_xz, threshold_output_xz, Size(4,4) ); //blur image to remove small blips etc
			threshold( threshold_output_xz, threshold_output_xz, visionParam1_xz, 255, THRESH_BINARY_INV );
			//adaptiveThreshold(img_m, threshold_output, 255,	ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,91,0);
			if(dilater_xz>0) //if dilater = 0, just use original edgemap
			{
				dilate( threshold_output_xz, threshold_output_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
				//smooth( img_m, img_m, CV_MEDIAN, 5, 5);
				erode( threshold_output_xz, threshold_output_xz, Mat(), Point(-1, -1), 2*dilater_xz, 1, 1);
				dilate( threshold_output_xz, threshold_output_xz, Mat(), Point(-1, -1), dilater_xz, 1, 1);
			}
			if(binary_xz==1) //show binary image, don't do any more processing
			{
				cvtColor(threshold_output_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color
				gdk_threads_enter();	//display video image in program window
				gtk_image_set_from_pixbuf(videoWindow2, gdk_pixbuf_new_from_data(img_m_color_xz.data, GDK_COLORSPACE_RGB, false, 8, img_m_color_xz.cols, img_m_color_xz.rows, img_m_color_xz.step, NULL, NULL));
				gdk_threads_leave();
				continue; //don't do any more processing
			}
			cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color
		}
		else
			cvtColor(img_m_xz, img_m_color_xz, CV_GRAY2BGR); //convert to color anyways

        // draw mouse clicks
        if (draw_points)
        {
            if(mouse_xz.x>0)
                circle( img_m_color_xz, mouse_xz, 4, Scalar(  200, 50, 0), 2, 8, 0 );
            if(mouseR_xz.x>0)
                circle( img_m_color_xz, mouseR_xz,4, Scalar( 20, 250, 300 ), 2, 8, 0 );
            if(mouseC_xz.x>0)
                circle( img_m_color_xz, mouseC_xz, 4, Scalar( 220, 130, 100 ), 2, 8, 0 );
        }

        // display y field amplitude
/*
        char angle_text[51];
        sprintf(angle_text, "%5.1f mT", field_x);
        putText( img_m_color_xz, angle_text, Point(5,475), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,0,0), 2);

*/
		// draw boundary of ROI
		Point startP; Point endP;
        if (draw_roi)
        {
            Scalar roi_color = Scalar(72, 72, 72);

            startP = mouse_xz; endP.x = mouseC_xz.x - needle_thick/2; endP.y = mouse_xz.y;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP.x = mouseC_xz.x - needle_thick/2; endP.y = mouseC_xz.y;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP.x = mouseC_xz.x + needle_thick/2; endP.y = mouseC_xz.y;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP.x = mouseC_xz.x + needle_thick/2; endP.y = mouse_xz.y;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP.x = mouseR_xz.x; endP.y = mouse_xz.y;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP = mouseR_xz;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP.x = mouse_xz.x; endP.y = mouseR_xz.y;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);

            startP = endP; endP = mouse_xz;
            line( img_m_color_xz, startP, endP, roi_color, 1, 8, 0);
        }

        // put texts
        if (draw_annotation && 0)   //////////////////////////////  MODIFIED
        {
            Point textpoint;
            double fab_fontscale = 0.6;
            Scalar fab_color = Scalar(255, 0, 0); // red
            int fab_thickness = 1;
            //show angle
            char angle_text[51];
            sprintf(angle_text, "Angle:%6.1f", m_a);
            textpoint.x=5; textpoint.y=475;
            putText( img_m_color_xz, angle_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
            //show temperature
            sprintf(angle_text, "T:%5.1f", current_temp);
            textpoint.x=160; textpoint.y=475;
            putText( img_m_color_xz, angle_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
            // show time
            textpoint.x=5; textpoint.y=455;
            putText( img_m_color_xz, fab_time, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
            // show status
            textpoint.x=80; textpoint.y=455;
            putText( img_m_color_xz, fab_status, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
            // draw field at bottom right
            startP.x = 580;
            startP.y = 420;
            endP.x = 580 + 40*cos(field_angle*PI/180.0);
            endP.y = 420 - 40*sin(field_angle*PI/180.0);
            line( img_m_color_xz, startP, endP, Scalar(255,0,0), 2, 8, 0);
            endP.x = 580 + 40*field_x/14.0;
            endP.y = 420 - 40*field_z/14.0;
            circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
            circle( img_m_color_xz, startP, 40, Scalar(255,0,0), 1, 8, 0 );
        }
        else if(0)  //////////////////////////////  MODIFIED
        {
            Point textpoint;
            double fab_fontscale = 0.6;
            Scalar fab_color = Scalar(255, 0, 0); // red
            int fab_thickness = 1;
            //show angle
            char angle_text[51];
            sprintf(angle_text, "Angle:%6.1f", m_a);
            textpoint.x=5; textpoint.y=475;
            putText( img_m_color_xz, angle_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
            //show temperature
            sprintf(angle_text, "T:%5.1f", current_temp);
            textpoint.x=160; textpoint.y=475;
            putText( img_m_color_xz, angle_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);
            // draw field at bottom right
            startP.x = 580;
            startP.y = 420;
            endP.x = 580 + 40*cos(field_angle*PI/180.0);
            endP.y = 420 - 40*sin(field_angle*PI/180.0);
            line( img_m_color_xz, startP, endP, Scalar(255,0,0), 2, 8, 0);
            endP.x = 580 + 40*field_x/14.0;
            endP.y = 420 - 40*field_z/14.0;
            circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
            circle( img_m_color_xz, startP, 40, Scalar(255,0,0), 1, 8, 0 );
        }

        // this draws the x-y plane field direction and magnitude
        startP.x = 580;
        startP.y = 420;
        float field_angle_xy = atan2(field_y, field_x);
        endP.x = 580 + 40*cos(field_angle_xy);
        endP.y = 420 - 40*sin(field_angle_xy);
        line( img_m_color_xz, startP, endP, Scalar(255,0,0), 2, 8, 0);
        endP.x = 580 + 40*field_x/14.0;
        endP.y = 420 - 40*field_y/14.0;
        circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
        circle( img_m_color_xz, startP, 40, Scalar(255,0,0), 1, 8, 0 );

        Point textpoint;
        double fab_fontscale = 0.6;
        Scalar fab_color = Scalar(255, 0, 0); // red
        int fab_thickness = 1;
        char xy_text[2];
        sprintf(xy_text, "XY");
        textpoint.x=570; textpoint.y=475;
        putText( img_m_color_xz, xy_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);

        // this draws the x-z plane field direction and magnitude
        startP.x = 480;
        startP.y = 420;
        float field_angle_xz = atan2(field_z, field_x);
        endP.x = 480 + 40*cos(field_angle_xz);
        endP.y = 420 - 40*sin(field_angle_xz);
        line( img_m_color_xz, startP, endP, Scalar(255,0,0), 2, 8, 0);
        endP.x = 480 + 40*field_x/14.0;
        endP.y = 420 - 40*field_z/14.0;
        circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
        circle( img_m_color_xz, startP, 40, Scalar(255,0,0), 1, 8, 0 );

        //Point textpoint;
        //double fab_fontscale = 0.6;
        //Scalar fab_color = Scalar(255, 0, 0); // red
        //int fab_thickness = 1;
        char xz_text[2];
        sprintf(xz_text, "XZ");
        textpoint.x=470; textpoint.y=475;
        putText( img_m_color_xz, xz_text, textpoint, FONT_HERSHEY_SIMPLEX, fab_fontscale, fab_color, fab_thickness);


        if (drawMagetization && 0)  //////////////////////////////  MODIFIED
        {
            //draw magnet direction
            startP.x = m_x; startP.y = m_y;
            endP.x = m_x + Mlength/1.5 * sin(m_a*PI/180.0);
            endP.y = m_y + Mlength/1.5 * cos(m_a*PI/180.0);
            line( img_m_color_xz, startP, endP, Scalar(255,0,0), 1, 8, 0);
            circle( img_m_color_xz, endP, 2, Scalar(255,0,0), 2, 8, 0 );
            //draw destination angle
            if (showdestination)
            {
                startP.x = m_x; startP.y = m_y;
                endP.x = m_x + Mlength/1.5 * sin(destination_angle*PI/180.0);
                endP.y = m_y + Mlength/1.5 * cos(destination_angle*PI/180.0);
                line( img_m_color_xz, startP, endP, Scalar(0,255,0), 1, 8, 0);
                circle( img_m_color_xz, endP, 2, Scalar(0,255,0), 2, 8, 0 );
            }
            //draw field angle & magnitude
            if (showfielddirection)
            {
                startP.x = m_x; startP.y = m_y;
                endP.x = m_x + Mlength/1.5 * cos(field_angle*PI/180.0);
                endP.y = m_y - Mlength/1.5 * sin(field_angle*PI/180.0);
                line( img_m_color_xz, startP, endP, Scalar(255,99,0), 1, 8, 0);

                endP.x = m_x + Mlength/1.5 * field_x/14.0;
				endP.y = m_y - Mlength/1.5 * field_z/14.0;
                circle( img_m_color_xz, endP, 2, Scalar(255,99,0), 2, 8, 0 );
            }
            drawMagetization=0;
        }

		img_m_color_for_display2 = img_m_color_xz;
		//gdk_threads_enter();	//display video image in program window
		//gtk_image_set_from_pixbuf(videoWindow2, gdk_pixbuf_new_from_data(img_m_color.data, GDK_COLORSPACE_RGB, false, 8, img_m_color.cols, img_m_color.rows, img_m_color.step, NULL, NULL));
		//gdk_threads_leave();

//imshow("this is a test2",img_m);//display video image in separate cv window

		// compute 10 average fps
		gettimeofday(&tEnd_xz, NULL);
		current_time_xz = ((double)tEnd_xz.tv_sec + (double)tEnd_xz.tv_usec*1e-6) ;
		time_xz = (int)round( (((double)tEnd_xz.tv_sec + (double)tEnd_xz.tv_usec*1e-6) - ((double)tStart_xz.tv_sec + (double)tStart_xz.tv_usec*1e-6) )*1000.0);
		fpsVec_xz[fpsIndex_xz++] = 1000.0/time_xz;
		if(fpsIndex_xz > 9) fpsIndex_xz = 0;
		fpsReceive_xz = 0;
		for(int i = 0; i < 10; i++)
			fpsReceive_xz += fpsVec_xz[i];
		fpsReceive_xz /= 10.0;
		//g_print("  %.1f fps yz\n", fpsReceive_xz); //we now do this in the gui using a separate thread

	} //end vision loop due to killVisionthread==1

	cam_xz.stopGrabbingVideo_xz();
	usleep ((int)1e5); //make sure that ImageProc_xz has closed
	cam.deinitialize(); //taken care of by top camera deinitialize call

	printf("@ the End of visionThread_xz().\n");
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stopVision(void)
{
	killVisionThread = 1;
}

GdkPixbuf *convertOpenCv2Gtk(IplImage *image) //Currently only RGB images with 8 bits per sample are supported. From http://subinsebastien.tumblr.com/post/2839808825/opencv-gtk-and-a-day
{
	IplImage *gtkMask;
//    gtkMask = cvLoadImage("testimage.jpg",CV_LOAD_IMAGE_UNCHANGED);

	gtkMask = image;

	cvCvtColor( image, gtkMask, CV_BGR2RGB );
    	GdkPixbuf *pix;
	gdk_threads_enter(); //may not need this here???
    	pix = gdk_pixbuf_new_from_data((guchar*)gtkMask->imageData,
              GDK_COLORSPACE_RGB,
              FALSE,
              gtkMask->depth,
              gtkMask->width,
              gtkMask->height,
              (gtkMask->widthStep),
              NULL,
              NULL);
	gdk_threads_leave();
    	return pix;
}

void set_edgemap(int d)
{
	edgemap = d;
}
void set_binary(int d)
{
	binary = d;
}
void setGain_vision(int d)
{
	cam.setGain(d);
}
void setShutter_vision(int d)
{
	cam.setShutter(d);
}
void setDilate_vision(int d)
{
	dilater = d; //for image processing on edgemap
}
void setvisionParam1_vision(int d)
{
	visionParam1 = d; //for image processing
}
void setvisionParam2_vision(int d)
{
	visionParam2 = d; //for image processing
}
void setcannyLow_vision(int d)
{
	cannyLow = d; //for image processing
}
void setcannyHigh_vision(int d)
{
	cannyHigh = d; //for image processing
}
void setdetect_vision(int d)
{
	detect = d; //for image processing
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setTopCam_vision(int d)
{
	topcam_on = d; //is the topcam on?
}

void set_edgemap_xz(int d)
{
	edgemap_xz = d;
}
void set_binary_xz(int d)
{
	binary_xz = d;
}
void setGain_xz_vision(int d)
{
	cam_xz.setGain_xz(d);
}
void setShutter_xz_vision(int d)
{
	cam_xz.setShutter_xz(d);
}
void setDilate_xz_vision(int d)
{
	dilater_xz = d; //for image processing on edgemap
}
void setvisionParam1_xz_vision(int d)
{
	visionParam1_xz = d; //for image processing
}
void setvisionParam2_xz_vision(int d)
{
	visionParam2 = d; //for image processing
}
void setcannyLow_xz_vision(int d)
{
	cannyLow_xz = d; //for image processing
}
void setcannyHigh_xz_vision(int d)
{
	cannyHigh_xz = d; //for image processing
}
void setdetect_xz_vision(int d)
{
	detect_xz = d; //for image processing
}
void settopcam_xz_vision(int d)
{
	topcam_on = d; //is the sidecam on?
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//set magnet detection variables -- Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_magnetdetection(int d)
{
	magnetdetection = d; //are we performing magnet detection?
}

void set_showbox(int d)
{
	showbox = d; //for showing magnet boundary box
}

void set_draw_annotation(int d)
{
	draw_annotation = d; //for showing direction arrows
}

void set_showprocess(int d)
{
	showprocess = d; //for showing image processing results
}

void set_draw_roi(int d)
{
	draw_roi = d; //for showing boundary of ROI
}

void set_draw_points(int d)
{
	draw_points = d; //for showing mouse clicks
}

void set_closediameter(int d)
{
    closediameter = d;
}

void set_needle_thick(int d)
{
    needle_thick = d;
}

void set_magnet_threshold(double d)
{
    magnet_threshold = d;
}

void set_showdestination(int d)
{
    showdestination = d;
}

void set_showfielddirection(int d)
{
    showfielddirection = d;
}

void reverse_magent()
{
    m_a = anglePlus_v(m_a, 180.0);

    for ( int i = 0; i<6; i++)
    {
        m_a_history[i] = m_a;
    }

}

void set_magnet_trust()
{
    trust_area  = magnet_area;
    flag_magnet_sampled = true;
}

void orientation_display(Point2f p1, Point2f p2, Point2f p3, Point2f p4)
{
    float m_x_sum = 0, m_y_sum = 0 , m_a_ave = 0;
    float diff1x = p1.x-p2.x, diff2x = p2.x-p3.x, diff1y = p1.y-p2.y, diff2y = p2.y-p3.y;
    float distance1 = sqrt(diff1x*diff1x + diff1y*diff1y); // distance between 1 and 2
    float distance2 = sqrt(diff2x*diff2x + diff2y*diff2y); // distance between 2 and 3

    if (distance1 > distance2)
    {
        Mlength = distance1;
        Mwidth  = distance2;
        m_a_temp = atan2(diff1x, diff1y) * 180.0/PI;
    }
    else
    {
        Mlength = distance2;
        Mwidth  = distance1;
        m_a_temp = atan2(diff2x, diff2y) * 180.0/PI;
    }

    m_x_temp = (p1.x+p2.x+p3.x+p4.x)/4.0;
    m_y_temp = (p1.y+p2.y+p3.y+p4.y)/4.0;

    // decide based on the previous angle between 2 possible directions
    float m_aOp = anglePlus_v( m_a_temp, 180.0 ); // optional
    if ( abs( angleMinus_v(m_a_temp, m_a_history[5]) ) > abs( angleMinus_v(m_aOp, m_a_history[5]) ) )
        m_a_temp = m_aOp;

    ///////////////////////
    // whether to accept this m_a_temp based on the previous several values????????????
    ///////////////////////

    // update the historical data for m_x, m_y and m_a
    for ( int i = 0; i < 5; i++ )
    {
        m_x_history[i] = m_x_history[i+1];
        m_x_sum = m_x_sum + m_x_history[i];

        m_y_history[i] = m_y_history[i+1];
        m_y_sum = m_y_sum + m_y_history[i];

        m_a_history[i] = m_a_history[i+1];
    }
    m_x_history[5] = m_x_temp;
    m_x_sum = m_x_sum + m_x_history[5];
    m_y_history[5] = m_y_temp;
    m_y_sum = m_y_sum + m_y_history[5];
    m_a_history[5] = m_a_temp;
    m_a_ave = angleMiddle_v( angleMiddle_v(m_a_history[0],m_a_history[2]), angleMiddle_v(m_a_history[3],m_a_history[5]) );

    // output m_x, m_y and m_a;
    m_x = m_x_sum/6.0;
    m_y = m_y_sum/6.0;
    m_a = m_a_ave;
    //printf(" (%.2f  %.2f)  %.2f\n", m_x_sum/6.0, m_y_sum/6.0, m_a_sum/6.0);
}

float anglePlus_v( float a, float b) // rotate angle a by angle b => angle c, wrap c in (-pi, pi]. "v" means vision.c
{
    float c = a + b;
    while ( c > 180 )   { c = c - 360.0; }
    while ( c <= -180 ) { c = c + 360.0; }
    return c;
}

float angleMinus_v( float c, float a) // rotate angle a by angle b => angle c, wrap b in (-pi, pi]. "v" means vision.c
{
    float b = c - a;
    while ( b > 180 )   { b = b - 360.0; }
    while ( b <= -180 ) { b = b + 360.0; }
    return b;
}

float angleMiddle_v( float a, float b) // the middle angle between angle a and b. "v" means vision.c
{
    float d,m;
    d = angleMinus_v(b, a);
    m = anglePlus_v(a, d/2);
    while ( m > 180 )   { m = m - 360.0; }
    while ( m <= -180 ) { m = m + 360.0; }
    return m;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
{
	switch(whichScreen)
	{
		case 0: //top screen
			switch(whichMouse)
			{
				case 1: //left mouse
					mouse.x 		= mouseClick[0];
					mouse.y 		= mouseClick[1];
					break;
				case 2: //right mouse
					mouseR.x 		= mouseClick[0];
					mouseR.y		= mouseClick[1];
					break;
				case 3: //center mouse
					mouseC.x 		= mouseClick[0];
					mouseC.y 		= mouseClick[1];
					break;
			}
			break;
		case 1: //side screen
			switch(whichMouse)
			{
				case 1: //left mouse
					mouse_xz.x 		= mouseClick[0];
					mouse_xz.y 		= mouseClick[1];
					break;
				case 2: //centre mouse
					mouseC_xz.x 		= mouseClick[0];
					mouseC_xz.y 		= mouseClick[1];
					break;
				case 3: //right mouse
					mouseR_xz.x 		= mouseClick[0];
					mouseR_xz.y		= mouseClick[1];
					break;
			}
			break;
	}
}
