#include "twistField.h"
#include "AutoFabrication.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// twisting field walking variables
bool flag_twist_field = false, twisted_walking_go = false, flag_manual_field_loop = false, manual_field_go = false;
float theta = 0.0, beta = 0.0, omega = 0.0, phi = 0.0, Bmag = 0.0;
float bx_global = 0.0, by_global = 0.0, bz_global = 0.0, dbx_global = 0.0, dby_global = 0.0, dbz_global = 0.0;
float bx_old = 0.0, by_old = 0.0, bz_old = 0.0;

// discrete bending variables
bool flag_discrete_bending = false, discrete_go = false;
float kp = 0.08, ki = 0.2, destination_angle = 0.0;
float discrete_field_s; // starting angle of discrete bending field
extern int counter;
extern float m_x, m_y, m_a, m_a_temp; // the averaged and temporary magnet angle
bool discrete_done = false;

// continuous bending variables
bool flag_continuous_bending = 0, continuous_go = 0;
float continuous_radius, continuous_central, continuous_start, continuous_time;
bool continuous_feedback, continuous_done;
float continuous_PI;

// sinusoidal field (2D) variables
bool flag_sinusoidal_field = false, sinusoidalfield_go = false;
float fab_heading_s = 0.0; float fab_amp_s = 0.0, fab_fre_s = 0.0;

// rotational field (vertical) variables
bool flag_rotational_field = false, rotationalfield_go = false;
float fab_heading = 0.0; float fab_amp = 0.0, fab_fre = 0.0;

// rotational field (horizontal) variables
bool flag_rotational_field_h = false, rotationalfield_go_h = false;
float fab_heading_h = 1;

//sawtooth mode variables
bool flag_sawtooth_mode = false, sawtooth_go = false;
float peak_angle_st = -30.0;

// temperature control variables
bool flag_temp_control = false;
float destination_temp = 0;
bool temp_go = false;
float temp_kp = 0.1/500.0, temp_ki = 0.01/50000.0;
int temp_ind = 0;
float current_temp;
bool temp_done = false;

// motor control variables
bool stepper_on = false, runCalled = true, initSetup = false;
//const double res[] = {0.177879461046043, 0.0889397305230215, 0.0444698652615108, 0.0222349326307554, 0.0111174663153777}; // [OLD FEEDER] Linear extension resolution (mm) per step of stepper
const double res[] = {-0.0692418332755,-0.034620917127,-0.017310457585,-0.008655229771,-0.004327613907}; // [NEW FEEDER]  Linear extension resolution (mm) per step of stepper
int m = 5, stepSize = 5;

// automatic feeding variables
bool flag_auto_feeding = false, feeding_go = false;
float feeding_distance = 0.0, feeding_speed = 0.0;
bool feeding_done = false;
double DistanceGone = 0.0;

// manual feeding variables
float feeding_increments = 0.0;
bool flag_manual_feeding = false, flag_increment = false, flag_decrement = false, flag_setSpeed = false;

// automatic fabrication variables
bool left_constraint = false;
bool flag_auto_fab = false;
char fab_status[] = "Click 'Fabricate' to trigger fabrication.";
char fab_time[] = "00:00";

// timing thread
bool flag_timing = false;
double time_sec_f;

// cut off thread
float neutral_cut = 0.0, mag_cut = 180.0, fre_cut = 0.5;
bool flag_cut_off = false;

//field control
bool coil_3d = true, coil_2d_xz = false;
float factor_x = 5.0964, factor_y = 4.999, factor_z = 5.1677;
float field_mag_fab = 14.0;
float field_x, field_y, field_z, field_mag, field_angle = -90.0;
float field_angle_m = 0.0; // field_angle in magnet coordinate

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pins
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// pin assignments for motor driver
const int enablePin = 0;
const int MS1 = 1;
const int MS2 = 2;
const int MS3 = 3;
const int stepPin = 4;
const int dirPin = 5;

AccelStepper Stepper1 (1, stepPin, dirPin); //make Stepper1 object








///////////////////////////////////////////////////////////////

void* circ_twisting_field_walk_thread (void*threadid)
{
    printf("--twisting field thread started\n");

    struct timeval start;
    double time_current, time_last, time_elapsed, time_out, time_out_sum, vx, vy, vz; //time in second
	double theta_local, beta_local, phi_local, omega_local;
	double theta_localp, beta_localp, phi_localp, omega_localp;
    double vx_old, vy_old, vz_old;
    double prec = 0.000000001; //mT
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds.

    while ( flag_twist_field )
    {

        if (twisted_walking_go)
        {

            gettimeofday(&start, NULL);
            time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;
			
			 
			theta_local = theta;
			beta_local = beta;
			phi_local = phi;
			omega_local = omega; 


			vx = Bmag * ((cosd(theta_local)*cosd(beta_local))*(cosd(90-phi_local*0.5)*cosd(360.0*omega_local*time_elapsed)) - sind(theta_local)*(cosd(90-phi_local*0.5)*sind(360.0*omega_local*time_elapsed)) + cosd(theta_local)*sind(beta_local)*cosd(phi_local*0.5));
			vy = Bmag * ((sind(theta_local)*cosd(beta_local))*(cosd(90-phi_local*0.5)*cosd(360.0*omega_local*time_elapsed)) + cosd(theta_local)*(cosd(90-phi_local*0.5)*sind(360.0*omega_local*time_elapsed)) + sind(theta_local)*sind(beta_local)*cosd(phi_local*0.5));
			vz = Bmag * (-sind(beta_local)*(cosd(90-phi_local*0.5)*cosd(360.0*omega_local*time_elapsed)) + cosd(beta_local)*cosd(phi_local*0.5));

            set_field_xyz_2 (vx, vy, vz, 0.0, 0.0, 0.0);
			
            field_x = vx;
            field_y = vy;
            field_z = vz;

			/*theta_localp = theta_local;
			beta_localp = beta_local;
			phi_localp = phi_local;
			omega_localp = omega_local;*/
			
            time_elapsed = time_last - time_initial -time_out;
            time_out_sum = time_out;
        }
        else
        {
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_out = time_current - time_last + time_out_sum;
        }

    }
    resetCoils();
    printf("--twisting field thread ended\n");
    return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//OY circ_twisting_field_walk_thread related functions
void init_twist_field (int d)
{
    printf("@ the Beginning of circ_twisting_field_walk_thread.\n");
    flag_twist_field = true;
    pthread_t twisting_field;
    pthread_create(&twisting_field, NULL, circ_twisting_field_walk_thread, NULL);
}

void stop_twist_field (int d)
{
    flag_twist_field = false;
}

void init_twisted_walking (int d)
{
    twisted_walking_go = true;
}

void stop_twisted_walking (int d)
{
    twisted_walking_go = false;
}

void set_theta_heading (float d)
{
    theta = d;
}
void set_beta_tild (float d)
{
    beta = d;
}
void set_ang_freq (float d)
{
    omega = d;
}
void set_ang_span (float d)
{
    phi = d;
}
void set_tfield_mag (float d)
{
    Bmag = d;
}
void set_bx_mag(float d)
{
    bx_global = d;
}
void set_by_mag(float d)
{
    by_global = d;
}
void set_bz_mag(float d)
{
    bz_global = d;
}
void set_dbx_mag(float d)
{
    dbx_global = d;
}
void set_dby_mag(float d)
{
    dby_global = d;
}
void set_dbz_mag(float d)
{
    dbz_global = d;
}





void* set_manual_field_thread (void*threadid)
{
    printf("--manual field thread started\n");
    //double data[16];
    bool flag_manualisset =true, flag_resetset=false;
    //uint aiChan = (0b1000000000000000);
    //s826_aiInit(aiChan,1);
    while ( flag_manual_field_loop )
    {

        if (manual_field_go)
        {
            if(flag_manualisset){
            set_field_xyz_2 (bx_global, by_global, bz_global, dbx_global, dby_global, dbz_global);
            bx_old=bx_global;
            by_old=by_global;
            bz_old=bz_global;
            flag_manualisset = false;
            field_x = bx_global;
            field_y = by_global;
            field_z = bz_global;
            printf("--COILS SET MANUALLY\n");
            //sprintf(fab_status, "sample text....");
  			//s826_doPin(10, 1);

////

            }
            if(fabs(bx_global-bx_old)>0.01 || fabs(by_global-by_old)>0.01 || fabs(bz_global-bz_old)>0.01){
                flag_manualisset = true;
            }
            flag_resetset = true;
        //    s826_aiRead(aiChan, data);
        //    printf("Analog input %f\n",data[15]);
        }
        else if(flag_resetset)
        {
            resetCoils();
            flag_resetset = false;
            printf("--COILS RESETTED\n");
            //s826_doPin(10, 0);
        }

    }
    resetCoils();
    printf("--manual field thread ended\n");
    return NULL;
}

// set_manual_field_thread related functions
void init_manual_field (int d)
{
    printf("@ the Beginning of set_manual_field_thread.\n");
    flag_manual_field_loop = true;
    pthread_t manual_field;
    pthread_create(&manual_field, NULL, set_manual_field_thread, NULL);
}

void stop_manual_field (int d)
{
    flag_manual_field_loop = false;
}

void init_manual_field_go (int d)
{
    manual_field_go = true;
}

void stop_manual_field_go (int d)
{
    manual_field_go = false;
}

















///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// discrete bending thread (torque angle control, constant magnitude)
void* discrete_bending_thread (void*threadid)
{
    printf("--magnet control thread started\n");
    int counterP = -1, counterC;
    float mangle_d;
	bool discrete_set = false;
	float discrete_P, discrete_I, discrete_PI;
	float temp_discrete_field, discrete_field, discrete_field_s;
	bool discrete_satr = false;
    float des, dif;
    FILE *magnetangle=fopen("magnetangle.txt","w");
    float mangle_history[20]; // storing historic magnet angles
    float mangle_error[20]; //storing historic errors
    int discrete_counter;
    int temp_discrete_done; // 1: discrete_done = true; 0: false.
	//set_field_xyz(1, 0.0);
    // quick reflection after over shoot
    // torque angle > 180 and <270 = torque < -90
    while (flag_discrete_bending)
    {
        usleep(16666); // 60 Hz
        mangle_d = m_a;
        des = destination_angle;

        if (discrete_go)
        {
            if (!discrete_set)
            {
                discrete_field_s = field_angle_m;
                discrete_P = 0;
                discrete_I = 0;
                discrete_set = true;
            }

            discrete_counter++;
            if ( discrete_counter == 12 ) // update historical angle at 10 Hz
            {
                discrete_counter = 0;
                for (int i=0; i<19; i++)
                {
                    mangle_history[i] = mangle_history[i+1];
                    mangle_error[i] = fabs(des-mangle_history[i]);
                }
                mangle_history[19] = mangle_d;
                mangle_error[19] = fabs(angleMinus(des, mangle_history[19]));

                if ( (fabs(angleMinus(mangle_history[0], mangle_history[19])) < 1.5) && (mangle_error[19] < 1.0) ) // discrete_done: 4s = 1/60 * 12 * 20
                    discrete_done = true;
                else
                    discrete_done = false;

                printf("%d\n", discrete_done);
            }
            dif = angleMinus(des, mangle_d);

            if (!discrete_satr)
            {
                discrete_P = kp * dif;
                discrete_I = ki * dif/60.0 + discrete_I;
                discrete_PI = discrete_P + discrete_I;
            }

            temp_discrete_field = anglePlus(discrete_field_s, discrete_PI);

            if ( angleMinus(temp_discrete_field, mangle_d) > 90 ) // torque angle < 90 degrees
            {
                discrete_field = anglePlus(mangle_d, 90);
                discrete_satr = true;
            }
            else if ( angleMinus(temp_discrete_field, mangle_d) < -90 )
            {
                discrete_field = anglePlus(mangle_d, -90);
                discrete_satr = true;
            }
            else
            {
                discrete_field = anglePlus(discrete_field_s, discrete_PI);
                discrete_satr = false;
            }

            set_field_polar (14.0, magnet2field_angle(discrete_field));
        }
        else
        {
            discrete_set = false;
            discrete_done = false;
        }
    }
    fclose(magnetangle);
    coilCurrentClear();
    printf("--magnet control thread ended\n");
    return NULL;
}

// continuous bending thread
void* continuous_bending_thread (void*threadid)
{
    printf("--continuous bending thread started\n");
	bool continuous_set = false;
	int counterP_c = -1, counterC_c;
	float temp_continuous_field, continuous_magnet, continuous_field, continuous_field_s, mangle_c;
	float continuous_P = 0.0, continuous_I = 0.0; // continuous_PI;
	float dif;
	bool continuous_satr = false; // torque angle saturation
    while(flag_continuous_bending)
    {
        usleep(16666); // 60 Hz
        continuous_set = false;
        continuous_done = false;

        if (continuous_go)
        {
            set_feeding_distance( fabs(continuous_central) * M_PI/180.0 * continuous_radius );
            set_feeding_speed ( feeding_distance/continuous_time );
            DistanceGone = 0.0;
            // continuous_field_s = field_angle_m;
            //continuous_field_s = m_a;
            if (!flag_auto_feeding)
                init_auto_feeding();
            feeding_done = false;
            set_feeding_go(1);
            usleep(10000);

            while(!feeding_done & flag_continuous_bending) // continuous bending
            {
                mangle_c = m_a;
                continuous_magnet = anglePlus( continuous_start, continuous_central * DistanceGone/feeding_distance );
                destination_angle = continuous_magnet;
                // continuous_field = anglePlus( continuous_field_s, continuous_central * DistanceGone/feeding_distance );
                continuous_field = continuous_magnet;

                if (continuous_feedback)
                {
                    if (!continuous_set)
                    {
                        continuous_P = 0;
                        continuous_I = 0;
                        continuous_satr = false;
                        continuous_done = false;
                        continuous_set = true;
                    }

                    dif = angleMinus(continuous_magnet, mangle_c);

                    if (!continuous_satr)
                    {
                        continuous_P = kp/80.0 * dif;
                        continuous_I = ki/200.0 * dif/60.0 + continuous_I;
                        continuous_PI = continuous_P + continuous_I;
                    }

                    temp_continuous_field = anglePlus(continuous_field, continuous_PI);

                    if ( angleMinus(temp_continuous_field, mangle_c) > 90 ) // torque angle < 90 degrees
                    {
                        continuous_field = anglePlus(mangle_c, 90);
                        continuous_satr = true;
                    }
                    else if ( angleMinus(temp_continuous_field, mangle_c) < -90 )
                    {
                        continuous_field = anglePlus(mangle_c, -90);
                        continuous_satr = true;
                    }
                    else
                    {
                        // continuous_field = anglePlus(continuous_field, temp_continuous_field);
                        continuous_field = temp_continuous_field;
                        continuous_satr = false;
                    }
                }
                set_field_polar (14.0, magnet2field_angle(continuous_field));
            }

            set_feeding_go(0);

            if (continuous_feedback) // feedback bending for the ending point
            {
                set_destination_angle(continuous_magnet);
                if (!flag_discrete_bending)
                    init_discrete_bending();
                set_discrete_go(1);
                usleep(50000); // time to renew discrete history and error and to set discrete_done to false
                while(!discrete_done & flag_continuous_bending) {}
                continuous_done = true;
                set_discrete_go(0);
            }
            usleep(50000); // time for automatic fabrication to read continuous_done = true before it's set to false
            set_continuous_go(0);
        }
    }
    coilCurrentClear();
    printf("--continuous bending thread ended\n");
    return NULL;
}

//sinusoidal field (2D) thread
void* sinusoidal_field_thread (void*threadid)
{
    printf("--sinusoidal field (2D) thread started--\n");

    struct timeval start;
    double time_current, time_last, time_elapsed, time_out, time_out_sum, v_x, v_y, v_z; //time in second
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds.
    while( flag_sinusoidal_field )
    {
        if ( sinusoidalfield_go )
        {
            gettimeofday(&start, NULL);
            time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_elapsed = time_last - time_initial -time_out;
            v_x = fab_amp_s/2.0 * cos( 2.0 * M_PI * fab_fre_s * time_elapsed ) * cosd(fab_heading_s) + fab_amp_s/2.0 * cosd(fab_heading_s);
            v_y = fab_amp_s/2.0 * cos( 2.0 * M_PI * fab_fre_s * time_elapsed ) * sind(fab_heading_s) + fab_amp_s/2.0 * sind(fab_heading_s);
            set_field_xyz( 0, v_x );
            set_field_xyz( 2, v_y );
            time_out_sum = time_out;
        }
        else
        {
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_out = time_current - time_last + time_out_sum;
        }
    }
    coilCurrentClear();
    printf("--rotational field (vertical) thread ended--\n");
    return NULL;
}

//rotational field (vertical) thread
void* rotational_field_thread (void*threadid)
{
    printf("--rotational field (vertical) thread started--\n");

    struct timeval start;
    double time_current, time_last, time_elapsed, time_out, time_out_sum, v_x, v_y, v_z; //time in second
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds.
    while( flag_rotational_field )
    {
        if ( rotationalfield_go )
        {
            gettimeofday(&start, NULL);
            time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_elapsed = time_last - time_initial -time_out;
            v_x = fab_amp * cos( -2.0 * M_PI * fab_fre * time_elapsed ) * cosd(fab_heading);
            v_y = fab_amp * cos( -2.0 * M_PI * fab_fre * time_elapsed ) * sind(fab_heading);
            v_z = fab_amp * sin( -2.0 * M_PI * fab_fre * time_elapsed );

            set_field_xyz( 0, v_x );
            set_field_xyz( 1, v_y );
            set_field_xyz( 2, v_z );
            time_out_sum = time_out;
        }
        else
        {
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_out = time_current - time_last + time_out_sum;
        }
    }
    coilCurrentClear();
    printf("--rotational field (vertical) thread ended--\n");
    return NULL;
}

//rotational field (horizontal) thread
void* rotational_field_h_thread (void*threadid)
{
    printf("--rotational field (horizontal) thread started--\n");

    struct timeval start;
    double time_current, time_last, time_elapsed, time_out, time_out_sum, v_x, v_y, v_z; //time in second
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds.
    while( flag_rotational_field_h )
    {
        if ( rotationalfield_go_h )
        {
            gettimeofday(&start, NULL);
            time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_elapsed = time_last - time_initial -time_out;
            v_x = fab_amp * cos( fab_heading_h * -2.0 * M_PI * fab_fre * time_elapsed );
            v_y = fab_amp * sin( fab_heading_h * -2.0 * M_PI * fab_fre * time_elapsed );

            set_field_xyz( 0, v_x );
            set_field_xyz( 1, v_y );
            set_field_xyz( 2, 0.0 );
            time_out_sum = time_out;
        }
        else
        {
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_out = time_current - time_last + time_out_sum;
        }
    }
    coilCurrentClear();
    printf("--rotational field (horizontal) thread ended--\n");
    return NULL;
}
// sawtooth mode thread
void* sawtooth_mode_thread (void*threadid)
{
    printf("--sawtooth mode thread started\n");
    float v_x, v_y, v_z;
    float slope; // this is the direction of the lifting field
    float slope_re = 1.0; // this is the angle resolution of the lifting field direction
    float slope_sign = 1.0;
    while (flag_sawtooth_mode)
    {
        if (sawtooth_go)
        {
            int sleep_t = (1/fab_fre)/(fabs(peak_angle_st/slope_re))*1000000;
            if (peak_angle_st>0) slope_sign = 1.0; else slope_sign = -1.0;
            for (slope=0.0; slope <= fabs(peak_angle_st); slope = slope+slope_re) //lift the field gradually
            {
                v_x = fab_amp * cosd(slope*slope_sign) * cosd(fab_heading);
                v_y = fab_amp * cosd(slope*slope_sign) * sind(fab_heading);
                v_z = fab_amp * sind(slope*slope_sign);
                set_field_xyz( 0, v_x );
                set_field_xyz( 1, v_y );
                set_field_xyz( 2, v_z );
                usleep(sleep_t);
            }
            set_field_xyz( 0, fab_amp * cosd(fab_heading) ); // always lay the filed down after it reaches peak angle
            set_field_xyz( 1, fab_amp * sind(fab_heading) );
            set_field_xyz( 2, 0.0 );
        }
    }
    coilCurrentClear();
    printf("--sawtooth mode thread ended--\n");
    return NULL;
}

// temperature control thread
void* temp_control_thread (void*threadid)
{
    printf("--temperature control thread started\n");
    double V[16];
    float error_temp;
    float control_P_temp = 0.0, control_I_temp = 0.0, aoV = 0.0; //********changed here
    float start_point = 0.0, aoV_satr = 0.3; //this is the maximum input of the temperature control driver //was 0.22, changed for water
	float inst_temp, temp_history[6];
	float temp_sum;
	int temp_counter = 0;

    uint aiChan = (1<<15);
    uint airangecode = 1;
    s826_aiInit(aiChan, airangecode);
    // FILE *heating=fopen("heating_temeprature.txt","w");

    while(flag_temp_control)
    {
        s826_aiRead(aiChan, V);

		temp_sum = 0.0;
		for (int i=0; i<5; i++) // update the historic temperature
        {
            temp_history[i] = temp_history[i+1];
			temp_sum = temp_sum + temp_history[i];
        }
        inst_temp = (V[15]-1.2654)/0.0051;
        temp_history[5] = inst_temp;
		temp_sum = temp_sum + temp_history[5];

		temp_counter++;
		if (temp_counter == 6) // display frequency: 10 Hz
        {
            temp_counter = 0;
            current_temp = temp_sum/6.0; // current temperature is the average of last 6 data
        }

        if ( temp_go )
        {
            if ( inst_temp < (destination_temp-2) )
            {
                aoV = aoV_satr;
				temp_done = false;
            }
            else if ( inst_temp > (destination_temp+2) )
            {
                aoV = 0.0;
				temp_done = false;
            }
            else
            {
                temp_done = true;
                error_temp = destination_temp - inst_temp;
                control_P_temp = temp_kp * error_temp;
                control_I_temp = temp_ki * error_temp + control_I_temp;
                start_point = destination_temp /400.0;
                aoV = start_point + control_P_temp + control_I_temp;
                if (aoV > aoV_satr) aoV = aoV_satr;
                if (aoV < 0.0) aoV = 0.0;
            }

            s826_aoPin(3, 0, aoV);

            // printf(" T = %.2f, ", current_temp);
            // printf("V = %.3f\n", aoV);usleep(200000);
            // fprintf(heating, "%.4f\n", current_temp); //write the current temperature into file
        }
        else
        {
			temp_done = false;
            aoV = 0.0;
            s826_aoPin(3, 0, 0.0);
            // printf(" T = %.2f\n", current_temp);
            // fprintf(magnetangle, "%.4f\n", current_temp); //write the magnet angle into file

        }
        usleep(16666); // 60 Hz
        //waitUsePeriodicTimer(1e5); //1e5:10hz  5e4:20hz  2e4:50hz 1e4:100hz

    }
    s826_aoPin(3, 0, 0.0);
    // fclose(heating);
    printf("--temperature control thread ended\n");
    return NULL;
}

// set stepper stepping mode (1,2,3,4,5 for full,1/2,1/4,1/8, and 1/16 division)
void setStep(int mode) {
  switch (mode) {
    case 1:
      s826_doPin(MS1, 0);
      s826_doPin(MS2, 0);
      s826_doPin(MS3, 0);
      break;
    case 2:
      s826_doPin(MS1, 1);
      s826_doPin(MS2, 0);
      s826_doPin(MS3, 0);
      break;
    case 3:
      s826_doPin(MS1, 0);
      s826_doPin(MS2, 1);
      s826_doPin(MS3, 0);
      break;
    case 4:
      s826_doPin(MS1, 1);
      s826_doPin(MS2, 1);
      s826_doPin(MS3, 0);
      break;
    case 5:
      s826_doPin(MS1, 1);
      s826_doPin(MS2, 1);
      s826_doPin(MS3, 1);
      break;
    default:
      s826_doPin(MS1, 0);
      s826_doPin(MS2, 0);
      s826_doPin(MS3, 0);
      break;
  }
  m=mode-1;
}

void manualOverride ()
{
    Stepper1.setMaxSpeed(10/fabs(res[m])); //default

    while (flag_manual_feeding){
        if(flag_increment)
        {
            Stepper1.move(-feeding_increments/res[m]);
            flag_increment = false;
        }else if(flag_decrement)
        {
            Stepper1.move(feeding_increments/res[m]);
            flag_decrement = false;
        }
        Stepper1.run();
        if(!flag_setSpeed)
        {
            Stepper1.setMaxSpeed(feeding_speed/fabs(res[m]));
            flag_setSpeed = true;
        }
    }
}

// automatic feeding thread
void* auto_feeding_thread (void*threadid)
{
    printf("--automatic feeding thread started\n");
    Stepper1.setAcceleration(90000000);
    while(flag_auto_feeding)
    {
        if (!initSetup) { //Initial Motor Setup

            setStep(stepSize); //Set mode from UI
            //printf("->Step Size Set to: %d\n",stepSize);

            if (!stepper_on) { //stepper enabling check
                s826_doPin(enablePin,0); //low to activate driver
                printf("---Stepper engaged\n");
                stepper_on=true;
            }
            Stepper1.setMaxSpeed(feeding_speed/fabs(res[m]));
            //printf("->Stepper speed set to: %.1f mm/s\n",feeding_speed);
            initSetup=true;
        }

        if (feeding_go)
        {
            if(runCalled) {
                    //Stepper1.setSpeed(feeding_speed/fabs(res[m]));
                    Stepper1.setMaxSpeed(feeding_speed/fabs(res[m]));
                    Stepper1.move(-feeding_distance/res[m]);

                    runCalled = false;
                    feeding_done = false;
                    //printf("False.\n");
            }
            Stepper1.run();
            //Stepper1.runSpeed();
            DistanceGone = feeding_distance - fabs(res[m]) * (double) (abs(Stepper1.distanceToGo()));
            //printf("%f\n", DistanceGone);
            if (Stepper1.distanceToGo() == 0 && !feeding_done)
            {
                feeding_done = true;
                feeding_go   = false;
                //printf("True.\n");
            }

        }else{

            //printf("distance to go: %.2f\n",fabs(res[m])*Stepper1.distanceToGo());
            runCalled=true;
            if (flag_manual_feeding)
            {
                manualOverride();
            }
        }
    }
    s826_doPin(enablePin,1); //disengage stepper
    stepper_on=false; //flag
    printf("---stepper disengaged\n");
    initSetup=false; //flag
    runCalled=true;
    printf("--automatic feeding thread ended\n");
    return NULL;
}

// timing thread
void* timing_thread (void*threadid)
{
	printf("--timing thread started\n");
    struct timeval start;
    gettimeofday(&start, NULL);
    int start_sec = start.tv_sec; // Initial time in seconds.
	double start_sec_f = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds(double).
    int time_sec;
    int time_min;
	// double time_sec_f; this definition is global
	while ( flag_timing )
	{
	    gettimeofday(&start, NULL);
        time_sec = (start.tv_sec - start_sec)%60; // time in seconds.
        time_min = (start.tv_sec - start_sec)/60; // time in minutes;
		time_sec_f = (double) start.tv_sec + start.tv_usec*1e-6 - start_sec_f;
        sprintf(fab_time, "%02d:%02d", time_min, time_sec);
	}
    printf("--timing thread ended\n");
    return NULL;
}

// automatic fabrication thread
void* auto_fab_thread (void*threadid)
{
	printf("-automatic fabrication thread started\n");
	bool flagggg; // flag if the thread is terminated or finished
	float length_a, angle_b, radius_c, angle_c, this_angle = 0.0;
	float temp_cool = 43.0, temp_heat = 62.0; // (43, 62)  (55, 115)
	float feeding_v = 0.4;
	float length_now, total_length = 0;
	float period_rec = 50000; // period for writing record into text file

	// start related threads
	if (!flag_discrete_bending)
        init_discrete_bending();
	if (!flag_continuous_bending)
        init_continuous_bending();
    if (!flag_temp_control)
        init_temp_control();
    if (!flag_auto_feeding)
        init_auto_feeding();

	// start reading the shape
    char filename[] = "microrobot_shape.txt";
	FILE *file = fopen(filename, "r");
	char *line = NULL;
	size_t len = 0;
	ssize_t read; // number of characters read
	if (file == NULL)
		printf("Cannot open file: %s\n", filename);
	// file to record the fabrication
	FILE *fab_record = fopen("microrobot_shape_fab_record.txt","w");

    // write time and shape information into record
    time_t currTime;
    struct tm *localTime;
    currTime = time(NULL);
    fprintf(fab_record, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    fprintf(fab_record, " %s", ctime(&currTime));
    while (((read = getline(&line, &len, file)) != -1) & flag_auto_fab) // write shape information into record
    {
        line[read-1] = 0;
		fprintf(fab_record, " %s", line);
    }
    fprintf(fab_record, "\n");
    fprintf(fab_record, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
	fprintf(fab_record, "%12s%12s%12s%12s%12s%12s%12s%12s\n", "length", "time", "angle(tem)", "angle(ave)", "angle(des)", "C_PI", "field dir", "temperature");
	fclose(file); // close and reopen shape file
    fopen(filename, "r");

	// starting timing
	if (!flag_timing)
		init_timing();

	while (((getline(&line, &len, file)) != -1) & flag_auto_fab)
    {
		printf("%s", line);

		switch (line[0])
		{
			case 'a': // a: cooling down, and feeding
			if ( current_temp > temp_cool )
			{
				set_destination_temp(temp_cool);
				set_temp_go(1);
				sprintf(fab_status, "Cooling down....");
				usleep(period_rec);
				while(!temp_done & flag_auto_fab)
				{
					fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
					usleep(period_rec);
				}
			}
			getline(&line, &len, file);
			length_a = atof(line); // get the length
			length_now = total_length;
			set_feeding_speed(feeding_v);
			set_feeding_distance(length_a);
			set_feeding_go(1);
			sprintf(fab_status, "Feeding....");
			usleep(period_rec);
			while(!feeding_done & flag_auto_fab)
			{
				total_length = length_now + DistanceGone;
				fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
				usleep(period_rec);
			}
			set_feeding_go(0);
			break;

			case 'b': // b: heating up, and discrete bending
            set_destination_temp(temp_heat);
			set_temp_go(1);
			sprintf(fab_status, "Heating up....");
			usleep(period_rec);
			while(!temp_done & flag_auto_fab)
			{
				fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
				usleep(period_rec);
			}
			getline(&line, &len, file);
			angle_b = atof(line); // get the angle
			this_angle = anglePlus(this_angle, angle_b);
			set_destination_angle(this_angle);
			set_discrete_go(1);
			sprintf(fab_status, "Discrete bending....");
			usleep(period_rec);
			while(!discrete_done & flag_auto_fab)
			{
				fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
				usleep(period_rec);
			}
			set_discrete_go(0);
			break;

			case 'c': // c: heating up, and continuous bending
			set_destination_temp(temp_heat);
			set_temp_go(1);
			sprintf(fab_status, "Heating up....");
			usleep(period_rec);
			while(!temp_done & flag_auto_fab)
			{
				fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
				usleep(period_rec);
			}
			getline(&line, &len, file);
			radius_c = atof(line); // get the radius
			getline(&line, &len, file);
			angle_c = atof(line); // get the central angle
			length_now = total_length;
			set_continuous_feedback(1);
			set_continuous_start(this_angle);
			set_continuous_radius(radius_c);
			set_continuous_central(angle_c);
			set_continuous_time(fabs(continuous_central)/5.0); // angular velocity: 5 degrees/s
			set_continuous_go(1);
			sprintf(fab_status, "Continuous bending....");
			usleep(period_rec*2);
			while(!continuous_done & flag_auto_fab)
			{
				total_length = length_now + DistanceGone;
				fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
				usleep(period_rec);
			}
			set_continuous_go(0);
			this_angle = anglePlus(this_angle, angle_c);
			break;
		}
	}
    // cool down before well done
    if ( current_temp > temp_cool )
    {
        set_destination_temp(temp_cool);
        set_temp_go(1);
        sprintf(fab_status, "Cooling down....");
        usleep(period_rec);
        while(!temp_done & flag_auto_fab)
        {
            fprintf(fab_record, "%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f%12.2f\n", total_length, time_sec_f, m_a_temp, m_a, destination_angle, continuous_PI, field_angle_m, current_temp);
            usleep(period_rec);
        }
    }

	free(line);
	fclose(file);
	fclose(fab_record);
    this_angle = 0.0;

    // is it well done or terminated?
    if (flag_auto_fab)
	{
		sprintf(fab_status, "Well done!");
		flag_auto_fab = false;
	}
	else
	{
        while (((getline(&line, &len, file)) != -1)) {}; // read all the left lines
		printf("%s", line);
		sprintf(fab_status, "Fabrication terminated!");
	}
    stop_auto_feeding();
	stop_discrete_bending();
	stop_continuous_bending();
	stop_timing();

    printf("-automatic fabrication thread ended\n");
}

// cut off thread
void* cut_off_thread (void*threadid)
{
	printf("--cutting-off thread started\n");
	double quater_float, quater_p, phase, v_a, slope;
	int quater;
    struct timeval start;
    double time_last, time_elapsed;
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in second
	while ( flag_cut_off )
	{
        // neutral_cut  mag_cut  fre_cut
        gettimeofday(&start, NULL);
        time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;
        time_elapsed = time_last - time_initial; // elapsed time in second

        quater_p = 0.25/fabs(fre_cut);
        slope = (mag_cut/2.0) / quater_p;
        quater_float = time_elapsed / quater_p;
        quater = int(quater_float) % 4;
        phase = (quater_float - int(quater_float)) * quater_p;

        switch (quater)
        {
        case 0:
            v_a = slope*phase;
            break;
        case 1:
            v_a = mag_cut/2.0 - slope*phase;
            break;
        case 2:
            v_a = -slope*phase;
            break;
        case 3:
            v_a = -mag_cut/2.0 + slope*phase;
            break;
        }

        if (fre_cut>=0)
            v_a = neutral_cut + v_a - 90.0;
        else
            v_a = neutral_cut - v_a - 90.0;

        set_field_polar (14.0, v_a);
	}
    printf("--cutting-off thread ended\n");
}







































// discrete bending thread related functions
void init_discrete_bending(void) // initialize magnet feedback control thread
{
    //printf("@ the Beginning of discrete_bending_thread.\n");
    flag_discrete_bending = true;
	pthread_t discrete_bending;
    pthread_create(&discrete_bending, NULL, discrete_bending_thread, NULL);  // magnet feedback control thread
}

void stop_discrete_bending(void) // stop magnet feedback control thread
{
	flag_discrete_bending = false;
}

void set_kp(float d)
{
	kp = d; //for setting kp, the gain of Proportional
}

void set_ki(float d)
{
	ki = d; //for setting ki, the gain of Integral
}

void set_destination_angle(float d)
{
	destination_angle = d; // for setting the destination angle of the magnet
}

double magnet2field_angle(double anglein) // map an angle from magnet angle coordinate to field angle angle coordinate
{
    double angleout;
    if ( anglein <= -90)
        angleout = anglein + 270.0;
    else
        angleout = anglein - 90.0;

    return angleout;
}

double field2magnet_angle(double anglein) // map an angle from field angle coordinate to magnet angle angle coordinate
{
    double angleout;
    if ( anglein <= 90)
        angleout = anglein + 90.0;
    else
        angleout = anglein - 270.0;

    return angleout;
}

float anglePlus( float a, float b) // rotate angle by angle b => angle c, wrap c in (-pi, pi]
{
    float c = a + b;
    while ( c > 180 )   { c = c - 360.0; }
    while ( c <= -180 ) { c = c + 360.0; }
    return c;
}

float angleMinus( float c, float a) // rotate angle by angle b => angle c, wrap b in (-pi, pi]
{
    float b = c - a;
    while ( b > 180 )   { b = b - 360.0; }
    while ( b <= -180 ) { b = b + 360.0; }
    return b;
}

void set_discrete_go(int d)
{
    discrete_go = d;
}

// continuous bending thread related functions
void init_continuous_bending(void) // initialize continuous bending thread
{
    //printf("@ the Beginning of continuous_bending_thread.\n");
    flag_continuous_bending = true;
	pthread_t continuous_bending;
    pthread_create(&continuous_bending, NULL, continuous_bending_thread, NULL);
}

void stop_continuous_bending(void) // stop continuous bending thread
{
	flag_continuous_bending = false;
}

void set_continuous_radius(float d)
{
    continuous_radius = d;
}

void set_continuous_central(float d)
{
    continuous_central = d;
}

void set_continuous_start(float d)
{
    continuous_start = d;
}

void set_continuous_time(float d)
{
    continuous_time = d;
}

void set_continuous_feedback(int d)
{
    continuous_feedback = d;
}

void set_continuous_go(int d)
{
    continuous_go = d;
}

// sinusoidal field (2D) thread related functions
void init_sinusoidal_field(void) // initiate sinusoidal field thread
{
    //printf("@ the Beginning of sinusoidal_field_thread.\n");
    flag_sinusoidal_field = true;
	pthread_t sinusoidal_field;
    pthread_create(&sinusoidal_field, NULL, sinusoidal_field_thread, NULL);  //start swimmer thread
}

void stop_sinusoidal_field(void) // stop rotational field thread
{
	flag_sinusoidal_field = false;
}

void set_fab_heading_s(float d)
{
	fab_heading_s = d;
}

void set_fab_amp_s(float d) // set amplitude of the sinusoidal field
{
	fab_amp_s = d;
}

void set_fab_fre_s(float d) // set frequency of the sinusoidal field
{
	fab_fre_s = d;
}

void set_sinusoidalfield_go(int d)
{
    sinusoidalfield_go = d;
}

// rotational field (vertical) thread related functions
void init_rotational_field(void) // initiate rotational field thread
{
    //printf("@ the Beginning of rotational_field_thread.\n");
    flag_rotational_field = true;
	pthread_t rotational_field;
    pthread_create(&rotational_field, NULL, rotational_field_thread, NULL);  //start swimmer thread
}

void stop_rotational_field(void) // stop rotational field thread
{
	flag_rotational_field = false;
}

void set_fab_heading(float d)
{
	fab_heading = d;
	if ( (flag_sawtooth_mode & !sawtooth_go) || (flag_rotational_field & !rotationalfield_go))
    {
        float v_x = fab_amp * cosd(fab_heading);
        float v_y = fab_amp * sind(fab_heading);
        set_field_xyz( 0, v_x );
        set_field_xyz( 1, v_y );
    }
}


void set_fab_amp(float d) // set amplitude of the rotational field
{
	fab_amp = d;
    if ( (flag_sawtooth_mode & !sawtooth_go) || (flag_rotational_field & !rotationalfield_go) || (flag_rotational_field_h & !rotationalfield_go_h) )
    {
        float v_x = fab_amp * cosd(fab_heading);
        float v_y = fab_amp * sind(fab_heading);
        set_field_xyz( 0, v_x );
        set_field_xyz( 1, v_y );
    }
}

void set_fab_fre(float d) // set frequency of the rotational field
{
	fab_fre = d;
}

void set_rotationalfield_go(int d)
{
    rotationalfield_go = d;
}

// rotational field (horizontal) thread related functions
void init_rotational_field_h(void) // initiate rotational field thread
{
    //printf("@ the Beginning of rotational_field_thread.\n");
    flag_rotational_field_h = true;
	pthread_t rotational_field_h;
    pthread_create(&rotational_field_h, NULL, rotational_field_h_thread, NULL);  //start swimmer thread
}

void stop_rotational_field_h(void) // stop rotational field thread
{
	flag_rotational_field_h = false;
}

void set_fab_heading_h(int d)
{
	if (d ==1)
        fab_heading_h = 1;
    else
        fab_heading_h = -1;
}

void set_rotationalfield_go_h(int d)
{
    rotationalfield_go_h = d;
}

// sawtooth mode thread related functions
void init_sawtooth_mode(void) // initiate sawtooth mode thread
{
    //printf("@ the Beginning of sawtooth_mode_thread.\n");
    flag_sawtooth_mode = true;
	pthread_t sawtooth_mode;
    pthread_create(&sawtooth_mode, NULL, sawtooth_mode_thread, NULL);
}

void stop_sawtooth_mode(void) // stop sawtooth mode thread
{
	flag_sawtooth_mode = false;
}

void set_sawtooth_go(int d)
{
    sawtooth_go = d;
}

void set_peak_angle_st(float d)
{
    peak_angle_st = d;
}

// temperature control thread related functions

void init_temp_control(void) // initialize temperature control thread
{
    //printf("@ the Beginning of temp_control_thread.\n");
    flag_temp_control = true;
	pthread_t temp_control;
    pthread_create(&temp_control, NULL, temp_control_thread, NULL);  //start thread
}

void stop_temp_control(void) // stop temperature control thread
{
    flag_temp_control = false;
}

void set_destination_temp( float d ) // set destination temperature
{
    destination_temp = d;
}

void set_temp_go( int d ) // temperature go/stop
{
    temp_go = d;
}

float get_current_temp(void)
{
    return current_temp;
}

// automatic feeding thread related functions
void init_auto_feeding( void ) // initiate automatic feeding thread
{
    //printf("@ the Beginning of automatic_feeding_thread.\n");
    flag_auto_feeding = true;
	pthread_t auto_feeding;
    pthread_create(&auto_feeding, NULL, auto_feeding_thread, NULL);  //start thread
}

void stop_auto_feeding( void ) // stop automatic feeding thread
{
    flag_auto_feeding = false;
}

void set_feeding_distance ( float d ) // set feeding distance
{
    feeding_distance = d;
}

void set_feeding_speed ( float d ) // set feeding speed
{
    feeding_speed = d;
    flag_setSpeed = false;
}

void set_feeding_go ( int d ) //set feeding_go flag
{
    feeding_go = d;
}

void set_feeding_increments ( float d ) //set feeding increments (manual override)
{
    feeding_increments = d;
}

void init_manual_feeding( void ) // initiates manual feeding
{
    flag_manual_feeding = true;
}

void stop_manual_feeding( void ) // stops manual feeding
{
    flag_manual_feeding = false;
}

void feederIncrement ( void ) // handles incrementing the feeder
{
    flag_increment = true;
    //printf("feederIncrement Inside!");
}

void feederDecrement ( void ) // handles decrementing the feeder
{
    flag_decrement = true;
    //printf("feederDecrement Inside!");
}

// timing thread related functions
void init_timing( void ) // initiate timing thread
{
    //printf("@ the Beginning of timing_thread.\n");
    flag_timing = true;
	pthread_t timing;
    pthread_create(&timing, NULL, timing_thread, NULL);  //start thread
}

void stop_timing( void ) // stop timing thread
{
    flag_timing = false;
}

// automatic fabrication thread related functions

void set_left_constraint (int d)
{
    left_constraint = d;
}

void init_auto_fab( void ) // initiate automatic fabrication thread
{
    //printf("@ the Beginning of automatic_fabrication_thread.\n");
    flag_auto_fab = true;
	pthread_t auto_fab;
    pthread_create(&auto_fab, NULL, auto_fab_thread, NULL);  //start thread
}

void stop_auto_fab( void ) // stop automatic fabrication thread
{
    flag_auto_fab = false;
}

// cut off thread related functions
void set_neutral_cut( float d )
{
    neutral_cut = d;
}

void set_mag_cut( float d)
{
    mag_cut = d;
}

void set_fre_cut( float d )
{
    fre_cut = d;
}

void init_cut_off (void)
{
    flag_cut_off = true;
	pthread_t cut_off;
    pthread_create(&cut_off, NULL, cut_off_thread, NULL);
}

void stop_cut_off (void)
{
    flag_cut_off = false;
}

// field control
void set_factor (int index)
{
	switch (index) // 1:3D coil system; 0:2D coil system
	{
		case 0: coil_3d = true;
                factor_x = 5.0964; factor_y = 4.999; factor_z = 5.1677;
                printf("3D coil system selected\n");
				break;
		case 1: coil_3d = false; coil_2d_xz = true;
                factor_x = 5.2264; factor_z = 5.8538;
                printf("2D X-Z selected\n");
				break;
        case 2: coil_3d = false; coil_2d_xz = false;
                factor_x = 5.2264; factor_y = 5.8538;
                printf("2D X-Y selected\n");
				break;
	}
}

void set_field_xyz (int index, float d) // indices 123 -> xyz. for 3d, controller 012 -> xyz; for 2d, controller 0,1 -> x,(y or z)
{
    switch (index) // 0:x  1:y  2:z
    {
        case 0: field_x = d; set_coil_current_to (0, d);
                break;

        case 1: if ( !coil_3d & coil_2d_xz )
                    break;
                else
                {
                    field_y = d;
                    set_coil_current_to (1, d);
                    break;
                }

        case 2: if ( !coil_3d & !coil_2d_xz )
                    break;
                else
                {
                    field_z = d;
                    if (coil_3d)
                        set_coil_current_to (2, d);
                    else
                        set_coil_current_to (1, d);
                    break;
                }
    }
	field_mag = sqrt( pow(field_x,2) + pow(field_y,2) + pow(field_z,2) ) ;
	field_angle = atan2(field_z, field_x) * 180.0/M_PI; // atan2() => (-pi, pi]
	field_angle_m = field2magnet_angle(field_angle);
}



void set_field_mag_fab (float d)
{
    float field_mag_old = field_mag_fab;
    field_mag_fab = d;
    set_field_xyz( 0, field_x * d / field_mag_old );
    set_field_xyz( 1, field_y * d / field_mag_old );
    set_field_xyz( 2, field_z * d / field_mag_old );
}

void set_field_polar (float magnitude, float angle)
{
    set_field_xyz( 0, magnitude * cos(angle * M_PI/180.0) );
    set_field_xyz( 2, magnitude * sin(angle * M_PI/180.0) );

    if (left_constraint)
        if ( m_a > -135 & m_a <= 45 )
            set_field_xyz(1, 1);
        if ( m_a <= -135  || m_a >45 )
            set_field_xyz(1, -1);
    else
        set_field_xyz(1, 0.0);
}
