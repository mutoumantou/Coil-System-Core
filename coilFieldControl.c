// (08-15)
#include "coilFieldControl.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//static float coil_current_x = 0, coil_current_y = 0, coil_current_z = 0;
static float coil_current_voltage[3] = {0,0,0};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Coil Current Clear
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int coilCurrentClear(void)
{
    coil_current_voltage[0] = 0;
    coil_current_voltage[1] = 0;
    coil_current_voltage[2] = 0;
    //update_coil_current();

	uint aoRange = 2;   // 2: -5 ~ +5 V.
	s826_aoPin( 0 , aoRange, 0);
	s826_aoPin( 1 , aoRange, 0);
	s826_aoPin( 2 , aoRange, 0);

}

int set_coil_current_to (int index, float d)
{
    //printf("Inside set_coil_current_to.\n");
    coil_current_voltage[index] = d;
    switch (index)
    {
        case 0:
            s826_aoPin(0, 2, d/5.003);    // coil 1.0    x-left
            s826_aoPin(3, 2, d/4.879);    // coil 1.1    x-right
            break;
        case 1:
            s826_aoPin(1, 2, d/5.024);    // coil 2.0     y-left
            s826_aoPin(5, 2, d/4.433);    //  coil 3      y-right
            break;
        case 2:
            s826_aoPin(4, 2, d/5.143);    // coil 2.1     z-serial
            break;

    }
    //update_coil_current();
}

void set_field_xyz_2 (float bx, float by, float bz, float dbx, float dby, float dbz)
{

    s826_aoPin(3, 2, bx*0.18754719386756); //x-right
	s826_aoPin(0, 2, bx*0.18754719386756); //x-left

	s826_aoPin(1, 2, by*-0.19230420653328); //y-left
	s826_aoPin(5, 2, by*-0.19230420653328); //y-right

	s826_aoPin(4, 2, bz*-0.191929507962791); //z-serial

/*
    s826_aoPin(3, 2, bx*0.230689628282348); //x-right
	s826_aoPin(0, 2, bx*0.206143248520095); //x-left

	s826_aoPin(1, 2, by*-0.206687961346871); //y-left
	s826_aoPin(5, 2, by*-0.253911487039249); //y-right

	s826_aoPin(4, 2, bz*-0.212559426393856); //z-serial
	*/

	/*s826_aoPin(3, 2, bx*0.228728766441948); //x-right
	s826_aoPin(0, 2, bx*0.206534920692283); //x-left

	s826_aoPin(1, 2, by*-0.208233987297746); //y-left
	s826_aoPin(5, 2, by*-0.251806560811694); //y-right

	s826_aoPin(4, 2, bz*-0.208720603153183); //z-serial
*/
	/*
    s826_aoPin(3, 2, bx*0.234167936507937); //x-right
    s826_aoPin(0, 2, bx*0.200563996135069); //x-left

s826_aoPin(1, 2, by*-0.204410811290959); //y-left
s826_aoPin(5, 2, by*-0.254599095571096); //y-right

s826_aoPin(4, 2, bz*-0.215700220122625); //z-serial
*/
/*
    s826_aoPin(3, 2, bx*-0.197684572); //x-right
    s826_aoPin(0, 2, bx*-0.197194521); //x-left

    s826_aoPin(1, 2, by*0.207109034); //y-left
    s826_aoPin(5, 2, by*0.218446024); //y-right

    s826_aoPin(4, 2, bz*0.214600149); //z-serial
    */
}

void resetCoils(void){

    set_field_xyz_2(0.0,0.0,0.0,0.0,0.0,0.0);

}

/* Get current coil current control voltage value */
float get_coil_current(int index)
{
    float returnValue = 0;

    if ( (index > 2) || (index < 0) )
        printf("Error in get_coil_current.\n");
    else
        returnValue = coil_current_voltage[index];

    return returnValue;
}


