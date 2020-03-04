#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "ezcolor.h"

int main(void)
{
    cy_rslt_t result;
    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();
    init_colors();
    for(;;)
    {
        color( RED );
        Cy_SysLib_Delay( 1000 );
        color( GREEN );
        Cy_SysLib_Delay( 1000 );
        color( BLUE );
        Cy_SysLib_Delay( 1000 );
        color( YELLOW );
        Cy_SysLib_Delay( 1000 );
        color( CYAN );
        Cy_SysLib_Delay( 1000 );
        color( MAGENTA );
        Cy_SysLib_Delay( 1000 );
        color( WHITE );
        Cy_SysLib_Delay( 1000 );
        color( BLACK );
        Cy_SysLib_Delay( 1000 );
    }
}
