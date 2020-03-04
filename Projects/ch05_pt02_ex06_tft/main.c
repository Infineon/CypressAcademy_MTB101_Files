#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "GUI.h"

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    GUI_Init();
       GUI_SetColor(GUI_WHITE);
       GUI_SetBkColor(GUI_BLACK);
       GUI_SetFont(GUI_FONT_32B_1);
       GUI_SetTextAlign(GUI_TA_CENTER);
       /* Change this text as appropriate */
       GUI_DispStringAt("I feel good!", GUI_GetScreenSizeX()/2,GUI_GetScreenSizeY()/2 - GUI_GetFontSizeY()/2);

    for(;;)
    {
    }
}
