/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
*
* Graphic Display Support demonstration.
*
* To demonstrate the app, work through the following steps.
* 1. Connect display which supports MIPI_DBI_C specification to appropriate pins
*    [see comments in wiced_bt_display_app_start() for two example HW pin configuration.]
* 2. Plug the WICED eval board into your computer
* 3. In makefile.mk chose driver appropriate for your display (by default Freetronic 128x128 driver is chosen)
*    Check that DISPLAY and DISPLAY_ONLY flag is enabled.
*    When DISPLAY_ONLY flag is enabled, part of UART HW will be used to send commands to display
*    and traces will be redirected to puart (TX on LHL P32).
* 4. Build and download the application (to the WICED board)
* 5. Application will go through different screens demonstrating performance of the device using bitmaps or vector graphic.

*
*/

#include <stdlib.h>
#include <stdio.h>
#include <wchar.h>
#if TOOLCHAIN_wiced
    #include "u8g.h"
    #include "rtc.h"
    #include "brcm_fw_types.h"
    #define WICED_BT_TRACE(...)                 wiced_printf(NULL, 0, __VA_ARGS__)
#else
    #include "foundation/lib/graphic/u8g.h"
    #include "foundation/hal/gpio.h"
    #include "foundation/regmaps/regmaps.h"
    #include "foundation/hal/pmu/pmu_idle.h"
    #include "mpaf/drivers/rtc.h"
    #include "mpaf/drivers/hidddriversconfig.h"
    #include "foundation/api/clock_timer.h"
    #define WICED_BT_TRACE(...)
#endif

#include "bitmaps/clouds.h"
#include "bitmaps/moon_v6.h"
#include "bitmaps/watchface.h"
#include "bitmaps/wiced_logo.h"
#include "bitmaps/cy_logo.h"
#include "bitmaps/watch_hand_coordinates.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"

#define NO_TRANSCOLOR 0x10000000

extern int __2sprintf(char * str, const char * format, ...);
/*
 * This is u8g extension function to support multicolor translucent native bitmaps
 * As performance of this function is important it expects bitmap to be in native screen resolution
 * No conversion from 3 bytes to two bytes etc is supported.
 */
void u8g_TransNatBitmap( u8g_t *u8g_ptr, int x, int y, int width, int height, unsigned char *bitmap, int transColor)
{ // transparent native bitmap (has to match screen color depth)
    if ( u8g_IsBBXIntersection(u8g_ptr, x, y, width, height) == 0 )
        return;
    else
    {
        int i=0, j=0, depth=  U8G_MODE_GET_BITS_PER_PIXEL(u8g_GetMode(u8g_ptr))/8;
        u8g_pb_t *pb = (u8g_pb_t *)(u8g_ptr->dev->dev_mem);
        unsigned char * trans_ptr= (unsigned char *)&transColor;

        for (i = 0; i < height && (i+pb->p.page_y0)<=pb->p.page_y1 && (i+pb->p.page_y0) < y+height; i++)
        {
            if (pb->p.page_y0+i < y)
                continue;
            for (j = 0; j < width*depth; j++)
            {
                if (transColor != NO_TRANSCOLOR)
                {
                    if (!(j%depth))
                    {
                        int cnt=0, trans=1;
                        for (cnt=0; cnt < depth; cnt++)
                        {
                            if (bitmap[((i+pb->p.page_y0)-y)*width*depth+j+cnt]!= *(trans_ptr+cnt))
                            {
                                trans=0;
                            }
                        }
                        if (trans)
                        {
                            j+=depth-1;
                            continue;
                        }
                    }
                }
                ((char *)(pb->buf))[i*pb->width*depth+x*depth+j]= bitmap[((i+pb->p.page_y0)-y)*width*depth+j];
            }
        }
    }

}


int display_loop(IN void* ignore); // forward declaration

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
#ifdef DISPLAY_ONLY
void application_start(void)
#else
void wiced_bt_display_app_start(void)
#endif
{

#ifdef DISPLAY_ONLY
#ifdef WICED_BT_TRACE_ENABLE
    //Set the debug uart to enable the debug traces
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#endif //WICED_BT_TRACE_ENABLE
#endif
    WICED_BT_TRACE("wiced_bt_display_app_start \n");
    u8g_MutexCreate();
    u8g_RegisterIdleLoopHandler(display_loop);
    u8g_ScreenRefresh();
}

/*
 * Conversion function to deal with nonstandard order of RGB in SD1351 controller
 */
void u8g_SetSD1351HiColorByRGB(u8g_t *u8g, uint8_t r, uint8_t g, uint8_t b)
{
    u8g_SetRGB(u8g, b,g,r); // for SDS1351 controller R and B have to be swapped
}

#define BCG_COLOR u8g_SetSD1351HiColorByRGB(u8g_p, 100,100,100)
#define BCG_ACTIVE u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0)
#define FG_COLOR u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255)
#define SET_RBG(rw) if(row == rw) BCG_ACTIVE; else BCG_COLOR;


/*
 * Fill background color very fast, which is crucial for performance.
 * Call it to clear background before you draw new figures on it.
 * Do not use if you copy full image using u8g_BackgroundBitmap(), as it will replace all pixels anyway.
 * @ val16 - color value we use to set the pixel
 */
void display_backgroundFill16(u8g_t *u8g, UINT16 val16)
{
    u8g_pb_t * b= (u8g_pb_t *)(u8g->dev->dev_mem);
    UINT32 *ptr = (UINT32 *) (b->buf);

    // wait for SPI transmission to finish.
    u8g_WaitForPageDone();

    if (((unsigned int)ptr & 0x1) != 0)
    {// if buffer not aligned, do not proceed
        WICED_BT_TRACE("Display buffer has to be word aligned. Could not fill background\n");
        return;
    }

    wmemset((wchar_t *)ptr, (wchar_t)val16, b->p.page_height*b->width);
}

#ifdef POP_UP_SIMUL
char *slow_moon= "This screen is fully refreshed";
char *fast_moon= " Only refreshing necessary part of scr";
char *stocks=    "Vector graphic -only status refreshed";
char *sample_msg="Cypress - Embedded in Tomorrow";
#endif
volatile char *pop_up_str=NULL;

// the standard function is not linked in
//and we do not want to add full library just for msg simulation
#define strchr strchr_sim
char *strchr_sim(const char *str, int chr)
{
    while (*str)
    {
        if (*str == chr)
        {
            return(char *)str;
        }
        str++;
    }
    return NULL;
}

/*
 * Can be called from application (used by watch app) to set string displayed in pop up
 * Main loop of display demo will display pop up if string is not empty.
 */
void display_set_pop_up_str(char *str)
{
#ifndef POP_UP_SIMUL
    char* p_str_buf;

    u8g_MutexGet();
    if (pop_up_str) wiced_bt_free_buffer((void *)pop_up_str);
    p_str_buf= ( char* )wiced_bt_get_buffer( strlen(str)+1 );
    if (p_str_buf)
    {
        pop_up_str=p_str_buf;
        while (*str)
            *p_str_buf++= *str++;
        *p_str_buf=0;
    }
    u8g_MutexPut();
#endif
}




/*
 * Below are variables and two functions supporting pop up screens on display
 */
char pop_up_fsm=0;
char pop_up_line=0;
char pop_up_down=0;
char *last_space=0;
/*
 * This function is called by main display loop when pop_up_str is not empty
 * If new message is assigned to pop_up_str it will scroll up from the bottom
 * of the screen and stay there until canceled by calling  display_pop_down().
 * As this is just demo function, it support only message up to two lines long.
 * The longer strings will be cut by HW.
 */
void display_pop_up(u8g_t* u8g_p, char *msg)
{
    char *space;
    static char more_lines=0;
    int base;
    u8g_MutexGet();
    if (!pop_up_fsm)
    {
        pop_up_fsm=1;
        last_space=0;
        pop_up_line=0;
        space= strchr(msg, ' ');
        while (space && (space - msg) < 21) // 20 characters max in line
        {
            last_space=space;
            space= strchr(last_space+1, ' ');
        }

        if ((space ? (space - msg): (last_space-msg)) < strlen(msg))
        {
            if (last_space)
            {
                *last_space=0; // this is not perfect solution but for demo we can do it
                // more appropriate would be to have own copy of the string
                // before modifying it, but in this case it would put a limit
                // on storing length or make code more complicated
            }
            more_lines=1;   // in this demo version we support only up to two lines of msg
        }
        else
            more_lines=0;
    }
    base = u8g_GetHeight(u8g_p)- (pop_up_line/2+1);
    u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);
    u8g_DrawRBox(u8g_p,0,base,128,13,4);
    u8g_SetSD1351HiColorByRGB(u8g_p, 200,0,0);
    u8g_DrawRBox(u8g_p,0,base+2,128,26,5); // min 13 high
    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
    u8g_SetFont(u8g_p, u8g_font_6x12r);
    u8g_DrawStr(u8g_p, 1, base+13, (const char*)msg);
    if (more_lines)
    {
        u8g_DrawStr(u8g_p, 1, base+22, (const char*)last_space+1);

    }
    if (pop_up_line < (more_lines ? 48: 36))
    {
        pop_up_line++;
    }
    else
        pop_up_fsm=2;
    u8g_MutexPut();
}

/*
 * This function release copy of the message and reset pop up variables
 * to be ready for next message.
 */
void display_pop_down(void)
{
    u8g_MutexGet();
    if (pop_up_str)
    {
        pop_up_fsm=0;
#ifndef POP_UP_SIMUL
        wiced_bt_free_buffer((void *)pop_up_str);
#endif
        pop_up_str= NULL;
        pop_up_down=1;
    }
    u8g_MutexPut();
}


/*
 * Initial stage of display FSM - set up display HW and driver
 */
int display_hw_init(u8g_t *u8g_p, char *time, int stage)
{
    rtcConfig.oscillatorFrequencykHz= 32;
    rtc_init();

#ifdef NHD
    u8g_InitSPI_cy(u8g_p, &u8g_dev_ssd1351_128x96_hicolor_65k_hw_spi, 10, 28, 2, 26, 255); // last argument 255 means no reset line
#else
    u8g_InitSPI_cy(u8g_p, &u8g_dev_ssd1351_128x128_hicolor_65k_hw_spi, 10, 28, 2, 26, 255);
#endif
    return(++stage);
}

/*
 * Displays demo screen with background bitmap and transparent bitmap moving above it.
 * First part of demo refresh the whole screen, second part only refresh changing part
 * of it to show difference in performance.
 */
int display_bitmap(u8g_t *u8g_p, char *time, int stage)
{
    static int w=0, moon_pos_x=0;
    static char bitmap_fsm=0;

    w++;
    switch (bitmap_fsm)
    {
        case 0:
            {
                int i;

                u8g_FirstPage(u8g_p);

                for (i=0; i<2; i++)
                {
                    display_backgroundFill16(u8g_p, 0x0000);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                    u8g_SetFont(u8g_p, u8g_font_04b_03r);
                    u8g_DrawStr(u8g_p, 0, 6, "Photos");
                    u8g_DrawStr(u8g_p, 80, 6, time);
                    u8g_NextPage(u8g_p);
                }

                do
                {
                    u8g_BackgroundBitmap(u8g_p, clouds, sizeof(clouds));

                    // wait for SPI transmission to finish.
                    u8g_WaitForPageDone();

                    u8g_TransNatBitmap( u8g_p, moon_pos_x, 9, 20, 20, moon20, 0x0000);
                    if (pop_up_str) display_pop_up(u8g_p, (char*)pop_up_str);
                }while (u8g_NextPage(u8g_p));

                moon_pos_x+=1;
                if (moon_pos_x>108)
                {
                    moon_pos_x=0;
                }
#ifdef POP_UP_SIMUL
                if (w == 50)
                    pop_up_str= slow_moon;
                else
                    if (w == 140)
                {
                    display_pop_down();
                    *last_space= ' '; // use only if you own the string and want to reuse it,
                                      // writing to the string already freed could be very bad

                }
#endif
                if (w>300)
                {
                    bitmap_fsm++;
                    w=0;
                }
                break;
            }

        case 1:
            {
                static int i;
                u8g_FirstPage(u8g_p);

                for (i=0; i<2; i++)
                {
                    display_backgroundFill16(u8g_p, 0x0000);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                    u8g_SetFont(u8g_p, u8g_font_04b_03r);
                    u8g_DrawStr(u8g_p, 0, 6, "Photos");
                    u8g_DrawStr(u8g_p, 80, 6, time);
                    u8g_NextPage(u8g_p);
                }

                for (i=3; i<8; i++)
                {
                    u8g_BackgroundBitmap(u8g_p, clouds, sizeof(clouds));

                    // wait for SPI transmission to finish.
                    u8g_WaitForPageDone();

                    u8g_TransNatBitmap( u8g_p, moon_pos_x, 9, 20, 20, moon20, 0x0000);
                    u8g_NextPage(u8g_p);
                }

                moon_pos_x+=1;
                if (moon_pos_x>108)
                {
                    moon_pos_x=0;
                }
#ifdef POP_UP_SIMUL
                if (w == 120)
                    pop_up_str= fast_moon;
                else
                    if (w == 700)
                {
                    display_pop_down();
                    *last_space= ' '; // use only if you own the string and want to reuse it,
                                      // writing to the string already freed could be very bad

                }
#endif
                if ((pop_up_str && pop_up_fsm < 2) || pop_up_down)
                {
                    u8g_SetNextPage(u8g_p, u8g_GetHeight(u8g_p)-26);
                    do
                    {
                        u8g_BackgroundBitmap(u8g_p, clouds, sizeof(clouds));

                        // wait for SPI transmission to finish.
                        u8g_WaitForPageDone();

                        if (pop_up_str) display_pop_up(u8g_p, (char*)pop_up_str);
                    }while (u8g_NextPage(u8g_p));
                    pop_up_down=0;
                }
                if (w>1200)
                {
                    stage++;
                    bitmap_fsm= w= moon_pos_x= 0;
                }
                break;
            }
    }
    return stage;
}

/*
 * Displays demo screen with stock tickers.
 * Once tickers are loaded on the screen only status line with time is refreshed.
 * If pop up screen is loaded, part behind the dynamically extending pop up is also refreshed
 */
int display_stocks(u8g_t *u8g_p, char *time, int stage)
{
    static int w=0;
    static char stocks_fsm=0;

    w++;
    switch (stocks_fsm)
    {
        case 0:
            // STOCK TICKERS initial
            u8g_FirstPage(u8g_p);
            do
            {
                int base;
                display_backgroundFill16(u8g_p, 0x0000);
                // clear part of screen for top tekst
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);
                u8g_DrawBox(u8g_p,0,0,128,10);

                u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                u8g_SetFont(u8g_p, u8g_font_04b_03r);
                u8g_DrawStr(u8g_p, 0, 6, "Stocks");
                u8g_DrawStr(u8g_p, 80, 6, time);

                base=10;
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                u8g_SetSD1351HiColorByRGB(u8g_p, 160,170,160);
                u8g_DrawStr(u8g_p, 2, base+7, "NASDAQ");
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,255,0);
                u8g_DrawStr(u8g_p, 90, base+7, "+0.88%");
                u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                u8g_SetFont(u8g_p, u8g_font_6x12r);
                u8g_DrawStr(u8g_p, 10, base+20, " 4,767.59");

                base=38;
                u8g_SetFont(u8g_p, u8g_font_04b_03r);
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                u8g_SetSD1351HiColorByRGB(u8g_p, 160,170,160);
                u8g_DrawStr(u8g_p, 2, base+7, "DOW");
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,255,0);
                u8g_DrawStr(u8g_p, 90, base+7, "+0.30%");
                u8g_SetFont(u8g_p, u8g_font_6x12r);
                u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                u8g_DrawStr(u8g_p, 10, base+20, "18,200.32");

                base=66;
                u8g_SetFont(u8g_p, u8g_font_04b_03r);
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                u8g_SetSD1351HiColorByRGB(u8g_p, 160,170,160);
                u8g_DrawStr(u8g_p, 2, base+7, "S&P500");
                u8g_SetSD1351HiColorByRGB(u8g_p, 0,255,0);
                u8g_DrawStr(u8g_p, 90, base+7, "+2.34%");
                u8g_SetFont(u8g_p, u8g_font_6x12r);
                u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                u8g_DrawStr(u8g_p, 10, base+20, " 1,105.99");

                if (u8g_GetHeight(u8g_p)>96)
                {
                    base=94;
                    u8g_SetFont(u8g_p, u8g_font_04b_03r);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                    u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                    u8g_SetSD1351HiColorByRGB(u8g_p, 160,170,160);
                    u8g_DrawStr(u8g_p, 2, base+7, "NYSE");
                    u8g_SetSD1351HiColorByRGB(u8g_p, 0,255,0);
                    u8g_DrawStr(u8g_p, 90, base+7, "+1.24%");
                    u8g_SetFont(u8g_p, u8g_font_6x12r);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                    u8g_DrawStr(u8g_p, 10, base+20, "11,057.34");

                    base=122;
                    u8g_SetSD1351HiColorByRGB(u8g_p, 0,75,0);
                    u8g_DrawRBox(u8g_p,0,base,128,14,5); // min 13 high
                }
                if (pop_up_str) display_pop_up(u8g_p, (char*)pop_up_str);
            }while (u8g_NextPage(u8g_p));
                stocks_fsm++;
            break;
        case 1:
            {
                int i;
                // STOCK TICKERS loop
                u8g_SetFont(u8g_p, u8g_font_04b_03r);
                u8g_FirstPage(u8g_p);
                for (i=0; i<2; i++)
                {
                    display_backgroundFill16(u8g_p, 0x0000);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                    u8g_DrawStr(u8g_p, 0, 6, "Stocks");
                    u8g_DrawStr(u8g_p, 80, 6, time);
                    // first line is completely contained on first 3 pages
                    u8g_NextPage(u8g_p);
                }
                // do not refresh the rest (as it is not changing) except when msg is active
#ifdef POP_UP_SIMUL
                if (w == 300)
                {
                    pop_up_str= stocks;
                }
                else
                    if (w == 2000)
                {
                    display_pop_down();
                    *last_space= ' '; // use only if you own the string and want to reuse it,
                                      // writing to the string already freed could be very bad
                }
#endif
                if ((pop_up_str && pop_up_fsm < 2) || pop_up_down)
                {
                    int base;
                    u8g_SetNextPage(u8g_p, u8g_GetHeight(u8g_p)-26);
                    do
                    {
                        if (u8g_GetHeight(u8g_p)>96)
                        {
                            base=94;
                            u8g_SetFont(u8g_p, u8g_font_04b_03r);
                            u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);
                            u8g_DrawBox(u8g_p,0,base+20,128,5); // min 13 high
                            u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                            u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                            u8g_SetFont(u8g_p, u8g_font_6x12r);
                            u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                            u8g_DrawStr(u8g_p, 10, base+20, "11,057.34");

                            base=122;
                            u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);
                            u8g_DrawBox(u8g_p,0,base-2,128,7); // min 13 high
                            u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                            u8g_DrawRBox(u8g_p,0,base,128,14,5); // min 13 high
                        }
                        else
                        {
                            base=66;
                            u8g_SetFont(u8g_p, u8g_font_04b_03r);
                            u8g_SetSD1351HiColorByRGB(u8g_p, 0,95,0);
                            u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                            u8g_SetSD1351HiColorByRGB(u8g_p, 160,170,160);
                            u8g_DrawStr(u8g_p, 2, base+7, "S&P500");
                            u8g_SetSD1351HiColorByRGB(u8g_p, 0,255,0);
                            u8g_DrawStr(u8g_p, 90, base+7, "+2.34%");
                            u8g_SetFont(u8g_p, u8g_font_6x12r);
                            u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                            u8g_DrawStr(u8g_p, 10, base+20, " 1,105.99");

                        }
                        if (pop_up_str) display_pop_up(u8g_p, (char*)pop_up_str);
                    } while (u8g_NextPage(u8g_p));
                    pop_up_down=0;
                }
#if CONTRAST_DEMO
                {
                static int contrast=0;
                // dimming of the screen (value of 0 will switch display off for Freetronics display)
#ifdef POP_UP_SIMUL
                	if (w==2000)
#else
              		if (w==1000)
#endif
					{
              			contrast=2000;
					}
					if(contrast)
					{
						if(!(contrast%200))
						{
							u8g_SetContrast(u8g_p, contrast/200);
						}
						contrast--;
					}
					else
					{
#ifdef POP_UP_SIMUL
                	if (w==4002)
#else
              		if (w==3002)
#endif
              			u8g_SetContrast(u8g_p, 0);
#ifdef POP_UP_SIMUL
                	if (w==4500)
#else
              		if (w==3500)
#endif
                        u8g_SetContrast(u8g_p, 0xC); // standard contrast for Freetronics display
					}
#ifdef POP_UP_SIMUL
                	if (w>5000)
#else
              		if (w>4000)
#endif
					{
                        stage++;
                        stocks_fsm= w= 0;
					}
                }
#else
#ifdef POP_UP_SIMUL
                if (w>3000)
#else
                if (w>2000)
#endif
                {
                    stage++;
                    stocks_fsm= w= 0;
                }
#endif
                break;
            }
    }
    return stage;
}

/*
 * Bitmaps below are used in menu screen
 */
const u8g_pgm_uint8_t bitmap_bt[] =
{
    /* 00010000 */ 0x010,
    /* 10011000 */ 0x098,
    /* 01010100 */ 0x054,
    /* 00111000 */ 0x038,
    /* 00010000 */ 0x010,
    /* 00111000 */ 0x038,
    /* 01010100 */ 0x054,
    /* 10011000 */ 0x098,
    /* 00010000 */ 0x010,
};

const u8g_pgm_uint8_t bitmap_air[] =
{
    /* 00100000 */ 0x020,
    /* 00010000 */ 0x010,
    /* 10001000 */ 0x088,
    /* 11111111 */ 0x0FF,
    /* 10001000 */ 0x088,
    /* 00010000 */ 0x010,
    /* 00100000 */ 0x020,
};

const u8g_pgm_uint8_t bitmap_clock_A[] =
{
    /* 0011000000000000 */ 0x30, 0x00,
    /* 0011100000000000 */ 0x38, 0x00,
    /* 0001110000000000 */ 0x1C, 0x00,
    /* 0000111000000000 */ 0x0E, 0x00,
    /* 0000011111111100 */ 0x07, 0xFC,
    /* 0000001111111100 */ 0x03, 0xFC,
};

const u8g_pgm_uint8_t bitmap_clock_B[] =
{
    /* 000001000 */ 0x008,
    /* 000001000 */ 0x008,
    /* 000001000 */ 0x008,
    /* 000001000 */ 0x008,
    /* 000001000 */ 0x008,
    /* 000001000 */ 0x008,
};

const u8g_pgm_uint8_t bitmap_moon[] =
{
    /* 0000001100000000 */ 0x03, 0x00,
    /* 0000011100000000 */ 0x07, 0x00,
    /* 0000111000000000 */ 0x0E, 0x00,
    /* 0001111000000000 */ 0x1E, 0x00,
    /* 0001111000000010 */ 0x1E, 0x02,
    /* 0001111100001110 */ 0x1F, 0x0E,
    /* 0001111111111100 */ 0x0F, 0xFC,
    /* 0000111111111000 */ 0x0F, 0xF8,
    /* 0000011111100000 */ 0x07, 0xE0,
};

/*
 * Structures below are used in menu screen.
 * It shows how parameters of elements on the screen can be tabulated
 * instead of repeating the code as it is done with tickers screen.
 */
static char *row_str[4]={
    "Bluetooth",
    "Airplane Mode",
    "Do Not Disturb",
    "Time"
};
static int bases[4]={12, 40, 68, 96};
static const struct
{
    const u8g_pgm_uint8_t *bitmap;
    unsigned char x;
    unsigned char y_off;
    unsigned char bytes;
    unsigned char rows;
    unsigned char bg_r;
    unsigned char bg_b;
    unsigned char bg_g;
}bitmaps[4]={
    bitmap_bt,12,9,1,9,0,0,255,
    bitmap_air,11,10,1,7,255,128,0,
    bitmap_moon,6,8,2,9, 128,128,255,
    bitmap_clock_A,8,8,2,6,0,0,0
};

/*
 * Displays demo screen with scrolling menu.
 * Only dynamically changed lines are refreshed (including po up area).
 * Performance of this movement is so high, that scrolling of the menu is slowed down
 *  and slowly increased to avoid impression of flickering screen.
 */
int display_menu(u8g_t *u8g_p, char *time, int stage)
{
    static int w=0;
    static char menu_fsm=0;

    w++;
    switch (menu_fsm)
    {
        case 0:
            {
                int i, row=0;

                // MENU initial page setting
                u8g_FirstPage(u8g_p);
                for (i=0; i<3; i++)
                {
                    // clear screen below changing time
                    display_backgroundFill16(u8g_p, 0x0000);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                    u8g_SetFont(u8g_p, u8g_font_04b_03r);
                    u8g_DrawStr(u8g_p, 0, 8, "Settings");
                    u8g_DrawStr(u8g_p, 80, 8, time);
                    SET_RBG(0)
                    u8g_DrawRBox(u8g_p,0,12,128,26,5); // min 13 high
                    u8g_NextPage(u8g_p);
                }
                u8g_SetFont(u8g_p, u8g_font_6x12r);

                for (i=0; i<7; i++)//28 lines per menu item, page has 4 lines
                {
                    // clear screen
                    display_backgroundFill16(u8g_p, 0x0000);
                    BCG_ACTIVE;
                    u8g_DrawRBox(u8g_p,0,bases[0],128,26,5); // min 13 high
                    u8g_SetSD1351HiColorByRGB(u8g_p, bitmaps[0].bg_r,bitmaps[0].bg_b,bitmaps[0].bg_g);
                    u8g_DrawRBox(u8g_p,6, bases[0]+4,18,18,5); // min 13 high
                    FG_COLOR;
                    u8g_DrawBitmap(u8g_p, bitmaps[0].x, bases[0]+bitmaps[0].y_off, bitmaps[0].bytes, bitmaps[0].rows, bitmaps[0].bitmap);
                    u8g_DrawStr(u8g_p, 30, bases[0]+16, row_str[0]);
                    u8g_NextPage(u8g_p);
                }
                for (row=1; row <4; row++)
                {
                    for (i=0; i<7; i++)//28 lines per menu item, page has 4 lines
                    {
                        // clear screen
                        display_backgroundFill16(u8g_p, 0x0000);
                        BCG_COLOR;
                        u8g_DrawRBox(u8g_p,0,bases[row],128,26,5); // min 13 high
                        u8g_SetSD1351HiColorByRGB(u8g_p, bitmaps[row].bg_r,bitmaps[row].bg_b,bitmaps[row].bg_g);
                        u8g_DrawRBox(u8g_p,6, bases[row]+4,18,18,5); // min 13 high
                        FG_COLOR;
                        u8g_DrawBitmap(u8g_p, bitmaps[row].x, bases[row]+bitmaps[row].y_off, bitmaps[row].bytes, bitmaps[row].rows, bitmaps[row].bitmap);
                        u8g_DrawStr(u8g_p, 30, bases[row]+16, row_str[row]);
                        if(!u8g_NextPage(u8g_p))
                        { // for 128x96 NHD screen we cannot display last row
                            row=3;
                            break;
                        }
                    }
                }
                if (u8g_GetHeight(u8g_p)>96)
                    do
                    {
                        // clear screen
                        display_backgroundFill16(u8g_p, 0x0000);
                        u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);
                        BCG_COLOR;
                        u8g_DrawRBox(u8g_p,0,124,128,14,5); // min 13 high
                    }while (u8g_NextPage(u8g_p));

                menu_fsm++;
                break;
            }

        case 1:
            {
                int i;
                static int row=1;
                static int last_row=0;
                int pop_up_done=0;

                // MENU dynamic part
                u8g_FirstPage(u8g_p);
                for (i=0; i<3; i++)
                {// status line is NOT completely contained on first page
                    // clear screen below changing time
                    display_backgroundFill16(u8g_p, 0x0000);
                    u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255);
                    u8g_SetFont(u8g_p, u8g_font_04b_03r);
                    u8g_DrawStr(u8g_p, 0, 6, "Settings");
                    u8g_DrawStr(u8g_p, 80, 6, time);
                    SET_RBG(0)
                    u8g_DrawRBox(u8g_p,0,12,128,26,5); // min 13 high
                    u8g_NextPage(u8g_p);
                }
                u8g_SetFont(u8g_p, u8g_font_6x12r);
                if (last_row)
                { // only if there is gap between header and row, we have to jump pages
                    u8g_SetNextPage(u8g_p, bases[last_row]);
                }
                for (i=0; i<7; i++)//28 bit rows per menu item, page has 4 rows
                {
                    // clear screen
                    display_backgroundFill16(u8g_p, 0x0000);
                    BCG_COLOR;
                    u8g_DrawRBox(u8g_p,0,bases[last_row],128,26,5); // min 13 high
                    u8g_SetSD1351HiColorByRGB(u8g_p, bitmaps[last_row].bg_r,bitmaps[last_row].bg_b,bitmaps[last_row].bg_g);
                    u8g_DrawRBox(u8g_p,6, bases[last_row]+4,18,18,5); // min 13 high
                    FG_COLOR;
                    u8g_DrawBitmap(u8g_p, bitmaps[last_row].x, bases[last_row]+bitmaps[last_row].y_off, bitmaps[last_row].bytes, bitmaps[last_row].rows, bitmaps[last_row].bitmap);
                    u8g_DrawStr(u8g_p, 30, bases[last_row]+16, row_str[last_row]);

                    if (pop_up_str && (u8g_GetHeight(u8g_p)>96 ? last_row == 3: last_row == 2))
                    {
                        display_pop_up(u8g_p, (char*)pop_up_str);
                        pop_up_done=1;
                    }
                    u8g_NextPage(u8g_p);
                }

                if (row != last_row+1)
                {
                    u8g_SetNextPage(u8g_p, bases[row]);
                }

                for (i=0; i<7; i++)//28 bit rows per menu item, page has 4 rows
                {
                    // clear screen
                    display_backgroundFill16(u8g_p, 0x0000);
                    BCG_ACTIVE;
                    u8g_DrawRBox(u8g_p,0,bases[row],128,26,5); // min 13 high
                    u8g_SetSD1351HiColorByRGB(u8g_p, bitmaps[row].bg_r,bitmaps[row].bg_b,bitmaps[row].bg_g);
                    u8g_DrawRBox(u8g_p,6, bases[row]+4,18,18,5); // min 13 high
                    FG_COLOR;
                    u8g_DrawBitmap(u8g_p, bitmaps[row].x, bases[row]+bitmaps[row].y_off, bitmaps[row].bytes, bitmaps[row].rows, bitmaps[row].bitmap);
                    u8g_DrawStr(u8g_p, 30, bases[row]+16, row_str[row]);

                    if (pop_up_str && (u8g_GetHeight(u8g_p)>96 ? row == 3: row == 2))
                    {
                        display_pop_up(u8g_p, (char*)pop_up_str);
                        pop_up_done=1;
                    }
                    u8g_NextPage(u8g_p);
                }

                if (w>90 || (w>30 && !(w%3)) || !(w%8))
                {
                    last_row=row;
                    row++;
                }
                if (u8g_GetHeight(u8g_p)>96)
                    row%=4;
                else
                    row%=3;
                if (pop_up_str)
                {
                    if (pop_up_done)
                    {
                        if (u8g_GetHeight(u8g_p)>96)
                        { // still need to refresh 4 bit rows after last menu item
                            u8g_SetNextPage(u8g_p, u8g_GetHeight(u8g_p)-4);
                            pop_up_done=0;
                        }
                    }
                    else
                    {
                        u8g_SetNextPage(u8g_p, u8g_GetHeight(u8g_p)-26);
                    }
                    if (!pop_up_done)
                    {
                        int base;
                        do
                        {
                            if (u8g_GetHeight(u8g_p)>96)
                            {
                                base=96;
                                SET_RBG(3)
                                u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                                FG_COLOR;
                                u8g_DrawRBox(u8g_p,6,base+4,18,18,5); // min 13 high
                                u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);

                                u8g_DrawBitmap(u8g_p, 8, base+8, 2, 6, bitmap_clock_A);
                                u8g_SetSD1351HiColorByRGB(u8g_p, 204,0,102);
                                u8g_DrawBitmap(u8g_p, 10, base+14, 1, 6, bitmap_clock_B);
                                FG_COLOR;
                                u8g_DrawStr(u8g_p, 30, base+16, "Time");

                                base=124;
                                SET_RBG(4)
                                u8g_DrawRBox(u8g_p,0,base,128,14,5); // min 13 high
                            }
                            else
                            {
                                base=68;
                                SET_RBG(2)
                                u8g_DrawRBox(u8g_p,0,base,128,26,5); // min 13 high
                                u8g_SetSD1351HiColorByRGB(u8g_p, 90,70,100);
                                u8g_DrawRBox(u8g_p,6,base+4,18,18,5); // min 13 high
                                FG_COLOR;
                                u8g_DrawBitmap(u8g_p, 6, base+8, 2, 9, bitmap_moon);
                                FG_COLOR;
                                u8g_DrawStr(u8g_p, 30, base+16, "Do Not Disturb");
                            }
                            display_pop_up(u8g_p, (char*)pop_up_str);
                        } while (u8g_NextPage(u8g_p));
                    }
                }

#ifdef POP_UP_SIMUL
                if (w == 30)
                {
                    pop_up_str= sample_msg;
                }
                else
                    if (w == 120)
                {
                    display_pop_down();
                    *last_space= ' '; // use only if you own the string and want to reuse it,
                                      // writing to the string already freed could be very bad
                }
#endif
                if (pop_up_down)
                {
                    if (u8g_GetHeight(u8g_p)>96)
                    {
                        u8g_SetNextPage(u8g_p, 124);
                        u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,0);
                        u8g_DrawBox(u8g_p,0,124,128,5); // min 13 high
                        SET_RBG(4)
                        u8g_DrawRBox(u8g_p,0,124,128,14,5); // min 13 high
                        u8g_NextPage(u8g_p);
                    }
                    pop_up_down=0;
                }
                if (w == 160)
                {
                    stage++;
                    menu_fsm= w= 0;
                    row=1;
                    last_row=0;

                }
                break;
            }
    }
    return stage;
}


/*
 * Converts digital RTC readout to analog watch coordinates.
 * Displyas corresponding time and watchface
 * Optimization only refreshes half the screen that contains second hand
 * Every one minute, (at 0 sec), entire screen refreshes to accomodate new minute hand location
 */
uint8_t display_analog_watch(u8g_t *u8g_p, RtcTime rtcTime, int stage)
{

    uint8_t last_page = 0;
    static uint8_t counter = 0;
    static uint8_t tog = 0;
    uint8_t center_x = 64;
    uint8_t center_y;

#ifdef NHD
    center_y = 48;
#else
    center_y = 64;
#endif

#ifndef NHD
    /*optimization of screen update area*/
    if(rtcTime.second == 0 || counter == 0)
    {
        u8g_FirstPage(u8g_p);
        last_page = 32;
    }
    else if (rtcTime.second < 15 || rtcTime.second > 45 )
    {
		u8g_FirstPage(u8g_p);
		last_page = 18;
    }
    else
    {
        u8g_SetNextPage(u8g_p, 58);
        last_page = 15;
    }
#else
    u8g_FirstPage(u8g_p);
#endif

    do
    {
        display_backgroundFill16(u8g_p, 0x0000);
        u8g_SetFont(u8g_p, u8g_font_6x12r);
        u8g_SetSD1351HiColorByRGB(u8g_p, 255,255,255); /*black background*/

        /*draw watchface*/
#ifndef NHD
        u8g_DrawBitmap( u8g_p, 0, 0, 16, 128, watchface); /*watchface for 128x128*/
#else
        u8g_DrawBitmap( u8g_p, 1, 0, 16, 96, watchface96); /*watchface for 128x96*/
#endif

        u8g_SetSD1351HiColorByRGB(u8g_p, 0,0,255); /*blue*/

        u8g_DrawBitmap( u8g_p, 120, 5, 1, 9, bitmap_bt);/*BT logo*/

        if(tog)
        {
        	u8g_DrawBitmap( u8g_p, 26, center_y - 29, 10, 20, cy_logo); /*cypress logo*/
        }
        else
        {
        	u8g_DrawBitmap( u8g_p, 35, center_y - 38, 7, 38, wiced_logo); /*wiced logo*/
        }

        u8g_DrawStr(u8g_p, 32, center_y + 20, "Embedded in"); /*print slogan*/
        u8g_DrawStr(u8g_p, 40, center_y + 28, "Tomorrow");

        {/*hour hand*/
            uint8_t hhand = 0;
            uint8_t x = 0;
            uint8_t y = 0;
            uint8_t l_h_hand = 0;
            uint8_t r_h_hand = 0;

            if (rtcTime.hour > 11)
            {
            	hhand = rtcTime.hour - 12;
            }
            else
            {
            	hhand = rtcTime.hour;
            }

            hhand *= 5;

            //hhand += 10; /*at init, offset hhand with mhand, will not show real time, for demo*/

            if (rtcTime.minute >  47)
            {
            	hhand += 4;
            }
            else if (rtcTime.minute >  35)
            {
            	hhand += 3;
            }
            else if (rtcTime.minute >  23)
            {
            	hhand += 2;
            }
            else if(rtcTime.minute >  11)
            {
            	hhand += 1;
            }

            u8g_SetSD1351HiColorByRGB(u8g_p, 170,170,170); /*med gray*/

            u8g_DrawLine(u8g_p, center_x, center_y, (hour_x[59-hhand]), (hour_y[59-hhand]));

            x = hour_x[59-hhand];
            y = hour_y[59-hhand];

            if(hhand > 14 && hhand < 45)
            {
                r_h_hand = hhand + 15;
                l_h_hand = hhand - 15;
            }
            else if (hhand < 15)
            {
                r_h_hand = hhand + 15;
                l_h_hand = (59 - (15 - hhand));
            }
            else if (hhand > 44)
            {
                r_h_hand = (15 - (59 - hhand));
                l_h_hand = hhand - 15;
            }

#ifndef NHD

            u8g_DrawTriangle(
            					u8g_p,
								rad3_x[ 59 - (l_h_hand) ],
								rad3_y[ 59 - (l_h_hand) ],
								rad3_x[ 59 - (r_h_hand) ],
								rad3_y[ 59 - (r_h_hand) ],
								x,
								y
							);

#else
            u8g_DrawTriangle(
            					u8g_p,
								rad3_x[ 59 - (l_h_hand) ],
								rad3_y[ 59 - (l_h_hand) ] - 16,
								rad3_x[ 59 - (r_h_hand) ],
								rad3_y[ 59 - (r_h_hand) ] - 16,
								x,
								y
							);
#endif

        }

        {/*minute hand*/

            uint8_t mhand = rtcTime.minute;

            uint8_t r_m_hand = 0;
            uint8_t l_m_hand = 0;

            uint8_t x = min_x[59-rtcTime.minute];
            uint8_t y = min_y[59-rtcTime.minute];

            u8g_SetSD1351HiColorByRGB(u8g_p, 170,170,170); /*medium gray*/

            u8g_DrawLine(u8g_p, center_x, center_y, min_x[59-rtcTime.minute], min_y[59-rtcTime.minute]);

            if(mhand > 14 && mhand < 45)
            {
                r_m_hand = mhand + 15;
                l_m_hand = mhand - 15;
            }
            else if (mhand < 15)
            {
                r_m_hand = mhand + 15;
                l_m_hand = (59 - (15 - mhand));
            }
            else if (mhand > 44)
            {
                r_m_hand = (15 - (59 - mhand));
                l_m_hand = mhand - 15;
            }

#ifndef NHD
            u8g_DrawTriangle(   u8g_p,
                                rad2_x[ 59 - (l_m_hand) ],
								rad2_y[ 59 - (l_m_hand) ],
								rad2_x[ 59 - (r_m_hand) ],
								rad2_y[ 59 - (r_m_hand) ],
								x,
								y
							);
#else
            u8g_DrawTriangle(   u8g_p,
								rad2_x[ 59 - (l_m_hand) ],
								rad2_y[ 59 - (l_m_hand) ] - 16,
								rad2_x[ 59 - (r_m_hand) ],
								rad2_y[ 59 - (r_m_hand) ] - 16,
								x,
								y
							);
#endif

        }

        /*second hand*/
        u8g_SetSD1351HiColorByRGB(u8g_p, 0, 0, 255); /*set color blue*/

        u8g_DrawLine(	u8g_p,
        				center_x,
						center_y,
						(min_x[59-rtcTime.second]),
						(min_y[59-(rtcTime.second)])
        			);/*draw line through*/

        u8g_DrawCircle( u8g_p, center_x, center_y, 2, (U8G_DRAW_UPPER_RIGHT|U8G_DRAW_UPPER_LEFT|U8G_DRAW_LOWER_RIGHT|U8G_DRAW_LOWER_LEFT)); /*center circles*/
        u8g_DrawCircle( u8g_p, center_x, center_y, 1, (U8G_DRAW_UPPER_RIGHT|U8G_DRAW_UPPER_LEFT|U8G_DRAW_LOWER_RIGHT|U8G_DRAW_LOWER_LEFT));
        u8g_DrawCircle( u8g_p, center_x, center_y, 3, (U8G_DRAW_UPPER_RIGHT|U8G_DRAW_UPPER_LEFT|U8G_DRAW_LOWER_RIGHT|U8G_DRAW_LOWER_LEFT));
        u8g_DrawCircle( u8g_p, center_x, center_y, 0, (U8G_DRAW_UPPER_RIGHT|U8G_DRAW_UPPER_LEFT|U8G_DRAW_LOWER_RIGHT|U8G_DRAW_LOWER_LEFT));

#ifndef NHD
		last_page--;
		u8g_NextPage(u8g_p);

    }while(last_page);
#else
	}while(u8g_NextPage(u8g_p));
#endif


    if(counter < 70)  /*loop back around for about 12-13 seconds then cede control of display*/
    {
        ++counter;
        return stage;
    }
    else
    {
        counter = 0;
        tog ^= 1; /*used to switch bitmap on watchface*/
        return ++stage;
    }

}


/*
 * Main state machine to refresh display.
 * Has to return 0 and has to take void* argument which will be ignored in function body.
 */
int display_loop(IN void* ignore)
{
    static u8g_t u8g;
    static int w=0;
    static char stage=0;
    RtcTime rtcTime;
    char time_str[12]={0};
    extern void* g_bttransport;

    if (stage)
    {
        rtc_getRTCTime(&rtcTime);
#if TOOLCHAIN_wiced
        __2sprintf(time_str,"%02d:%02d:%02d.%02d", rtcTime.hour, rtcTime.minute, rtcTime.second, (w++)%60);
#else
        sprintf(time_str,"%02d:%02d:%02d.%02d", rtcTime.hour, rtcTime.minute, rtcTime.second, (w++)%60);
#endif
    }

    //WICED_BT_TRACE("display_loop stage=%d\n", stage);
    switch (stage)
    {
        case 0:
            stage= display_hw_init(&u8g, time_str, stage);
            break;
        case 1:
            stage= display_bitmap(&u8g, time_str, stage);
            break;
        case 2:
            stage = display_stocks(&u8g, time_str, stage);
            break;
        case 3:
            stage = display_menu(&u8g, time_str, stage);
            break;
        case 4:
            stage = display_analog_watch(&u8g, rtcTime, stage);
            break;
        default:
            stage= 1;
    }

    u8g_ScreenRefresh(); // send message to idle loop to call display_loop again
    //WICED_BT_TRACE("display_loop END\n");

    return 0;
}

