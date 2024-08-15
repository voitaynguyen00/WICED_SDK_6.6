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
 *  thermistor_temp_db.h
 *
 *  @brief
 *  This file maintains the functionality of mapping resistance to temperature values of the thermistor
 *
 * The r_t_look_up_table is populated with the "Resistance Vs Temperature" table provided by the particular thermistor manufacturer Murata.
 *
 *
 * The thermistor has 1 degree variation because of the self-heating property of current flowing through it. For more details,
 * please read the documentation or the below link.
 *
 * http://www.murata.com/en-us/products/productdetail?partno=NCU15WF104F60RC
 *
 *
 */
#include<stdint.h>

#define TABLE_SIZE 166                                   /* Table size from the R/T(Resistance/Temperature) table provided by the Thermistor part's datasheet */

typedef struct {
    int16_t temp_celsius;
    float resistance_ohms;
} r_t_look_up_table;

/*
 * This table maps the R-center value to its equivalent temperature value. Changed the Kohms to Ohms
 */
r_t_look_up_table r_t_centre[TABLE_SIZE] =
{
    {-40, 4397119.3},

    {-39, 4092873.7},

    {-38, 3811717},

    {-37, 3551748.5},

    {-36, 3311235.8},

    {-35, 3088599},

    {-34, 2882396},

    {-33, 2691310},

    {-32, 2514137},

    {-31, 2349778},

    {-30, 2197225},

    {-29, 2055558},

    {-28, 1923932},

    {-27, 1801573},

    {-26, 1687773},

    {-25, 1581881},

    {-24, 1483100},

    {-23, 1391113},

    {-22, 1305413},

    {-21, 1225531},

    {-20, 1151037},

    {-19, 1081535},

    {-18, 1016661},

    {-17, 956079.6},

    {-16, 899480.6},

    {-15, 846578.8},

    {-14, 797111},

    {-13, 750834.1},

    {-12, 707523.7},

    {-11, 666972.3},

    {-10, 628988.2},

    {-9, 593342.1},

    {-8, 559930.9},

    {-7, 528601.6},

    {-6, 499212.4},

    {-5, 471632.1},

    {-4, 445771.6},

    {-3, 421479.6},

    {-2, 398652.1},

    {-1, 377192.7},

    {0, 357011.7},

    {1, 338005.8},

    {2, 320121.6},

    {3, 303286.6},

    {4, 287433.5},

    {5, 272499.5},

    {6, 258426.4},

    {7, 245159.8},

    {8, 232649.1},

    {9, 220847.1},

    {10, 209709.8},

    {11, 199196.2},

    {12, 189268.1},

    {13, 179889.6},

    {14, 171027.5},

    {15, 162650.6},

    {16, 154726.4},

    {17, 147232.1},

    {18, 140142},

    {19, 133432.2},

    {20, 127080.2},

    {21, 121065.8},

    {22, 115368.4},

    {23, 109969.5},

    {24, 104852.1},

    {25, 100000},

    {26, 95398.1},

    {27, 91032.2},

    {28, 86889},

    {29, 82956.1},

    {30, 79221.6},

    {31, 75675.2},

    {32, 72306},

    {33, 69104.2},

    {34, 66060.8},

    {35, 63167.1},

    {36, 60415},

    {37, 57796.9},

    {38, 55305.6},

    {39, 52934.3},

    {40, 50676.6},

    {41, 48528.3},

    {42, 46482},

    {43, 44532.5},

    {44, 42674.5},

    {45, 40903.5},

    {46, 39213.2},

    {47, 37601},

    {48, 36062.9},

    {49, 34595.3},

    {50, 33194.6},

    {51, 31859.1},

    {52, 30583.9},

    {53, 29366},

    {54, 28202.6},

    {55, 27090.9},

    {56, 26028.4},

    {57, 25012.7},

    {58, 24041.6},

    {59, 23112.8},

    {60, 22224.3},

    {61, 21374.3},

    {62, 20560.7},

    {63, 19782},

    {64, 19036.4},

    {65, 18322.5},

    {66, 17640.1},

    {67, 16986.4},

    {68, 16360},

    {69, 15759.6},

    {70, 15184.1},

    {71, 14631},

    {72, 14100.6},

    {73, 13591.8},

    {74, 13103.7},

    {75, 12635.4},

    {76, 12187.1},

    {77, 11756.7},

    {78, 11343.6},

    {79, 10946.8},

    {80, 10565.7},

    {81, 10199.6},

    {82, 9847.9},

    {83, 9509.8},

    {84, 9184.9},

    {85, 8872.6},

    {86, 8572.2},

    {87, 8283.4},

    {88, 8005.5},

    {89, 7738.3},

    {90, 7481.1},

    {91, 7234.4},

    {92, 6997.1},

    {93, 6768.5},

    {94, 6548.4},

    {95, 6336.5},

    {96, 6131.6},

    {97, 5934.1},

    {98, 5743.9},

    {99, 5560.6},

    {100, 5383.9},

    {101, 5214.3},

    {102, 5050.7},

    {103, 4893},

    {104, 4740.9},

    {105, 4594.2},

    {106, 4452.7},

    {107, 4316.1},

    {108, 4184.3},

    {109, 4057},

    {110, 3934.2},

    {111, 3815.6},

    {112, 3701.1},

    {113, 3590.5},

    {114, 3483.6},

    {115, 3380.4},

    {116, 3281.2},

    {117, 3185.3},

    {118, 3092.6},

    {119, 3003.1},

    {120, 2916.4},

    {121, 2832.2},

    {122, 2750.8},

    {123, 2672},

    {124, 2595.8},

    {125, 2522}

};

float r_t_look_up(const r_t_look_up_table * table, float x);
