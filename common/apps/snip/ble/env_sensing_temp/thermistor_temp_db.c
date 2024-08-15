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
 * thermistor_temp_db.c
 *
 *  @brief
 * This file consists of the resistance to temperature lookup by finding the slope of the R Vs T curve
 *
 */


#include "thermistor_temp_db.h"

/*
Function name:
    r_t_look_up

Function Description:
@brief    function to map resistance to temperature using R Vs T look-up table

@param table            Pointer to R Vs T look-up table
@param therm_resist     Resistance of thermistor

@return Temperature in celsius as a floating point type
*/
float r_t_look_up(const r_t_look_up_table* table, float therm_resist) {
    int i=0;                                                                    //i is the index of the R Vs T lookup

    float m;                                                                    //m is the slope value between 2 temperature points

    while ((i < (TABLE_SIZE)) && (therm_resist < table[i].resistance_ohms)) {                            //find the two points in the table to use
        i++;
    }

    if (i == TABLE_SIZE) {                                                      //make sure the point isn't past the end of the table
        return table[i - 1].temp_celsius;
    }

    if (i == 0) {                                                               //make sure the point isn't before the beginning of the table
        return table[i].temp_celsius;
    }

    m = (table[i].temp_celsius - table[i - 1].temp_celsius) / (table[i].resistance_ohms - table[i - 1].resistance_ohms);          //calculate the slope

    return m * (therm_resist - table[i].resistance_ohms) + table[i].temp_celsius;                                   //this is the solution to the point slope formula
}
