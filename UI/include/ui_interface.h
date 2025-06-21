//
// Created by bismarckkk on 2025/3/22.
// Dynamic Edition
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "main.h"
#include "ui_types.h"

extern int ui_self_id;
extern HAL_UART_StateTypeDef huart2;

void print_message(uint8_t const* message, int length);

#define SEND_MESSAGE(message, length) HAL_UART_Transmit(&huart2, message, length,100);osDelay(35);

void ui_proc_1_frame(ui_1_frame_t* msg);
void ui_proc_2_frame(ui_2_frame_t* msg);
void ui_proc_5_frame(ui_5_frame_t* msg);
void ui_proc_7_frame(ui_7_frame_t* msg);
void ui_proc_string_frame(ui_string_frame_t* msg);
void ui_proc_delete_frame(ui_delete_frame_t* msg);

void ui_delete_layer(uint8_t const delete_type, uint8_t const layer);

void ui_scan_and_send(ui_interface_figure_t const* ui_now_figures, uint8_t* ui_dirty_figure,
                      ui_interface_string_t const* ui_now_strings, uint8_t* ui_dirty_string, int total_figures,
                      int total_strings);

#endif  // UI_INTERFACE_H
