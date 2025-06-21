//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[37];
extern uint8_t ui_g_dirty_figure[37];
extern ui_interface_string_t ui_g_now_strings[11];
extern uint8_t ui_g_dirty_string[11];

#define ui_g_Ungroup_RightArc ((ui_interface_arc_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_capLine ((ui_interface_line_t*)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_Pipe ((ui_interface_line_t*)&(ui_g_now_figures[2]))
#define ui_g_Ungroup_RIghtLine ((ui_interface_line_t*)&(ui_g_now_figures[3]))
#define ui_g_Ungroup_LeftLine ((ui_interface_line_t*)&(ui_g_now_figures[4]))
#define ui_g_Ungroup_pitch ((ui_interface_number_t*)&(ui_g_now_figures[5]))
#define ui_g_Ungroup_RestCap ((ui_interface_number_t*)&(ui_g_now_figures[6]))
#define ui_g_Ungroup_Power ((ui_interface_number_t*)&(ui_g_now_figures[7]))
#define ui_g_st_Line1 ((ui_interface_line_t*)&(ui_g_now_figures[8]))
#define ui_g_Ungroup_speed ((ui_interface_number_t*)&(ui_g_now_figures[9]))
#define ui_g_Ungroup_GYRORec ((ui_interface_rect_t*)&(ui_g_now_figures[10]))
#define ui_g_Ungroup_ShooterRec ((ui_interface_rect_t*)&(ui_g_now_figures[11]))
#define ui_g_Ungroup_pitchLow ((ui_interface_line_t*)&(ui_g_now_figures[12]))
#define ui_g_st_Line2 ((ui_interface_line_t*)&(ui_g_now_figures[13]))
#define ui_g_Ungroup_LIne11 ((ui_interface_line_t*)&(ui_g_now_figures[14]))
#define ui_g_Ungroup_Line12 ((ui_interface_line_t*)&(ui_g_now_figures[15]))
#define ui_g_Ungroup_Line13 ((ui_interface_line_t*)&(ui_g_now_figures[16]))
#define ui_g_Ungroup_Line14 ((ui_interface_line_t*)&(ui_g_now_figures[17]))
#define ui_g_Ungroup_Line15 ((ui_interface_line_t*)&(ui_g_now_figures[18]))
#define ui_g_Ungroup_Line16 ((ui_interface_line_t*)&(ui_g_now_figures[19]))
#define ui_g_Ungroup_Target_P ((ui_interface_number_t*)&(ui_g_now_figures[20]))
#define ui_g_Ungroup_LIne3 ((ui_interface_line_t*)&(ui_g_now_figures[21]))
#define ui_g_Ungroup_ForceRec ((ui_interface_rect_t*)&(ui_g_now_figures[22]))
#define ui_g_Ungroup_PitchHigh ((ui_interface_line_t*)&(ui_g_now_figures[23]))
#define ui_g_Ungroup_Line4 ((ui_interface_line_t*)&(ui_g_now_figures[24]))
#define ui_g_Ungroup_vLine1 ((ui_interface_line_t*)&(ui_g_now_figures[25]))
#define ui_g_Ungroup_capRact ((ui_interface_rect_t*)&(ui_g_now_figures[26]))
#define ui_g_Ungroup_chassis ((ui_interface_rect_t*)&(ui_g_now_figures[27]))
#define ui_g_Ungroup_BC_Rec ((ui_interface_rect_t*)&(ui_g_now_figures[28]))
#define ui_g_Ungroup_VTM_Rec ((ui_interface_rect_t*)&(ui_g_now_figures[29]))
#define ui_g_Ungroup_follow_rec ((ui_interface_rect_t*)&(ui_g_now_figures[30]))
#define ui_g_Ungroup_aim_rec ((ui_interface_rect_t*)&(ui_g_now_figures[31]))
#define ui_g_Ungroup_backWheel ((ui_interface_round_t*)&(ui_g_now_figures[32]))
#define ui_g_Ungroup_forwardWheel ((ui_interface_round_t*)&(ui_g_now_figures[33]))
#define ui_g_Ungroup_GM ((ui_interface_round_t*)&(ui_g_now_figures[34]))
#define ui_g_Ungroup_LeftArc ((ui_interface_arc_t*)&(ui_g_now_figures[35]))
#define ui_g_Ungroup_LED ((ui_interface_line_t*)&(ui_g_now_figures[36]))

#define ui_g_Ungroup_Aim (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_aimmode (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_powerMode (&(ui_g_now_strings[2]))
#define ui_g_Ungroup_shooter (&(ui_g_now_strings[3]))
#define ui_g_Ungroup_gyro (&(ui_g_now_strings[4]))
#define ui_g_Ungroup_customP (&(ui_g_now_strings[5]))
#define ui_g_Ungroup_Power_mode_out (&(ui_g_now_strings[6]))
#define ui_g_Ungroup_Force_Ctrl (&(ui_g_now_strings[7]))
#define ui_g_Ungroup_boardCom (&(ui_g_now_strings[8]))
#define ui_g_Ungroup_vtm (&(ui_g_now_strings[9]))
#define ui_g_Ungroup_follow (&(ui_g_now_strings[10]))

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_RightArc_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_capLine_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_Pipe_dirty (ui_g_dirty_figure[2])
#define ui_g_Ungroup_RIghtLine_dirty (ui_g_dirty_figure[3])
#define ui_g_Ungroup_LeftLine_dirty (ui_g_dirty_figure[4])
#define ui_g_Ungroup_pitch_dirty (ui_g_dirty_figure[5])
#define ui_g_Ungroup_RestCap_dirty (ui_g_dirty_figure[6])
#define ui_g_Ungroup_Power_dirty (ui_g_dirty_figure[7])
#define ui_g_st_Line1_dirty (ui_g_dirty_figure[8])
#define ui_g_Ungroup_speed_dirty (ui_g_dirty_figure[9])
#define ui_g_Ungroup_GYRORec_dirty (ui_g_dirty_figure[10])
#define ui_g_Ungroup_ShooterRec_dirty (ui_g_dirty_figure[11])
#define ui_g_Ungroup_pitchLow_dirty (ui_g_dirty_figure[12])
#define ui_g_st_Line2_dirty (ui_g_dirty_figure[13])
#define ui_g_Ungroup_LIne11_dirty (ui_g_dirty_figure[14])
#define ui_g_Ungroup_Line12_dirty (ui_g_dirty_figure[15])
#define ui_g_Ungroup_Line13_dirty (ui_g_dirty_figure[16])
#define ui_g_Ungroup_Line14_dirty (ui_g_dirty_figure[17])
#define ui_g_Ungroup_Line15_dirty (ui_g_dirty_figure[18])
#define ui_g_Ungroup_Line16_dirty (ui_g_dirty_figure[19])
#define ui_g_Ungroup_Target_P_dirty (ui_g_dirty_figure[20])
#define ui_g_Ungroup_LIne3_dirty (ui_g_dirty_figure[21])
#define ui_g_Ungroup_ForceRec_dirty (ui_g_dirty_figure[22])
#define ui_g_Ungroup_PitchHigh_dirty (ui_g_dirty_figure[23])
#define ui_g_Ungroup_Line4_dirty (ui_g_dirty_figure[24])
#define ui_g_Ungroup_vLine1_dirty (ui_g_dirty_figure[25])
#define ui_g_Ungroup_capRact_dirty (ui_g_dirty_figure[26])
#define ui_g_Ungroup_chassis_dirty (ui_g_dirty_figure[27])
#define ui_g_Ungroup_BC_Rec_dirty (ui_g_dirty_figure[28])
#define ui_g_Ungroup_VTM_Rec_dirty (ui_g_dirty_figure[29])
#define ui_g_Ungroup_follow_rec_dirty (ui_g_dirty_figure[30])
#define ui_g_Ungroup_aim_rec_dirty (ui_g_dirty_figure[31])
#define ui_g_Ungroup_backWheel_dirty (ui_g_dirty_figure[32])
#define ui_g_Ungroup_forwardWheel_dirty (ui_g_dirty_figure[33])
#define ui_g_Ungroup_GM_dirty (ui_g_dirty_figure[34])
#define ui_g_Ungroup_LeftArc_dirty (ui_g_dirty_figure[35])
#define ui_g_Ungroup_LED_dirty (ui_g_dirty_figure[36])

#define ui_g_Ungroup_Aim_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_aimmode_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_powerMode_dirty (ui_g_dirty_string[2])
#define ui_g_Ungroup_shooter_dirty (ui_g_dirty_string[3])
#define ui_g_Ungroup_gyro_dirty (ui_g_dirty_string[4])
#define ui_g_Ungroup_customP_dirty (ui_g_dirty_string[5])
#define ui_g_Ungroup_Power_mode_out_dirty (ui_g_dirty_string[6])
#define ui_g_Ungroup_Force_Ctrl_dirty (ui_g_dirty_string[7])
#define ui_g_Ungroup_boardCom_dirty (ui_g_dirty_string[8])
#define ui_g_Ungroup_vtm_dirty (ui_g_dirty_string[9])
#define ui_g_Ungroup_follow_dirty (ui_g_dirty_string[10])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
