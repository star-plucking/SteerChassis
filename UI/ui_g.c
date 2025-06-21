//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 37
#define TOTAL_STRING 11

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_Ungroup_RightArc->figure_type = 4;
    ui_g_Ungroup_RightArc->operate_type = 1;
    ui_g_Ungroup_RightArc->layer = 3;
    ui_g_Ungroup_RightArc->color = 1;
    ui_g_Ungroup_RightArc->start_x = 941;
    ui_g_Ungroup_RightArc->start_y = 540;
    ui_g_Ungroup_RightArc->width = 15;
    ui_g_Ungroup_RightArc->start_angle = 55;
    ui_g_Ungroup_RightArc->end_angle = 125;
    ui_g_Ungroup_RightArc->rx = 395;
    ui_g_Ungroup_RightArc->ry = 395;

    ui_g_Ungroup_capLine->figure_type = 0;
    ui_g_Ungroup_capLine->operate_type = 1;
    ui_g_Ungroup_capLine->layer = 1;
    ui_g_Ungroup_capLine->color = 2;
    ui_g_Ungroup_capLine->start_x = 720;
    ui_g_Ungroup_capLine->start_y = 857;
    ui_g_Ungroup_capLine->width = 20;
    ui_g_Ungroup_capLine->end_x = 1202;
    ui_g_Ungroup_capLine->end_y = 857;

    ui_g_Ungroup_Pipe->figure_type = 0;
    ui_g_Ungroup_Pipe->operate_type = 1;
    ui_g_Ungroup_Pipe->layer = 2;
    ui_g_Ungroup_Pipe->color = 0;
    ui_g_Ungroup_Pipe->start_x = 960;
    ui_g_Ungroup_Pipe->start_y = 144;
    ui_g_Ungroup_Pipe->width = 10;
    ui_g_Ungroup_Pipe->end_x = 960;
    ui_g_Ungroup_Pipe->end_y = 244;

    ui_g_Ungroup_RIghtLine->figure_type = 0;
    ui_g_Ungroup_RIghtLine->operate_type = 1;
    ui_g_Ungroup_RIghtLine->layer = 2;
    ui_g_Ungroup_RIghtLine->color = 6;
    ui_g_Ungroup_RIghtLine->start_x = 910;
    ui_g_Ungroup_RIghtLine->start_y = 100;
    ui_g_Ungroup_RIghtLine->width = 8;
    ui_g_Ungroup_RIghtLine->end_x = 1010;
    ui_g_Ungroup_RIghtLine->end_y = 200;

    ui_g_Ungroup_LeftLine->figure_type = 0;
    ui_g_Ungroup_LeftLine->operate_type = 1;
    ui_g_Ungroup_LeftLine->layer = 2;
    ui_g_Ungroup_LeftLine->color = 6;
    ui_g_Ungroup_LeftLine->start_x = 1010;
    ui_g_Ungroup_LeftLine->start_y = 100;
    ui_g_Ungroup_LeftLine->width = 8;
    ui_g_Ungroup_LeftLine->end_x = 910;
    ui_g_Ungroup_LeftLine->end_y = 200;

    ui_g_Ungroup_pitch->figure_type = 5;
    ui_g_Ungroup_pitch->operate_type = 1;
    ui_g_Ungroup_pitch->layer = 2;
    ui_g_Ungroup_pitch->color = 3;
    ui_g_Ungroup_pitch->start_x = 1089;
    ui_g_Ungroup_pitch->start_y = 563;
    ui_g_Ungroup_pitch->width = 2;
    ui_g_Ungroup_pitch->font_size = 20;
    ui_g_Ungroup_pitch->number = 12345;

    ui_g_Ungroup_RestCap->figure_type = 6;
    ui_g_Ungroup_RestCap->operate_type = 1;
    ui_g_Ungroup_RestCap->layer = 2;
    ui_g_Ungroup_RestCap->color = 8;
    ui_g_Ungroup_RestCap->start_x = 1213;
    ui_g_Ungroup_RestCap->start_y = 880;
    ui_g_Ungroup_RestCap->width = 2;
    ui_g_Ungroup_RestCap->font_size = 20;
    ui_g_Ungroup_RestCap->number = 12345;

    ui_g_Ungroup_Power->figure_type = 6;
    ui_g_Ungroup_Power->operate_type = 1;
    ui_g_Ungroup_Power->layer = 2;
    ui_g_Ungroup_Power->color = 1;
    ui_g_Ungroup_Power->start_x = 1246;
    ui_g_Ungroup_Power->start_y = 315;
    ui_g_Ungroup_Power->width = 2;
    ui_g_Ungroup_Power->font_size = 20;
    ui_g_Ungroup_Power->number = 12345;

    ui_g_st_Line1->figure_type = 0;
    ui_g_st_Line1->operate_type = 1;
    ui_g_st_Line1->layer = 0;
    ui_g_st_Line1->color = 6;
    ui_g_st_Line1->start_x = 860;
    ui_g_st_Line1->start_y = 539;
    ui_g_st_Line1->width = 1;
    ui_g_st_Line1->end_x = 1059;
    ui_g_st_Line1->end_y = 538;

    ui_g_Ungroup_speed->figure_type = 6;
    ui_g_Ungroup_speed->operate_type = 1;
    ui_g_Ungroup_speed->layer = 3;
    ui_g_Ungroup_speed->color = 1;
    ui_g_Ungroup_speed->start_x = 591;
    ui_g_Ungroup_speed->start_y = 316;
    ui_g_Ungroup_speed->width = 2;
    ui_g_Ungroup_speed->font_size = 20;
    ui_g_Ungroup_speed->number = 12345;

    ui_g_Ungroup_GYRORec->figure_type = 1;
    ui_g_Ungroup_GYRORec->operate_type = 1;
    ui_g_Ungroup_GYRORec->layer = 5;
    ui_g_Ungroup_GYRORec->color = 1;
    ui_g_Ungroup_GYRORec->start_x = 482;
    ui_g_Ungroup_GYRORec->start_y = 744;
    ui_g_Ungroup_GYRORec->width = 1;
    ui_g_Ungroup_GYRORec->end_x = 515;
    ui_g_Ungroup_GYRORec->end_y = 777;

    ui_g_Ungroup_ShooterRec->figure_type = 1;
    ui_g_Ungroup_ShooterRec->operate_type = 1;
    ui_g_Ungroup_ShooterRec->layer = 3;
    ui_g_Ungroup_ShooterRec->color = 1;
    ui_g_Ungroup_ShooterRec->start_x = 482;
    ui_g_Ungroup_ShooterRec->start_y = 700;
    ui_g_Ungroup_ShooterRec->width = 1;
    ui_g_Ungroup_ShooterRec->end_x = 515;
    ui_g_Ungroup_ShooterRec->end_y = 733;

    ui_g_Ungroup_pitchLow->figure_type = 0;
    ui_g_Ungroup_pitchLow->operate_type = 1;
    ui_g_Ungroup_pitchLow->layer = 1;
    ui_g_Ungroup_pitchLow->color = 1;
    ui_g_Ungroup_pitchLow->start_x = 1568;
    ui_g_Ungroup_pitchLow->start_y = 537;
    ui_g_Ungroup_pitchLow->width = 14;
    ui_g_Ungroup_pitchLow->end_x = 1568;
    ui_g_Ungroup_pitchLow->end_y = 593;

    ui_g_st_Line2->figure_type = 0;
    ui_g_st_Line2->operate_type = 1;
    ui_g_st_Line2->layer = 0;
    ui_g_st_Line2->color = 6;
    ui_g_st_Line2->start_x = 880;
    ui_g_st_Line2->start_y = 490;
    ui_g_st_Line2->width = 1;
    ui_g_st_Line2->end_x = 1040;
    ui_g_st_Line2->end_y = 490;

    ui_g_Ungroup_LIne11->figure_type = 0;
    ui_g_Ungroup_LIne11->operate_type = 1;
    ui_g_Ungroup_LIne11->layer = 4;
    ui_g_Ungroup_LIne11->color = 4;
    ui_g_Ungroup_LIne11->start_x = 568;
    ui_g_Ungroup_LIne11->start_y = 536;
    ui_g_Ungroup_LIne11->width = 5;
    ui_g_Ungroup_LIne11->end_x = 596;
    ui_g_Ungroup_LIne11->end_y = 536;

    ui_g_Ungroup_Line12->figure_type = 0;
    ui_g_Ungroup_Line12->operate_type = 1;
    ui_g_Ungroup_Line12->layer = 4;
    ui_g_Ungroup_Line12->color = 4;
    ui_g_Ungroup_Line12->start_x = 1318;
    ui_g_Ungroup_Line12->start_y = 539;
    ui_g_Ungroup_Line12->width = 5;
    ui_g_Ungroup_Line12->end_x = 1351;
    ui_g_Ungroup_Line12->end_y = 539;

    ui_g_Ungroup_Line13->figure_type = 0;
    ui_g_Ungroup_Line13->operate_type = 1;
    ui_g_Ungroup_Line13->layer = 4;
    ui_g_Ungroup_Line13->color = 7;
    ui_g_Ungroup_Line13->start_x = 1304;
    ui_g_Ungroup_Line13->start_y = 655;
    ui_g_Ungroup_Line13->width = 5;
    ui_g_Ungroup_Line13->end_x = 1333;
    ui_g_Ungroup_Line13->end_y = 665;

    ui_g_Ungroup_Line14->figure_type = 0;
    ui_g_Ungroup_Line14->operate_type = 1;
    ui_g_Ungroup_Line14->layer = 4;
    ui_g_Ungroup_Line14->color = 7;
    ui_g_Ungroup_Line14->start_x = 610;
    ui_g_Ungroup_Line14->start_y = 657;
    ui_g_Ungroup_Line14->width = 5;
    ui_g_Ungroup_Line14->end_x = 586;
    ui_g_Ungroup_Line14->end_y = 665;

    ui_g_Ungroup_Line15->figure_type = 0;
    ui_g_Ungroup_Line15->operate_type = 1;
    ui_g_Ungroup_Line15->layer = 4;
    ui_g_Ungroup_Line15->color = 6;
    ui_g_Ungroup_Line15->start_x = 588;
    ui_g_Ungroup_Line15->start_y = 404;
    ui_g_Ungroup_Line15->width = 5;
    ui_g_Ungroup_Line15->end_x = 616;
    ui_g_Ungroup_Line15->end_y = 418;

    ui_g_Ungroup_Line16->figure_type = 0;
    ui_g_Ungroup_Line16->operate_type = 1;
    ui_g_Ungroup_Line16->layer = 4;
    ui_g_Ungroup_Line16->color = 6;
    ui_g_Ungroup_Line16->start_x = 1332;
    ui_g_Ungroup_Line16->start_y = 419;
    ui_g_Ungroup_Line16->width = 5;
    ui_g_Ungroup_Line16->end_x = 1304;
    ui_g_Ungroup_Line16->end_y = 423;

    ui_g_Ungroup_Target_P->figure_type = 6;
    ui_g_Ungroup_Target_P->operate_type = 1;
    ui_g_Ungroup_Target_P->layer = 5;
    ui_g_Ungroup_Target_P->color = 1;
    ui_g_Ungroup_Target_P->start_x = 1045;
    ui_g_Ungroup_Target_P->start_y = 805;
    ui_g_Ungroup_Target_P->width = 2;
    ui_g_Ungroup_Target_P->font_size = 20;
    ui_g_Ungroup_Target_P->number = 12345;

    ui_g_Ungroup_LIne3->figure_type = 0;
    ui_g_Ungroup_LIne3->operate_type = 1;
    ui_g_Ungroup_LIne3->layer = 0;
    ui_g_Ungroup_LIne3->color = 6;
    ui_g_Ungroup_LIne3->start_x = 900;
    ui_g_Ungroup_LIne3->start_y = 440;
    ui_g_Ungroup_LIne3->width = 1;
    ui_g_Ungroup_LIne3->end_x = 1020;
    ui_g_Ungroup_LIne3->end_y = 440;

    ui_g_Ungroup_ForceRec->figure_type = 1;
    ui_g_Ungroup_ForceRec->operate_type = 1;
    ui_g_Ungroup_ForceRec->layer = 5;
    ui_g_Ungroup_ForceRec->color = 1;
    ui_g_Ungroup_ForceRec->start_x = 482;
    ui_g_Ungroup_ForceRec->start_y = 785;
    ui_g_Ungroup_ForceRec->width = 1;
    ui_g_Ungroup_ForceRec->end_x = 515;
    ui_g_Ungroup_ForceRec->end_y = 818;

    ui_g_Ungroup_PitchHigh->figure_type = 0;
    ui_g_Ungroup_PitchHigh->operate_type = 1;
    ui_g_Ungroup_PitchHigh->layer = 1;
    ui_g_Ungroup_PitchHigh->color = 1;
    ui_g_Ungroup_PitchHigh->start_x = 1552;
    ui_g_Ungroup_PitchHigh->start_y = 591;
    ui_g_Ungroup_PitchHigh->width = 8;
    ui_g_Ungroup_PitchHigh->end_x = 1620;
    ui_g_Ungroup_PitchHigh->end_y = 591;

    ui_g_Ungroup_Line4->figure_type = 0;
    ui_g_Ungroup_Line4->operate_type = 1;
    ui_g_Ungroup_Line4->layer = 0;
    ui_g_Ungroup_Line4->color = 6;
    ui_g_Ungroup_Line4->start_x = 920;
    ui_g_Ungroup_Line4->start_y = 400;
    ui_g_Ungroup_Line4->width = 1;
    ui_g_Ungroup_Line4->end_x = 999;
    ui_g_Ungroup_Line4->end_y = 400;

    ui_g_Ungroup_vLine1->figure_type = 0;
    ui_g_Ungroup_vLine1->operate_type = 1;
    ui_g_Ungroup_vLine1->layer = 0;
    ui_g_Ungroup_vLine1->color = 6;
    ui_g_Ungroup_vLine1->start_x = 960;
    ui_g_Ungroup_vLine1->start_y = 359;
    ui_g_Ungroup_vLine1->width = 1;
    ui_g_Ungroup_vLine1->end_x = 960;
    ui_g_Ungroup_vLine1->end_y = 578;

    ui_g_Ungroup_capRact->figure_type = 1;
    ui_g_Ungroup_capRact->operate_type = 1;
    ui_g_Ungroup_capRact->layer = 1;
    ui_g_Ungroup_capRact->color = 2;
    ui_g_Ungroup_capRact->start_x = 719;
    ui_g_Ungroup_capRact->start_y = 848;
    ui_g_Ungroup_capRact->width = 4;
    ui_g_Ungroup_capRact->end_x = 1199;
    ui_g_Ungroup_capRact->end_y = 868;

    ui_g_Ungroup_chassis->figure_type = 1;
    ui_g_Ungroup_chassis->operate_type = 1;
    ui_g_Ungroup_chassis->layer = 1;
    ui_g_Ungroup_chassis->color = 0;
    ui_g_Ungroup_chassis->start_x = 1516;
    ui_g_Ungroup_chassis->start_y = 511;
    ui_g_Ungroup_chassis->width = 8;
    ui_g_Ungroup_chassis->end_x = 1622;
    ui_g_Ungroup_chassis->end_y = 536;

    ui_g_Ungroup_BC_Rec->figure_type = 1;
    ui_g_Ungroup_BC_Rec->operate_type = 1;
    ui_g_Ungroup_BC_Rec->layer = 6;
    ui_g_Ungroup_BC_Rec->color = 1;
    ui_g_Ungroup_BC_Rec->start_x = 482;
    ui_g_Ungroup_BC_Rec->start_y = 657;
    ui_g_Ungroup_BC_Rec->width = 1;
    ui_g_Ungroup_BC_Rec->end_x = 515;
    ui_g_Ungroup_BC_Rec->end_y = 690;

    ui_g_Ungroup_VTM_Rec->figure_type = 1;
    ui_g_Ungroup_VTM_Rec->operate_type = 1;
    ui_g_Ungroup_VTM_Rec->layer = 6;
    ui_g_Ungroup_VTM_Rec->color = 1;
    ui_g_Ungroup_VTM_Rec->start_x = 482;
    ui_g_Ungroup_VTM_Rec->start_y = 614;
    ui_g_Ungroup_VTM_Rec->width = 1;
    ui_g_Ungroup_VTM_Rec->end_x = 515;
    ui_g_Ungroup_VTM_Rec->end_y = 647;

    ui_g_Ungroup_follow_rec->figure_type = 1;
    ui_g_Ungroup_follow_rec->operate_type = 1;
    ui_g_Ungroup_follow_rec->layer = 6;
    ui_g_Ungroup_follow_rec->color = 1;
    ui_g_Ungroup_follow_rec->start_x = 482;
    ui_g_Ungroup_follow_rec->start_y = 571;
    ui_g_Ungroup_follow_rec->width = 1;
    ui_g_Ungroup_follow_rec->end_x = 515;
    ui_g_Ungroup_follow_rec->end_y = 604;

    ui_g_Ungroup_aim_rec->figure_type = 1;
    ui_g_Ungroup_aim_rec->operate_type = 1;
    ui_g_Ungroup_aim_rec->layer = 7;
    ui_g_Ungroup_aim_rec->color = 6;
    ui_g_Ungroup_aim_rec->start_x = 693;
    ui_g_Ungroup_aim_rec->start_y = 312;
    ui_g_Ungroup_aim_rec->width = 1;
    ui_g_Ungroup_aim_rec->end_x = 1239;
    ui_g_Ungroup_aim_rec->end_y = 759;

    ui_g_Ungroup_backWheel->figure_type = 2;
    ui_g_Ungroup_backWheel->operate_type = 1;
    ui_g_Ungroup_backWheel->layer = 1;
    ui_g_Ungroup_backWheel->color = 0;
    ui_g_Ungroup_backWheel->start_x = 1527;
    ui_g_Ungroup_backWheel->start_y = 495;
    ui_g_Ungroup_backWheel->width = 10;
    ui_g_Ungroup_backWheel->r = 13;

    ui_g_Ungroup_forwardWheel->figure_type = 2;
    ui_g_Ungroup_forwardWheel->operate_type = 1;
    ui_g_Ungroup_forwardWheel->layer = 1;
    ui_g_Ungroup_forwardWheel->color = 0;
    ui_g_Ungroup_forwardWheel->start_x = 1606;
    ui_g_Ungroup_forwardWheel->start_y = 494;
    ui_g_Ungroup_forwardWheel->width = 10;
    ui_g_Ungroup_forwardWheel->r = 13;

    ui_g_Ungroup_GM->figure_type = 2;
    ui_g_Ungroup_GM->operate_type = 1;
    ui_g_Ungroup_GM->layer = 1;
    ui_g_Ungroup_GM->color = 1;
    ui_g_Ungroup_GM->start_x = 1548;
    ui_g_Ungroup_GM->start_y = 591;
    ui_g_Ungroup_GM->width = 8;
    ui_g_Ungroup_GM->r = 10;

    ui_g_Ungroup_LeftArc->figure_type = 4;
    ui_g_Ungroup_LeftArc->operate_type = 1;
    ui_g_Ungroup_LeftArc->layer = 3;
    ui_g_Ungroup_LeftArc->color = 1;
    ui_g_Ungroup_LeftArc->start_x = 960;
    ui_g_Ungroup_LeftArc->start_y = 542;
    ui_g_Ungroup_LeftArc->width = 15;
    ui_g_Ungroup_LeftArc->start_angle = 235;
    ui_g_Ungroup_LeftArc->end_angle = 305;
    ui_g_Ungroup_LeftArc->rx = 387;
    ui_g_Ungroup_LeftArc->ry = 395;

    ui_g_Ungroup_LED->figure_type = 0;
    ui_g_Ungroup_LED->operate_type = 1;
    ui_g_Ungroup_LED->layer = 2;
    ui_g_Ungroup_LED->color = 0;
    ui_g_Ungroup_LED->start_x = 940;
    ui_g_Ungroup_LED->start_y = 110;
    ui_g_Ungroup_LED->width = 5;
    ui_g_Ungroup_LED->end_x = 980;
    ui_g_Ungroup_LED->end_y = 110;

    ui_g_Ungroup_Aim->figure_type = 7;
    ui_g_Ungroup_Aim->operate_type = 1;
    ui_g_Ungroup_Aim->layer = 3;
    ui_g_Ungroup_Aim->color = 6;
    ui_g_Ungroup_Aim->start_x = 882;
    ui_g_Ungroup_Aim->start_y = 357;
    ui_g_Ungroup_Aim->width = 2;
    ui_g_Ungroup_Aim->font_size = 20;
    ui_g_Ungroup_Aim->str_length = 4;
    strcpy(ui_g_Ungroup_Aim->string, "AIM:");

    ui_g_Ungroup_aimmode->figure_type = 7;
    ui_g_Ungroup_aimmode->operate_type = 1;
    ui_g_Ungroup_aimmode->layer = 3;
    ui_g_Ungroup_aimmode->color = 6;
    ui_g_Ungroup_aimmode->start_x = 962;
    ui_g_Ungroup_aimmode->start_y = 355;
    ui_g_Ungroup_aimmode->width = 2;
    ui_g_Ungroup_aimmode->font_size = 20;
    ui_g_Ungroup_aimmode->str_length = 4;
    strcpy(ui_g_Ungroup_aimmode->string, "Text");

    ui_g_Ungroup_powerMode->figure_type = 7;
    ui_g_Ungroup_powerMode->operate_type = 1;
    ui_g_Ungroup_powerMode->layer = 3;
    ui_g_Ungroup_powerMode->color = 1;
    ui_g_Ungroup_powerMode->start_x = 785;
    ui_g_Ungroup_powerMode->start_y = 841;
    ui_g_Ungroup_powerMode->width = 2;
    ui_g_Ungroup_powerMode->font_size = 20;
    ui_g_Ungroup_powerMode->str_length = 11;
    strcpy(ui_g_Ungroup_powerMode->string, "POWER_MODE:");

    ui_g_Ungroup_shooter->figure_type = 7;
    ui_g_Ungroup_shooter->operate_type = 1;
    ui_g_Ungroup_shooter->layer = 5;
    ui_g_Ungroup_shooter->color = 1;
    ui_g_Ungroup_shooter->start_x = 330;
    ui_g_Ungroup_shooter->start_y = 727;
    ui_g_Ungroup_shooter->width = 2;
    ui_g_Ungroup_shooter->font_size = 20;
    ui_g_Ungroup_shooter->str_length = 7;
    strcpy(ui_g_Ungroup_shooter->string, "SHOOTER");

    ui_g_Ungroup_gyro->figure_type = 7;
    ui_g_Ungroup_gyro->operate_type = 1;
    ui_g_Ungroup_gyro->layer = 5;
    ui_g_Ungroup_gyro->color = 1;
    ui_g_Ungroup_gyro->start_x = 385;
    ui_g_Ungroup_gyro->start_y = 770;
    ui_g_Ungroup_gyro->width = 2;
    ui_g_Ungroup_gyro->font_size = 20;
    ui_g_Ungroup_gyro->str_length = 4;
    strcpy(ui_g_Ungroup_gyro->string, "gyro");

    ui_g_Ungroup_customP->figure_type = 7;
    ui_g_Ungroup_customP->operate_type = 1;
    ui_g_Ungroup_customP->layer = 5;
    ui_g_Ungroup_customP->color = 1;
    ui_g_Ungroup_customP->start_x = 785;
    ui_g_Ungroup_customP->start_y = 805;
    ui_g_Ungroup_customP->width = 2;
    ui_g_Ungroup_customP->font_size = 20;
    ui_g_Ungroup_customP->str_length = 13;
    strcpy(ui_g_Ungroup_customP->string, "CUSTOM_POWER:");

    ui_g_Ungroup_Power_mode_out->figure_type = 7;
    ui_g_Ungroup_Power_mode_out->operate_type = 1;
    ui_g_Ungroup_Power_mode_out->layer = 5;
    ui_g_Ungroup_Power_mode_out->color = 2;
    ui_g_Ungroup_Power_mode_out->start_x = 1008;
    ui_g_Ungroup_Power_mode_out->start_y = 841;
    ui_g_Ungroup_Power_mode_out->width = 2;
    ui_g_Ungroup_Power_mode_out->font_size = 20;
    ui_g_Ungroup_Power_mode_out->str_length = 6;
    strcpy(ui_g_Ungroup_Power_mode_out->string, "MAXOUT");

    ui_g_Ungroup_Force_Ctrl->figure_type = 7;
    ui_g_Ungroup_Force_Ctrl->operate_type = 1;
    ui_g_Ungroup_Force_Ctrl->layer = 6;
    ui_g_Ungroup_Force_Ctrl->color = 1;
    ui_g_Ungroup_Force_Ctrl->start_x = 266;
    ui_g_Ungroup_Force_Ctrl->start_y = 809;
    ui_g_Ungroup_Force_Ctrl->width = 2;
    ui_g_Ungroup_Force_Ctrl->font_size = 20;
    ui_g_Ungroup_Force_Ctrl->str_length = 10;
    strcpy(ui_g_Ungroup_Force_Ctrl->string, "FORCE_CTRL");

    ui_g_Ungroup_boardCom->figure_type = 7;
    ui_g_Ungroup_boardCom->operate_type = 1;
    ui_g_Ungroup_boardCom->layer = 6;
    ui_g_Ungroup_boardCom->color = 1;
    ui_g_Ungroup_boardCom->start_x = 290;
    ui_g_Ungroup_boardCom->start_y = 684;
    ui_g_Ungroup_boardCom->width = 2;
    ui_g_Ungroup_boardCom->font_size = 20;
    ui_g_Ungroup_boardCom->str_length = 9;
    strcpy(ui_g_Ungroup_boardCom->string, "BOARD_COM");

    ui_g_Ungroup_vtm->figure_type = 7;
    ui_g_Ungroup_vtm->operate_type = 1;
    ui_g_Ungroup_vtm->layer = 6;
    ui_g_Ungroup_vtm->color = 1;
    ui_g_Ungroup_vtm->start_x = 329;
    ui_g_Ungroup_vtm->start_y = 645;
    ui_g_Ungroup_vtm->width = 2;
    ui_g_Ungroup_vtm->font_size = 20;
    ui_g_Ungroup_vtm->str_length = 7;
    strcpy(ui_g_Ungroup_vtm->string, "VTM_COM");

    ui_g_Ungroup_follow->figure_type = 7;
    ui_g_Ungroup_follow->operate_type = 1;
    ui_g_Ungroup_follow->layer = 6;
    ui_g_Ungroup_follow->color = 1;
    ui_g_Ungroup_follow->start_x = 348;
    ui_g_Ungroup_follow->start_y = 609;
    ui_g_Ungroup_follow->width = 2;
    ui_g_Ungroup_follow->font_size = 20;
    ui_g_Ungroup_follow->str_length = 6;
    strcpy(ui_g_Ungroup_follow->string, "FOLLOW");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
