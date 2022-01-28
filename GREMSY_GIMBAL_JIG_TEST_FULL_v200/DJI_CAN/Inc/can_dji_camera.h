/**
  ******************************************************************************
  * @file:   gs_can_camera.h 
  * @author: Gremsy Team
  * @version: v1.0
  * @date:    12/05/2018
  * @brief:    Decode can data of the camera 
  * @company: GREMSY Co., Ltd
  ******************************************************************************
  *
  *
*/

#ifndef __GS_CAN_DJI_CAMERA_H
#define __GS_CAN_DJI_CAMERA_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
/* Includes ------------------------------------------------------------------*/
#include "can_dji_protocol.h"
/* Private typedef -----------------------------------------------------------*/
/**
 * @brief -- Camera - Camera Shutter Cmd - 0x7c
 *55	0F	4	A2	9	1	17	0	20	2	7C
 */
typedef struct _can_dji_cam_shutter_cmd_t
{
    uint8_t shutter_type;
}_can_dji_cam_shutter_cmd_t;


/**
 * @brief - Camera - Camera State Info - 0x80
 *
 */
typedef struct __can_dji_cam_state_info_t
{
	uint32_t masked00;
	uint32_t connect_state;
	uint32_t usb_state;
	uint32_t time_sync_state;
	uint32_t photo_state;
	uint32_t record_state;
	uint32_t sensor_state;
	uint32_t sd_card_insert_state;
	uint32_t sd_card_state;
	uint32_t firm_upgrade_state;
	uint32_t firm_upgrade_error_state;
	uint32_t hot_state;
	uint32_t not_enabled_photo;
	uint32_t is_storing;
	uint32_t is_time_photoing;
	uint32_t encrypt_status;
	uint32_t is_gimbal_busy;
	uint32_t in_tracking_mode;
	uint8_t mode;
	uint32_t sd_card_total_size;
	uint32_t sd_card_free_size;
	uint32_t remained_shots;
	uint32_t remained_time;
	uint8_t file_index_mode;
	uint8_t fast_play_back_info;
	uint8_t fast_play_back_enabled;
	uint8_t fast_play_back_time;
	uint16_t photo_osd_info;
	uint16_t photo_osd_time_is_show;
	uint16_t photo_osd_aperture_is_show;
	uint16_t photo_osd_shutter_is_show;
	uint16_t photo_osd_iso_is_show;
	uint16_t photo_osd_exposure_is_show;
	uint16_t photo_osd_sharpe_is_show;
	uint16_t photo_osd_contrast_is_show;
	uint16_t photo_osd_saturation_is_show;
	uint8_t unknown19;
	uint8_t in_debug_mode;
	uint8_t unknown1c;
	uint16_t video_record_time;
	uint8_t max_photo_num;
	uint8_t masked20;
	uint8_t histogram_enable;
	uint8_t camera_type;
	uint8_t unknown22;
	uint8_t version;
} can_dji_cam_state_info_t;

/**
 * @brief -- Camera - Camera Shot Params - 0x81
 *C: Send:1, r:2, cmdSet:2, cmd:129, len:104, crc:150
 */
typedef struct _gs_can_dji_cam_shot_param
{
	uint16_t aperture_size;
	uint16_t user_shutter;
	uint8_t shutter_speed_decimal;
	uint8_t iso;
	uint8_t exposure_compensation;
	uint8_t ctr_object_for_one;
	uint8_t ctr_object_for_two;
	uint8_t image_size;
	uint8_t image_ratio;
	uint8_t image_quality;
	uint8_t image_format;
	uint8_t video_format;
	uint8_t video_fps;
	uint8_t video_fov;
	uint8_t video_second_open;
	uint8_t video_second_ratio;
	uint8_t video_quality;
	uint8_t video_store_format;
	uint8_t exposure_mode;
	uint8_t scene_mode;
	uint8_t metering;
	uint8_t white_balance;
	uint8_t color_temp;
	uint8_t mctf_enable;
	uint8_t mctf_strength;
	uint8_t sharpe;
	uint8_t contrast;
	uint8_t saturation;
	uint8_t tonal;
	uint8_t digital_filter;
	uint8_t anti_flicker;
	uint8_t continuous;
	uint8_t time_params_type;
	uint8_t time_params_num;
	uint16_t time_params_period;
	uint16_t real_aperture_size;
	uint16_t real_shutter;
	uint8_t rel_shutter_speed_decimal;
	uint32_t rel_iso;
	uint8_t rel_exposure_compensation;
	uint8_t time_countdown;
	uint16_t cap_min_shutter;
	uint8_t cap_min_shutter_decimal;
	uint16_t cap_max_shutter;
	uint8_t cap_max_shutter_decimal;
	uint8_t video_standard;
	uint8_t ae_lock;
	uint8_t photo_type;
	uint8_t spot_area_bottom_right_pos;
	uint8_t unknown3b;
	uint8_t aeb_number;
	uint8_t pano_mode;
	uint16_t cap_min_aperture;
	uint16_t cap_max_aperture;
	uint8_t auto_turn_off_fore_led;
	uint8_t exposure_status;
	uint8_t locked_gimbal_when_shot;
	uint8_t encode_types;
	uint8_t not_auto_ae_unlock;
	uint8_t unknown47;
	uint8_t constrast_ehance;
	uint8_t video_record_mode;
	uint8_t timelapse_save_type;
	uint16_t video_record_interval_time;
	uint32_t timelapse_duration;
	uint16_t timelapse_time_count_down;
	uint32_t timelapse_recorded_frame;
	uint16_t optics_scale;
	uint16_t digital_zoom_scale;
}gs_can_dji_cam_shot_params_t;

/**
 * @brief -- Camera - Camera Play Back Params - 0x82
 *55	36	4	3D	1	2	BA	7	0	2	82
 */
typedef struct _gs_can_dji_cam_play_back_params
{
    uint8_t mode;
    uint16_t file_type;
    uint8_t file_num;
    uint16_t total_num;
    uint16_t index;
    uint8_t progress;
    uint16_t total_time;
    uint16_t current;
    uint16_t delete_chioce_num;
    uint16_t zoom_size;
    uint16_t total_photo_num;
    uint16_t total_video_num;
    uint32_t photo_width;
    uint32_t photo_height;
    uint32_t center_x;
    uint32_t center_y;
    uint8_t cur_page_selected;
    uint8_t del_file_status;
    uint8_t not_select_file_valid;
    uint8_t single_downloaded;
} can_dji_cam_play_back_params_t;

/**
 * @brief -- Camera - Camera Shot Info - 0x87
 *55	35	4	68	1	2	CE	7	0	2	87
 */
typedef struct _can_dji_cam_shot_info
{
    uint8_t masked00;
    uint8_t fuselage_focus_mode;
    uint8_t shot_focus_mode;
    uint8_t zoom_focus_type;
    uint8_t shot_type;
    uint8_t shot_fd_type;
    uint8_t shot_connected;
    uint16_t shot_focus_max_stroke;
    uint16_t shot_focus_cur_stroke;
    float obj_distance;
    uint16_t min_aperture;
    uint16_t max_aperture;
    float spot_af_axis_x;
    float spot_af_axis_y;
    uint8_t masked15;
    uint8_t focus_status;
    uint8_t mf_focus_probability;
    uint16_t min_focus_distance;
    uint16_t max_focus_distance;
    uint16_t cur_focus_distance;
    uint16_t min_focus_distance_step;
    uint8_t masked1f;
    uint8_t digital_focus_m_enable;
    uint8_t digital_focus_a_enable;
    uint8_t x_axis_focus_window_num;
    uint8_t y_axis_focus_window_num;
    uint8_t mf_focus_status;
    uint8_t focus_window_start_x;
    uint8_t focus_window_real_num_x;
    uint8_t focus_window_start_y;
    uint8_t focus_window_real_num_y;
    uint8_t support_type;
} can_dji_cam_shot_info_t;

/**
 * @brief -- Camera - Camera Recording Name - 0x84
 *55	1E	4	8A	1	2	1C	0F	20	2	84
 */
typedef struct _can_dji_cam_recording_name
{
    uint8_t     file_type;
    uint32_t    name_index;
    uint64_t     name_size;
    uint32_t    name_time;
} can_dji_cam_recording_name;

/**
 * @brief -- Camera - structure
 *
 */
typedef struct gs_can_dji_cam_t
{
	/** Shot params */
	gs_can_dji_cam_shot_params_t	shot_params;
	
	uint16_t						*camera_mode;
	can_dji_cam_state_info_t		*cam_state_info;
	uint8_t							*cam_iso;
	float							*cam_shutter;
}gs_can_dji_cam_t;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void gs_can_handle_camera(gs_can_message_t* packet, gs_can_dji_cam_t *cam);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GS_CAN_DJI_CAMERA_H */
/************************ GREMSY Co., Ltd *****END OF FILE****/
