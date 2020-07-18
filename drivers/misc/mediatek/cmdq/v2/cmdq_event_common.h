/* MDP start frame */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RDMA0_SOF, 0, mdp_rdma0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RDMA1_SOF, 1, mdp_rdma1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RSZ0_SOF, 2, mdp_rsz0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RSZ1_SOF, 3, mdp_rsz1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RSZ2_SOF, 4, mdp_rsz2_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_TDSHP_SOF, 5, mdp_tdshp_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_TDSHP0_SOF, 6, mdp_tdshp0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_TDSHP1_SOF, 7, mdp_tdshp1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WDMA_SOF, 8, mdp_wdma_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT_SOF, 9, mdp_wrot_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT0_SOF, 10, mdp_wrot0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT1_SOF, 11, mdp_wrot1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_COLOR_SOF, 12, mdp_color_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_MVW_SOF, 13, mdp_mvw_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_CROP_SOF, 14, mdp_crop_sof)

/* Display start frame */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_OVL0_SOF, 15, disp_ovl0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_OVL1_SOF, 16, disp_ovl1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_2L_OVL0_SOF, 17, disp_2l_ovl0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_2L_OVL1_SOF, 18, disp_2l_ovl1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA0_SOF, 19, disp_rdma0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA1_SOF, 20, disp_rdma1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA2_SOF, 21, disp_rdma2_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_WDMA0_SOF, 22, disp_wdma0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_WDMA1_SOF, 23, disp_wdma1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_COLOR_SOF, 24, disp_color_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_COLOR0_SOF, 25, disp_color0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_COLOR1_SOF, 26, disp_color1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_CCORR_SOF, 27, disp_ccorr_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_AAL_SOF, 28, disp_aal_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_GAMMA_SOF, 29, disp_gamma_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DITHER_SOF, 30, disp_dither_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_UFOE_SOF, 31, disp_ufoe_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_PWM0_SOF, 32, disp_pwm0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_PWM1_SOF, 33, disp_pwm1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_OD_SOF, 34, disp_od_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DSC_SOF, 35, disp_dsc_sof)

DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L0_SOF, 36, ufod_rdma0_l0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L1_SOF, 37, ufod_rdma0_l1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L2_SOF, 38, ufod_rdma0_l2_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L3_SOF, 39, ufod_rdma0_l3_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L0_SOF, 40, ufod_rdma1_l0_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L1_SOF, 41, ufod_rdma1_l1_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L2_SOF, 42, ufod_rdma1_l2_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L3_SOF, 43, ufod_rdma1_l3_sof)

/* MDP frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RDMA0_EOF, 44, mdp_rdma0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RDMA1_EOF, 45, mdp_rdma1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RSZ0_EOF, 46, mdp_rsz0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RSZ1_EOF, 47, mdp_rsz1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_RSZ2_EOF, 48, mdp_rsz2_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_TDSHP_EOF, 49, mdp_tdshp_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_TDSHP0_EOF, 50, mdp_tdshp0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_TDSHP1_EOF, 51, mdp_tdshp1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WDMA_EOF, 52, mdp_wdma_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT_WRITE_EOF, 53, mdp_wrot_write_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT_READ_EOF, 54, mdp_wrot_read_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT0_WRITE_EOF, 55, mdp_wrot0_write_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT0_READ_EOF, 56, mdp_wrot0_read_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT1_WRITE_EOF, 57, mdp_wrot1_write_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT1_READ_EOF, 58, mdp_wrot1_read_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT0_W_EOF, 59, mdp_wrot0_write_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT0_R_EOF, 60, mdp_wrot0_read_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT1_W_EOF, 61, mdp_wrot1_write_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_WROT1_R_EOF, 62, mdp_wrot1_read_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_COLOR_EOF, 63, mdp_color_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_CROP_EOF, 64, mdp_crop_frame_done)

/* Display frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_OVL0_EOF, 65, disp_ovl0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_OVL1_EOF, 66, disp_ovl1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_2L_OVL0_EOF, 67, disp_2l_ovl0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_2L_OVL1_EOF, 68, disp_2l_ovl1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA0_EOF, 69, disp_rdma0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA1_EOF, 70, disp_rdma1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA2_EOF, 71, disp_rdma2_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_WDMA0_EOF, 72, disp_wdma0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_WDMA1_EOF, 73, disp_wdma1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_COLOR_EOF, 74, disp_color_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_COLOR0_EOF, 75, disp_color0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_COLOR1_EOF, 76, disp_color1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_CCORR_EOF, 77, disp_ccorr_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_AAL_EOF, 78, disp_aal_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_GAMMA_EOF, 79, disp_gamma_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DITHER_EOF, 80, disp_dither_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_UFOE_EOF, 81, disp_ufoe_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_OD_EOF, 82, disp_od_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DSC_EOF, 83, disp_dsc_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DSI0_EOF, 84, disp_dsi0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DSI1_EOF, 85, disp_dsi1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_DPI0_EOF, 86, disp_dpi0_frame_done)

DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L0_EOF, 87, ufod_rdma0_l0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L1_EOF, 88, ufod_rdma0_l1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L2_EOF, 89, ufod_rdma0_l2_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA0_L3_EOF, 90, ufod_rdma0_l3_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L0_EOF, 91, ufod_rdma1_l0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L1_EOF, 92, ufod_rdma1_l1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L2_EOF, 93, ufod_rdma1_l2_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_UFOD_RAMA1_L3_EOF, 94, ufod_rdma1_l3_frame_done)

/* Mutex frame done */
/* DISPSYS */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX0_STREAM_EOF, 95, stream_done_0)
/* DISPSYS */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX1_STREAM_EOF, 96, stream_done_1)
/* DISPSYS */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX2_STREAM_EOF, 97, stream_done_2)
/* DISPSYS */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX3_STREAM_EOF, 98, stream_done_3)
/* DISPSYS, please refer to disp_hal.h */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX4_STREAM_EOF, 99, stream_done_4)
/* DpFramework */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX5_STREAM_EOF, 100, stream_done_5)
/* DpFramework */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX6_STREAM_EOF, 101, stream_done_6)
/* DpFramework */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX7_STREAM_EOF, 102, stream_done_7)
/* DpFramework */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX8_STREAM_EOF, 103, stream_done_8)
/* DpFramework via CMDQ_IOCTL_LOCK_MUTEX */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MUTEX9_STREAM_EOF, 104, stream_done_9)

/* Display underrun */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA0_UNDERRUN, 105, buf_underrun_event_0)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA1_UNDERRUN, 106, buf_underrun_event_1)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DISP_RDMA2_UNDERRUN, 107, buf_underrun_event_2)

/* Display TE */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DSI_TE, 108, dsi0_te_event)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DSI0_TE, 109, dsi0_te_event)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DSI1_TE, 110, dsi1_te_event)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_DSI0_TE_SOF, 111, mdp_dsi0_te_sof)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_MDP_DSI1_TE_SOF, 112, mdp_dsi1_te_sof)

/* ISP frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_PASS2_2_EOF, 113, isp_frame_done_p2_2)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_PASS2_1_EOF, 114, isp_frame_done_p2_1)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_PASS2_0_EOF, 115, isp_frame_done_p2_0)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_PASS1_1_EOF, 116, isp_frame_done_p1_1)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_PASS1_0_EOF, 117, isp_frame_done_p1_0)

/* ISP (IMGSYS) frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD0_EOF, 118, dip_cq_thread0_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD1_EOF, 119, dip_cq_thread1_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD2_EOF, 120, dip_cq_thread2_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD3_EOF, 121, dip_cq_thread3_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD4_EOF, 122, dip_cq_thread4_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD5_EOF, 123, dip_cq_thread5_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD6_EOF, 124, dip_cq_thread6_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD7_EOF, 125, dip_cq_thread7_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD8_EOF, 126, dip_cq_thread8_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD9_EOF, 127, dip_cq_thread9_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD10_EOF, 128, dip_cq_thread10_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD11_EOF, 129, dip_cq_thread11_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD12_EOF, 130, dip_cq_thread12_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD13_EOF, 131, dip_cq_thread13_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DIP_CQ_THREAD14_EOF, 132, dip_cq_thread14_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_DPE_EOF, 133, dpe_frame_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_WMF_EOF, 134, wmf_frame_done)

/* ISP (IMGSYS) engine events */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_SENINF_CAM1_2_3_FULL, 135, seninf_cam1_2_3_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_SENINF_CAM0_FULL, 136, seninf_cam0_fifo_full)

/* VENC frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_VENC_EOF, 137, venc_done)

/* JPEG frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_JPEG_ENC_EOF, 138, jpgenc_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_JPEG_ENC_PASS2_EOF, 139, jpgenc_pass2_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_JPEG_ENC_PASS1_EOF, 140, jpgenc_pass1_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_JPEG_DEC_EOF, 141, jpgdec_done)

/* VENC engine events */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_VENC_MB_DONE, 142, venc_mb_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_VENC_128BYTE_CNT_DONE, 143, venc_128byte_cnt_done)

/* ISP (CAMSYS) frame done */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_FRAME_DONE_A, 144, isp_frame_done_a)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_FRAME_DONE_B, 145, isp_frame_done_b)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_CAMSV_0_PASS1_DONE, 146, camsv_0_pass1_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_CAMSV_1_PASS1_DONE, 147, camsv_1_pass1_done)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_ISP_CAMSV_2_PASS1_DONE, 148, camsv_2_pass1_done)

/* ISP (CAMSYS) engine events */
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_0_FIFO_FULL, 149, seninf_0_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_1_FIFO_FULL, 150, seninf_1_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_2_FIFO_FULL, 151, seninf_2_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_3_FIFO_FULL, 152, seninf_3_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_4_FIFO_FULL, 153, seninf_4_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_5_FIFO_FULL, 154, seninf_5_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_6_FIFO_FULL, 155, seninf_6_fifo_full)
DECLARE_CMDQ_EVENT(CMDQ_EVENT_SENINF_7_FIFO_FULL, 156, seninf_7_fifo_full)

/* Keep this at the end of HW events */
DECLARE_CMDQ_EVENT(CMDQ_MAX_HW_EVENT_COUNT, 400, hw_event_conunt)

/* SW Sync Tokens (Pre-defined) */
/* Config thread notify trigger thread */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_CONFIG_DIRTY, 401, sw_token)
/* Trigger thread notify config thread */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_STREAM_EOF, 402, sw_token)
/* Block Trigger thread until the ESD check finishes. */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_ESD_EOF, 403, sw_token)
/* check CABC setup finish */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_CABC_EOF, 404, sw_token)
/* Block Trigger thread until the path freeze finishes */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_FREEZE_EOF, 405, sw_token)
/* Pass-2 notifies VENC frame is ready to be encoded */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_VENC_INPUT_READY, 406, sw_token)
/* VENC notifies Pass-2 encode done so next frame may start */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_VENC_EOF, 407, sw_token)

/* Notify normal CMDQ there are some secure task done */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_SECURE_THR_EOF, 408, sw_token)
/* Lock WSM resource */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_SECURE_WSM_LOCK, 409, sw_token)

/* SW Sync Tokens (User-defined) */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_USER_0, 410, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_USER_1, 411, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_POLL_MONITOR, 412, sw_token)

/* Event for CMDQ to block executing command when append command
* Plz sync CMDQ_SYNC_TOKEN_APPEND_THR(id) in cmdq_core source file. */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR0, 422, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR1, 423, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR2, 424, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR3, 425, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR4, 426, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR5, 427, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR6, 428, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR7, 429, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR8, 430, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR9, 431, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR10, 432, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR11, 433, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR12, 434, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR13, 435, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR14, 436, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_APPEND_THR15, 437, sw_token)

/* GPR access tokens (for HW register backup) */
/* There are 15 32-bit GPR, 3 GPR form a set (64-bit for address, 32-bit for value) */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_GPR_SET_0, 450, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_GPR_SET_1, 451, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_GPR_SET_2, 452, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_GPR_SET_3, 453, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_GPR_SET_4, 454, sw_token)

/* Resource lock event to control resource in GCE thread */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_RESOURCE_WROT0, 460, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_RESOURCE_WROT1, 461, sw_token)

/* Secure video path  notify SW token */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_OVL0_2NONSEC_END, 470, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_OVL1_2NONSEC_END, 471, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_2LOVL0_2NONSEC_END, 472, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_2LOVL1_2NONSEC_END, 473, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_RDMA0_2NONSEC_END, 474, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_RDMA1_2NONSEC_END, 475, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_WDMA0_2NONSEC_END, 476, sw_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_DISP_WDMA1_2NONSEC_END, 477, sw_token)
/* add for ESD check timeout*/
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_POLL_DSI_RD_RDY, 478, sw_token)

/* event id is 9 bit */
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_MAX, (0x1FF), max_token)
DECLARE_CMDQ_EVENT(CMDQ_SYNC_TOKEN_INVALID, (-1), invalid_token)
