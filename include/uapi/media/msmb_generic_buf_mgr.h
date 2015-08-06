#ifndef __UAPI_MEDIA_MSMB_BUF_MNGR_H__
#define __UAPI_MEDIA_MSMB_BUF_MNGR_H__

#define VIDIOC_MSM_BUF_MNGR_GET_BUF \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 33, struct msm_buf_mngr_info *)

#define VIDIOC_MSM_BUF_MNGR_PUT_BUF \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 34, struct msm_buf_mngr_info *)

#define VIDIOC_MSM_BUF_MNGR_BUF_DONE \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 35, struct msm_buf_mngr_info *)

/* LGE_CHANGE_S, jaehan.jeong, 2013.12.29,  Cleanup msm generic buf queue handling, [STARTS HERE] */
#define VIDIOC_MSM_BUF_MNGR_INIT \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 36, struct msm_buf_mngr_info)

#define VIDIOC_MSM_BUF_MNGR_DEINIT \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 37, struct msm_buf_mngr_info)
/* LGE_CHANGE_E, jaehan.jeong, 2013.12.29,  Cleanup msm generic buf queue handling,  [ENDS HERE] */

struct msm_buf_mngr_info {
	uint32_t session_id;
	uint32_t stream_id;
	uint32_t index;
};

#endif
