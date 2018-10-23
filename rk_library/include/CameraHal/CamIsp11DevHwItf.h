#ifndef _CAM_ISP11_DEV_HW_ITF_IMC_H_
#define _CAM_ISP11_DEV_HW_ITF_IMC_H_
#include "CamHwItf.h"
#include "CamIspCtrItf.h"
using namespace std;

#define CAMERA_DEVICE_NAME              "/dev/video"
#define CAMERA_CAPTURE_DEV_NAME   "/dev/video2"
#define CAMERA_OVERLAY_DEV_NAME   "/dev/video0"
#define CAMERA_DMA_DEV_NAME   "/dev/video3"
#define CAMERA_ISP_DEV_NAME   "/dev/video1"
#define CAMERA_IQ_FIRST_DIR     "/tmp/"
#define CAMERA_IQ_SECOND_DIR  "/etc/cam_iq/"

#define CAMERAHAL_VIDEODEV_NONBLOCK

class CamIsp11CtrItf;

class CamIsp11DevHwItf: public CamHwItf {
 public:
  CamIsp11DevHwItf(struct rk_isp_dev_info* isp_dev_info = NULL);
  virtual ~CamIsp11DevHwItf(void);
  //derived interfaces from CamHwItf
  virtual shared_ptr<CamHwItf::PathBase> getPath(enum CamHwItf::PATHID id);
  virtual bool initHw(int inputId);
  virtual void deInitHw();

  //ISP dev  inerfaces
  virtual int setExposure(unsigned int vts, unsigned int exposure, unsigned int gain, unsigned int gain_percent);
  int SetAutoAdjustFps(bool auto_adjust_fps);
  virtual int setAutoAdjustFPS(bool on) {UNUSED_PARAM(on); return -1;}
  virtual bool configureISPModules(const void* config);
  virtual int getAeMeanLuma(int &meanLuma);
  virtual int getAeMaxGainRange(float &maxGainRange);
  virtual int SetAeMaxExposureTime(float time);
  virtual int SetAeMaxExposureGain(float gain);

  class Path: public CamHwItf::PathBase {
    friend class CamIsp11DevHwItf;
   public:
    virtual bool prepare(
        frm_info_t& frmFmt,
        unsigned int numBuffers,
        CameraBufferAllocator& allocator,
        bool cached,
        unsigned int minNumBuffersQueued = 1);

    virtual bool prepare(
        frm_info_t& frmFmt,
        list<shared_ptr<BufferBase> >& bufPool,
        unsigned int numBuffers,
        unsigned int minNumBuffersQueued = 1);

    virtual void addBufferNotifier(NewCameraBufferReadyNotifier* bufferReadyNotifier);
    virtual bool removeBufferNotifer(NewCameraBufferReadyNotifier* bufferReadyNotifier);
    virtual void releaseBuffers(void);
    virtual bool start(void);
    virtual void stop(void);
    virtual bool releaseBufToOwener(weak_ptr<BufferBase> camBuf);
    Path(CamIsp11DevHwItf* camIsp, V4L2DevIoctr* camDev, PATHID pathID, unsigned long dequeueTimeout = 1000);
    virtual ~Path(void);

   private:
    CamIsp11DevHwItf* mCamIsp;

  };

  virtual int setWhiteBalance(HAL_WB_MODE wbMode);
  virtual int setAeMode(enum HAL_AE_OPERATION_MODE aeMode);
  virtual int setAntiBandMode(enum HAL_AE_FLK_MODE flkMode);
  virtual int setAeBias(int aeBias);
  virtual int setManualGainAndTime(float hal_gain, float hal_time);
  virtual int setFps(HAL_FPS_INFO_t fps);
  virtual int setAeWindow(int left_hoff, int top_voff, int right_width, int bottom_height);
  virtual int getAeWindow(int &left_hoff, int &top_voff, int &right_width, int &bottom_height);
  virtual int setExposureMeterMode(enum HAL_AE_METERING_MODE aeMeterMode);
  virtual int getExposureMeterMode(enum HAL_AE_METERING_MODE& aeMeterMode);
  virtual int setExposureMeterCoeff(unsigned char meter_coeff[]);
  virtual int getExposureMeterCoeff(unsigned char meter_coeff[]);
  virtual int setAeSetPoint(float set_point);
  virtual int getAeSetPoint(float &set_point);
  //brightness
  virtual int getSupportedBtRange(HAL_RANGES_t& brightRange);
  virtual int setBrightness(int brightVal);
  virtual int getBrithtness(int& brightVal);
  //contrast
  virtual int getSupportedCtRange(HAL_RANGES_t& contrastRange);
  virtual int setContrast(int contrast);
  virtual int getContrast(int& contrast);
  //saturation
  virtual int getSupportedStRange(HAL_RANGES_t& saturationRange);
  virtual int setSaturation(int sat);
  virtual int getSaturation(int& sat);
  //hue
  virtual int getSupportedHueRange(HAL_RANGES_t& hueRange);
  virtual int setHue(int hue);
  virtual int getHue(int& hue);
  //af
  virtual int setFocusMode(enum HAL_AF_MODE fcMode);
  virtual int getFocusMode(enum HAL_AF_MODE& fcMode);
  virtual int setFocusWin(HAL_Window_t afwin);
  virtual int getFocusWin(HAL_Window_t& afwin);
  virtual int trigggerAf(bool trigger);
  //wdr
  virtual int setWdrMode(enum HAL_MODE_e mode);
  virtual int getWdrMode(enum HAL_MODE_e& mode);
  virtual int setWdrLevel(int level);
  virtual int getWdrLevel(int& level);
  virtual int setWdrCurve(HAL_ISP_WDR_MODE_e mode,
      unsigned short (&wdr_dy)[HAL_ISP_WDR_SECTION_MAX + 1]);
  virtual int getWdrCurve(HAL_ISP_WDR_MODE_e& mode,
      unsigned short (&wdr_dy)[HAL_ISP_WDR_SECTION_MAX + 1]);
  //3dnr
  virtual int set3DnrMode(enum HAL_MODE_e mode);
  virtual int get3DnrMode(enum HAL_MODE_e& mode);
  virtual int set3DnrLevel(struct HAL_3DnrLevelCfg& cfg);
  virtual int get3DnrLevel(struct HAL_3DnrLevelCfg& cfg);
  virtual int set3DnrParam(struct HAL_3DnrParamCfg& cfg);
  virtual int get3DnrParam(struct HAL_3DnrParamCfg& cfg);
  //filter
  virtual int setFilterLevel(enum HAL_MODE_e mode,
	enum HAL_FLT_DENOISE_LEVEL_e denoise, enum HAL_FLT_SHARPENING_LEVEL_e sharp);
  virtual int getFilterLevel(enum HAL_MODE_e& mode,
	enum HAL_FLT_DENOISE_LEVEL_e& denoise, enum HAL_FLT_SHARPENING_LEVEL_e& sharp);
 private:
  virtual void transDrvMetaDataToHal(const void* drvMeta, struct HAL_Buffer_MetaData* halMeta);
  virtual int configIsp(struct isp_supplemental_sensor_mode_data* sensor_mode_data, bool enable);
  int configIsp_l(struct isp_supplemental_sensor_mode_data* sensor);
  int setCprocDefault();
  unsigned int mExposureSequence;

  shared_ptr<CamIsp11CtrItf> mISPDev;
  CamIspCtrItf::Configuration mIspCfg;
  osMutex mApiLock;
  struct rk_isp_dev_info* mISPDevInfo;
  signed char mISPBrightness;
  float mISPContrast;
  float mISPSaturation;
  float mISPHue;
  char mIqPath[64];
};

#endif

