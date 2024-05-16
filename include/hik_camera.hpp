#include <MvCameraControl.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

/*
Head-only camera wapper library for hikvision industry camera.
*/

struct ImageFrame {
  std::vector<uint8_t> imageData;
  unsigned int width, height, channels;
  std::string format;
  ImageFrame(std::vector<uint8_t>&& data, int w, int h, int c, std::string f)
      : imageData(std::move(data)),
        width(w),
        height(h),
        channels(c),
        format(f) {}
};

std::unordered_map<int, std::string> errorMessages = {
    {MV_E_HANDLE, "Invalid or erroneous handle"},
    {MV_E_SUPPORT, "Unsupported feature"},
    {MV_E_BUFOVER, "Buffer overflow"},
    {MV_E_CALLORDER, "Incorrect function call order"},
    {MV_E_PARAMETER, "Invalid parameter"},
    {MV_E_RESOURCE, "Resource allocation failure"},
    {MV_E_NODATA, "No data available"},
    {MV_E_PRECONDITION,
     "Precondition violation or runtime environment changed"},
    {MV_E_VERSION, "Version mismatch"},
    {MV_E_NOENOUGH_BUF, "Insufficient memory buffer"},
    {MV_E_ABNORMAL_IMAGE, "Abnormal image, possibly due to packet loss"},
    {MV_E_LOAD_LIBRARY, "Failed to dynamically load DLL"},
    {MV_E_NOOUTBUF, "No output buffer available"},
    {MV_E_UNKNOW, "Unknown error"},

    // GenICam
    {MV_E_GC_GENERIC, "Generic error"},
    {MV_E_GC_ARGUMENT, "Invalid argument"},
    {MV_E_GC_RANGE, "Value out of range"},
    {MV_E_GC_PROPERTY, "Property related error"},
    {MV_E_GC_RUNTIME, "Runtime environment issue"},
    {MV_E_GC_LOGICAL, "Logical error"},
    {MV_E_GC_ACCESS, "Incorrect node access condition"},
    {MV_E_GC_TIMEOUT, "Timeout"},
    {MV_E_GC_DYNAMICCAST, "Dynamic cast exception"},
    {MV_E_GC_UNKNOW, "Unknown GenICam error"},

    // GigE
    {MV_E_NOT_IMPLEMENTED, "Command not supported by the device"},
    {MV_E_INVALID_ADDRESS, "Target address does not exist"},
    {MV_E_WRITE_PROTECT, "Target address is read-only"},
    {MV_E_ACCESS_DENIED, "Access to the device is denied"},
    {MV_E_BUSY, "Device is busy or network disconnected"},
    {MV_E_PACKET, "Network packet data error"},
    {MV_E_NETER, "Network-related error"},
    {MV_E_IP_CONFLICT, "Device IP conflict"},

    // USB
    {MV_E_USB_READ, "Read USB error"},
    {MV_E_USB_WRITE, "Write USB error"},
    {MV_E_USB_DEVICE, "USB device exception"},
    {MV_E_USB_GENICAM, "GenICam related USB error"},
    {MV_E_USB_BANDWIDTH, "Insufficient USB bandwidth"},
    {MV_E_USB_DRIVER, "USB driver mismatch or not installed"},
    {MV_E_USB_UNKNOW, "Unknown USB error"},
};

std::pair<std::string, int> convertToRosEncodingWithChannels(
    MvGvspPixelType pixelType) {
  switch (pixelType) {
    case PixelType_Gvsp_Mono8:
    case PixelType_Gvsp_Mono8_Signed:
      return std::make_pair("mono8", 1);
    case PixelType_Gvsp_Mono16:
      return std::make_pair("mono16", 1);
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
      return std::make_pair("bayer_rggb8", 1);
    case PixelType_Gvsp_RGB8_Packed:
      return std::make_pair("rgb8", 3);
    case PixelType_Gvsp_BGR8_Packed:
      return std::make_pair("bgr8", 3);
    case PixelType_Gvsp_RGBA8_Packed:
      return std::make_pair("rgba8", 4);
    case PixelType_Gvsp_BGRA8_Packed:
      return std::make_pair("bgra8", 4);
    default:
      return std::make_pair("unknown", 0);
  }
}

namespace HikvisionCameraWrapper {
class NoOutBufException : public std::runtime_error {
 public:
  NoOutBufException() : std::runtime_error("No output buffer available") {}
};

class HikvisionIndustrialCamera {
 private:
  void* handle_;
  std::string interfaceType_;
  std::string device_serial_;
  MV_IMAGE_BASIC_INFO info_{0};
  MV_FRAME_OUT img_buffer_{0};
  MV_CC_PIXEL_CONVERT_PARAM convert_param_{0};

  std::mutex mtx_;

 public:
  HikvisionIndustrialCamera(const std::string _interfaceType,
                            const std::string _device_serial);
  ~HikvisionIndustrialCamera();
  void initialize();
  void release();
  void startCapture();
  void stopCapture();
  std::unique_ptr<ImageFrame> captureImage();
  void setExposureTime(unsigned int _timeUs);
  unsigned int getExposureTime();
  void getExposureRange(unsigned int &min, unsigned int &max);
  void setExposureAuto(bool _isAuto);
  bool getExposureAuto();
  void setTriggerMode(const std::string& _mode);
  void getTriggerMode(std::string& _mode);
  bool getGainAuto();
  void setGainAuto(bool _isAuto);
  unsigned int getGain();
  void getGainRange(unsigned int &min, unsigned int &max);
  void setGain(unsigned int _gain);
  void setGammaEnable(bool _isEnable);
  bool getGammaEnable();
  unsigned int getGamma();
  void getGammaRange(unsigned int &min, unsigned int &max);
  void setGamma(unsigned int _gamma);
  void checkError(unsigned int _result, const std::string& _operation);
};

}  // namespace HikvisionCameraWrapper

using hik_camera = HikvisionCameraWrapper::HikvisionIndustrialCamera;

// Initialize first camera when device serial is empty.
hik_camera::HikvisionIndustrialCamera(const std::string _interfaceType,
                                      const std::string _device_serial = "")
    : interfaceType_(_interfaceType), device_serial_(_device_serial) {}

hik_camera::~HikvisionIndustrialCamera() {
  release();
}

void hik_camera::initialize() {
  std::lock_guard<std::mutex> lck(mtx_);

  unsigned int interfaceType;
  if (interfaceType_ == "GigE") {
    interfaceType = MV_GIGE_DEVICE;
  } else if (interfaceType_ == "USB") {
    interfaceType = MV_USB_DEVICE;
  } else {
    throw std::runtime_error("Invalid interface type.");
  }

  MV_CC_DEVICE_INFO_LIST device_list;
  checkError(MV_CC_EnumDevices(interfaceType, &device_list), "Enum camera");
  if (device_list.nDeviceNum == 0) {
    throw std::runtime_error("Camera not found.");
  }

  int specific_id{-1};
  for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
    auto& device = device_list.pDeviceInfo[i];
    if (device->nTLayerType == interfaceType) {
      const unsigned char* serial_number = nullptr;
      if (interfaceType_ == "GigE") {
        serial_number = device->SpecialInfo.stGigEInfo.chSerialNumber;
      } else if (interfaceType_ == "USB") {
        serial_number = device->SpecialInfo.stUsb3VInfo.chSerialNumber;
      }
      if (serial_number && device_serial_.compare(reinterpret_cast<const char*>(
                               serial_number)) == 0) {
        specific_id = i;
        break;
      }
    }
  }

  if (device_serial_.empty()) {
    checkError(MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]),
               "Create camera handle");
    checkError(MV_CC_OpenDevice(handle_), "Open camera");
  }

  if (specific_id != -1 && !device_serial_.empty()) {
    checkError(
        MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[specific_id]),
        "Create camera handle");
    checkError(MV_CC_OpenDevice(handle_), "Open camera");
  }

  if (specific_id == -1 && !device_serial_.empty()) {
    throw std::runtime_error("Specific id not found when opening camera.");
  }
}

void hik_camera::release() {
  std::lock_guard<std::mutex> lck(mtx_);
  if (handle_ != nullptr) {
    checkError(MV_CC_CloseDevice(handle_), "Close device");
    checkError(MV_CC_DestroyHandle(handle_), "Destroy handle");
  }
}

void hik_camera::startCapture() {
  std::lock_guard<std::mutex> lck(mtx_);
  checkError(MV_CC_StartGrabbing(handle_), "Start grabbing");
  checkError(MV_CC_GetImageInfo(handle_, &info_), "Get image information");
  convert_param_.nWidth = info_.nWidthValue;
  convert_param_.nHeight = info_.nHeightValue;

  // Some cameras default trigger mode is on
  MVCC_ENUMVALUE trigger_mode{0};
  MV_CC_GetEnumValue(handle_, "TriggerMode", &trigger_mode);
  if (trigger_mode.nCurValue == MV_TRIGGER_MODE_ON) {
    checkError(MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF),
               "Set trigger mode off");
  }
}

void hik_camera::stopCapture() {
  std::lock_guard<std::mutex> lck(mtx_);
  checkError(MV_CC_StopGrabbing(handle_), "Stop grabbing");
}

std::unique_ptr<ImageFrame> hik_camera::captureImage() {
  std::lock_guard<std::mutex> lck(mtx_);

  std::vector<unsigned char> imageData;

  // Get image buffer
  try {
    checkError(MV_CC_GetImageBuffer(handle_, &img_buffer_, 1000),
               "Get image buffer");
  } catch (const HikvisionCameraWrapper::NoOutBufException& e) {
    MV_CC_FreeImageBuffer(handle_, &img_buffer_);
    MV_CC_GetImageBuffer(handle_, &img_buffer_, 1000);
  } catch (const std::runtime_error& e) {
    throw e;
  }

  // Convert image pixel type
  auto encodingAndChannels =
      convertToRosEncodingWithChannels(convert_param_.enSrcPixelType);
  auto encoding = encodingAndChannels.first;
  auto channels = encodingAndChannels.second;
  imageData.resize(channels * info_.nWidthValue * info_.nHeightValue);

  convert_param_.pSrcData = img_buffer_.pBufAddr;
  convert_param_.nSrcDataLen = img_buffer_.stFrameInfo.nFrameLen;
  convert_param_.enSrcPixelType = img_buffer_.stFrameInfo.enPixelType;
  convert_param_.enDstPixelType = img_buffer_.stFrameInfo.enPixelType;
  convert_param_.pDstBuffer = imageData.data();
  convert_param_.nDstBufferSize = imageData.size();
  checkError(MV_CC_ConvertPixelType(handle_, &convert_param_),
             "Convert pixel type");

  checkError(MV_CC_FreeImageBuffer(handle_, &img_buffer_), "Free image buffer");
  return std::make_unique<ImageFrame>(std::move(imageData), info_.nWidthValue,
                                      info_.nHeightValue, channels, encoding);
}

void hik_camera::setExposureTime(unsigned int _timeUs) {
  std::lock_guard<std::mutex> lck(mtx_);
  checkError(
      MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(_timeUs)),
      "Set exposure time");
}

unsigned int hik_camera::getExposureTime() {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_FLOATVALUE exposure_time{0};
  checkError(MV_CC_GetFloatValue(handle_, "ExposureTime", &exposure_time),
             "Get exposure time");
  return static_cast<unsigned int>(exposure_time.fCurValue);
}

void hik_camera::getExposureRange(unsigned int &min, unsigned int &max) {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_FLOATVALUE exposure_time{0};
  checkError(MV_CC_GetFloatValue(handle_, "ExposureTime", &exposure_time),
             "Get exposure time range");
  min = exposure_time.fMin;
  max = exposure_time.fMax;
}

void hik_camera::setExposureAuto(bool _isAuto) {
  std::lock_guard<std::mutex> lck(mtx_);
  if (!_isAuto) {
    checkError(MV_CC_SetEnumValue(handle_, "ExposureAuto", 0),
               "Set exposure auto");
  } else {
    checkError(MV_CC_SetEnumValue(handle_, "ExposureAuto", 2),
               "Set exposure auto");
  }
}

bool hik_camera::getExposureAuto() {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_ENUMVALUE exposure_auto{0};
  checkError(MV_CC_GetEnumValue(handle_, "ExposureAuto", &exposure_auto),
             "Get exposure auto");
  if (exposure_auto.nCurValue == 2) {
    return true;
  }
  return false;
}

void hik_camera::getTriggerMode(std::string& _mode) {
  std::lock_guard<std::mutex> lck(mtx_);
}

void hik_camera::setTriggerMode(const std::string& _mode) {
  std::lock_guard<std::mutex> lck(mtx_);
}

bool hik_camera::getGainAuto() {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_ENUMVALUE gain_auto{0};
  checkError(MV_CC_GetEnumValue(handle_, "GainAuto", &gain_auto),
             "Get gain auto");
  if (gain_auto.nCurValue == 2) {
    return true;
  }
  return false;
}

void hik_camera::setGainAuto(bool _isAuto) {
  std::lock_guard<std::mutex> lck(mtx_);
  if (!_isAuto) {
    checkError(MV_CC_SetEnumValue(handle_, "GainAuto", 0), "Set gain auto");
  } else {
    checkError(MV_CC_SetEnumValue(handle_, "GainAuto", 2), "Set gain auto");
  }
}

unsigned int hik_camera::getGain() {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_FLOATVALUE gain{0};
  checkError(MV_CC_GetFloatValue(handle_, "Gain", &gain), "Get gain");
  return static_cast<unsigned int>(gain.fCurValue);
}

void hik_camera::getGainRange(unsigned int &min, unsigned int &max) {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_FLOATVALUE gain{0};
  checkError(MV_CC_GetFloatValue(handle_, "Gain", &gain),
             "Get gain range");
  min = gain.fMin;
  max = gain.fMax;
}

void hik_camera::setGain(unsigned int _gain) {
  std::lock_guard<std::mutex> lck(mtx_);
  checkError(MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(_gain)),
             "Set gain");
}

void hik_camera::setGammaEnable(bool _isEnable) {
  std::lock_guard<std::mutex> lck(mtx_);
  checkError(MV_CC_SetBoolValue(handle_, "GammaEnable", _isEnable),
             "Set gamma auto");
}

bool hik_camera::getGammaEnable() {
  bool gamma_enable{false};
  checkError(MV_CC_GetBoolValue(handle_, "GammaEnable", &gamma_enable),
             "Get gamma enable");
  return gamma_enable;
}

unsigned int hik_camera::getGamma() {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_FLOATVALUE gamma{0};
  checkError(MV_CC_GetFloatValue(handle_, "Gamma", &gamma), "Get gamma");
  return static_cast<unsigned int>(gamma.fCurValue);
}

void hik_camera::getGammaRange(unsigned int &min, unsigned int &max) {
  std::lock_guard<std::mutex> lck(mtx_);
  MVCC_FLOATVALUE gamma{0};
  checkError(MV_CC_GetFloatValue(handle_, "Gamma", &gamma),
             "Get gamma");
  min = gamma.fMin;
  max = gamma.fMax;
}

void hik_camera::setGamma(unsigned int _gamma) {
  std::lock_guard<std::mutex> lck(mtx_);
  checkError(MV_CC_SetFloatValue(handle_, "Gamma", static_cast<float>(_gamma)),
             "Set gamma");
}

void hik_camera::checkError(unsigned int _result,
                            const std::string& _operation) {
  if (_result != MV_OK) {
    if (_result == MV_E_NOOUTBUF) {
      throw HikvisionCameraWrapper::NoOutBufException();
    }
    auto it = errorMessages.find(_result);
    if (it != errorMessages.end()) {
      throw std::runtime_error(_operation + " failed with " + it->second);
    } else {
      throw std::runtime_error(_operation + " failed.");
    }
  }
}