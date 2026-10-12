#pragma once

#include <string>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"

void pandad_main_thread(std::string serial);

class PandaSafety {
public:
  PandaSafety(Panda *panda) : panda_(panda) {}
  void configureSafetyMode(bool is_onroad);
  // ALT_EXP_KEEP_AUTHORISATION (opendbc/safety/declarations.h) is set for this drive
  bool keepsAuthorisation() const { return safety_configured_ && (alternative_experience_ & 128U) != 0U; }

private:
  void updateMultiplexingMode();
  std::string fetchCarParams();
  void setSafetyMode(const std::string &params_string);

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  uint16_t alternative_experience_ = 0;
  Panda *panda_;
  Params params_;
};
