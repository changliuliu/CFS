/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "KTRException.h"
#include "zlm.h"
#include "knitro.h"

namespace knitro {

inline ZLM::ZLM()
    : _zlm(ZLM_checkout_license()) {
  checkLicenseValidity();
}

inline ZLM_context * ZLM::getLicenseManager() const {
  return _zlm;
}

inline void ZLM::checkLicenseValidity() const {
  if (_zlm == NULL) {
    std::string message = "ZLM license checkout failed. No high volume license available";
    throw KTRException(message, __FILE__, __LINE__);
  }
}

inline ZLM::~ZLM() {
  ZLM_release_license(_zlm);
}
}

