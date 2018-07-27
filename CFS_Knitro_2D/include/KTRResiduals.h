#pragma once

#include "KTRElements.h"

namespace knitro {
	
class KTRResiduals : public KTRElements {
	
public:
	explicit KTRResiduals(int n);
};

}

#include "impl/KTRResiduals.hxx"

