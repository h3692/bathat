ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=bathat

define PINFO
PINFO DESCRIPTION=Dual-camera side-by-side NV12 viewfinder with EV-biased exposure
endef

INSTALLDIR=usr/bin

# Application sources live under src/. PROJECT_ROOT is expanded lazily by the
# recold rules, so referencing it here before qtargets.mk is fine.
EXTRA_SRCVPATH += $(PROJECT_ROOT)/src
EXTRA_INCVPATH += $(PROJECT_ROOT)/src

CXXFLAGS += -std=gnu++17 -Wall -Wextra

include $(MKFILES_ROOT)/qmacros.mk
include $(MKFILES_ROOT)/qtargets.mk

LIBS += camapi screen
