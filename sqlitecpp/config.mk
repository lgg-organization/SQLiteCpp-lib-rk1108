IS_ANDROID_OS = false
IS_NEED_SHARED_PTR = false
IS_NEED_COMPILE_STLPORT = false
IS_NEED_LINK_STLPORT = false
IS_NEED_COMPILE_TINYXML2 = true
IS_NEED_COMPILE_EXPAT = true
IS_SUPPORT_ION = true
IS_BUILD_TEST_APP = true
IS_CAM_IA10_API = false
IS_RK_ISP10 = false
IS_USE_RK_V4L2_HEAD = true
TARGET_ARCH_ABI = armeabi-v7a

BASE_DIR ?= $(CURDIR)/out
BUILD_OUPUT_EXTERNAL_LIBS:=$(BASE_DIR)/lib
CROSS_COMPILE ?= arm-linux-
TARGET_DIR ?= $(BASE_DIR)/system

#CROSS_COMPILE ?= /extra/zyc/android6.0/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
#CROSS_COMPILE ?=
