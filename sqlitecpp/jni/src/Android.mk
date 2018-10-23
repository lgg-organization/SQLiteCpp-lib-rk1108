#include $(call all-subdir-makefiles)
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

MY_CPP_LIST := $(wildcard $(LOCAL_PATH)/*.cpp)
MY_CPP_LIST += $(wildcard $(LOCAL_PATH)/cppsqlite/*.cpp)
MY_CPP_LIST += $(wildcard $(LOCAL_PATH)/sqlite3/*.c)

MY_CPP_INCLUDES := $(LOCAL_PATH)
MY_CPP_INCLUDES += $(LOCAL_PATH)/cppsqlite/include
MY_CPP_INCLUDES += $(LOCAL_PATH)/sqlite3/include

common_shared_libraries := 
common_static_libraries := 

LOCAL_MODULE := libCppSQLite3
LOCAL_SRC_FILES:= $(MY_CPP_LIST:$(LOCAL_PATH)/%=%)
LOCAL_C_INCLUDES := $(MY_CPP_INCLUDES)
LOCAL_SHARED_LIBRARIES := $(common_shared_libraries)
LOCAL_STATIC_LIBRARIES := $(common_static_libraries)

LOCAL_CPPFLAGS := -std=c++11 -pthread
LOCAL_CPPFLAGS += -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D_GLIBCXX_USE_SCHED_YIELD -D_GLIBCXX_USE_NANOSLEEP -DPOSIX
LOCAL_CPPFLAGS += -Wall -Werror -Wno-literal-suffix -Wno-unused-variable

#LOCAL_LDLIBS += -ldl -lpthread
LOCAL_ALLOW_UNDEFINED_SYMBOLS := true

#include $(BUILD_EXECUTABLE)
#include $(BUILD_STATIC_LIBRARY)
include $(BUILD_SHARED_LIBRARY)

#$(call import-module,android/native_app_glue)
#$(call import-module,cxx-stl/gnu-libstdc++)
#$(call import-module,third_party/googletest)
