LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
    mongoose.c \
    home_automation.c

LOCAL_SHARED_LIBRARIES := \
    liblog \

LOCAL_STATIC_LIBRARIES :=

LOCAL_C_INCLUDES :=

LOCAL_CFLAGS := -DMONGOOSE_NO_CGI \
                -DMONGOOSE_NO_DIRECTORY_LISTING \
                -DMONGOOSE_NO_SSI \
                -DMONGOOSE_NO_DAV \
                -DMONGOOSE_NO_WEBSOCKET \
                -DMONGOOSE_NO_THREADS \
                -DNS_DISABLE_THREADS

LOCAL_LDLIBS := -lpthread

LOCAL_MODULE:= home_automation

include $(BUILD_EXECUTABLE)
