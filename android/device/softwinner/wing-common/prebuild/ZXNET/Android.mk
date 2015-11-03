LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)            
  LOCAL_POST_PROCESS_COMMAND := $(shell cp -rf $(LOCAL_PATH)/system/* $(ANDROID_PRODUCT_OUT)/system)

