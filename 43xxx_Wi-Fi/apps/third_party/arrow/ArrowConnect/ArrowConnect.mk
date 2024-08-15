NAME := QuickSilver_ArrowConnect

ANC_SDK_DIR = ../../../../libraries/third_party/arrow/libs/acn_sdk_c
ARROW_DRIVER_DIR = ../../../../libraries/third_party/arrow/drivers

WIFI_CONFIG_DCT_H := wifi_config_dct.h
APPLICATION_DCT := acn_config_dct.c

#$(NAME)_DEFINES    += DEBUG
#$(NAME)_DEFINES    += DEBUG_WOLFSSL
#$(NAME)_DEFINES    += HTTP_DEBUG
$(NAME)_DEFINES    += USER_BYTE_CONVERTER
#$(NAME)_DEFINES    += _POSIX_THREADS
$(NAME)_DEFINES    += ARCH_SSL
$(NAME)_DEFINES    += AUTO_IP_ENABLED
$(NAME)_DEFINES    += DBG_LINE_SIZE=256

GLOBAL_INCLUDES += \
                   acnsdkc \
                   $(ANC_SDK_DIR) \
                   $(ANC_SDK_DIR)/include \
                   $(ANC_SDK_DIR)/sys \
                   $(ANC_SDK_DIR)/src/wolfSSL \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfssl\
                   $(ARROW_DRIVER_DIR)

$(NAME)_SOURCES	:= arrow_connect.c \
                   acn_config.c \
                   acn_common.c \
                   acnsdkc/bsd/socket.c \
                   acnsdkc/time/time.c \
                   acnsdkc/debug.c \
                   acnsdkc/sys/mac.c \
                   acnsdkc/arrow/storage.c \
                   acnsdkc/json/telemetry.c \
                   $(ARROW_DRIVER_DIR)/Sensors/LIS2DH12/lis2dh12.c \
                   $(ARROW_DRIVER_DIR)/Sensors/HTS221/hts221.c \
                   $(ARROW_DRIVER_DIR)/LED/APA102/apa102.c \
                   $(ANC_SDK_DIR)/src/ntp/client.c \
                   $(ANC_SDK_DIR)/src/ntp/ntp.c \
                   $(ANC_SDK_DIR)/src/time/watchdog_weak.c \
                   $(ANC_SDK_DIR)/src/time/time.c \
                   $(ANC_SDK_DIR)/src/bsd/inet.c \
                   $(ANC_SDK_DIR)/src/arrow/mqtt.c \
                   $(ANC_SDK_DIR)/src/arrow/routine.c \
                   $(ANC_SDK_DIR)/src/arrow/device.c \
                   $(ANC_SDK_DIR)/src/arrow/gateway.c \
                   $(ANC_SDK_DIR)/src/arrow/sign.c \
                   $(ANC_SDK_DIR)/src/arrow/api/device/device.c \
                   $(ANC_SDK_DIR)/src/arrow/api/device/info.c \
                   $(ANC_SDK_DIR)/src/arrow/api/gateway/info.c \
                   $(ANC_SDK_DIR)/src/arrow/api/gateway/gateway.c \
                   $(ANC_SDK_DIR)/src/arrow/api/json/parse.c \
                   $(ANC_SDK_DIR)/src/arrow/utf8.c \
                   $(ANC_SDK_DIR)/src/http/client.c \
                   $(ANC_SDK_DIR)/src/http/request.c \
                   $(ANC_SDK_DIR)/src/http/response.c \
                   $(ANC_SDK_DIR)/src/http/routine.c \
                   $(ANC_SDK_DIR)/src/arrow/events.c \
                   $(ANC_SDK_DIR)/src/arrow/state.c \
                   $(ANC_SDK_DIR)/src/arrow/software_update.c \
                   $(ANC_SDK_DIR)/src/arrow/software_release.c \
                   $(ANC_SDK_DIR)/src/arrow/gateway_payload_sign.c \
                   $(ANC_SDK_DIR)/src/arrow/device_command.c \
                   $(ANC_SDK_DIR)/src/arrow/telemetry_api.c \
                   $(ANC_SDK_DIR)/src/json/json.c \
                   $(ANC_SDK_DIR)/src/debug.c \
                   $(ANC_SDK_DIR)/src/data/property.c \
                   $(ANC_SDK_DIR)/src/data/propmap.c \
                   $(ANC_SDK_DIR)/src/data/linkedlist.c \
                   $(ANC_SDK_DIR)/src/data/ringbuffer.c \
                   $(ANC_SDK_DIR)/src/data/find_by.c \
                   $(ANC_SDK_DIR)/src/ssl/crypt.c \
                   $(ANC_SDK_DIR)/src/ssl/md5sum.c \
                   $(ANC_SDK_DIR)/src/ssl/ssl.c \
                   $(ANC_SDK_DIR)/src/mqtt/client/src/network.c \
                   $(ANC_SDK_DIR)/src/mqtt/client/src/MQTTClient.c \
                   $(ANC_SDK_DIR)/src/mqtt/client/src/timer.c \
                   $(ANC_SDK_DIR)/src/mqtt/packet/src/MQTTPacket.c \
                   $(ANC_SDK_DIR)/src/mqtt/packet/src/MQTTDeserializePublish.c \
                   $(ANC_SDK_DIR)/src/mqtt/packet/src/MQTTSerializePublish.c \
                   $(ANC_SDK_DIR)/src/mqtt/packet/src/MQTTConnectClient.c \
                   $(ANC_SDK_DIR)/src/mqtt/packet/src/MQTTSubscribeClient.c \
                   $(ANC_SDK_DIR)/src/sys/reboot_weak.c \
                   $(ANC_SDK_DIR)/src/arrow/mqtt/acn.c   \
                   $(ANC_SDK_DIR)/src/arrow/mqtt/azure.c   \
                   $(ANC_SDK_DIR)/src/arrow/mqtt/ibm.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/crl.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/internal.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/io.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/keys.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/ocsp.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/sniffer.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/ssl.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/src/tls.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/aes.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/asm.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/asn.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/ecc.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/coding.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/compress.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/error.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/fe_low_mem.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/fe_operations.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/ge_low_mem.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/ge_operations.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/hash.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/hmac.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/integer.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/tfm.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/md5.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/memory.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/misc.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/random.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/sha.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/rsa.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/sha256.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/signature.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/wc_encrypt.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/wc_port.c \
                   $(ANC_SDK_DIR)/src/wolfSSL/wolfcrypt/src/logging.c \

$(NAME)_RESOURCES += images/cypresslogo.png \
                     images/cypresslogo_line.png \
                     images/favicon.ico \
                     images/scan_icon.png \
                     images/wps_icon.png \
                     images/64_0bars.png \
                     images/64_1bars.png \
                     images/64_2bars.png \
                     images/64_3bars.png \
                     images/64_4bars.png \
                     images/64_5bars.png \
                     images/tick.png \
                     images/cross.png \
                     images/lock.png \
                     images/progress.gif \
                     scripts/general_ajax_script.js \
                     scripts/wpad.dat \
                     apps/aws_iot/aws_config.html \
                     config/scan_page_outer.html \
                     styles/buttons.css \
                     styles/border_radius.htc

$(NAME)_COMPONENTS := utilities/command_console \
                      utilities/command_console/ping \
                      daemons/device_onboarding \
                      daemons/HTTP_server \
                      daemons/DNS_redirect \
                      protocols/DNS \
                      daemons/ota_server

OTA_APPLICATION       := snip.ota2_extract-$(PLATFORM)
OTA_APP               := build/$(OTA_APPLICATION)/binary/$(OTA_APPLICATION).stripped.elf
