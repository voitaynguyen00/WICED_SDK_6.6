#-------------------------------------------------
#
# Project created by QtCreator 2016-11-07T14:38:11
#
#-------------------------------------------------

QT += core gui
QT += serialport
QT += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ClientControl
TEMPLATE = app

win32|win64 {
    DESTDIR = $$PWD/../Windows
}

macx {
    DESTDIR = $$PWD/../OSX
}

SOURCES += main.cpp\
        mainwindow.cpp \
        audio_src.cpp \
        device_manager.cpp \
        handsfree.cpp \
        pbap_client.cpp \
        spp.cpp \
        audio_gateway.cpp \
        hid_device.cpp \
        hid_host.cpp \
        home_kit.cpp \
        avrc_tg.cpp \
        avrc_ct.cpp \
        iap2.cpp \
        fw_download.cpp \
        gatt.cpp \
        audio_snk.cpp \
        bt_serial_gatt.cpp \ 
        battery_client.cpp \
        findme_locator.cpp \
        demo.cpp \
        opp_server.cpp \
    gatt_db.cpp \
    ../../host/app_host/app_host.c \
    ../../host/app_host/app_host_ag.c \
    ../../host/wiced_hci/wiced_hci.c \
    ../../host/wiced_hci/wiced_hci_ag.c \
    ../../host/wiced_hci/wiced_hci_audio_src.c \
	../../host/app_host/app_host_spp.c \
    ../../host/app_host/app_host_hidh.c \
    ../../host/app_host/app_host_hidd.c \
    ../../host/app_host/app_host_hf.c \
    ../../host/wiced_hci/wiced_hci_spp.c \
    ../../host/wiced_hci/wiced_hci_hidh.c \
    ../../host/wiced_hci/wiced_hci_hidd.c \
    ../../host/wiced_hci/wiced_hci_hf.c \
    ../../host/wiced_hci/wiced_hci_gatt.c \
    ../../host/wiced_hci/wiced_hci_dm.c \
    ../../host/app_host/app_host_audio_src.c \
    anp.cpp \
    ../../host/app_host/app_host_anp.c \
    ../../host/wiced_hci/wiced_hci_anp.c \
    le_coc.cpp \
    led_demo.cpp \
    ../../host/app_host/app_host_le_coc.c \
    ../../host/wiced_hci/wiced_hci_le_coc.c \
    ../../host/wiced_hci/wiced_hci_gatt_db.c \
    otp_client.cpp \
    ../../host/app_host/app_host_otp_client.c \
    ../../host/wiced_hci/wiced_hci_otp_client.c \
    ../../host/wiced_hci/wiced_hci_avrc_ct.c \
    ../../host/app_host/app_host_avrc_ct.c \
    ../../host/app_host/app_host_dm.c \
    ../../host/app_host/app_host_gatt.c \
    nanopb/pb_common.c \
    nanopb/pb_decode.c \
    nanopb/pb_encode.c \
    nanopb/rpc.pb.c \
    nanopb/wiced_hci_gatt_db.pb.c \
    nanopb/wiced_hci_spp.pb.c \
    nanopb/wiced_hci_gatt_db_rpc.c \
    nanopb/wiced_hci_spp_rpc.c \
    nanopb/protobuf_rpc.c

unix {
    SOURCES += serial_port_linux.cpp
    SOURCES += btspy_ux.cpp
}

win32|win64 {
    SOURCES += btspy_win32.cpp
    SOURCES += serial_port_win32.cpp
}


HEADERS  += mainwindow.h \
            fw_download.h \
            avrc.h \
            serial_port.h \
            app_include.h \
    wiced_bt_defs.h \
    ../../include/btle_homekit2_lightbulb.h \
    ../../include/hci_control_api.h \
    ../../host/app_host/app_host.h \
    ../../host/wiced_hci/wiced_hci.h \
    ../../host/wiced_hci/wiced_types.h \
    ../../host/app_host/app_host_spp.h \
    ../../host/app_host/app_host_otp_client.h \
    ../../host/app_host/app_host_le_coc.h \
    ../../host/app_host/app_host_hidh.h \
    ../../host/app_host/app_host_hidd.h \
    ../../host/app_host/app_host_hf.h \
    ../../host/app_host/app_host_gatt.h \
    ../../host/app_host/app_host_dm.h \
    ../../host/app_host/app_host_avrc.h \
    ../../host/app_host/app_host_audio_src.h \
    ../../host/app_host/app_host_ans_anc.h \
    ../../host/app_host/app_host_ag.h \
    ../../host/app_host/app_host.h \
    ../../host/wiced_hci/wiced_types.h \
    ../../host/wiced_hci/wiced_hci_spp.h \
    ../../host/wiced_hci/wiced_hci_otp.h \
    ../../host/wiced_hci/wiced_hci_le_coc.h \
    ../../host/wiced_hci/wiced_hci_hidh.h \
    ../../host/wiced_hci/wiced_hci_hidd.h \
    ../../host/wiced_hci/wiced_hci_hf.h \
    ../../host/wiced_hci/wiced_hci_gatt_db.h \
    ../../host/wiced_hci/wiced_hci_gatt.h \
    ../../host/wiced_hci/wiced_hci_dm.h \
    ../../host/wiced_hci/wiced_hci_avrc_ct.h \
    ../../host/wiced_hci/wiced_hci_audio_src.h \
    ../../host/wiced_hci/wiced_hci_anp_ans.h \
    ../../host/wiced_hci/wiced_hci_ag.h \
    ../../host/wiced_hci/wiced_hci.h \ 
    nanopb/pb.h \
    nanopb/pb_common.h \
    nanopb/pb_decode.h \
    nanopb/pb_encode.h \
    nanopb/rpc.pb.h \
    nanopb/wiced_hci_gatt_db.pb.h \
    protobuf_rpc.h

FORMS    += mainwindow.ui

INCLUDEPATH += common/include
INCLUDEPATH += ../../include
INCLUDEPATH += ../../host/app_host
INCLUDEPATH += ../../host/wiced_hci
INCLUDEPATH += ./nanopb

# ws2_32.lib and winmm.lib path might need to be adjusted on user PC, for example
# C:\WINDDK\7600.16385.0\lib\win7\i386\ws2_32.lib
# -L"Windows" -lws2_32
win32: LIBS += -lQt5Network ..\Windows\ws2_32.lib ..\Windows\winmm.lib

unix:!macx {
LIBS += -lasound -lrt
DEFINES += PCM_ALSA
}

RESOURCES     = resources.qrc

RC_ICONS = CY_Logo.ico

ICON = CY_Logo.icns

DISTFILES += \
    ../README.txt
