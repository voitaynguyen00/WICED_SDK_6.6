#-------------------------------------------------
#
# Project created by QtCreator 2013-07-01T18:28:57
#
#-------------------------------------------------

QT       += core gui
QT += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

# build for WICED SDK, use UART to talk to embedded app
CONFIG += wiced

# build for BSA, use local socket to talk to local BSA server, no support for UART
#CONFIG += bsa

TARGET = mesh_client
TEMPLATE = app

QMAKE_CXXFLAGS += -fpermissive
QMAKE_CFLAGS += -fpermissive
#QMAKE_CXXFLAGS += -fpermissive -MM
#QMAKE_CFLAGS += -fpermissive -MM

SOURCES += main.cpp mainwindow.cpp serial_port_linux.cpp \    
    btspy_ux.cpp

INCLUDEPATH += .

wiced {
    message("wiced build")

    DEFINES += WICED_MESH_REPO
    DEFINES += PROVISION_SCAN_REPORT_INCLUDE_BDADDR

    SOURCES += ../../../../libraries/mesh_client_lib/wiced_mesh_client.c
    SOURCES += ../../../../libraries/mesh_client_lib/wiced_bt_mesh_db.c
    SOURCES += ../../../../libraries/mesh_client_lib/wiced_mesh_api.c
    SOURCES += ../../../../libraries/mesh_client_lib/meshdb.c

    INCLUDEPATH += $$_PRO_FILE_PWD_/.
    INCLUDEPATH += $$_PRO_FILE_PWD_/../../../../include
    INCLUDEPATH += $$_PRO_FILE_PWD_/../../../../libraries/mesh_client_lib
    INCLUDEPATH += $$_PRO_FILE_PWD_/include
}

bsa {
    message("BSA build")
    DEFINES += BSA
    DEFINES += __ANDROID__
    INCLUDEPATH += ./include
    SOURCES += app_manager.c
    SOURCES += app_mesh.c
#    SOURCES += wiced_mesh_client.c \
#            wiced_bt_mesh_db.c \
#           wiced_mesh_api.c \
#           meshdb.c

#    SOURCES += wiced_mesh_api.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/wiced_mesh_api.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/wiced_bt_mesh_db.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/meshdb.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/wiced_mesh_client.c

    BSA_PATH = ../../../brcm/bsa

    INCLUDEPATH += ../../../../include
    INCLUDEPATH += ../../../../../../3rdparty/embedded/bsa_examples/linux/app_common/include
    INCLUDEPATH += ../../../../../../3rdparty/embedded/bsa_examples/linux/libbsa/include
    INCLUDEPATH += ../libbsa/include
    INCLUDEPATH += ../../../brcm/bsa/include
    INCLUDEPATH += ../../../bsa_examples/linux/app_common/include
    INCLUDEPATH += ../../../bsa_examples/linux/libbsa/include
    INCLUDEPATH += $$BSA_PATH/server/mesh/common/libraries/mesh_client_lib
    INCLUDEPATH += $$BSA_PATH/server/mesh/common/include
    INCLUDEPATH += $$BSA_PATH/server/mesh/20719-B1_Bluetooth/include/20719/
    INCLUDEPATH += $$BSA_PATH/server/mesh/20719-B1_Bluetooth/include/20719/hal
    INCLUDEPATH += $$BSA_PATH/20719-B1_Bluetooth/include/20719/stack
    LIBS += -L../libbsa/build/x86_64 -lbsa
}

bsa_repo {
    message("BSA build in BSA repo")
    DEFINES += BSA
    DEFINES += BSA_REPO
    DEFINES += __ANDROID__

    SOURCES += app_manager.c
    SOURCES += app_mesh.c
#    SOURCES += wiced_mesh_client.c \
#            wiced_bt_mesh_db.c \
#            wiced_mesh_api.c \
#            meshdb.c

#    SOURCES += wiced_mesh_api.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/wiced_mesh_api.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/wiced_bt_mesh_db.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/meshdb.c
    SOURCES += ../../../brcm/bsa/server/mesh/common/libraries/mesh_client_lib/wiced_mesh_client.c

    BSA_PATH = ../../../brcm/bsa
    INCLUDEPATH += ../../../../../../3rdparty/embedded/bsa_examples/linux/app_common/include
    INCLUDEPATH += ../../../../../../3rdparty/embedded/bsa_examples/linux/libbsa/include
    INCLUDEPATH += ../libbsa/include
    INCLUDEPATH += ../../../brcm/bsa/include
    INCLUDEPATH += ../../../bsa_examples/linux/app_common/include
    INCLUDEPATH += ../../../bsa_examples/linux/libbsa/include
    INCLUDEPATH += $$BSA_PATH/server/mesh/common/libraries/mesh_client_lib
    INCLUDEPATH += $$BSA_PATH/server/mesh/common/include
    INCLUDEPATH += $$BSA_PATH/server/mesh/20719-B1_Bluetooth/include/20719/
    INCLUDEPATH += $$BSA_PATH/server/mesh/20719-B1_Bluetooth/include/20719/hal
    INCLUDEPATH += $$BSA_PATH/20719-B1_Bluetooth/include/20719/stack
    LIBS += -L../libbsa/build/x86_64 -lbsa
#    LIBS += -L../libmesh/linux -lmesh
#    LIBS += -L../libmesh_client/linux -lmesh_client
}



HEADERS  += include/bsa.h \
            include/data_types.h \
            include/bsa_extern.h \
            include/add_defines.h \
            mainwindow.h \
            include/wiced_mesh_client.h \
            include/meshdb.h \
            include/wiced_bt_mesh_db.h \
            include/mesh_main.h \
            include/remote_provision_server.h \
            include/wiced_bt_mesh_models.h \
            include/serial_port.h \
            include/win_data_types.h \
            include/app_mesh.h \
    wsdownloader.h \
    btinterface.h \
    linuxbtinterface.h

unix {
#    SOURCES += serial_port_linux.cpp
#    SOURCES += btspy_ux.cpp
}

#win32|win64 {
#    SOURCES += btspy_win32.cpp
#    SOURCES += serial_port_win32.cpp
#}

FORMS    += mainwindow.ui 

DEFINES += QT_APP
DEFINES += BLE_INCLUDED
DEFINES += WICEDX_LINUX



