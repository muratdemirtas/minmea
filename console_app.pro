QT += core
QT -= gui

TARGET = console_app
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    main.c \
    minmea.c

HEADERS += \
    minmea.h

