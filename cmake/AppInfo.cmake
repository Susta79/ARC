set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(APP_NAME "ARC")
set(APP_VERSION 0.1.0)
set(COMPANY FBC)
#set(IDENTIFIER com.fbc.qt-template)
set(COPYRIGHT "Copyright (c) 2022 Fabrizio Bergamin. All rights reserved.")
set(ICON_NAME arc_icon)
set(MACOSX_IS_AGENT FALSE)

set(SOURCE_FILES
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/joint.cpp
    src/joint.h
    src/robot.cpp
    src/robot.h
    src/link.cpp
    src/link.h
    src/pose.cpp
    src/pose.h
    src/conf.cpp
    src/conf.h
    src/rpose.cpp
    src/rpose.h
    src/kinematic.cpp
    src/kinematic.h
    src/tcpclient.cpp
    src/tcpclient.h
    src/robi.cpp
    src/robi.h
    include/ARC/error_def.h
)

set(QT_COMPONENTS Widgets Gui Core Network)
