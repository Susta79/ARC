##
##  Qt.cmake
##  qt-cmake-vscode
##
##  Created by Gino Bollaert on 05/10/2022.
##

if (APPLE)
    set(QT6_DIR ~/Qt/6.8.1/macos)
elseif (WIN32)
    set(QT6_DIR c:/Qt/6.8.1/mingw_64)
endif()