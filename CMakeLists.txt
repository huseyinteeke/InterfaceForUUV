cmake_minimum_required(VERSION 3.10)
project(SARA_GCS VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt'nin MOC (Meta-Object Compiler), UIC (User Interface) ve RCC (Resource) sistemlerini aktifleştir
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 1. Qt5 Kütüphaneleri (GUI ve Seri Port için)
find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort REQUIRED)

# 2. OpenCV Kütüphanesi (EasyCap Video için)
find_package(OpenCV REQUIRED)

# 3. MAVLink Kütüphanesi Yolu (Header'ları projeye tanıtıyoruz)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2)
# Kendi yazdığımız header'ların yolu
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Kaynak (cpp) dosyalarımız
set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/MavlinkWorker.cpp
    src/VideoWorker.cpp
)

# Çalıştırılabilir dosyayı oluştur
add_executable(SARA_GCS ${SOURCES})

# Bulduğumuz tüm kütüphaneleri projemize bağla (Linkleme)
target_link_libraries(SARA_GCS
    Qt5::Core 
    Qt5::Gui 
    Qt5::Widgets 
    Qt5::SerialPort
    ${OpenCV_LIBS}
)