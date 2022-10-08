add_library(myslam SHARED    #给类的头文件和源文件链接
    KeyPoint.cpp
    FAST_detect.cpp
    GetBrief.cpp
    BFMatch.cpp
    BundleAdjustment.cpp
    pixel2cam.cpp)

target_link_libraries(myslam
    ${THIRD_PARTY_LIBS})

