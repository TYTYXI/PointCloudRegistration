set(sources
        FusionEKF.h
        FusionEKF.cpp
        kalman_filter.h
        kalman_filter.cpp
        PointCloudKF.h
        PointCloudKF.cpp
        tools.h
        tools.cpp
        measurement_package.h
)

extend_ons_target(Registration
        SOURCES ${sources}
)