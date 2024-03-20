add_library(GL_SLAM SHARED frame.cpp
                           feature.cpp
                           mappoint.cpp
                           map.cpp
                           frontend.cpp
                           pinhole_camera.cpp)
target_link_libraries(GL_SLAM ${THIRD_PARTY_LIBS})
