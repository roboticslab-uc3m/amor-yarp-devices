if(NOT DEFINED ENABLE_AmorCartesianControl OR ENABLE_AmorCartesianControl)
    if(NOT TARGET AMOR::amor_api)
        message(WARNING "amor_api target not found, disabling AmorCartesianControl")
    elseif(NOT TARGET ROBOTICSLAB::KinematicRepresentationLib)
        message(WARNING "KinematicRepresentationLib target not found, disabling AmorCartesianControl")
    elseif(NOT TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
        message(WARNING "KinematicsDynamicsInterfaces target not found, disabling AmorCartesianControl")
    endif()
endif()

yarp_prepare_plugin(AmorCartesianControl
                    CATEGORY device
                    TYPE roboticslab::AmorCartesianControl
                    INCLUDE AmorCartesianControl.hpp
                    DEFAULT ON
                    DEPENDS "TARGET ROBOTICSLAB::KinematicRepresentationLib;TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces;TARGET AMOR::amor_api"
                    EXTRA_CONFIG WRAPPER=CartesianControlServer)

if(NOT SKIP_AmorCartesianControl)

    yarp_add_plugin(AmorCartesianControl AmorCartesianControl.hpp
                                         AmorCartesianControl.cpp
                                         DeviceDriverImpl.cpp
                                         ICartesianControlImpl.cpp
                                         LogComponent.hpp
                                         LogComponent.cpp)

    target_link_libraries(AmorCartesianControl YARP::YARP_os
                                               YARP::YARP_dev
                                               AMOR::amor_api
                                               ROBOTICSLAB::KinematicRepresentationLib
                                               ROBOTICSLAB::KinematicsDynamicsInterfaces)

    yarp_install(TARGETS AmorCartesianControl
                 LIBRARY DESTINATION ${AMOR-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${AMOR-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${AMOR-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AmorCartesianControl OFF CACHE BOOL "Enable/disable AmorCartesianControl device" FORCE)

endif()
