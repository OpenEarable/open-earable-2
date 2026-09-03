if(CONFIG_USB_DEVICE_STACK_NEXT AND CONFIG_USBD_MSC_CLASS)
    target_sources(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/SDMassStorage.cpp
    )
endif()
