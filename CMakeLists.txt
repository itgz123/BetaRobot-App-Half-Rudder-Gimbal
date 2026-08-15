set(MODULE_SOURCES
    app/app.c
    app_cmd/app_cmd.c
    app_gimbal/app_gimbal.c
    app_sensor/app_sensor.c
    app_shoot/app_shoot.c
    app_shoot/app_proto_demo.c
    app_usb/app_usb.c
    # app_referee_system/app_referee_system.c
)

set(MODULE_INCLUDE_DIRS
    .
    app_cmd
    app_gimbal
    app
    app_sensor
    app_shoot
    app_usb
    # app_referee_system
)
