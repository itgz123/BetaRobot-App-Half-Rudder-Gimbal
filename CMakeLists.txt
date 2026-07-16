set(MODULE_SOURCES
    app/app.c
    app_cmd/app_cmd.c
    app_chassis/app_chassis.c
    app_gimbal/app_gimbal.c
    app_sensor/app_sensor.c
)

set(MODULE_INCLUDE_DIRS
    .
    app_chassis
    app_cmd
    app_gimbal
    app
    app_sensor
)
