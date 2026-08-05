set(MODULE_SOURCES
    app/app.c
    app_cmd/app_cmd.c
    app_gimbal/app_gimbal.c
    app_sensor/app_sensor.c
    app_shoot/app_shoot.c
    # app_referee_system/app_referee_system.c
)

set(MODULE_INCLUDE_DIRS
    .
    app_cmd
    app_gimbal
    app
    app_sensor
    app_shoot
    # app_referee_system
)
