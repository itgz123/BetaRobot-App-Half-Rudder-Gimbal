set(MODULE_SOURCES
    app/app.c
    app_cmd/app_cmd.c
    app_gimbal/app_gimbal.c
    app_shoot/app_shoot.c
    app_comm_proto_visual/app_proto_visual.c
)

set(MODULE_INCLUDE_DIRS
    .
    app_cmd
    app_gimbal
    app
    app_shoot
    app_comm_proto_visual
)
