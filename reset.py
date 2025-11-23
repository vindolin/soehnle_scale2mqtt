import os

Import("env")

env.Replace(
    RESETTOOL=os.path.join(
        env.PioPlatform().get_package_dir("tool-esptoolpy") or "", "esptool.py"
    ),
    RESETFLAGS=[
        "--no-stub",
        "--chip",
        env.BoardConfig().get("build.mcu", "esp32"),
        "--port",
        "$UPLOAD_PORT",
        "flash_id",
    ],
    RESETCMD='"$PYTHONEXE" "$RESETTOOL" $RESETFLAGS',
)

env.AddCustomTarget(
    name="reset_target",
    dependencies=None,
    actions=[
        env.VerboseAction(env.AutodetectUploadPort, "Looking for target port..."),
        env.VerboseAction("$RESETCMD", "Resetting target"),
    ],
    title="Reset ESP32 target",
    description="This command resets ESP32 target via esptoolpy",
)
