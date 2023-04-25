colcon build --packages-up-to ethercat_device_configurator\
  --symlink-install\
  --cmake-args -Wdev -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release

cp build/compile_commands.json .
