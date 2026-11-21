#!/usr/bin/env python3
"""Test actual IMU caches, Payload::update_estimator(), and EKF without Gazebo.

Requires the existing Linux build's generated DDS messages and XRCE libraries.
Native mutexes and no-op device backends replace FreeRTOS/device initialization.
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[4]
components = root / 'components'
install = root / 'build/linux/esp-idf/DDS/xrce_deps/install'
generated = root / 'build/linux/esp-idf/DDS/generated'
with tempfile.TemporaryDirectory(prefix='imu-host-tests-') as temporary:
    work = pathlib.Path(temporary)
    (work / 'sdkconfig.h').write_text('#define CONFIG_IDF_TARGET_LINUX 1\n')
    (work / 'esp_log.h').write_text('''#pragma once
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
''')
    (work / 'freertos').mkdir()
    for name in ('FreeRTOS.h', 'task.h'):
        (work / 'freertos' / name).write_text('#pragma once\n')
    (work / 'task_mutex.hpp').write_text('''#pragma once
#include <mutex>
namespace util {
using TaskMutex = std::mutex;
using TaskMutexGuard = std::lock_guard<std::mutex>;
}
''')
    include_paths = [work, generated, install / 'include',
                     root / 'managed_components/espressif__eigen/eigen']
    include_paths += [components / path for path in
                      ('payload', 'DDS', 'util', 'drivers/imu', 'drivers/encoder',
                       'drivers/motor', 'drivers/camera')]
    includes = [f'-I{path}' for path in include_paths]
    objects = []
    messages = ('builtin_interfaces/msg/Time.c', 'geometry_msgs/msg/Vector3.c',
                'geometry_msgs/msg/Quaternion.c', 'payload_interfaces/msg/MotorState.c',
                'sensor_msgs/msg/Imu.c', 'std_msgs/msg/Header.c')
    for index, message in enumerate(messages):
        obj = work / f'message-{index}.o'
        subprocess.run(['cc', *includes, '-c', str(generated / message), '-o', str(obj)], check=True)
        objects.append(str(obj))
    sources = ('drivers/imu/imu.cpp', 'drivers/encoder/encoder.cpp',
               'drivers/encoder/encoder_hw.cpp', 'drivers/camera/camera.cpp',
               'payload/payload.cpp', 'payload/ekf.cpp', 'DDS/dds_client.cpp',
               'util/pwmcontroller.cpp', 'util/pid.cpp', 'util/feedforward.cpp',
               'drivers/imu/test/test_imu_pipeline.cpp')
    executable = work / 'test_imu_pipeline'
    subprocess.run([
        'c++', '-std=c++20', '-pthread', *includes,
        *(str(components / source) for source in sources), *objects,
        str(install / 'lib/libmicroxrcedds_client.a'), str(install / 'lib/libmicrocdr.a'),
        '-o', str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True, timeout=10)
