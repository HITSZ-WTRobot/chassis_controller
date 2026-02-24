# Chassis Controller

**!IMPORTANT This repository is no longer maintained. Please visit https://github.com/HITSZ-WTRobot-Packages/ChassisController**

_A modular and extensible chassis control framework_

## Include

1. Add this repository as a submodule

2. Add it to your `CMakeLists.txt`

   ```cmake
   set(ChassisIF_ChassisType <your_chassis_type>)
   add_subdirectory(<submodule_path>/UserCode)
   ```

   `ChassisIF_ChassisType` can be:
    - `MECANUM4`: 4-wheel mecanum chassis
    - `OMNI4`: 4-wheel omni chassis

3. Link the module

  ```cmake
  target_link_libraries(${PROJECT_NAME}.elf PRIVATE chassis_controller)
  ```

## Prepare

1. choose your chassis type in `interfaces/chassis_if.h`

2. define a `Chassis_t` variable

3. define your wheel motors (such as `DJI_t`) and the motor velocity controllers (`Motor_VelCtrl_t`)

4. initialize your motors and controllers

5. initialize your `Chassis_t` using a `Chassis_Config_t` instance

## Configuration

please read the `Chassis_Config_t` definition in `chassis_if.h`

## Usage

- you can set chassis velocity with the functions below
  ```c
  void Chassis_SetVelWorldFrame(Chassis_t*                chassis,
                                const Chassis_Velocity_t* world_velocity,
                                const bool                target_in_world);
  
  void Chassis_SetVelBodyFrame(Chassis_t*                chassis,
                               const Chassis_Velocity_t* body_velocity,
                               const bool                target_in_world);
  ```
  `target_in_world` indicates the reference frame in which the velocity remains constant:
    - when `true`, the chassis maintains a constant velocity **with respect to the world frame**;
    - when `false`, the chassis maintains a constant velocity **with respect to the body frame**.

- you can set a posture target, and the chassis will follow an S-curve trajectory to reach it
  ```c
  SCurve_Result_t Chassis_SetTargetPostureInBody(Chassis_t*                     chassis,
                                                 const Chassis_PostureTarget_t* relative_target);
  SCurve_Result_t Chassis_SetTargetPostureInWorld(Chassis_t*                     chassis,
                                                  const Chassis_PostureTarget_t* absolute_target);
  ```
  here is the S-curve library: https://github.com/HITSZ-WTRobot/s-curve-planner
  use this function to check whether the trajectory has finished:
  ```c
  static bool Chassis_TrajectoryIsFinished(const Chassis_t* chassis);
  ```

- if you are not using an omnidirectional positioning system (OPS), you can reset the world frame:
  ```c
  void Chassis_SetWorldFromCurrent(Chassis_t* chassis);
  ```

## License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

You may use, modify, and distribute this project under the terms of the GPL-3.0.
Any derivative work must also be released under the same license.
For full details, please refer to the LICENSE file.
