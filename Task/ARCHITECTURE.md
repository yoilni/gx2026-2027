# Task application layer

`Task` owns the robot application policy. Hardware drivers remain in their
existing directories and must not contain mission-state transitions.

## Current entry point

- `mission_task.c/.h`: root mission state, start/stop/reset requests, coherent
  sensor snapshot, latched safety faults, and the first implemented S1 action:
  lock the PE13 team selector plus the PE14/PE15 start-area selector, then turn
  45 degrees with JY901S feedback. Start areas 1/4 turn left and 2/3 turn right.
  The same lock also records the field-relative safe-zone heading for
  diagnostics and as the final safe-zone normal heading after event 26.
  It then crosses the speed
  bump at 110 rpm for three seconds while a JY901S yaw-hold P loop preserves the completed
  45-degree heading, stops briefly to let vibration settle, sends MaixCam
  command 03, and uses the signed x pixel error from the 11-byte coordinate
  frame in a bounded PID turn to align with the first green target. If no
  green error arrives within two seconds, a JY901S search turns 90 degrees
  clockwise and then 180 degrees counter-clockwise; a coordinate at any point
  immediately resumes vision PID. If a previously acquired target is lost for
  two seconds, the same search is re-armed instead of remaining stopped. After
  Before state 03 starts, the frame is lowered and allowed to settle for one
  second while the camera remains at its 80-degree wide view. Horizontal
  alignment is followed by signed-y forward control while x error continues
  steering; both wheel targets are limited to 100 rpm. Event 04 starts the S4
  arrangement sequence with the frame already down, so fresh image errors can
  immediately guide the object to image center. After event 14, the chassis
  drives forward at 60 rpm for one second while X error alone corrects its
  heading, then the MCU sends its command-14 acknowledgement.
  Event 02 then starts two arrangement passes. The first drives forward at
  1.5-second right-curving push at about a 60-rpm average with 71/50-rpm wheel
  targets, then reverses and sends command 12. Event 12 enables a matching
  1.5-second left-curving push with 50/71-rpm wheel targets followed by its
  reverse. Neither vision axis is applied during these fixed differential
  pushes. After event 02, MaixCam may send event 24 at any time to skip
  the remaining corner-arrangement stages. The chassis stops, raises the frame, and enters final XY
  tracking without waiting for event 12 or transmitting command 24 again. On the normal path, the
  frame is raised and command 24 starts final tracking to
  the frame midpoint. Two seconds without coordinates in mode 24 starts a
  recovery that reverses at 50 rpm for 700 ms, forces a left turn to absolute
  yaw 270 degrees, turns to absolute yaw 90 degrees, and resumes mode-24
  waiting. Fresh coordinates cancel any recovery phase immediately. Event 34
  first drives the chassis forward at 60 rpm for
  150 ms, then lowers the frame around the final object. The MCU sends command
  15 for the red safe zone or 25 for the blue safe zone and waits in S5.
  MaixCam event 05 moves the MG90 from its 80-degree wide view to the
  45-degree near view for an in-frame load check. Event 02 restores the wide
  view, uses the JY901S yaw PID to align with the mapped safe-zone heading,
  raises the frame for one second, reverses at 50 rpm for 700 ms while holding
  that heading, lowers the frame for one second, and then repeats the
  arrangement sequence. Event 06 also restores the wide
  view and enters S6 without another arrangement. Because the
  safe-zone normal heading was latched from the team and start area, the
  chassis uses JY901S to turn toward it over the shortest angular path, with
  turn output limited to 30 rpm. MaixCam event 16 immediately stops the turn
  and clears stale coordinates; if the heading is reached first, the chassis
  waits two seconds for recognition. If event 16 is still absent, it drives
  forward at 40 rpm for 500 ms, uses the JY901S yaw PID to turn 45 degrees
  left, and uses the same loop to turn 90 degrees right from that position.
  Recovery rounds three and later use a
  stronger 70-rpm, 2000-ms forward segment. Event 16 stops any recovery phase
  immediately; unsuccessful rounds can repeat while the 20-second overall
  search timeout remains active.
  After event 16, a chassis lying up to 90 degrees on the configured positive
  side of the safe-zone heading first turns to that heading plus 90 degrees,
  drives forward at 50 rpm for 700 ms, and uses safe-zone X error to face the
  target again. If the resulting yaw remains in the same side sector, this
  lateral reposition repeats; otherwise the normal XY approach begins. The
  reposition and XY approach share the 15-second approach timeout.
  After event 16, fresh signed X/Y errors for the safe-zone left-quarter point
  directly drive image steering and approach-speed PID, limited to 35 rpm of
  turn, 60 rpm forward, and 80 rpm per wheel. Two seconds without fresh
  coordinates starts the same forward/left/right recovery; fresh coordinates
  stop it immediately and resume PID. The 15-second approach timeout spans
  both normal tracking and recovery. MaixCam event 26 indicates that
  the safe zone occupies at least half the image. The MCU then discards image
  control, turns to the field-mapped safe-zone normal heading with JY901S,
  reverses at 80 rpm with yaw hold for 500 ms, and brakes for 200 ms. It then
  drives forward at 70 rpm for 1300 ms to push the object into the safe zone.
  Once the push completes, it stops, raises the collection frame, and allows
  one second for the servos to settle. Finally it reverses at 70 rpm with yaw
  hold for 2000 ms, turns 180 degrees, and stops. Commands 06, 16, and 26 are
  never transmitted by the MCU.
- `start_button_task.c/.h`: debounces the active-low PE12 start button and
  publishes one mission start request per press. A button held during reset is
  ignored until it is released and pressed again.
- `debug_uart_task.c/.h`: owns UART2 at 115200 baud and serializes diagnostic
  text through a fixed-size FreeRTOS queue. The former Bluetooth transport and
  control tasks remain on disk but are excluded from the build.
- `servo_test_task.c/.h`: retained temporary MG90 test code, no longer created
  because PE12 is now the match start button. The PA2/PA3 frame-servo endpoints
  remain recorded in `robot_config.h`.
- `actuator_task.c/.h`: owns production servo PWM writes. At power-up it starts
  PA5/TIM2 CH1 continuously at the calibrated MG90 80-degree wide-angle view;
  entering S3 starts PA2/CH3 and PA3/CH4 and holds the frame raised.
- `uart_callback_router.c`: the single HAL UART callback owner that dispatches
  received bytes and UART errors to independent device drivers.
- Bluetooth control is disabled. Multi-object S5 decisions remain
  intentionally unimplemented.

## Planned modules

- `chassis_control_task.c/.h`: outer yaw and image tracking loops. It publishes
  left/right speed targets; the existing M2006 task remains the wheel-speed
  inner loop and sole CAN current producer.
- `actuator_task.c/.h`: will later add non-blocking action sequences for the two
  guide bars and runtime camera-position changes.
- `input_task.c/.h`: future paired photoelectric gates with direction-qualified
  entry/exit events. PE12 already has a dedicated start-button task. PE13 is
  the red/blue selector (high is red, low is blue); PE14 and PE15 encode start
  areas 1..4 as 01, 10, 11, and 00 respectively. All three selectors are
  debounced and latched at start.
- `safety_task.c/.h`: emergency input, communication deadlines, actuator
  timeout, and match timeout. Faults are latched and cleared only by an
  explicit reset request.
- `robot_config.h`: motor IDs/directions, servo limits, image errors, timeout
  thresholds, and competition-tunable parameters.
- `robot_events.h`: fixed-size event and command payloads shared by Task
  modules.

## Ownership rules

1. Only the M2006 control task sends CAN current commands.
2. Only the actuator module writes servo compare registers.
3. Only the mission task changes competition state and transported-object
   records.
4. UART and CAN callbacks parse or publish data only; they never change the
   mission state directly.
5. Tasks wait with queues, event flags, notifications, or deadlines. Busy
   `while` loops and application delays are not permitted.
6. Sensor freshness and target visibility are separate concepts. In
   particular, the current MaixCam protocol cannot distinguish an online
   camera with no detection from an offline camera; a heartbeat/no-target
   frame must be added before camera-link health can be supervised.

## Integration boundary

CubeMX-generated startup remains in `Core`. `Core/Src/freertos.c` should only
create or call Task-layer entry points inside USER CODE sections. Application
logic must stay under `Task` so CubeMX regeneration cannot overwrite it.
