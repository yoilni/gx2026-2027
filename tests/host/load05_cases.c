/* Current final05 protocol. Compiled with the real parser and the extracted
   mission functions by verify_supplement_capture.ps1; no target hardware. */
static void tick(uint32_t now) {
  fake_now=now; MissionTask_UpdateVisionInput(now);
  MissionTask_CheckRunningHealth(now); (void)MissionTask_HandleLoadCheckRequest(now);
  MissionTask_HandleSupplementControl(now); MissionTask_HandleRedPriorityRequest(now);
  MissionTask_DropDuplicateRecoveryEvent(now);
  MissionTask_RunS3LowerFrame(now);
  run_normal_s4(now); now=fake_now;
  MissionTask_RunS6LoadRecheck(now);
  MissionTask_RunS5WaitSingleGreen(now); MissionTask_RunS5PrepareArrange(now);
  MissionTask_RunSupplement(now);
}
static void setup(uint8_t cmd,bool after07) {
  assert(MaixCam_Init(&uart)==HAL_OK); ++uart_inits; MissionTask_ResetSupplement();
  memset(&mission_snapshot,0,sizeof(mission_snapshot));
  mission_snapshot.state=MISSION_STATE_WAIT_START;
  mission_snapshot.team=ROBOT_TEAM_BLUE; mission_snapshot.yaw_cdeg=30000;
  mission_snapshot.safe_zone_yaw_target_cdeg=18000;
  s7_target_switch_enabled=after07; s3_target_select_command=cmd;
  s51_active_object_id=6; s6_carried_object_id=6; s5_close_view_active=false;
  s6_corner_completed=false; s6_track_elapsed_ms=0;
  s7_red_switch_sync_pending=false; s7_red_coordinate_fence_active=false;
  s5_recheck_resume_safe_tracking=s5_recheck_resume_supplement=false;
  s5_load_class_confirmed=true; s4_final_event34_pending=false; s4_e4_protocol_active=false;
  stops=wide_calls=track_view_calls=safe_view_calls=near_calls=frame_raise_calls=frame_partial_calls=frame_lower_calls=tx_count=0;
  final_view_calls=0;final_view_ok=true;
  s5_casualty_adjust_active=s5_casualty_adjust_done=false;inject06_on45=false;
  claw_close_calls=claw_open_calls=nonzero_wheel_requests=baffle_stop_calls=0;
  s6_found_baffle_pwm_active=false;
  claw_angle=ROBOT_CLAW_CLOSED_DEG;
  claw_close_inhibited=0;
  frame_lift=0; wide_ok=near_ok=frame_lower_ok=claw_close_ok=claw_open_ok=tx_ok=true; health_faults=0;
  inject06_on_tx=0; expire_on_tx=false; turn_soft_logs=search_soft_logs=red_switch_success_logs=0;
  first34_ack_delay_ms=0;
  inject_retry03_id=0;
  inject_e3_on13_tx=false;
  inject_push_reply_on_tx=0;s4_fixed_motion_ack=MAIXCAM_COMMAND_SIDE_PUSH_DONE_ACK;
  inject05_result_on_tx=inject05_result_on_stop=0;
  fake_now=100; MissionTask_EnterState(MISSION_STATE_S5_WAIT_SINGLE_GREEN,100);
  MaixCam_SetSupplementWindowEnabled(after07 && cmd==0x51);
}
static void recovery_setup(uint8_t previous_count,bool from_tracking) {
  setup(0x51,true);
  MaixCam_SetLoadRecheckEnabled(false);MaixCam_SetSupplementWindowEnabled(false);
  s5_close_view_active=false;mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
  MissionTask_EnterState(MISSION_STATE_S6_TRACK_SAFE_ZONE,100);
  s6_recovery_count=previous_count;
  MissionTask_StartS6Recovery(120,from_tracking);
}
static void recovery_tick(uint32_t now,bool new_coord) {
  fake_now=now;if(new_coord) coords(2,30,50);
  MissionTask_UpdateVisionInput(now);MissionTask_CheckRunningHealth(now);
  MissionTask_RunS6Recovery(now);
}
static void verify_first_recovery_forward(void) {
  const int32_t errors[]={0,300,1001,-1500,9000,-9000,18000,-18000};
  const uint32_t offsets[]={0U,150U,299U};
  assert(ROBOT_S6_RECOVERY_FIRST_FORWARD_RPM==170 && ROBOT_S6_RECOVERY_FIRST_FORWARD_MS==300);
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned e=0;e<sizeof(errors)/sizeof(errors[0]);++e) {
    recovery_setup(0,true);
    uint32_t start=wrap?UINT32_MAX-100U:120U;
    mission_snapshot.state_entry_tick=start;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(s6_recovery_base_yaw_cdeg+errors[e]);
    s6_yaw_pid_initialized=true;s6_yaw_pid_update_tick=start-10U;
    s6_yaw_pid_integral=123.0f;s6_yaw_pid_last_error=456.0f;
    for(unsigned t=0;t<sizeof(offsets)/sizeof(offsets[0]);++t) {
      recovery_tick(start+offsets[t],false);
      assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_recovery_first_forward_active);
      assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
      assert(mission_snapshot.vision_forward_rpm==170 && !mission_snapshot.vision_turn_rpm);
      assert(!mission_snapshot.safe_zone_fixed_heading_active && s6_recovery_count==1);
      assert(s6_yaw_pid_initialized && s6_yaw_pid_update_tick==start-10U);
      assert(s6_yaw_pid_integral==123.0f && s6_yaw_pid_last_error==456.0f);
    }
    recovery_tick(start+300U,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && !s6_recovery_first_forward_active);
    assert(mission_snapshot.state_entry_tick==start+300U && !s6_yaw_pid_initialized);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && s6_recovery_count==1);
  }
  // Turning starts AFTER300ms and receives its full independent2s deadline.
  recovery_setup(0,true);mission_snapshot.yaw_cdeg=9000;
  recovery_tick(419,false);assert(mission_snapshot.left_target_rpm==170);
  recovery_tick(420,false);recovery_tick(440,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_yaw_pid_initialized);
  assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm);
  recovery_tick(2120,false);recovery_tick(2419,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
  recovery_tick(2420,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  assert(!mission_snapshot.fault_flags && s6_recovery_count==1);
  // Original safe-relative left45/right90/return-safe path remains unchanged.
  recovery_setup(0,true);recovery_tick(420,false);recovery_tick(440,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  assert(s6_recovery_base_yaw_cdeg==mission_snapshot.safe_zone_yaw_target_cdeg);
  assert(s6_recovery_left_yaw_cdeg==MissionTask_WrapYaw(s6_recovery_base_yaw_cdeg+ROBOT_YAW_LEFT_SIGN*4500L));
  assert(s6_recovery_right_yaw_cdeg==MissionTask_WrapYaw(s6_recovery_base_yaw_cdeg-ROBOT_YAW_LEFT_SIGN*4500L));
  mission_snapshot.yaw_cdeg=s6_recovery_left_yaw_cdeg;recovery_tick(460,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_BACK);
  mission_snapshot.yaw_cdeg=s6_recovery_right_yaw_cdeg;recovery_tick(480,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_RETURN_SAFE);
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;recovery_tick(500,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_recovery_count==1);
  assert(!s6_recovery_first_forward_active && !tx_count);
  // First recovery still accepts16/new coordinates: only later long recovery
  // has the previously requested mandatory-forward lock.
  recovery_setup(0,true);recovery_tick(140,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !s6_recovery_first_forward_active);
  recovery_setup(0,false);fake_now=140;rx(0x16);recovery_tick(140,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !s6_recovery_first_forward_active);
  // Priority05/26/stop/health cancel short forward without a later stale move.
  recovery_setup(0,true);fake_now=140;rx(0x05);tick(140);recovery_tick(440,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !s6_recovery_first_forward_active);
  assert(!mission_snapshot.left_target_rpm && !tx_count);
  recovery_setup(0,true);fake_now=140;rx(0x26);MissionTask_HandleS6AlignRequest(140);recovery_tick(160,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && !s6_recovery_first_forward_active);
  recovery_setup(0,true);health_faults=MISSION_FAULT_IMU;recovery_tick(140,false);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !s6_recovery_first_forward_active && !mission_snapshot.left_target_rpm);
  recovery_setup(0,true);MissionTask_EnterState(MISSION_STATE_STOPPED,140);recovery_tick(440,false);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !s6_recovery_first_forward_active && !mission_snapshot.right_target_rpm);
  puts("PASS:first6E equal170/300 before SAFE/L45/R90/SAFE,no yaw PID or steering gate,299/300 and tick wrap,fresh2s turn budget;first XY/16 interruption retained,05/26/health/stop cancel;later120/1500 unchanged");
}
static void verify_prepare_reverse_without_yaw(void) {
  const int32_t errors[]={0,9000,-9000,18000};
  assert(ROBOT_S5_ARRANGE_REVERSE_RPM==100 && ROBOT_S5_ARRANGE_REVERSE_MS==500U);
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned e=0;e<4;++e) {
    setup(0x51,true);
    uint32_t start=wrap?UINT32_MAX-100U:200U;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg+errors[e]);
    fake_now=start;rx(0x02);tick(start);
    assert(mission_snapshot.state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE);
    assert(claw_open_calls==1 && claw_angle==74 && !frame_raise_calls && !frame_lower_calls);
    assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
    fake_now=start+400U;rx(0x02);coords(6,300,300);tick(fake_now);
    tick(start+499U);
    assert(mission_snapshot.state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE);
    assert(mission_snapshot.state_entry_tick==start && claw_open_calls==1 && !tx_count);
    assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
    assert(mission_snapshot.vision_forward_rpm==-100 && !mission_snapshot.vision_turn_rpm);
    assert(!mission_snapshot.safe_zone_fixed_heading_active && !s6_yaw_pid_initialized);
    assert(!mission_snapshot.vision_target_valid);
    tick(start+500U);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!frame_lower_calls && claw_angle==36 && claw_close_calls==1 && track_view_calls==1 && !wide_calls);
    assert(tx_count==1 && tx_commands[0]==0x03 && s3_target_select_command==0x51);
    MaixCam_Object object;
    assert(!MaixCam_GetObjectSnapshot(&object));
  }
  for(unsigned failure=0;failure<2;++failure) {
    setup(0x51,true);claw_open_ok=failure!=0;
    fake_now=120;rx(0x02);tick(120);
    if(failure) {claw_close_ok=false;tick(620);}
    assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !tx_count);
  }
  setup(0x51,true);fake_now=120;rx(0x02);tick(120);
  fake_now=200;rx(0x05);tick(200);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT &&
         claw_angle==36 && !mission_snapshot.left_target_rpm && !tx_count);

  setup(0x51,true);supplement_capture_active=supplement_budget_started=true;
  supplement_first_request_tick=101; supplement_capture_count=1;
  fake_now=120;rx(0x02);tick(120);
  assert(supplement_capture_active && mission_snapshot.state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE);
  tick(620);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && tx_count==1 && tx_commands[0]==0x03);
  assert(supplement_capture_active && supplement_capture_count==1 && supplement_first_request_tick==101);
  // Fresh image-center frames and events arriving within TX03 survive.
  setup(0x11,true);fake_now=120;rx(0x02);tick(120);
  inject_retry03_id=4;tick(620);
  MaixCam_Object object;uint8_t event;
  assert(MaixCam_GetObjectSnapshot(&object) && object.object_id==4 && object.x_error_px==15);
  assert(MaixCam_PeekEvent(&event) && event==0x04);
  puts("PASS:05->02 claw74,reverse100/500 no yaw or baffle change,duplicate/old XY fenced,close36 then TX03,wrap/safety/supplement budget");
}
static void verify_recovery_forward_without_yaw(void) {
  verify_first_recovery_forward();
  const int32_t errors[]={0,300,1000,1001,-1500,9000,-9000,18000,-18000};
  const uint32_t offsets[]={0U,700U,1499U};
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned e=0;e<sizeof(errors)/sizeof(errors[0]);++e) {
    recovery_setup(1,true);
    uint32_t start=wrap?UINT32_MAX-1000U:120U;
    mission_snapshot.state_entry_tick=start;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(s6_recovery_base_yaw_cdeg+errors[e]);
    mission_snapshot.safe_zone_fixed_heading_active=true;
    s6_yaw_pid_initialized=true;s6_yaw_pid_update_tick=start-10U;
    s6_yaw_pid_integral=123.0f;s6_yaw_pid_last_error=456.0f;
    for(unsigned t=0;t<sizeof(offsets)/sizeof(offsets[0]);++t) {
      recovery_tick(start+offsets[t],true);
      assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_recovery_count==2);
      assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
      assert(mission_snapshot.vision_forward_rpm==120 && !mission_snapshot.vision_turn_rpm);
      assert(!mission_snapshot.safe_zone_fixed_heading_active);
      assert(s6_yaw_pid_initialized && s6_yaw_pid_update_tick==start-10U);
      assert(s6_yaw_pid_integral==123.0f && s6_yaw_pid_last_error==456.0f);
    }
    recovery_tick(start+1500U,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_recovery_count==2);
  }
  puts("PASS:second+6E equal-wheel forward120/1500,no yaw PID/gate even180deg error;fresh coordinates deferred,tick wrap and recovery count retained");
}
static void verify_recovery_forward_lock(void) {
  const uint32_t end=120U+ROBOT_S6_RECOVERY_LONG_FORWARD_MS;
  assert(ROBOT_S6_RECOVERY_LONG_FORWARD_RPM==120 && ROBOT_S6_RECOVERY_LONG_FORWARD_MS==1500);
  assert(ROBOT_S6_RECOVERY_SCAN_MAX_RPM==20.0f && ROBOT_S6_RECOVERY_SCAN_MIN_RPM==20.0f);
  //Second and subsequent long-forward recoveries cannot finish from coordinates early.
  for(uint8_t count=1;count<=3;++count) {
    recovery_setup(count,true);recovery_tick(140,true);recovery_tick(1000,true);recovery_tick(end-1U,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_recovery_count==count+1);
    assert(mission_snapshot.state_entry_tick==120 && mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    recovery_tick(end,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
    assert(s6_recovery_count==count+1);
    assert(mission_snapshot.vision_target_valid && mission_snapshot.vision_x_error_px==30 && mission_snapshot.vision_y_error_px==-50);
  }
  //One-shot16 is deferred, not lost. Repeated16 does not discard newer coordinates.
  recovery_setup(1,false);fake_now=200;rx(0x16);recovery_tick(200,false);
  assert(s6_recovery_found16_pending && mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
  assert(!s6_found_baffle_pwm_active);
  recovery_tick(300,true);fake_now=400;rx(0x16);recovery_tick(400,false);
  assert(mission_snapshot.vision_target_valid && s6_recovery_found16_pending && mission_snapshot.state_entry_tick==120);
  recovery_tick(end-1U,true);recovery_tick(end,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !s6_recovery_found16_pending);
  assert(s6_found_baffle_pwm_active && s6_found_baffle_pwm_tick==end);
  assert(s6_recovery_count==2);
  assert(mission_snapshot.vision_target_valid && mission_snapshot.vision_x_error_px==30 && mission_snapshot.vision_y_error_px==-50);
  //Stale coordinates after saved16 never feed PID; resume stopped, awaiting new frames.
  recovery_setup(1,false);fake_now=200;rx(0x16);recovery_tick(200,false);
  recovery_tick(300,true);recovery_tick(end,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !mission_snapshot.vision_target_valid);
  assert(s6_recovery_count==2);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  //Without16, initial search coordinates do not authorize tracking; the scan resumes.
  recovery_setup(1,false);recovery_tick(end-1U,true);recovery_tick(end,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  fake_now=end+20U;rx(0x16);recovery_tick(end+20U,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  //No live coordinates: finish the forward leg, then allow scan interruption.
  recovery_setup(1,true);recovery_tick(end,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  recovery_tick(end+20U,true);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  //First recovery retains immediate coordinate interruption during the new
  // short forward; without a target, drive300ms before safe-facing/sweeping.
  recovery_setup(0,true);recovery_tick(140,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  recovery_setup(0,true);recovery_tick(140,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_recovery_first_forward_active);
  assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
  recovery_tick(419,false);assert(s6_recovery_first_forward_active);
  recovery_tick(420,false);assert(!s6_recovery_first_forward_active && mission_snapshot.state_entry_tick==420);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  recovery_tick(440,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  //Clock wrapping must not unlock the forward leg early.
  recovery_setup(1,true);uint32_t start=UINT32_MAX-1000U;
  mission_snapshot.state_entry_tick=start;
  recovery_tick(start+ROBOT_S6_RECOVERY_LONG_FORWARD_MS-1U,true);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
  recovery_tick(start+ROBOT_S6_RECOVERY_LONG_FORWARD_MS,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  //Explicit26, universal05, health faults and stop still preempt the forward leg.
  recovery_setup(1,true);fake_now=200;rx(0x26);MissionTask_HandleS6AlignRequest(200);recovery_tick(200,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
  recovery_setup(1,true);fake_now=200;rx(0x05);tick(200);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm);
  recovery_setup(1,true);health_faults=MISSION_FAULT_IMU;recovery_tick(200,true);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  recovery_setup(1,true);MissionTask_EnterState(MISSION_STATE_STOPPED,200);recovery_tick(220,true);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !mission_snapshot.right_target_rpm);
}
static void verify_recovery_count_persists(void) {
  // Finding coordinates OR16 after the first recovery must not buy another
  // first sweep. The next loss starts forward immediately at120rpm/1500ms.
  for(unsigned found16=0;found16<2;++found16) {
    recovery_setup(0,found16==0);
    fake_now=140;if(found16) rx(0x16);
    recovery_tick(140,found16==0);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_recovery_count==1);
    MissionTask_StartS6Recovery(160,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_recovery_count==2);
    recovery_tick(180,true);
    assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    recovery_tick(1659,true);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
    recovery_tick(1660,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_recovery_count==2);
    MissionTask_StartS6Recovery(1680,true);recovery_tick(1700,false);
    assert(s6_recovery_count==3 && mission_snapshot.left_target_rpm==120);
  }
  recovery_setup(UINT8_MAX,true);assert(s6_recovery_count==UINT8_MAX);
  // A genuinely new capture/load06 starts a new delivery allowance.
  setup(0x51,true);s6_recovery_count=5;
  MissionTask_StartS6FromLoad(120);assert(s6_recovery_count==0);
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
  MissionTask_StartS6Recovery(140,false);recovery_tick(160,false);
  assert(s6_recovery_count==1 && mission_snapshot.left_target_rpm==170 && s6_recovery_first_forward_active);
  puts("PASS:6E count persists after coordinates/16/saved16 reacquisition,second+loss120/1500 without coordinate interruption,saturates255;new delivery resets");
}
static void request(void) {
  fake_now+=20; rx(0x05); tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  fake_now+=500; rx(0x03); tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WIDE_SETTLE);
  assert(frame_lift==0 && s6_carried_object_id==6);
}
static void start_tracking(void) {
  uint32_t start=mission_snapshot.state_entry_tick; unsigned before=tx_count;
  tick(start+499); assert(tx_count==before);
  tick(start+500); assert(tx_count==before+2);
  assert(tx_commands[before]==0x51 && tx_commands[before+1]==0x03);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK);
  assert(frame_lift==0);
}
static void final_report(uint8_t id,uint32_t now) {
  fake_now=now; coords(id,0,0); tick(now);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm);
  fake_now=now+50; rx(0x06); tick(fake_now);
}
static void verify_speed_case(MissionState state,uint8_t reference,int16_t y,int expected) {
  setup(0x51,true);
  mission_snapshot.state=state;
  supplement_reference_command=reference;
  MissionTask_ResetVisionPid();
  mission_snapshot.vision_target_valid=true;
  mission_snapshot.target_age_ms=0;
  mission_snapshot.target_sequence=1;
  mission_snapshot.vision_x_error_px=0;
  mission_snapshot.vision_y_error_px=y;
  if(state==MISSION_STATE_S3_TRACK_GREEN || state==MISSION_STATE_S6_TRACK_SAFE_ZONE) {
    assert(MissionTask_CalculateVisionForward(fake_now,y)==expected);
  } else {
    bool reached;
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(mission_snapshot.left_target_rpm==expected && mission_snapshot.right_target_rpm==expected);
    // Expired coordinates never retain the newly increased minimum speed.
    mission_snapshot.target_age_ms=MAIXCAM_DATA_TIMEOUT_MS;
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
}
static uint32_t budget_at_phase(MissionState phase) {
  setup(0x51,true); request(); start_tracking();
  uint32_t first=supplement_first_request_tick;
  fake_now=first+ROBOT_SUPPLEMENT_TOTAL_MS-20U;
  MissionTask_EnterState(phase,fake_now);
  mission_snapshot.left_target_rpm=80; mission_snapshot.right_target_rpm=60;
  if(phase==MISSION_STATE_S4_TRACK_FINAL_BLOCK) frame_lift=25; //PA2 fixture, not a24 command.
  return first;
}
static void assert_review_route(uint8_t result) {
  if(result==0x06) {
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
    assert(s6_carried_object_id==5 && s5_load_class_confirmed && frame_lift==0);
    assert(!supplement_budget_started && !supplement_capture_active);
  } else if(result==0x02) {
    assert(mission_snapshot.state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE && claw_angle==74);
  } else {
    assert(result==0x24 && mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME);
    assert(!final_view_calls);
    assert(s6_carried_object_id==0);
    assert(!supplement_budget_started && !supplement_capture_active);
  }
}
static void verify_budget_request_handshake(void) {
  assert(ROBOT_SUPPLEMENT_TOTAL_MS==12000U);
  //Budget expiry can preempt03,04,24 and their recovery/capture motion.
  const MissionState phases[]={MISSION_STATE_SUPPLEMENT_WIDE_SETTLE,
      MISSION_STATE_SUPPLEMENT_TRACK,MISSION_STATE_SUPPLEMENT_TURN_LEFT,
      MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW,
      MISSION_STATE_SUPPLEMENT_TURN_RIGHT,MISSION_STATE_S4_TRACK_CENTER,
      MISSION_STATE_S4_WAIT_ARRANGE_READY,MISSION_STATE_S4_TRACK_FINAL_BLOCK,
      MISSION_STATE_S4_FINAL_CENTER_OBJECT,MISSION_STATE_S4_FINAL_LOWER_FRAME,
      MISSION_STATE_S4_FINAL_TURN_SAFE};
  for(unsigned i=0;i<sizeof(phases)/sizeof(phases[0]);++i) {
    uint32_t first=budget_at_phase(phases[i]); unsigned before=tx_count;
    uint8_t count=supplement_capture_count;
    //An old partial coordinate must not survive the review transition.
    byte(0xF1);byte(0xF2);byte(6);byte(0);
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    assert(tx_count==before+1 && tx_commands[before]==0x05 && frame_lift==0);
    assert(s6_carried_object_id==6 && supplement_capture_count==count);
    assert(supplement_first_request_tick==first && supplement_budget_exhausted);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!supplement_capture_active && !vision_search_forward_active);
    byte(0);byte(1);byte(0);byte(0);byte(1);byte(0x1F);byte(0x2F);
    coords(5,30,40);rx(0x34);rx(0xE3);rx(0x14);
    uint32_t review=first+ROBOT_SUPPLEMENT_TOTAL_MS+1000U;
    tick(review); assert(tx_count==before+1 && !mission_snapshot.vision_target_valid);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    //No matching05: a zero-ID frame and06 cannot bypass final review.
    coords(5,0,0);rx(0x06);tick(review+20U);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    assert(tx_count==before+1);
    fake_now=review+40U;rx(0x05);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    tick(review+40U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    final_report(5,review+40U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS); assert_review_route(0x06);
    assert(tx_count==before+1);
  }
  //A reply received while TX05 is completing may contain05/ID/result/16
  //in one burst. No clearing or duplicate request after sending.
  const uint8_t results[]={0x06,0x02,0x24};
  for(unsigned i=0;i<sizeof(results);++i) {
    uint32_t first=budget_at_phase(MISSION_STATE_S4_TRACK_FINAL_BLOCK);
    unsigned before=tx_count;inject05_result_on_tx=results[i];
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    assert(tx_count==before+1 && tx_commands[before]==0x05);
    assert(!mission_snapshot.vision_target_valid && frame_lift==0);
    uint32_t review_start=mission_snapshot.state_entry_tick;
    //Duplicate05 must not reset this hold, erase the result or echo05.
    fake_now=review_start+20U;rx(0x05);tick(fake_now);
    assert(mission_snapshot.state_entry_tick==review_start && tx_count==before+1);
    tick(review_start+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    tick(review_start+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);assert_review_route(results[i]);
    if(results[i]==0x06) {
      uint8_t event;assert(MaixCam_PeekEvent(&event) && event==0x16);
    }
    assert(tx_count==before+1);
  }
  //An unsolicited05 + result can arrive between the budget decision and
  //opening the result gate. Keep that window and skip redundant TX05.
  for(unsigned i=0;i<sizeof(results);++i) {
    uint32_t first=budget_at_phase(MISSION_STATE_S4_TRACK_FINAL_BLOCK);
    unsigned before=tx_count;inject05_result_on_stop=results[i];
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && tx_count==before);
    uint32_t review_start=mission_snapshot.state_entry_tick;
    tick(review_start+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);assert_review_route(results[i]);assert(tx_count==before);
    if(results[i]==0x06) {
      uint8_t event;assert(MaixCam_PeekEvent(&event) && event==0x16);
    }
  }
  //MAX2 outside an existing count window also requests05 once, keeping
  //the original session. Calling the wait entry again cannot resend it.
  uint32_t first=budget_at_phase(MISSION_STATE_S4_TRACK_FINAL_BLOCK);
  supplement_capture_count=2;unsigned before=tx_count;
  MissionTask_WaitFinalLoadCheck(fake_now,"MAX2_LIMIT");
  assert(tx_count==before+1 && tx_commands[before]==0x05);
  MissionTask_WaitFinalLoadCheck(fake_now+20U,"BUDGET_ALREADY_USED");
  assert(tx_count==before+1 && supplement_capture_count==2 && supplement_first_request_tick==first);
  //The deadline and the reply hold remain valid across tick wrapping.
  setup(0x51,true);request();start_tracking();
  first=UINT32_MAX-(ROBOT_SUPPLEMENT_TOTAL_MS-10U);supplement_first_request_tick=first;
  fake_now=first+ROBOT_SUPPLEMENT_TOTAL_MS-20U;MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,fake_now);
  before=tx_count;
  assert(!MissionTask_HandleSupplementFinish(first+ROBOT_SUPPLEMENT_TOTAL_MS-1U));
  tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);assert(tx_count==before+1 && tx_commands[before]==0x05);
  //Any TX/servo failure stops safely, never turns an assumed load into06.
  for(unsigned failure=0;failure<3;++failure) {
    first=budget_at_phase(MISSION_STATE_S4_TRACK_FINAL_BLOCK);before=tx_count;
    if(failure==0) tx_ok=false;
    if(failure==1) frame_lower_ok=false;
    if(failure==2) near_ok=false;
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
    assert(mission_snapshot.fault_flags & (failure==0?MISSION_FAULT_VISION_TX:MISSION_FAULT_ACTUATOR));
    assert(tx_count==before+(failure==0?1:0));
    unsigned sent=tx_count;tick(first+20000U);assert(tx_count==sent);
  }
  puts("PASS: exhausted03/04/24 budgets TX05 once, full-frame closure/retained session, preTX and inTX05+ID+06/02/24 bursts, no postTX event clear/no AUTO06,300ms/duplicate/partial-frame/wrap and failure safety");
}
static void verify_direct_supplement03(void) {
  const MissionState phases[]={MISSION_STATE_SUPPLEMENT_WIDE_SETTLE,
      MISSION_STATE_SUPPLEMENT_TRACK,MISSION_STATE_SUPPLEMENT_TURN_LEFT,
      MISSION_STATE_SUPPLEMENT_TURN_RIGHT,MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW};
  for(unsigned i=0;i<sizeof(phases)/sizeof(phases[0]);++i) {
    setup(0x51,true);request();start_tracking();
    uint32_t first=supplement_first_request_tick;uint8_t count=supplement_capture_count;
    uint32_t now=fake_now+20;MissionTask_EnterState(phases[i],now);
    coords(5,40,60);MaixCam_Object old,current;assert(MaixCam_GetObjectSnapshot(&old));
    unsigned near_before=near_calls,sent=tx_count,lifts=frame_raise_calls+frame_partial_calls;
    rx(0x05);rx(0x05);
    assert(MaixCam_GetObjectSnapshot(&current) && current.sequence==old.sequence);
    assert(!maixcam_load_recheck_enabled && !MaixCam_TakeLoadRecheckAck());
    assert(!MaixCam_TakeSupplementLoadCheckRequest());
    assert(MaixCam_TakeIgnoredSupplement05Count()==2);
    rx(0x05);fake_now=now;tick(now);
    assert(near_calls==near_before && tx_count==sent && lifts==frame_raise_calls+frame_partial_calls);
    assert(mission_snapshot.state!=MISSION_STATE_S6_RECHECK_WAIT_RESULT && !s5_close_view_active);
    assert(supplement_first_request_tick==first && supplement_capture_count==count && s6_carried_object_id==6);
    // Repeated legacy03 cannot create another attempt or resend51/03.
    rx(0x03);tick(now+20);
    assert(supplement_first_request_tick==first && supplement_capture_count==count && tx_count==sent);
  }
  //05 cannot overwrite a one-shot04/24 already received in the same burst.
  for(unsigned k=0;k<2;++k) {
    setup(0x51,true);request();start_tracking();
    fake_now+=20;rx(k?0x24:0x04);rx(0x05);tick(fake_now);
    assert(mission_snapshot.state==(k?MISSION_STATE_S4_RAISE_FRAME:MISSION_STATE_S4_TRACK_CENTER));
    assert(!maixcam_supplement_direct_search);
    fake_now+=20;rx(0x05);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT); // final review still valid after handoff
  }
  // A five-byte05 partial frame stays suppressed after entering04, but
  // eleven-byte green coordinates (same ID byte05) are never suppressed.
  setup(0x51,true);request();start_tracking();
  byte(0xF1);byte(0xF2);byte(0x05);byte(0x1F);
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,fake_now+20);
  byte(0x2F);assert(!MaixCam_TakeLoadRecheckAck() && MaixCam_TakeIgnoredSupplement05Count()==1);
  //TX51/TX03-time legacy05 cannot cancel the trailing03 handshake.
  setup(0x51,true);request();inject06_on_tx=0x51;
  uint32_t entry=mission_snapshot.state_entry_tick;unsigned before=tx_count;
  tick(entry+500);
  assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x03);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && !maixcam_load_recheck_enabled);
  fake_now=entry+520;coords(5,40,60);tick(fake_now);
  assert(mission_snapshot.vision_target_valid);
  tick(entry+540); // consume the fixture's obsolete16 before the ordinary PID tick
  assert(mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm>0);
  // Budget-end requested05 overrides suppression even while the old03
  // phase is still selected: accept an immediate05+ID+06 reply safely.
  setup(0x51,true);request();start_tracking();
  uint32_t first=supplement_first_request_tick;inject05_result_on_tx=0x06;
  tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  tick(first+15500);assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  puts("PASS:supplement03 direct wide tracking, legacy entry05 ignored without coordinate/event/cargo/budget loss, duplicate03/TX burst/partial05 fences;04/24 and final budget05 review preserved");
}
static uint32_t supplement_first_park(unsigned partial) {
  setup(0x51,true);request();start_tracking();
  fake_now+=20;rx(0xE3);tick(fake_now);
  int32_t center=mission_snapshot.yaw_cdeg;
  assert(maixcam_supplement_e3_turn_active);
  fake_now+=20;mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+9000L);
  coords(5,20,60);rx(0x04);rx(0x14);rx(0x24);rx(0x34);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && !mission_snapshot.vision_target_valid);
  assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==100);
  if(partial) { byte(0xF1);byte(0xF2);byte(partial==1?5:(partial==2?0x04:0xAE)); }
  fake_now+=20;mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+18000L);tick(fake_now);
  assert(supplement_e3_first_hold_active && !maixcam_supplement_e3_turn_active);
  assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  uint8_t event;assert(!MaixCam_PeekEvent(&event));
  return fake_now;
}
static void verify_supplement_first_hold(void) {
  // Parking, not any turn-era coordinate/full arrival packet, is the new
  // control boundary. Even an old half-frame completing while parked is stale.
  for(unsigned partial=1;partial<=2;++partial) {
    uint32_t parked=supplement_first_park(partial);unsigned sent=tx_count;
    if(partial==1) {
      const uint8_t tail[]={0,20,1,0,60,2,0x1F,0x2F};
      for(unsigned i=0;i<sizeof(tail);++i) byte(tail[i]);
    } else { byte(0x1F);byte(0x2F); }
    tick(parked+20);
    assert(supplement_e3_first_hold_active && !mission_snapshot.vision_target_valid && tx_count==sent);
    fake_now=parked+40;coords(5,20,60);rx(0x04);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && !supplement_e3_first_hold_active);
    assert(tx_count==sent && !supplement_e3_first_ack_sent);
    tick(parked+60); // new04 is retained, not cleared by the parking handoff
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && tx_commands[tx_count-1]==0x04);
  }
  const uint8_t bad_ids[]={3,4};
  for(unsigned i=0;i<2;++i) {
    uint32_t parked=supplement_first_park(0);unsigned sent=tx_count;
    fake_now=parked+20;coords(bad_ids[i],20,60);tick(fake_now);
    assert(supplement_e3_first_hold_active && !mission_snapshot.vision_target_valid);
    tick(parked+499);assert(tx_count==sent && !mission_snapshot.left_target_rpm);
    tick(parked+500);assert(tx_count==sent+1 && tx_commands[sent]==0x13);
  }
  // Stale parking coordinates do not suppress TX13; ID6 at499ms does.
  uint32_t parked=supplement_first_park(0);unsigned sent=tx_count;
  fake_now=parked-10;coords(5,20,60);tick(parked+500);
  assert(!mission_snapshot.vision_target_valid && tx_count==sent+1 && tx_commands[sent]==0x13);
  parked=supplement_first_park(0);sent=tx_count;
  fake_now=parked+499;coords(6,20,60);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && tx_count==sent && !supplement_e3_first_ack_sent);
  // AE still cancels the turn/hold; its independent request is not mistaken
  // for the old motion frames, even if it crosses the parking boundary.
  parked=supplement_first_park(3);sent=tx_count;byte(0x1F);byte(0x2F);
  fake_now=parked+20;MissionTask_UpdateVisionInput(fake_now);
  MissionTask_HandleStartZoneAvoidanceRequest(fake_now);MissionTask_RunStartZoneAvoidance(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && !supplement_e3_first_hold_active);
  assert(!maixcam_supplement_e3_turn_active && tx_count==sent);
  setup(0x51,true);request();start_tracking();fake_now+=20;rx(0xE3);tick(fake_now);
  sent=tx_count;fake_now+=20;rx(0xAE);
  MissionTask_HandleStartZoneAvoidanceRequest(fake_now);MissionTask_RunStartZoneAvoidance(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && !maixcam_supplement_e3_turn_active && tx_count==sent);
  // Budget and safety preempt the nonblocking hold, never sending old13.
  parked=supplement_first_park(0);sent=tx_count;
  supplement_first_request_tick=parked-(ROBOT_SUPPLEMENT_TOTAL_MS-100U);
  tick(parked+99);assert(supplement_e3_first_hold_active && tx_count==sent);
  tick(parked+100);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK && !supplement_e3_first_hold_active);
  assert(supplement_budget_exhausted && tx_count==sent+1 && tx_commands[sent]==0x05);
  const uint32_t faults[]={MISSION_FAULT_IMU,MISSION_FAULT_LEFT_MOTOR,MISSION_FAULT_RIGHT_MOTOR};
  for(unsigned i=0;i<3;++i) {
    parked=supplement_first_park(0);sent=tx_count;health_faults=faults[i];tick(parked+20);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !supplement_e3_first_hold_active && tx_count==sent);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !maixcam_supplement_e3_turn_active);
  }
  parked=supplement_first_park(0);sent=tx_count;
  MissionTask_EnterState(MISSION_STATE_STOPPED,parked+20);tick(parked+40);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && tx_count==sent && !supplement_e3_first_hold_active);
}
static void verify_supplement_e3_recovery(void) {
  verify_supplement_first_hold();
  assert(ROBOT_SUPPLEMENT_E3_TURN_MAX_RPM==30.0f && ROBOT_SUPPLEMENT_E3_SPIN_RPM==30);
  assert(ROBOT_SUPPLEMENT_E3_SPIN_CDEG==36000 && ROBOT_SUPPLEMENT_TURN_MAX_RPM==20.0f);
  assert(ROBOT_SUPPLEMENT_E3_FIRST_SPIN_RPM==100 && ROBOT_SUPPLEMENT_E3_FIRST_SPIN_CDEG==18000);
  assert(ROBOT_SUPPLEMENT_E3_FIRST_HOLD_MS==500U);
  assert(MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360==97 && MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW==98);
  // First actual E3: accumulated CCW180 at100rpm; duplicates cannot restart it.
  setup(0x51,true);request();start_tracking();
  unsigned sent=tx_count;uint32_t first=supplement_first_request_tick;
  uint32_t now=fake_now+20;fake_now=now;rx(0xE3);tick(now);
  assert(supplement_e3_recovery_count==1 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  int32_t center=mission_snapshot.yaw_cdeg;
  assert(supplement_e3_spin_goal_cdeg==18000 && supplement_e3_spin_rpm==100);
  rx(0xE3);tick(now+20);
  assert(supplement_e3_recovery_count==1 && mission_snapshot.state_entry_tick==now);
  assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==100);
  tick(now+2020);assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && tx_count==sent);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+9000L);tick(now+2040);
  assert(supplement_e3_spin_progress_cdeg==9000 && !supplement_e3_first_ack_sent);
  rx(0xE3);tick(now+2060);
  assert(supplement_e3_spin_progress_cdeg==9000 && mission_snapshot.state_entry_tick==now);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+17999L);tick(now+2080);
  assert(supplement_e3_spin_progress_cdeg==17999 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  assert(tx_count==sent && mission_snapshot.left_target_rpm==-100);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+18000L);tick(now+2100);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && supplement_e3_first_hold_active);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && tx_count==sent);
  assert(!maixcam_supplement_e3_turn_active && !supplement_e3_first_ack_sent);
  fake_now=now+2200;rx(0xE3);tick(fake_now);
  assert(supplement_e3_first_hold_tick==now+2100 && supplement_e3_recovery_count==1);
  tick(now+2599);assert(supplement_e3_first_hold_active && tx_count==sent);
  tick(now+2600);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && supplement_e3_recovery_count==1);
  assert(tx_count==sent+1 && tx_commands[sent]==0x13 && supplement_e3_first_ack_sent);
  assert(supplement_first_request_tick==first && supplement_capture_count==1);
  sent=tx_count;
  // Second E3 accumulates a real360 across0/360; no fixed-turn2s shortcut.
  fake_now=now+2620;rx(0xE3);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && supplement_e3_recovery_count==2);
  assert(supplement_e3_spin_goal_cdeg==36000 && supplement_e3_spin_rpm==30);
  uint32_t spin=mission_snapshot.state_entry_tick;
  assert(MissionTask_ActiveRecoveryLossEvent()==0xE3 && maixcam_supplement_direct_search);
  tick(spin+20);assert(mission_snapshot.left_target_rpm==-30 && mission_snapshot.right_target_rpm==30);
  // Wrong/stale target classes must not cancel the spin.
  fake_now=spin+40;coords(4,20,60);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && !mission_snapshot.vision_target_valid);
  fake_now=spin+60;coords(5,20,60);tick(fake_now+501);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && !mission_snapshot.vision_target_valid);
  tick(spin+2020);assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  for(unsigned i=1;i<=3;++i) {
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+9000L);
    fake_now=spin+2020+i*20;rx(0xE3);tick(fake_now);
    assert(supplement_e3_recovery_count==2 && mission_snapshot.state_entry_tick==spin);
  }
  assert(supplement_e3_spin_progress_cdeg==27000 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+8999L);tick(spin+2100);
  assert(supplement_e3_spin_progress_cdeg==35999 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+1L);tick(spin+2120);
  assert(supplement_e3_spin_progress_cdeg==36000 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK);
  assert(!mission_snapshot.left_target_rpm && tx_count==sent && supplement_first_request_tick==first);
  // First180 ignores all turn-era XY. Only fresh parked-view ID5/6 may
  // interrupt its500ms hold; later360 keeps its original XY interruption.
  for(unsigned i=0;i<2;++i) {
    setup(0x51,true);request();start_tracking();first=supplement_first_request_tick;sent=tx_count;
    supplement_e3_recovery_count=i?1:0;supplement_e3_first_ack_sent=i!=0;
    now=fake_now+20;fake_now=now;rx(0xE3);tick(now);
    uint8_t used=supplement_e3_recovery_count;
    fake_now=now+40;coords(4,20,60);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && !mission_snapshot.vision_target_valid);
    fake_now=now+60;coords(6,20,60);tick(fake_now+501U);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && !mission_snapshot.vision_target_valid);
    fake_now=now+581U;coords(i&1?6:5,20,60);tick(fake_now);
    if(i==0) {
      assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && maixcam_supplement_e3_turn_active);
      assert(!mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm==-100);
      center=mission_snapshot.yaw_cdeg;
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+9000L);tick(now+600);
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+18000L);tick(now+620);
      assert(supplement_e3_first_hold_active && !mission_snapshot.vision_target_valid && tx_count==sent);
      fake_now=now+640;coords(5,20,60);tick(fake_now);
    }
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && supplement_e3_recovery_count==used);
    assert(tx_count==sent && supplement_first_request_tick==first && supplement_capture_count==1);
    // If interrupted before TX13, an8s resend still belongs to round1.
    // If TX13 already completed, the next accepted E3 remains the spin.
    fake_now=now+8000;rx(0xE3);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
    assert(supplement_e3_spin_goal_cdeg==(i?36000:18000) && supplement_e3_spin_rpm==(i?30:100));
    assert(supplement_e3_recovery_count==(i?used+1:used) && tx_count==sent);
    //The deadline still ends the maneuver and requests final review exactly once.
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(supplement_budget_exhausted && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && tx_commands[tx_count-1]==0x05);
  }
  // Silent timeout uses the same search, but does not consume an actual E3.
  setup(0x51,true);request();start_tracking();now=fake_now;
  tick(now+1000);assert(supplement_e3_recovery_count==0 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TURN_LEFT);
  fake_now=now+1020;coords(5,20,60);tick(fake_now);
  fake_now=now+1540;rx(0xE3);tick(fake_now);
  assert(supplement_e3_recovery_count==1 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  // A real first E3 during a silent sweep starts the new180 once. Duplicates
  // cannot restart it. TX13-time E3 survives for the next task tick.
  setup(0x51,true);request();start_tracking();now=fake_now;
  tick(now+1000);
  fake_now=now+1020;rx(0xE3);tick(fake_now);
  assert(supplement_e3_recovery_count==1 && mission_snapshot.state_entry_tick==now+1020U);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && supplement_e3_spin_goal_cdeg==18000);
  rx(0xE3);tick(now+1040);
  assert(mission_snapshot.state_entry_tick==now+1020U && mission_snapshot.left_target_rpm==-100);
  center=mission_snapshot.yaw_cdeg;
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+17999L);tick(now+1060);
  sent=tx_count;inject_e3_on13_tx=true;
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+18000L);tick(now+1080);
  assert(supplement_e3_first_hold_active && !supplement_e3_first_ack_sent && tx_count==sent);
  tick(now+1580);
  assert(supplement_e3_first_ack_sent && tx_count==sent+1 && tx_commands[sent]==0x13);
  tick(now+1600);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && supplement_e3_recovery_count==2);
  assert(supplement_e3_spin_goal_cdeg==36000 && supplement_e3_spin_rpm==30);
  // Failed TX13 must not open round2 or continue moving.
  setup(0x51,true);request();start_tracking();now=fake_now+20;fake_now=now;rx(0xE3);tick(now);
  center=mission_snapshot.yaw_cdeg;
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+9000L);tick(now+20);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(center+18000L);tx_ok=false;tick(now+40);
  assert(supplement_e3_first_hold_active && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW);
  tick(now+540);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !supplement_e3_first_ack_sent);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  // Recheck03 continues the same attempt/count; a new extra attempt in the
  // same session also retains this counter.
  setup(0x51,true);request();start_tracking();supplement_e3_recovery_count=2;supplement_e3_first_ack_sent=true;
  first=supplement_first_request_tick;fake_now+=20;rx(0x04);tick(fake_now);
  fake_now+=20;rx(0x05);tick(fake_now);fake_now+=500;rx(0x03);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WIDE_SETTLE && supplement_e3_recovery_count==2 && supplement_e3_first_ack_sent);
  assert(supplement_first_request_tick==first && supplement_capture_count==1);
  MissionTask_StartSupplement(fake_now+20,fake_now+20,true);
  assert(supplement_e3_recovery_count==2 && supplement_capture_count==2 && supplement_first_request_tick==first);
  // First180 remains a real angle accumulation across both yaw and tick wrap.
  setup(0x51,true);request();start_tracking();
  now=UINT32_MAX-2200U;supplement_first_request_tick=now;
  mission_snapshot.yaw_cdeg=35900;fake_now=now;rx(0xE3);tick(now);
  tick(now+1999U);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW && !supplement_e3_first_ack_sent);
  mission_snapshot.yaw_cdeg=17899;tick(now+2000U);
  assert(supplement_e3_spin_progress_cdeg==17999 && !supplement_e3_first_ack_sent);
  sent=tx_count;mission_snapshot.yaw_cdeg=17900;tick(now+2020U);
  assert(supplement_e3_first_hold_active && !supplement_e3_first_ack_sent);
  tick(now+2519U);assert(supplement_e3_first_hold_active && tx_count==sent);
  tick(now+2520U);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && supplement_e3_first_ack_sent);
  assert(tx_count==sent+1 && tx_commands[sent]==0x13);
  // Stop and hardware faults remain effective during both spin profiles.
  const uint32_t faults[]={MISSION_FAULT_IMU,MISSION_FAULT_LEFT_MOTOR,MISSION_FAULT_RIGHT_MOTOR};
  for(unsigned round=0;round<2;++round) for(unsigned i=0;i<3;++i) {
    setup(0x51,true);request();start_tracking();supplement_e3_recovery_count=round;supplement_e3_first_ack_sent=round!=0;
    fake_now+=20;rx(0xE3);tick(fake_now);health_faults=faults[i];tick(fake_now+20);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!supplement_e3_recovery_count && !supplement_capture_active && !supplement_e3_first_ack_sent);
  }
  for(unsigned round=0;round<2;++round) {
    setup(0x51,true);request();start_tracking();supplement_e3_recovery_count=round;supplement_e3_first_ack_sent=round!=0;
    fake_now+=20;rx(0xE3);tick(fake_now);MissionTask_EnterState(MISSION_STATE_STOPPED,fake_now+20);
    tick(fake_now+40);assert(mission_snapshot.state==MISSION_STATE_STOPPED && !supplement_e3_recovery_count);
  }
  puts("PASS:supplement03 firstE3 CCW100/180deg ignores turn-era XY,then parked500ms accepts only new ID5/6 or TX13 at deadline;later CCW30/360 remains XY-interruptible,yaw/hold tick wrap;pre13 resends never escalate,duplicates/silent conversion,TX13 race/failure,budget review and spin/hold health/stop safety");
}
static uint32_t finish_34_safe_turn(uint32_t now) {
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE);
  unsigned sent=tx_count;
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
  tick(now);
  tick(now+ROBOT_S6_TURN_STABLE_MS-1U);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && tx_count==sent);
  tick(now+ROBOT_S6_TURN_STABLE_MS);
  return fake_now;
}

static void verify_14_direct24_claw(void) {
  const uint8_t commands[]={0x21,0x31,0x11,0x51,0x51};
  const uint8_t ids[]={5,6,4,5,6};
  const uint8_t adjacent[]={0x02,0x32,0x42,0x12};
  assert(ROBOT_S4_DIRECT_CAPTURE_AFTER_14==1U);
  for(unsigned c=0;c<sizeof(commands);++c) {
    setup(commands[c],c!=0);frame_lift=77; //PA2 must not move.
    supplement_first_supply_ack_sent=c!=0 && ids[c]!=4U; // Later captures retain normal05.
    MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,100);
    fake_now=120;coords(ids[c],0,20);rx(0x14);tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY && s4_direct_capture_wait24);
    assert(tx_count==1 && tx_commands[0]==0x14 && !claw_open_calls && !claw_close_calls);
    for(unsigned a=0;a<sizeof(adjacent);++a) {
      fake_now+=20;rx(adjacent[a]);MissionTask_HandleS4FineArrangeRequest(fake_now);tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY && mission_snapshot.state_entry_tick==120);
      assert(tx_count==1 && !claw_open_calls && !frame_lower_calls && !nonzero_wheel_requests);
    }
    fake_now+=20;rx(0x24);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME && !s4_direct_capture_wait24);
    unsigned is_casualty=ids[c]==4;
    assert(final_view_calls==is_casualty && wide_calls==1U-is_casualty && !near_calls);
    assert(claw_open_calls==1 && claw_angle==(ids[c]==4?68U:64U));
    assert(frame_lift==77 && !frame_partial_calls && !frame_lower_calls && tx_count==1);
    coords(ids[c],0,50);tick(fake_now); //No elapsed-time gate at all.
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_FINAL_BLOCK && mission_snapshot.vision_target_valid);
    tick(fake_now+20U);assert(nonzero_wheel_requests && claw_open_calls==1 && !claw_close_calls);
    assert(final_view_calls==is_casualty && wide_calls==1U-is_casualty && !near_calls);
    unsigned moves=nonzero_wheel_requests;
    fake_now+=20U;rx(0x34);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_close_calls==1);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && nonzero_wheel_requests==moves);
    (void)finish_34_safe_turn(fake_now+20U);
    assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && tx_count==2 && tx_commands[1]==0x25);
    assert(claw_close_calls==1 && claw_angle==ROBOT_CLAW_CLOSED_DEG && frame_lift==77);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
  //24 arriving during the14 ACK and34 before the next dispatch both survive.
  setup(0x11,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,100);
  inject_push_reply_on_tx=0x24;fake_now=120;rx(0x14);tick(120);tick(140);
  assert(claw_open_calls==1 && claw_angle==68 && tx_count==1);
  fake_now=140;rx(0x34);tick(140);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_close_calls==1);
  (void)finish_34_safe_turn(fake_now+20U);
  assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && claw_angle==ROBOT_CLAW_CLOSED_DEG);
  assert(tx_count==2 && tx_commands[1]==0x25 && !nonzero_wheel_requests);
  //Direct black31 stays wide before sending31/24.
  setup(0x51,true);frame_lift=77;MissionTask_StartBlackFinalTrack(120);
  assert(claw_open_calls==1 && claw_angle==64U && frame_lift==77);
  assert(!final_view_calls && wide_calls==1 && !near_calls);
  assert(tx_count==1 && tx_commands[0]==0x31 && !frame_partial_calls);
  tick(120);assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_FINAL_BLOCK);
  assert(tx_count==2 && tx_commands[1]==0x24);
  setup(0x51,true);request();start_tracking();
  unsigned wide_before=wide_calls;
  fake_now+=20;rx(0x24);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME && supplement_capture_active);
  assert(!final_view_calls && wide_calls==wide_before+1 && claw_angle==64U);
  tick(fake_now+20U);
  assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_FINAL_BLOCK && supplement_capture_active);
  assert(!final_view_calls && wide_calls==wide_before+1);
  setup(0x11,true);claw_open_ok=false;fake_now=120;rx(0x24);tick(120);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags&MISSION_FAULT_ACTUATOR));
  assert(!nonzero_wheel_requests && !tx_count && claw_open_calls==1);
  setup(0x11,true);final_view_ok=false;fake_now=120;rx(0x24);tick(120);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && (mission_snapshot.fault_flags&MISSION_FAULT_ACTUATOR));
  assert(final_view_calls==1 && !claw_open_calls && !nonzero_wheel_requests && !tx_count);
  puts("PASS:14 waits24/E4;24 material and supplement stay wide,casualty uses capture view,opens PA3 by class,keeps new XY/early34;34 stops/closes;black direct31/24 and actuator faults checked");
}

static uint32_t finish_24_timeout_backoff(uint32_t start) {
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF);
  unsigned sent=tx_count;
  assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
  assert(claw_angle==36 && !mission_snapshot.vision_turn_rpm);
  // Late ROI/loss/34 cannot steer, interrupt, or be mistaken for a new capture.
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+18000L);
  fake_now=start+20U;coords(5,90,300);rx(0xE4);rx(0x34);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF && !mission_snapshot.vision_target_valid);
  tick(start+399U);
  assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100 && tx_count==sent);
  tick(start+400U);
  return start+400U;
}

static void verify_24_timeout_retry03(void) {
  assert(ROBOT_S4_FINAL_TRACK_RETRY_MS==1000U);
  assert(ROBOT_S4_FINAL_TIMEOUT_BACKOFF_RPM==100 && ROBOT_S4_FINAL_TIMEOUT_BACKOFF_MS==400U);
  const uint8_t targets[]={0x21,0x11,0x31,0x51};
  const uint8_t ids[]={5,4,6,6};
  for(unsigned i=0;i<sizeof(targets);++i) for(unsigned wrap=0;wrap<3;++wrap) {
    setup(targets[i],i!=0);s4_green_e4_push_cycles=2;
    uint32_t start=wrap==1?UINT32_MAX-500U:wrap==2?UINT32_MAX-1100U:500U;
    uint32_t deadline=start+ROBOT_S4_FINAL_TRACK_RETRY_MS;
    MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,start);
    fake_now=deadline-1U;coords(ids[i],25,90);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_FINAL_BLOCK && !tx_count);
    //The previous ROI coordinate must not survive the timeout.
    fake_now=deadline;tick(deadline);
    bool backoff=targets[i]!=0x11;
    uint32_t restarted=backoff?finish_24_timeout_backoff(deadline):deadline;
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && mission_snapshot.state_entry_tick==restarted);
    assert(tx_count==1 && tx_commands[0]==0x03 && s3_target_select_command==targets[i]);
    assert(frame_lower_calls==(backoff?2U:1U) && claw_angle==ROBOT_CLAW_CLOSED_DEG && track_view_calls==1 && !wide_calls);
    assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!mission_snapshot.fault_flags && s4_green_e4_push_cycles==2);
    MissionTask_UpdateVisionInput(restarted+20U);assert(!mission_snapshot.vision_target_valid);
  }
  setup(0x21,false);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,120);
  const uint32_t retry_tick=120U+ROBOT_S4_FINAL_TRACK_RETRY_MS;
  fake_now=retry_tick;rx(0x34);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && !tx_count && !nonzero_wheel_requests);

  setup(0x21,false);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,120);
  fake_now=retry_tick;inject_retry03_id=5;tick(fake_now);
  (void)finish_24_timeout_backoff(retry_tick);
  MissionTask_UpdateVisionInput(fake_now);
  uint8_t event;
  assert(mission_snapshot.vision_target_valid && mission_snapshot.vision_y_error_px==-95);
  assert(MaixCam_TakeEvent(&event) && event==0x04 && tx_count==1);
  MissionTask_StartS4TrackCenterFrom04(fake_now+20U);
  assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && track_view_calls==1 && wide_calls==1);
  assert(tx_count==2 && tx_commands[1]==0x04);

  for(unsigned expiry=0;expiry<2;++expiry) {
    setup(0x51,true);request();start_tracking();
    uint32_t first=supplement_first_request_tick;
    uint32_t start=first+(expiry?ROBOT_SUPPLEMENT_TOTAL_MS-ROBOT_S4_FINAL_TRACK_RETRY_MS:700U);
    supplement_e3_recovery_count=1;supplement_e3_first_ack_sent=true;
    uint8_t count=supplement_capture_count;unsigned sent=tx_count;
    MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,start);
    tick(start+ROBOT_S4_FINAL_TRACK_RETRY_MS);
    if(!expiry) (void)finish_24_timeout_backoff(start+ROBOT_S4_FINAL_TRACK_RETRY_MS);
    assert(supplement_first_request_tick==first && supplement_capture_count==count);
    assert(s6_carried_object_id==6 && tx_count==sent+1);
    if(expiry) {
      assert(tx_commands[sent]==0x05 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    } else {
      assert(tx_commands[sent]==0x03 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK);
      assert(supplement_capture_active && supplement_reference_command==0x03);
      assert(supplement_e3_recovery_count==1 && supplement_e3_first_ack_sent);
    }
  }
  for(unsigned mode=0;mode<3;++mode) {
    setup(0x21,false);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,120);
    if(mode==0) frame_lower_ok=false;
    if(mode==1) wide_ok=false;
    if(mode==2) tx_ok=false;
    tick(retry_tick);
    if(mode!=0) (void)finish_24_timeout_backoff(retry_tick);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
    assert(mission_snapshot.fault_flags&(mode==2?MISSION_FAULT_VISION_TX:MISSION_FAULT_ACTUATOR));
    assert(tx_count==(mode==2?1U:0U));
  }
  // Final review/health can still stop an in-progress backoff.
  for(unsigned mode=0;mode<3;++mode) {
    setup(0x51,true);
    uint32_t deadline=retry_tick;
    if(mode==0) {
      request();start_tracking();
      deadline=supplement_first_request_tick+ROBOT_SUPPLEMENT_TOTAL_MS-100U;
    }
    s51_active_object_id=0; // Even a51 that never got any XY is still a supply search.
    MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,deadline-ROBOT_S4_FINAL_TRACK_RETRY_MS);
    tick(deadline);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TIMEOUT_BACKOFF);
    tick(deadline+40U);
    assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
    unsigned sent=tx_count;
    fake_now=deadline+100U;
    if(mode==1) health_faults=MISSION_FAULT_IMU;
    if(mode==2) rx(0x05);
    tick(fake_now);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(mission_snapshot.state==(mode==0?MISSION_STATE_SUPPLEMENT_WAIT_RECHECK:
        mode==1?MISSION_STATE_FAULT:MISSION_STATE_S6_RECHECK_WAIT_RESULT));
    if(mode==0) assert(tx_count==sent+1 && tx_commands[sent]==0x05);
    else assert(tx_count==sent);
  }
  puts("PASS:24 999/1000: supply closes and backs at 100rpm for400ms then TX03; red direct03; wrap,old ROI/loss fenced,new XY/04 retained,34 priority,budget and faults");
}
static void verify_exit_supplement_then_red_e3(void) {
  for(unsigned route=0;route<2;++route) {
    setup(0x51,true);request();start_tracking();
    uint32_t first=supplement_first_request_tick;
    uint8_t count=supplement_capture_count;
    if(route) s7_casualty_delivery_count=ROBOT_S7_MATERIAL_ONLY_AFTER_CASUALTIES;
    else s7_material_delivery_count=ROBOT_S7_MATERIAL_ONLY_AFTER_TRIPS;
    unsigned sent=tx_count;
    inject05_result_on_tx=route?0x02:0x24;
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(tx_count==sent+1 && tx_commands[sent]==0x05 && supplement_budget_exhausted);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    uint32_t review=mission_snapshot.state_entry_tick+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS;
    tick(review);
    if(route) {
      assert(mission_snapshot.state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE && supplement_budget_started);
      tick(review+ROBOT_S5_ARRANGE_REVERSE_MS);
    } else {
      assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME && s6_carried_object_id==0);
      assert(!supplement_budget_started && !supplement_capture_active);
      tick(review+20U);
      uint32_t deadline=mission_snapshot.state_entry_tick+ROBOT_S4_FINAL_TRACK_RETRY_MS;
      tick(deadline);
      (void)finish_24_timeout_backoff(deadline);
    }
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !supplement_budget_started && !supplement_capture_active);
    assert(!maixcam_supplement_window_enabled && !maixcam_supplement_direct_search);
    assert(supplement_budget_exhausted && supplement_capture_count==count && supplement_first_request_tick==first);
    assert(tx_count==sent+2 && tx_commands[sent+1]==0x03);
    sent=tx_count;fake_now+=20U;
    coords(6,0,100);rx(0xE3);rx(0x11);tick(fake_now);
    assert(tx_count==sent+2 && tx_commands[sent]==0x11 && tx_commands[sent+1]==0x03);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s3_target_select_command==0x11);
    assert(red_switch_success_logs==1 && !mission_snapshot.vision_target_valid);
    assert(supplement_budget_exhausted && supplement_capture_count==count);
    fake_now+=20U;coords(6,0,100);MissionTask_UpdateVisionInput(fake_now);
    assert(!mission_snapshot.vision_target_valid);
    coords(4,0,100);MissionTask_UpdateVisionInput(fake_now);
    assert(MissionTask_RedCoordinateIsFresh());
  }
  // An empty, unexpired05 also stops this run. A later eligible attempt
  // uses the existing clock and remaining allowance, not a fresh budget.
  setup(0x51,true);request();start_tracking();
  uint32_t first=supplement_first_request_tick;
  fake_now+=20U;rx(0x04);tick(fake_now);
  fake_now+=20U;rx(0x05);rx(0x24);tick(fake_now);
  tick(fake_now+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);
  assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME && !supplement_budget_started);
  assert(!supplement_capture_active && !supplement_budget_exhausted && supplement_capture_count==1);
  MissionTask_StartSupplement(fake_now+20U,fake_now+20U,true);
  assert(supplement_capture_count==2 && supplement_first_request_tick==first);
  puts("PASS:SUPP timeout05->24/02->ordinary03 E3+11 yields TX11/03 OK and fresh ID4 despite material policy; unexpired 02 keeps clock, 24 does not replenish attempts");
}

static void verify_34_claw_and_review(void) {
  const int32_t errors[]={500,1000,1500,2000,9000};
  const int16_t capture_rpm[]={8,12,42,75,90};
  const int16_t arrange_rpm[]={8,12,18,25,40};
  for(unsigned capture=0;capture<2;++capture) for(int sign=-1;sign<=1;sign+=2) {
    for(unsigned i=0;i<sizeof(errors)/sizeof(errors[0]);++i) {
      setup(0x51,true);
      MissionTask_EnterState(capture?MISSION_STATE_S4_FINAL_TURN_SAFE:MISSION_STATE_S5_ALIGN_FOR_ARRANGE,100);
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.safe_zone_yaw_target_cdeg+sign*errors[i]);
      MissionTask_ResetS6ControllerState();
      s6_phase_stable_since=0U;
      assert(!MissionTask_RunS6TurnToHeading(120,mission_snapshot.safe_zone_yaw_target_cdeg));
      assert(mission_snapshot.left_target_rpm==sign*ROBOT_YAW_LEFT_SIGN*(capture?capture_rpm[i]:arrange_rpm[i]));
      assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    }
  }
  const uint8_t commands[]={0x21,0x31,0x11,0x51,0x51};
  const uint8_t ids[]={5,6,4,5,6};
  for(unsigned c=0;c<sizeof(commands);++c) for(unsigned red=0;red<2;++red)
    for(unsigned wrap=0;wrap<2;++wrap) {
      setup(commands[c],c!=0);frame_lift=25; //PA2 fixture, not a24 command.
      // For post07 supplies this fixture exercises later captures, not the first34 bypass.
      supplement_first_supply_ack_sent=c!=0 && ids[c]!=4U;
      uint32_t start=wrap?UINT32_MAX-10U:100U;
      mission_snapshot.team=red?ROBOT_TEAM_RED:ROBOT_TEAM_BLUE;
      MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,start);
      claw_angle=ids[c]==4?ROBOT_CLAW_CASUALTY_CAPTURE_OPEN_DEG:ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG;
      mission_snapshot.left_target_rpm=80;mission_snapshot.right_target_rpm=60;
      fake_now=start;coords(ids[c],5,30);rx(0x34);tick(start);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && !tx_count && claw_close_calls==1);
      assert(mission_snapshot.state_entry_tick==start && claw_angle==ROBOT_CLAW_CLOSED_DEG);
      assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
      assert(!nonzero_wheel_requests && !claw_open_calls);
      fake_now=start+20U;rx(0x34);rx(0xEE);coords(ids[c],75,140);
      uint32_t reviewed=finish_34_safe_turn(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN);
      assert(mission_snapshot.state_entry_tick==reviewed && s6_carried_object_id==ids[c]);
      assert(tx_count==1 && tx_commands[0]==(red?0x15:0x25));
      assert(claw_close_calls==1 && !frame_lower_calls && !frame_raise_calls && !frame_partial_calls);
      assert(frame_lift==25 && !wide_calls && !near_calls);
      assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
      //A repeated34 during05 does not move, reclose or resend.
      fake_now=reviewed+20U;rx(0x34);coords(ids[c],20,120);tick(fake_now);tick(reviewed+600U);
      assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN);
      assert(claw_close_calls==1 && tx_count==1 && !nonzero_wheel_requests);
      fake_now=reviewed+620U;rx(0x05);tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
      assert(near_calls==1 && claw_close_calls==1 && tx_count==1 && !frame_lower_calls);
    }
  //Already-in24 early34 and EE recovery stop and close without advancing.
  setup(0x21,false);s4_final_mode_already_active=true;
  MissionTask_EnterState(MISSION_STATE_S4_RAISE_FRAME,100);fake_now=120;rx(0x34);tick(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_close_calls==1);
  (void)finish_34_safe_turn(140);
  assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && claw_close_calls==1);
  assert(tx_count==1 && tx_commands[0]==0x25 && !nonzero_wheel_requests && !frame_lower_calls);
  setup(0x21,false);MissionTask_EnterState(MISSION_STATE_S4_FINAL_RECOVERY_REVERSE,100);
  fake_now=120;rx(0x34);assert(MissionTask_TryResumeS4FinalRecovery(120));
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_close_calls==1);
  (void)finish_34_safe_turn(140);
  assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && claw_close_calls==1);
  assert(tx_count==1 && tx_commands[0]==0x25 && !nonzero_wheel_requests && !frame_lower_calls);
  //Direct supplementary34 enters the same05 handshake without resetting its budget.
  setup(0x51,true);request();start_tracking();
  unsigned sent=tx_count, lower=frame_lower_calls, moves=nonzero_wheel_requests;
  uint32_t first=supplement_first_request_tick;uint8_t count=supplement_capture_count;
  fake_now+=20U;coords(6,0,20);rx(0x34);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_angle==ROBOT_CLAW_CLOSED_DEG && claw_close_calls==1);
  assert(!claw_open_calls && tx_count==sent && nonzero_wheel_requests==moves);
  (void)finish_34_safe_turn(fake_now+20U);
  assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && claw_close_calls==1);
  assert(tx_count==sent+1 && tx_commands[sent]==0x25);
  assert(supplement_budget_started && !supplement_capture_active && supplement_first_request_tick==first);
  assert(supplement_capture_count==count && frame_lower_calls==lower && nonzero_wheel_requests==moves);
  const uint32_t capture_tick=120U;
  for(unsigned failed_tx=0;failed_tx<2;++failed_tx) {
    setup(0x11,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
    claw_close_ok=failed_tx!=0;tx_ok=failed_tx==0;
    fake_now=120;rx(0x34);tick(120);
    if(failed_tx) (void)finish_34_safe_turn(capture_tick+20U);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(claw_close_calls==1 && !frame_lower_calls && !nonzero_wheel_requests);
    assert(tx_count==failed_tx);
    assert(mission_snapshot.fault_flags&(failed_tx?MISSION_FAULT_VISION_TX:MISSION_FAULT_ACTUATOR));
  }
  for(unsigned review=0;review<2;++review) {
    setup(0x21,false);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
    claw_close_inhibited=1;
    fake_now=120;rx(0x34);tick(120);
    if(review) { fake_now=140;rx(0x05); }
    else health_faults=MISSION_FAULT_IMU;
    tick(140);
    assert(mission_snapshot.state==(review?MISSION_STATE_S6_RECHECK_WAIT_RESULT:MISSION_STATE_FAULT));
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !tx_count);
    assert(claw_close_calls==review && near_calls==review);
    tick(160);assert(!nonzero_wheel_requests && !tx_count);
  }
  setup(0x11,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
  claw_close_inhibited=1;
  fake_now=120;rx(0x34);tick(120);tick(240);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_CENTER_OBJECT);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !claw_close_calls && !tx_count);
  claw_close_inhibited=0;tick(260);
  (void)finish_34_safe_turn(280);
  assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && claw_close_calls==1 && tx_count==1);

  for(unsigned result=0;result<3;++result) {
    setup(0x11,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
    mission_snapshot.safe_zone_yaw_target_cdeg=35000;
    mission_snapshot.yaw_cdeg=8000; // -90 degree shortest turn across the wrap.
    fake_now=120;rx(0x34);tick(120);
    fake_now=capture_tick+20U;rx(0x05);rx(0x34);coords(4,100,100);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && mission_snapshot.state_entry_tick==capture_tick);
    assert(mission_snapshot.left_target_rpm==90*ROBOT_YAW_LEFT_SIGN);
    assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    assert(mission_snapshot.safe_zone_yaw_command_cdeg==35000 && !mission_snapshot.vision_target_valid);
    assert(claw_close_calls==1 && !tx_count && !near_calls);
    if(result==0) {
      (void)finish_34_safe_turn(capture_tick+40U);
    } else if(result==1) {
      tick(capture_tick+ROBOT_HEADING_TURN_TIMEOUT_MS-1U);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && !tx_count);
      tick(capture_tick+ROBOT_HEADING_TURN_TIMEOUT_MS);
      assert(mission_snapshot.yaw_cdeg==8000); // No fabricated heading on timeout.
    } else {
      health_faults=MISSION_FAULT_IMU;
      tick(capture_tick+40U);assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count);
    }
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !near_calls);
    if(result!=2) assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && tx_count==1 && tx_commands[0]==0x25);
  }
  puts("PASS:34 all cargo stop and close without advancing,claw interlock preserved,shortest IMU turn75..90 before15/25,near taper retained,early05/XY/duplicates ignored,turn deadline and faults,supplement budget preserved");
}
static void verify_first_supply_capture_auto_supplement(void) {
  const uint8_t commands[]={0x21,0x31,0x51,0x51,0x51};
  const uint8_t ids[]={5,6,5,6,0};
  for(unsigned i=0;i<sizeof(commands);++i) {
    setup(commands[i],true);s51_active_object_id=0;
    uint32_t captured=i==4?UINT32_MAX-200U:120U;
    mission_snapshot.team=(i&1)?ROBOT_TEAM_RED:ROBOT_TEAM_BLUE;
    MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,captured-20U);
    claw_angle=ROBOT_CLAW_MATERIAL_CAPTURE_OPEN_DEG;
    first34_ack_delay_ms=37U;
    fake_now=captured;
    if(ids[i]) coords(ids[i],0,10);
    rx(0x34);tick(captured);
    const uint8_t retained_id=ids[i]?ids[i]:5U;
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WIDE_SETTLE && !near_calls);
    assert(claw_close_calls==1 && claw_angle==36 && !nonzero_wheel_requests);
    assert(tx_count==1 && tx_commands[0]==0x34 && s3_target_select_command==0x51);
    assert(supplement_first_supply_ack_sent && supplement_capture_active && supplement_budget_started);
    assert(supplement_capture_count==1 && supplement_first_request_tick==captured);
    assert(s6_carried_object_id==retained_id && !s5_load_class_confirmed && track_view_calls==1);
    uint32_t entry=captured+37U, ready=entry+ROBOT_SUPPLEMENT_WIDE_SETTLE_MS;
    assert(mission_snapshot.state_entry_tick==entry);
    fake_now=entry+50U;rx(0x34);coords(6,20,120);tick(fake_now);
    tick(ready-1U);
    assert(tx_count==1 && supplement_capture_count==1 && !nonzero_wheel_requests);
    tick(ready);
    uint8_t event;
    assert(tx_count==3 && tx_commands[1]==0x51 && tx_commands[2]==0x03);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK);
    assert(!MaixCam_PeekEvent(&event) && !nonzero_wheel_requests);
    const uint8_t next_id=retained_id==5U?6U:5U;
    fake_now=ready+250U;coords(next_id,0,40);tick(fake_now);
    assert(mission_snapshot.vision_target_valid && s6_carried_object_id==retained_id);
    fake_now=ready+270U;rx(0x24);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME);
    tick(ready+290U);
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_FINAL_BLOCK && claw_open_calls==1);
    fake_now=ready+310U;coords(next_id,0,10);rx(0x34);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_close_calls==2 && tx_count==3);
    uint32_t reviewed=finish_34_safe_turn(ready+330U);
    assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN);
    assert(tx_count==4 && tx_commands[3]==((i&1)?0x15:0x25));
    assert(supplement_capture_count==1 && supplement_first_request_tick==captured && !supplement_capture_active);
    fake_now=reviewed+20U;rx(0x05);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && near_calls==1);
    final_report(5,reviewed+20U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE && s5_load_class_confirmed);
  }
  // Deferred claw close must not ACK or start the budget early.
  setup(0x31,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
  claw_close_inhibited=1;fake_now=120;rx(0x34);tick(120);tick(140);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_CENTER_OBJECT && !tx_count && !supplement_budget_started);
  claw_close_inhibited=0;tick(160);
  assert(tx_count==1 && tx_commands[0]==0x34 && supplement_first_request_tick==160U);
  tick(160U+ROBOT_SUPPLEMENT_WIDE_SETTLE_MS);
  tick(160U+ROBOT_SUPPLEMENT_TOTAL_MS-1U);
  assert(!supplement_budget_exhausted);
  tick(160U+ROBOT_SUPPLEMENT_TOTAL_MS);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK && tx_commands[tx_count-1]==0x05);
  assert(supplement_budget_exhausted && !supplement_capture_active && s6_carried_object_id==6);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);

  for(unsigned failure=0;failure<3;++failure) {
    setup(0x51,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
    claw_close_ok=failure!=0;tx_ok=failure!=1;wide_ok=failure!=2;
    fake_now=120;rx(0x34);tick(120);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !nonzero_wheel_requests);
    assert(tx_count==(failure?1U:0U));
    assert(mission_snapshot.fault_flags&(failure==1?MISSION_FAULT_VISION_TX:MISSION_FAULT_ACTUATOR));
  }
  puts("PASS:post07 first ID5/6/51-no-XY closes and TX34 then settles500 and TX51/03; old34/XY fenced,wrap,second34 normal15/25 and ID+06,12s from claw close,interlock and failures");
}

static uint32_t start_casualty_regrip(unsigned mode, bool wrap) {
  setup(mode==4?0x21:0x11,mode!=4);
  uint32_t start=wrap?UINT32_MAX-400U:120U;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,start);
  fake_now=start;
  if(mode==3) rx(0x54); //Out of order: it must not pair with a later34.
  rx(0x34);
  if(mode==0 || mode==4) { rx(0x54);rx(0x54); }
  tick(start);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && claw_close_calls==1);
  assert(!claw_open_calls && !tx_count && !mission_snapshot.vision_forward_rpm);
  tick(start+20U);
  assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm && mission_snapshot.left_target_rpm);
  if(mode==1) { fake_now=start+40U;rx(0x54);tick(fake_now); }
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && !claw_open_calls && !tx_count);
  assert(!mission_snapshot.vision_forward_rpm);
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
  tick(start+100U);
  uint32_t aligned=start+100U+ROBOT_S6_TURN_STABLE_MS;
  tick(aligned-1U);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_TURN_SAFE && !claw_open_calls && !tx_count);
  return aligned;
}

static void verify_54_casualty_regrip(void) {
  assert(ROBOT_S4_CASUALTY_REGRIP_RPM==70 && ROBOT_S4_CASUALTY_REGRIP_MS==300U);
  for(unsigned mode=0;mode<5;++mode) {
    uint32_t aligned=start_casualty_regrip(mode,mode==1);
    tick(aligned);
    if(mode>=2) {
      assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && !claw_open_calls);
      assert(tx_count==1 && tx_commands[0]==0x25 && !mission_snapshot.vision_forward_rpm);
      fake_now=aligned+20U;rx(0x54);tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && !claw_open_calls && tx_count==1);
      continue;
    }
    assert(mission_snapshot.state==MISSION_STATE_S4_CASUALTY_REGRIP && claw_angle==62);
    assert(claw_open_calls==1 && claw_close_calls==1 && !tx_count);
    assert(mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);
    fake_now=aligned+100U;rx(0x54);rx(0x34);rx(0x54);coords(4,200,220);tick(fake_now);
    assert(!mission_snapshot.vision_target_valid && mission_snapshot.state_entry_tick==aligned);
    tick(aligned+299U);
    assert(mission_snapshot.state==MISSION_STATE_S4_CASUALTY_REGRIP && !tx_count && claw_close_calls==1);
    tick(aligned+300U);
    assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && claw_close_calls==2 && claw_open_calls==1);
    assert(claw_angle==36 && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(tx_count==1 && tx_commands[0]==0x25 && s6_carried_object_id==4);
  }
  for(unsigned failure=0;failure<3;++failure) {
    uint32_t aligned=start_casualty_regrip(0,false);
    if(failure==0) claw_open_ok=false;
    tick(aligned);
    if(failure==1) { claw_close_ok=false;tick(aligned+300U); }
    if(failure==2) { health_faults=MISSION_FAULT_IMU;tick(aligned+20U); }
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
  setup(0x11,true);MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
  fake_now=120;rx(0x34);rx(0x54);tick(120);
  tick(120U+ROBOT_HEADING_TURN_TIMEOUT_MS);
  assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && !claw_open_calls && tx_count==1);
  puts("PASS:54 only after red34,burst and mid-turn RX,turn-first,70rpm/300ms and wrap,repeat/out-of-order/material ignored,open/close/health safety and no push on unaligned turn timeout");
}

static uint32_t start_45_adjust(bool wrap) {
  setup(0x11,true);s6_carried_object_id=4;
  uint32_t count_start=wrap?UINT32_MAX-400U:120U;
  fake_now=count_start;rx(0x05);rx(0x45);tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && near_calls==1);
  uint32_t start=count_start+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS;
  tick(start-1U);
  assert(!claw_open_calls && !tx_count && !mission_snapshot.left_target_rpm);
  tick(start);
  return start;
}

static void verify_45_casualty_load_adjust(void) {
  assert(ROBOT_CLAW_CASUALTY_ADJUST_OPEN_DEG==59U && ROBOT_CLAW_MATERIAL_OPEN_DEG==62U);
  const uint32_t duration=ROBOT_S5_CASUALTY_ADJUST_REVERSE_MS;
  assert(ROBOT_S5_CASUALTY_ADJUST_REVERSE_RPM==70 && duration==300U);
  for(unsigned wrap=0;wrap<2;++wrap) {
    uint32_t start=start_45_adjust(wrap!=0);
    assert(s5_casualty_adjust_active && claw_angle==59U && claw_open_calls==1 && !tx_count);
    assert(mission_snapshot.left_target_rpm==-70 && mission_snapshot.right_target_rpm==-70);
    fake_now=start+200U;rx(0x05);rx(0x45);coords(4,0,0);rx(0x06);rx(0x16);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && near_calls==1);
    assert(s5_casualty_adjust_start_tick==start && mission_snapshot.left_target_rpm==-70);
    tick(start+duration-1U);
    assert(s5_casualty_adjust_active && !tx_count && !claw_close_calls && mission_snapshot.right_target_rpm==-70);
    if(wrap) inject06_on45=true;
    else { byte(0xF1);byte(0xF2);byte(0x06);byte(0x1F); } //Old partial06.
    tick(start+duration);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    assert(!s5_casualty_adjust_active && claw_angle==36 && claw_close_calls==1);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(tx_count==1 && tx_commands[0]==0x45);
    if(!wrap) {
      byte(0x2F);
      fake_now=start+duration+20U;rx(0x45);tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && tx_count==2 && tx_commands[1]==0x45);
      assert(claw_open_calls==1 && claw_close_calls==1);
      fake_now=start+duration+40U;coords(4,0,0);rx(0x06);rx(0x16);
    }
    tick(start+duration+40U);
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE && s6_carried_object_id==4);
    uint8_t event;assert(MaixCam_TakeEvent(&event) && event==0x16);
  }
  setup(0x51,true);fake_now=120;rx(0x05);coords(5,0,0);rx(0x45);tick(fake_now);
  tick(420);
  assert(!s5_casualty_adjust_active && !claw_open_calls && !tx_count);
  uint32_t start=start_45_adjust(false);
  health_faults=MISSION_FAULT_IMU;tick(start+20U);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !s5_casualty_adjust_active &&
      !mission_snapshot.left_target_rpm && !tx_count && !maixcam_load_recheck_enabled);
  for(unsigned failed_tx=0;failed_tx<2;++failed_tx) {
    start=start_45_adjust(false);
    claw_close_ok=failed_tx!=0;tx_ok=failed_tx==0;
    tick(start+duration);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.right_target_rpm);
    assert(tx_count==failed_tx && (mission_snapshot.fault_flags&
        (failed_tx?MISSION_FAULT_VISION_TX:MISSION_FAULT_ACTUATOR)));
  }
  puts("PASS:05 casualty45 waits settle,claw59,straight-70/300,close36 then TX45;duplicates,partial early06,in-TX ID4/06/16,wrap,non-casualty and safety");
}

static void verify_05_stops_before_counting(void) {
  for(unsigned red=0;red<2;++red) {
    setup(0x21,false);
    mission_snapshot.team=red?ROBOT_TEAM_RED:ROBOT_TEAM_BLUE;
    MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
    MissionTask_SetWheelTargets(100,80);
    fake_now=120;MissionTask_EnterS5(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN);
    assert(tx_count==1 && tx_commands[0]==(red?0x15:0x25) && !near_calls);
    unsigned stops_at_entry=stops;
    fake_now=140;coords(5,30,80);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN && stops>stops_at_entry);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    fake_now=160;rx(0x05);tick(fake_now);
    assert(near_calls==1 && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(nonzero_wheel_requests==1);
  }
  puts("PASS:ordinary05 stops before15/25,keeps zero targets while waiting,and stops before near view");
}

int main(void) {
  verify_start_zone_avoidance();
  verify_fine_arrange_integration();
  verify_14_direct24_claw();
  verify_24_timeout_retry03();
  verify_exit_supplement_then_red_e3();
  verify_34_claw_and_review();
  verify_first_supply_capture_auto_supplement();
  verify_54_casualty_regrip();
  verify_45_casualty_load_adjust();
  verify_05_stops_before_counting();
  assert(ROBOT_S6_RECHECK_CAMERA_SETTLE_MS==300U);
  verify_vision_approach();
  verify_prepare_reverse_without_yaw();
  verify_recovery_forward_without_yaw();
  verify_recovery_forward_lock();
  verify_recovery_count_persists();
  verify_budget_request_handshake();
  verify_direct_supplement03();
  verify_supplement_e3_recovery();
  //03/supplement03 have independent100..150/150..170/170..200 bands.
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-50,128);
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-140,160);
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-200,170);
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-324,185);
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-448,200);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-50,128);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-140,160);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-200,170);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-324,185);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-448,200);
  verify_speed_case(MISSION_STATE_S6_TRACK_SAFE_ZONE,0x03,-50,150);
  verify_speed_case(MISSION_STATE_S6_TRACK_SAFE_ZONE,0x03,-448,179);
  verify_speed_case(MISSION_STATE_S6_TRACK_SAFE_ZONE,0x03,-450,180);
  verify_speed_case(MISSION_STATE_S6_TRACK_SAFE_ZONE,0x03,450,-150);
  //04 remains70..100 near,120..150 middle,150..170 far;24 is also unchanged.
  verify_speed_case(MISSION_STATE_S4_TRACK_CENTER,0x04,-50,87);
  verify_speed_case(MISSION_STATE_S4_TRACK_CENTER,0x04,-140,135);
  verify_speed_case(MISSION_STATE_S4_TRACK_CENTER,0x04,-200,150);
  verify_speed_case(MISSION_STATE_S4_TRACK_CENTER,0x04,-448,170);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x04,-50,87);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x04,-448,170);
  verify_speed_case(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x24,-50,57);
  verify_speed_case(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x24,-200,70);
  verify_speed_case(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x24,-448,120);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x24,-50,57);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x24,-448,120);
  assert(MISSION_STATE_S3_E3_TURN_AWAY==95 && MISSION_STATE_SUPPLEMENT_WAIT_RECHECK==96);
  const MissionState phases[]={MISSION_STATE_S5_WAIT_SINGLE_GREEN,
    MISSION_STATE_S3_TRACK_GREEN,MISSION_STATE_S3_SEARCH_TURN_CW,
    MISSION_STATE_S3_SEARCH_TURN_CCW,MISSION_STATE_S3_E3_TURN_AWAY,
    MISSION_STATE_S4_TRACK_CENTER,MISSION_STATE_S4_WAIT_ARRANGE_READY,
    MISSION_STATE_S4_TRACK_RIGHT_BLOCK,MISSION_STATE_S4_REVERSE_RIGHT_BLOCK,
    MISSION_STATE_S4_TRACK_LEFT_BLOCK,MISSION_STATE_S4_REVERSE_LEFT_BLOCK,
    MISSION_STATE_S4_WAIT_FINAL_READY,MISSION_STATE_S4_RAISE_FRAME,
    MISSION_STATE_S4_TRACK_FINAL_BLOCK,MISSION_STATE_S4_FINAL_CENTER_OBJECT,
    MISSION_STATE_S4_FINAL_LOWER_FRAME,MISSION_STATE_S4_E4_PUSH_RIGHT,
    MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360,
    MISSION_STATE_S4_E4_REVERSE_RIGHT,MISSION_STATE_S4_E4_PUSH_LEFT,
    MISSION_STATE_S4_E4_REVERSE_LEFT,MISSION_STATE_S4_POST22_REVERSE,
    MISSION_STATE_S4_POST22_TURN_LEFT,MISSION_STATE_S4_POST22_TURN_RIGHT,
    MISSION_STATE_S4_FINAL_RECOVERY_REVERSE,MISSION_STATE_S4_FINAL_RECOVERY_TURN_270,
    MISSION_STATE_S4_FINAL_RECOVERY_TURN_90,MISSION_STATE_S5_ALIGN_FOR_ARRANGE,
    MISSION_STATE_S5_RAISE_FOR_ARRANGE,MISSION_STATE_S5_REVERSE_FOR_ARRANGE,
    MISSION_STATE_S5_LOWER_FOR_ARRANGE,MISSION_STATE_SUPPLEMENT_LOAD_CHECK,
    MISSION_STATE_S6_SEARCH_SAFE_ZONE,MISSION_STATE_S6_TRACK_SAFE_ZONE,
    MISSION_STATE_S6_RECOVERY_START,MISSION_STATE_S6_RECOVERY_TURN_LEFT,
    MISSION_STATE_S6_CORNER_TURN_SAFE,MISSION_STATE_S6_RECHECK_WAIT_ACK};
  for(unsigned i=0;i<sizeof(phases)/sizeof(phases[0]);++i) {
    setup(0x51,true);
    if(phases[i]>=MISSION_STATE_SUPPLEMENT_WIDE_SETTLE && phases[i]<=MISSION_STATE_SUPPLEMENT_LOAD_CHECK) {
      supplement_budget_started=true; supplement_first_request_tick=100;
      supplement_capture_count=1; supplement_capture_active=true;
    }
    MissionTask_EnterState(phases[i],100);
    mission_snapshot.left_target_rpm=100; mission_snapshot.right_target_rpm=-100;
    s4_final_event34_pending=s4_e4_protocol_active=s4_post22_search_active=true;
    vision_search_forward_active=vision_search_e3_mode=true; supplement_recovery_event=0xE3;
    unsigned lifts=frame_raise_calls+frame_partial_calls, lowers=frame_lower_calls;
    fake_now=120; rx(0x05); rx(0xE3); rx(0x14); rx(0x26); coords(4,60,70); tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!mission_snapshot.vision_target_valid && s6_carried_object_id==6);
    assert(!s4_final_event34_pending && !s4_e4_protocol_active && !s4_post22_search_active);
    assert(!vision_search_forward_active && !vision_search_e3_mode && !supplement_recovery_event);
    assert(near_calls==1 && tx_count==0 && lifts==frame_raise_calls+frame_partial_calls && lowers==frame_lower_calls);
    tick(120U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    claw_angle=ROBOT_CLAW_MATERIAL_OPEN_DEG; // The06 entry must explicitly restore the closed pose.
    final_report(i&1?4:5,120U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
    assert(s6_carried_object_id==(i&1?4:5) && s5_load_class_confirmed && !supplement_budget_started);
    assert(frame_lift==0 && claw_angle==ROBOT_CLAW_CLOSED_DEG && safe_view_calls==1 && !wide_calls && baffle_stop_calls==1 && tx_count==0);
    assert(MissionTask_S6IsCasualtyTarget()==(bool)(i&1));
  }
  // A new05 is accepted while already near; same-window repeats never erase
  // a report, restart300ms, move the frame or echo05 back to the camera.
  setup(0x11,true); s5_close_view_active=true;
  fake_now=120; rx(0x05); tick(120); unsigned near_before=near_calls;
  fake_now=300; coords(4,0,0); rx(0x05); rx(0xE4); tick(300);
  assert(near_calls==near_before+1 && mission_snapshot.state_entry_tick==120);
  assert(maixcam_load_recheck_report.object_id==4 && !tx_count && !frame_raise_calls);
  fake_now=620; rx(0x06); tick(620); assert(s6_carried_object_id==4);
  // Full burst must preserve05, its frozen report and06 against later events.
  setup(0x51,true); fake_now=120; rx(0x05); coords(5,0,0); rx(0x06); rx(0x02); rx(0x34); rx(0x16);
  tick(120); assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  tick(620); assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE && s6_carried_object_id==5);
  assert(!mission_snapshot.left_target_rpm && tx_count==0);
  // Nonzero target coordinates, missing and expired reports cannot confirm06.
  for(unsigned variant=0;variant<3;++variant) {
    setup(0x11,true); fake_now=120; rx(0x05); tick(120);
    fake_now=620;
    if(variant==1) coords(4,10,10);
    if(variant==2) { coords(4,0,0); fake_now+=501; }
    rx(0x06); tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm);
    final_report(4,fake_now+20); assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  }
  // Initial green remains locked to a fresh ID5 report, not ID4/ID6/default.
  for(uint8_t id=3;id<=6;++id) if(id!=5) {
    setup(0x21,false); s6_carried_object_id=5;
    fake_now=120; rx(0x05); tick(120); final_report(id,620);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm);
    final_report(5,700); assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  }
  //02/24 are accepted only after the count hold and keep the existing routes.
  for(unsigned result=0;result<2;++result) {
    setup(0x51,true); fake_now=120; rx(0x05); rx(result?0x24:0x02); tick(120);
    tick(120U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS-1U);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    tick(120U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);
    assert(mission_snapshot.state==(result?MISSION_STATE_S4_RAISE_FRAME:MISSION_STATE_S5_REVERSE_FOR_ARRANGE));
    assert(!maixcam_load_recheck_enabled && tx_count==0);
    assert(wide_calls==result && !final_view_calls);
  }
  // Post-capture04/24 review can still continue the same attempt with the
  // original budget clock. Only the03-entry recount is removed.
  setup(0x51,true); frame_lift=88; request(); start_tracking();
  uint32_t first=supplement_first_request_tick; unsigned count=supplement_capture_count;
  coords(6,40,60); tick(fake_now+20); assert(mission_snapshot.left_target_rpm>0);
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,fake_now);
  fake_now+=20; rx(0x05); tick(fake_now);
  assert(s5_recheck_resume_supplement && supplement_capture_count==count && supplement_first_request_tick==first);
  uint32_t paused=mission_snapshot.state_entry_tick;
  fake_now=paused+20; rx(0x03); tick(fake_now); //early03 remains latched until300ms
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  frame_lift=88;
  tick(paused+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS-1U);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  tick(paused+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WIDE_SETTLE);
  assert(frame_lift==0 && s6_carried_object_id==6);
  start_tracking(); assert(supplement_capture_count==count && supplement_first_request_tick==first);
  //The deadline never transports the cargo without a final05, including when counting.
  for(unsigned counting=0;counting<2;++counting) {
    setup(0x51,true); request(); start_tracking(); first=supplement_first_request_tick;
    if(counting) { MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,fake_now); fake_now+=20; rx(0x05); tick(fake_now); }
    unsigned before=tx_count; tick(first+ROBOT_SUPPLEMENT_TOTAL_MS-1U); assert(tx_count==before);
    tick(first+ROBOT_SUPPLEMENT_TOTAL_MS);
    assert(supplement_budget_exhausted && !supplement_capture_active && supplement_budget_started);
    assert(mission_snapshot.state==(counting?MISSION_STATE_S6_RECHECK_WAIT_RESULT:MISSION_STATE_SUPPLEMENT_WAIT_RECHECK));
    unsigned expected=before+(counting?0:1);
    assert(!mission_snapshot.left_target_rpm && tx_count==expected && s6_carried_object_id==6);
    if(!counting) assert(tx_commands[before]==0x05 && frame_lift==0);
    tick(first+20000); assert(mission_snapshot.state!=MISSION_STATE_S6_SEARCH_SAFE_ZONE && tx_count==expected);
    fake_now=first+20020; rx(0x05); tick(fake_now);
    fake_now+=20; rx(0x03); tick(fake_now); assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    final_report(5,first+20520); assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  }
  //MAX2 stops a third attempt but does not restart budgets or automatically06.
  setup(0x51,true); request(); start_tracking();
  supplement_capture_count=2; supplement_capture_active=false;
  MissionTask_EnterState(MISSION_STATE_S5_WAIT_SINGLE_GREEN,fake_now);
  fake_now+=20; rx(0x05); tick(fake_now); uint32_t final_start=mission_snapshot.state_entry_tick;
  fake_now=final_start+500; rx(0x03); tick(fake_now);
  assert(supplement_budget_exhausted && supplement_capture_count==2 && mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  unsigned before=tx_count; tick(fake_now+20); assert(tx_count==before && !mission_snapshot.left_target_rpm);
  final_report(5,fake_now+20); assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  //When06 and03 coexist, the frozen final06 has priority over continuation.
  setup(0x51,true); request(); start_tracking(); MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,fake_now); fake_now+=20; rx(0x05); tick(fake_now);
  fake_now+=500; coords(5,0,0); rx(0x03); rx(0x06); tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE && !supplement_budget_started);
  //05 received during direct03 is ignored (covered above); a genuine budget
  // expiry during TX still cancels the old03 and requests final review.
  setup(0x51,true); request();
  uint32_t entry=mission_snapshot.state_entry_tick; before=tx_count; tick(entry+500);
  assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x03 && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK);
  //A local budget expires during TX too: no trailing03 and no transport.
  setup(0x51,true); request(); expire_on_tx=true; entry=mission_snapshot.state_entry_tick; before=tx_count;
  tick(entry+500); assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x05);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
  //Corner review keeps its completed marker and accumulated8s tracking count.
  setup(0x51,true); s6_corner_completed=true; s6_track_elapsed_ms=1234;s6_recovery_count=2;
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK,100);
  fake_now=120; rx(0x05); tick(120); claw_angle=ROBOT_CLAW_MATERIAL_OPEN_DEG; final_report(5,620);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE && s6_corner_completed);
  assert(safe_view_calls==1 && !wide_calls);
  assert(s6_track_elapsed_ms==1234 && tx_count==0 && baffle_stop_calls==1 && claw_angle==ROBOT_CLAW_CLOSED_DEG);
  entry=mission_snapshot.state_entry_tick; tick(entry+799);
  assert(!mission_snapshot.left_target_rpm); tick(entry+800);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed && s6_track_elapsed_ms==1234);
  assert(s6_recovery_count==2);
  //Corner review02 may arrange the SAME retained load; only24/new07 clears
  //the completed corner, not another15/25 ->05 after that arrangement.
  setup(0x51,true); s6_corner_completed=true;
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK,100);
  fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0x02); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_S5_REVERSE_FOR_ARRANGE && s5_recheck_resume_safe_tracking);
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_LOWER_FRAME,700);
  MissionTask_EnterS5(700); assert(s6_corner_completed && s5_recheck_resume_safe_tracking);
  fake_now=720; rx(0x05); tick(720); final_report(5,1220);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE && s6_corner_completed);
  //Original first-green empty05 E5 still starts its uninterruptible pushes;
  //the completed-corner review must not start E5 or a new supplement budget.
  setup(0x21,false); fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0xE5); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT && !maixcam_load_recheck_enabled);
  setup(0x21,false); s6_corner_completed=true;
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK,100);
  fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0xE5); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !supplement_budget_started);
  //Closing05 discards an old half coordinate, but the UART remains armed.
  setup(0x11,true); fake_now=120; rx(0x05); tick(120); fake_now=620; coords(4,0,0); rx(0x06);
  byte(0xF1); byte(0xF2); byte(4); byte(0); tick(620);
  byte(0); byte(1); byte(0); byte(0); byte(1); byte(0x1F); byte(0x2F);
  MissionTask_UpdateVisionInput(640); assert(!mission_snapshot.vision_target_valid);
  MissionTask_EnterS6TrackingFrom16(660); fake_now=680; coords(4,0,0);
  MissionTask_UpdateVisionInput(680); assert(!mission_snapshot.vision_target_valid && s6_carried_object_id==4);
  coords(2,30,40); MissionTask_UpdateVisionInput(680); assert(mission_snapshot.vision_target_valid);
  //Camera/TX/IMU/stop failures cannot enter transport or resume old recovery.
  setup(0x51,true); frame_lift=88; frame_lower_ok=false;
  fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0x03); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && tx_count==0);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  setup(0x51,true); request(); start_tracking(); frame_lower_ok=false;
  unsigned sent_before_error=tx_count;
  assert(!MissionTask_SendSupplementCommand(0x03,fake_now+20));
  assert(mission_snapshot.state==MISSION_STATE_FAULT && tx_count==sent_before_error);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_ACTUATOR);
  setup(0x51,true); near_ok=false; rx(0x05); tick(120); assert(mission_snapshot.state==MISSION_STATE_FAULT);
  setup(0x51,true); rx(0x05); tick(120); health_faults=MISSION_FAULT_IMU; tick(140);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  setup(0x51,true); rx(0x05); tick(120); MissionTask_EnterState(MISSION_STATE_STOPPED,140); tick(160);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !mission_snapshot.left_target_rpm && !maixcam_load_recheck_enabled);
  assert(fake_rx_arms==rx_bytes+uart_inits);
  puts("PASS:6E long-forward coordinate/16 lock, fresh/stale resume and command/safety priority; final05 interrupts ordinary/capture/arrangement/recovery/corner states except direct supplementary03; stop/near/retained load/no ACK; repeated05, zero-ID reports+50ms06,300ms result hold,02/24 routes; priority/freshness/partial-frame fences; budget limits request05 ONCE/no AUTO06; same-attempt03/no budget reset; corner guard/8s count; servo/health/stop/UART safety");
}
