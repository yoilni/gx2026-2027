/* Current final05 protocol. Compiled with the real parser and the extracted
   mission functions by verify_supplement_capture.ps1; no target hardware. */
static void tick(uint32_t now) {
  fake_now=now; MissionTask_UpdateVisionInput(now);
  MissionTask_CheckRunningHealth(now); (void)MissionTask_HandleLoadCheckRequest(now);
  MissionTask_HandleSupplementControl(now); MissionTask_HandleRedPriorityRequest(now);
  run_normal_s4(now); MissionTask_RunS6LoadRecheck(now);
  MissionTask_RunS5WaitSingleGreen(now); MissionTask_RunSupplement(now);
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
  stops=wide_calls=near_calls=frame_raise_calls=frame_partial_calls=frame_lower_calls=tx_count=0;
  frame_lift=0; wide_ok=near_ok=frame_lower_ok=tx_ok=true; health_faults=0;
  inject06_on_tx=0; expire_on_tx=false; turn_soft_logs=search_soft_logs=0;
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
static void verify_recovery_forward_lock(void) {
  assert(ROBOT_S6_RECOVERY_LONG_FORWARD_RPM==70 && ROBOT_S6_RECOVERY_LONG_FORWARD_MS==2000);
  //Second and subsequent long-forward recoveries cannot finish from coordinates early.
  for(uint8_t count=1;count<=3;++count) {
    recovery_setup(count,true);recovery_tick(140,true);recovery_tick(1000,true);recovery_tick(2119,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && s6_recovery_count==count+1);
    assert(mission_snapshot.state_entry_tick==120 && mission_snapshot.left_target_rpm==70 && mission_snapshot.right_target_rpm==70);
    recovery_tick(2120,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
    assert(mission_snapshot.vision_target_valid && mission_snapshot.vision_x_error_px==30 && mission_snapshot.vision_y_error_px==-50);
  }
  //One-shot16 is deferred, not lost. Repeated16 does not discard newer coordinates.
  recovery_setup(1,false);fake_now=200;rx(0x16);recovery_tick(200,false);
  assert(s6_recovery_found16_pending && mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
  recovery_tick(300,true);fake_now=400;rx(0x16);recovery_tick(400,false);
  assert(mission_snapshot.vision_target_valid && s6_recovery_found16_pending && mission_snapshot.state_entry_tick==120);
  recovery_tick(2119,true);recovery_tick(2120,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !s6_recovery_found16_pending);
  assert(mission_snapshot.vision_target_valid && mission_snapshot.vision_x_error_px==30 && mission_snapshot.vision_y_error_px==-50);
  //Stale coordinates after saved16 never feed PID; resume stopped, awaiting new frames.
  recovery_setup(1,false);fake_now=200;rx(0x16);recovery_tick(200,false);
  recovery_tick(300,true);recovery_tick(2120,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !mission_snapshot.vision_target_valid);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  //Without16, initial search coordinates do not authorize tracking; the scan resumes.
  recovery_setup(1,false);recovery_tick(2119,true);recovery_tick(2120,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  fake_now=2140;rx(0x16);recovery_tick(2140,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  //No live coordinates: finish the forward leg, then allow scan interruption.
  recovery_setup(1,true);recovery_tick(2120,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_TURN_LEFT);
  recovery_tick(2140,true);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  //First sweep remains immediately interruptible, without mandatory forward motion.
  recovery_setup(0,true);recovery_tick(140,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  //Clock wrapping must not unlock the forward leg early.
  recovery_setup(1,true);uint32_t start=UINT32_MAX-1000U;
  mission_snapshot.state_entry_tick=start;
  recovery_tick(start+1999U,true);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
  recovery_tick(start+2000U,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
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
  fake_now=first+14980U;
  MissionTask_EnterState(phase,fake_now);
  mission_snapshot.left_target_rpm=80; mission_snapshot.right_target_rpm=60;
  if(phase==MISSION_STATE_S4_TRACK_FINAL_BLOCK) frame_lift=20;
  return first;
}
static void assert_review_route(uint8_t result) {
  if(result==0x06) {
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
    assert(s6_carried_object_id==5 && s5_load_class_confirmed && frame_lift==0);
    assert(!supplement_budget_started && !supplement_capture_active);
  } else if(result==0x02) {
    assert(mission_snapshot.state==MISSION_STATE_S5_ALIGN_FOR_ARRANGE);
  } else {
    assert(result==0x24 && mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME);
    assert(s6_carried_object_id==0);
  }
}
static void verify_budget_request_handshake(void) {
  //Budget expiry can preempt03,04,24 and their recovery/capture motion.
  const MissionState phases[]={MISSION_STATE_SUPPLEMENT_WIDE_SETTLE,
      MISSION_STATE_SUPPLEMENT_TRACK,MISSION_STATE_SUPPLEMENT_TURN_LEFT,
      MISSION_STATE_SUPPLEMENT_TURN_RIGHT,MISSION_STATE_S4_TRACK_CENTER,
      MISSION_STATE_S4_WAIT_ARRANGE_READY,MISSION_STATE_S4_TRACK_FINAL_BLOCK,
      MISSION_STATE_S4_FINAL_CENTER_OBJECT,MISSION_STATE_S4_FINAL_LOWER_FRAME};
  for(unsigned i=0;i<sizeof(phases)/sizeof(phases[0]);++i) {
    uint32_t first=budget_at_phase(phases[i]); unsigned before=tx_count;
    uint8_t count=supplement_capture_count;
    //An old partial coordinate must not survive the review transition.
    byte(0xF1);byte(0xF2);byte(6);byte(0);
    tick(first+15000U);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    assert(tx_count==before+1 && tx_commands[before]==0x05 && frame_lift==0);
    assert(s6_carried_object_id==6 && supplement_capture_count==count);
    assert(supplement_first_request_tick==first && supplement_budget_exhausted);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!supplement_capture_active && !vision_search_forward_active);
    byte(0);byte(1);byte(0);byte(0);byte(1);byte(0x1F);byte(0x2F);
    coords(5,30,40);rx(0x34);rx(0xE3);rx(0x14);
    tick(first+16000U); assert(tx_count==before+1 && !mission_snapshot.vision_target_valid);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    //No matching05: a zero-ID frame and06 cannot bypass final review.
    coords(5,0,0);rx(0x06);tick(first+16020U);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
    assert(tx_count==before+1);
    fake_now=first+16040U;rx(0x05);tick(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    tick(first+16539U);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    final_report(5,first+16540U); assert_review_route(0x06);
    assert(tx_count==before+1);
  }
  //A reply received while TX05 is completing may contain05/ID/result/16
  //in one burst. No clearing or duplicate request after sending.
  const uint8_t results[]={0x06,0x02,0x24};
  for(unsigned i=0;i<sizeof(results);++i) {
    uint32_t first=budget_at_phase(MISSION_STATE_S4_TRACK_FINAL_BLOCK);
    unsigned before=tx_count;inject05_result_on_tx=results[i];
    tick(first+15000U);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    assert(tx_count==before+1 && tx_commands[before]==0x05);
    assert(!mission_snapshot.vision_target_valid && frame_lift==0);
    uint32_t review_start=mission_snapshot.state_entry_tick;
    //Duplicate05 must not reset this hold, erase the result or echo05.
    fake_now=review_start+20U;rx(0x05);tick(fake_now);
    assert(mission_snapshot.state_entry_tick==review_start && tx_count==before+1);
    tick(review_start+499U);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    tick(review_start+500U);assert_review_route(results[i]);
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
    tick(first+15000U);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && tx_count==before);
    uint32_t review_start=mission_snapshot.state_entry_tick;
    tick(review_start+500U);assert_review_route(results[i]);assert(tx_count==before);
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
  //The15s decision and the reply hold remain valid across tick wrapping.
  setup(0x51,true);request();start_tracking();
  first=UINT32_MAX-14990U;supplement_first_request_tick=first;
  fake_now=first+14980U;MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,fake_now);
  before=tx_count;
  assert(!MissionTask_HandleSupplementFinish(first+14999U));
  tick(first+15000U);assert(tx_count==before+1 && tx_commands[before]==0x05);
  //Any TX/servo failure stops safely, never turns an assumed load into06.
  for(unsigned failure=0;failure<3;++failure) {
    first=budget_at_phase(MISSION_STATE_S4_TRACK_FINAL_BLOCK);before=tx_count;
    if(failure==0) tx_ok=false;
    if(failure==1) frame_lower_ok=false;
    if(failure==2) near_ok=false;
    tick(first+15000U);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
    assert(mission_snapshot.fault_flags & (failure==0?MISSION_FAULT_VISION_TX:MISSION_FAULT_ACTUATOR));
    assert(tx_count==before+(failure==0?1:0));
    unsigned sent=tx_count;tick(first+20000U);assert(tx_count==sent);
  }
  puts("PASS: exhausted03/04/24 budgets TX05 once, full-frame closure/retained session, preTX and inTX05+ID+06/02/24 bursts, no postTX event clear/no AUTO06,500ms/duplicate/partial-frame/wrap and failure safety");
}
static void verify_direct_supplement03(void) {
  const MissionState phases[]={MISSION_STATE_SUPPLEMENT_WIDE_SETTLE,
      MISSION_STATE_SUPPLEMENT_TRACK,MISSION_STATE_SUPPLEMENT_TURN_LEFT,
      MISSION_STATE_SUPPLEMENT_TURN_RIGHT};
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
  tick(first+15000);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  tick(first+15500);assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  puts("PASS:supplement03 direct wide tracking, legacy entry05 ignored without coordinate/event/cargo/budget loss, duplicate03/TX burst/partial05 fences;04/24 and final budget05 review preserved");
}
int main(void) {
  verify_vision_approach();
  verify_recovery_forward_lock();
  verify_budget_request_handshake();
  verify_direct_supplement03();
  //03/supplement03 slow down near the center; far and06 limits remain.
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-50,31);
  verify_speed_case(MISSION_STATE_S3_TRACK_GREEN,0x03,-448,150);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-50,31);
  verify_speed_case(MISSION_STATE_SUPPLEMENT_TRACK,0x03,-448,150);
  verify_speed_case(MISSION_STATE_S6_TRACK_SAFE_ZONE,0x03,-50,100);
  verify_speed_case(MISSION_STATE_S6_TRACK_SAFE_ZONE,0x03,-448,150);
  //04/24 far limits stay unchanged;24 slows down near its ROI reference.
  verify_speed_case(MISSION_STATE_S4_TRACK_CENTER,0x04,-448,70);
  verify_speed_case(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x24,-50,31);
  verify_speed_case(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x24,-448,120);
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
    tick(619); assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    final_report(i&1?4:5,620);
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
    assert(s6_carried_object_id==(i&1?4:5) && s5_load_class_confirmed && !supplement_budget_started);
    assert(frame_lift==0 && wide_calls==1 && tx_count==0);
    assert(MissionTask_S6IsCasualtyTarget()==(bool)(i&1));
  }
  // A new05 is accepted while already near; same-window repeats never erase
  // a report, restart500ms, move the frame or echo05 back to the camera.
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
    tick(619); assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    tick(620);
    assert(mission_snapshot.state==(result?MISSION_STATE_S4_RAISE_FRAME:MISSION_STATE_S5_ALIGN_FOR_ARRANGE));
    assert(!maixcam_load_recheck_enabled && wide_calls==1 && tx_count==0);
  }
  // Post-capture04/24 review can still continue the same attempt with the
  // original15s clock. Only the03-entry recount is removed.
  setup(0x51,true); frame_lift=88; request(); start_tracking();
  uint32_t first=supplement_first_request_tick; unsigned count=supplement_capture_count;
  coords(6,40,60); tick(fake_now+20); assert(mission_snapshot.left_target_rpm>0);
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,fake_now);
  fake_now+=20; rx(0x05); tick(fake_now);
  assert(s5_recheck_resume_supplement && supplement_capture_count==count && supplement_first_request_tick==first);
  uint32_t paused=mission_snapshot.state_entry_tick;
  fake_now=paused+20; rx(0x03); tick(fake_now); //early03 remains latched until500ms
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  frame_lift=88;
  tick(paused+500); assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WIDE_SETTLE);
  assert(frame_lift==0 && s6_carried_object_id==6);
  start_tracking(); assert(supplement_capture_count==count && supplement_first_request_tick==first);
  //15s never transports the cargo without a final05, including when counting.
  for(unsigned counting=0;counting<2;++counting) {
    setup(0x51,true); request(); start_tracking(); first=supplement_first_request_tick;
    if(counting) { MissionTask_EnterState(MISSION_STATE_S4_TRACK_CENTER,fake_now); fake_now+=20; rx(0x05); tick(fake_now); }
    unsigned before=tx_count; tick(first+14999); assert(tx_count==before);
    tick(first+15000);
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
  setup(0x51,true); s6_corner_completed=true; s6_track_elapsed_ms=1234;
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK,100);
  fake_now=120; rx(0x05); tick(120); final_report(5,620);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE && s6_corner_completed);
  assert(s6_track_elapsed_ms==1234 && tx_count==0);
  entry=mission_snapshot.state_entry_tick; tick(entry+799);
  assert(!mission_snapshot.left_target_rpm); tick(entry+800);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed && s6_track_elapsed_ms==1234);
  //Corner review02 may arrange the SAME retained load; only24/new07 clears
  //the completed corner, not another15/25 ->05 after that arrangement.
  setup(0x51,true); s6_corner_completed=true;
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK,100);
  fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0x02); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_S5_ALIGN_FOR_ARRANGE && s5_recheck_resume_safe_tracking);
  MissionTask_EnterState(MISSION_STATE_S4_FINAL_LOWER_FRAME,700);
  MissionTask_EnterS5(700); assert(s6_corner_completed && s5_recheck_resume_safe_tracking);
  fake_now=720; rx(0x05); tick(720); final_report(5,1220);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE && s6_corner_completed);
  //Original34 timing/class choice and downstream15/25 handshake still run
  //when there is no review interruption.
  //Original first-green empty05 E5 still starts its uninterruptible pushes;
  //the completed-corner review must not start E5 or a new supplement budget.
  setup(0x21,false); fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0xE5); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT && !maixcam_load_recheck_enabled);
  setup(0x21,false); s6_corner_completed=true;
  MissionTask_EnterState(MISSION_STATE_S6_RECHECK_WAIT_ACK,100);
  fake_now=120; rx(0x05); tick(120); fake_now=620; rx(0xE5); tick(620);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !supplement_budget_started);
  const uint8_t commands[]={0x21,0x11,0x31,0x51};
  for(unsigned i=0;i<4;++i) {
    setup(commands[i],i!=0); MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK,100);
    MissionTask_EnterS4FinalCenter(120);
    uint32_t duration=i>=2?250:(i==1?500:350); int rpm=i>=2?78:60;
    tick(120+duration-1); assert(mission_snapshot.left_target_rpm==rpm);
    tick(120+duration); assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_LOWER_FRAME && frame_lift==0);
    uint32_t down=mission_snapshot.state_entry_tick;
    tick(down+300); assert(mission_snapshot.left_target_rpm==-100);
    tick(down+579); assert(mission_snapshot.state==MISSION_STATE_S4_FINAL_LOWER_FRAME);
    tick(down+580); assert(mission_snapshot.state==MISSION_STATE_S5_WAIT_SINGLE_GREEN);
    assert(tx_count==1 && tx_commands[0]==0x25 && !s6_corner_completed);
  }
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
  puts("PASS:6E long-forward coordinate/16 lock, fresh/stale resume and command/safety priority; final05 interrupts ordinary/capture/arrangement/recovery/corner states except direct supplementary03; stop/near/retained load/no ACK; repeated05, zero-ID reports+50ms06,500ms result hold,02/24 routes; priority/freshness/partial-frame fences; budget limits request05 ONCE/no AUTO06; same-attempt03/no budget reset; corner guard/8s count; servo/health/stop/UART safety");
}
