/* Real AE parser/mission handlers, phase PID,05 review and supplement budget. */
static void ae_tick(uint32_t now)
{
  fake_now=now;MissionTask_UpdateVisionInput(now);MissionTask_CheckRunningHealth(now);
  (void)MissionTask_HandleLoadCheckRequest(now);MissionTask_HandleSupplementControl(now);
  MissionTask_HandleStartZoneAvoidanceRequest(now);MissionTask_RunStartZoneAvoidance(now);
  if(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN ||
      mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK)
    MissionTask_HandleRedPriorityRequest(now);
  MissionTask_UpdateVisionInput(fake_now);
}

static void ae_setup(MissionState phase,uint8_t cmd,bool supplement,int32_t yaw)
{
  setup(cmd,cmd!=0x21);MaixCam_SetLoadRecheckEnabled(false);
  MaixCam_SetSupplementWindowEnabled(supplement);
  inject_ae_coord_id=inject_ae_event=0;inject_ae_tx_delay=0;
  inject_ae_before_coordinate=false;
  ae_ack_completed=ae_reply_pending=false;
  ae_resume_prepared=false;
  s4_post22_search_active=false;
  s5_empty_recovery_active=false;
  vision_search_forward_active=vision_search_e3_mode=vision_search_green_e3_mode=false;
  vision_search_resume_state=cmd==0x11?MISSION_STATE_S7_SEARCH_RED:MISSION_STATE_S3_TRACK_GREEN;
  if(supplement) {
    supplement_capture_active=supplement_budget_started=true;
    supplement_first_request_tick=50;supplement_capture_count=1;
    supplement_reference_command=phase==MISSION_STATE_S4_TRACK_FINAL_BLOCK?0x24:
        (phase==MISSION_STATE_S4_TRACK_CENTER?0x04:0x03);
  }
  s51_e3_recovery_count=2;supplement_e3_recovery_count=1;
  s7_red_recovery_used=true;s4_green_e4_push_cycles=1;s4_material_e4_recovery_count=2;
  frame_lift=(phase==MISSION_STATE_S4_TRACK_FINAL_BLOCK || phase==MISSION_STATE_S4_RAISE_FRAME)?25:0;
  MissionTask_EnterState(phase,100);mission_snapshot.yaw_cdeg=yaw;
  fake_now=100;coords(cmd==0x11?4:5,20,50);MissionTask_UpdateVisionInput(100);
}

static uint32_t ae_to_ack(MissionState phase,uint8_t cmd,bool supplement,int32_t yaw)
{
  ae_setup(phase,cmd,supplement,yaw);
  // Simulate an in-flight old recovery and cached arrival decision. The AE
  // switch must cancel the work, without resetting any of its allowances.
  vision_search_forward_active=vision_search_e3_mode=vision_search_green_e3_mode=true;
  s51_second_e3_pending=s3_search_sweep_completed=s3_search_ccw_midpoint_done=true;
  s4_final_event34_pending=s4_final_mode_already_active=s4_e4_protocol_active=true;
  supplement_recovery_event=0xE3;
  vision_motion.x.initialized=vision_motion.y.initialized=s6_yaw_pid.initialized=true;
  vision_motion.x.integral=vision_motion.y.integral=s6_yaw_pid.integral=123.0f;
  fake_now=120;rx(0xAE);ae_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && maixcam_start_avoidance_active);
  assert(ae_resume_state==(supplement?MISSION_STATE_SUPPLEMENT_TRACK:MISSION_STATE_S3_TRACK_GREEN) && !tx_count);
  assert(mission_snapshot.left_target_rpm==-40 && mission_snapshot.right_target_rpm==40);
  assert(ae_turn_target_cdeg==MissionTask_WrapYaw(yaw+ROBOT_YAW_LEFT_SIGN*18000L));
  assert(!vision_search_forward_active && !vision_search_e3_mode && !vision_search_green_e3_mode);
  assert(!s51_second_e3_pending && !s3_search_sweep_completed && !s3_search_ccw_midpoint_done);
  assert(!s4_final_event34_pending && !s4_final_mode_already_active && !s4_e4_protocol_active);
  assert(!supplement_recovery_event && !vision_motion.x.initialized && !vision_motion.y.initialized && !s6_yaw_pid.initialized);
  assert(vision_motion.x.integral==0 && vision_motion.y.integral==0 && s6_yaw_pid.integral==0);
  const int32_t progress[]={6000,12000,16500,18000};
  for(unsigned i=0;i<4;++i) {
    fake_now=220+i*100;rx(0xAE);coords(cmd==0x11?4:5,0,0);
    rx(0x04);rx(0x14);rx(0x34);rx(0xE3);rx(0xE4);rx(0xEE);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(yaw+ROBOT_YAW_LEFT_SIGN*progress[i]);
    ae_tick(fake_now);
    assert(ae_turn_progress_cdeg==progress[i] && !mission_snapshot.vision_target_valid && !tx_count);
    assert(s51_e3_recovery_count==2 && supplement_e3_recovery_count==1);
    assert(s7_red_recovery_used && s4_green_e4_push_cycles==1 && s4_material_e4_recovery_count==2);
    if(i<3) {
      assert(mission_snapshot.state_entry_tick==120 && mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN);
      assert(mission_snapshot.left_target_rpm==-(i==2?20:40));
      assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
    }
  }

  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK && ae_reply_pending);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(!frame_lower_calls && !frame_raise_calls && !frame_partial_calls && !wide_calls && !track_view_calls && !near_calls);
  if(supplement) assert(supplement_first_request_tick==50 && supplement_capture_count==1);
  return 540;
}

static void verify_start_zone_avoidance(void)
{
  assert(MAIXCAM_EVENT_START_ZONE_AVOID==0xAE && MAIXCAM_COMMAND_START_ZONE_AVOID_DONE==0xAE);
  const MissionState phases[]={MISSION_STATE_S3_TRACK_GREEN,MISSION_STATE_S4_TRACK_CENTER,
    MISSION_STATE_S4_TRACK_FINAL_BLOCK,MISSION_STATE_SUPPLEMENT_TRACK,
    MISSION_STATE_S7_SEARCH_RED,MISSION_STATE_S7_WAIT_SECOND_E3,MISSION_STATE_S4_RAISE_FRAME,
    MISSION_STATE_S3_SEARCH_TURN_CW,MISSION_STATE_S3_SEARCH_TURN_CCW,MISSION_STATE_S3_E3_TURN_AWAY,
    MISSION_STATE_S4_FINAL_RECOVERY_REVERSE,MISSION_STATE_S4_FINAL_RECOVERY_TURN_270,
    MISSION_STATE_S4_FINAL_RECOVERY_TURN_90,MISSION_STATE_S4_E4_TURN_AWAY,
    MISSION_STATE_S4_E4_FORWARD,MISSION_STATE_S4_E4_TURN_LEFT,MISSION_STATE_S4_E4_TURN_RIGHT,
    MISSION_STATE_S4_E4_PUSH_RIGHT,MISSION_STATE_S4_E4_REVERSE_RIGHT,
    MISSION_STATE_S4_E4_PUSH_LEFT,MISSION_STATE_S4_E4_REVERSE_LEFT,
    MISSION_STATE_S4_E4_GREEN_SPIN_360,MISSION_STATE_S4_E4_RED_SPIN_360,
    MISSION_STATE_S4_E4_MATERIAL_SPIN_CW_360,MISSION_STATE_S7_SEARCH_BLACK,
    MISSION_STATE_S7_SILENT_TURN_LEFT,MISSION_STATE_S7_SILENT_TURN_BACK,
    MISSION_STATE_SUPPLEMENT_TURN_LEFT,MISSION_STATE_SUPPLEMENT_TURN_RIGHT,
    MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW};
  const int32_t yaws[]={0,35000,17900};
  uint8_t event;
  for(unsigned p=0;p<sizeof(phases)/sizeof(phases[0]);++p) for(unsigned y=0;y<3;++y) {
    bool supplement=phases[p]==MISSION_STATE_SUPPLEMENT_TRACK ||
        phases[p]==MISSION_STATE_SUPPLEMENT_TURN_LEFT || phases[p]==MISSION_STATE_SUPPLEMENT_TURN_RIGHT ||
        phases[p]==MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW;
    uint8_t cmd=(phases[p]==MISSION_STATE_S7_SEARCH_RED || phases[p]==MISSION_STATE_S7_WAIT_SECOND_E3)?0x11:(supplement?0x51:0x21);
    uint32_t ack=ae_to_ack(phases[p],cmd,supplement,yaws[y]);
    ae_tick(ack);
    assert(tx_count==1 && tx_commands[0]==0xAE && mission_snapshot.state==ae_resume_state);
    assert(!maixcam_start_avoidance_active && !mission_snapshot.vision_target_valid && ae_ack_completed);
    assert(mission_snapshot.state_entry_tick==fake_now && frame_lift==0);
    assert(frame_lower_calls==1 && track_view_calls==1 && !wide_calls);
    if(supplement) assert(supplement_reference_command==0x03 && supplement_first_request_tick==50 &&
        supplement_capture_count==1 && s6_carried_object_id==6 && s5_load_class_confirmed);
    assert(!mission_snapshot.left_target_rpm && !MaixCam_PeekEvent(&event));
    // Every former04/24/recovery resumes03's image-center PID, never old ROI.
    fake_now=ack+20;coords(cmd==0x11?4:5,0,50);ae_tick(fake_now);
    assert(mission_snapshot.vision_target_valid);
    assert(ae_fresh_coordinate_seen);
    if(supplement) MissionTask_RunSupplement(fake_now);
    else MissionTask_RunS3Track(fake_now);
    assert(mission_snapshot.left_target_rpm>0 && mission_snapshot.right_target_rpm>0);
  }

  // An active supplement capture can be in ordinary04/24 recovery too.
  // It must resume SUPPLEMENT_TRACK with03 reference, not an old ROI24.
  const MissionState supplement_phases[]={MISSION_STATE_S4_TRACK_CENTER,MISSION_STATE_S4_TRACK_FINAL_BLOCK,
      MISSION_STATE_S3_SEARCH_TURN_CW,MISSION_STATE_S3_SEARCH_TURN_CCW,
      MISSION_STATE_S4_FINAL_RECOVERY_REVERSE};
  for(unsigned p=0;p<sizeof(supplement_phases)/sizeof(supplement_phases[0]);++p) {
    uint32_t ack=ae_to_ack(supplement_phases[p],0x51,true,35000);
    supplement_e3_first_ack_sent=true;
    ae_tick(ack);
    assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_TRACK && supplement_reference_command==0x03);
    assert(supplement_first_request_tick==50 && supplement_capture_count==1 && supplement_e3_recovery_count==1);
    assert(supplement_e3_first_ack_sent && !supplement_recovery_event);
    MissionTask_RunSupplement(ack);MissionTask_RunSupplement(ack+20);
    assert(tx_count==1 && tx_commands[0]==0xAE); // canceled recovery never emits13
  }

  // E3-like forward/reverse legs share a state; either sign is canceled.
  for(unsigned backward=0;backward<2;++backward) {
    ae_setup(MISSION_STATE_S3_SEARCH_TURN_CW,0x51,false,35000);
    vision_search_forward_active=vision_search_e3_mode=true;
    vision_search_forward_rpm=backward?-80:80;vision_search_forward_ms=800;
    MissionTask_SetWheelTargets(vision_search_forward_rpm,vision_search_forward_rpm);
    rx(0xE3);rx(0xAE);ae_tick(120);
    assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && !vision_search_forward_active);
    assert(mission_snapshot.left_target_rpm==-40 && mission_snapshot.right_target_rpm==40 && !tx_count);
  }

  // Post22 is interruptible only once it actually uses03 search semantics.
  const MissionState post22[]={MISSION_STATE_S4_POST22_REVERSE,MISSION_STATE_S4_POST22_TURN_LEFT,
      MISSION_STATE_S4_POST22_TURN_RIGHT};
  for(unsigned p=0;p<3;++p) for(unsigned searching=0;searching<2;++searching) {
    ae_setup(post22[p],0x51,false,35000);s4_post22_search_active=searching;
    rx(0xAE);ae_tick(120);
    assert(mission_snapshot.state==(searching?MISSION_STATE_VISION_AVOID_START_TURN:post22[p]));
    if(searching) assert(!s4_post22_search_active);
  }

  // Valid postACK coordinate/event bursts survive blocking TX and tick wrap.
  const uint32_t delays[]={0,1,20,80};
  const uint8_t replies[]={0x04,0x14,0x34};
  for(unsigned d=0;d<4;++d) for(unsigned r=0;r<3;++r) {
    uint32_t ack=ae_to_ack(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
    inject_ae_coord_id=5;inject_ae_event=replies[r];inject_ae_tx_delay=delays[d];ae_tick(ack);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && mission_snapshot.vision_target_valid);
    assert(mission_snapshot.target_age_ms==0 && mission_snapshot.state_entry_tick==fake_now);
    assert(MaixCam_PeekEvent(&event) && event==replies[r]);
  }
  (void)ae_to_ack(MISSION_STATE_S4_TRACK_CENTER,0x21,false,35000);
  inject_ae_event=0x05;ae_tick(540);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm);
  assert(maixcam_load_recheck_enabled && !maixcam_start_avoidance_active);

  // A retry arriving DURING TX, BEFORE the first good coordinate, stays a
  // retry even when the fresh coordinate is processed before its mailbox.
  uint32_t race_ack=ae_to_ack(MISSION_STATE_S3_SEARCH_TURN_CCW,0x21,false,35000);
  inject_ae_before_coordinate=true;inject_ae_coord_id=5;ae_tick(race_ack);
  assert(mission_snapshot.vision_target_valid && ae_fresh_coordinate_seen && maixcam_start_avoidance_pending);
  ae_tick(race_ack+20);
  assert(tx_count==2 && tx_commands[1]==0xAE && ae_turn_progress_cdeg==18000);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);

  // Conversely, a new AE following a valid postACK coordinate is a real
  // next encounter, even if the entire burst arrives before TX returns.
  race_ack=ae_to_ack(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
  inject_ae_coord_id=5;inject_ae_event=0xAE;ae_tick(race_ack);ae_tick(race_ack+20);
  assert(tx_count==1 && mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && ae_turn_progress_cdeg==0);

  race_ack=ae_to_ack(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
  byte(0xF1);byte(0xF2);byte(0xAE);ae_tick(race_ack); // half duplicate from the turn
  byte(0x1F);byte(0x2F);fake_now=race_ack+20;coords(5,20,50);ae_tick(fake_now);
  assert(tx_count==1 && !maixcam_start_avoidance_pending && mission_snapshot.vision_target_valid);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);

  // A successful ACK lost at the camera must remain deduplicated even if
  // our normal no-coordinate timeout has already started a new E3 search.
  race_ack=ae_to_ack(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);ae_tick(race_ack);
  MissionTask_EnterState(MISSION_STATE_S3_SEARCH_TURN_CCW,race_ack+2200);
  rx(0xAE);ae_tick(race_ack+2220);
  assert(tx_count==2 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && ae_turn_progress_cdeg==18000);

  // Failed ACK retries and lost successful ACKs must not repeat180 degrees.
  uint32_t ack=ae_to_ack(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x21,false,35000);
  tx_ok=false;ae_tick(ack);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK && maixcam_start_avoidance_active);
  assert(!mission_snapshot.fault_flags && !mission_snapshot.left_target_rpm);
  assert(ae_reply_pending && frame_lower_calls==1 && track_view_calls==1 && !wide_calls);
  fake_now=ack+20;rx(0xAE);ae_tick(fake_now);assert(tx_count==1 && ae_next_reply_tick==ack+100);
  ae_tick(ack+99);assert(tx_count==1 && ae_reply_pending);
  ae_tick(ack+100);assert(tx_count==2 && ae_reply_pending);
  assert(frame_lower_calls==1 && track_view_calls==1 && !wide_calls); // no repeated servo setup on failed TX
  tx_ok=true;ae_tick(ack+200); // no new RXAE: autonomous retry
  assert(tx_count==3 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && ae_ack_completed && !ae_reply_pending);
  fake_now=ack+220;rx(0xAE);ae_tick(fake_now);
  assert(tx_count==4 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && ae_turn_progress_cdeg==18000);
  // Wrong-class data is not evidence that vision received ACK.
  fake_now=ack+240;coords(6,20,50);ae_tick(fake_now);assert(ae_ack_completed);
  tx_ok=false;fake_now=ack+260;rx(0xAE);ae_tick(fake_now);
  assert(tx_count==5 && mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK && ae_reply_pending);
  tx_ok=true;ae_tick(ack+360);
  assert(tx_count==6 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && ae_turn_progress_cdeg==18000);
  fake_now=ack+380;coords(5,20,50);ae_tick(fake_now);assert(ae_fresh_coordinate_seen);
  fake_now=ack+400;rx(0xAE);ae_tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && ae_turn_progress_cdeg==0 && tx_count==6);

  // AE is independent of later loss/arrival notifications, which cannot
  // hijack the turn. A valid11 remains pending, including partial frames.
  ae_setup(MISSION_STATE_S4_TRACK_CENTER,0x51,false,35000);
  fake_now=120;rx(0xAE);rx(0xE4);ae_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN);
  fake_now=140;rx(0x11);ae_tick(140);assert(maixcam_red_priority_pending);
  mission_snapshot.yaw_cdeg=5000;ae_tick(150);
  mission_snapshot.yaw_cdeg=17000;ae_tick(160);
  byte(0xF1);byte(0xF2);byte(0x11);ae_tick(180);byte(0x1F);byte(0x2F);
  assert(maixcam_red_priority_pending && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);

  // A half old coordinate/34 from the maneuver cannot become postACK data.
  for(unsigned arrival=0;arrival<2;++arrival) {
    ack=ae_to_ack(MISSION_STATE_S4_TRACK_FINAL_BLOCK,0x21,false,35000);
    byte(0xF1);byte(0xF2);byte(arrival?0x34:5);
    if(arrival) byte(0x1F);
    ae_tick(ack);
    if(arrival) byte(0x2F);
    else { const uint8_t tail[]={0,0,1,0,50,2,0x1F,0x2F};for(unsigned i=0;i<sizeof(tail);++i) byte(tail[i]); }
    MissionTask_UpdateVisionInput(ack+20);
    assert(!mission_snapshot.vision_target_valid && !MaixCam_PeekEvent(&event));
  }

  //05 with a partial ID report interrupts the turn without losing cargo.
  ae_setup(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
  fake_now=120;rx(0xAE);ae_tick(120);
  fake_now=140;rx(0x05);byte(0xF1);byte(0xF2);byte(5);byte(0);ae_tick(140);
  const uint8_t cargo_tail[]={0,1,0,0,1,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(cargo_tail);++i) byte(cargo_tail[i]);
  rx(0x06);
  assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !maixcam_start_avoidance_active);
  assert(!tx_count && MaixCam_LoadRecheckResultPending());

  // A failed reply cannot survive05 review/stop/fault and leak into a later
  // task. Budget expiry also cancels an outstanding AE retransmission.
  ack=ae_to_ack(MISSION_STATE_S3_SEARCH_TURN_CCW,0x51,true,35000);
  tx_ok=false;ae_tick(ack);assert(ae_reply_pending);
  tx_ok=true;ae_tick(15050);
  assert(!ae_reply_pending && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
  assert(tx_count==2 && tx_commands[1]==0x05);

  // Bare supplement06 still cannot bypass the existing final05+ID review.
  ae_setup(MISSION_STATE_SUPPLEMENT_TRACK,0x51,true,35000);
  fake_now=120;rx(0xAE);ae_tick(120);
  fake_now=140;coords(5,0,0);rx(0x06);ae_tick(140);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && s6_carried_object_id==6);
  assert(maixcam_start_avoidance_active && !tx_count && frame_lift==0);

  // Clockwise drift cannot be mistaken for the requested CCW half-turn.
  ae_setup(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
  fake_now=120;rx(0xAE);ae_tick(120);
  mission_snapshot.yaw_cdeg=30000;ae_tick(140);
  assert(ae_turn_progress_cdeg==0 && mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN);
  assert(mission_snapshot.left_target_rpm==-40 && mission_snapshot.right_target_rpm==40);

  // The supplement deadline is never restarted by the AE maneuver.
  ae_setup(MISSION_STATE_SUPPLEMENT_TRACK,0x51,true,35000);
  uint32_t deadline=supplement_first_request_tick+ROBOT_SUPPLEMENT_TOTAL_MS;
  fake_now=deadline-150U;mission_snapshot.state_entry_tick=fake_now-100U;rx(0xAE);ae_tick(fake_now);
  ae_tick(deadline);
  assert(supplement_budget_exhausted && mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK);
  assert(tx_count==1 && tx_commands[0]==0x05 && !maixcam_start_avoidance_active);

  // Keep the existing2s soft-turn deadline, with no duplicate restart/fault.
  ae_setup(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
  fake_now=120;rx(0xAE);ae_tick(120);
  fake_now=2119;rx(0xAE);ae_tick(2119);
  assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_TURN && mission_snapshot.state_entry_tick==120);
  ae_tick(2120);assert(mission_snapshot.state==MISSION_STATE_VISION_AVOID_START_WAIT_ACK && !tx_count);
  ae_tick(2140);assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==1 && !mission_snapshot.fault_flags);

  ae_setup(MISSION_STATE_S4_TRACK_CENTER,0x21,false,35000);
  uint32_t start=UINT32_MAX-300U;mission_snapshot.state_entry_tick=start-20U;
  fake_now=start;rx(0xAE);ae_tick(start);
  mission_snapshot.yaw_cdeg=5000;ae_tick(start+100U);
  mission_snapshot.yaw_cdeg=11000;ae_tick(start+200U);
  mission_snapshot.yaw_cdeg=17000;ae_tick(start+400U);
  inject_ae_coord_id=5;inject_ae_tx_delay=20;ae_tick(start+420U);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && mission_snapshot.vision_target_valid && tx_count==1);
  assert(mission_snapshot.state_entry_tick==fake_now && mission_snapshot.target_age_ms==0);

  // Failed TX retry deadlines also cross the uint32 millisecond wrap.
  ack=ae_to_ack(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);
  uint32_t wrap_ack=UINT32_MAX-50U;
  ae_next_reply_tick=wrap_ack;tx_ok=false;ae_tick(wrap_ack);
  assert(ae_reply_pending && tx_count==1);
  ae_tick(wrap_ack+99U);assert(tx_count==1);
  tx_ok=true;ae_tick(wrap_ack+100U);
  assert(tx_count==2 && !ae_reply_pending && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);

  // Wrong classes still cannot drive after AE (first green, red, supplement).
  const uint8_t commands[]={0x21,0x11,0x51},bad_ids[]={6,5,4};
  for(unsigned i=0;i<3;++i) {
    ack=ae_to_ack(i==2?MISSION_STATE_SUPPLEMENT_TRACK:MISSION_STATE_S3_TRACK_GREEN,commands[i],i==2,35000);
    ae_tick(ack);fake_now=ack+20;coords(bad_ids[i],0,50);ae_tick(fake_now);
    assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm);
  }

  const MissionState e5_scatter[]={MISSION_STATE_S4_E4_PUSH_RIGHT,MISSION_STATE_S4_E4_REVERSE_RIGHT,
      MISSION_STATE_S4_E4_PUSH_LEFT,MISSION_STATE_S4_E4_REVERSE_LEFT};
  for(unsigned i=0;i<4;++i) {
    ae_setup(e5_scatter[i],0x21,false,35000);s5_empty_recovery_active=true;
    rx(0xAE);MissionTask_HandleStartZoneAvoidanceRequest(120);
    assert(mission_snapshot.state==e5_scatter[i] && !tx_count);
  }
  const MissionState inactive[]={MISSION_STATE_S4_WAIT_ARRANGE_READY,
    MISSION_STATE_S4_FINE42_TURN,MISSION_STATE_S5_WAIT_SINGLE_GREEN,MISSION_STATE_S6_TRACK_SAFE_ZONE,
    MISSION_STATE_S6_EXIT_TURN_180,MISSION_STATE_FAULT,MISSION_STATE_STOPPED,
    MISSION_STATE_S2_CROSS_BUMP,MISSION_STATE_S4_FINAL_CENTER_OBJECT,MISSION_STATE_S6_BORDER_LOCATE};
  for(unsigned i=0;i<sizeof(inactive)/sizeof(inactive[0]);++i) {
    ae_setup(inactive[i],0x21,false,35000);fake_now=120;rx(0xAE);
    MissionTask_HandleStartZoneAvoidanceRequest(120);
    assert(mission_snapshot.state==inactive[i] && !tx_count && !maixcam_start_avoidance_active);
  }
  const MissionState active[]={MISSION_STATE_VISION_AVOID_START_TURN,MISSION_STATE_VISION_AVOID_START_WAIT_ACK};
  for(unsigned s=0;s<2;++s) for(unsigned f=0;f<3;++f) {
    ae_setup(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);MissionTask_EnterState(active[s],120);
    health_faults=f==0?MISSION_FAULT_IMU:(f==1?MISSION_FAULT_LEFT_MOTOR:MISSION_FAULT_RIGHT_MOTOR);
    ae_tick(140);assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm && !tx_count);
    assert(!maixcam_start_avoidance_active);
  }
  ae_setup(MISSION_STATE_S3_TRACK_GREEN,0x21,false,35000);fake_now=120;rx(0xAE);ae_tick(120);
  MissionTask_EnterState(MISSION_STATE_STOPPED,140);ae_tick(160);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !mission_snapshot.left_target_rpm && !tx_count);
  puts("PASS:AE interrupts03/04/24 and their E3/E4/EE recoveries,always resumes image-center03 with retained cargo/budget/counts;gyro CCW180 across zero,40/20rpm,duplicates merge,ACK auto-retry and failed REACK retry,no old13/ROI,pre-TX fence keeps immediate new XY/events/05/11,2s deadline/wrap and safety");
}
