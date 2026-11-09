/* Real UART parser, E4 recovery handlers,04/14 TX helpers and arrange wait. */
static void setup_green_push(void)
{
  setup(0x21);
  s7_target_switch_enabled = false;
  MissionTask_StartS4E4Recovery(120);
  assert(mission_snapshot.state == MISSION_STATE_S4_E4_PUSH_RIGHT && s4_green_e4_half_turn_active);
}

static void setup_green_empty_scatter(void)
{
  setup(0x21);s7_target_switch_enabled=false;s5_empty_recovery_active=true;
  s4_e4_protocol_active=true;MissionTask_StartS4GreenScatter(120);
  assert(!s4_green_e4_half_turn_active);
}

static uint32_t finish_green_push(void)
{
  if(s4_green_e4_half_turn_active) {
    uint32_t start=mission_snapshot.state_entry_tick;
    int32_t origin=mission_snapshot.yaw_cdeg;
    unsigned consumed=s4_green_e4_push_cycles;
    for(unsigned i=1;i<=6;++i) {
      fake_now=start+i*20U;coords(5);rx(0xE4);
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(origin+ROBOT_YAW_LEFT_SIGN*(int32_t)i*3000L);
      tick(fake_now);
      assert(s4_e4_spin_progress_cdeg==(int32_t)i*3000L && s4_green_e4_push_cycles==consumed);
      if(i<6) {
        assert(mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT && !tx_count && !mission_snapshot.vision_target_valid);
        assert(mission_snapshot.left_target_rpm<0 && mission_snapshot.right_target_rpm>0);
      }
    }
    assert(!s4_green_e4_half_turn_active && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
    assert(tx_count==1 && tx_commands[0]==0x44 && !maixcam_e4_stage_buffering);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    return fake_now;
  }
  const MissionState states[] = {MISSION_STATE_S4_E4_PUSH_RIGHT,
    MISSION_STATE_S4_E4_REVERSE_RIGHT, MISSION_STATE_S4_E4_PUSH_LEFT,
    MISSION_STATE_S4_E4_REVERSE_LEFT};
  const uint32_t durations[] = {ROBOT_HEADING_TURN_TIMEOUT_MS,
    ROBOT_S5_E5_FORWARD_MS, ROBOT_HEADING_TURN_TIMEOUT_MS,
    ROBOT_HEADING_TURN_TIMEOUT_MS};
  unsigned consumed=s4_green_e4_push_cycles;
  uint32_t t = mission_snapshot.state_entry_tick;
  for (unsigned i = 0; i < 4; ++i)
  {
    assert(mission_snapshot.state == states[i]);
    fake_now = t + 20;
    coords(5); rx(0xE4);
    tick(t + 20);
    assert(mission_snapshot.state == states[i] && tx_count == 0);
    if(i==1) assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
    else {
      assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm);
      assert(i==2 ? mission_snapshot.left_target_rpm>0 : mission_snapshot.left_target_rpm<0);
      assert(selected_turn_limit==(i==0?35:60));
    }
    tick(t + durations[i] - 1);
    assert(mission_snapshot.state == states[i] && tx_count == 0);
    t += durations[i]; tick(t);
  }
  assert(mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN);
  assert(tx_count == 1 && tx_commands[0] == 0x44);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(s4_green_e4_push_cycles == consumed && !maixcam_e4_stage_buffering);
  return t;
}

static void verify_green_scatter_path(void)
{
  setup(0x21);s7_target_switch_enabled=false;mission_snapshot.yaw_cdeg=35900;
  MissionTask_StartS4E4Recovery(100);
  assert(frame_down && s4_green_e4_push_cycles==1 && s4_green_e4_half_turn_active);
  tick(120);
  assert(mission_snapshot.left_target_rpm==-20 && mission_snapshot.right_target_rpm==20);
  mission_snapshot.yaw_cdeg=8900;tick(140);
  fake_now=160;coords(5);rx(0xE4);tick(160);
  assert(s4_e4_spin_progress_cdeg==9000 && !mission_snapshot.vision_target_valid && !tx_count);
  assert(mission_snapshot.state_entry_tick==100 && mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT);
  tick(2100);assert(s4_green_e4_half_turn_active && !tx_count); // not a2s soft completion
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(35900+17000L);tick(2120);
  assert(mission_snapshot.left_target_rpm==-12 && mission_snapshot.right_target_rpm==12);
  mission_snapshot.yaw_cdeg=17899;tick(2140);
  assert(s4_e4_spin_progress_cdeg==17999 && !tx_count);
  mission_snapshot.yaw_cdeg=17900;tick(2160);
  assert(tx_count==1 && tx_commands[0]==0x44 && !mission_snapshot.left_target_rpm);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s4_green_e4_push_cycles==1);

  //44 resumes03. A new04 handshake precedes E4's second-stage CCW360.
  fake_now=2170;rx(0x04);MissionTask_RunS3Track(fake_now);
  assert(tx_count==2 && tx_commands[1]==0x04 && mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER);
  fake_now=2180;MissionTask_StartS4E4Recovery(fake_now);tick(2180);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_GREEN_SPIN_360);
  assert(mission_snapshot.left_target_rpm==-20 && mission_snapshot.right_target_rpm==20);
  uint32_t entry=mission_snapshot.state_entry_tick;
  for(unsigned i=1;i<=6;++i) {
    fake_now=2180+i*20;rx(0xE4);
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(17900+(int32_t)i*6000);
    tick(fake_now);
    if(i<6) assert(mission_snapshot.state==MISSION_STATE_S4_E4_GREEN_SPIN_360 &&
        mission_snapshot.state_entry_tick==entry && s4_e4_spin_progress_cdeg==(int32_t)i*6000);
  }
  assert(s4_e4_spin_progress_cdeg==36000 && s4_green_e4_spin_exhausted);
  assert(tx_count==3 && tx_commands[2]==0x04 && mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER);

  // The identical E5 motion does NOT consume the first E4 scatter allowance.
  setup(0x21);s7_target_switch_enabled=false;s5_empty_recovery_active=true;
  s4_e4_protocol_active=true;MissionTask_StartS4GreenScatter(120);
  fake_now=140;coords(5);rx(0x14);rx(0x24);
  uint32_t done=finish_green_push();uint8_t event;
  assert(!s4_green_e4_push_cycles && !MaixCam_TakeEvent(&event));
  assert(!mission_snapshot.target_present && mission_snapshot.target_age_ms==UINT32_MAX);
  fake_now=done+20;rx(0x04);MissionTask_RunS3Track(fake_now);
  MissionTask_StartS4E4Recovery(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT && s4_green_e4_push_cycles==1);
  // Failure to lower the frame is a real fault, not a successful44 handshake.
  setup(0x21);s7_target_switch_enabled=false;frame_lower_ok=false;
  MissionTask_StartS4E4Recovery(120);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count && !mission_snapshot.left_target_rpm);
  puts("PASS:unused first-green E4 forced gyro CCW180 at20/12,no XY/translation/early44;179.99/180 across0 and no2s shortcut;used E4 remains interruptible CCW360;E5 retains L10/F170450/R45/L20,no E4 quota,frame fault safety");
}

static void verify_44_resumes03(void)
{
  uint8_t event;
  MaixCam_Object object;
  uint32_t done;
  // Both44 producers use03/image-center, never initialize21/03 or clear quota.
  for(unsigned e5=0;e5<2;++e5) {
    if(e5) setup_green_empty_scatter(); else setup_green_push();
    s51_e3_recovery_count=2;s4_material_e4_recovery_count=2;
    s4_final_mode_already_active=s4_final_event34_pending=true;
    s3_search_sweep_completed=true;s3_track_stable_since=80;
    unsigned quota=s4_green_e4_push_cycles;
    inject44_coord=5;done=finish_green_push();
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && frame_down);
    assert(!s4_e4_protocol_active && !s4_final_mode_already_active && !s4_final_event34_pending);
    assert(s4_green_e4_push_cycles==quota && s51_e3_recovery_count==2 && s4_material_e4_recovery_count==2);
    assert(s3_target_select_command==0x21 && !s3_search_sweep_completed && !s3_track_stable_since);
    assert(s3_target_lost_since==done && s3_search_origin_yaw_cdeg==MissionTask_WrapYaw(mission_snapshot.yaw_cdeg));
    assert(tx_count==1 && tx_commands[0]==0x44);
    assert(MaixCam_GetObjectSnapshot(&object) && object.object_id==5);
    fake_now=done+20;MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
    assert(mission_snapshot.left_target_rpm>0 && mission_snapshot.right_target_rpm>0);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==1);
    // No obsolete5s arrange-wait fault: fresh03 image coordinates keep driving.
    fake_now=done+6000;coords(5);MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !mission_snapshot.fault_flags);
    fake_now+=501;MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
    assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm);
    coords(6);MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
    assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm && s3_target_select_command==0x21);
    coords(5);MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
    assert(mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm>0);
    rx(0x04);MissionTask_RunS3Track(fake_now);
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER && tx_count==2 && tx_commands[1]==0x04);
    assert(s4_green_e4_push_cycles==quota);
  }
  // An E3 immediately in TX44 uses the EXISTING green recovery, not wait02/24.
  setup_green_push();inject44_event=0xE3;done=finish_green_push();
  fake_now=done+20;MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
  assert(vision_search_green_e3_mode && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
  assert(s4_green_e4_push_cycles==1 && tx_count==1 && vision_search_resume_state==MISSION_STATE_S3_TRACK_GREEN);
  uint32_t entry=mission_snapshot.state_entry_tick;
  fake_now+=20;rx(0xE3);MissionTask_DropDuplicateRecoveryEvent(fake_now);
  assert(mission_snapshot.state_entry_tick==entry && !MaixCam_TakeEvent(&event));
  fake_now=entry+799;MissionTask_RunS3GreenE3Recovery(fake_now);
  assert(mission_snapshot.left_target_rpm==80 && tx_count==1);
  fake_now=entry+800;MissionTask_RunS3GreenE3Recovery(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
  int32_t origin=mission_snapshot.yaw_cdeg;
  for(unsigned i=1;i<=6;++i) {
    fake_now+=20;mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(origin+ROBOT_YAW_LEFT_SIGN*(int32_t)i*6000L);
    MissionTask_RunS3GreenE3Recovery(fake_now);
  }
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==1 && s4_green_e4_push_cycles==1);
  rx(0x04);MissionTask_RunS3Track(fake_now);
  assert(tx_count==2 && tx_commands[1]==0x04 && mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER);

  // Partial OLD coordinate/04/14/24 frames cannot cross44 as new inputs.
  const uint8_t old_events[]={0x04,0x14,0x24,0xE3,0xE4};
  for(unsigned i=0;i<sizeof(old_events);++i) {
    setup_green_push();
    byte(0xF1);byte(0xF2);byte(old_events[i]);byte(0x1F);
    inject44_partial_tail=true;
    MissionTask_FinishS4GreenRecovery(140,false);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && tx_count==1);
    assert(!MaixCam_TakeEvent(&event));
    rx(0x04);assert(MaixCam_TakeEvent(&event) && event==0x04);
  }
  setup_green_push();
  const uint8_t partial_xy[]={0xF1,0xF2,5,0,20,1,0,30,2,0x1F};
  for(unsigned i=0;i<sizeof(partial_xy);++i) byte(partial_xy[i]);
  inject44_partial_tail=true;MissionTask_FinishS4GreenRecovery(140,false);
  assert(!MaixCam_GetObjectSnapshot(&object));
  coords(5);assert(MaixCam_GetObjectSnapshot(&object));
  // This is a target-reference fence, not a blanket mailbox/parser reset.
  setup_green_push();rx(0x11);rx(0xAE);rx(0x14);rx(0x24);
  (void)MaixCam_FenceSearchReference();uint32_t sequence;
  assert(MaixCam_TakeRedPriorityRequest() && MaixCam_TakeStartZoneAvoidanceRequest(&sequence));
  assert(!MaixCam_TakeEvent(&event));
  const uint8_t keep_events[]={0x05,0x06,0xAE,0x11};
  for(unsigned i=0;i<sizeof(keep_events);++i) {
    setup_green_push();byte(0xF1);byte(0xF2);byte(keep_events[i]);byte(0x1F);
    (void)MaixCam_FenceSearchReference();byte(0x2F);
    if(keep_events[i]==0x05) assert(MaixCam_TakeLoadRecheckAck());
    else if(keep_events[i]==0xAE) assert(MaixCam_TakeStartZoneAvoidanceRequest(&sequence));
    else if(keep_events[i]==0x11) assert(MaixCam_TakeRedPriorityRequest());
    else assert(MaixCam_TakeEvent(&event) && event==0x06);
  }
  // Tick wrap, stalled gyro safety and TX failure never yield false success.
  setup_green_push();mission_snapshot.state_entry_tick=UINT32_MAX-50U;
  (void)finish_green_push();assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  setup_green_push();tick(120+ROBOT_S4_E4_GREEN_SPIN_TIMEOUT_MS);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count && !mission_snapshot.left_target_rpm);
  setup_green_push();tx_ok=false;mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+9000);
  tick(140);mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+9000);tick(160);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && tx_count==1 && !mission_snapshot.left_target_rpm);
  assert(mission_snapshot.fault_flags & MISSION_FAULT_VISION_TX);
  puts("PASS:44->03 image-center XY,ID5/freshness,normal E3 then04 ACK,no old5s wait,no21/03 reinit,no budget reset;old full/half stages fenced BEFORE44,new TX replies and05/06/AE/11 retained;tick-wrap,gyro/TX faults");
}

static void verify_e4_event_buffering(void)
{
  verify_green_scatter_path();
  verify_44_resumes03();
  uint8_t event;
  uint32_t done;
  MaixCam_Object object;

  // All three phase messages, each once and in order, survive burst duplicates.
  setup_green_push();
  rx(0x14); rx(0xE4); rx(0x02); rx(0x14); rx(0x24); rx(0x02); rx(0xE4);
  assert(maixcam_e4_stage_count == 3);
  assert(MaixCam_PeekEvent(&event) && event == 0x14);
  assert(!MaixCam_DropEventIf(0xE4));
  assert(MaixCam_TakeEvent(&event) && event == 0x14);
  assert(MaixCam_TakeEvent(&event) && event == 0x02);
  assert(MaixCam_TakeEvent(&event) && event == 0x24);
  assert(!MaixCam_TakeEvent(&event));

  setup_green_push(); rx(0x14); rx(0x02); rx(0x24);
  assert(MaixCam_DropEventIf(0x02)); // Selective removal must retain the others.
  assert(MaixCam_TakeEvent(&event) && event == 0x14);
  assert(MaixCam_TakeEvent(&event) && event == 0x24);
  assert(!MaixCam_TakeEvent(&event));

  // Old04/14/02/24 cannot interrupt180 or route the NEW03 reference backward.
  setup_green_push(); rx(0x14); rx(0xE4); rx(0x02);
  done = finish_green_push();
  assert(!acknowledgements && !MaixCam_PeekEvent(&event));
  tick(done + 20);
  assert(tx_count == 1 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);

  // A pre44 old24 at any quarter of180 cannot start capture in the new epoch.
  for (unsigned leg = 0; leg < 4; ++leg)
  {
    setup_green_push();
    uint32_t t = 120;
    int32_t origin=mission_snapshot.yaw_cdeg;
    for (unsigned i = 0; i < 4; ++i)
    {
      if (i == leg) { fake_now = t + 1; rx(0x24); rx(0xE4); }
      tick(t+19);assert(tx_count==0);
      t+=20;mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(origin+(int32_t)(i+1)*4500L);
      tick(t);
    }
    assert(tx_count == 1 && tx_commands[0] == 0x44);
    tick(t + 20);
    assert(mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN && !MaixCam_PeekEvent(&event));
  }

  // New03 XY/04 received inside synchronous44 TX survive: no post-TX clear.
  setup_green_push(); inject44_event=0x04;inject44_coord=5;
  done = finish_green_push();
  assert(MaixCam_PeekEvent(&event) && event==0x04);
  assert(MaixCam_GetObjectSnapshot(&object) && object.object_id==5);
  fake_now=done+20;MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
  assert(tx_count==2 && tx_commands[1]==0x04 && mission_snapshot.state==MISSION_STATE_S4_TRACK_CENTER);

  // Keep the explicit E5 old-input fence, but preserve the post44 reply.
  setup_green_empty_scatter();
  rx(0x14); rx(0x24); done = finish_green_push();
  assert(!MaixCam_TakeEvent(&event) && !acknowledgements);
  assert(!MaixCam_GetObjectSnapshot(&object));
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  fake_now = done + 20; rx(0x04);MissionTask_RunS3Track(fake_now);
  assert(mission_snapshot.state == MISSION_STATE_S4_TRACK_CENTER && tx_commands[1]==0x04);
  setup_green_empty_scatter();
  rx(0x14); inject44_event=0xE3;
  done = finish_green_push();
  fake_now=done+20;MissionTask_UpdateVisionInput(fake_now);MissionTask_RunS3Track(fake_now);
  assert(!acknowledgements && mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW && vision_search_green_e3_mode);

  // Interruptible after07 recovery returns with04 BEFORE acknowledging14.
  setup(0x51); MissionTask_StartS4E4Recovery(120);
  fake_now = 140; coords(6); rx(0x14); rx(0x24); rx(0xE4); tick(140);
  assert(tx_count == 2 && tx_commands[0] == 0x04 && tx_commands[1] == 0x14);
  assert(s4_material_e4_recovery_count == 1);
  tick(160); assert(mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME);

  // A quick14/24 burst inside return04 TX cannot turn into only24.
  setup(0x51); MissionTask_StartS4E4Recovery(120); inject_stage_on_tx = 0x04;
  fake_now = 140; coords(6); tick(140);
  assert(tx_count == 2 && tx_commands[0] == 0x04 && tx_commands[1] == 0x14);
  tick(160); assert(mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME);

  // CW360 no longer consumes and ignores02/24 while searching for14.
  setup(0x51); s4_material_e4_recovery_count = 1;
  MissionTask_StartS4E4Recovery(120); fake_now = 140; rx(0x02); rx(0xE4); tick(140);
  assert(tx_count == 1 && tx_commands[0] == 0x04);
  tick(160); assert(mission_snapshot.state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK);

  // Preserve a valid stage received immediately before entering the E4 spin.
  setup(0x51); s4_material_e4_recovery_count = 1; rx(0x14);
  MissionTask_StartS4E4Recovery(120); tick(140);
  assert(tx_count == 2 && tx_commands[0] == 0x04 && tx_commands[1] == 0x14);

  //05 is still an explicit new counting window, not a deferred motion phase.
  setup_green_push(); rx(0x14); rx(0x24); rx(0x05); rx(0x24);
  assert(MaixCam_TakeLoadRecheckAck() && maixcam_e4_stage_count == 0);
  assert(MaixCam_TakeEvent(&event) && event == 0x24 && !MaixCam_TakeEvent(&event));

  // Safety and TX errors must not fall through into arrival acknowledgement.
  setup_green_push(); rx(0x14); health_faults = MISSION_FAULT_IMU; tick(140);
  assert(mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 0);
  assert(!mission_snapshot.left_target_rpm);
  setup(0x51); MissionTask_StartS4E4Recovery(120); rx(0x14); tx_ok = false; tick(140);
  assert(mission_snapshot.state == MISSION_STATE_FAULT && tx_count == 1);
  assert(!acknowledgements && !maixcam_e4_stage_buffering);
  puts("PASS:E4 ordered stage buffering during actions,forced green180 non-interruption;44 resumes03 and fences OLD stages,TX44 new XY/E3/04 survive;after07 E4 remains04-before14,E5/05 and safety");
}
