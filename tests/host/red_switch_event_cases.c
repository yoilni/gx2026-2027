/* Real black->red switch entry,11/03 sender and continuous UART parser. */
static void setup_red_switch(uint8_t tx_cmd, uint8_t event, uint8_t frame_kind)
{
  setup(3, ROBOT_TEAM_BLUE, 0);
  s51_active_object_id = 6;
  s7_red_recovery_used = false;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK, 110);
  fake_now = 110;
  coords(6); rx(0xE3); rx(0x11); // Old phase inputs must be fenced BEFORE TX.
  inject_red_on_tx = tx_cmd;
  inject_red_event = event;
  inject_red_frame_kind = frame_kind;
  if (frame_kind == 4)
  {
    byte(0xF1); byte(0xF2); byte(6); byte(0); byte(20); byte(1); byte(0);
  }
  MissionTask_HandleRedPriorityRequest(120);
  assert(red_switches == 1 && s3_target_select_command == 0x11);
  assert(tx_count == 4 && tx_commands[2] == 0x11 && tx_commands[3] == 0x03);
  assert(s7_red_coordinate_fence_active && !s7_red_switch_sync_pending);
  assert(mission_snapshot.state == MISSION_STATE_S3_TRACK_GREEN);
  assert(!mission_snapshot.vision_target_valid);
}

static void verify_material_e3_priority(void)
{
  const uint8_t locks[]={0,5,6};
  for(unsigned phase=0;phase<6;++phase) for(unsigned lock=0;lock<sizeof(locks);++lock) {
    setup(3,ROBOT_TEAM_BLUE,0);
    s51_active_object_id=locks[lock];
    fake_now=120;
    if(phase==5) {
      s4_post22_search_active=true;
      MissionTask_EnterState(MISSION_STATE_S4_POST22_TURN_RIGHT,110);
    }
    if(phase==2) s51_e3_recovery_count=1;
    rx(0xE3);
    if(phase>=1 && phase<=4) {
      tick(120);
      assert(mission_snapshot.state==(phase==2?MISSION_STATE_S3_E3_TURN_AWAY:MISSION_STATE_S3_SEARCH_TURN_CW));
      if(phase==1) { tick(140);assert(mission_snapshot.left_target_rpm==80); }
      if(phase>=3) {
        tick(120+ROBOT_S51_E3_FORWARD_MS);
        assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CCW);
        if(phase==4) {
          e3_spin_progress_cdeg=36000;
          tick(fake_now+20);
          assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && s3_search_sweep_completed);
        }
      }
    }
    unsigned sent=tx_count;
    inject_red_on_tx=0x03;inject_red_frame_kind=5;
    rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now);
    assert(s3_target_select_command==0x11 && tx_count==sent+2);
    assert(tx_commands[sent]==0x11 && tx_commands[sent+1]==0x03);
    assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN && !vision_search_e3_mode);
    assert(!s4_post22_search_active && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!maixcam_has_event); // No old E3 / material search resumes.
    MissionTask_UpdateVisionInput(fake_now);assert(MissionTask_RedCoordinateIsFresh());
    rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now+20);assert(tx_count==sent+2);
  }

  // A genuine ordinary E3 overrides either material-only threshold.
  for(unsigned black=0;black<2;++black) for(unsigned casualty_limit=0;casualty_limit<2;++casualty_limit) {
    setup(3,ROBOT_TEAM_BLUE,0);
    uint8_t original=black?0x31:0x51;
    s3_target_select_command=original;
    if(casualty_limit) s7_casualty_delivery_count=ROBOT_S7_MATERIAL_ONLY_AFTER_CASUALTIES;
    else s7_material_delivery_count=ROBOT_S7_MATERIAL_ONLY_AFTER_TRIPS;
    fake_now=120;rx(0xE3);tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);
    unsigned sent=tx_count;
    rx(0x11);MissionTask_HandleRedPriorityRequest(140);
    assert(s3_target_select_command==0x11 && tx_count==sent+2);
    assert(tx_commands[sent]==0x11 && tx_commands[sent+1]==0x03);
    assert(MissionTask_S7MaterialOnly() && !s7_material_only_reselect_pending &&
        !vision_search_e3_mode && !mission_snapshot.left_target_rpm);
    rx(0x11);MissionTask_HandleRedPriorityRequest(160);assert(tx_count==sent+2);
    fake_now=180;coords(black?6:5);MissionTask_UpdateVisionInput(180);
    assert(!mission_snapshot.vision_target_valid);
    coords(4);MissionTask_UpdateVisionInput(180);
    assert(MissionTask_RedCoordinateIsFresh());
  }

  // Do not loosen the legacy51 tracked-green gate outside the E3 search,
  // or switch the first mission or a supplement with retained cargo.
  for(unsigned blocked=0;blocked<3;++blocked) {
    setup(3,ROBOT_TEAM_BLUE,0);
    s51_active_object_id=5;
    if(blocked!=0) {
      fake_now=120;rx(0xE3);tick(120);
      if(blocked==1) { s7_target_switch_enabled=false;s3_target_select_command=0x21; }
      else {
        supplement_budget_started=supplement_capture_active=true;
        supplement_capture_count=1;s6_carried_object_id=5;supplement_first_request_tick=80;
      }
    }
    unsigned sent=tx_count;MissionState state=mission_snapshot.state;
    rx(0x11);MissionTask_HandleRedPriorityRequest(140);
    assert(tx_count==sent && mission_snapshot.state==state);
    if(blocked==2) assert(s6_carried_object_id==5 && supplement_capture_count==1 && supplement_first_request_tick==80);
  }
  puts("PASS:ordinary material E3 accepts11 with no51 lock or ID5/6,including both material-only thresholds;all legs,early E3+11,post22,end wait,stop/11+03/fresh ID4;duplicates and retained-load rules");
}

static void verify_e3_after_red_switch(void)
{
  setup(3,ROBOT_TEAM_BLUE,0);
  fake_now=120;rx(0xE3);tick(120);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW);

  unsigned sent=tx_count;
  fake_now=140;rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now);
  assert(s3_target_select_command==0x11 && mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  assert(tx_count==sent+2 && tx_commands[sent]==0x11 && tx_commands[sent+1]==0x03);

  fake_now=160;coords(4);tick(160);
  assert(MissionTask_RedCoordinateIsFresh() && !s7_red_recovery_used);
  fake_now=180;rx(0xE3);tick(180);
  assert(!mission_snapshot.vision_target_valid && s7_red_recovery_used);
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW &&
      vision_search_resume_state==MISSION_STATE_S7_SEARCH_RED && tx_count==sent+2);

  fake_now=200;rx(0xE3);tick(200); // Repeat while recovering, not a second loss.
  assert(mission_snapshot.state==MISSION_STATE_S3_SEARCH_TURN_CW && tx_count==sent+2);
  fake_now=220;coords(4);tick(220);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN &&
      MissionTask_RedCoordinateIsFresh() && s7_red_recovery_used && tx_count==sent+2);

  fake_now=240;rx(0xE3);tick(240);
  assert(s3_target_select_command==0x51 && tx_count==sent+4);
  assert(tx_commands[sent+2]==0x51 && tx_commands[sent+3]==0x03);
  puts("PASS:ordinary51 E3->11->ID4 tracking accepts a new E3;first red recovery,duplicates ignored,reacquisition keeps budget,second red loss falls back to51/03");
}

static void verify_red_switch_event_fence(void)
{
  uint8_t event;
  MaixCam_Object object;

  for (unsigned tx = 0; tx < 2; ++tx)
  {
    for (unsigned evt = 0; evt < 2; ++evt)
    {
      uint8_t cmd = tx ? 0x03 : 0x11;
      uint8_t response = evt ? 0x04 : 0xE3;
      setup_red_switch(cmd, response, 1);
      assert(MaixCam_PeekEvent(&event) && event == response);
      MissionTask_UpdateVisionInput(120);
      assert(!mission_snapshot.vision_target_valid && !MaixCam_GetObjectSnapshot(&object));
      assert(MissionTask_TryStartS3Capture(120));
      assert(mission_snapshot.state == (evt ? MISSION_STATE_S4_TRACK_CENTER
                                            : MISSION_STATE_S3_SEARCH_TURN_CW));
      // No repeated11/03 when processing the surviving new-stage response.
      assert(tx_count == 4);
    }
  }

  // In-flight E3 and04 are NOT coordinate frames; their tail survives the fence.
  for (unsigned evt = 0; evt < 2; ++evt)
  {
    uint8_t response = evt ? 0x04 : 0xE3;
    setup_red_switch(0x03, response, 2);
    assert(!MaixCam_TakeEvent(&event));
    byte(0x2F);
    assert(MaixCam_TakeEvent(&event) && event == response);
    assert(!MaixCam_GetObjectSnapshot(&object));
  }

  // Old-reference ID4 started before TX03 cannot publish after it.
  setup_red_switch(0x11, 0, 3);
  uint32_t floor = s7_red_coordinate_sequence_floor;
  byte(30); byte(2); byte(0x1F); byte(0x2F);
  assert(!MaixCam_GetObjectSnapshot(&object) && object.sequence == floor);
  coords(6); MissionTask_UpdateVisionInput(140);
  assert(!mission_snapshot.vision_target_valid && !MissionTask_RedCoordinateIsFresh());
  coords(4); MissionTask_UpdateVisionInput(140);
  assert(mission_snapshot.vision_target_valid && MissionTask_RedCoordinateIsFresh());
  assert(object.sequence == floor && mission_snapshot.target_sequence > floor);

  // A new complete or partial ID4 in the TX03 window must survive.
  setup_red_switch(0x03, 0, 5);
  MissionTask_UpdateVisionInput(140);
  assert(MissionTask_RedCoordinateIsFresh());
  setup_red_switch(0x03, 0, 3);
  byte(30); byte(2); byte(0x1F); byte(0x2F);
  MissionTask_UpdateVisionInput(140);
  assert(MissionTask_RedCoordinateIsFresh());

  // Pre-command partial coordinates are fenced too, without stopping RX.
  setup_red_switch(0x11, 0x04, 4);
  assert(s7_red_coordinate_sequence_floor == 3); // PreTX one + TX two; old partial not counted.
  assert(MaixCam_TakeEvent(&event) && event == 0x04);
  verify_material_e3_priority();
  verify_e3_after_red_switch();

  // With no new reply the OLD E3 must not survive the preTX cleanup.
  setup(3, ROBOT_TEAM_BLUE, 0); s51_active_object_id = 6;
  MissionTask_EnterState(MISSION_STATE_S4_TRACK_FINAL_BLOCK, 110);
  rx(0xE3); rx(0x11); MissionTask_HandleRedPriorityRequest(120);
  assert(!MaixCam_TakeEvent(&event));

  // Repeated11 on an already-red task must not erase its pending04 or budget.
  setup_red_switch(0x03, 0x04, 1);
  s7_red_recovery_used = true;
  rx(0x11); MissionTask_HandleRedPriorityRequest(140);
  assert(tx_count == 4 && s7_red_recovery_used);
  assert(MaixCam_TakeEvent(&event) && event == 0x04);
  puts("PASS:black->red preTX input fence,11/03 new E3/04 and partial events retained, complete/partial stale coordinates rejected, fresh ID4 and duplicate11 budget unchanged");
}
