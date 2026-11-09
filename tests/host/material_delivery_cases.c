/* Count successful releases at the border, then follow the real back/exit07
   path. Setup/actuator/driver spies and UART parser come from the parent test. */
static void start_border_advance(uint32_t start) {
  // This fixture skips06, whose entry closes the claw before the approach.
  deferred_close=false;
  assert(Actuator_SetClawClosed()==HAL_OK);
  s6_push_yaw_target_cdeg=mission_snapshot.yaw_cdeg;
  MissionTask_EnterState(MISSION_STATE_S6_BORDER_LOCATE,start);
  MissionTask_RunS6BorderLocate(start);
}

static uint32_t finish_border_delivery(uint32_t start) {
  uint32_t back=start+ROBOT_S6_BORDER_LOCATE_MS;
  uint32_t push=back+ROBOT_S6_BORDER_BACKOFF_MS;
  bool already_counted=s7_delivery_counted;
  MissionTask_RunS6BorderLocate(back);
  assert(mission_snapshot.state==MISSION_STATE_S6_BORDER_BACKOFF && frame_down && s7_delivery_counted==already_counted);
  MissionTask_RunS6BorderLocate(push);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_PUSH && !frame_down && s7_delivery_counted==already_counted);
  fake_now=push+ROBOT_S6_FINAL_PUSH_MS;
  MissionTask_RunS6BorderLocate(fake_now);
  return fake_now;
}

static uint32_t material_trip_push(uint32_t start,uint8_t id) {
  s6_carried_object_id=id;s5_load_class_confirmed=true;
  start_border_advance(start);
  fake_now=start+ROBOT_S6_BORDER_LOCATE_MS-1U;
  unsigned previous=s7_material_delivery_count;
  unsigned previous_red=s7_casualty_delivery_count;
  MissionTask_RunS6BorderLocate(fake_now);
  assert(s7_material_delivery_count==previous && s7_casualty_delivery_count==previous_red && mission_snapshot.left_target_rpm==ROBOT_S6_BORDER_LOCATE_RPM);
  assert(delivery_claw==ROBOT_CLAW_CLOSED_DEG);
  finish_border_delivery(start);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE && s7_delivery_counted);
  uint8_t delivered=MissionTask_GetCarriedObjectId();
  bool supply=delivered==3 || delivered==5 || delivered==6 || (delivered==0 && s3_target_select_command==0x51);
  assert(delivery_claw==(delivered==4?73U:supply?74U:63U));
  assert(!frame_down);
  MissionTask_RunS6BorderLocate(fake_now+20); // State guard prevents duplicate completion.
  return fake_now;
}

static uint32_t material_trip_exit(uint32_t released) {
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE);
  assert(delivery_claw!=ROBOT_CLAW_CLOSED_DEG);
  fake_now=released+ROBOT_S6_REVERSE_DRIVE_MS;
  MissionTask_RunS6FinalReverse(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180);
  MissionTask_RunS6ExitTurn180(fake_now+20U);
  assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180);
  assert(delivery_claw==ROBOT_CLAW_CLOSED_DEG && !frame_down);
  fake_now+=ROBOT_HEADING_TURN_TIMEOUT_MS;
  MissionTask_RunS6ExitTurn180(fake_now);
  assert(!s7_delivery_counted && frame_down && deferred_close);
  assert(delivery_claw!=ROBOT_CLAW_CLOSED_DEG);
  return fake_now;
}

static void verify_timed_out_delivery_exit(uint32_t timeout_tick) {
  unsigned search_commands=tx_count; // setup already started the original search.
  unsigned track_before=track_view_calls;
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_REVERSE && !mission_snapshot.fault_flags);
  assert(delivery_claw==(MissionTask_GetCarriedObjectId()==4U?73U:74U) && !frame_down);
  MissionTask_RunS6FinalReverse(timeout_tick);
  assert(mission_snapshot.left_target_rpm==-100 && mission_snapshot.right_target_rpm==-100);
  uint32_t turn_start=timeout_tick+ROBOT_S6_REVERSE_DRIVE_MS;
  MissionTask_RunS6FinalReverse(turn_start);
  MissionTask_RunS6ExitTurn180(turn_start+20U);
  assert(mission_snapshot.state==MISSION_STATE_S6_EXIT_TURN_180 && tx_count==search_commands);
  fake_now=turn_start+ROBOT_HEADING_TURN_TIMEOUT_MS;
  MissionTask_RunS6ExitTurn180(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED && frame_down);
  assert(track_view_calls==track_before+1); // Apply the03 pose on leaving26.
  assert(tx_count==search_commands+2 && tx_commands[search_commands]==0x11 && tx_commands[search_commands+1]==0x03);
}

static void verify_material_delivery_policy(void) {
  assert(ROBOT_S7_MATERIAL_ONLY_AFTER_TRIPS==2U);
  // First green, standalone black31, red, non-material or unknown cargo are
  // not completed51 material trips, regardless of seeing a final26 push.
  const uint8_t commands[]={0x21,0x31,0x11,0x51,0x51,0x51,0x51};
  const uint8_t ids[]={5,6,5,4,3,0,5};
  for(unsigned i=0;i<sizeof(commands);++i) {
    setup(3,ROBOT_TEAM_BLUE,0);s3_target_select_command=commands[i];
    if(i==6) s7_target_switch_enabled=false;
    s51_active_object_id=0;
    (void)material_trip_push(1000,ids[i]);
    assert(!s7_material_delivery_count);
  }

  setup(3,ROBOT_TEAM_BLUE,0);
  // Searching/reselecting51, recovery and multiple supplements add no trip.
  for(unsigned i=0;i<3;++i) {
    MissionTask_ResetSupplement();
    assert(MissionTask_BeginS3Vision(200+i*20));
    supplement_capture_count=i+1;
    assert(!s7_material_delivery_count && !s7_delivery_counted);
  }
  uint32_t pushed=material_trip_push(1000,5);
  assert(s7_material_delivery_count==1);
  // Even a forced replay of the fixed push or a duplicate51 cannot recount
  // the same load; only starting the NEXT07 cycle rearms the delivery latch.
  MissionTask_ResetSupplement();
  assert(MissionTask_BeginS3Vision(pushed+40));
  assert(s7_delivery_counted && s7_material_delivery_count==1);
  (void)material_trip_push(pushed+60,5);
  assert(s7_material_delivery_count==1);
  pushed=mission_snapshot.state_entry_tick;
  unsigned before=tx_count;
  uint32_t ready=material_trip_exit(pushed);
  assert(s7_material_delivery_count==1 && s3_target_select_command==0x11);
  assert(mission_snapshot.state==MISSION_STATE_S7_SEARCH_RED);
  assert(tx_count==before+2 && tx_commands[before]==0x11 && tx_commands[before+1]==0x03);

  // No red target: the same existing fallback selects51. Second completed
  // delivery switches the very next post180 search to ordinary51/03.
  assert(MissionTask_StartS3Align(ready+20,0x51));
  MissionTask_ResetSupplement();supplement_capture_count=2;
  pushed=material_trip_push(ready+100,6);
  assert(s7_material_delivery_count==2);
  before=tx_count;inject_exit_on_tx=0x04;
  ready=material_trip_exit(pushed);
  assert(s7_material_delivery_count==2 && s3_target_select_command==0x51);
  assert(mission_snapshot.state==MISSION_STATE_S3_TRACK_GREEN);
  assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x03);
  MissionTask_UpdateVisionInput(fake_now);
  uint8_t event;
  assert(mission_snapshot.vision_target_valid && mission_snapshot.target_id==6);
  assert(MaixCam_PeekEvent(&event) && event==0x04); // In-TX new reference survives.
  (void)MaixCam_TakeEvent(&event);
  fake_now=ready+20;coords(5);MissionTask_UpdateVisionInput(fake_now);
  assert(mission_snapshot.vision_target_valid && mission_snapshot.target_id==5);

  // A camera waiting for11/03 is explicitly redirected to51/03, not ignored.
  s51_active_object_id=6;before=tx_count;
  fake_now+=20;rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now);
  assert(s3_target_select_command==0x51 && s7_material_only_reselect_pending);
  assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x03);
  for(unsigned i=0;i<3;++i) { rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now+20); }
  assert(tx_count==before+2 && s7_material_delivery_count==2);
  fake_now+=40;coords(4);MissionTask_UpdateVisionInput(fake_now);
  assert(!mission_snapshot.vision_target_valid && s7_material_only_reselect_pending);
  coords(6);MissionTask_UpdateVisionInput(fake_now);
  assert(mission_snapshot.vision_target_valid && !s7_material_only_reselect_pending);
  // Threshold persists through supplements and third/later full deliveries.
  MissionTask_ResetSupplement();assert(s7_material_delivery_count==2);
  pushed=material_trip_push(fake_now+100,5);assert(s7_material_delivery_count==3);
  before=tx_count;ready=material_trip_exit(pushed);
  assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x03);

  // Stop/fault preserve completed trips; explicit RESET and a fresh START
  // clear all mission policy state. Neither needs a hardware power cycle.
  MissionTask_ProcessEvents(MISSION_EVENT_STOP,ready+20);
  assert(s7_material_delivery_count==3 && mission_snapshot.state==MISSION_STATE_STOPPED);
  MissionTask_ProcessEvents(MISSION_EVENT_EMERGENCY_STOP,ready+40);
  assert(s7_material_delivery_count==3 && mission_snapshot.state==MISSION_STATE_FAULT);
  s7_delivery_counted=s7_material_only_reselect_pending=true;
  MissionTask_ProcessEvents(MISSION_EVENT_RESET,ready+60);
  assert(!s7_material_delivery_count && !s7_delivery_counted && !s7_material_only_reselect_pending);
  assert(mission_snapshot.state==MISSION_STATE_WAIT_START && s3_target_select_command==0x21);
  s7_material_delivery_count=2;s7_delivery_counted=s7_material_only_reselect_pending=true;
  MissionTask_ProcessEvents(MISSION_EVENT_START,ready+80);
  assert(!s7_material_delivery_count && !s7_delivery_counted && !s7_material_only_reselect_pending);
  assert(mission_snapshot.state==MISSION_STATE_S1_DEPART && s3_target_select_command==0x21);

  // Timeout/actuator failure before the successful completion never counts.
  for(unsigned timeout=0;timeout<2;++timeout) {
    setup(3,ROBOT_TEAM_BLUE,0);s6_carried_object_id=5;
    claw_release_ok=timeout!=0;
    start_border_advance(1000);
    if(timeout) {
      MissionTask_RunS6BorderLocate(1000+ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS);
      verify_timed_out_delivery_exit(1000+ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS);
    } else {
      MissionTask_RunS6BorderLocate(1000+ROBOT_S6_BORDER_LOCATE_MS);
      assert(mission_snapshot.state==MISSION_STATE_FAULT);
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    }
    assert(!s7_material_delivery_count && !s7_delivery_counted);
  }
  setup(3,ROBOT_TEAM_BLUE,0);s7_material_delivery_count=UINT16_MAX;
  (void)material_trip_push(1000,6);assert(s7_material_delivery_count==UINT16_MAX);
  puts("PASS:51 delivery count only after final push,not first21/red31/hazard/unknown;0/1->11+03,2+->ordinary51+03 ID5/6;duplicate26/51/supplement/recovery do not recount;RX11 redirected/dedup/new frames;new mission/reset clears,stop/fault retain,saturation and actuator/timeout safety");
}

static void verify_casualty_delivery_policy(void) {
  assert(ROBOT_S7_MATERIAL_ONLY_AFTER_CASUALTIES==4U);
  // Four real confirmed single-red deliveries, independent of51 trip count.
  // Exercise every start-zone/team; each exit actually lowers and sends commands.
  for(uint8_t zone=1;zone<=4;++zone) for(uint8_t team=ROBOT_TEAM_RED;team<=ROBOT_TEAM_BLUE;++team) {
    setup(zone,team,35000);s3_target_select_command=0x11;
    uint32_t ready=1000;
    for(unsigned n=1;n<=4;++n) {
      uint32_t pushed=material_trip_push(ready+100U,4);
      assert(s7_casualty_delivery_count==n && !s7_material_delivery_count);
      assert(MissionTask_S7MaterialOnly()==(n>=4));
      // Replaying26 or entering recovery/supplement cannot recount the load.
      MissionTask_ResetSupplement();
      (void)material_trip_push(pushed+40U,4);
      assert(s7_casualty_delivery_count==n && !s7_material_delivery_count);
      unsigned before=tx_count;
      ready=material_trip_exit(mission_snapshot.state_entry_tick);
      assert(tx_count==before+2 && tx_commands[before]==(n>=4?0x51:0x11) && tx_commands[before+1]==0x03);
      assert(mission_snapshot.state==(n>=4?MISSION_STATE_S3_TRACK_GREEN:MISSION_STATE_S7_SEARCH_RED));
    }
    // A pending visual11 request is actively rejected after four casualties.
    s51_active_object_id=6;unsigned before=tx_count;
    fake_now=ready+20U;rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now);
    assert(s7_material_only_reselect_pending && s3_target_select_command==0x51);
    assert(tx_count==before+2 && tx_commands[before]==0x51 && tx_commands[before+1]==0x03);
    rx(0x11);MissionTask_HandleRedPriorityRequest(fake_now+20U);
    assert(tx_count==before+2 && s7_casualty_delivery_count==4);
    fake_now+=40U;coords(4);MissionTask_UpdateVisionInput(fake_now);
    assert(!mission_snapshot.vision_target_valid);
    coords(5);MissionTask_UpdateVisionInput(fake_now);
    assert(mission_snapshot.vision_target_valid && !s7_material_only_reselect_pending);
  }

  // Count the confirmed load, never the original task colour or RX ID packets.
  setup(3,ROBOT_TEAM_BLUE,0);s3_target_select_command=0x51;
  (void)material_trip_push(1000,4);
  assert(s7_casualty_delivery_count==1 && !s7_material_delivery_count);
  setup(3,ROBOT_TEAM_BLUE,0);s3_target_select_command=0x11;
  (void)material_trip_push(1000,5);
  assert(!s7_casualty_delivery_count && !s7_material_delivery_count);
  setup(3,ROBOT_TEAM_BLUE,0);s3_target_select_command=0x11;s6_carried_object_id=4;
  s5_load_class_confirmed=false;
  start_border_advance(1000);
  finish_border_delivery(1000);
  assert(!s7_casualty_delivery_count);
  setup(3,ROBOT_TEAM_BLUE,0);s7_target_switch_enabled=false;
  (void)material_trip_push(1000,4);assert(!s7_casualty_delivery_count);

  // OR, not addition: one51 trip plus three casualties still seeks red.
  setup(3,ROBOT_TEAM_BLUE,0);s7_material_delivery_count=1;s7_casualty_delivery_count=3;
  assert(!MissionTask_S7MaterialOnly());
  MissionTask_StartS7Decision(120);assert(s3_target_select_command==0x11);
  s3_target_select_command=0x51;
  (void)material_trip_push(1000,6);
  assert(s7_material_delivery_count==2 && s7_casualty_delivery_count==3 && MissionTask_S7MaterialOnly());

  // A failed delivery does not count, nor can a uint16 counter wrap to zero.
  for(unsigned timeout=0;timeout<2;++timeout) {
    setup(3,ROBOT_TEAM_BLUE,0);s3_target_select_command=0x11;
    s6_carried_object_id=4;s5_load_class_confirmed=true;
    claw_release_ok=timeout!=0;start_border_advance(1000);
    if(timeout) {
      MissionTask_RunS6BorderLocate(1000+ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS);
      verify_timed_out_delivery_exit(1000+ROBOT_S6_BORDER_LOCATE_TIMEOUT_MS);
    } else {
      MissionTask_RunS6BorderLocate(1000+ROBOT_S6_BORDER_LOCATE_MS);
      assert(mission_snapshot.state==MISSION_STATE_FAULT);
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    }
    assert(!s7_casualty_delivery_count && !s7_delivery_counted);
  }
  setup(3,ROBOT_TEAM_BLUE,0);s7_casualty_delivery_count=UINT16_MAX;
  (void)material_trip_push(1000,4);assert(s7_casualty_delivery_count==UINT16_MAX);
  s7_material_delivery_count=1;
  MissionTask_ProcessEvents(MISSION_EVENT_STOP,2000);
  assert(s7_material_delivery_count==1 && s7_casualty_delivery_count==UINT16_MAX);
  MissionTask_ProcessEvents(MISSION_EVENT_EMERGENCY_STOP,2020);
  assert(s7_material_delivery_count==1 && s7_casualty_delivery_count==UINT16_MAX);
  MissionTask_ProcessEvents(MISSION_EVENT_RESET,2040);
  assert(!s7_material_delivery_count && !s7_casualty_delivery_count && !MissionTask_S7MaterialOnly());
  s7_casualty_delivery_count=4;s7_material_delivery_count=1;
  MissionTask_ProcessEvents(MISSION_EVENT_START,2060);
  assert(!s7_material_delivery_count && !s7_casualty_delivery_count && !MissionTask_S7MaterialOnly());
  puts("PASS:four confirmed ID4 deliveries OR two51 trips select51/03 in all zone/team cases;1..3 red stays11/03,actual class/report gate,duplicate26/recovery/supplement no recount,RX11 reject/dedup,independent counters/failure/saturation/stop/reset/start safety");
}
