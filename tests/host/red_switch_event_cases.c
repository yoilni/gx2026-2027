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

  // Old-reference ID4 partially received during TX cannot republish as fresh.
  setup_red_switch(0x03, 0, 3);
  uint32_t floor = s7_red_coordinate_sequence_floor;
  byte(30); byte(2); byte(0x1F); byte(0x2F);
  assert(!MaixCam_GetObjectSnapshot(&object) && object.sequence == floor);
  coords(6); MissionTask_UpdateVisionInput(140);
  assert(!mission_snapshot.vision_target_valid && !MissionTask_RedCoordinateIsFresh());
  coords(4); MissionTask_UpdateVisionInput(140);
  assert(mission_snapshot.vision_target_valid && MissionTask_RedCoordinateIsFresh());
  assert(object.sequence == floor && mission_snapshot.target_sequence > floor);

  // Pre-command partial coordinates are fenced too, without stopping RX.
  setup_red_switch(0x11, 0x04, 4);
  assert(s7_red_coordinate_sequence_floor == 3); // PreTX one + TX two; old partial not counted.
  assert(MaixCam_TakeEvent(&event) && event == 0x04);

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
