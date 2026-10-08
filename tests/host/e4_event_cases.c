/* Real UART parser, E4 recovery handlers,04/14 TX helpers and arrange wait. */
static void setup_green_push(void)
{
  setup(0x21);
  s7_target_switch_enabled = false;
  MissionTask_StartS4E4Recovery(120);
  assert(mission_snapshot.state == MISSION_STATE_S4_E4_PUSH_RIGHT);
}

static uint32_t finish_green_push(void)
{
  const MissionState states[] = {MISSION_STATE_S4_E4_PUSH_RIGHT,
    MISSION_STATE_S4_E4_REVERSE_RIGHT, MISSION_STATE_S4_E4_PUSH_LEFT,
    MISSION_STATE_S4_E4_REVERSE_LEFT};
  const uint32_t durations[] = {ROBOT_S4_RIGHT_CORNER_FOLLOW_MS, 500,
    ROBOT_S4_LEFT_CORNER_FOLLOW_MS, 500};
  uint32_t t = mission_snapshot.state_entry_tick;
  for (unsigned i = 0; i < 4; ++i)
  {
    assert(mission_snapshot.state == states[i]);
    fake_now = t + 20;
    coords(5); rx(0xE4);
    tick(t + 20);
    assert(mission_snapshot.state == states[i] && tx_count == 0);
    assert(i % 2 == 0 ? mission_snapshot.left_target_rpm > 0
                      : mission_snapshot.left_target_rpm < 0);
    tick(t + durations[i] - 1);
    assert(mission_snapshot.state == states[i] && tx_count == 0);
    t += durations[i]; tick(t);
  }
  assert(mission_snapshot.state == MISSION_STATE_S4_WAIT_ARRANGE_READY);
  assert(tx_count == 1 && tx_commands[0] == 0x44);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(s4_green_e4_push_cycles == 1 && !maixcam_e4_stage_buffering);
  return t;
}

static void verify_e4_event_buffering(void)
{
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

  //14 and02 must not interrupt even a single timed leg. ACK44 precedes ACK14.
  setup_green_push(); rx(0x14); rx(0xE4); rx(0x02);
  done = finish_green_push();
  assert(!acknowledgements && MaixCam_PeekEvent(&event) && event == 0x14);
  tick(done + 20);
  assert(tx_count == 2 && tx_commands[1] == 0x14 && acknowledgements == 1);
  assert(MaixCam_PeekEvent(&event) && event == 0x02);
  tick(done + 40);
  assert(mission_snapshot.state == MISSION_STATE_S4_TRACK_RIGHT_BLOCK);

  // A24 arriving on any leg waits for all four legs and then enters final track.
  for (unsigned leg = 0; leg < 4; ++leg)
  {
    const uint32_t d[] = {ROBOT_S4_RIGHT_CORNER_FOLLOW_MS, 500,
                         ROBOT_S4_LEFT_CORNER_FOLLOW_MS, 500};
    setup_green_push();
    uint32_t t = 120;
    for (unsigned i = 0; i < 4; ++i)
    {
      if (i == leg) { fake_now = t + 1; rx(0x24); rx(0xE4); }
      tick(t + d[i] - 1); assert(tx_count == 0);
      t += d[i]; tick(t);
    }
    assert(tx_count == 1 && tx_commands[0] == 0x44);
    tick(t + 20);
    assert(mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME);
  }

  // New replies received inside synchronous44 TX also survive, in order.
  setup_green_push(); inject_stage_on_tx = 0x44;
  done = finish_green_push(); tick(done + 20); tick(done + 40);
  assert(tx_count == 2 && tx_commands[1] == 0x14);
  assert(mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME);

  // Keep the explicit E5 old-input fence, but preserve the post44 reply.
  setup_green_push(); s5_empty_recovery_active = true;
  rx(0x14); rx(0x24); done = finish_green_push();
  assert(!MaixCam_TakeEvent(&event) && !acknowledgements);
  assert(!MaixCam_GetObjectSnapshot(&object));
  fake_now = done + 20; rx(0x24); tick(done + 20);
  assert(mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME);
  setup_green_push(); s5_empty_recovery_active = true;
  rx(0x14); inject_stage_on_tx = 0x44;
  done = finish_green_push(); tick(done + 20); tick(done + 40);
  assert(acknowledgements == 1 && mission_snapshot.state == MISSION_STATE_S4_RAISE_FRAME);

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
  puts("PASS:E4 ordered14/02/24 buffering, duplicate loss drops, full green four-leg non-interruption,44/04-before14 handoff, TX bursts, E5 fence,05 and safety");
}
