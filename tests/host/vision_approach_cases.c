/* Compiled against the real forward guard, PID and03/04/24 wheel callers. */
static void approach_setup(MissionState state, uint8_t reference) {
  memset(&mission_snapshot, 0, sizeof(mission_snapshot));
  mission_snapshot.state = state;
  mission_snapshot.state_entry_tick = fake_now = 100;
  mission_snapshot.target_id = 5;
  mission_snapshot.vision_target_valid = true;
  supplement_reference_command = reference;
  s3_search_sweep_completed = false;
  MissionTask_ResetVisionControllerState();
}
static int approach_frame(int16_t y) {
  fake_now += 100;
  ++mission_snapshot.target_sequence;
  mission_snapshot.target_age_ms = 0;
  mission_snapshot.vision_y_error_px = y;
  return MissionTask_CalculateVisionForward(fake_now, y);
}
static void verify_vision_approach(void) {
  const MissionState states[] = {MISSION_STATE_S3_TRACK_GREEN,
    MISSION_STATE_S4_TRACK_CENTER, MISSION_STATE_S4_TRACK_FINAL_BLOCK,
    MISSION_STATE_SUPPLEMENT_TRACK, MISSION_STATE_SUPPLEMENT_TRACK,
    MISSION_STATE_SUPPLEMENT_TRACK};
  const uint8_t refs[] = {0x03,0x04,0x24,0x03,0x04,0x24};
  for (unsigned i=0; i<sizeof(refs); ++i) {
    approach_setup(states[i],refs[i]);
    assert(approach_frame(-12)==0);
    assert(approach_frame(-13)==20);
    assert(approach_frame(-33)==26);
    assert(approach_frame(-50)==31);
    assert(approach_frame(-80)==40);
    int previous=40;
    for (int y=81; y<=448; ++y) {
      int speed=approach_frame(-y);
      assert(speed>=previous && speed<=150);
      previous=speed;
    }
    //Repeated task ticks are not new camera frames.
    assert(approach_frame(448)==0);
    for (unsigned n=0; n<10; ++n) {
      fake_now+=20; mission_snapshot.target_age_ms+=20;
      assert(MissionTask_CalculateVisionForward(fake_now,448)==0);
      assert(vision_forward_pending_frames==1);
    }
    assert(approach_frame(448)==-30);
    assert(approach_frame(13)==-20);
    assert(approach_frame(-13)==0);
    assert(approach_frame(-13)==20);
    //A zero/error-in-tolerance frame interrupts the candidate, not the
    //accepted direction; forward -> zero -> backward still needs two frames.
    assert(approach_frame(13)==0);
    assert(approach_frame(0)==0);
    assert(vision_forward_direction==1 && !vision_forward_pending_frames);
    assert(approach_frame(13)==0);
    assert(approach_frame(13)==-20);
    //A fresh frame from a different class cannot complete confirmation.
    assert(approach_frame(-50)==0);
    mission_snapshot.target_id=6;
    assert(approach_frame(-50)==0);
    assert(approach_frame(-50)==31);
    //A real gap above the500ms coordinate lifetime starts confirmation over.
    assert(approach_frame(50)==0);
    fake_now+=501;
    assert(approach_frame(50)==0);
    assert(approach_frame(50)==-30);
    assert(approach_frame(-50)==0);
    mission_snapshot.vision_target_valid=false;
    assert(MissionTask_CalculateVisionForward(fake_now,-50)==0);
    assert(!vision_forward_pending_frames);
    mission_snapshot.vision_target_valid=true;
    assert(approach_frame(-50)==0);
    assert(approach_frame(-50)==31);
  }
  //Replay the large fore/aft oscillation seen in the real log, with the
  //actual03 caller (not just the helper). Positive -> negative is held at0.
  approach_setup(MISSION_STATE_S3_TRACK_GREEN,0x03);
  const int16_t errors[]={-128,33,-132,0,50,50};
  const int expected[]={72,0,75,0,0,-30};
  for (unsigned i=0; i<sizeof(errors)/sizeof(errors[0]); ++i) {
    fake_now+=100; ++mission_snapshot.target_sequence;
    mission_snapshot.target_age_ms=0;
    mission_snapshot.vision_y_error_px=errors[i];
    MissionTask_RunS3Track(fake_now);
    assert(mission_snapshot.left_target_rpm==expected[i]);
    assert(mission_snapshot.right_target_rpm==expected[i]);
  }
  //500ms expiry and progressive coordinate-age fade cannot reapply a floor.
  approach_setup(MISSION_STATE_S3_TRACK_GREEN,0x03);
  assert(approach_frame(-50)==31);
  MissionTask_RunS3Track(fake_now);
  assert(mission_snapshot.left_target_rpm==31);
  fake_now+=375; mission_snapshot.target_age_ms=375;
  MissionTask_RunS3Track(fake_now);
  assert(mission_snapshot.left_target_rpm==15);
  fake_now+=125; mission_snapshot.target_age_ms=500;
  MissionTask_RunS3Track(fake_now);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  //All04/24 and supplement callers use the same near approach profile.
  for (unsigned i=1;i<sizeof(refs);++i) {
    approach_setup(states[i],refs[i]);
    assert(approach_frame(-50)==31);
    bool reached;
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(mission_snapshot.left_target_rpm==31 && mission_snapshot.right_target_rpm==31);
    assert(approach_frame(50)==0);
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(!mission_snapshot.left_target_rpm);
    assert(approach_frame(50)==-30);
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(mission_snapshot.left_target_rpm==-30 && mission_snapshot.right_target_rpm==-30);
  }
  //Changing supplement reference or task resets the pending candidate.
  approach_setup(MISSION_STATE_SUPPLEMENT_TRACK,0x03);
  assert(approach_frame(50)==0);
  supplement_reference_command=0x24;
  assert(approach_frame(50)==0);
  assert(approach_frame(50)==-30);
  mission_snapshot.state=MISSION_STATE_S4_TRACK_CENTER;
  assert(approach_frame(50)==0);
  MissionTask_ResetVisionControllerState();
  assert(approach_frame(50)==0);
  //Sequence UINT32_MAX is a legitimate first frame, not an empty sentinel.
  approach_setup(MISSION_STATE_S3_TRACK_GREEN,0x03);
  mission_snapshot.target_sequence=UINT32_MAX-1U;
  fake_now=UINT32_MAX-150U;
  assert(approach_frame(-50)==31);
  assert(approach_frame(50)==0);
  assert(approach_frame(50)==-30);
  //06 is explicitly outside the new object approach/reversal guard.
  approach_setup(MISSION_STATE_S6_TRACK_SAFE_ZONE,0);
  assert(approach_frame(-50)==100);
  assert(approach_frame(50)==-100);
  puts("PASS:03/04/24/supplement near approach, monotonic far transition, fresh-frame reversal, reverse cap, actual wheel callers, deadband/freshness/wrap/fades;06 unchanged");
}
