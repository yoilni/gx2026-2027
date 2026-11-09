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
  assert(ROBOT_VISION_03_NEAR_MIN_RPM==100 && ROBOT_VISION_03_NEAR_MAX_RPM==150);
  assert(ROBOT_VISION_03_MID_MIN_RPM==150 && ROBOT_VISION_03_MIN_FORWARD_RPM==170);
  assert(ROBOT_VISION_03_MAX_FORWARD_RPM==200 && ROBOT_VISION_03_WHEEL_MAX_RPM==200);
  assert(ROBOT_VISION_03_FAR_FULL_SPEED_PX==MAIXCAM_Y_ERROR_MAX);
  for (unsigned i=0; i<sizeof(refs); ++i) {
    bool search=refs[i]==0x03;
    bool faster=refs[i]!=0x24;
    int near_edge=search?101:(faster?70:40), near33=search?115:(faster?79:49);
    int near50=search?128:(faster?87:57), near80=search?150:(faster?100:70);
    approach_setup(states[i],refs[i]);
    assert(approach_frame(-12)==0);
    assert(approach_frame(-13)==near_edge);
    assert(approach_frame(-33)==near33);
    assert(approach_frame(-50)==near50);
    assert(approach_frame(-80)==near80);
    int previous=near80;
    for (int y=81; y<=448; ++y) {
      int speed=approach_frame(-y);
      assert(speed>=previous && speed<=(search?200:(faster?170:120)));
      if(search && y<200) assert(speed>=150 && speed<=170);
      if(search && y>=200) assert(speed>=170 && speed<=200);
      if(!search && faster && y<200) assert(speed>=120 && speed<=150);
      if(!search && faster && y>=200) assert(speed>=150 && speed<=170);
      previous=speed;
    }
    //Repeated task ticks are not new camera frames.
    assert(approach_frame(448)==0);
    for (unsigned n=0; n<10; ++n) {
      fake_now+=20; mission_snapshot.target_age_ms+=20;
      assert(MissionTask_CalculateVisionForward(fake_now,448)==0);
      assert(vision_motion.reverse.pending_frames==1);
    }
    assert(approach_frame(448)==-30);
    assert(approach_frame(13)==-20);
    assert(approach_frame(-13)==0);
    assert(approach_frame(-13)==near_edge);
    //A zero/error-in-tolerance frame interrupts the candidate, not the
    //accepted direction; forward -> zero -> backward still needs two frames.
    assert(approach_frame(13)==0);
    assert(approach_frame(0)==0);
    assert(vision_motion.reverse.direction==1 && !vision_motion.reverse.pending_frames);
    assert(approach_frame(13)==0);
    assert(approach_frame(13)==-20);
    //A fresh frame from a different class cannot complete confirmation.
    assert(approach_frame(-50)==0);
    mission_snapshot.target_id=6;
    assert(approach_frame(-50)==0);
    assert(approach_frame(-50)==near50);
    //A real gap above the500ms coordinate lifetime starts confirmation over.
    assert(approach_frame(50)==0);
    fake_now+=501;
    assert(approach_frame(50)==0);
    assert(approach_frame(50)==-30);
    assert(approach_frame(-50)==0);
    mission_snapshot.vision_target_valid=false;
    assert(MissionTask_CalculateVisionForward(fake_now,-50)==0);
    assert(!vision_motion.reverse.pending_frames);
    mission_snapshot.vision_target_valid=true;
    assert(approach_frame(-50)==0);
    assert(approach_frame(-50)==near50);
  }
  //Replay the large fore/aft oscillation seen in the real log, with the
  //actual03 caller (not just the helper). Positive -> negative is held at0.
  approach_setup(MISSION_STATE_S3_TRACK_GREEN,0x03);
  const int16_t errors[]={-128,33,-132,0,50,50};
  const int expected[]={158,0,159,0,0,-30};
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
  assert(approach_frame(-50)==128);
  MissionTask_RunS3Track(fake_now);
  assert(mission_snapshot.left_target_rpm==128);
  fake_now+=375; mission_snapshot.target_age_ms=375;
  MissionTask_RunS3Track(fake_now);
  assert(mission_snapshot.left_target_rpm==64);
  fake_now+=125; mission_snapshot.target_age_ms=500;
  MissionTask_RunS3Track(fake_now);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  //Actual04/24 and supplementary references all use the faster approach.
  for (unsigned i=1;i<sizeof(refs);++i) {
    int near50=refs[i]==0x03?128:(refs[i]==0x24?57:87);
    approach_setup(states[i],refs[i]);
    assert(approach_frame(-50)==near50);
    bool reached;
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(mission_snapshot.left_target_rpm==near50 && mission_snapshot.right_target_rpm==near50);
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
  assert(approach_frame(-50)==128);
  assert(approach_frame(50)==0);
  assert(approach_frame(50)==-30);
  //06 is explicitly outside the new object approach/reversal guard.
  approach_setup(MISSION_STATE_S6_TRACK_SAFE_ZONE,0);
  assert(approach_frame(-50)==150);
  assert(approach_frame(50)==-100);
  assert(approach_frame(-400)==160);
  assert(approach_frame(-450)==180);
  assert(approach_frame(450)==-150);
  //Final wheel commands must not inherit04's previous70/90 caps.
  for(unsigned i=0;i<sizeof(refs);++i) if(refs[i]!=0x24) {
    bool search=refs[i]==0x03;
    int cap=search?200:170;
    approach_setup(states[i],refs[i]);
    assert(approach_frame(-200)==(search?170:150));
    assert(approach_frame(-448)==cap);
    bool reached;
    if(i==0) MissionTask_RunS3Track(fake_now);
    else assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(mission_snapshot.left_target_rpm==cap && mission_snapshot.right_target_rpm==cap);
    mission_snapshot.vision_x_error_px=300;
    if(i==0) MissionTask_RunS3Track(fake_now+20);
    else assert(MissionTask_CommandS4VisionTracking(fake_now+20,&reached));
    assert(mission_snapshot.left_target_rpm<=cap && mission_snapshot.right_target_rpm<=cap);
    assert(mission_snapshot.left_target_rpm>90 && mission_snapshot.right_target_rpm>90);
  }
  // Every03 band boundary reaches the real ordinary/supplement wheel callers.
  const int16_t ys[]={0,12,13,50,80,81,140,199,200,324,448};
  const int speeds[]={0,0,101,128,150,150,160,170,170,185,200};
  for(unsigned i=0;i<sizeof(refs);++i) if(refs[i]==0x03) {
    approach_setup(states[i],refs[i]);
    for(unsigned y=0;y<sizeof(ys)/sizeof(ys[0]);++y) {
      assert(approach_frame(-ys[y])==speeds[y]);
      bool reached;
      if(i==0) MissionTask_RunS3Track(fake_now);
      else assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
      assert(mission_snapshot.left_target_rpm==speeds[y] && mission_snapshot.right_target_rpm==speeds[y]);
    }
  }
  //24's higher near endpoint never slopes back to the old50rpm far floor.
  for(unsigned i=0;i<sizeof(refs);++i) if(refs[i]==0x24) {
    approach_setup(states[i],refs[i]);
    assert(approach_frame(-80)==70);
    assert(approach_frame(-81)==70);
    assert(approach_frame(-120)==70);
    assert(approach_frame(-200)==70);
    assert(approach_frame(-448)==120);
    bool reached;
    assert(MissionTask_CommandS4VisionTracking(fake_now,&reached));
    assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    mission_snapshot.vision_x_error_px=300;
    assert(MissionTask_CommandS4VisionTracking(fake_now+20,&reached));
    assert(mission_snapshot.left_target_rpm<=120 && mission_snapshot.right_target_rpm<=120);
    assert(approach_frame(50)==0 && approach_frame(50)==-30);
  }
  puts("PASS:03+supplement03 forward100..150/150..170/170..200,448px reaches200 through actual wheel callers,80/81+199/200 boundaries and200 wheel cap;04 unchanged70..100/120..150/150..170,24 near40..70/far120,reverse30/two-frame guard,deadband/freshness/wrap/fades;06 unchanged");
}
