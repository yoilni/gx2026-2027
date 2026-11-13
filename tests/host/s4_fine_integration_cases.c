/* Real pixel/yaw PID, fine actions,05 mailbox and supplement budget. */
static void fine_tick(uint32_t now)
{
  fake_now=now;MissionTask_UpdateVisionInput(now);MissionTask_CheckRunningHealth(now);
  (void)MissionTask_HandleLoadCheckRequest(now);MissionTask_HandleSupplementControl(now);
  MissionTask_HandleS4FineArrangeRequest(now);run_normal_s4(now);
  MissionTask_RunS4FineArrange(now);MissionTask_RunS6LoadRecheck(now);
}

static void fine_setup(void)
{
  setup(0x21,false);MaixCam_SetLoadRecheckEnabled(false);
  s4_green_e4_push_cycles=0;s4_fine_next_left=true;MissionTask_ResetVisionControllerState();
  //Legacy path cases start after first02; dedicated first02 tests rearm it.
  s4_initial_green02_used=true;s4_initial_green02_active=false;
  MissionTask_ResetS6ControllerState();MissionTask_EnterState(MISSION_STATE_S4_WAIT_ARRANGE_READY,100);
}

static void verify_s4_push_trial(void);

static void verify_casualty_skips_finish_turn(void)
{
  /* Real04/02 and42 entry, both42 sides, normal/timeout/E4/TX-failure completion.
     A previous non-red cargo report must not override this red capture. */
  for(unsigned kind=0;kind<2;++kind) for(unsigned side=0;side<2;++side)
    for(unsigned mode=0;mode<4;++mode) {
      fine_setup();s7_target_switch_enabled=true;
      s3_target_select_command=MAIXCAM_COMMAND_SELECT_RED;
      s6_carried_object_id=5;s4_fine_next_left=side==0;
      uint8_t request=kind==0?0x02:0x42,ack=kind==0?0x12:0x42;
      fake_now=100;rx(request);
      coords(4,0,50);
      fine_tick(100);
      if(kind==0) fine_tick(120);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK);
      uint32_t track=mission_snapshot.state_entry_tick;
      fake_now=track+199;coords(4,0,50);fine_tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && !tx_count);
      fine_tick(track+200);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN && !tx_count);
      bool left=s4_fine_turn_left;
      assert(left==(kind==0 || side==0));
      assert(s4_fine_turn_yaw_cdeg==MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+
          (left?1L:-1L)*ROBOT_YAW_LEFT_SIGN*1200L));
      uint32_t turn=mission_snapshot.state_entry_tick;
      mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;fine_tick(turn+20);
      uint32_t push=mission_snapshot.state_entry_tick;
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
      fine_tick(push+349);assert(mission_snapshot.left_target_rpm==170 && !tx_count);
      fine_tick(push+350);
      uint32_t counter=mission_snapshot.state_entry_tick;
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN);
      int32_t goal=s4_fine_turn_yaw_cdeg;
      fake_now=counter+20;rx(request);coords(6,200,300);
      if(mode==2) rx(0xE4);
      fine_tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && !tx_count);
      assert(mission_snapshot.state_entry_tick==counter);
      if(mode==1) {
        fine_tick(counter+1999);assert(!tx_count);
      } else mission_snapshot.yaw_cdeg=goal;
      if(mode==3) tx_ok=false;
      fine_tick(counter+(mode==1?2000:40));
      assert(mission_snapshot.state!=MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT);
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
      assert(tx_count==1 && tx_commands[0]==(mode==2?0x04:ack));
      assert(frame_lift==0 && !frame_raise_calls && !frame_partial_calls);
      if(mode==2) {
        assert(mission_snapshot.state==MISSION_STATE_S4_E4_TURN_AWAY);
        assert(s4_fine_next_left==(side==0));
      } else if(mode==3) assert(mission_snapshot.state==MISSION_STATE_FAULT);
      else {
        assert(mission_snapshot.state==(kind==0?MISSION_STATE_S4_WAIT_LEFT_TARGET:
            MISSION_STATE_S4_WAIT_ARRANGE_READY));
        assert(s4_fine_next_left==(kind==0?(side==0):!left));
      }
    }
  // A stale RED cargo report cannot remove a supply task's final LEFT20.
  fine_setup();s6_carried_object_id=4;
  MissionTask_EnterState(MISSION_STATE_S4_FINE42_COUNTER_TURN,100);
  s4_fine_turn_yaw_cdeg=mission_snapshot.yaw_cdeg;
  fine_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT && !tx_count);
  puts("PASS:red02/42 ROI200 then initial12deg and170/350;skip finalLEFT20 after counter30;42 side order unchanged;normal/2s timeout/latchedE4/no oldACK/TX failure,old cargo isolated;supplies retain7deg+LEFT20");
}

static void verify_fixed_forward_without_yaw(void)
{
  const int32_t errors[]={-18000,-1001,-500,0,500,1001,9000,17900};
  const uint8_t acks[]={0x12,0x42};
  for(unsigned a=0;a<2;++a) for(unsigned e=0;e<8;++e) {
    fine_setup();s4_fixed_motion_ack=acks[a];s4_fine_turn_left=a==0;
    MissionTask_EnterState(MISSION_STATE_S4_FINE42_FORWARD,100);
    s4_fine_forward_yaw_cdeg=35900;
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(35900L-errors[e]);
    s6_yaw_pid.update_tick=90;s6_yaw_pid.integral=123.0f;
    fake_now=120;coords(5,200,300);rx(a==0?0x02:0x42);fine_tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD && mission_snapshot.state_entry_tick==100);
    assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
    assert(mission_snapshot.vision_forward_rpm==170 && !mission_snapshot.vision_turn_rpm);
    assert(!mission_snapshot.safe_zone_fixed_heading_active);
    assert(s6_yaw_pid.update_tick==90 && s6_yaw_pid.integral==123.0f);
    fine_tick(449);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD && !tx_count);
    assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
    int32_t reached=mission_snapshot.yaw_cdeg;
    fine_tick(450);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && !tx_count);
    assert(s4_fine_turn_yaw_cdeg==MissionTask_WrapYaw(reached-(a==0?1L:-1L)*ROBOT_YAW_LEFT_SIGN*3000L));
  }
}

static void verify_fixed_push_speed_limits(void)
{
  const MissionState phases[]={MISSION_STATE_S4_FINE42_TURN,MISSION_STATE_S4_FINE42_COUNTER_TURN,
    MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT};
  const int32_t errors[]={500,1500,10000};
  for(unsigned p=0;p<3;++p) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned e=0;e<3;++e) {
      fine_setup();MissionTask_EnterState(phases[p],100);
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(10000L-sign*errors[e]);
      MissionTask_ResetS6ControllerState();
      assert(!MissionTask_RunS6TurnToHeading(120,10000L));
      int magnitude=abs(mission_snapshot.left_target_rpm);
      assert(magnitude>=(p==0?10:30) && magnitude<=(p==0?35:60));
      assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
      if(e==0) assert(magnitude==(p==0?13:30));
      if(e==2) assert(magnitude==(p==0?35:60));
      mission_snapshot.yaw_cdeg=10000L;
      assert(MissionTask_RunS6TurnToHeading(140,10000L));
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    }
  assert(ROBOT_S4_FINE42_FORWARD_RPM==170 && ROBOT_S4_FINE42_WHEEL_MAX_RPM==170);
  assert(ROBOT_S4_FINE42_FORWARD_MS==350 && ROBOT_S4_PUSH_TRIAL_FORWARD_MS==350);
  assert(ROBOT_S5_E5_FORWARD_MS==450);
  assert(ROBOT_S4_FINE42_TURN_CDEG==700 && ROBOT_S4_FINE42_COUNTER_TURN_CDEG==3000);
  assert(ROBOT_S4_FINE42_CASUALTY_TURN_CDEG==1200);
  assert(ROBOT_S5_E5_TURN_CDEG==1000 && ROBOT_S5_E5_COUNTER_TURN_CDEG==4500);
  assert(ROBOT_S4_FINE32_FORWARD_MS==300 && ROBOT_S4_FINE32_FORWARD_RPM==100);
  assert(ROBOT_S4_FINE42_TRACK_MS==200 && ROBOT_S4_PUSH_TRIAL_TRACK_MS==200);
  assert(ROBOT_S4_FINE42_FINISH_LEFT_CDEG==2000);
  assert(ROBOT_S4_FINE42_REVERSE_RPM==0 && ROBOT_S4_FINE42_REVERSE_MS==0);
  assert(ROBOT_S6_YAW_FORWARD_ENABLE_CDEG==1000 && ROBOT_S6_TURN_TOLERANCE_CDEG==400);
  // The initial7deg turn slows proportionally, not a constant30rpm.
  // Exercise both physical directions across wrapped headings with real PID.
  const int32_t near_errors[]={700,600,500,401,400};
  const int expected[]={18,15,13,10,0};
  for(int sign=-1;sign<=1;sign+=2) {
    fine_setup();MissionTask_EnterState(MISSION_STATE_S4_FINE42_TURN,100);
    for(unsigned e=0;e<5;++e) {
      mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(100L-sign*near_errors[e]);
      bool done=MissionTask_RunS6TurnToHeading(120+e*20,100);
      assert(done==(e==4));
      assert(abs(mission_snapshot.left_target_rpm)==expected[e]);
      assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
      if(e<4) assert((sign>0 && mission_snapshot.left_target_rpm<0) ||
          (sign<0 && mission_snapshot.left_target_rpm>0));
    }
  }
}

static void verify_02_roi_entry(void)
{
  MaixCam_Object object;
  uint8_t event;
  const uint8_t targets[]={0x21,0x11,0x51};
  const uint8_t ids[]={5,4,6};
  for(unsigned t=0;t<3;++t) {
    fine_setup();s7_target_switch_enabled=t!=0;s3_target_select_command=targets[t];
    s4_fine_next_left=false;
    fake_now=100;coords(ids[t],300,300);rx(0x02);
    assert(!MaixCam_GetObjectSnapshot(&object)); //Old image-center frame fenced at RX02.
    coords(ids[t],20,50);rx(0x02); //Queued duplicate must keep new ROI coordinates.
    fine_tick(100);assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_RIGHT_BLOCK);
    assert(MaixCam_GetObjectSnapshot(&object) && object.x_error_px==20);
    fine_tick(120);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK);
    assert(mission_snapshot.vision_target_valid && frame_lift==0 && s4_fine_turn_left);
    assert(mission_snapshot.vision_forward_rpm==57 && mission_snapshot.vision_turn_rpm>0);
    assert(mission_snapshot.left_target_rpm!=mission_snapshot.right_target_rpm);
    assert(!claw_open_calls); //74° backoff is only from05.
    uint32_t entry=mission_snapshot.state_entry_tick;
    fake_now=300;coords(ids[t],0,50);rx(0x02);fine_tick(fake_now);
    assert(mission_snapshot.state_entry_tick==entry && mission_snapshot.vision_target_valid);
    assert(s4_green_e4_push_cycles==(t==0?1U:0U));
    mission_snapshot.yaw_cdeg=35900;fine_tick(319);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && !tx_count);
    fine_tick(320);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
    assert(s4_fine_turn_yaw_cdeg==(t==1?1100:600) && s4_fine_turn_left && !s4_fine_next_left);
  }
  // New ROI half-frames after04/02 survive both task-entry steps.
  fine_setup();fake_now=100;coords(5,300,300);rx(0x02);
  const uint8_t frame[]={0xF1,0xF2,5,0,0,1,0,50,2,0x1F,0x2F};
  for(unsigned i=0;i<5;++i) byte(frame[i]);
  fine_tick(100);assert(!MaixCam_GetObjectSnapshot(&object));
  for(unsigned i=5;i<sizeof(frame);++i) byte(frame[i]);
  fine_tick(120);assert(mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm==57);
  // A missing/wrong-class target must not start a blind fixed push at200ms.
  const uint8_t commands[]={0x02,0x42};
  for(unsigned c=0;c<2;++c) for(unsigned wrong=0;wrong<2;++wrong) {
    fine_setup();fake_now=100;coords(5,0,200);rx(commands[c]);
    if(wrong) coords(4,0,50);
    fine_tick(100);if(c==0) fine_tick(120);
    uint32_t entry=mission_snapshot.state_entry_tick;
    fine_tick(entry+199);fine_tick(entry+200);fine_tick(entry+499);
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && !tx_count);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    fine_tick(entry+500);
    assert(mission_snapshot.state==MISSION_STATE_S4_E4_TURN_AWAY && tx_count==1 && tx_commands[0]==0x04);
  }
  // E4 between RX02 and tracking entry cancels without a successful12 reply.
  fine_setup();fake_now=100;rx(0x02);coords(5,0,50);fine_tick(100);
  rx(0xE4);assert(MaixCam_FineArrangeLossPending());fine_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_TURN_AWAY && tx_count==1 && tx_commands[0]==0x04);
  //02 during42 is NOT a duplicate02: retain it as a new deferred decision.
  fine_setup();fake_now=100;rx(0x42);coords(5,0,50);fine_tick(100);
  rx(0x02);assert(MaixCam_PeekEvent(&event) && event==0x02);
  assert(!MaixCam_GetObjectSnapshot(&object));
  puts("PASS:04/02 ROI XY200ms then left-first,green/red/51 filters and quota,no backoff;full/partial ROI preserved;02/42 no blind push,loss->E4,pending02 E4 latch,02 during42 deferred");
}

static void verify_initial_green02(void)
{
  assert(ROBOT_S4_INITIAL_GREEN02_FORWARD_MS==500U);
  assert(ROBOT_S4_INITIAL_GREEN02_SPIN_CDEG==28000L);
  assert(ROBOT_S4_INITIAL_GREEN02_SPIN_MAX_RPM==100.0f);
  const int32_t progress[]={0,9000,18000,27000,27900,27999,28000};
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned yaw_wrap=0;yaw_wrap<2;++yaw_wrap)
    for(unsigned loss=0;loss<2;++loss) for(unsigned prior=0;prior<2;++prior) {
      fine_setup();s4_initial_green02_used=false;s4_green_e4_push_cycles=prior;
      uint32_t start=wrap?UINT32_MAX-100U:100U;
      int32_t origin=yaw_wrap?35000L:1000L;
      mission_snapshot.yaw_cdeg=origin;
      fake_now=start;rx(0x02);coords(5,0,50);fine_tick(start);fine_tick(start+20U);
      assert(s4_initial_green02_used && s4_initial_green02_active);
      assert(s4_green_e4_push_cycles==prior+1U && frame_lift==0);
      uint32_t forward=start+20U+ROBOT_S4_PUSH_TRIAL_TRACK_MS;
      fake_now=forward-1U;rx(0x02);coords(5,0,50);fine_tick(fake_now);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK);
      fine_tick(forward);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD && !tx_count);
      assert(mission_snapshot.yaw_cdeg==origin); //No initial7-degree turn.
      fake_now=forward+1U;rx(0x02);coords(5,300,300);fine_tick(fake_now);
      assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
      assert(mission_snapshot.state_entry_tick==forward && !mission_snapshot.vision_turn_rpm);
      fine_tick(forward+499U);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD && !tx_count);
      uint32_t spin=forward+500U;
      fine_tick(spin);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN);
      assert(!s4_initial_green02_progress_cdeg && !tx_count);
      //No2s soft completion, no coordinate interruption, duplicate02 cannot restart.
      for(unsigned p=0;p<sizeof(progress)/sizeof(progress[0]);++p) {
        fake_now=spin+2001U+p*20U;
        mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(origin+ROBOT_YAW_LEFT_SIGN*progress[p]);
        rx(0x02);coords(5,0,0);
        if(loss && p==0) rx(0xE4);
        fine_tick(fake_now);
        if(p+1U<sizeof(progress)/sizeof(progress[0])) {
          assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && !tx_count);
          assert(mission_snapshot.state_entry_tick==spin);
          assert(s4_initial_green02_progress_cdeg==progress[p]);
          assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm);
          assert(mission_snapshot.left_target_rpm*ROBOT_YAW_LEFT_SIGN<0);
          assert(abs(mission_snapshot.left_target_rpm)<=100);
          if(p==0) assert(abs(mission_snapshot.left_target_rpm)==100);
          if(progress[p]==27900) assert(abs(mission_snapshot.left_target_rpm)==10);
        }
      }
      assert(s4_initial_green02_used && !s4_initial_green02_active);
      assert(s4_green_e4_push_cycles==prior+1U && tx_count==1);
      assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
      assert(frame_lift==0 && !frame_raise_calls && !frame_partial_calls);
      assert(tx_commands[0]==(loss?0x04:0x12)); //Latched E4 suppresses old12.
      assert(mission_snapshot.state==(loss?MISSION_STATE_S4_E4_TURN_AWAY:MISSION_STATE_S4_WAIT_LEFT_TARGET));
      //A later02 in the same first-green mission returns to the original7-degree path.
      uint32_t next=fake_now+20U;
      fake_now=next;coords(5,0,50);
      MissionTask_EnterState(MISSION_STATE_S4_TRACK_RIGHT_BLOCK,next);
      MissionTask_StartS4PushTrial(next,true);
      assert(!s4_initial_green02_active && s4_initial_green02_used);
      tx_count=0;fake_now=next+199U;coords(5,0,50);fine_tick(fake_now);
      fine_tick(next+200U);
      assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
    }
  //No IMU progress is a failed280-degree move, not a successful12 handshake.
  fine_setup();s4_initial_green02_used=false;fake_now=100;rx(0x02);coords(5,0,50);
  fine_tick(100);fine_tick(120);fine_tick(320);fine_tick(820);
  fine_tick(820+ROBOT_S4_INITIAL_GREEN02_SPIN_TIMEOUT_MS-1U);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && !tx_count);
  fine_tick(820+ROBOT_S4_INITIAL_GREEN02_SPIN_TIMEOUT_MS);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count);
  assert(!mission_snapshot.left_target_rpm && !s4_initial_green02_active);
  //05 and actuator/health failures retain their preemption authority.
  for(unsigned phase=0;phase<2;++phase) {
    fine_setup();s4_initial_green02_used=false;fake_now=100;rx(0x02);coords(5,0,50);
    fine_tick(100);fine_tick(120);fine_tick(320);
    uint32_t at=phase?840U:340U;
    if(phase) fine_tick(820);
    fake_now=at;rx(0x05);fine_tick(at);
    assert(!s4_initial_green02_active && s4_initial_green02_used && !tx_count);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  }
  fine_setup();s4_initial_green02_used=false;frame_lower_ok=false;
  MissionTask_StartS4PushTrial(100,true);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !s4_initial_green02_used);
  //An unused first02 flag must not enable the override for other tasks or42.
  const uint8_t selectors[]={0x21,0x11,0x31,0x51};
  for(unsigned t=0;t<4;++t) {
    fine_setup();s4_initial_green02_used=false;s7_target_switch_enabled=true;
    s3_target_select_command=selectors[t];MissionTask_StartS4PushTrial(100,true);
    assert(!s4_initial_green02_active && !s4_initial_green02_used);
  }
  fine_setup();s4_initial_green02_used=false;fake_now=100;rx(0x42);coords(5,0,50);
  fine_tick(100);
  assert(!s4_initial_green02_active && !s4_initial_green02_used);
  puts("PASS:first-green FIRST02 only:ROI200,NO L7,F170/500,gyro PID CCW280 cap100 taper10,stopACK12;signed wrap/no2s shortcut/no XY interrupt/duplicate reset,quota independent,latchedE4 suppresses12,05/timeout/actuator safety;later02 unchanged");
}

static void verify_fine_arrange_integration(void)
{
  verify_initial_green02();
  verify_fixed_push_speed_limits();
  verify_fixed_forward_without_yaw();
  verify_s4_push_trial();
  verify_02_roi_entry();
  verify_casualty_skips_finish_turn();
  approach_setup(MISSION_STATE_S4_FINE42_TRACK,0);
  assert(approach_frame(-50)==57);
  assert(MissionTask_CommandS4VisionTracking(fake_now,NULL));
  assert(mission_snapshot.left_target_rpm==57 && mission_snapshot.right_target_rpm==57);
  assert(approach_frame(-448)==120);
  assert(MissionTask_CommandS4VisionTracking(fake_now,NULL));
  assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
  mission_snapshot.vision_x_error_px=300;
  assert(MissionTask_CommandS4VisionTracking(fake_now,NULL));
  assert(mission_snapshot.vision_turn_rpm<=25 && mission_snapshot.left_target_rpm<=120);
  fake_now+=375;mission_snapshot.target_age_ms=375;
  assert(MissionTask_CommandS4VisionTracking(fake_now,NULL));
  assert(mission_snapshot.vision_forward_rpm==60 && mission_snapshot.vision_turn_rpm<=12);
  assert(mission_snapshot.left_target_rpm<=72);

  // Actual command burst+ROI coordinates starts the PID with frame DOWN;
  // duplicate42 never erases fresh coordinates or shifts its200ms deadline.
  fine_setup();fake_now=100;rx(0x42);coords(5,0,50);fine_tick(100);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && frame_lift==0);
  assert(mission_snapshot.left_target_rpm==57 && !frame_raise_calls && !frame_partial_calls);
  fake_now=280;coords(5,0,50);rx(0x42);fine_tick(280);
  assert(mission_snapshot.vision_target_valid && mission_snapshot.state_entry_tick==100);
  mission_snapshot.yaw_cdeg=35900;fine_tick(299);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && !tx_count);
  fine_tick(300);
  assert(s4_fine_turn_yaw_cdeg==600 && mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
  fine_tick(320);
  assert(mission_snapshot.left_target_rpm==-18 && mission_snapshot.right_target_rpm==18);
  fine_tick(2299);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
  fine_tick(2300);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
  assert(s4_fine_forward_yaw_cdeg==35900); //Reached yaw is diagnostic-only, not a drive PID goal.
  fine_tick(2649);assert(mission_snapshot.left_target_rpm==170 && !tx_count);
  fine_tick(2650);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && s4_fine_turn_yaw_cdeg==32900);
  fine_tick(2670);
  assert(mission_snapshot.left_target_rpm>30 && mission_snapshot.left_target_rpm<=60);
  assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
  fine_tick(4649);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN);
  fine_tick(4650); //Each turn, including the new left20, gets its own2s deadline.
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT && !tx_count);
  assert(s4_fine_turn_yaw_cdeg==1900 && s4_fine_next_left);
  fine_tick(4670);
  assert(mission_snapshot.left_target_rpm<=-30 && mission_snapshot.left_target_rpm>=-60);
  fine_tick(6649);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT && !tx_count);
  fine_tick(6650);
  assert(tx_count==1 && tx_commands[0]==0x42 && !s4_fine_next_left);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(frame_lift==0 && !frame_raise_calls && !frame_partial_calls);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY);
  // Vision is responsible for max3; it may choose02 OR12 after a fine ACK.
  rx(0x12);fine_tick(7470);assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_LEFT_BLOCK);

  // Inclusive turn arrival has no extra wait; forward always uses equal
  // wheel targets, then left20 ends with an immediate stopped ACK.
  fine_setup();MissionTask_EnterState(MISSION_STATE_S4_FINE42_TURN,100);
  s4_fine_turn_yaw_cdeg=1000;mission_snapshot.yaw_cdeg=1000;
  fine_tick(120);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
  mission_snapshot.yaw_cdeg=500;fine_tick(140);
  assert(mission_snapshot.left_target_rpm==170 && mission_snapshot.right_target_rpm==170);
  fine_tick(469);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
  fine_tick(470);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN);
  assert(s4_fine_turn_yaw_cdeg==33500);
  mission_snapshot.yaw_cdeg=33500;fine_tick(490);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT && s4_fine_turn_yaw_cdeg==35500);
  mission_snapshot.yaw_cdeg=35500;fine_tick(510);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY);
  assert(tx_count==1 && tx_commands[0]==0x42);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  // The reserved reverse ID also immediately ACKs stopped, even at tick wrap.
  fine_setup();MissionTask_EnterState(MISSION_STATE_S4_FINE42_REVERSE,100);
  fine_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY && tx_count==1);
  assert(tx_commands[0]==0x42 && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  fine_setup();uint32_t reverse_start=UINT32_MAX-150U;
  MissionTask_EnterState(MISSION_STATE_S4_FINE42_REVERSE,reverse_start);
  fine_tick(reverse_start+160U);assert(tx_count==1 && tx_commands[0]==0x42);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);

  //05 preempts every fine leg and cancels the old ACK/E4, retaining cargo.
  for(MissionState s=MISSION_STATE_S4_FINE32_TRACK;s<=MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT;++s) {
    setup(0x51,true);MaixCam_SetLoadRecheckEnabled(false);MissionTask_EnterState(s,100);
    fake_now=120;rx(0xE4);rx(0x05);coords(5,0,0);rx(0x06);fine_tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && near_calls==1);
    assert(!mission_snapshot.left_target_rpm && !maixcam_fine_arrange_active && !tx_count);
    fine_tick(419);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
    fine_tick(420);assert(s6_carried_object_id==5 && mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
    assert(!tx_count); //No stale32/42 acknowledgment after the load review.
  }

  //The supplement deadline also preempts42 fixed motion; no local AUTO06.
  setup(0x51,true);supplement_budget_started=supplement_capture_active=true;
  supplement_first_request_tick=100;supplement_capture_count=1;
  MaixCam_SetLoadRecheckEnabled(false);
  uint32_t deadline=supplement_first_request_tick+ROBOT_SUPPLEMENT_TOTAL_MS;
  MissionTask_EnterState(MISSION_STATE_S4_FINE42_COUNTER_TURN,deadline-200U);
  fine_tick(deadline);
  assert(supplement_budget_exhausted && tx_count==1 && tx_commands[0]==0x05);
  assert(mission_snapshot.state==MISSION_STATE_SUPPLEMENT_WAIT_RECHECK && !mission_snapshot.left_target_rpm);

  // Normal timers work across tick wrap. A command invalidates the old
  // picture-center frame, but keeps a complete/partial new ROI frame.
  fine_setup();uint32_t start=UINT32_MAX-200U;fake_now=start;
  rx(0x32);coords(5,0,50);fine_tick(start);
  fake_now=start+280U;coords(5,300,300);fine_tick(fake_now);
  assert(mission_snapshot.left_target_rpm==100 && mission_snapshot.right_target_rpm==100);
  fine_tick(start+299U);assert(!tx_count);fine_tick(start+300U);
  assert(tx_count==1 && tx_commands[0]==0x32 && frame_lift==0);

  fine_setup();coords(5,0,300);rx(0x32);
  const uint8_t partial[]={0xF1,0xF2,5,0,0,1,0,50,2,0x1F,0x2F};
  for(unsigned i=0;i<5;++i) byte(partial[i]);
  fine_tick(100);
  assert(!mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm==100);
  for(unsigned i=5;i<sizeof(partial);++i) byte(partial[i]);
  fine_tick(120);
  assert(mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm==100);
  assert(mission_snapshot.right_target_rpm==100 && !mission_snapshot.vision_turn_rpm);
  puts("PASS:32 fixed100/300 ignores XY/no target or yaw PID;42 center200ms PID;initial7deg dedicated PID0.025/0/0,18->15->13->10rpm taper,cap35,arrival4deg;30/20 turns30..60;02/42 forward170/350 equal wheels NO YAW PID or10deg gate even at180deg error,supplies LEFT20 immediate ACK NO REVERSE;independent2s deadlines,05/06/budget preemption,wrap/partial frames");
}

/*04/02 has200ms ROI entry then a350ms fixed push;12 stays handshake-only. */
static uint32_t trial_route_to_turn_end(uint32_t start,bool loss)
{
  fine_setup();s4_fine_next_left=false;frame_lift=20;
  fake_now=start;rx(0x02);coords(5,0,50);fine_tick(start);
  assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_RIGHT_BLOCK);
  fine_tick(start+20U);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && frame_lift==0);
  assert(mission_snapshot.left_target_rpm==57 && mission_snapshot.right_target_rpm==57);
  assert(s4_fixed_motion_ack==0x12 && s4_fine_turn_left);
  assert(!s4_fine_next_left);
  uint32_t turn=start+20U+ROBOT_S4_PUSH_TRIAL_TRACK_MS;
  fake_now=start+30U;rx(0x02);fine_tick(fake_now);
  assert(mission_snapshot.state_entry_tick==start+20U && !tx_count);
  fake_now=turn-1U;coords(5,0,50);fine_tick(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && !tx_count);
  fine_tick(turn);assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
  assert(s4_fine_turn_yaw_cdeg==30700);
  uint32_t push=turn+20U;
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;fine_tick(push);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
  if(loss) { rx(0xE4);assert(MaixCam_FineArrangeLossPending()); }
  fake_now=push+20U;rx(0x02);fine_tick(fake_now);
  assert(mission_snapshot.state_entry_tick==push && mission_snapshot.left_target_rpm==170);
  assert(ROBOT_S4_PUSH_TRIAL_FORWARD_MS==350U);
  uint32_t counter=push+ROBOT_S4_PUSH_TRIAL_FORWARD_MS;
  fine_tick(counter-1U);assert(mission_snapshot.left_target_rpm==170 && !tx_count);
  fine_tick(counter);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN);
  assert(s4_fine_turn_yaw_cdeg==27700);
  fake_now=counter+20U;rx(0x02);fine_tick(fake_now);
  assert(mission_snapshot.state_entry_tick==counter && !tx_count);
  assert(mission_snapshot.left_target_rpm>30);
  assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;
  uint32_t finish=counter+40U;fine_tick(finish);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT && !tx_count);
  assert(s4_fine_turn_yaw_cdeg==29700 && !s4_fine_next_left);
  fake_now=finish+20U;rx(0x02);fine_tick(fake_now);
  assert(mission_snapshot.state_entry_tick==finish && !tx_count);
  assert(mission_snapshot.left_target_rpm<=-30 && mission_snapshot.left_target_rpm>=-60);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;
  assert(frame_lift==0 && !frame_partial_calls && !frame_raise_calls);
  return finish+40U;
}

static void verify_s4_push_trial(void)
{
  uint8_t event;
  uint32_t end=trial_route_to_turn_end(100,false);
  inject_push_reply_on_tx=0x12;fine_tick(end);
  assert(tx_count==1 && tx_commands[0]==0x12);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_LEFT_TARGET);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(!maixcam_fine_arrange_active && !s4_fine_next_left);
  assert(MaixCam_PeekEvent(&event) && event==0x12);
  unsigned lowers=frame_lower_calls;
  fine_tick(end+20U);assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_LEFT_BLOCK);
  inject_push_reply_on_tx=0x24;fine_tick(end+40U);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_FINAL_READY);
  assert(tx_count==2 && tx_commands[1]==0x22 && frame_lower_calls==lowers);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(MaixCam_PeekEvent(&event) && event==0x24);
  fine_tick(end+60U);assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME);

  // Deferred E4 waits for left20 to finish, then cancels oldTX12.
  end=trial_route_to_turn_end(100,true);fine_tick(end);
  assert(tx_count==1 && tx_commands[0]==0x04 && !s4_fine_next_left);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_TURN_AWAY);
  end=trial_route_to_turn_end(100,false);tx_ok=false;fine_tick(end);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  assert(tx_count==1 && tx_commands[0]==0x12);

  // RX12 from both normal and fine-decision waits only sends22. No wheel,
  // frame or direction changes, no coordinate wait, no300ms reverse delay.
  for(unsigned i=0;i<2;++i) {
    fine_setup();s4_fine_next_left=i!=0;frame_lift=20;
    if(i==0) MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET,100);
    else s4_fine_waiting_decision=true;
    fake_now=100;rx(0x12);fine_tick(100);
    assert(mission_snapshot.state==MISSION_STATE_S4_TRACK_LEFT_BLOCK && !tx_count);
    rx(0x12);coords(5,0,50);inject_push_reply_on_tx=0x24;fine_tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_FINAL_READY);
    assert(tx_count==1 && tx_commands[0]==0x22 && frame_lift==20);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(!frame_raise_calls && !frame_lower_calls && !frame_partial_calls);
    assert(mission_snapshot.yaw_cdeg==30000 && s4_fine_next_left==(i!=0));
    assert(!mission_snapshot.vision_target_valid && !maixcam_fine_arrange_active);
    assert(MaixCam_PeekEvent(&event) && event==0x24);
  }
  // Duplicates while waiting never restart motion or flood22 acknowledgments.
  fine_setup();MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET,100);
  rx(0x12);fine_tick(100);fine_tick(120);rx(0x12);fine_tick(140);
  assert(tx_count==1 && tx_commands[0]==0x22);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_FINAL_READY);
  assert(!frame_lower_calls && !mission_snapshot.left_target_rpm);
  // A new E4 arriving at the skipped-action boundary must remain for post22.
  fine_setup();MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET,100);
  rx(0x12);fine_tick(100);rx(0xE4);fine_tick(120);
  assert(tx_count==1 && tx_commands[0]==0x22);
  assert(MaixCam_PeekEvent(&event) && event==0xE4);
  // Failed22 transmission and health faults stop safely without a route.
  fine_setup();MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET,100);
  rx(0x12);fine_tick(100);tx_ok=false;fine_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm);
  assert(tx_count==1 && tx_commands[0]==0x22 && !frame_lower_calls);
  fine_setup();MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET,100);
  rx(0x12);fine_tick(100);health_faults=MISSION_FAULT_IMU;fine_tick(120);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count && !frame_lower_calls);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  for(unsigned i=0;i<2;++i) {
    fine_setup();MissionTask_EnterState(MISSION_STATE_S4_WAIT_LEFT_TARGET,100);
    rx(0x12);fine_tick(100);rx(i==0?0x24:0x05);fine_tick(120);
    assert(!tx_count && !mission_snapshot.left_target_rpm);
    assert(mission_snapshot.state==(i==0?MISSION_STATE_S4_RAISE_FRAME:MISSION_STATE_S6_RECHECK_WAIT_RESULT));
  }
  //24 keeps its original priority over02 in every fixed leg.
  const MissionState legs[]={MISSION_STATE_S4_FINE42_TURN,MISSION_STATE_S4_FINE42_FORWARD,
    MISSION_STATE_S4_FINE42_COUNTER_TURN,MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT,
    MISSION_STATE_S4_FINE42_REVERSE};
  for(unsigned i=0;i<5;++i) {
    fine_setup();MissionTask_StartS4PushTrial(100,true);
    MissionTask_EnterState(legs[i],100);fake_now=120;rx(0x24);fine_tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S4_RAISE_FRAME && !claw_open_calls && claw_angle==ROBOT_CLAW_CLOSED_DEG);
    assert(!frame_partial_calls);
    assert(!tx_count && !maixcam_fine_arrange_active && !mission_snapshot.left_target_rpm);
    fine_setup();MissionTask_StartS4PushTrial(100,true);
    MissionTask_EnterState(legs[i],100);fake_now=120;rx(0x05);fine_tick(120);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && near_calls==1);
    assert(!tx_count && !maixcam_fine_arrange_active && !mission_snapshot.left_target_rpm);
  }
  // Timer wrap does not shorten02's200ms tracking/350ms push or add retreat.
  end=trial_route_to_turn_end(UINT32_MAX-450U,false);fine_tick(end);
  assert(tx_count==1 && tx_commands[0]==0x12 && mission_snapshot.state==MISSION_STATE_S4_WAIT_LEFT_TARGET);
  fake_now=end+20U;rx(0x12);fine_tick(fake_now);fine_tick(end+40U);
  assert(tx_count==2 && tx_commands[1]==0x22 && mission_snapshot.state==MISSION_STATE_S4_WAIT_FINAL_READY);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  puts("PASS:02 ROI200 L7 push170/350 R30 LEFT20 immediateTX12 NO REVERSE;12 handshake-only >TX22,normal/fine entry,no wheel/frame motion/no42 direction change;ACK bursts,all-leg duplicates/E4 retention,24/05 preemption,TX safety/tick wrap");
}
