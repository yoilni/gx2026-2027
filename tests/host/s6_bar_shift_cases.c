/* Uses the real UART parser, yaw PID and46 mission functions in the06 suite. */
static void bar_byte(uint8_t b) {
  ++rx_bytes;maixcam_rx_byte=b;MaixCam_UART_RxCpltCallback(maixcam_uart);
}
static uint32_t bar_route_to_ack(uint8_t zone,uint8_t team,uint8_t cargo,int32_t zero) {
  setup(zone,team,cargo,zero,26000);
  int32_t start=mission_snapshot.yaw_cdeg;
  int32_t safe=mission_snapshot.safe_zone_yaw_target_cdeg;
  fake_now=140;rx(0x46);rx(0x26);tick(140,false);
  assert(start!=safe && mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && s6_bar_initial_align_pending);
  assert(!s6_align26_pending && s6_bar_reverse_yaw_cdeg==safe && s6_carried_object_id==cargo && !s6_bar_shift_completed);
  assert(s6_bar_side_yaw_cdeg==MissionTask_WrapYaw(safe+(cargo==4?-1L:1L)*ROBOT_YAW_LEFT_SIGN*9000L));
  assert(!mission_snapshot.vision_forward_rpm && mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm);
  assert(!tx_calls && maixcam_bar_shift_active);
  fake_now=160;rx(0x46);rx(0x16);coords();rx(0x26);tick(160,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && mission_snapshot.state_entry_tick==140);
  assert(s6_bar_initial_align_pending && !s6_bar_shift_completed);
  assert(!mission_snapshot.vision_target_valid && !MaixCam_TakeSafeZoneAlignRequest());
  mission_snapshot.yaw_cdeg=safe;tick(180,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_REVERSE && mission_snapshot.state_entry_tick==180);
  assert(!s6_bar_initial_align_pending && !s6_bar_shift_completed && !tx_calls);
  fake_now=200;rx(0x46);coords();rx(0x26);tick(200,false);
  assert(mission_snapshot.left_target_rpm==-150 && mission_snapshot.right_target_rpm==-150);
  assert(mission_snapshot.state_entry_tick==180 && !mission_snapshot.vision_target_valid);
  tick(979,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_REVERSE);
  tick(980,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SIDE);
  tick(1000,false);
  assert(mission_snapshot.left_target_rpm==(cargo==4?40:-40));
  assert(mission_snapshot.right_target_rpm==-mission_snapshot.left_target_rpm);
  mission_snapshot.yaw_cdeg=s6_bar_side_yaw_cdeg;tick(1020,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD && mission_snapshot.state_entry_tick==1020);
  tick(1040,false);assert(mission_snapshot.left_target_rpm==150 && mission_snapshot.right_target_rpm==150);
  fake_now=1220;rx(0x46);rx(0x26);tick(1220,true);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD && mission_snapshot.state_entry_tick==1020);
  assert(!tx_calls && !mission_snapshot.vision_target_valid && !s6_align26_pending);
  tick(1419,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD);
  tick(1420,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD);
  tick(1619,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD);
  tick(1620,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && !s6_bar_initial_align_pending);
  mission_snapshot.yaw_cdeg=safe;tick(1640,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_WAIT_ACK && s6_bar_shift_completed && !tx_calls);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  return 1660;
}
static void verify_s6_bar_tick_handoff(void) {
  const uint32_t delays[]={0,1,2,20,80};
  const unsigned responses[]={0,5,6,7,3};
  for(unsigned d=0;d<sizeof(delays)/sizeof(delays[0]);++d) {
    for(unsigned r=0;r<sizeof(responses)/sizeof(responses[0]);++r) {
      uint32_t cycle=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
      uint32_t resumed=cycle+delays[d];
      inject46_tx_delay_ms=delays[d];inject46_tx_response=responses[r];
      tick(cycle,false);
      assert(!mission_snapshot.fault_flags && mission_snapshot.state!=MISSION_STATE_FAULT);
      assert(tx_calls==1 && last_tx_command==0x46);
      if(responses[r]==3) {
        assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
        assert(!mission_snapshot.left_target_rpm && maixcam_load_recheck_enabled);
        continue;
      }
      assert(s6_approach_start_tick==resumed);
      assert(s6_pre_reposition_track_tick==resumed);
      assert(!maixcam_bar_shift_active);
      if(responses[r]==6) {
        assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
        assert(mission_snapshot.state_entry_tick==resumed);
        assert(!MissionTask_HeadingTurnExpired(resumed,s6_push_yaw_target_cdeg));
        continue;
      }
      assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
      if(responses[r]!=0) {
        assert(mission_snapshot.vision_target_valid && mission_snapshot.target_id==1);
        assert(mission_snapshot.target_age_ms==0);
        assert(mission_snapshot.vision_y_error_px==-128);
        assert(mission_snapshot.left_target_rpm==60 && mission_snapshot.right_target_rpm==60);
      } else {
        assert(!mission_snapshot.vision_target_valid && !mission_snapshot.left_target_rpm);
        assert(s6_missing_since==resumed);
        tick(resumed+999U,false);
        assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !recovery_calls);
        tick(resumed+1000U,false);
        assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && recovery_calls==1);
        assert(!mission_snapshot.fault_flags);
      }
    }
  }
  // Even a stale external caller must not mistake a future start epoch for
  // fifteen seconds elapsed. Defer that one call without clearing RX events.
  uint32_t cycle=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
  fake_now=cycle+1U;MissionTask_RunS6BarShift(cycle);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  rx(0x16);MissionTask_RunS6TrackSafeZone(cycle);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !mission_snapshot.fault_flags);
  uint8_t event;
  assert(MaixCam_PeekEvent(&event) && event==0x16);

  // TX crossing uint32 wrap is an ordinary short interval, not a timeout.
  for(unsigned response=0;response<2;++response) {
    (void)bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
    cycle=UINT32_MAX-2U;mission_snapshot.state_entry_tick=cycle-20U;
    inject46_tx_delay_ms=5U;inject46_tx_response=response?5:0;
    tick(cycle,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !mission_snapshot.fault_flags);
    assert(s6_approach_start_tick==2U && s6_pre_reposition_track_tick==2U);
    if(response) assert(mission_snapshot.target_id==1 && mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm==60);
    else {
      tick(1001U,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
      tick(1002U,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START && recovery_calls==1);
    }
  }
  // A failed/slow ACK stays stopped; a successful retry resumes normally.
  cycle=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
  fail46_tx=true;inject46_tx_delay_ms=20;tick(cycle,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_WAIT_ACK && !mission_snapshot.fault_flags);
  fail46_tx=false;inject46_tx_delay_ms=20;inject46_tx_response=5;
  fake_now=cycle+500U;rx(0x46);tick(fake_now,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !mission_snapshot.fault_flags);
  assert(s6_approach_start_tick==cycle+520U && tx_calls==2 && mission_snapshot.left_target_rpm==60);
  puts("PASS:46 TX tick advances0/1/2/20/80ms and wraps;ID1/-128 moves,ID1+16 survives,26/05 priority,no false0x200 FAULT,loss999/1000ms,stale-caller guard and slow ACK retry");
}

static void verify_s6_bar_shift(void) {
  verify_s6_bar_tick_handoff();
  assert(MAIXCAM_EVENT_SAFE_ZONE_BAR_SHIFT==0x46 && MAIXCAM_COMMAND_BAR_SHIFT_DONE_ACK==0x46);
  assert(ROBOT_S6_BAR_REVERSE_RPM==150 && ROBOT_S6_BAR_REVERSE_MS==800);
  assert(ROBOT_S6_BAR_FORWARD_RPM==150 && ROBOT_S6_BAR_FORWARD_MS==600);
  assert(ROBOT_S6_BAR_WHEEL_MAX_RPM==160 && ROBOT_S6_BAR_TURN_MAX_RPM==40);
  assert(MISSION_STATE_SUPPLEMENT_E3_SPIN_CCW==98 && MISSION_STATE_S6_BAR_REVERSE==99);
  const uint8_t teams[]={ROBOT_TEAM_RED,ROBOT_TEAM_BLUE},cargos[]={3,4,5,6};
  const int32_t zeroes[]={0,35000};
  for(uint8_t zone=1;zone<=4;++zone) for(unsigned t=0;t<2;++t)
    for(unsigned c=0;c<4;++c) for(unsigned z=0;z<2;++z) {
      uint32_t ack=bar_route_to_ack(zone,teams[t],cargos[c],zeroes[z]);
      tick(ack,false);
      assert(tx_calls==1 && last_tx_command==0x46 && mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
      assert(s6_bar_shift_completed && !maixcam_bar_shift_active && !mission_snapshot.vision_target_valid);
      assert(!mission_snapshot.left_target_rpm && s6_carried_object_id==cargos[c]);
      fake_now=ack+20;rx(0x46);tick(fake_now,true);
      assert(tx_calls==2 && mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && mission_snapshot.vision_target_valid);
      // No new16 is required. A fresh26 now performs the original delivery.
      fake_now=ack+40;rx(0x26);tick(fake_now,false);
      assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
    }
  // Immediate postACK frames arriving before UART transmit returns survive.
  for(unsigned response=1;response<=4;++response) {
    uint32_t ack=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
    inject46_tx_response=response;tick(ack,false);
    if(response==1) assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && mission_snapshot.vision_target_valid);
    if(response==2) assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
    if(response==3) {
      assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm);
      tick(ack+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS,false);
      assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE ||
          mission_snapshot.state==MISSION_STATE_S6_RECOVERY_START);
    }
    if(response==4) {
      tick(ack+20,false);assert(tx_calls==2 && mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
    }
  }
  // A26 half-frame from inside46 cannot become a new postACK delivery.
  uint32_t ack=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
  bar_byte(0xF1);bar_byte(0xF2);bar_byte(0x26);bar_byte(0x1F);tick(ack,false);bar_byte(0x2F);
  tick(ack+20,false);assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !s6_align26_pending);
  fake_now=ack+40;rx(0x26);tick(fake_now,false);assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
  // A partial46 is not thrown away together with old motion coordinates.
  ack=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);
  bar_byte(0xF1);bar_byte(0xF2);bar_byte(0x46);bar_byte(0x1F);tick(ack,false);bar_byte(0x2F);
  tick(ack+20,false);assert(tx_calls==2 && mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  // TX failure stays stopped waiting for46. A retry never repeats the route.
  ack=bar_route_to_ack(3,ROBOT_TEAM_BLUE,4,35000);fail46_tx=true;tick(ack,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_WAIT_ACK && s6_bar_shift_completed && maixcam_bar_shift_active);
  tick(ack+100,false);assert(tx_calls==1 && !mission_snapshot.left_target_rpm);
  fake_now=ack+500;rx(0x46);tick(fake_now,true);assert(tx_calls==2 && mission_snapshot.state==MISSION_STATE_S6_BAR_WAIT_ACK);
  fail46_tx=false;fake_now=ack+1000;rx(0x46);tick(fake_now,false);
  assert(tx_calls==3 && mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && !maixcam_bar_shift_active);
  // Delayed26 during any action phase is ignored; repeated46 has no restart.
  for(unsigned phase=MISSION_STATE_S6_BAR_REVERSE;phase<=MISSION_STATE_S6_BAR_WAIT_ACK;++phase) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,26000);
    s6_bar_reverse_yaw_cdeg=mission_snapshot.yaw_cdeg;
    s6_bar_side_yaw_cdeg=MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+9000);
    s6_bar_shift_completed=phase==MISSION_STATE_S6_BAR_WAIT_ACK;
    MissionTask_EnterState((MissionState)phase,140);
    if(phase==MISSION_STATE_S6_BAR_FORWARD) mission_snapshot.yaw_cdeg=s6_bar_side_yaw_cdeg;
    fake_now=160;coords();rx(0x26);tick(160,false);
    assert(mission_snapshot.state==(MissionState)phase && !s6_align26_pending && !mission_snapshot.vision_target_valid);
    if(phase!=MISSION_STATE_S6_BAR_WAIT_ACK) {
      fake_now=180;rx(0x46);tick(180,false);
      assert(mission_snapshot.state==(MissionState)phase && mission_snapshot.state_entry_tick==140 && tx_calls==0);
    }
    // A new final05 is allowed to pause46, with its cargo report retained.
    fake_now=200;rx(0x05);cargo_report(5);rx(0x06);tick(200,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !mission_snapshot.left_target_rpm);
    tick(200+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE && !s6_bar_shift_completed);
  }
  // A05 interrupted46 can have a new cargo report already partially parsed.
  setup(3,ROBOT_TEAM_BLUE,4,35000,26000);fake_now=140;rx(0x46);tick(140,false);
  fake_now=160;rx(0x05);bar_byte(0xF1);bar_byte(0xF2);bar_byte(5);bar_byte(0);
  tick(160,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT);
  const uint8_t cargo_tail[]={0,1,0,0,1,0x1F,0x2F};
  for(unsigned i=0;i<sizeof(cargo_tail);++i) bar_byte(cargo_tail[i]);
  rx(0x06);tick(160+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS,false);
  assert(s6_carried_object_id==5 && mission_snapshot.state==MISSION_STATE_S6_SEARCH_SAFE_ZONE);
  // Moving corrections use at most10rpm, without a stationary turn floor.
  setup(3,ROBOT_TEAM_BLUE,5,35000,26000);fake_now=140;rx(0x46);tick(140,false);
  mission_snapshot.yaw_cdeg=s6_bar_reverse_yaw_cdeg;tick(160,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_REVERSE && !s6_bar_initial_align_pending);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(s6_bar_reverse_yaw_cdeg+500L);tick(180,false);
  assert(mission_snapshot.vision_forward_rpm==-150 && abs(mission_snapshot.vision_turn_rpm)<=10);
  assert(mission_snapshot.left_target_rpm>=-160 && mission_snapshot.left_target_rpm<=-140);
  assert(mission_snapshot.right_target_rpm>=-160 && mission_snapshot.right_target_rpm<=-140);
  MissionTask_EnterState(MISSION_STATE_S6_BAR_FORWARD,200);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(s6_bar_side_yaw_cdeg-500L);tick(220,false);
  assert(mission_snapshot.vision_forward_rpm==150 && abs(mission_snapshot.vision_turn_rpm)<=10);
  assert(mission_snapshot.left_target_rpm>=140 && mission_snapshot.left_target_rpm<=160);
  assert(mission_snapshot.right_target_rpm>=140 && mission_snapshot.right_target_rpm<=160);
  //46 is independently retained when other events arrive after it.
  const MissionState approach[]={MISSION_STATE_S6_SEARCH_SAFE_ZONE,MISSION_STATE_S6_TRACK_SAFE_ZONE,
    MISSION_STATE_S6_RECOVERY_START,MISSION_STATE_S6_RECOVERY_TURN_BACK,
    MISSION_STATE_S6_REPOSITION_FORWARD,MISSION_STATE_S6_OBSTACLE_FACE_SAFE,
    MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE};
  for(unsigned i=0;i<sizeof(approach)/sizeof(approach[0]);++i) {
    setup(3,ROBOT_TEAM_BLUE,4,35000,26000);MissionTask_EnterState(approach[i],120);
    fake_now=140;rx(0x46);rx(0x16);rx(0x36);rx(0x26);tick(140,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && s6_bar_initial_align_pending && !s6_align26_pending);
    assert(MissionTask_YawError(s6_bar_side_yaw_cdeg,s6_bar_reverse_yaw_cdeg)==-9000L);
  }
  // All THREE fixed turns have independent2s soft limits. No early46 ACK.
  setup(3,ROBOT_TEAM_BLUE,5,35000,26000);fake_now=140;rx(0x46);tick(140,false);
  tick(2139,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && s6_bar_initial_align_pending);
  tick(2140,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_REVERSE && !s6_bar_initial_align_pending && !tx_calls);
  mission_snapshot.yaw_cdeg=s6_bar_reverse_yaw_cdeg;
  tick(2939,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_REVERSE && mission_snapshot.left_target_rpm==-150);
  tick(2940,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SIDE);
  tick(4939,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SIDE);
  tick(4940,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD);
  mission_snapshot.yaw_cdeg=s6_bar_side_yaw_cdeg;tick(5539,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD && mission_snapshot.left_target_rpm==150);
  tick(5540,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && !s6_bar_initial_align_pending);
  tick(7539,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE);
  tick(7540,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_WAIT_ACK && !tx_calls);
  assert(mission_snapshot.fault_flags==0);
  //800ms reverse and600ms forward keep their own full windows at tick wrap.
  setup(3,ROBOT_TEAM_BLUE,5,35000,26000);
  uint32_t begin=UINT32_MAX-300U;fake_now=begin;rx(0x46);tick(begin,false);
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;tick(begin+20U,false);
  uint32_t reverse=begin+20U;
  tick(reverse+799U,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_REVERSE && mission_snapshot.left_target_rpm==-150);
  tick(reverse+800U,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SIDE);
  mission_snapshot.yaw_cdeg=s6_bar_side_yaw_cdeg;tick(reverse+820U,false);
  uint32_t forward=reverse+820U;
  tick(forward+599U,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_FORWARD && mission_snapshot.left_target_rpm==150);
  tick(forward+600U,false);assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && !tx_calls);
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;tick(forward+620U,false);
  tick(forward+640U,false);assert(tx_calls==1 && last_tx_command==0x46 && mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE);
  // Health and stop protections apply to every appended state.
  for(unsigned phase=MISSION_STATE_S6_BAR_REVERSE;phase<=MISSION_STATE_S6_BAR_WAIT_ACK;++phase)
    for(unsigned f=0;f<3;++f) {
      setup(3,ROBOT_TEAM_BLUE,5,35000,26000);MissionTask_EnterState((MissionState)phase,140);
      health_faults=f==0?MISSION_FAULT_IMU:(f==1?MISSION_FAULT_LEFT_MOTOR:MISSION_FAULT_RIGHT_MOTOR);
      tick(160,true);assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm && !tx_calls);
      assert(!maixcam_bar_shift_active);
    }
  setup(3,ROBOT_TEAM_BLUE,5,35000,26000);fake_now=140;rx(0x46);tick(140,false);
  MissionTask_EnterState(MISSION_STATE_STOPPED,160);tick(180,true);
  assert(mission_snapshot.state==MISSION_STATE_STOPPED && !mission_snapshot.left_target_rpm && !tx_calls);
  const MissionState inactive[]={MISSION_STATE_WAIT_START,MISSION_STATE_S3_TRACK_GREEN,
    MISSION_STATE_S5_WAIT_SINGLE_GREEN,MISSION_STATE_S6_CORNER_TURN_AWAY,
    MISSION_STATE_S6_FINAL_ALIGN,MISSION_STATE_S6_RECHECK_WAIT_RESULT};
  for(unsigned i=0;i<sizeof(inactive)/sizeof(inactive[0]);++i) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,26000);MissionTask_EnterState(inactive[i],140);
    fake_now=160;rx(0x46);MissionTask_HandleS6BarShiftRequest(160);
    assert(mission_snapshot.state==inactive[i] && !tx_calls);
  }
  // Fresh cargo resets completion;6E and36 resume do not.
  setup(3,ROBOT_TEAM_BLUE,5,35000,26000);s6_bar_shift_completed=true;
  MissionTask_StartS6FromLoad(140);assert(!s6_bar_shift_completed);
  fake_now=160;rx(0x46);tick(160,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_BAR_TURN_SAFE && s6_bar_initial_align_pending);
  assert(fake_rx_arms==rx_bytes+uart_inits);
  puts("PASS:46 independent RX,64 zone/team/cargo/zero routes: SAFE first,back150/800,safe-relative left-supply/right-casualty90,fwd150/600,safe heading;160 wheel cap/10 correction,all3 turns2s,wrap/duplicate/REACK,TX-race fresh XY/26/05,partial26/46,ACK-failure retry and health/stop safety;UART stays armed");
}
