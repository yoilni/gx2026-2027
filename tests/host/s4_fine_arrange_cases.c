/* Real32 fixed/42 tracking handlers, parser, E4 recovery and phase wait.
   Pixel PID is independently exercised in verify_supplement_capture. */
static void fine_step(uint32_t now)
{
  fake_now=now;MissionTask_UpdateVisionInput(now);MissionTask_CheckRunningHealth(now);
  MissionTask_HandleS4FineArrangeRequest(now);
  run_arrange_wait(now);MissionTask_RunS4FineArrange(now);
  MissionTask_RunS4E4Recovery(now);
}

static void fine_request(uint8_t cmd,uint32_t now,uint8_t id)
{
  fake_now=now;rx(cmd);coords(id);fine_step(now);
  assert(frame_down && maixcam_fine_arrange_active);
}

static uint32_t fine_reach42_turn(uint32_t now,uint8_t id)
{
  fine_request(0x42,now,id);
  fake_now=now+199;coords(id);fine_step(fake_now);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && !tx_count);
  mission_snapshot.yaw_cdeg=35900;
  fine_step(now+200);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
  assert(s4_fine_turn_yaw_cdeg==600 && s4_fine_turn_left);
  return now+200;
}

static uint32_t fine_finish42_tail(uint32_t now,uint8_t id)
{
  unsigned sent=tx_count;
  bool next_left=s4_fine_next_left;
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT);
  assert(s4_fine_turn_yaw_cdeg==MissionTask_WrapYaw(mission_snapshot.yaw_cdeg+2000));
  fake_now=now+20;rx(0x42);coords(id);turn_aligned=false;fine_step(now+20);
  assert(selected_turn_limit==60 && tx_count==sent && maixcam_fine_arrange_active);
  assert(s4_fine_next_left==next_left && mission_snapshot.state_entry_tick==now);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;
  fine_step(now+40);
  assert(tx_count==sent+1 && mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && frame_down);
  return now+40;
}

static void verify_initial_green_shared_budget(void)
{
  const uint8_t commands[]={0x02,0x32,0x42};
  for(unsigned c=0;c<3;++c) {
    setup(0x21);s7_target_switch_enabled=false;
    if(commands[c]==0x02) {
      MissionTask_EnterState(MISSION_STATE_S4_TRACK_RIGHT_BLOCK,100);
      MissionTask_StartS4PushTrial(100,true);
    } else fine_request(commands[c],100,5);
    assert(s4_green_e4_push_cycles==1);
    uint32_t entry=mission_snapshot.state_entry_tick;
    for(unsigned i=1;i<=5;++i) {
      fake_now=100+i*20;rx(commands[c]);coords(5);turn_aligned=false;fine_step(fake_now);
      assert(s4_green_e4_push_cycles==1 && mission_snapshot.state_entry_tick==entry);
    }
    // E4 in02/32/42 tracking acts immediately, before any fixed push.
    fake_now+=20;rx(0xE4);fine_step(fake_now);
    for(unsigned guard=0;guard<8 && maixcam_fine_arrange_active;++guard) {
      uint32_t next=mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD
          ? mission_snapshot.state_entry_tick+350U : fake_now+20U;
      mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;fine_step(next);
    }
    assert(s4_green_e4_push_cycles==1 && mission_snapshot.state==MISSION_STATE_S4_E4_GREEN_SPIN_360);
    assert(tx_count==1 && tx_commands[0]==0x04 && !maixcam_fine_arrange_active);
  }
  // Each newly accepted action consumes one unit, even after the E4 limit
  // of1. The camera's normal02/32/42 actions and ACKs are not suppressed.
  setup(0x21);s7_target_switch_enabled=false;
  fine_request(0x32,100,5);fine_step(400);
  assert(s4_green_e4_push_cycles==1 && tx_commands[0]==0x32);
  fine_request(0x42,420,5);assert(s4_green_e4_push_cycles==2);
  (void)MaixCam_CloseFineArrange();MissionTask_EnterState(MISSION_STATE_S4_TRACK_RIGHT_BLOCK,440);
  MissionTask_StartS4PushTrial(440,true);assert(s4_green_e4_push_cycles==3);
  (void)MaixCam_CloseFineArrange();MissionTask_EnterState(MISSION_STATE_S4_TRACK_LEFT_BLOCK,460);
  MissionTask_StartS4PushTrial(460,false);assert(s4_green_e4_push_cycles==3); //12 is ACK-only
  //07/red/51/31 tasks never consume the first-green allowance.
  const uint8_t targets[]={0x11,0x51,0x31,0x21};
  for(unsigned t=0;t<4;++t) for(unsigned c=0;c<3;++c) {
    setup(targets[t]);s4_green_e4_push_cycles=7;
    if(commands[c]==0x02) MissionTask_StartS4PushTrial(100,true);
    else fine_request(commands[c],100,targets[t]==0x11?4:6);
    assert(s4_green_e4_push_cycles==7);
  }
  setup(0x21);s7_target_switch_enabled=false;
  frame_lower_ok=false;MissionTask_StartS4PushTrial(100,true);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !s4_green_e4_push_cycles);
  for(unsigned c=1;c<3;++c) {
    setup(0x21);s7_target_switch_enabled=false;frame_lower_ok=false;
    fake_now=100;rx(commands[c]);MissionTask_HandleS4FineArrangeRequest(100);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !s4_green_e4_push_cycles);
  }
  setup(0x21);s7_target_switch_enabled=false;s4_green_e4_push_cycles=UINT8_MAX;
  MissionTask_RecordInitialGreenPush(0x32);assert(s4_green_e4_push_cycles==UINT8_MAX);
  assert(MissionTask_StartS3Align(120,0x21) && s4_green_e4_push_cycles==0);
  puts("PASS:first-green02/32/42 share E4 quota,one count per accepted action,no PID/duplicate/ACK double charge;quota-used E4 chooses gyro CCW360 not another scatter;normal commands still run,12/E5 and07 excluded,actuator failures not charged,saturation/new-green reset");
}

static void verify_s4_fine_arrange(void)
{
  verify_initial_green_shared_budget();
  uint8_t event;
  MaixCam_Object object;
  //32's100rpm/300ms fixed push ignores coordinates; no42 side/lift/capture34.
  setup(0x21);s7_target_switch_enabled=false;
  fine_request(0x32,100,5);
  for(unsigned i=1;i<=14;++i) {
    fake_now=100+i*20;rx(0x32);coords(5);fine_step(fake_now);
    assert(mission_snapshot.state_entry_tick==100 && !tx_count);
    assert(mission_snapshot.left_target_rpm==100 && mission_snapshot.right_target_rpm==100);
  }
  fake_now=399;coords(5);fine_step(399);
  assert(!tx_count && mission_snapshot.state==MISSION_STATE_S4_FINE32_TRACK);
  rx(0x34);fine_step(400);
  assert(tx_count==1 && tx_commands[0]==0x32 && s4_fine_next_left);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY && frame_down);
  assert(!mission_snapshot.left_target_rpm && !maixcam_fine_arrange_active);
  assert(!MaixCam_GetObjectSnapshot(&object));

  //42 alternates7/30 direction, then ALWAYS left20, no reverse.
  // ACK and alternation advance as soon as the final left20 completes.
  setup(0x31);uint32_t turn=fine_reach42_turn(100,6);
  rx(0x42);fine_step(turn+20);
  assert(selected_turn_limit==35 && !tx_count && mission_snapshot.state_entry_tick==turn);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;
  fine_step(turn+40);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
  uint32_t push=mission_snapshot.state_entry_tick;
  fine_step(push+349);assert(mission_snapshot.left_target_rpm==170 && !tx_count);
  rx(0x42);fine_step(push+350);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && s4_fine_turn_yaw_cdeg==33600);
  uint32_t counter=mission_snapshot.state_entry_tick;
  turn_aligned=false;fine_step(counter+20);assert(selected_turn_limit==60 && !tx_count);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;fine_step(counter+40);
  uint32_t back=fine_finish42_tail(counter+40,6);
  assert(tx_count==1 && tx_commands[0]==0x42 && !s4_fine_next_left && frame_down);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY);
  //32 in between must not change the next42 right-first direction.
  fine_request(0x32,back+420,6);
  fake_now=back+700;coords(6);fine_step(fake_now);fine_step(back+720);
  assert(tx_count==2 && tx_commands[1]==0x32 && !s4_fine_next_left);
  fine_request(0x42,back+1040,6);
  fake_now=back+1220;coords(6);fine_step(fake_now);fine_step(back+1240);
  assert(!s4_fine_turn_left && s4_fine_turn_yaw_cdeg==34900);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;fine_step(back+1260);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
  fine_step(back+1609);assert(mission_snapshot.left_target_rpm==170 && tx_count==2);
  fine_step(back+1610);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN && s4_fine_turn_yaw_cdeg==1900);
  mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;fine_step(back+1630);
  (void)fine_finish42_tail(back+1630,6);
  assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY);
  assert(tx_count==3 && tx_commands[2]==0x42 && s4_fine_next_left);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);

  // Started32 consumes first-green's allowance: loss goes straight to360.
  setup(0x21);s7_target_switch_enabled=false;fine_request(0x32,100,5);
  rx(0xE4);coords(5);fine_step(120);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_GREEN_SPIN_360);
  assert(tx_count==1 && tx_commands[0]==0x04 && s4_green_e4_push_cycles==1);
  //32 does not need coordinates; an irrelevant class cannot block its timer.
  setup(0x21);s7_target_switch_enabled=false;fine_request(0x32,100,4);
  assert(!mission_snapshot.vision_target_valid && mission_snapshot.left_target_rpm==100);
  fine_step(399);
  assert(!tx_count && mission_snapshot.state==MISSION_STATE_S4_FINE32_TRACK);
  assert(mission_snapshot.left_target_rpm==100 && mission_snapshot.right_target_rpm==100);
  fine_step(400);
  assert(tx_count==1 && tx_commands[0]==0x32 && mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);

  // E4 is retained through ALL FOUR fixed42 legs, later coordinates/14 and
  // generic event clearing. Finish with04+E4, never old42, never toggle side.
  for(unsigned phase=0;phase<4;++phase) {
    setup(0x21);s7_target_switch_enabled=false;turn=fine_reach42_turn(100,5);
    if(phase==0) { rx(0xE4);MaixCam_ClearEvent();coords(5);rx(0x14); }
    fine_step(turn+20);assert(!tx_count && mission_snapshot.state==MISSION_STATE_S4_FINE42_TURN);
    mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;fine_step(turn+40);
    push=mission_snapshot.state_entry_tick;
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FORWARD);
    if(phase==1) { rx(0xE4);MaixCam_ClearEvent();coords(5);rx(0x14); }
    rx(0x42);fine_step(push+349);assert(!tx_count && mission_snapshot.left_target_rpm==170);
    fine_step(push+350);counter=mission_snapshot.state_entry_tick;
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_COUNTER_TURN);
    if(phase==2) { rx(0xE4);MaixCam_ClearEvent();coords(5);rx(0x14); }
    turn_aligned=false;fine_step(counter+20);assert(!tx_count && selected_turn_limit==60);
    if(phase<3) assert(MaixCam_FineArrangeLossPending());
    mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;fine_step(counter+40);
    uint32_t finish=counter+40;
    assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT && !tx_count);
    if(phase==3) { rx(0xE4);MaixCam_ClearEvent();coords(5);rx(0x14); }
    turn_aligned=false;fine_step(finish+20);
    assert(selected_turn_limit==60 && !tx_count && maixcam_fine_arrange_active);
    mission_snapshot.yaw_cdeg=s4_fine_turn_yaw_cdeg;turn_aligned=true;fine_step(finish+40);
    /*42 already used the quota: E4 enters interruptible search, so queued14
       is honored immediately after04 sync instead of a second fixed push. */
    /* Keep the existing fine-loss04 and search-return04 syncs, then ACK14. */
    assert(tx_count==3 && tx_commands[0]==0x04 && tx_commands[1]==0x04 && tx_commands[2]==0x14 && s4_fine_next_left);
    assert(mission_snapshot.state==MISSION_STATE_S4_WAIT_ARRANGE_READY && s4_green_e4_push_cycles==1);
    assert(!MaixCam_PeekEvent(&event));
  }

  // ACK-time one-shot32/42 + following ROI coordinates survive; no extra14.
  setup(0x21);s7_target_switch_enabled=false;fine_request(0x32,100,5);
  fake_now=380;coords(5);fine_step(380);fine_inject_on_ack=0x42;fine_step(400);
  assert(tx_count==1 && tx_commands[0]==0x32 && MaixCam_PeekEvent(&event) && event==0x42);
  assert(MaixCam_GetObjectSnapshot(&object));fine_step(420);
  assert(mission_snapshot.state==MISSION_STATE_S4_FINE42_TRACK && mission_snapshot.vision_target_valid);

  // Green's E4 fixed push cannot be interrupted by fine requests. All five
  // valid phases are bounded/deduplicated and retained in receive order.
  setup_green_push();rx(0x14);rx(0x32);rx(0x42);rx(0x02);rx(0x24);rx(0x32);rx(0xE4);
  assert(maixcam_e4_stage_count==5);fine_step(140);
  assert(mission_snapshot.state==MISSION_STATE_S4_E4_PUSH_RIGHT && !tx_count);
  const uint8_t expected[]={0x14,0x32,0x42,0x02,0x24};
  for(unsigned i=0;i<sizeof(expected);++i) { assert(MaixCam_TakeEvent(&event) && event==expected[i]); }

  // Actuator/TX/health errors stop rather than acknowledge; fixed-motion
  // commands remain in live health coverage, and ordinary05 can pause all.
  setup(0x21);frame_lower_ok=false;fake_now=100;rx(0x32);fine_step(100);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_count);
  setup(0x21);fine_request(0x32,100,5);fake_now=380;coords(5);fine_step(380);
  tx_ok=false;fine_step(400);assert(mission_snapshot.state==MISSION_STATE_FAULT);
  for(MissionState s=MISSION_STATE_S4_FINE32_TRACK;s<=MISSION_STATE_S4_FINE42_FINISH_TURN_LEFT;++s) {
    setup(0x21);MissionTask_EnterState(s,100);
    assert(MissionTask_CanPauseForLoadCheck());health_faults=MISSION_FAULT_IMU;fine_step(120);
    assert(mission_snapshot.state==MISSION_STATE_FAULT && !mission_snapshot.left_target_rpm && !tx_count);
  }
  assert(fake_rx_arms>=rx_bytes);
  puts("PASS:32 fixed100rpm/300ms/down/no XY required/TX32;supplies42 center200ms,L7 push170/350 R30 and mirror,then fixed LEFT20 and immediateTX42 NO REVERSE;turn caps35/60,duplicates/reference/ACK bursts,class/freshness,no34/no extra14,all4 legs deferred E4/no old42,FIFO and safety");
}
