/* RX26 rechecks the current field yaw before delivery, without the normal
   one-second XY gate. Only corner evacuation may override26, never side motion. */
static void verify_s6_align26_review(void)
{
  const int32_t fields[]={10000,25000};
  for(uint8_t cargo=4;cargo<=5;++cargo) for(unsigned i=0;i<2;++i) {
    setup(3,ROBOT_TEAM_BLUE,cargo,35000,fields[i]);
    fake_now=120;rx(0x26);tick(120,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY && !s6_align26_pending);
    mission_snapshot.yaw_cdeg=s6_corner_away_yaw_cdeg;tick(140,false);
    uint32_t away_start=140U+ROBOT_S6_TURN_STABLE_MS;
    tick(away_start,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_AWAY);
    fake_now=away_start+20U;rx(0x26);tick(fake_now,false);
    assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    uint32_t away_end=away_start+ROBOT_S6_CORNER_AWAY_FORWARD_MS;
    tick(away_end-1U,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_AWAY);
    tick(away_end,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_SIDE);
    mission_snapshot.yaw_cdeg=s6_corner_side_yaw_cdeg;tick(away_end+20U,false);
    uint32_t side_start=away_end+20U+ROBOT_S6_TURN_STABLE_MS;
    tick(side_start,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_SIDE);
    fake_now=side_start+20U;rx(0x26);tick(fake_now,false);
    assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    uint32_t side_end=side_start+ROBOT_S6_CORNER_SIDE_FORWARD_MS;
    tick(side_end-1U,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_FORWARD_SIDE);
    tick(side_end,false);assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_SAFE);
    mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;tick(side_end+20U,false);
    uint32_t safe_done=side_end+20U+ROBOT_S6_TURN_STABLE_MS;
    tick(safe_done,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_ACK && s6_corner_completed);
    assert(tx_calls==1 && last_tx_command==0x05 && s6_carried_object_id==cargo);
    assert(!s6_align26_pending && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    fake_now=safe_done+20U;rx(0x26);rx(0x05);cargo_report(cargo);rx(0x06);tick(fake_now,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT && !s6_align26_pending);
    uint32_t wide=safe_done+20U+ROBOT_S6_RECHECK_CAMERA_SETTLE_MS;
    tick(wide,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WIDE_SETTLE);
    wide+=ROBOT_S6_RECHECK_WIDE_SETTLE_MS;
    tick(wide,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_TRACK_SAFE_ZONE && s6_corner_completed);
    assert(tx_calls==1 && !s6_align26_pending && !mission_snapshot.fault_flags);
    assert(s6_carried_object_id==cargo);
    // No replay of the old26. Only a NEW26 starts delivery, and this load
    // does not evacuate again even if the car is still within a corner sector.
    mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(35000+fields[i]);
    fake_now=wide+20;rx(0x26);tick(fake_now,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && tx_calls==1);
  }
}

static void verify_s6_align26_side_deferred(void)
{
  assert(ROBOT_S6_REPOSITION_ALIGN26_FORWARD_MS==200U && ROBOT_S6_REPOSITION_FORWARD_RPM==120);
  const uint8_t teams[]={ROBOT_TEAM_RED,ROBOT_TEAM_BLUE};
  const int32_t side_fields[]={15700,19492};
  for(uint8_t zone=1;zone<=4;++zone) for(unsigned team=0;team<2;++team)
    for(uint8_t cargo=4;cargo<=6;++cargo) for(unsigned side=0;side<2;++side)
      for(unsigned phase=0;phase<3;++phase) for(unsigned wrap=0;wrap<2;++wrap) {
    int32_t safe=MissionTask_GetSafeZoneYawOffset(zone,teams[team]);
    setup(zone,teams[team],cargo,35000,MissionTask_WrapYaw(side_fields[side]+safe-18000));
    uint32_t start=wrap?UINT32_MAX-500U:120U;
    fake_now=start;MissionTask_StartS6SideReposition(start,mission_snapshot.yaw_cdeg);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE && s6_reposition_count==1);
    uint32_t duration=s6_reposition_forward_ms;
    uint32_t actual_duration=phase<=1?200U:duration;
    fake_now=start+20U;if(phase==0) rx(0x26);tick(fake_now,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
    assert(mission_snapshot.state_entry_tick==start && s6_align26_pending==(phase==0));
    assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm && mission_snapshot.left_target_rpm);
    fake_now=start+40U;if(s6_align26_pending) { rx(0x26);rx(0x26); } tick(fake_now,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE && mission_snapshot.state_entry_tick==start);
    mission_snapshot.yaw_cdeg=s6_reposition_side_yaw_cdeg;tick(start+60U,true);
    tick(start+60U+ROBOT_S6_TURN_STABLE_MS-1U,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SIDE);
    tick(start+60U+ROBOT_S6_TURN_STABLE_MS,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD);
    uint32_t forward=mission_snapshot.state_entry_tick;
    fake_now=forward+20U;if(phase<=1) rx(0x26);tick(fake_now,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD && mission_snapshot.state_entry_tick==forward);
    assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    assert(s6_reposition_forward_ms==duration && s6_align26_pending==(phase<=1));
    fake_now=forward+actual_duration-1U;if(s6_align26_pending) rx(0x26);tick(fake_now,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD);
    assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    tick(forward+actual_duration,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE);
    uint32_t returning=mission_snapshot.state_entry_tick;
    fake_now=returning+20U;rx(0x26);tick(fake_now,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && s6_align26_pending);
    assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm && mission_snapshot.left_target_rpm);
    fake_now=returning+40U;rx(0x26);tick(fake_now,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && mission_snapshot.state_entry_tick==returning);
    assert(!maixcam_corner_evacuation_active && !tx_calls && s6_reposition_count==1);
    mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
    tick(returning+60U,true);
    tick(returning+60U+ROBOT_S6_TURN_STABLE_MS-1U,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && s6_align26_pending);
    uint32_t done=returning+60U+ROBOT_S6_TURN_STABLE_MS;
    fake_now=done;rx(0x26);tick(done,true);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && mission_snapshot.state_entry_tick==done);
    assert(!s6_align26_pending && !s6_pre_reposition_decision_pending && !tx_calls);
    assert(!maixcam_corner_evacuation_active && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    assert(s6_push_yaw_target_cdeg==mission_snapshot.safe_zone_yaw_target_cdeg && s6_carried_object_id==cargo);
    assert(!MaixCam_TakeSafeZoneAlignRequest() && !mission_snapshot.fault_flags);
    fake_now=done+20U;rx(0x26);tick(fake_now,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && mission_snapshot.state_entry_tick==done);
  }
  // A late26 never restarts a200ms clock or retroactively adds travel.
  const uint32_t offsets[]={0U,20U,199U,200U,250U,600U};
  for(unsigned wrap=0;wrap<2;++wrap) for(unsigned o=0;o<sizeof(offsets)/sizeof(offsets[0]);++o) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
    uint32_t start=wrap?UINT32_MAX-100U:120U;
    s6_approach_start_tick=start;s6_reposition_forward_ms=1225U;
    MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_FORWARD,start);
    if(offsets[o]) {
      tick(start+offsets[o]-1U,false);
      assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD);
      assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
    }
    fake_now=start+offsets[o];rx(0x26);rx(0x26);tick(fake_now,false);
    assert(s6_align26_pending && s6_reposition_forward_ms==1225U && !tx_calls);
    uint32_t returning=start+(offsets[o]<200U?200U:offsets[o]);
    if(offsets[o]<200U) {
      assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD && mission_snapshot.state_entry_tick==start);
      assert(mission_snapshot.left_target_rpm==120 && mission_snapshot.right_target_rpm==120);
      fake_now=start+199U;rx(0x26);tick(fake_now,false);
      assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_FORWARD && mission_snapshot.state_entry_tick==start);
      tick(start+200U,false);
    }
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && mission_snapshot.state_entry_tick==returning);
    assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
    tick(returning+20U,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && s6_align26_pending);
    mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
    tick(returning+40U,false);tick(returning+40U+ROBOT_S6_TURN_STABLE_MS,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && !s6_align26_pending && !mission_snapshot.fault_flags);
  }
  // The legacy FACE_SAFE state must still turn back before a held26 runs.
  setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
  MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_FACE_SAFE,100);
  fake_now=120;rx(0x26);tick(120,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && s6_align26_pending);
  tick(140,false);assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE);
  mission_snapshot.yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
  tick(160,false);tick(160U+ROBOT_S6_TURN_STABLE_MS,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && !s6_align26_pending);

  const MissionState phases[]={MISSION_STATE_S6_REPOSITION_TURN_SIDE,MISSION_STATE_S6_REPOSITION_FORWARD,
      MISSION_STATE_S6_REPOSITION_TURN_SAFE,MISSION_STATE_S6_REPOSITION_FACE_SAFE};
  for(unsigned p=0;p<4;++p) for(unsigned interrupt=0;interrupt<3;++interrupt) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,18000);
    MissionTask_EnterState(phases[p],100);fake_now=120;rx(0x26);
    MissionTask_HandleS6AlignRequest(120);
    assert(s6_align26_pending && mission_snapshot.state==phases[p]);
    if(interrupt==0) { rx(0x05);tick(140,false);assert(mission_snapshot.state==MISSION_STATE_S6_RECHECK_WAIT_RESULT); }
    else if(interrupt==1) { health_faults=MISSION_FAULT_IMU;tick(140,false);assert(mission_snapshot.state==MISSION_STATE_FAULT); }
    else { MissionTask_EnterState(MISSION_STATE_STOPPED,140);tick(160,false);assert(mission_snapshot.state==MISSION_STATE_STOPPED); }
    assert(!s6_align26_pending && !mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm && !tx_calls);
  }
  // Keep the normal2s turn soft deadline and recheck ACTUAL yaw afterward.
  setup(3,ROBOT_TEAM_BLUE,5,35000,9000);
  MissionTask_EnterState(MISSION_STATE_S6_REPOSITION_TURN_SAFE,100);
  fake_now=120;rx(0x26);tick(120,false);tick(2099,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_REPOSITION_TURN_SAFE && s6_align26_pending);
  tick(2100,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY && !s6_align26_pending);
  assert(mission_snapshot.yaw_cdeg==MissionTask_WrapYaw(35000+9000) && !mission_snapshot.fault_flags);
  puts("PASS:side26 forward170rpm TOTAL200ms only while held;normal angle duration retained,late26 at0/20/199/200/250/600ms never restarts clock;all8 zone/team,cargo4/5/6,both sides,3 receipt phases,wrap;full turn/return+200ms stable before delivery,duplicates/fresh XY,05/fault/stop and2s corner safety preserved");
}

static void verify_s6_align26_corner(void)
{
  const uint8_t teams[]={ROBOT_TEAM_RED,ROBOT_TEAM_BLUE};
  const uint8_t cargos[]={3,4,5,6};
  const int32_t zeroes[]={0,35000,17900};
  const int32_t angles[]={0,8999,9000,9001,13499,13500,13501,15700,
    16500,18000,19000,19001,22400,22401,26999,27000,27001,35999};
  for(uint8_t zone=1;zone<=4;++zone) for(unsigned t=0;t<2;++t)
    for(unsigned c=0;c<4;++c) for(unsigned z=0;z<3;++z)
      for(unsigned a=0;a<sizeof(angles)/sizeof(angles[0]);++a) {
        int32_t safe=MissionTask_GetSafeZoneYawOffset(zone,teams[t]);
        int32_t field=MissionTask_WrapYaw(angles[a]+safe-18000);
        setup(zone,teams[t],cargos[c],zeroes[z],field);
        assert(!mission_snapshot.vision_target_valid && !s6_pre_reposition_track_elapsed_ms);
        fake_now=120;rx(0x26);tick(120,false);
        assert(!s6_align26_pending && !s6_pre_reposition_decision_pending);
        assert(!s6_reposition_count && !tx_calls && !mission_snapshot.fault_flags);
        assert(!s6_pre_reposition_track_elapsed_ms);
        if(corner_expected(angles[a])) {
          assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
          assert(s6_corner_away_yaw_cdeg==MissionTask_WrapYaw(zeroes[z]+safe+18000));
          int32_t side=angles[a]<18000?9000:27000;
          assert(s6_corner_side_yaw_cdeg==MissionTask_WrapYaw(zeroes[z]+safe-18000+side));
          assert(maixcam_corner_evacuation_active && !s6_corner_completed);
          fake_now=140;rx(0x26);rx(0x26);tick(140,false);
          assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
          assert(mission_snapshot.state_entry_tick==120 && !s6_align26_pending && !tx_calls);
        } else {
          assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN);
          assert(s6_push_yaw_target_cdeg==mission_snapshot.safe_zone_yaw_target_cdeg);
          assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
        }
        assert(mission_snapshot.left_target_rpm==-mission_snapshot.right_target_rpm);
      }

  // RX26 still interrupts non-side approach branches. Side motion is covered
  // separately and must finish before the current-yaw corner check.
  const MissionState approaches[]={MISSION_STATE_S6_SEARCH_SAFE_ZONE,
    MISSION_STATE_S6_TRACK_SAFE_ZONE,MISSION_STATE_S6_RECOVERY_START,
    MISSION_STATE_S6_RECOVERY_TURN_LEFT,MISSION_STATE_S6_RECOVERY_TURN_BACK,
    MISSION_STATE_S6_RECOVERY_RETURN_SAFE,MISSION_STATE_S6_OBSTACLE_TURN_OUT,
    MISSION_STATE_S6_OBSTACLE_FORWARD,MISSION_STATE_S6_OBSTACLE_TURN_BACK,
    MISSION_STATE_S6_OBSTACLE_FACE_SAFE,MISSION_STATE_S6_TRACK_TIMEOUT_REVERSE};
  for(unsigned i=0;i<sizeof(approaches)/sizeof(approaches[0]);++i) {
    setup(3,ROBOT_TEAM_BLUE,4,35000,25000);
    MissionTask_EnterState(approaches[i],100);
    s6_obstacle_side_decision_pending=true;
    s6_obstacle_decision_yaw_cdeg=mission_snapshot.safe_zone_yaw_target_cdeg;
    fake_now=120;rx(0x26);tick(120,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY);
    assert(!s6_obstacle_side_decision_pending && !s6_align26_pending && !s6_reposition_count);
    assert(s6_corner_side_yaw_cdeg==MissionTask_WrapYaw(35000+27000));
  }
  // The deferred26 path uses the same check, with no coordinate/time gate.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);s6_align26_pending=true;
  fake_now=120;MissionTask_RunS6TrackSafeZone(120);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY && !s6_align26_pending);

  // A completed corner is not repeated for the same cargo, even on26.
  // Ordinary side-only sectors likewise deliver directly instead of side-moving.
  const int32_t completed_angles[]={9000,10000,13499,22401,25000,27000};
  for(unsigned i=0;i<sizeof(completed_angles)/sizeof(completed_angles[0]);++i) {
    setup(3,ROBOT_TEAM_BLUE,5,35000,completed_angles[i]);s6_corner_completed=true;
    fake_now=120;rx(0x26);tick(120,false);
    assert(mission_snapshot.state==MISSION_STATE_S6_FINAL_ALIGN && !tx_calls);
  }

  // After46's successful ACK,26 also uses this corner check, not the old
  // unconditional final-alignment jump. The46 route itself is not restarted.
  uint32_t ack=bar_route_to_ack(3,ROBOT_TEAM_BLUE,5,35000);tick(ack,false);
  mission_snapshot.yaw_cdeg=MissionTask_WrapYaw(35000+10000);
  fake_now=ack+20;rx(0x26);tick(fake_now,false);
  assert(mission_snapshot.state==MISSION_STATE_S6_CORNER_TURN_AWAY && s6_bar_shift_completed);
  assert(tx_calls==1 && last_tx_command==0x46);

  // Safety still precedes the26 request; it must not restart a faulted car.
  setup(3,ROBOT_TEAM_BLUE,5,35000,10000);health_faults=MISSION_FAULT_IMU;
  fake_now=120;rx(0x26);tick(120,false);
  assert(mission_snapshot.state==MISSION_STATE_FAULT && !tx_calls);
  assert(!mission_snapshot.left_target_rpm && !mission_snapshot.right_target_rpm);
  verify_s6_align26_review();
  verify_s6_align26_side_deferred();
  puts("PASS:RX26 checks CURRENT corner yaw immediately without XY/1000ms gate;all8 zone/team rules,4 cargos,3 zeroes,centidegree boundaries;no ordinary side motion,all approach branches/deferred26/post46,duplicate26/once-per-load,full corner->TX05/RX05+ID+06->NEW26 and health safety");
}
