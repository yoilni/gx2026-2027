/* Test actual timer gates; logging must not throttle motion or vision input. */
static void debug_tick(uint32_t now) {
  MissionTask_DebugYaw(now);
  MissionTask_DebugVisionAge(now);
  MissionTask_DebugAngles(now,false);
}
static void verify_debug_period(void) {
  const uint32_t periods[]={ROBOT_ANGLE_DEBUG_PERIOD_MS,ROBOT_YAW_DEBUG_PERIOD_MS,
    ROBOT_S1_DEBUG_PERIOD_MS,ROBOT_S2_DEBUG_PERIOD_MS,ROBOT_VISION_DEBUG_PERIOD_MS,
    ROBOT_VISION_COORD_DEBUG_PERIOD_MS,ROBOT_S4_DEBUG_PERIOD_MS,
    ROBOT_S5_DEBUG_PERIOD_MS,ROBOT_S6_DEBUG_PERIOD_MS};
  for(unsigned i=0;i<sizeof(periods)/sizeof(periods[0]);++i) assert(periods[i]==750U);
  assert(MISSION_TASK_PERIOD_TICKS==20U && MAIXCAM_DATA_TIMEOUT_MS==500U);
  assert(ROBOT_HEADING_TURN_TIMEOUT_MS==2000U && ROBOT_S7_SILENT_TIMEOUT_MS==2000U);
  for(unsigned wrapped=0;wrapped<2;++wrapped) {
    setup(3,ROBOT_TEAM_BLUE,0);
    mission_snapshot.imu_valid=true;
    uint32_t start=wrapped?UINT32_MAX-374U:100U;
    angle_next_debug_tick=yaw_next_debug_tick=vision_age_next_debug_tick=start;
    yaw_prints=age_prints=angle_prints=0;
    debug_tick(start);
    assert(yaw_prints==1 && age_prints==1 && angle_prints==1);
    for(uint32_t elapsed=20;elapsed<750;elapsed+=20) debug_tick(start+elapsed);
    debug_tick(start+749);
    assert(yaw_prints==1 && age_prints==1 && angle_prints==1);
    debug_tick(start+750);
    assert(yaw_prints==2 && age_prints==2 && angle_prints==2);
    //State-entry snapshots still bypass periodic throttling.
    MissionTask_DebugAngles(start+751,true);
    assert(angle_prints==3);
    MissionTask_DebugAngles(start+1500,false);
    assert(angle_prints==3);
    MissionTask_DebugAngles(start+1501,false);
    assert(angle_prints==4);
  }
  puts("PASS:750ms periodic debug configuration and actual yaw/age/angle timer gates, wrap and immediate state-entry snapshots; motion/vision timeouts unchanged");
}
