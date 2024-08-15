void executeWaypoints(){
  float vmax = 15.0/1000.0; //m/s
  float amax = 15.0/1000.0; //m/s2
  float xf[NUM_MOTORS];
  if(traj_iter < 20 && go){
    is_movement_done = false;
    sent_done = false;
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      xf[i] = trajectory[traj_iter][i];
    }
    controller.ramp2pos(xf, vmax, amax);
    traj_iter++;
 
  }else if(traj_iter >= 20 && !sent_done){
    is_movement_done = true;
    Serial.print(traj_iter);Serial.println(" No. of iters done! ");
    send_done_signal();
    go = false;
    controller.stop_moving();
  }
}

void updateTrajectory(){
  traj_iter = 0;
  go = true;
  reset_flag = false;
  
}

void reset() {
  controller.reset_joints();
}
