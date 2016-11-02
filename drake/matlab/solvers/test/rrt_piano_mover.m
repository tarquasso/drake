function rrt_piano_mover(n_obstacles, planning_mode, n_smoothing_passes, visualize)
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  %rng(2)
  if nargin < 1 || isempty(n_obstacles)
    n_obstacles = 50;
  end
  if nargin < 2 || isempty(planning_mode), planning_mode = 'rrt'; end
  if nargin < 3 || isempty(n_smoothing_passes), n_smoothing_passes = 10; end
  if nargin < 4 || isempty(visualize), visualize = false; end
  urdf = fullfile(getDrakePath, 'matlab', 'systems', 'plants', 'test', 'FallingBrick.urdf');
  options.floating = true;
  xyz0 = [0; 0; -2];
  xyzf = [0; 0; 12];
  quat0 = [1;0;0;0];
  quatf = [1;0;0;0];
  rpy0 = quat2rpy(quat0);
  rpyf = quat2rpy(quatf);
  q0 = [xyz0; quat0];
  qf = [xyzf; quatf];

  %prob = MotionPlanningProblem(7);
  %min_distance = 0.2;
  %min_distance_penalty = drakeFunction.kinematic.SmoothDistancePenalty(r, min_distance);
  TA = SE3MotionPlanningTree();
  TA.min_distance = 0.2;
  TA.max_edge_length = 0.5;
  TA.max_length_between_constraint_checks = 0.1;
  TA = TA.setOrientationWeight(1);
  TA = TA.setTranslationSamplingBounds([-5; -5; -2], [5; 5; 12]);

  box_size = [1;1;1];
  xyz_min = [-10; -10; 0];
  xyz_max = [10; 10; 10];
  for i = 1:n_obstacles
    xyz = xyz_min + rand(3,1).*(xyz_max - xyz_min);
    rpy = uniformlyRandomRPY();
    obstacle = RigidBodyBox(box_size, xyz, rpy);
    TA = TA.addGeometryToWorld(obstacle);
  end
  robot_geom = RigidBodyBox([1, 10, 0.5], [0;0;0], [0;pi/2;0]);
  TA = TA.addGeometryToRobot(robot_geom);
  robot_geom = RigidBodyBox([1, 10, 0.5], [0;0;0], [0;pi/2;pi/2]);
  TA = TA.addGeometryToRobot(robot_geom);
  TA = TA.compile();

  TB = TA;

  TA = TA.setLCMGL('TA',[1,0,0]);
  TB = TB.setLCMGL('TB',[0,0,1]);

  v = TA.rbm.constructVisualizer();
  v.draw(0,[xyz0; rpy0]);

  options.display_after_every = 100;
  options.visualize = visualize;
  rrt_timer = tic;
  switch planning_mode
    case 'rrt'
      [TA, path_ids_A, info] = TA.rrt(q0, qf, options);
    case 'rrt_connect'
      [TA, path_ids_A, info] = TA.rrtConnect(q0, qf, TB, options);
  end
  rrt_time = toc(rrt_timer);
  fprintf('Timing:\n');
  fprintf('  RRT:       %5.2f s\n', rrt_time);
  if info == 1
    smoothing_timer = tic;
    [TA_smooth, id_last] = TA.recursiveConnectSmoothing(path_ids_A, n_smoothing_passes);
    path_ids_A = TA_smooth.getPathToVertex(id_last);
    smoothing_time = toc(smoothing_timer);
    fprintf('  Smoothing: %5.2f s\n', smoothing_time);

    TA_smooth = TA_smooth.setLCMGL('TA_smooth', TA_smooth.line_color);
    %drawTree(TA);
    %drawTree(TB);
    drawPath(TA_smooth, path_ids_A);

    %q_path = extractPath(TA_smooth, path_ids_A, TB_smooth, path_ids_B);
    q_path = extractPath(TA_smooth, path_ids_A);
    path_length = size(q_path,2);
    rpy_path = zeros(3, path_length);
    for i = 1:path_length
      rpy_path(:,i) = quat2rpy(q_path(4:7,i));
    end
    traj = PPTrajectory(foh(linspace(0,10,path_length), [q_path(1:3,:); rpy_path]));
    traj = traj.setOutputFrame(TA.rbm.getPositionFrame());
    TA.lcmgl.switchBuffers;
    %v.playback(traj, struct('slider', true));
    v.playback(traj);
  end

  warning(w);
end
