function trueParameters = tensionWParamsExp


r = PlanarRigidBodyManipulator('tensionWParamsExp.urdf');
% r = TimeSteppingRigidBodyManipulator('tensionWParamsExp.urdf',.01,struct('twoD',true));

trueParameters = getParams(r);

v = r.constructVisualizer();
v.xlim = [-0.26 0.26];
v.ylim = [-0.11 0.26];

x0 = Point(getStateFrame(r));
x0.load_x = 0;
x0.load_z = 0.08849;
x0.tensioner_angle = .0298;%pi/2;
x0.load_zdot = -2;

v.drawWrapper(0,x0(1:3));

%manip = r.getManipulator();
%[l,dl]=manip.position_constraints{1}.eval(x0(1:3));
%gradTest(@eval,manip.position_constraints{1}.fcn,x0(1:3));
%%manip.position_constraints{1}.checkGradient(.001,x0(1:3));
%return;

x0 = resolveConstraints(r,x0,v);
v.drawWrapper(0,x0(1:3));

ytraj = simulate(r,[0 0.22],x0);
if(1)
ts = ytraj.getBreaks();
len = [];
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  len(i) = r.position_constraints{1}.eval(x(1:3));
end
figure(1); clf; plot(ts,len);
end

v.playback(ytraj,struct('slider',true));
%v.playbackSWF(ytraj,'tension')

