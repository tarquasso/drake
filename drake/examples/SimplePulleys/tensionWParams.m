%function trueParameters = tensionWParams

r = PlanarRigidBodyManipulator('tensionWParams.urdf');
%r = TimeSteppingRigidBodyManipulator('tensionWParams.urdf',.01,struct('twoD',true));

trueParameters = getParams(r);

v = r.constructVisualizer();
v.xlim = [-3.1 -2.512];
v.ylim = [2.79 3.21];

x0 = Point(getStateFrame(r));
x0.load_x = -2.811; %TODO: adjust x point of collision, we canassume it is center
x0.load_z = 3.0+88.46/2/1000; % radius of sphre is  
x0.tensioner_angle = .1414;%pi/2;
x0.load_zdot = -2; %TODO: take original speed from capture data




v.drawWrapper(0,x0(1:3));

%manip = r.getManipulator();
%[l,dl]=manip.position_constraints{1}.eval(x0(1:3));
%gradTest(@eval,manip.position_constraints{1}.fcn,x0(1:3));
%%manip.position_constraints{1}.checkGradient(.001,x0(1:3));
%return;

x0 = resolveConstraints(r,x0,v);
v.drawWrapper(0,x0(1:3));

ytraj = simulate(r,[0 0.22],x0);
len = [];
if(1)
ts = ytraj.getBreaks();
for i=1:numel(ts)
  x = ytraj.eval(ts(i));
  len(i) = r.position_constraints{1}.eval(x(1:3));
end
figure(1); clf; plot(ts,len);
end

v.playback(ytraj,struct('slider',true));
%v.playbackSWF(ytraj,'tension')

