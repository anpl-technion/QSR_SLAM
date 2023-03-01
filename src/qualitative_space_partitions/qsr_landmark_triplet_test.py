close
all;
clear;
clc;

addpath('EDC');
addpath('Simulation');

rng(7);
map_lims = [-2, 2, -2, 3];
rmin = 0.01;
n = 10;
maxid = 15;
data_mode = 'prob+id'; % 'prob', 'id', 'prob+id'

f1 = figure('name', sprintf('triplet test - global frame'));
f2 = figure('name', sprintf('triplet test - ABaligned frame'));

for i=1:n

% generate
landmark
ids
ids = [1, 3, 7];
tripletid = genTripletId(ids(1), ids(2), ids(3), maxid);

% generate
landmark
locations
mapid = i;
map = SimulationMap(mapid, map_lims, rmin);
map.add_landmark(3, ids(:));
xylims = map.map_lims;

a = map.landmark_ids;
Aid = a(1);
Bid = a(2);
Cid = a(3);
Aloc = map.get_landmark(Aid);
Bloc = map.get_landmark(Bid);
Cloc = map.get_landmark(Cid);

% set
triplet
Cstate = rand(20, 1);
Cstate = Cstate. / (sum(Cstate));
EDCgT = EdcGtTriplet(tripletid, Aid, Bid, Cid, Aloc, Bloc, Cloc, xylims, Cstate);

% show
triplet
fprintf('---------- EDC triplet test %d\n', i);
disp(EDCgT);

% plot in
global frame
xlim = [];
ylim = [];
hold
off;

options = struct('clr', 'm', ...
'xlim', [], 'ylim', [], ...
'frame', 'global');
EDCgT.plotLmrks(f1, options);
hold
on;
title(sprintf('test %d - global frame', i));

options = struct('clr', 'm', ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'global');
EDCgT.plotConstraints(f1, options);

options = struct('clr', 'g', ...
'alpha', 0.2, ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'global');
EDCgT.plotGtState(f1, options);

options = struct('clr', 'b', ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'global');
EDCgT.plotStates(data_mode, f1, options);

% plot in AB
frame
xlim = [EDCgT.xylims(1) - 0.5, EDCgT.xylims(2) + 0.5];
ylim = [EDCgT.xylims(3) - 0.5, EDCgT.xylims(4) + 0.5];
hold
off;

options = struct('clr', 'm', ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'ABaligned');
EDCgT.plotLmrks(f2, options);
hold
on;
title(sprintf('test %d - ABaligned frame', i));

options = struct('clr', 'm', ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'ABaligned');
EDCgT.plotConstraints(f2, options);

options = struct('clr', 'g', ...
'alpha', 0.2, ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'ABaligned');
EDCgT.plotGtState(f2, options);

options = struct('clr', 'b', ...
'xlim', xlim, 'ylim', ylim, ...
'frame', 'ABaligned');
EDCgT.plotStates(data_mode, f2, options);
hold
off;

fprintf('test done! \n  ---------------\n\n')
end

