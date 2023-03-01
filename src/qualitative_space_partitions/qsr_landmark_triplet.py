classdef
EdcGtTriplet < EdcTriplet & handle
properties
Apos % landmark
A
2
D
location(x, y)
Bpos % landmark
B
2
D
location(x, y)
Cpos % landmark
C
2
D
location(x, y)
GtState % a, b: c
EDC
ground
truth
state
R % rotation
from

global frame
to
A = (0, 0)
B = (0, 1)
frame
T % translation
from

global frame
to
A = (0, 0)
B = (0, 1)
frame
S % scale
from

global frame
to
A = (0, 0)
B = (0, 1)
frame
xylims % x, y
limits
of
reference in AB
frame
a % rotation
angle
from

global frame
to
A = (0, 0)
B = (0, 1)
frame

end
methods

function
obj = EdcGtTriplet(triplet_id, Aid, Bid, Cid, Apos, Bpos, Cpos, xylims, Cstate)
% % Class
constructor
creates
objects
using:
% EdcTriplet
existing
object and landmark
poses
% Apos - A
location[x, y]
% Bpos - B
location[x, y]
% Cpos - C
location[x, y]
% xylims - x, y
limits
of
reference in AB
frame

if nargin ~= 9
error('invalid input: must be EdcT,Apose,Bpose,Cpose');
end

obj @ EdcTriplet(triplet_id, Aid, Bid, Cid, Cstate);
obj.Apos = Apos(:);
obj.Bpos = Bpos(:);
obj.Cpos = Cpos(:);
obj.xylims = xylims;

% find
global frame
to
A = (0, 0)
B = (0, 1)
frame
transform
obj = obj.update_transform();
% BA = Bpos(:)-Apos(:);
% a = wrapToPi(atan2(BA(2), BA(1)) - pi / 2);
% obj.S = 1. / norm(BA);
% obj.R = [cos(a), -sin(a);
sin(a), cos(a)]';
% obj.T = -Apos(:);
% obj.a = a;

% A = obj.R * (Apos(:) + obj.T).*s;
% B = obj.R * (Bpos(:) + obj.T).*s;

obj.GtState = obj.eval_state(Cpos(1), Cpos(2));

end

function
obj = update_transform(obj)
% find
global frame
to
A = (0, 0)
B = (0, 1)
frame
transform
if ~isempty(obj.Apos) & & ~isempty(obj.Bpos)
    BA = obj.Bpos(:)-obj.Apos(:);
    obj.a = wrapToPi(atan2(BA(2), BA(1)) - pi / 2);
    obj.S = 1. / norm(BA);
    obj.R = [cos(obj.a), -sin(obj.a);
    sin(obj.a), cos(obj.a)]';
    obj.T = -obj.Apos(:);
    else
    obj.a = [];
    obj.S = [];
    obj.R = [];
    obj.T = [];
end

% A = obj.R * (Apos(:) + obj.T).*s;
% B = obj.R * (Bpos(:) + obj.T).*s;
end

function[xab, yab] = World2AB(obj, xw, yw)
% % transforms[x, y]
points
from

global
% to
A = [0, 0].B = [0, 1]
frame
% pw = R * (p + T) * S
W = obj.R * (bsxfun( @ plus, [xw(:)
';yw(:)'], obj.T)).*obj.S;
xab = W(1,:);
yab = W(2,:);
end

function[xab, yab, azab] = World2AB_cam(obj, xw, yw, azw)
% % transforms[x, y, az]
camera
from

global
% to
A = [0, 0].B = [0, 1]
frame
% pw = R * (p + T) * S
% azw = az + a
[xab, yab] = obj.World2AB(xw, yw);
azab = azw - obj.a;
end

function[xw, yw] = AB2World(obj, xab, yab)
% % transforms[x, y]
points
from A=[0, 0].B = [0, 1]
frame
% to
global coordinates
% p = R'*pw/S-T
W = bsxfun( @ minus, obj.
R'*[xab(:)';
yab(:)']./obj.S,obj.T);
xw = W(1,:);
yw = W(2,:);
end

function[xw, yw, azw] = AB2World_cam(obj, xab, yab, azab)
                        % % transforms[x, y]
points
from A=[0, 0].B = [0, 1]
frame
% to
global coordinates
% p = R'*pw/S-T
[xw, yw] = AB2World_cam(obj, xab, yab);
azw = azab + obj.a;
end

function
res = eval_state(obj, xin, yin, frame)
      % % returns
qualitative
state
for query points
          % loc - [N, 2] - [x, y] of query points - in global frame
% frame - xin, yin are given in global / AB frame

N = numel(xin);
if ~isnumeric(xin) | | ~isnumeric(yin) | | numel(xin)~=numel(yin)
error('Cpos must be numerical scalar')
end

if nargin < 4
frame = 'global';
end

% transform x, y to AB frame
if strcmp(frame, 'global')
[x, y]=obj.World2AB(xin(:), yin(:));
else
x = xin(:);
y = yin(:);
end
dA = hypot(x, y);
dB = hypot(x, y - 1);

% find
each
point
EDC
state in AB
frame
InFrntB = y >= 1;
BhndB = ~InFrntB;
InFrntA = y >= 0;
BhndA = ~InFrntA;
Closer2B = y >= 0.5;
Closer2A = ~Closer2B;
Left = x < 0;
Right = ~Left;
NearA = dA <= 1;
FarfrmA = ~NearA;
NearB = dB <= 1;
FarfrmB = ~NearB;

res = nan();
res(InFrntB & FarfrmB & Left) = 1;
res(InFrntB & FarfrmB & Right) = 2;
res(InFrntB & NearB & Left) = 3;
res(InFrntB & NearB & Right) = 4;
res(BhndB & Closer2B & Left & FarfrmA & FarfrmB) = 5;
res(BhndB & Closer2B & Left & FarfrmA & NearB) = 6;
res(BhndB & Closer2B & Left & NearA & NearB) = 7;
res(BhndB & Closer2B & Right & NearA & NearB) = 8;
res(BhndB & Closer2B & Right & FarfrmA & NearB) = 9;
res(BhndB & Closer2B & Right & FarfrmA & FarfrmB) = 10;
res(InFrntA & Closer2A & Left & FarfrmA & FarfrmB) = 11;
res(InFrntA & Closer2A & Left & FarfrmB & NearA) = 12;
res(InFrntA & Closer2A & Left & NearA & NearB) = 13;
res(InFrntA & Closer2A & Right & NearA & NearB) = 14;
res(InFrntA & Closer2A & Right & FarfrmB & NearA) = 15;
res(InFrntA & Closer2A & Right & FarfrmA & FarfrmB) = 16;
res(BhndA & NearA & Left) = 17;
res(BhndA & NearA & Right) = 18;
res(BhndA & FarfrmA & Left) = 19;
res(BhndA & FarfrmA & Right) = 20;

if any(isnan(res))
error('weird error in calculating GT EDC state');
end

% if InFrntB & & FarfrmB & & Left
     % res = 1;
% elseif
InFrntB & & FarfrmB & & Right
% res = 2;
% elseif
InFrntB & & NearB & & Left
% res = 3;
% elseif
InFrntB & & NearB & & Right
% res = 4;
% elseif
BhndB & & Closer2B & & Left & & FarfrmA & & FarfrmB
% res = 5;
% elseif
BhndB & & Closer2B & & Left & & FarfrmA & & NearB
% res = 6;
% elseif
BhndB & & Closer2B & & Left & & NearA & & NearB
% res = 7;
% elseif
BhndB & & Closer2B & & Right & & NearA & & NearB
% res = 8;
% elseif
BhndB & & Closer2B & & Right & & FarfrmA & & NearB
% res = 9;
% elseif
BhndB & & Closer2B & & Right & & FarfrmA & & FarfrmB
% res = 10;
% elseif
InFrntA & & Closer2A & & Left & & FarfrmA & & FarfrmB
% res = 11;
% elseif
InFrntA & & Closer2A & & Left & & FarfrmB & & NearA
% res = 12;
% elseif
InFrntA & & Closer2A & & Left & & NearA & & NearB
% res = 13;
% elseif
InFrntA & & Closer2A & & Right & & NearA & & NearB
% res = 14;
% elseif
InFrntA & & Closer2A & & Right & & FarfrmB & & NearA
% res = 15;
% elseif
InFrntA & & Closer2A & & Right & & FarfrmA & & FarfrmB
% res = 16;
% elseif
BhndA & & FarfrmA & & Left
% res = 19;
% elseif
BhndA & & FarfrmA & & Right
% res = 20;
% elseif
BhndA & & NearA & & Left
% res = 17;
% elseif
BhndA & & NearA & & Right
% res = 18;
% else
% error('weird error in calculating GT EDC state');
% end

end

function
obj = set.Apos(obj, val)
      % % updates
Apos - must
be
length
2
numerical
location(x, y)
if isnumeric(val) & & length(val) == 2
obj.Apos = val;
obj = obj.update_transform();
else
error('Apos must be numerical scalar')
end
end

function
obj = set.Bpos(obj, val)
      % % updates
Bpos - must
be
length
2
numerical
location(x, y)
if isnumeric(val) & & length(val) == 2
obj.Bpos = val;
obj = obj.update_transform();
else
error('Bpos must be numerical scalar')
end
end

function
obj = set.Cpos(obj, val)
      % % updates
Cpos - must
be
length
2
numerical
location(x, y)
if isnumeric(val) & & length(val) == 2
obj.Cpos = val;
else
error('Cpos must be numerical scalar')
end
end

function
location = get_landmarks(obj, id)
           % % updates
Cpos - must
be
length
2
numerical
location(x, y)

[ia, locb] = ismember(id, [obj.Aid, obj.Bid, obj.Cid]);
landmark_locations = [obj.Apos(:), obj.Bpos(:), obj.Cpos(:)];
if ~all(ia)
error('invalid landmark ids')
else
location = landmark_locations(:,
    locb);
end
end

function
disp(obj)
% % Overload
disp
function.Display
objects as output
of
char
method. \
    fprintf('EDC triplet %d\n', obj.tripletid);
fprintf('landmarks:\n  A: id=%d, pos=(%f,%f)\n  B: id=%d, pos=(%f,%f)\n  C: id=%d, pos=(%f,%f)\n', obj.Aid, obj.Apos,
        obj.Bid, obj.Bpos, obj.Cid, obj.Cpos);
fprintf('C EDC state: %d\n', obj.GtState);
end

function[p] = plotLmrks(obj, f, options)
              % % plot
EDC
landmarks
% f - axis
% options:
% clr - landmarks
color
% xlim\ylim - plot
limits
% frame - which
frame
to
use: 'global'\'ABaligned'

              % f - axis
if nargin < 2 | | isempty(f)
f = figure;
end
optn = struct('clr', 'm', ...
'xlim', [], 'ylim', [], ...
'frame', 'global');
if nargin >= 3
fn=fieldnames(options);
for i=1:numel(fn)
eval(sprintf('optn.%s = options.%s;', fn
{i}, fn
{i}));
end
end
figure(f);
% hold
on;
xmn = obj.xylims(1);
xmx = obj.xylims(2);
ymn = obj.xylims(3);
ymx = obj.xylims(4);

% EDC
constraints( in AB
frame)
switch
optn.frame
case
'global'
Ap = obj.Apos;
Bp = obj.Bpos;
Cp = obj.Cpos;
% find
plot
lims
if isempty(optn.xlim) | | isempty(optn.ylim)
l = [[xmn;
ymn], [xmn;
ymx], [xmx;
ymn], [xmx;
ymx]];
[lx, ly] = AB2World(obj, l(1,:), l(2,:)); l = [lx;
ly];
optn.xlim = [min(l(1,:)), max(l(1,:))];
optn.ylim = [min(l(2,:)), max(l(2,:))];
end

case
'ABaligned'
% convert
landmarks
to
AB
frame
Ap = [0;
0];
Bp = [0;
1];
[cx, cy] = World2AB(obj, obj.Cpos(1), obj.Cpos(2));
Cp = [cx;
cy];

otherwise
error('invalid plot frame - must be global \ ABaligned');
end

p = {};
p
{1} = plot([Ap(1), Bp(1), Cp(1)], [Ap(2), Bp(2), Cp(2)], [optn.clr, 'o'], 'markerSize', 10);
p
{2} = text(Ap(1), Ap(2), 'A', 'fontSize', 12, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
p
{3} = text(Bp(1), Bp(2), 'B', 'fontSize', 12, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
p
{4} = text(Cp(1), Cp(2), 'C', 'fontSize', 12, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
axis([optn.xlim, optn.ylim]);
axis
equal;

end

function[p] = plotConstraints(obj, f, options)
              % % plot
EDC
Constraints
% f - axis
% options:
% clr - Constraint
lines
color
% xlim\ylim - plot
limits
% frame - which
frame
to
use: 'global'\'ABaligned'

if nargin < 2 | | isempty(f)
f = figure;
end
optn = struct('clr', 'm', ...
'xlim', [], 'ylim', [], ...
'frame', 'global');
if nargin >= 3
fn=fieldnames(options);
for i=1:numel(fn)
eval(sprintf('optn.%s = options.%s;', fn
{i}, fn
{i}));
end
end
figure(f);
% hold
on;

% EDC
constraints( in AB
frame)
xmn = obj.xylims(1);
xmx = obj.xylims(2);
ymn = obj.xylims(3);
ymx = obj.xylims(4);
l1 = [0, 0;
...
ymn, ymx];
l2 = [xmn, xmx;
...
0, 0];
l3 = [xmn, xmx;
...
0.5, 0.5];
l4 = [xmn, xmx;
...
1, 1];
a = 0:0.01: 2 * pi;
l5 = [cos(a);
sin(a)];
l6 = [cos(a);
sin(a) + 1];

switch
optn.frame
case
'global'
% convert
constraints
to
global frame
[lx, ly] = AB2World(obj, l1(1,:), l1(2,:)); l1 = [lx;
ly];
[lx, ly] = AB2World(obj, l2(1,:), l2(2,:)); l2 = [lx;
ly];
[lx, ly] = AB2World(obj, l3(1,:), l3(2,:)); l3 = [lx;
ly];
[lx, ly] = AB2World(obj, l4(1,:), l4(2,:)); l4 = [lx;
ly];
[lx, ly] = AB2World(obj, l5(1,:), l5(2,:)); l5 = [lx;
ly];
[lx, ly] = AB2World(obj, l6(1,:), l6(2,:)); l6 = [lx;
ly];
% find
plot
lims
if isempty(optn.xlim) | | isempty(optn.ylim)
l = [[xmn;
ymn], [xmn;
ymx], [xmx;
ymn], [xmx;
ymx]];
[lx, ly] = AB2World(obj, l(1,:), l(2,:)); l = [lx;
ly];
optn.xlim = [min(l(1,:)), max(l(1,:))];
optn.ylim = [min(l(2,:)), max(l(2,:))];
end

case
'ABaligned'
% all
OK

otherwise
error('invalid plot frame - must be global \ ABaligned');
end

p = {};
p
{1} = plot(l1(1,:), l1(2,:), [optn.clr, '-']);
p
{2} = plot(l2(1,:), l2(2,:), [optn.clr, '-']);
p
{3} = plot(l3(1,:), l3(2,:), [optn.clr, '-']);
p
{4} = plot(l4(1,:), l4(2,:), [optn.clr, '-']);
p
{5} = plot(l5(1,:), l5(2,:), [optn.clr, '-']);
p
{6} = plot(l6(1,:), l6(2,:), [optn.clr, '-']);
axis([optn.xlim, optn.ylim]);
axis
equal;
end

function[ff] = plotGtState(obj, f, options)
               % % plot
EDC
Ground
Truth
state
% f - axis
% options:
% clr - state
area
color
% alpha - transparancy
% xlim\ylim - plot
limits
% frame - which
frame
to
use: 'global'\'ABaligned'

if nargin < 2 | | isempty(f)
f = figure;
end
optn = struct('clr', 'g', ...
'alpha', 0.2, ...
'xlim', [], 'ylim', [], ...
'frame', 'global');
if nargin >= 3
fn=fieldnames(options);
for i=1:numel(fn)
eval(sprintf('optn.%s = options.%s;', fn
{i}, fn
{i}));
end
end
figure(f);
% hold
on;

% EDC
constraints( in AB
frame)
xmn = obj.xylims(1);
xmx = obj.xylims(2);
ymn = obj.xylims(3);
ymx = obj.xylims(4);

l1 = [0, 0;
...
ymn, ymx];
l2 = [xmn, xmx;
...
0, 0];
l3 = [xmn, xmx;
...
0.5, 0.5];
l4 = [xmn, xmx;
...
1, 1];
a = 0:0.01: 2 * pi;
l5 = [cos(a);
sin(a)];
l6 = [cos(a);
sin(a) + 1];

p0 = [[xmn;
ymn], [xmn;
ymx], [xmx;
ymx], [xmx;
ymn]];
pL = [l1, [xmn;
ymx], [xmn;
ymn]]; % left
pBhA = [l2, [xmx;
ymn], [xmn;
ymn]]; % behind
A
pc2A = [l3, [xmx;
ymn], [xmn;
ymn]]; % closer
to
A
pBhB = [l4, [xmx;
ymn], [xmn;
ymn]]; % behind
B
pA = l5; % close
to
A
pB = l6; % close
to
B

switch
optn.frame
case
'global'
% convert
constraints
to
global frame
[px, py] = AB2World(obj, p0(1,:), p0(2,:)); p0 = [px;
py];
[px, py] = AB2World(obj, pL(1,:), pL(2,:)); pL = [px;
py];
[px, py] = AB2World(obj, pBhA(1,:), pBhA(2,:)); pBhA = [px;
py];
[px, py] = AB2World(obj, pc2A(1,:), pc2A(2,:)); pc2A = [px;
py];
[px, py] = AB2World(obj, pBhB(1,:), pBhB(2,:)); pBhB = [px;
py];
[px, py] = AB2World(obj, pA(1,:), pA(2,:)); pA = [px;
py];
[px, py] = AB2World(obj, pB(1,:), pB(2,:)); pB = [px;
py];
% find
plot
lims
if isempty(optn.xlim) | | isempty(optn.ylim)
l = [[xmn;
ymn], [xmn;
ymx], [xmx;
ymn], [xmx;
ymx]];
[lx, ly] = AB2World(obj, l(1,:), l(2,:)); l = [lx;
ly];
optn.xlim = [min(l(1,:)), max(l(1,:))];
optn.ylim = [min(l(2,:)), max(l(2,:))];
end

case
'ABaligned'
% all
OK

otherwise
error('invalid plot frame - must be global \ ABaligned');
end

[px, py] = poly2cw(p0(1,:), p0(2,:)); p0 = [px;
py];
[px, py] = poly2cw(pL(1,:), pL(2,:)); pL = [px;
py];
[px, py] = poly2cw(pBhA(1,:), pBhA(2,:)); pBhA = [px;
py];
[px, py] = poly2cw(pc2A(1,:), pc2A(2,:)); pc2A = [px;
py];
[px, py] = poly2cw(pBhB(1,:), pBhB(2,:)); pBhB = [px;
py];
[px, py] = poly2cw(pA(1,:), pA(2,:)); pA = [px;
py];
[px, py] = poly2cw(pB(1,:), pB(2,:)); pB = [px;
py];

if ismember(obj.GtState, [1, 3, 5, 6, 7, 11, 12, 13, 17, 19]) % left
[x, y]=polybool('intersection', p0(1,:), p0(2,:), pL(1,:), pL(2,:));
else
[x, y] = polybool('subtraction', p0(1,:), p0(2,:), pL(1,:), pL(2,:));
end
if obj.GtState >= 17 % behind
A
[x, y] = polybool('intersection', x, y, pBhA(1,:), pBhA(2,:));
else
[x, y] = polybool('subtraction', x, y, pBhA(1,:), pBhA(2,:));
end
if obj.GtState <= 4 % before
B
[x, y] = polybool('subtraction', x, y, pBhB(1,:), pBhB(2,:));
else
[x, y] = polybool('intersection', x, y, pBhB(1,:), pBhB(2,:));
end
if obj.GtState >= 10 % closer
to
A
[x, y] = polybool('intersection', x, y, pc2A(1,:), pc2A(2,:));
else
[x, y] = polybool('subtraction', x, y, pc2A(1,:), pc2A(2,:));
end
if ismember(obj.GtState, [7, 8, 12, 13, 14, 15, 17, 18]) % close
to
A
[x, y] = polybool('intersection', x, y, pA(1,:), pA(2,:));
else
[x, y] = polybool('subtraction', x, y, pA(1,:), pA(2,:));
end
if ismember(obj.GtState, [3, 4, 6, 7, 8, 9, 13, 14]) % close
to
B
[x, y] = polybool('intersection', x, y, pB(1,:), pB(2,:));
else
[x, y] = polybool('subtraction', x, y, pB(1,:), pB(2,:));
end
ff = fill(x, y, optn.clr);
alpha(ff, optn.alpha);
axis([optn.xlim, optn.ylim]);
axis
equal;
end

function[t] = plotStates(obj, data_mode, f, options)
              % % plot
EDC
eatimated
state
% data_mode - which
dta
to
plot
for each state
         % 'prob': state
probability
% 'id': state
id
% 'prob+id': both
             %
             % f - axis
             % options:
% clr - state
probability
color
% xlim\ylim - plot
limits
% frame - which
frame
to
use: 'global'\'ABaligned'

if nargin < 2 | | isempty(f)
f = figure;
end
optn = struct('clr', 'b', ...
'xlim', [], 'ylim', [], ...
'frame', 'global');
if nargin >= 3
fn=fieldnames(options);
for i=1:numel(fn)
eval(sprintf('optn.%s = options.%s;', fn
{i}, fn
{i}));
end
end

xmn = obj.xylims(1);
xmx = obj.xylims(2);
ymn = obj.xylims(3);
ymx = obj.xylims(4);

figure(f);
% hold
on;

% EDC
constraints( in AB
frame)
p = zeros(2, 20);
p(:, 1)=[-1.5;
2];
p(:, 2)=[1.5;
2];
p(:, 3)=[-0.5;
1.5];
p(:, 4)=[0.5;
1.5];

p(:, 5)=[-1.5;
0.75];
p(:, 6)=[-0.9;
0.8];
p(:, 7)=[-0.4;
0.7];

p(:, 8)=[0.4;
0.7];
p(:, 9)=[0.75;
0.8];
p(:, 10)=[1.5;
0.75];

p(:, 11)=[-1.5;
0.25];
p(:, 12)=[-0.9;
0.2];
p(:, 13)=[-0.4;
0.35];

p(:, 14)=[0.4;
0.35];
p(:, 15)=[0.75;
0.2];
p(:, 16)=[1.5;
0.25];

p(:, 17)=[-0.5;
-0.5];
p(:, 18)=[0.5;
-0.5];
p(:, 19)=[-1.5;
-1];
p(:, 20)=[1.5;
-1];

switch
optn.frame
case
'global'
% convert
constraints
to
global frame
[px, py] = AB2World(obj, p(1,:), p(2,:)); p = [px;
py];
% find
plot
lims
if isempty(optn.xlim) | | isempty(optn.ylim)
l = [[xmn;
ymn], [xmn;
ymx], [xmx;
ymn], [xmx;
ymx]];
[lx, ly] = AB2World(obj, l(1,:), l(2,:)); l = [lx;
ly];
optn.xlim = [min(l(1,:)), max(l(1,:))];
optn.ylim = [min(l(2,:)), max(l(2,:))];
end

case
'ABaligned'
% all
OK

otherwise
error('invalid plot frame - must be global \ ABaligned');
end

switch
data_mode
case
'id'
C = num2str((1:20)
',' % d
');
case
'prob'
C = num2str(obj.Cstate(:), '%.2f');
case
'prob+id'
C = num2str([(1:20)',obj.Cstate(:)],' % d( % .2f)
');
otherwise
error('invalid data mode!');
end
t = {};
t
{1} = text(p(1,:), p(2,:), C, 'color', optn.clr, ...
'HorizontalAlignment', 'center', ...
'VerticalAlignment', 'middle ', ...
'FontSize', 7);
axis([optn.xlim, optn.ylim]);
axis
equal;
end

% function[t] = plotStateIds(obj, f, options)
                % % % plot
EDC
state
IDs
% % f - axis
% % options:
% % clr - state
id
color
% % xlim\ylim - plot
limits
% % frame - which
frame
to
use: 'global'\'ABaligned'
              %
              % if nargin < 2 | | isempty(f)
                   % f = figure;
% end \
  % optn = struct('clr', 'b', ...
                  % 'xlim', [], 'ylim', [], ...
                  % 'frame', 'global');
% if nargin >= 3
        % fn=fieldnames(options);
% for i=1:numel(fn)
          % eval(sprintf('optn.%s = options.%s;', fn
{i}, fn
{i}));
% end
  % end
  %
  % xmn = obj.xylims(1);
xmx = obj.xylims(2);
% ymn = obj.xylims(3);
ymx = obj.xylims(4);
%
% figure(f);
% hold
on;
%
% % EDC
constraints( in AB
frame)
% p = zeros(2, 20);
% p(:, 1)=[-1.5;
2];
% p(:, 2)=[1.5;
2];
% p(:, 3)=[-0.5;
1.5];
% p(:, 4)=[0.5;
1.5];
%
% p(:, 5)=[-1.5;
0.75];
% p(:, 6)=[-0.9;
0.8];
% p(:, 7)=[-0.4;
0.7];
%
% p(:, 8)=[0.4;
0.7];
% p(:, 9)=[0.75;
0.8];
% p(:, 10)=[1.5;
0.75];
%
% p(:, 11)=[-1.5;
0.25];
% p(:, 12)=[-0.9;
0.2];
% p(:, 13)=[-0.4;
0.35];
%
% p(:, 14)=[0.4;
0.35];
% p(:, 15)=[0.75;
0.2];
% p(:, 16)=[1.5;
0.25];
%
% p(:, 17)=[-0.5;
-0.5];
% p(:, 18)=[0.5;
-0.5];
% p(:, 19)=[-1.5;
-1];
% p(:, 20)=[1.5;
-1];
%
% switch
optn.frame
% case
'global'
% % convert
constraints
to
global frame
% [px, py] = AB2World(obj, p(1,:), p(2,:)); p = [px;
py];
% % find
plot
lims
% if isempty(optn.xlim) | | isempty(optn.ylim)
     % l = [[xmn;
ymn], [xmn;
ymx], [xmx;
ymn], [xmx;
ymx]];
% [lx, ly] = AB2World(obj, l(1,:), l(2,:)); l = [lx;
ly];
% optn.xlim = [min(l(1,:)), max(l(1,:))];
% optn.ylim = [min(l(2,:)), max(l(2,:))];
% end
  %
  % case
'ABaligned'
% % all
OK
%
% otherwise
% error('invalid plot frame - must be global \ ABaligned');
% end
  %
  % switch
mode
% case
'id'
% C = num2str((1:20)
',' % d
');
% case
'prob'
% C = num2str(obj.Cstate(:), '%d');
% case
'prob+id'
% C = num2str([(1:20)',obj.Cstate(:)],' % d: % .2
f');
% otherwise \
  % error('invalid data mode!');
% end \
  % t = {};
% t
{1} = text(p(1,:), p(2,:), num2str(C(:), '%d'), 'color', optn.clr, ...
                                                % 'HorizontalAlignment', 'center', ...
                                                % 'VerticalAlignment', 'middle ', ...
                                                % 'FontSize', 7);
% axis([optn.xlim, optn.ylim]);
% axis
equal;
% end

end

end