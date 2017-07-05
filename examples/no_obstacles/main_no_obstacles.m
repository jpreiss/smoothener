% Load a real problem instance and solve.
% ---------------------------------------
function main_no_obstacles()

% the duration of each step in the discrete plan, in seconds
TIMESCALE = 0.25;

% read input file
discrete_plan_file = 'examples/no_obstacles/warehouse4.json';
s = read_schedule(discrete_plan_file);
[~, ~, N] = size(s);

% make a padded bounding box around the schedule
all_pts = reshape(s, 3, []);
bbox = zeros(3,2);
bbox(:,1) = min(all_pts, [], 2) - 0.5;
bbox(:,2) = max(all_pts, [], 2) + 0.5;

% print some info about the discrete plan input
analyze_schedule(s);

% optional: clip the number of robots so it runs faster
%N = 5;
s = s(:,:,1:N);

% add extra stationary steps at begin and end for smooth acceleration
s = cat(2, s(:,1,:), s(:,1,:), s, s(:,end,:));

% polynomial degree
deg = 7;

% how many derivatives must be continuous
cont = 4;

% robot-robot collision ellipsoid
ellipsoid = [0.12 0.12 0.3];

% robot-obstacle collision ellipsoid
obs_ellipsoid = [0.15 0.15 0.15];

% number of iterations of refinement
iters = 2;

% null robot/obstacle separating hyperplane function - no obstacles
pp_obs_sep_fun = @pp_obs_sep_none;

% main routine
[pps, costs, corridors] = smoothener(s, bbox, deg, cont, TIMESCALE, ellipsoid, obs_ellipsoid, iters, pp_obs_sep_fun);

% Plot the results.
% -----------------

% smoothener returns the piecewise polynomials for every iteration of refinement.
% here, we plot only the results of the final iteration.
pps = pps(end,:);

% set up the figure.
clf; hold on; axis equal;
light('Position', [1 1 1]);
light('Position', [1 1 -1]);
campos([-32 45 21]);

% render the output
for i=1:N
	% plot the discrete plan
	h = plot3n(s(:,:,i));

	% plot the continuous trajectory
	color = get(h, 'color');
	duration = pps{i}.breaks(end);
	t = 0:0.1:duration;
	x = ppval(pps{i}, t);
	h = plot3n(x, 'color', color, 'LineWidth', 3);

	% plot the start and endpoints as markers
	lastPos = ppval(pps{i}, duration);
	scatter3(lastPos(1), lastPos(2), lastPos(3),50,'k','filled');
	firstPos = ppval(pps{i}, 0);
	scatter3(firstPos(1), firstPos(2), firstPos(3),50,'k', 'square');%'g','filled');
end


% Save the piecewise polynomial coefficients to a directory of CSV files.
% -----------------------------------------------------------------------
SAVE_CSVS = true;
if SAVE_CSVS
	DIR = 'examples/no_obstacles/no_obstacles_pps';
	mkdir(DIR);
	for i=1:N
		filename = sprintf('%s/pp%d.csv', DIR, i);
		pp2csv(pps{i}, filename)
	end
end

end