% Load a real problem instance and solve.
% ---------------------------------------
function main_octomap()

% the duration of each step in the discrete plan, in seconds
TIMESCALE = 0.25;

% read input files
EXAMPLE = 'examples/octomap/warehouse4';
discrete_plan_file = [EXAMPLE '.json'];
octree_file = [EXAMPLE '.bt'];
s = read_schedule(discrete_plan_file);
[~, ~, N] = size(s);
bbox = read_octomap_bbox_mex(octree_file);

% print some info about the discrete plan input
analyze_schedule(s);

% optional: clip the number of robots so it runs faster
% N = 5;
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

% robot/obstacle separating hyperplane function
pp_obs_sep_fun = @(pps, obs_ellipsoid) pp_obs_sep_octomap(pps, obs_ellipsoid, octree_file);

% main routine
[pps, costs, corridors] = smoothener(s, bbox, deg, cont, TIMESCALE, ellipsoid, obs_ellipsoid, iters, ...
	pp_obs_sep_fun, @corridor_trajectory_optimize);

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

% read and render the STL file of the octree
stl_file = [EXAMPLE '.stl'];
fv = stlread(stl_file);
patch(fv, ...
	'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', ...
	'SpecularStrength', 0.1, 'AmbientStrength', 0.5, 'facealpha', 0.5);

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
	DIR = [EXAMPLE '_pps'];
	mkdir(DIR);
	for i=1:N
		filename = sprintf('%s/pp%d.csv', DIR, i);
		pp2csv(pps{i}, filename)
	end
end

end
