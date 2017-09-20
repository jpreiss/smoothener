% main function of Smoothener package.
% given a discrete multi-robot motion plan as a synchronized waypoint list,
% and a description of obstacles in the environment [1],
% computes smooth piecewise polynomial trajectories for each robot
% that locally minimize a weighted integral-squared-derivative cost
% while ensuring robot/robot and robot/obstacle collision avoidance.
%
% it is an implementation of the continuous portion of:
% Downwash-Aware Trajectory Planning for Large Quadcopter Teams
% James A. Preiss, Wolfgang H\"onig, Nora Ayanian, and Gaurav S. Sukhatme
% to appear in IEEE IROS 2017, preprint at arxiv.org/abs/1704.04852.
% Octomap extension currently under review.
%
% [1] "list of boxes" and "octomap" are currently supported obstacle models.
%     Adding another obstacle model requires implementing a single function
%     with the pp_obs_sep_fun signature described below.
%
%
% inputs:
%   paths:          [dim k N]  discrete planner waypoints
%   bbox:           [3 2]      bounding box around environment
%   deg:            [1]        polynomial degree
%   cont:           [1]        derivative continuity (e.g. 2 == cts accel)
%   timescale:      [1]        duration in seconds of step in discrete plan
%   ellipsoid:      [3 1]      radii of robot/robot collision ellipsoid
%   obs_ellipsoid:  [3 1]      radii of robot/obstacle collision ellipsoid
%   iters:          [1]        number of iterations of refinement (>= 1)
%   pp_obs_sep_fun: function handle with signature described below,
%                   that computes robot/obstacle separating hyperplanes
%
%     function polytopes = pp_obs_sep_fun(pps, obs_ellipsoid)
%       pps: {N} cell array of matlab ppform structs
%       obs_ellipsoid: as above
%       polytopes: {N k-1} cell array of halfspaces
%         such that p = polytopes{i} is a [4 nfaces] array
%         describing the linear system p(:,1:3)x <= p(:,4)
%
% output: cell array of [iters, N] ppform piecewise polynomials
%         array of [iters, N] costs
%
function [all_pps, all_costs, all_corridors] = smoothener(...
	paths, bbox, ...
	deg, cont, timescale, ...
	ellipsoid, obs_ellipsoid, ...
	iters, ...
	pp_obs_sep_fun, ...
	corridor_fun)

	[dim, k, N] = size(paths);

	lb = bbox(:,1) + obs_ellipsoid(:);
	ub = bbox(:,2) - obs_ellipsoid(:);

	% for a reasonable problem, cost should converge after ~5 iterations.
	assert(iters >= 1);
	assert(iters <= 20);

	% outputs
	all_pps = cell(iters, N);
	all_costs = zeros(iters, N);
	all_corridors = cell(iters, N);

	% piecewise linear (physically impossible) pps of path
	% for input to pp-vs-octree obstacle hyperplane function
	pps = path_linear_pps(paths, timescale, deg + 1);

	for iter=1:iters
		fprintf('iteration %d of %d...\n', iter, iters);
		tic;
		if iter==1
			% first iteration: decompose by segments
			[A, b] = all_hyperplanes_waypoints_mex(paths, ellipsoid);
		else
			% continuing iteration: decompose by pps
			[A, b] = all_hyperplanes_pps(pps, ellipsoid);
		end

		if iter > 1
			for irobot=1:N
				paths(:,:,irobot) = ppval(pps{irobot}, pps{irobot}.breaks);
			end
		end

		hs = pp_obs_sep_fun(pps, obs_ellipsoid);

		t_hyperplanes = toc;
		fprintf('hyperplanes: %f sec\n', t_hyperplanes);

		% solve the independent spline trajectory optimization problems.
		tic;
		pps = cell(1,N);
		iter_costs = zeros(1,N);
		% parfor
		parfor j=1:N
			hs_slice = squeeze(hs(j,:));
			step_n_faces = cellfun(@(a) size(a, 1), hs_slice);
			assert(length(step_n_faces) == (k-1));
			max_n_faces = max(step_n_faces(:));

			Aobs = nan(dim, max_n_faces, k-1);
			bobs = nan(max_n_faces, k-1);
			Arobots = squeeze(A(:,j,:,:));
			brobots = squeeze(b(j,:,:));
			for i=1:(k-1)
				n_faces = step_n_faces(i);
				hs_slice_step = hs_slice{i};
				assert(size(hs_slice_step, 2) == 4);
				Aobs(:,1:n_faces,i) = hs_slice_step(:,1:3)';
				bobs(1:n_faces,i) = hs_slice_step(:,4);
			end
			[pps{j}, iter_costs(j)] = corridor_fun(...
				Arobots, brobots, ...
				Aobs, bobs, ...
				lb, ub,...
				paths(:,:,j), deg, cont, timescale, ellipsoid, obs_ellipsoid);

			s = [];
			s.Arobots = Arobots;
			s.brobots = brobots;
			s.Aobs = Aobs;
			s.bobs = bobs;
			all_corridors{iter,j} = s;
		end
		t_splines = toc;
		fprintf('splines: %f sec\n', t_splines);
		fprintf('total: %f sec\n', t_hyperplanes + t_splines);
		fprintf('cost: %f\n', sum(iter_costs));
		all_costs(iter,:) = iter_costs;
		all_pps(iter,:) = pps;
	end
end
