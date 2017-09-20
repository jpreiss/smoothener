% solves the corridor-constrained trajectory optimization problem for one robot.
% uses "sequence of points" parameterization instead of Bezier polynomial basis.
% if the discrete plan is high-resolution (meaning there is a very small
% amount of space and/or time per step) then the high-order Bezier curves
% are more fine-grained control than we actually need. But the Bezier formulation
% requires one polynomial piece per polytope no matter what.
% By switching to a sequence-of-points representation, we lose the exact
% collision avoidance guarantee of the Bezier convex hull property,
% but we can reduce the number of variables a lot,
% making optimization much faster.

% inputs:
%   Arobots, brobots: [DIM x NROBOTS x NSTEPS] and [NROBOTS x NSTEPS]
%                     hyperplanes separating this robot from the other robots at each step
%   Aobs, bobs:       [DIM x <problem-dependent> x NSTEPS] and [<problem-dependent> x NSTEPS]
%                     hyperplanes separating this robot from obstacles at each step.
%                     second dimension is large enough for the polytope with the most faces,
%                     so some rows are allowed to be NaN for polytopes with fewer faces
%   lb, ub:           [3] and [3] lower/upper bound of environment box
%   path:             [3 NSTEPS+1] the discrete plan
%   npts:             [1]        number of points to use per polytope
%   cont:             [1]        derivative continuity (here, indicates number of 0 derivatives at start)
%   timescale:        [1]        duration in seconds of step in discrete plan
%   ellipsoid:        [3 1]      radii of robot/robot collision ellipsoid
%   obs_ellipsoid:    [3 1]      radii of robot/obstacle collision ellipsoid
%
% outputs:
%   pp:   a matlab ppform struct containing the trajectory
%   cost: the cost value of the quadratic program
%
function [pp, cost] = corridor_trajectory_optimize(...
	Arobots, brobots, Aobs, bobs, lb, ub, ...
	path, npts, cont, timescale, ellipsoid, obs_ellipsoid)

	[dim, ~, steps] = size(Arobots);
	assert(size(path, 2) == steps + 1);
	init = path(:,1);
	goal = path(:,end);

	ellipsoid = diag(ellipsoid);
	obs_ellipsoid = diag(obs_ellipsoid);

	% TODO move this outside
	me = find(isnan(brobots(:,1)));
	assert(length(me) == 1);
	brobots(me,:) = [];
	Arobots(:,me,:) = [];

	% number of decision variables
	nvars = dim * npts * steps;
	lb = repmat(lb, steps * npts, 1);
	ub = repmat(ub, steps * npts, 1);

	Aineq = {};
	bineq = [];
	Aeq = {};
	beq = [];

	dt = timescale / npts;
	diff1_mtx = diff_matrix(dim, steps * npts, dt, 1);
	diff2_mtx = diff_matrix(dim, steps * npts, dt, 2);
	diff3_mtx = diff_matrix(dim, steps * npts, dt, 3);
	diff4_mtx = diff_matrix(dim, steps * npts, dt, 4);
	diff_mtxs = {diff1_mtx diff2_mtx diff3_mtx diff4_mtx};

	for step=1:steps
		dim_select = 1:steps == step;

		% offset the corridor bounding polyhedra by the ellipsoid
		Astep = [Arobots(:,:,step)'; Aobs(:,:,step)'];
		bstep = [polytope_erode_by_ellipsoid(Arobots(:,:,step)', brobots(:,step), ellipsoid); ...
				 polytope_erode_by_ellipsoid(Aobs(:,:,step)', bobs(:,step), obs_ellipsoid)];

		% delete NaN inputs coming from "ragged" Aobs, bobs
		nan_rows = isnan(bstep);
		Astep(nan_rows,:) = [];
		bstep(nan_rows) = [];

		% try to eliminate redundant half-space constraints
		interior_pt = (path(:,step) + path(:,step+1)) ./ 2;
		[Astep,bstep] = noredund(Astep,bstep,interior_pt);

		% add bounding polyhedron constraints on trajectory points
		Aineq = [Aineq; kron(dim_select, kron(eye(npts), Astep))];
		bineq = [bineq; repmat(bstep, npts, 1)];

		if step == 1
			% initial position and 0 derivatives
			Aeq = [Aeq; kron(dim_select, kron(onehot(1, npts), eye(dim)))];
			beq = [beq; init];

			for d=1:cont
				D = diff_mtxs{d};
				Aeq = [Aeq; D(1:dim,:)];
				beq = [beq; zeros(dim,1)];
			end
		end

		if step == steps
			% final position and 0 derivatives
			Aeq = [Aeq; kron(dim_select, kron(onehot(npts, npts), eye(dim)))];
			beq = [beq; goal];

			for d=1:cont
				D = diff_mtxs{d};
				Aeq = [Aeq; D((end-dim+1):end,:)];
				beq = [beq; zeros(dim,1)];
			end
		end
	end

	Aineq = cat(1, Aineq{:});
	Aeq = cat(1, Aeq{:});
	assert(size(Aineq, 2) == nvars);
	assert(size(Aineq, 1) == length(bineq));
	assert(size(Aeq, 2) == nvars);
	assert(size(Aeq, 1) == length(beq));

	Q = ...
		1 * (diff2_mtx' * dt * diff2_mtx) + ...
		5e-3 * (diff4_mtx' * dt * diff4_mtx);

	options = optimoptions('quadprog', 'Display', 'off');

	if exist('cplexqp')
		[x, cost] = cplexqp(Q, zeros(1,nvars), Aineq, bineq, Aeq, beq, lb, ub);
	else
		[x, cost] = quadprog(sparse(Q), zeros(1,nvars), ...
			sparse(Aineq), sparse(bineq), sparse(Aeq), sparse(beq), ...
			lb, ub, [], options);
	end

	x = reshape(x, [dim, npts * steps]);
	t = 0:(npts*steps-1);
	t = ((steps * timescale) / (length(t) - 1)) .* t;

	knots = linspace(t(1), t(end), steps + 1);
	augknots = augknt(knots, cont);
	bspline = spap2(augknots, cont, t, x);
	pp = fn2fm(bspline, 'pp');
end

function x = onehot(i, n)
	x = zeros(1, n);
	x(i) = 1;
end
