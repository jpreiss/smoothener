% read a schedule from Wolfgang's discrete planner
% into a [3 x Kpts x Nrobots] waypoint array
%
function paths = read_schedule(fname)
	json = loadjson(fname);
	N = length(json.agents);
	k = 0;
	for i=1:N
		k = max(k, length(json.agents{i}.path));
	end
	paths = nan(3,k,N);
	for i=1:N
		p = json.agents{i}.path;
		len = length(p);
		for j=1:len
			paths(1,j,i) = str2num(p{j}.x);
			paths(2,j,i) = str2num(p{j}.y);
			paths(3,j,i) = str2num(p{j}.z);
		end
		for j=(len+1):k
			paths(:,j,i) = paths(:,len,i);
		end
	end
	assert(~any(isnan(paths(:))));
end
