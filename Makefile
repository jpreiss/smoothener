smoothener:
	matlab -nosplash -nodesktop -nojvm -r "mex_required,quit"

octomap:
	matlab -nosplash -nodesktop -nojvm -r "mex_octomap,quit"
	make -C octomap_corridor

clean:
	make clean -C octomap_corridor
	find . -name "*.mexa64" -type f -delete
