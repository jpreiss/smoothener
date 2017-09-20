function D = diff_matrix(dim, npts, dt, order)
	first_row = [-1 1 zeros(1, npts - 2)];
	first_col = [-1 0 zeros(1, npts - 3)];
	D1 = kron(toeplitz(first_col, first_row), eye(dim) ./ dt);
	D = D1;
	dims = size(D1);
	for i=order:(-1):2
		dims = dims - dim;
		D1 = D1(1:dims(1),1:dims(2));
		D = D1 * D;
	end
end
